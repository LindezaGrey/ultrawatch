#!/usr/bin/env python3
"""Convert a raster MBTiles archive to the UltraWatch offline map format."""

from argparse import ArgumentParser
from io import BytesIO
from pathlib import Path
import hashlib
import sqlite3
import struct
import shutil
import tempfile
from typing import Optional

from PIL import Image


MAGIC = b"UWMAP001"
VERSION = 1
TILE_SIZE = 256
HEADER = struct.Struct("<8sIIIIIIIddddii")
ENTRY = struct.Struct("<B3xIIIQI")
FAT32_MAX_FILE = 0xFFFFFFFF
ESP_IDF_MAX_SEEK = 0x7FFFFFFF
SEGMENT_MAX_FILE = min(FAT32_MAX_FILE, ESP_IDF_MAX_SEEK)


def metadata(connection: sqlite3.Connection) -> dict[str, str]:
    try:
        return dict(connection.execute("SELECT name, value FROM metadata"))
    except sqlite3.DatabaseError as error:
        raise ValueError("MBTiles metadata table is missing or invalid") from error


def parse_pair(value: str, count: int, name: str) -> tuple[float, ...]:
    try:
        values = tuple(float(part) for part in value.split(","))
    except ValueError as error:
        raise ValueError(f"invalid {name} metadata") from error
    if len(values) != count:
        raise ValueError(f"{name} metadata must contain {count} values")
    return values


def encode_jpeg(blob: bytes, quality: int) -> bytes:
    try:
        image = Image.open(BytesIO(blob))
        if image.format not in {"PNG", "JPEG", "WEBP"}:
            raise ValueError("tile is not PNG, JPEG, or WebP")
        image.load()
    except Exception as error:
        raise ValueError("tile is not a supported PNG, JPEG, or WebP image") from error
    if image.size != (TILE_SIZE, TILE_SIZE):
        raise ValueError(f"tile size is {image.size}; expected 256 x 256")
    output = BytesIO()
    image.convert("RGB").save(output, "JPEG", quality=quality,
                              optimize=False, progressive=False,
                              subsampling="4:2:0")
    return output.getvalue()


def build(source: Path, output_dir: Path, quality: int,
          attribution_override: Optional[str],
          *, segment_limit: int = SEGMENT_MAX_FILE) -> tuple[Path, list[Path]]:
    if not 1 <= quality <= 95:
        raise ValueError("JPEG quality must be from 1 through 95")
    connection = sqlite3.connect(f"file:{source}?mode=ro", uri=True)
    info = metadata(connection)
    if info.get("format", "").lower() in {"pbf", "mvt"}:
        raise ValueError("vector MBTiles are not supported")
    attribution = attribution_override or info.get("attribution", "")
    if not attribution.strip():
        raise ValueError("attribution metadata or --attribution is required")
    try:
        bounds = parse_pair(info["bounds"], 4, "bounds")
    except KeyError as error:
        raise ValueError("bounds metadata is required") from error
    center_value = info.get("center")
    if center_value:
        center = parse_pair(center_value, 3, "center")
        center_lon, center_lat = center[0], center[1]
    else:
        center_lon = (bounds[0] + bounds[2]) / 2
        center_lat = (bounds[1] + bounds[3]) / 2

    try:
        rows = connection.execute(
            "SELECT zoom_level, tile_column, tile_row, tile_data "
            "FROM tiles ORDER BY zoom_level, tile_column, tile_row DESC"
        )
    except sqlite3.DatabaseError as error:
        raise ValueError("MBTiles tiles table is missing or invalid") from error

    output_dir.mkdir(parents=True, exist_ok=True)
    segment_paths: list[Path] = []
    segment = None
    segment_index = -1
    segment_size = 0
    tile_count = 0
    min_zoom = 255
    max_zoom = 0
    max_tile_bytes = 0
    zoom_mask = 0
    entries_temp = tempfile.NamedTemporaryFile(dir=output_dir, delete=False)
    try:
        try:
            for zoom, tile_x, tms_y, blob in rows:
                if not 0 <= zoom <= 23 or not 0 <= tile_x < (1 << zoom) or not 0 <= tms_y < (1 << zoom):
                    raise ValueError("tile coordinate is outside the Web Mercator grid")
                xyz_y = (1 << zoom) - 1 - tms_y
                encoded = encode_jpeg(blob, quality)
                if len(encoded) > segment_limit:
                    raise ValueError("one encoded tile exceeds the segment size")
                if segment is None or segment_size + len(encoded) > segment_limit:
                    if segment is not None:
                        segment.close()
                    segment_index += 1
                    path = output_dir / f"map.{segment_index:03d}"
                    segment_paths.append(path)
                    segment = path.open("wb")
                    segment_size = 0
                offset = segment_size
                segment.write(encoded)
                segment_size += len(encoded)
                max_tile_bytes = max(max_tile_bytes, len(encoded))
                entries_temp.write(ENTRY.pack(zoom, tile_x, xyz_y,
                                              segment_index, offset,
                                              len(encoded)))
                tile_count += 1
                min_zoom = min(min_zoom, zoom)
                max_zoom = max(max_zoom, zoom)
                zoom_mask |= 1 << zoom
        finally:
            if segment is not None:
                segment.close()
            entries_temp.close()
            connection.close()
    except Exception:
        Path(entries_temp.name).unlink(missing_ok=True)
        for path in segment_paths:
            path.unlink(missing_ok=True)
        raise
    if tile_count == 0:
        Path(entries_temp.name).unlink(missing_ok=True)
        raise ValueError("MBTiles archive contains no tiles")

    attribution_bytes = attribution.strip().encode("utf-8")
    index_path = output_dir / "map.uwi"
    with index_path.open("wb") as index:
        index.write(HEADER.pack(MAGIC, VERSION, tile_count, min_zoom,
                                max_zoom, len(segment_paths), max_tile_bytes,
                                zoom_mask, *bounds,
                                int(round(center_lon * 1e7)),
                                int(round(center_lat * 1e7))))
        index.write(struct.pack("<I", len(attribution_bytes)))
        index.write(attribution_bytes)
        with open(entries_temp.name, "rb") as entries_file:
            shutil.copyfileobj(entries_file, index)
    Path(entries_temp.name).unlink()

    checksum = hashlib.sha256()
    for path in [index_path, *segment_paths]:
        checksum.update(path.name.encode("ascii") + b"\0")
        with path.open("rb") as source_file:
            while chunk := source_file.read(1024 * 1024):
                checksum.update(chunk)
    (output_dir / "map.sha256").write_text(checksum.hexdigest() + "\n")
    return index_path, segment_paths


def main() -> None:
    parser = ArgumentParser()
    parser.add_argument("mbtiles", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--jpeg-quality", type=int, required=True)
    parser.add_argument("--attribution")
    args = parser.parse_args()
    try:
        index, segments = build(args.mbtiles, args.output_dir,
                                args.jpeg_quality, args.attribution)
    except (OSError, sqlite3.DatabaseError, ValueError) as error:
        parser.error(str(error))
    print(f"wrote {index} and {len(segments)} tile segment(s)")


if __name__ == "__main__":
    main()
