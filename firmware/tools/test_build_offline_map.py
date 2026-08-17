#!/usr/bin/env python3

from io import BytesIO
from pathlib import Path
import sqlite3
import struct
import tempfile
import unittest
from typing import Optional

from PIL import Image

import build_offline_map as subject


def image_blob(fmt: str = "PNG", size: tuple[int, int] = (256, 256),
               color: tuple[int, int, int] = (20, 80, 140)) -> bytes:
    output = BytesIO()
    Image.new("RGB", size, color).save(output, fmt)
    return output.getvalue()


def make_mbtiles(path: Path, *, fmt: str = "png",
                 attribution: Optional[str] = "© OpenStreetMap contributors",
                 tile_size: tuple[int, int] = (256, 256)) -> None:
    connection = sqlite3.connect(path)
    connection.execute("CREATE TABLE metadata (name TEXT, value TEXT)")
    connection.execute("CREATE TABLE tiles (zoom_level INTEGER, tile_column INTEGER, tile_row INTEGER, tile_data BLOB)")
    values = {"format": fmt, "bounds": "-1,-1,1,1", "center": "0,0,2"}
    if attribution is not None:
        values["attribution"] = attribution
    connection.executemany("INSERT INTO metadata VALUES (?, ?)", values.items())
    for tms_y, color in ((2, (200, 20, 20)), (1, (20, 200, 20))):
        connection.execute("INSERT INTO tiles VALUES (2, 2, ?, ?)",
                           (tms_y, image_blob("PNG", tile_size, color)))
    connection.commit()
    connection.close()


class OfflineMapBuilderTests(unittest.TestCase):
    def test_builds_sorted_xyz_index_and_baseline_jpeg(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.mbtiles"
            make_mbtiles(source)
            index, segments = subject.build(source, root / "out", 82, None)
            raw = index.read_bytes()
            header = subject.HEADER.unpack_from(raw)
            self.assertEqual(header[0], subject.MAGIC)
            self.assertEqual(header[2:6], (2, 2, 2, 1))
            self.assertEqual(header[7], 1 << 2)
            attribution_length = struct.unpack_from("<I", raw, subject.HEADER.size)[0]
            start = subject.HEADER.size + 4 + attribution_length
            entries = [subject.ENTRY.unpack_from(raw, start + offset * subject.ENTRY.size)
                       for offset in range(2)]
            self.assertEqual([(entry[0], entry[1], entry[2]) for entry in entries],
                             [(2, 2, 1), (2, 2, 2)])
            self.assertEqual(header[6], max(entry[5] for entry in entries))
            segment = segments[0].read_bytes()
            for entry in entries:
                jpeg = segment[entry[4]:entry[4] + entry[5]]
                image = Image.open(BytesIO(jpeg))
                self.assertEqual(image.format, "JPEG")
                self.assertEqual(image.size, (256, 256))
                self.assertNotIn(b"\xff\xc2", jpeg)

    def test_output_is_deterministic_and_splits_at_platform_limit(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.mbtiles"
            make_mbtiles(source)
            first = root / "first"
            second = root / "second"
            _, first_segments = subject.build(source, first, 80, None,
                                               segment_limit=1800)
            _, second_segments = subject.build(source, second, 80, None,
                                                segment_limit=1800)
            self.assertGreater(len(first_segments), 1)
            self.assertEqual((first / "map.uwi").read_bytes(),
                             (second / "map.uwi").read_bytes())
            self.assertEqual((first / "map.sha256").read_text(),
                             (second / "map.sha256").read_text())

    def test_rejects_vector_tiles_missing_attribution_and_wrong_size(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            vector = root / "vector.mbtiles"
            make_mbtiles(vector, fmt="pbf")
            with self.assertRaisesRegex(ValueError, "vector"):
                subject.build(vector, root / "vector-out", 80, None)
            missing = root / "missing.mbtiles"
            make_mbtiles(missing, attribution=None)
            with self.assertRaisesRegex(ValueError, "attribution"):
                subject.build(missing, root / "missing-out", 80, None)
            wrong = root / "wrong.mbtiles"
            make_mbtiles(wrong, tile_size=(128, 256))
            with self.assertRaisesRegex(ValueError, "tile size"):
                subject.build(wrong, root / "wrong-out", 80, None)


if __name__ == "__main__":
    unittest.main()
