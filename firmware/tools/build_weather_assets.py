#!/usr/bin/env python3
"""Build the 128 px weather-picture atlas used by the watch."""

from argparse import ArgumentParser
from pathlib import Path
import struct

from PIL import Image, ImageDraw


TILE_SIZE = 128
TILE_ORDER = (
    "clear",
    "few_clouds",
    "scattered_clouds",
    "broken_clouds",
    "shower_rain",
    "rain",
    "thunderstorm",
    "snow",
    "mist",
)
ATLAS_SIZE = len(TILE_ORDER) * TILE_SIZE * TILE_SIZE * 2


def normalize(source: Path) -> Image.Image:
    image = Image.open(source).convert("RGBA")
    bounds = image.getchannel("A").getbbox()
    if bounds is None:
        raise ValueError(f"{source}: no visible pixels")
    picture = image.crop(bounds)
    picture.thumbnail((TILE_SIZE - 4, TILE_SIZE - 4),
                      Image.Resampling.LANCZOS)
    tile = Image.new("RGBA", (TILE_SIZE, TILE_SIZE), (0, 0, 0, 0))
    tile.alpha_composite(
        picture,
        ((TILE_SIZE - picture.width) // 2,
         (TILE_SIZE - picture.height) // 2),
    )
    return tile


def rgb565_bytes(tile: Image.Image) -> bytes:
    black = Image.new("RGB", tile.size, "black")
    black.paste(tile, mask=tile.getchannel("A"))
    output = bytearray()
    for red, green, blue in black.getdata():
        value = ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3)
        output.extend(struct.pack(">H", value))
    return bytes(output)


def main() -> None:
    parser = ArgumentParser()
    parser.add_argument("source_dir", type=Path)
    parser.add_argument("atlas", type=Path)
    parser.add_argument("contact_sheet", type=Path)
    parser.add_argument("--tile-dir", type=Path)
    args = parser.parse_args()

    tile_dir = args.tile_dir or args.contact_sheet.parent / "tiles"
    tile_dir.mkdir(parents=True, exist_ok=True)
    atlas = bytearray()
    tiles = []
    for name in TILE_ORDER:
        tile = normalize(args.source_dir / f"{name}.png")
        tile.save(tile_dir / f"{name}.png", optimize=True)
        atlas.extend(rgb565_bytes(tile))
        tiles.append((name, tile))

    if len(atlas) != ATLAS_SIZE:
        raise AssertionError(f"atlas is {len(atlas)} bytes, expected {ATLAS_SIZE}")
    args.atlas.parent.mkdir(parents=True, exist_ok=True)
    args.atlas.write_bytes(atlas)

    columns = 3
    cell_width = 180
    cell_height = 170
    sheet = Image.new("RGB", (columns * cell_width, 3 * cell_height), "#05070b")
    draw = ImageDraw.Draw(sheet)
    for index, (name, tile) in enumerate(tiles):
        x = index % columns * cell_width
        y = index // columns * cell_height
        sheet.paste(tile, (x + 26, y + 4), tile)
        draw.text((x + 12, y + 140), f"{index:02d}  {name}", fill="#b9d7ff")
    args.contact_sheet.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(args.contact_sheet, optimize=True)
    print(f"wrote {args.atlas} ({len(atlas)} bytes)")
    print("tile order: " + ", ".join(TILE_ORDER))


if __name__ == "__main__":
    main()
