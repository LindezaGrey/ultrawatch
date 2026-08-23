#!/usr/bin/env python3
"""Normalize UltraWatch icon sources and build the SD-card RGB565 atlas."""

from argparse import ArgumentParser
from pathlib import Path
import struct

from PIL import Image, ImageDraw


TILE_SIZE = 48
TILE_ORDER = (
    "launcher", "clock", "settings", "activity", "heart", "sleep",
    "wellness", "weather", "music", "messages", "rings",
    "ble_off", "ble_on", "battery_empty", "battery_low",
    "battery_medium", "battery_full", "map",
    "zoom_in", "zoom_out", "map_center",
    "wifi_off", "wifi_connecting", "wifi_connected", "wifi_error",
)
ATLAS_SIZE = len(TILE_ORDER) * TILE_SIZE * TILE_SIZE * 2


def normalize(source: Path) -> Image.Image:
    image = Image.open(source).convert("RGBA")
    if image.getpixel((0, 0))[3] != 0 or image.getpixel((image.width - 1, image.height - 1))[3] != 0:
        raise ValueError(f"{source}: chroma-key removal did not clear the corners")
    bounds = image.getchannel("A").getbbox()
    if bounds is None:
        raise ValueError(f"{source}: no visible icon pixels")
    icon = image.crop(bounds)
    maximum = 48 if source.stem in {
        "zoom_in", "zoom_out", "map_center", "wifi_off",
        "wifi_connecting", "wifi_connected", "wifi_error",
    } else 42
    icon.thumbnail((maximum, maximum), Image.Resampling.LANCZOS)
    tile = Image.new("RGBA", (TILE_SIZE, TILE_SIZE), (0, 0, 0, 0))
    tile.alpha_composite(icon, ((TILE_SIZE - icon.width) // 2,
                                (TILE_SIZE - icon.height) // 2))
    return tile


def rgb565_bytes(tile: Image.Image) -> bytes:
    black = Image.new("RGB", tile.size, "black")
    black.paste(tile, mask=tile.getchannel("A"))
    output = bytearray()
    pixels = black.load()
    for y in range(black.height):
        for x in range(black.width):
            red, green, blue = pixels[x, y]
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
    tiles = []
    atlas = bytearray()
    for name in TILE_ORDER:
        tile = normalize(args.source_dir / f"{name}.png")
        tile.save(tile_dir / f"{name}.png", optimize=True)
        tiles.append((name, tile))
        atlas.extend(rgb565_bytes(tile))

    if len(atlas) != ATLAS_SIZE:
        raise AssertionError(f"atlas is {len(atlas)} bytes, expected {ATLAS_SIZE}")
    args.atlas.parent.mkdir(parents=True, exist_ok=True)
    args.atlas.write_bytes(atlas)

    columns = 4
    cell_width = 116
    cell_height = 80
    rows = (len(tiles) + columns - 1) // columns
    sheet = Image.new("RGB", (columns * cell_width, rows * cell_height), "#080b12")
    draw = ImageDraw.Draw(sheet)
    for index, (name, tile) in enumerate(tiles):
        x = (index % columns) * cell_width
        y = (index // columns) * cell_height
        sheet.paste(tile, (x + (cell_width - TILE_SIZE) // 2, y + 5), tile)
        draw.text((x + 8, y + 58), f"{index:02d} {name}", fill="#b9d7ff")
    args.contact_sheet.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(args.contact_sheet, optimize=True)
    print(f"wrote {args.atlas} ({len(atlas)} bytes)")
    print("tile order: " + ", ".join(TILE_ORDER))


if __name__ == "__main__":
    main()
