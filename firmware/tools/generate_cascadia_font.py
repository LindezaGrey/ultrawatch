#!/usr/bin/env python3
"""Generate the small Cascadia Code bitmap subset used by the watch UI."""

from argparse import ArgumentParser
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


UI_FONT_SIZE = 72
TIME_FONT_SIZE = 120
UI_STATIC_STRINGS = (
    " 0123456789:%+-?°",
    "SO MO DI MI DO FR SA",
    "JAN FEB MRZ APR MAI JUN JUL AUG SEP OKT NOV DEZ",
    "EINSTELLUNGEN HELLIGKEIT BLUETOOTH AN AUS",
    "GPS SUCHE KARTE FEHLT SD FEHLER ZOOM",
    "WLAN WETTER HEUTE MIN MAX AKTUELL CACHE WIRD GELADEN POSITION API KEY",
    "KEIN NICHT ERREICHBAR WETTERDATEN ONE CALL AKTIV",
    "(C) GEOBASIS-DE / BKG 2026 CC BY 4.0 I",
)
UI_CHARACTERS = "".join(dict.fromkeys("".join(UI_STATIC_STRINGS)))
TIME_CHARACTERS = "0123456789:"
THRESHOLD = 128


def c_byte_string(value: str) -> str:
    """Encode the font lookup table as one byte per rendered character."""
    encoded = []
    for character in value:
        codepoint = ord(character)
        if character in ('"', "\\"):
            encoded.append("\\" + character)
        elif 0x20 <= codepoint <= 0x7E:
            encoded.append(character)
        elif codepoint <= 0xFF:
            encoded.append(f"\\{codepoint:03o}")
        else:
            raise ValueError(
                f"character {character!r} cannot use the byte renderer")
    return "".join(encoded)


def packed_rows(font: ImageFont.FreeTypeFont, character: str,
                width: int, top: int, height: int) -> list[list[int]]:
    image = Image.new("L", (width, height), 0)
    ImageDraw.Draw(image).text((0, -top), character, font=font, fill=255)
    bytes_per_row = (width + 7) // 8
    rows = []
    for y in range(height):
        row = [0] * bytes_per_row
        for x in range(width):
            if image.getpixel((x, y)) >= THRESHOLD:
                row[x // 8] |= 1 << (7 - x % 8)
        rows.append(row)
    return rows


def write_header(font_path: Path, output: Path, font_size: int,
                 characters: str, prefix: str, array_prefix: str) -> None:
    font = ImageFont.truetype(str(font_path), font_size)
    boxes = [font.getbbox(character) for character in characters]
    width = round(font.getlength("0"))
    top = min(box[1] for box in boxes)
    bottom = max(box[3] for box in boxes)
    height = bottom - top
    bytes_per_row = (width + 7) // 8

    lines = [
        "#pragma once",
        "",
        "#include <stdint.h>",
        "",
        f"/* Generated from CascadiaCode-Regular.otf at {font_size} px.",
        " * Copyright (c) 2019 - Present, Microsoft Corporation.",
        " * Cascadia Code is licensed under the SIL Open Font License 1.1.",
        " */",
        f"#define {prefix}_CELL_WIDTH {width}",
        f"#define {prefix}_GLYPH_HEIGHT {height}",
        f"#define {prefix}_BYTES_PER_ROW {bytes_per_row}",
        f"#define {prefix}_GLYPH_COUNT {len(characters)}",
        "",
        (f'static const char {array_prefix}_characters[] = '
         f'"{c_byte_string(characters)}";'),
        f"static const uint8_t {array_prefix}_glyphs",
        f"    [{prefix}_GLYPH_COUNT][{prefix}_GLYPH_HEIGHT]",
        f"    [{prefix}_BYTES_PER_ROW] = {{",
    ]

    for character in characters:
        lines.append(f"    /* {character} */ {{")
        for row in packed_rows(font, character, width, top, height):
            values = ", ".join(f"0x{value:02x}" for value in row)
            lines.append(f"        {{{values}}},")
        lines.append("    },")
    lines.extend(["};", ""])

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines))


def main() -> None:
    parser = ArgumentParser()
    parser.add_argument("font", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--time-output", type=Path)
    args = parser.parse_args()

    write_header(args.font, args.output, UI_FONT_SIZE, UI_CHARACTERS,
                 "CASCADIA_CODE", "cascadia_code")
    if args.time_output is not None:
        write_header(args.font, args.time_output, TIME_FONT_SIZE,
                     TIME_CHARACTERS, "CASCADIA_TIME", "cascadia_time")


if __name__ == "__main__":
    main()
