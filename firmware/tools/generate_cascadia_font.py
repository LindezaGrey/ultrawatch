#!/usr/bin/env python3
"""Generate the small Cascadia Code bitmap subset used by the watch UI."""

from argparse import ArgumentParser
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


FONT_SIZE = 72
CHARACTERS = "0123456789:%-"
THRESHOLD = 128


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


def main() -> None:
    parser = ArgumentParser()
    parser.add_argument("font", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    font = ImageFont.truetype(str(args.font), FONT_SIZE)
    boxes = [font.getbbox(character) for character in CHARACTERS]
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
        "/* Generated from CascadiaCode-Regular.otf at 72 px.",
        " * Copyright (c) 2019 - Present, Microsoft Corporation.",
        " * Cascadia Code is licensed under the SIL Open Font License 1.1.",
        " */",
        f"#define CASCADIA_CODE_CELL_WIDTH {width}",
        f"#define CASCADIA_CODE_GLYPH_HEIGHT {height}",
        f"#define CASCADIA_CODE_BYTES_PER_ROW {bytes_per_row}",
        f"#define CASCADIA_CODE_GLYPH_COUNT {len(CHARACTERS)}",
        "",
        f'static const char cascadia_code_characters[] = "{CHARACTERS}";',
        "static const uint8_t cascadia_code_glyphs",
        "    [CASCADIA_CODE_GLYPH_COUNT][CASCADIA_CODE_GLYPH_HEIGHT]",
        "    [CASCADIA_CODE_BYTES_PER_ROW] = {",
    ]

    for character in CHARACTERS:
        lines.append(f"    /* {character} */ {{")
        for row in packed_rows(font, character, width, top, height):
            values = ", ".join(f"0x{value:02x}" for value in row)
            lines.append(f"        {{{values}}},")
        lines.append("    },")
    lines.extend(["};", ""])

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text("\n".join(lines))


if __name__ == "__main__":
    main()
