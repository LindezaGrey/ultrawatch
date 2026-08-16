#!/usr/bin/env python3
"""Create reproducible BLE and battery source icons for the UI atlas."""

from argparse import ArgumentParser
from pathlib import Path

from PIL import Image, ImageDraw


CANVAS = 256
BLUE = (54, 172, 255, 255)
MUTED = (112, 126, 146, 255)
GREEN = (79, 226, 157, 255)
RED = (238, 92, 99, 255)


def new_icon() -> tuple[Image.Image, ImageDraw.ImageDraw]:
    image = Image.new("RGBA", (CANVAS, CANVAS), (0, 0, 0, 0))
    return image, ImageDraw.Draw(image)


def draw_ble(enabled: bool) -> Image.Image:
    image, draw = new_icon()
    color = BLUE if enabled else MUTED
    points = [(128, 30), (190, 91), (128, 128), (190, 165), (128, 226)]
    draw.line(points, fill=color, width=22, joint="curve")
    draw.line((128, 30, 128, 226), fill=color, width=22)
    draw.line((65, 70, 190, 165), fill=color, width=22)
    draw.line((65, 186, 190, 91), fill=color, width=22)
    if not enabled:
        draw.line((48, 45, 208, 211), fill=(238, 92, 99, 255), width=24)
    return image


def draw_battery(level: int) -> Image.Image:
    image, draw = new_icon()
    outline = GREEN if level else RED
    draw.rounded_rectangle((31, 61, 207, 195), radius=20,
                           outline=outline, width=18)
    draw.rounded_rectangle((207, 101, 231, 155), radius=7, fill=outline)
    if level:
        right = 50 + round(137 * level / 3)
        draw.rounded_rectangle((50, 80, right, 176), radius=10, fill=GREEN)
    return image


def main() -> None:
    parser = ArgumentParser()
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    draw_ble(False).save(args.output_dir / "ble_off.png", optimize=True)
    draw_ble(True).save(args.output_dir / "ble_on.png", optimize=True)
    for name, level in (("empty", 0), ("low", 1), ("medium", 2),
                        ("full", 3)):
        draw_battery(level).save(args.output_dir / f"battery_{name}.png",
                                 optimize=True)


if __name__ == "__main__":
    main()
