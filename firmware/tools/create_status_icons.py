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


def draw_map() -> Image.Image:
    image, draw = new_icon()
    cyan = (62, 220, 255, 255)
    blue = (24, 99, 255, 255)
    green = (82, 242, 178, 255)
    draw.polygon([(34, 62), (100, 37), (151, 63), (218, 40),
                  (218, 191), (151, 216), (100, 190), (34, 215)],
                 fill=(5, 18, 31, 255), outline=cyan, width=13)
    draw.line([(100, 37), (100, 190)], fill=blue, width=10)
    draw.line([(151, 63), (151, 216)], fill=blue, width=10)
    draw.ellipse((125, 89, 181, 145), fill=green)
    draw.polygon([(135, 134), (171, 134), (153, 184)], fill=green)
    return image


def draw_zoom(plus: bool) -> Image.Image:
    image, draw = new_icon()
    white = (244, 249, 255, 255)
    draw.rounded_rectangle((38, 111, 218, 145), radius=17, fill=white)
    if plus:
        draw.rounded_rectangle((111, 38, 145, 218), radius=17, fill=white)
    return image


def draw_map_center() -> Image.Image:
    image, draw = new_icon()
    white = (244, 249, 255, 255)
    draw.ellipse((57, 57, 199, 199), outline=white, width=22)
    draw.rounded_rectangle((116, 24, 140, 91), radius=12, fill=white)
    draw.rounded_rectangle((116, 165, 140, 232), radius=12, fill=white)
    draw.rounded_rectangle((24, 116, 91, 140), radius=12, fill=white)
    draw.rounded_rectangle((165, 116, 232, 140), radius=12, fill=white)
    draw.ellipse((111, 111, 145, 145), fill=white)
    return image


def draw_wifi(state: str) -> Image.Image:
    image, draw = new_icon()
    color = MUTED if state == "off" else RED if state == "error" else BLUE
    center = (128, 184)
    for radius, width in ((34, 17), (70, 18), (108, 19)):
        box = (center[0] - radius, center[1] - radius,
               center[0] + radius, center[1] + radius)
        draw.arc(box, 215, 325, fill=color, width=width)
    draw.ellipse((115, 177, 141, 203), fill=color)
    if state == "off":
        draw.line((48, 48, 208, 208), fill=RED, width=22)
    elif state == "connecting":
        draw.ellipse((181, 42, 215, 76), fill=color)
    elif state == "error":
        draw.rounded_rectangle((184, 39, 208, 116), radius=12, fill=color)
        draw.ellipse((184, 129, 208, 153), fill=color)
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
    draw_map().save(args.output_dir / "map.png", optimize=True)
    draw_zoom(True).save(args.output_dir / "zoom_in.png", optimize=True)
    draw_zoom(False).save(args.output_dir / "zoom_out.png", optimize=True)
    draw_map_center().save(args.output_dir / "map_center.png", optimize=True)
    for state in ("off", "connecting", "connected", "error"):
        draw_wifi(state).save(args.output_dir / f"wifi_{state}.png",
                              optimize=True)


if __name__ == "__main__":
    main()
