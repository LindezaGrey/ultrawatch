#!/usr/bin/env python3
"""Decode a UWatch screenshot from serial console capture.

The watch prints, on the 'shot' console command:
    ==SHOT:<w>x<h>==
    <raw RGB565 little-endian pixels>
    ==ENDSHOT==

Usage:
    python3 tools/decode_screenshot.py capture.bin out.png

The capture may contain other console output before/after the framed shot;
the script extracts the first complete ==SHOT==..==ENDSHOT== block.
"""
import re
import struct
import sys


def main():
    if len(sys.argv) != 3:
        print(__doc__, file=sys.stderr)
        return 2

    with open(sys.argv[1], "rb") as f:
        data = f.read()

    m = re.search(rb"==SHOT:(\d+)x(\d+)==\r?\n(.*?)==ENDSHOT==", data, re.S)
    if not m:
        print("no ==SHOT==..==ENDSHOT== frame found", file=sys.stderr)
        return 1

    w, h = int(m.group(1)), int(m.group(2))
    raw = m.group(3)
    need = w * h * 2
    if len(raw) > need:
        print(f"warning: payload {len(raw)}/{need} B; truncating", file=sys.stderr)
        raw = raw[:need]
    elif len(raw) < need:
        print(f"warning: short payload {len(raw)}/{need} B; black-padding", file=sys.stderr)
        raw = raw + b"\x00\x00" * (need - len(raw))

    # RGB565 little-endian -> 24-bit RGB, top-down.
    px = bytearray(w * h * 3)
    for i in range(w * h):
        lo, hi = raw[i * 2], raw[i * 2 + 1]
        v = lo | (hi << 8)
        r = (v >> 11) & 0x1F
        g = (v >> 5) & 0x3F
        b = v & 0x1F
        px[i * 3 + 0] = (r * 255 + 15) // 31
        px[i * 3 + 1] = (g * 255 + 31) // 63
        px[i * 3 + 2] = (b * 255 + 15) // 31

    # Write PNG (no deps) or fall back to PPM.
    if sys.argv[2].lower().endswith(".png"):
        write_png(sys.argv[2], w, h, px)
    else:
        with open(sys.argv[2], "wb") as f:
            f.write(f"P6\n{w} {h}\n255\n".encode())
            f.write(px)

    print(f"wrote {w}x{h} screenshot -> {sys.argv[2]}")
    return 0


def write_png(path, w, h, rgb):
    import zlib

    def chunk(tag, payload):
        c = tag + payload
        return struct.pack(">I", len(payload)) + c + struct.pack(">I", zlib.crc32(c))

    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
    raw = b"".join(b"\x00" + bytes(rgb[y * w * 3:(y + 1) * w * 3]) for y in range(h))
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", ihdr))
        f.write(chunk(b"IDAT", zlib.compress(raw, 9)))
        f.write(chunk(b"IEND", b""))


if __name__ == "__main__":
    sys.exit(main())
