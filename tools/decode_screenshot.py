#!/usr/bin/env python3
"""Decode e-ink framebuffer dump from Serial output into a PNG screenshot.

Usage:
  1. Copy the base64 block between ---FRAMEBUFFER_START--- and
     ---FRAMEBUFFER_END--- from Serial Monitor into a text file, e.g. dump.txt
  2. Run:  python3 tools/decode_screenshot.py dump.txt screenshot.png

  Or pipe from serial directly:
     python3 tools/decode_screenshot.py - screenshot.png < dump.txt

The framebuffer is 256x128 pixels (padded from visible 250x122),
page-based layout: each byte holds 8 vertical pixels (LSB = topmost).
Byte index = x + (y/8) * 256.  Output PNG is cropped to 250x122.
"""

import base64
import sys
import struct


def decode_framebuffer(b64_text: str) -> bytes:
    """Decode base64 text to raw framebuffer bytes."""
    clean = b64_text.strip()
    # Remove marker lines if present
    lines = clean.splitlines()
    filtered = [
        ln for ln in lines
        if "FRAMEBUFFER_START" not in ln and "FRAMEBUFFER_END" not in ln
    ]
    return base64.b64decode("".join(filtered))


def framebuffer_to_png_bytes(raw: bytes, buf_w: int = 256, buf_h: int = 128,
                              crop_w: int = 250, crop_h: int = 122) -> bytes:
    """Convert page-based framebuffer to a PNG image (no PIL dependency).

    Buffer layout (matches OLEDDisplay / Heltec ScreenDisplay):
      byte_index = x + (y / 8) * buf_w
      bit_pos    = y % 8    (LSB = topmost pixel in the page)
    Each byte holds 8 vertical pixels; pages run top-to-bottom.

    In the framebuffer, bit=1 means WHITE pixel, bit=0 means BLACK pixel
    (WHITE color sets bits, clear() zeroes the buffer).
    """
    import zlib

    # Extract cropped pixel data (8-bit grayscale: 0=black, 255=white)
    rows = []
    for y in range(crop_h):
        row = bytearray(crop_w)
        page = y >> 3
        bit = y & 7
        for x in range(crop_w):
            byte_idx = x + page * buf_w
            pixel = (raw[byte_idx] >> bit) & 1
            row[x] = 0 if pixel else 255
        rows.append(row)

    # Build minimal PNG
    # IHDR
    ihdr_data = struct.pack(">IIBBBBB", crop_w, crop_h, 8, 0, 0, 0, 0)
    # 8-bit depth, color type 0 (grayscale), compression 0, filter 0, interlace 0

    def make_chunk(chunk_type: bytes, data: bytes) -> bytes:
        chunk = chunk_type + data
        crc = zlib.crc32(chunk) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + chunk + struct.pack(">I", crc)

    # IDAT: each row gets a filter byte (0 = None)
    raw_image = bytearray()
    for row in rows:
        raw_image.append(0)  # filter byte
        raw_image.extend(row)

    compressed = zlib.compress(bytes(raw_image), 9)

    png = bytearray()
    png.extend(b"\x89PNG\r\n\x1a\n")  # PNG signature
    png.extend(make_chunk(b"IHDR", ihdr_data))
    png.extend(make_chunk(b"IDAT", compressed))
    png.extend(make_chunk(b"IEND", b""))

    return bytes(png)


def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <input.txt | -> <output.png>")
        sys.exit(1)

    input_path = sys.argv[1]
    output_path = sys.argv[2]

    if input_path == "-":
        b64_text = sys.stdin.read()
    else:
        with open(input_path, "r") as f:
            b64_text = f.read()

    raw = decode_framebuffer(b64_text)
    expected = 4096
    if len(raw) != expected:
        print(f"Warning: expected {expected} bytes, got {len(raw)}")

    png_data = framebuffer_to_png_bytes(raw)

    with open(output_path, "wb") as f:
        f.write(png_data)

    print(f"Screenshot saved to {output_path} (250x122 px)")


if __name__ == "__main__":
    main()
