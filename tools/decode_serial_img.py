#!/usr/bin/env python3
import re
import struct
import sys
import zlib


def write_png(path, width, height, rgb):
    def chunk(tag, data):
        return (
            struct.pack(">I", len(data))
            + tag
            + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    raw = bytearray()
    for y in range(height):
        raw.append(0)
        raw += rgb[y * width * 3:(y + 1) * width * 3]

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(bytes(raw), 9))
    png += chunk(b"IEND", b"")

    with open(path, "wb") as f:
        f.write(png)


def rgb565_to_rgb(data):
    out = bytearray(len(data) // 2 * 3)
    j = 0
    for i in range(0, len(data) - 1, 2):
        c = (data[i] << 8) | data[i + 1]
        out[j] = ((c >> 11) & 0x1F) * 255 // 31
        out[j + 1] = ((c >> 5) & 0x3F) * 255 // 63
        out[j + 2] = (c & 0x1F) * 255 // 31
        j += 3
    return out


def main():
    if len(sys.argv) < 3:
        print(f"usage: {sys.argv[0]} <serial.log> <out.png>")
        return 1

    with open(sys.argv[1], "rb") as f:
        text = f.read().decode("latin1", errors="ignore")

    blocks = re.findall(
        r"IMGSTART\s+(\d+)\s+(\d+)\s+(RGB565|GRAY8|YUYV)(.*?)IMGEND", text, re.S)
    if not blocks:
        print("no IMGSTART/IMGEND block found")
        return 2

    width, height, fmt, body = blocks[-1]
    width = int(width)
    height = int(height)
    hexstr = "".join(re.findall(r"IMGDATA\s+([0-9A-Fa-f]+)", body))
    raw = bytes.fromhex(hexstr)
    need = width * height * (1 if fmt == "GRAY8" else 2)

    if len(raw) < need:
        print(f"short image data: {len(raw)} < {need}")
        return 3

    if fmt == "RGB565":
        rgb = rgb565_to_rgb(raw[:need])
    elif fmt == "YUYV":
        rgb = bytearray()
        gray = bytearray()
        us = []
        vs = []
        for k in range(width * height):
            base = (k // 2) * 4
            y = raw[2 * k]
            u = raw[base + 1]
            v = raw[base + 3]
            us.append(u)
            vs.append(v)
            r = y + 1.402 * (v - 128)
            g = y - 0.344 * (u - 128) - 0.714 * (v - 128)
            b = y + 1.772 * (u - 128)
            rgb += bytes((max(0, min(255, int(r))), max(0, min(255, int(g))), max(0, min(255, int(b)))))
            gray += bytes((y, y, y))
        import statistics
        print(f"chroma U mean:{statistics.mean(us):.1f} stdev:{statistics.pstdev(us):.1f} "
              f"V mean:{statistics.mean(vs):.1f} stdev:{statistics.pstdev(vs):.1f}")
        write_png(sys.argv[2].replace(".png", "_gray.png"), width, height, gray)
    else:
        rgb = bytearray()
        for v in raw[:need]:
            rgb += bytes((v, v, v))

    write_png(sys.argv[2], width, height, rgb)
    print(f"saved {sys.argv[2]} ({width}x{height})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
