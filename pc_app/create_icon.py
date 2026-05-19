"""
Generates icon.ico for TempMonitor (thermometer, dark background).
Run once: python create_icon.py
Requires: pip install Pillow
"""
import io
import struct
from PIL import Image, ImageDraw


def _rr(d: ImageDraw.Draw, box, radius: int, fill):
    x0, y0, x1, y1 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
    max_r = max(0, min((x1 - x0) // 2 - 1, (y1 - y0) // 2 - 1, radius))
    if max_r < 1 or (x1 - x0) < 3 or (y1 - y0) < 3:
        d.rectangle([x0, y0, x1, y1], fill=fill)
    else:
        d.rounded_rectangle([x0, y0, x1, y1], radius=max_r, fill=fill)


def draw(size: int) -> Image.Image:
    img = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    d   = ImageDraw.Draw(img)
    s   = float(size)

    _rr(d, [0, 0, s-1, s-1], int(s * 0.18), (22, 27, 58, 255))

    cx       = s / 2
    tube_w   = max(3, s * 0.20)
    tube_top = s * 0.10
    tube_bot = s * 0.62
    bulb_r   = max(3, s * 0.24)
    cy_bulb  = s * 0.74

    tx0 = cx - tube_w / 2
    tx1 = cx + tube_w / 2
    _rr(d, [tx0, tube_top, tx1, tube_bot], int(tube_w / 2), (230, 230, 240, 255))
    d.ellipse([cx - bulb_r, cy_bulb - bulb_r,
               cx + bulb_r, cy_bulb + bulb_r], fill=(230, 230, 240, 255))

    fill_col = (225, 50, 50, 255)
    margin   = max(1, s * 0.04)
    fill_top = s * 0.30
    ftx0 = cx - tube_w / 2 + margin
    ftx1 = cx + tube_w / 2 - margin
    if ftx1 > ftx0 + 1:
        _rr(d, [ftx0, fill_top, ftx1, tube_bot], int((ftx1 - ftx0) / 2), fill_col)
    br = bulb_r - margin
    if br > 0:
        d.ellipse([cx - br, cy_bulb - br, cx + br, cy_bulb + br], fill=fill_col)

    if size >= 32:
        tick_col = (130, 140, 180, 255)
        tick_len = max(2, int(s * 0.07))
        tw       = max(1, int(s * 0.025))
        for i in range(4):
            ty = int(fill_top + (tube_bot - fill_top) * i / 3)
            d.line([int(tx1) + 1, ty, int(tx1) + tick_len, ty],
                   fill=tick_col, width=tw)

    return img


def save_ico(path: str, images: list):
    """Write a proper multi-size ICO file (PNG frames, Vista+ compatible)."""
    png_blobs = []
    for img in images:
        buf = io.BytesIO()
        img.save(buf, format="PNG")
        png_blobs.append(buf.getvalue())

    n       = len(images)
    # ICO header: 6 bytes; each directory entry: 16 bytes
    data_offset = 6 + n * 16
    header = struct.pack("<HHH", 0, 1, n)

    dir_entries = b""
    for img, blob in zip(images, png_blobs):
        w, h = img.size
        dir_entries += struct.pack(
            "<BBBBHHII",
            w if w < 256 else 0,   # width  (0 means 256)
            h if h < 256 else 0,   # height (0 means 256)
            0,                      # colour count
            0,                      # reserved
            1,                      # planes
            32,                     # bit count
            len(blob),
            data_offset,
        )
        data_offset += len(blob)

    with open(path, "wb") as f:
        f.write(header)
        f.write(dir_entries)
        for blob in png_blobs:
            f.write(blob)


def main():
    sizes  = [16, 24, 32, 48, 64, 128, 256]
    images = [draw(s) for s in sizes]
    save_ico("icon.ico", images)
    print(f"icon.ico created successfully ({len(sizes)} sizes)")


if __name__ == "__main__":
    main()
