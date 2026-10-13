#!/usr/bin/env python3
"""Regenerate the synthetic shared field-map fixture; no external packages."""
import hashlib
import json
from pathlib import Path
import struct
import zlib

ROOT = Path(__file__).resolve().parent


def canonical(value):
    if value is None or isinstance(value, bool):
        return json.dumps(value, separators=(",", ":"))
    if isinstance(value, (float, int)):
        number = float(value)
        if number == 0:
            number = 0.0
        return json.dumps("f64:" + struct.pack(">d", number).hex())
    if isinstance(value, str):
        return json.dumps(value, ensure_ascii=False, separators=(",", ":"))
    if isinstance(value, list):
        return "[" + ",".join(canonical(v) for v in value) + "]"
    return "{" + ",".join(json.dumps(k, ensure_ascii=False) + ":" + canonical(value[k])
                            for k in sorted(value)) + "}"


def png_chunk(kind, body):
    return struct.pack(">I", len(body)) + kind + body + struct.pack(">I", zlib.crc32(kind + body))


def main():
    width, height = 400, 200
    rows = bytearray()
    for v in range(height):
        rows.append(0)
        for u in range(width):
            # The dark candidate region is a synthetic physical outer outline
            # with a pale hole. Pixel coordinates follow image +v down.
            obstacle = 150 <= u < 250 and 50 <= v < 150
            hole = 175 <= u < 225 and 75 <= v < 125
            color = (40, 52, 70) if obstacle and not hole else (231, 240, 238)
            rows.extend(color)
    png = (b"\x89PNG\r\n\x1a\n" + png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
           + png_chunk(b"IDAT", zlib.compress(rows, 9)) + png_chunk(b"IEND", b""))
    (ROOT / "synthetic-top-down.png").write_bytes(png)
    approved = {"state": "approved", "reviewed_revision": 1}
    manual = {"kind": "manual", "label": "Synthetic fixture coordinates; not an FRC field", "uri": None}
    points = [{"pixel": [0, 200], "field_m": [0, 0]},
              {"pixel": [400, 200], "field_m": [8, 0]},
              {"pixel": [0, 0], "field_m": [0, 4]}]
    doc = {
        "schema_version": "frc-field-map/1",
        "map": {"id": "synthetic-shared-8x4", "revision": 1, "season": None,
                "variant": "synthetic-fixture", "frame": "wpilib_nwu", "units": "m", "width_m": 8, "height_m": 4},
        "image": {"file_name": "synthetic-top-down.png", "sha256": hashlib.sha256(png).hexdigest(),
                  "width_px": width, "height_px": height, "mime_type": "image/png",
                  "attribution": "Locally generated synthetic test diagram", "license": "CC0-1.0"},
        "source": {"kind": "synthetic", "label": "SYNTHETIC TEST DIAGRAM — not verified field geometry", "uri": None},
        "calibration": {"model": "affine", "image_to_field": [0.02, 0, 0, 0, -0.02, 4, 0, 0, 1],
                        "control_points": points, "distortion": "not_applicable", "fit_error_m": 0,
                        "independent_check_error_m": 0,
                        "independent_check_points": [{"pixel": [200, 100], "field_m": [4, 2]}]},
        "boundary": {"outer": [[0, 0], [8, 0], [8, 4], [0, 4], [0, 0]], "holes": [],
                     "review": approved, "provenance": manual},
        "obstacles": [{"id": "synthetic-ring", "outer": [[3, 1], [5, 1], [5, 3], [3, 3], [3, 1]],
                       "holes": [[[3.5, 1.5], [3.5, 2.5], [4.5, 2.5], [4.5, 1.5], [3.5, 1.5]]],
                       "review": approved, "provenance": manual, "vertical_range_m": None}],
        "approval": {"state": "approved", "reviewed_revision": 1, "content_sha256": None},
    }
    digest = hashlib.sha256(canonical({k: v for k, v in doc.items() if k != "approval"}).encode()).hexdigest()
    doc["approval"]["content_sha256"] = digest
    (ROOT / "synthetic-approved.json").write_text(json.dumps(doc, indent=2, ensure_ascii=False) + "\n")
    (ROOT / "expected-digests.json").write_text(json.dumps({"image_sha256": doc["image"]["sha256"],
                                                          "content_sha256": digest}, indent=2) + "\n")
    print(json.dumps({"image_sha256": doc["image"]["sha256"], "content_sha256": digest}))


if __name__ == "__main__":
    main()
