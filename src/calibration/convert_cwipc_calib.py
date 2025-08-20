#!/usr/bin/env python3
import argparse
import json
import os
import sys
from typing import Any, Dict, List


def load_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(data: Dict[str, Any], path: str) -> None:
    # Ensure target directory exists
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4, ensure_ascii=False)
        f.write("\n")


def flatten_row_major(matrix: List[List[float]]) -> List[float]:
    # Expect 4x4
    if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
        raise ValueError("Expected a 4x4 matrix for 'trafo'")
    return [matrix[i][j] for i in range(4) for j in range(4)]


def convert_cwipc_to_devices_config(cwipc: Dict[str, Any]) -> Dict[str, Any]:
    cameras = cwipc.get("camera")
    if not isinstance(cameras, list):
        raise ValueError("cwipc calibration must contain a 'camera' array")

    # ROI dimensions from cwipc hardware section if available
    hardware = cwipc.get("hardware", {}) if isinstance(cwipc.get("hardware"), dict) else {}
    roi_width = int(hardware.get("color_width", 1280))
    roi_height = int(hardware.get("color_height", 720))

    devices_config: Dict[str, Any] = {}

    for cam in cameras:
        serial = cam.get("serial")
        trafo = cam.get("trafo")
        if not serial or trafo is None:
            # Skip malformed entries
            continue

        try:
            flat = flatten_row_major(trafo)
        except Exception as exc:
            raise ValueError(f"Camera with serial {serial} has invalid 'trafo': {exc}") from exc

        coord_transform = {str(i): float(flat[i]) for i in range(16)}

        devices_config[str(serial)] = {
            "disabled": False,
            "ROI": {
                "start_x": 0,
                "start_y": 0,
                "width": roi_width,
                "height": roi_height,
            },
            "coord_transform": coord_transform,
        }

    if not devices_config:
        raise ValueError("No valid cameras with 'serial' and 4x4 'trafo' found in cwipc calibration")

    return devices_config


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert cwipc cameraconfig 'trafo' into UVG calibration format.")
    parser.add_argument(
        "--cwipc_calib_path",
        required=True,
        help="Path to cwipc cameraconfig.json (input)",
    )
    parser.add_argument(
        "--template",
        default=os.path.join("src", "calibration", "template.json"),
        help="Path to UVG calibration template.json (input)",
    )
    parser.add_argument(
        "--output",
        default="test_calib.json",
        help="Path to write the generated calibration JSON",
    )

    args = parser.parse_args()

    try:
        template = load_json(args.template)
    except FileNotFoundError:
        print(f"Template not found: {args.template}", file=sys.stderr)
        sys.exit(1)

    try:
        cwipc = load_json(args.cwipc_calib_path)
    except FileNotFoundError:
        print(f"cwipc calibration not found: {args.cwipc_calib_path}", file=sys.stderr)
        sys.exit(1)

    devices_config = convert_cwipc_to_devices_config(cwipc)

    # Merge into template
    output_data = dict(template)
    output_data["devices_config"] = devices_config

    save_json(output_data, args.output)
    print(f"Wrote calibration to: {os.path.abspath(args.output)}")


if __name__ == "__main__":
    main()


