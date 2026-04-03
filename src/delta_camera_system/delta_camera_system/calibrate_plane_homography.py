import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def _parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Solve the planar homography that maps image pixel points (u, v) "
            "to robot-base plane coordinates (x, y) in millimeters."
        )
    )
    parser.add_argument(
        "--input",
        required=True,
        help="Path to a JSON file containing paired pixel_uv and base_xy points.",
    )
    parser.add_argument(
        "--max-rms-mm",
        type=float,
        default=3.0,
        help="Warn if RMS reprojection error exceeds this threshold in millimeters.",
    )
    return parser.parse_args()


def _load_pairs(path: Path):
    data = json.loads(path.read_text())
    if isinstance(data, dict):
        pairs = data.get("pairs", [])
    elif isinstance(data, list):
        pairs = data
    else:
        raise ValueError("JSON must be a list or an object with a field named pairs")

    parsed = []
    for index, pair in enumerate(pairs, start=1):
        if not isinstance(pair, dict):
            raise ValueError(f"Pair {index} must be an object")
        if ("pixel_uv" not in pair) or ("base_xy" not in pair):
            raise ValueError(f"Pair {index} must contain pixel_uv and base_xy")

        pixel_uv = np.asarray(pair["pixel_uv"], dtype=np.float64).reshape(-1)
        base_xy = np.asarray(pair["base_xy"], dtype=np.float64).reshape(-1)
        if pixel_uv.size != 2 or base_xy.size != 2:
            raise ValueError(f"Pair {index} must contain 2D vectors")

        parsed.append(
            {
                "label": pair.get("label", f"p{index}"),
                "pixel_uv": pixel_uv,
                "base_xy": base_xy,
            }
        )

    if len(parsed) < 4:
        raise ValueError("At least 4 point pairs are required")
    return parsed


def _format_matrix(matrix: np.ndarray) -> str:
    rows = []
    for row in matrix:
        rows.append(f"    ({row[0]:.9f}, {row[1]:.9f}, {row[2]:.9f}),")
    return "(\n" + "\n".join(rows) + "\n)"


def main():
    args = _parse_args()
    input_path = Path(args.input).expanduser().resolve()
    pairs = _load_pairs(input_path)

    image_points = np.asarray([pair["pixel_uv"] for pair in pairs], dtype=np.float64).reshape(-1, 1, 2)
    base_points = np.asarray([pair["base_xy"] for pair in pairs], dtype=np.float64).reshape(-1, 1, 2)

    homography, inlier_mask = cv2.findHomography(image_points, base_points, method=0)
    if homography is None:
        raise RuntimeError("Failed to solve planar homography")

    projected = cv2.perspectiveTransform(image_points.astype(np.float32), homography.astype(np.float64))
    errors = np.linalg.norm(projected.reshape(-1, 2) - base_points.reshape(-1, 2), axis=1)
    rms_error = float(np.sqrt(np.mean(errors ** 2)))
    max_error = float(np.max(errors))

    print("Image to Base Plane Homography")
    print("==============================")
    print(f"Input file : {input_path}")
    print(f"Pairs      : {len(pairs)}")
    print(f"RMS error  : {rms_error:.3f} mm")
    print(f"Max error  : {max_error:.3f} mm")
    if rms_error > args.max_rms_mm:
        print(f"Warning    : RMS error is above {args.max_rms_mm:.1f} mm. Re measure your points.")

    print("\nPer point errors")
    print("----------------")
    for pair, prediction, error in zip(pairs, projected.reshape(-1, 2), errors):
        print(
            f"{pair['label']}: uv=({pair['pixel_uv'][0]:.3f}, {pair['pixel_uv'][1]:.3f}) -> "
            f"pred_base=({prediction[0]:.3f}, {prediction[1]:.3f}) "
            f"actual_base=({pair['base_xy'][0]:.3f}, {pair['base_xy'][1]:.3f}) err={error:.3f} mm"
        )

    print("\nPaste this into delta_common/config.py")
    print("-------------------------------------")
    print("PLANE_HOMOGRAPHY_ENABLE = True")
    print(f"PLANE_HOMOGRAPHY_MATRIX = {_format_matrix(homography)}")


if __name__ == "__main__":
    main()
