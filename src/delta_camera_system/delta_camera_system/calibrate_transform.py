import argparse
import json
from pathlib import Path

import numpy as np


def _parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Solve the rigid transform that maps camera frame XYZ points to robot base XYZ points. "
            "Input must be a JSON file containing paired camera_xyz and base_xyz measurements in millimeters."
        )
    )
    parser.add_argument(
        "--input",
        required=True,
        help="Path to a JSON file with calibration pairs.",
    )
    parser.add_argument(
        "--max-rms-mm",
        type=float,
        default=8.0,
        help="Warn if RMS fit error exceeds this threshold in millimeters.",
    )
    return parser.parse_args()


def _load_pairs(path: Path):
    data = json.loads(path.read_text())
    if isinstance(data, dict):
        pairs = data.get("pairs", [])
    elif isinstance(data, list):
        pairs = data
    else:
        raise ValueError("JSON must be either a list of pairs or an object with a field named pairs")

    parsed = []
    for index, pair in enumerate(pairs, start=1):
        if not isinstance(pair, dict):
            raise ValueError(f"Pair {index} must be an object")
        if ("camera_xyz" not in pair) or ("base_xyz" not in pair):
            raise ValueError(f"Pair {index} must contain camera_xyz and base_xyz")

        camera_xyz = np.asarray(pair["camera_xyz"], dtype=np.float64).reshape(-1)
        base_xyz = np.asarray(pair["base_xyz"], dtype=np.float64).reshape(-1)
        if not ((camera_xyz.size == 3) and (base_xyz.size == 3)):
            raise ValueError(f"Pair {index} must contain 3D XYZ vectors")

        parsed.append(
            {
                "label": pair.get("label", f"p{index}"),
                "camera_xyz": camera_xyz,
                "base_xyz": base_xyz,
            }
        )

    if len(parsed) < 3:
        raise ValueError("At least 3 point pairs are required")
    return parsed


def _solve_rigid_transform(camera_points: np.ndarray, base_points: np.ndarray):
    camera_centroid = np.mean(camera_points, axis=0)
    base_centroid = np.mean(base_points, axis=0)

    camera_centered = camera_points - camera_centroid
    base_centered = base_points - base_centroid

    cross_covariance = camera_centered.T @ base_centered
    u_mat, singular_values, vt_mat = np.linalg.svd(cross_covariance)
    rotation = vt_mat.T @ u_mat.T

    if np.linalg.det(rotation) < 0.0:
        vt_mat[-1, :] *= -1.0
        rotation = vt_mat.T @ u_mat.T

    translation = base_centroid - rotation @ camera_centroid
    return rotation, translation, singular_values


def _format_vector(vector: np.ndarray) -> str:
    return f"({vector[0]:.3f}, {vector[1]:.3f}, {vector[2]:.3f})"


def _format_matrix(matrix: np.ndarray) -> str:
    rows = []
    for row in matrix:
        rows.append(f"    ({row[0]:.9f}, {row[1]:.9f}, {row[2]:.9f}),")
    return "(\n" + "\n".join(rows) + "\n)"


def main():
    args = _parse_args()
    input_path = Path(args.input).expanduser().resolve()
    pairs = _load_pairs(input_path)

    camera_points = np.vstack([pair["camera_xyz"] for pair in pairs])
    base_points = np.vstack([pair["base_xyz"] for pair in pairs])

    rotation, translation, singular_values = _solve_rigid_transform(camera_points, base_points)

    if np.min(singular_values) <= 1e-6:
        print("Warning: point set is nearly degenerate. Spread calibration points across the workspace.")

    predicted_base = (rotation @ camera_points.T).T + translation
    errors = np.linalg.norm(predicted_base - base_points, axis=1)
    rms_error = float(np.sqrt(np.mean(errors ** 2)))
    max_error = float(np.max(errors))

    print("Camera to Base Calibration Result")
    print("=================================")
    print(f"Input file : {input_path}")
    print(f"Pairs      : {len(pairs)}")
    print(f"RMS error  : {rms_error:.3f} mm")
    print(f"Max error  : {max_error:.3f} mm")
    print(f"det(R)     : {np.linalg.det(rotation):.6f}")
    if rms_error > args.max_rms_mm:
        print(f"Warning    : RMS error is above {args.max_rms_mm:.1f} mm. Re measure your points.")

    print("\nPer point errors")
    print("----------------")
    for pair, predicted, error in zip(pairs, predicted_base, errors):
        label = pair["label"]
        camera_xyz = pair["camera_xyz"]
        actual_base = pair["base_xyz"]
        print(
            f"{label}: cam={_format_vector(camera_xyz)} -> "
            f"pred_base={_format_vector(predicted)} actual_base={_format_vector(actual_base)} "
            f"err={error:.3f} mm"
        )

    print("\nPaste this into delta_common/config.py")
    print("-------------------------------------")
    print("CAMERA_USE_DIRECT_MATRIX = True")
    print(f"CAMERA_DIRECT_MATRIX = {_format_matrix(rotation)}")
    print(f"CAM_TX_MM = {translation[0]:.6f}")
    print(f"CAM_TY_MM = {translation[1]:.6f}")
    print(f"CAM_TZ_MM = {translation[2]:.6f}")


if __name__ == "__main__":
    main()
