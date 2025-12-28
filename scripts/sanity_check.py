#!/usr/bin/env python3
"""
Sanity check for the Visual Odometry dataset.

What it does:
- Loads camera.dat (K, width/height, z_near/z_far)
- Finds available meas-XXXXX.dat frames
- Loads the first (or a selected) meas file
- Prints basic stats and checks formatting assumptions:
  - each measurement line has: local_id, gt_id, u, v, 10 appearance floats

Usage:
  python scripts/sanity_check.py --data_dir data/dataset_name
  python scripts/sanity_check.py --data_dir data/dataset_name --frame 17
"""

from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple, Optional

import numpy as np


@dataclass
class CameraInfo:
    K: np.ndarray          # (3,3)
    width: int
    height: int
    z_near: float
    z_far: float


@dataclass
class Observation:
    local_meas_id: int
    gt_landmark_id: int
    u: float   # col
    v: float   # row
    appearance: np.ndarray  # (10,)


_FLOAT_RE = re.compile(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?")


def _floats_in_line(s: str) -> List[float]:
    return [float(x) for x in _FLOAT_RE.findall(s)]


def load_camera_dat(camera_path: Path) -> CameraInfo:
    if not camera_path.exists():
        raise FileNotFoundError(f"camera.dat not found at: {camera_path}")

    lines = [ln.strip() for ln in camera_path.read_text().splitlines() if ln.strip()]

    # camera matrix block
    iK = next(i for i, ln in enumerate(lines) if ln.lower().startswith("camera matrix"))
    K = np.array(
        [
            _floats_in_line(lines[iK + 1]),
            _floats_in_line(lines[iK + 2]),
            _floats_in_line(lines[iK + 3]),
        ],
        dtype=float,
    )
    if K.shape != (3, 3):
        raise ValueError(f"Bad camera matrix shape {K.shape}. Parsed K=\n{K}")

    def read_scalar(prefix: str) -> float:
        idx = next(i for i, ln in enumerate(lines) if ln.lower().startswith(prefix))
        vals = _floats_in_line(lines[idx])
        if not vals:
            raise ValueError(f"Could not parse scalar from line: '{lines[idx]}'")
        return float(vals[0])

    z_near = read_scalar("z_near")
    z_far = read_scalar("z_far")
    width = int(read_scalar("width"))
    height = int(read_scalar("height"))

    return CameraInfo(K=K, width=width, height=height, z_near=z_near, z_far=z_far)


def list_meas_files(data_dir: Path) -> List[Tuple[int, Path]]:
    meas_files = sorted(data_dir.glob("meas-*.dat"))
    out: List[Tuple[int, Path]] = []
    for p in meas_files:
        m = re.match(r"meas-(\d+)\.dat$", p.name)
        if not m:
            continue
        out.append((int(m.group(1)), p))
    out.sort(key=lambda x: x[0])
    return out

def load_meas_file(meas_path: Path, expected_app_dim: int = 10) -> List[Observation]:
    if not meas_path.exists():
        raise FileNotFoundError(f"meas file not found at: {meas_path}")

    obs: List[Observation] = []
    with meas_path.open("r") as f:
        for line_no, ln in enumerate(f, start=1):
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue

            # Skip header/meta lines like: "seq: 0", "gt: ...", "odom: ..."
            # Heuristic: if it contains ":" OR doesn't start with a number/sign, skip.
            if ":" in ln:
                continue
            if not (ln[0].isdigit() or ln[0] == "-" or ln[0] == "+"):
                continue

            tok = ln.split()
            if len(tok) < 4 + expected_app_dim:
                raise ValueError(
                    f"{meas_path.name}:{line_no}: "
                    f"Expected at least {4+expected_app_dim} tokens, got {len(tok)}. Line='{ln[:120]}'"
                )

            local_id = int(tok[0])
            gt_id = int(tok[1])
            u = float(tok[2])
            v = float(tok[3])
            app = np.array([float(x) for x in tok[4:4 + expected_app_dim]], dtype=np.float32)

            obs.append(
                Observation(
                    local_meas_id=local_id,
                    gt_landmark_id=gt_id,
                    u=u,
                    v=v,
                    appearance=app,
                )
            )
    return obs



def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data_dir", type=str, required=True, help="Folder containing camera.dat and meas-*.dat")
    parser.add_argument("--frame", type=int, default=None, help="Frame index to load (default: first available)")
    parser.add_argument("--print_rows", type=int, default=3, help="How many sample observations to print")
    args = parser.parse_args()

    data_dir = Path(args.data_dir).expanduser().resolve()
    if not data_dir.exists():
        raise FileNotFoundError(f"data_dir does not exist: {data_dir}")

    camera_path = data_dir / "camera.dat"
    cam = load_camera_dat(camera_path)

    print("=== CAMERA ===")
    print("K:\n", cam.K)
    print(f"width x height: {cam.width} x {cam.height}")
    print(f"z_near / z_far: {cam.z_near} / {cam.z_far}")

    meas = list_meas_files(data_dir)
    if not meas:
        raise FileNotFoundError(f"No meas-*.dat files found in {data_dir}")

    print("\n=== MEAS FILES ===")
    print(f"Found {len(meas)} frames: [{meas[0][0]} .. {meas[-1][0]}]")
    if len(meas) <= 10:
        print("Frames:", [i for i, _ in meas])

    # pick frame
    if args.frame is None:
        frame_idx, meas_path = meas[0]
    else:
        d = dict(meas)
        if args.frame not in d:
            raise ValueError(f"Requested frame {args.frame} not found. Available: {meas[0][0]}..{meas[-1][0]}")
        frame_idx, meas_path = args.frame, d[args.frame]

    obs = load_meas_file(meas_path)

    # basic stats
    uv = np.array([(o.u, o.v) for o in obs], dtype=float)
    app = np.stack([o.appearance for o in obs], axis=0) if obs else np.zeros((0, 10), dtype=np.float32)

    print("\n=== FRAME CONTENT ===")
    print(f"Loaded frame {frame_idx} from {meas_path.name}")
    print(f"#observations: {len(obs)}")
    if len(obs) > 0:
        print(f"u range: [{uv[:,0].min():.3f}, {uv[:,0].max():.3f}]  (expected within [0,{cam.width}))")
        print(f"v range: [{uv[:,1].min():.3f}, {uv[:,1].max():.3f}]  (expected within [0,{cam.height}))")
        print(f"appearance shape: {app.shape} (expected N x 10)")
        print(f"appearance stats: mean={app.mean():.4f}, std={app.std():.4f}, min={app.min():.4f}, max={app.max():.4f}")

        # check out-of-bounds pixels
        oob = np.sum((uv[:, 0] < 0) | (uv[:, 0] >= cam.width) | (uv[:, 1] < 0) | (uv[:, 1] >= cam.height))
        print(f"out-of-bounds image points: {int(oob)}")

        print("\nSample observations:")
        for i in range(min(args.print_rows, len(obs))):
            o = obs[i]
            print(
                f"  [{i}] local_id={o.local_meas_id} gt_id={o.gt_landmark_id} "
                f"uv=({o.u:.2f},{o.v:.2f}) app[:3]={o.appearance[:3].tolist()}"
            )

    print("\n✅ Sanity check done.")


if __name__ == "__main__":
    main()

