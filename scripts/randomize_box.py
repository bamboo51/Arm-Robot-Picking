import argparse
import json
from pathlib import Path
import random
import re
import subprocess
import time

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--episode", required=True)
    parser.add_argument("--world", default="default")
    parser.add_argument("--model", default="pick_box")

    # はじめての最初位置（元々Boxが置かれたところより少し変えたもの）
    parser.add_argument("--xmin", type=float, default=-0.02)
    parser.add_argument("--xmax", type=float, default=0.02)
    parser.add_argument("--ymin", type=float, default=-0.12)
    parser.add_argument("--ymax", type=float, default=-0.08)

    # モデルの元々の高さ (原点は箱の底面、テーブル上面 = 1.015)
    parser.add_argument("--z", type=float, default=1.015)
    args = parser.parse_args()

    if args.xmin > args.xmax or args.ymin > args.ymax:
        parser.error("Minimum bounds must not exceed maximum bounds.")

    output = (
        Path(__file__).resolve().parents[1]
        / "datasets"
        / args.episode
        / "initial_pose.json"
    )
    if output.exists():
        parser.error(f"Episode metadata already exists: {output}")

    rng = random.Random(args.seed)
    x = rng.uniform(args.xmin, args.xmax)
    y = rng.uniform(args.ymin, args.ymax)

    request = (
        f"name: {json.dumps(args.model)} "
        f"position: {{x: {x:.9f}, y: {y:.9f}, z: {args.z:.9f}}} "
        "orientation: {x: 0, y: 0, z: 0, w: 1}"
    )

    command = [
        "ign", "service",
        "-s", f"/world/{args.world}/set_pose",
        "--reqtype", "ignition.msgs.Pose",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", "5000",
        "--req", request,
    ]

    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise SystemExit(f"Could not call Gazebo service: {exc}")

    response = result.stdout + result.stderr
    if result.returncode != 0 or not re.search(
        r"\bdata:\s*true\b", response
    ):
        raise SystemExit(f"Gazebo did not accept the pose:\n{response}")

    time.sleep(1.0)

    metadata = {
        "episode": args.episode,
        "seed": args.seed,
        "world": args.world,
        "model": args.model,
        "requested_model_pose": {
            "position": {"x": x, "y": y, "z": args.z},
            "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
        },
        "sampling_bounds": {
            "xmin": args.xmin, "xmax": args.xmax,
            "ymin": args.ymin, "ymax": args.ymax,
        },
        "settled_pose_verified": False,
    }

    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as file:
        json.dump(metadata, file, indent=2)

    print(f"Requested box position: ({x:.4f}, {y:.4f}, {args.z:.4f})")
    print(f"Saved: {output}")


if __name__ == "__main__":
    main()