"""Randomize the box and car within the reachable ring (config/scene.yaml: sampling)"""
import argparse
import json
from pathlib import Path
import random
import shutil
import time
import yaml
import math
from sim_utils import (
    REPO_ROOT,
    load_scene, polar_to_xy, read_pose, place_scene
)


def sample_scene(rng: random.Random, sampling: dict, roof_offset: float) -> dict:
    """
    One random scene, not yet checked for overlap.

    Returns:
        {
            "box": {"x": 0.0, "y": -0.10, "yaw": -1.57},
            "car": {"roof_x": 0.10, "roof_y": 0.0, "x": 0.10, "y": -0.05, "yaw": 1.57}
        }
    """
    # box: yaw = theta so its faces line up with the fingers
    box_r = rng.uniform(*sampling["box_r"])
    box_theta = math.radians(rng.uniform(*sampling["box_theta"]))
    box_x, box_y = polar_to_xy(box_r, box_theta)

    # car: own angle; long side along the circle, so yaw = theta + 90 deg
    roof_r = rng.uniform(*sampling["car_r"])
    roof_theta = math.radians(rng.uniform(*sampling["car_theta"]))
    roof_x, roof_y = polar_to_xy(roof_r, roof_theta)
    car_yaw = roof_theta + math.pi / 2
    # The roof center sits roof_offset along the car's own x axis from its origin.
    car_x = roof_x - roof_offset * math.cos(car_yaw)
    car_y = roof_y - roof_offset * math.sin(car_yaw)

    return {
        "box": {"x": box_x, "y": box_y, "yaw": box_theta},
        "car": {"roof_x": roof_x, "roof_y": roof_y, "x": car_x, "y": car_y, "yaw": car_yaw},
    }

def scene_ok(s: dict, min_gap: float) -> bool:
    """False if the box is too close to the car roof (min_gap in meters).
    Box and car are on opposite sides (box_theta / car_theta), so their angles never clash."""
    box, car = s["box"], s["car"]
    return math.hypot(box["x"] - car["roof_x"], box["y"] - car["roof_y"]) >= min_gap

def sample_valid_scene(rng: random.Random, sampling: dict, roof_offset: float, max_tries: int = 100) -> dict:

    for _ in range(max_tries):
        scene = sample_scene(rng, sampling, roof_offset)
        if scene_ok(scene, sampling["min_gap"]):
            return scene
    raise ValueError(
        f"No valid scene after {max_tries} tries "
        f"(min_gap={sampling['min_gap']})"
    )

def main():
    scene = load_scene()
    box, car = scene["box"], scene["car"]

    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--episode", required=True)
    parser.add_argument("--world", default="default")
    parser.add_argument("--dry-run", action="store_true",
                        help="only print the sampled scene; do not touch Gazebo")
    args = parser.parse_args()

    s = sample_valid_scene(random.Random(args.seed), scene["sampling"], car["roof_offset"])
    if args.dry_run:
        print(json.dumps(s, indent=2))
        return

    output = REPO_ROOT / "datasets" / args.episode / "initial_pose.json"
    if output.exists():
        parser.error(f"Episode metadata already exists: {output}")

    if shutil.which("ign") is None:
        print("Warning: ign not found; saving the sampled scene without moving Gazebo.")
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("x", encoding="utf-8") as file:
            json.dump({"episode": args.episode, "seed": args.seed, "world": args.world,
                       "scene": s, "settled_pose": None, "settled_pose_verified": False},
                      file, indent=2)
        print(f"Saved: {output}")
        return

    requested = {
        box["name"]: (s["box"]["x"], s["box"]["y"], box["spawn_z"], s["box"]["yaw"]),
        car["name"]: (s["car"]["x"], s["car"]["y"], car["pose"][2], s["car"]["yaw"]),
    }
    place_scene(args.world, s, scene)

    time.sleep(1.0)  # let the box settle

    # Verify: each model ended up within 5 mm (x/y)
    settled = {name: read_pose(name) for name in requested}
    verified = all(
        math.hypot(settled[name][0] - x, settled[name][1] - y) < 0.005
        for name, (x, y, _, _) in requested.items()
    )

    metadata = {
        "episode": args.episode,
        "seed": args.seed,
        "world": args.world,
        "scene": s,
        "settled_pose": {name: dict(zip("xyz", pose)) for name, pose in settled.items()},
        "settled_pose_verified": verified,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as file:
        json.dump(metadata, file, indent=2)

    print(f"Box at ({s['box']['x']:.3f}, {s['box']['y']:.3f}), "
          f"car roof at ({s['car']['roof_x']:.3f}, {s['car']['roof_y']:.3f}), "
          f"verified={verified}")
    print(f"Saved: {output}")
    if not verified:
        raise SystemExit("A model did not settle where it was placed; see the saved file.")


if __name__ == "__main__":
    main()