"""Randomize the box and car within the reachable ring (config/scene.yaml: sampling)"""
import argparse
import json
from pathlib import Path
import random
import re
import shutil
import subprocess
import time
import yaml
import math

REPO_ROOT = Path(__file__).resolve().parents[1] 
SCENE_FILE = REPO_ROOT / "config" / "scene.yaml"

def polar_to_xy(r: float, theta: float) -> tuple[float, float]:
    """convert polar to xy, theta in rad"""
    return (r * math.cos(theta), r * math.sin(theta))

def yaw_quaternion(yaw: float) -> dict:
    """Rotation about z only. yaw 0 -> {x:0, y:0, w:1}"""
    return {"x": 0, "y": 0, "z": math.sin(yaw/2), "w": math.cos(yaw/2)}

def sample_scene(rng: random.Random, sampling: dict, roof_offset: float) -> dict:
    """
    One random scene, not yet checked for overlap.

    Returns:
        {
            "box": {"x": 0.0, "y": -0.10, "yaw": -1.57},
            "car": {"roof_x": 0.10, "roof_y": 0.0, "x": 0.10, "y": -0.05, "yaw": 1.57}
        }
    """
    theta_min, theta_max = sampling["theta_range"]

    # box: yaw = theta so its faces line up with the fingers
    box_r = rng.uniform(*sampling["box_r"])
    box_theta = math.radians(rng.uniform(theta_min, theta_max))
    box_x, box_y = polar_to_xy(box_r, box_theta)

    # car: own angle; long side along the circle, so yaw = theta + 90 deg
    roof_r = rng.uniform(*sampling["car_r"])
    roof_theta = math.radians(rng.uniform(theta_min, theta_max))
    roof_x, roof_y = polar_to_xy(roof_r, roof_theta)
    car_yaw = roof_theta + math.pi / 2
    # The roof center sits roof_offset along the car's own x axis from its origin.
    car_x = roof_x - roof_offset * math.cos(car_yaw)
    car_y = roof_y - roof_offset * math.sin(car_yaw)

    return {
        "box": {"x": box_x, "y": box_y, "yaw": box_theta},
        "car": {"roof_x": roof_x, "roof_y": roof_y, "x": car_x, "y": car_y, "yaw": car_yaw},
    }

def scene_ok(s: dict, min_gap: float, min_angle: float) -> bool:
    """False if the box is too close to the car roof, or the two are too close in angle.

    min_gap in meters, min_angle in degrees.
    """
    box, car = s["box"], s["car"]
    if math.hypot(box["x"] - car["roof_x"], box["y"] - car["roof_y"]) < min_gap:
        return False

    box_angle = math.degrees(math.atan2(box["y"], box["x"]))
    car_angle = math.degrees(math.atan2(car["roof_y"], car["roof_x"]))
    # Wrap into [-180, 180) so -170 vs +170 counts as 20 degrees apart, not 340.
    diff = (box_angle - car_angle + 180) % 360 - 180
    return abs(diff) >= min_angle

def sample_valid_scene(rng: random.Random, sampling: dict, roof_offset: float, max_tries: int = 100) -> dict:

    for _ in range(max_tries):
        scene = sample_scene(rng, sampling, roof_offset)
        if(scene_ok(scene, sampling["min_gap"], sampling["min_angle"])):
            return scene
    raise ValueError(
        f"No valid scene after {max_tries} tries "
        f"(min_gap={sampling['min_gap']}, min_angle={sampling['min_angle']})"
    )


def run(command: list[str]) -> str:
    """Run a Gazebo CLI command and return its output. Exit with a clear message on failure."""
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise SystemExit(f"Could not run {command[0]} (is Gazebo running?): {exc}")
    return result.stdout + result.stderr


def set_pose(world: str, name: str, x: float, y: float, z: float, yaw: float) -> None:
    """Move one Gazebo model. Exit if Gazebo does not accept it."""
    q = yaw_quaternion(yaw)
    request = (
        f"name: {json.dumps(name)} "
        f"position: {{x: {x:.9f}, y: {y:.9f}, z: {z:.9f}}} "
        f"orientation: {{x: {q['x']}, y: {q['y']}, z: {q['z']:.9f}, w: {q['w']:.9f}}}"
    )
    response = run([
        "ign", "service", "-s", f"/world/{world}/set_pose",
        "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
        "--timeout", "5000", "--req", request,
    ])
    if not re.search(r"\bdata:\s*true\b", response):
        raise SystemExit(f"Gazebo did not accept the pose for {name}:\n{response}")


def read_pose(name: str, attempts: int = 3) -> tuple[float, float, float]:
    """x, y, z of a model, from the first [x y z] line after "Pose" in `ign model -p`."""
    for _ in range(attempts):
        response = run(["ign", "model", "-m", name, "-p"])
        match = re.search(r"Pose[^\n]*\n\s*\[([^\]]+)\]", response)
        if match:
            x, y, z = (float(v) for v in match.group(1).split())
            return x, y, z
        time.sleep(1.0)  # Gazebo was busy (e.g. "/gazebo/worlds timed out"); try again
    raise SystemExit(f"Could not read the pose of {name} after {attempts} tries:\n{response}")



def main():
    scene = yaml.safe_load(SCENE_FILE.read_text(encoding="utf-8"))
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
    for name, (x, y, z, yaw) in requested.items():
        set_pose(args.world, name, x, y, z, yaw)

    time.sleep(1.0)  # let the box settle

    # Verify: each model ended up within 5 mm (x/y) of where we put it.
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