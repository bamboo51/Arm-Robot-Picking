"""Shared helpers for talking to the Gazebo scene."""
import json
import math
import re
import subprocess
import time
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1] 
SCENE_FILE = REPO_ROOT / "config" / "scene.yaml"
DATASET_DIR = REPO_ROOT / "datasets"

def load_scene() -> dict:
    return yaml.safe_load(SCENE_FILE.read_text(encoding="utf-8"))

def polar_to_xy(r: float, theta: float) -> tuple[float, float]:
    """convert polar to xy, theta in rad"""
    return (r * math.cos(theta), r * math.sin(theta))

def yaw_quaternion(yaw: float) -> dict:
    """Rotation about z only. yaw 0 -> {x:0, y:0, w:1}"""
    return {"x": 0, "y": 0, "z": math.sin(yaw/2), "w": math.cos(yaw/2)}

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

def place_scene(world: str, s: dict, cfg: dict) -> None:
    """Move the box and the car to a sampled scene (the `scene` dict from initial_pose.json)."""
    box, car = cfg["box"], cfg["car"]
    set_pose(world, box["name"], s["box"]["x"], s["box"]["y"], box["spawn_z"], s["box"]["yaw"])
    set_pose(world, car["name"], s["car"]["x"], s["car"]["y"], car["pose"][2], s["car"]["yaw"])