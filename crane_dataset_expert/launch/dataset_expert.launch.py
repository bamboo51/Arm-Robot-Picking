"""Load the running simulator's robot configuration into the expert."""

import subprocess
import os
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from typing import Any
import json

def without_nulls(value):
    if isinstance(value, dict):
        return {
            key: without_nulls(item)
            for key, item in value.items()
            if item is not None
        }
    return value

def scene_file():
    # Override with SCENE_FILE=/path/to/scene.yaml if needed.
    if "SCENE_FILE" in os.environ:
        return Path(os.environ["SCENE_FILE"])
    return Path(__file__).resolve().parents[2]/"config"/"scene.yaml"

def generate_launch_description():
    # The camera world launch must already be running. Query its actual model
    # instead of keeping a second, potentially different URDF/SRDF in this package.
    result = subprocess.run(
        ["ros2", "param", "dump", "/move_group"],
        capture_output=True, text=True, timeout=30, check=True,
    )
    document = yaml.safe_load(result.stdout)
    if not isinstance(document, dict) or "/move_group" not in document:
        raise RuntimeError("Start the camera world and /move_group before the expert.")
    source = document["/move_group"]["ros__parameters"]
    parameters: dict[str, Any] = {"use_sim_time": True}
    for key in ("robot_description", "robot_description_semantic"):
        value = source.get(key)
        if not isinstance(value, str) or not value.strip():
            raise RuntimeError(f"/move_group has no valid {key}")
        parameters[key] = ParameterValue(value, value_type=str)

    kinematics = without_nulls(source.get("robot_description_kinematics") or {})
    # The book image's launch loads arm kinematics at the top level.
    arm = without_nulls(kinematics.get("arm") or source.get("arm") or {})
    if not arm.get("kinematics_solver"):
        raise RuntimeError("No arm kinematics solver found on /move_group.")
    kinematics["arm"] = arm
    parameters["robot_description_kinematics"] = kinematics

    scene = yaml.safe_load(scene_file().read_text(encoding="utf-8"))
    grasp = scene["grasp"]
    car = scene["car"]
    box_x, box_y = scene["box"]["xy"]
    roof_x, roof_y = car["roof_center"]

    # EPISODE=<name> uses the randomized poses frm datasets/<name>/initial_pose.json
    episode = os.environ.get("EPISODE")
    if episode:
        pose_file = scene_file().parents[1] / "datasets" / episode / "initial_pose.json"
        s = json.loads(pose_file.read_text(encoding="utf-8"))["scene"]
        box_x, box_y = s["box"]["x"], s["box"]["y"]
        roof_x, roof_y = s["car"]["roof_x"], s["car"]["roof_y"]

    parameters.update({
        "approach_z": float(grasp["approach_z"]),
        "grasp_z": float(grasp["grasp_z"]),
        "hand_offset_y": float(grasp["hand_offset_y"]),
        "release_clearance": float(grasp["release_clearance"]),
        "transfer_clearance": float(grasp["transfer_clearance"]),
        "box_x": float(box_x),
        "box_y": float(box_y),
        "car_roof_x": float(roof_x),
        "car_roof_y": float(roof_y),
        "car_roof_z": float(car["roof_z"])
    })

    return LaunchDescription([
        # Leave name unspecified: the executable creates separate arm/gripper nodes.
        Node(
            package="crane_dataset_expert",
            executable="dataset_expert",
            output="screen",
            parameters=[parameters],
        ),
    ])
