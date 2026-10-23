"""Load the running simulator's robot configuration into the expert."""

import subprocess

import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def without_nulls(value):
    if isinstance(value, dict):
        return {
            key: without_nulls(item)
            for key, item in value.items()
            if item is not None
        }
    return value


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
    parameters = {"use_sim_time": True}
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

    return LaunchDescription([
        # Leave name unspecified: the executable creates separate arm/gripper nodes.
        Node(
            package="crane_dataset_expert",
            executable="dataset_expert",
            output="screen",
            parameters=[parameters],
        ),
    ])
