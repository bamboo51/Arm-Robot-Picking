// Copyright 2020 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Reference:
// https://github.com/ros-planning/moveit2/blob/main/moveit_demo_nodes
// /run_move_group/src/run_move_group.cpp

#include <cmath>
#include <algorithm>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place");

double to_radians(const double deg_angle)
{
  return deg_angle * M_PI / 180.0;
}

bool move_checked(MoveGroupInterface & group, const char * stage)
{
  const auto result = group.move();

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "%s failed; stopping episode.", stage);
    return false;
  }

  RCLCPP_INFO(LOGGER, "%s completed.", stage);
  return true;
}

bool cartesian_checked(
  MoveGroupInterface & group,
  const geometry_msgs::msg::Pose & goal,
  const char * stage)
{
  const auto state = group.getCurrentState(5.0);
  if (!state) {
    RCLCPP_ERROR(LOGGER, "%s: no current robot state.", stage);
    return false;
  }
  group.clearPoseTargets();
  group.setStartState(*state);
  moveit_msgs::msg::RobotTrajectory trajectory;
  const std::vector<geometry_msgs::msg::Pose> waypoints{goal};
  const double fraction = group.computeCartesianPath(
    waypoints, 0.005, 2.0, trajectory, true);
  if (!std::isfinite(fraction) || fraction < 1.0 - 1e-6 ||
    trajectory.joint_trajectory.points.empty())
  {
    RCLCPP_ERROR(LOGGER, "%s: incomplete Cartesian path (%.1f%%); not executing.",
      stage, fraction * 100.0);
    return false;
  }

  // Apply the same 20%% speed/acceleration limits to Cartesian trajectories.
  robot_trajectory::RobotTrajectory timed(group.getRobotModel(), group.getName());
  timed.setRobotTrajectoryMsg(*state, trajectory);
  trajectory_processing::IterativeParabolicTimeParameterization timing;
  if (!timing.computeTimeStamps(timed, 0.2, 0.2)) {
    RCLCPP_ERROR(LOGGER, "%s: trajectory timing failed.", stage);
    return false;
  }
  timed.getRobotTrajectoryMsg(trajectory);
  const auto result = group.execute(trajectory);
  group.setStartStateToCurrentState();
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "%s: Cartesian execution failed.", stage);
    return false;
  }
  RCLCPP_INFO(LOGGER, "%s completed.", stage);
  return true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);

  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_gripper_node);
  std::thread executor_thread([&executor]() {executor.spin();});
  const auto finish = [&](int status) {
      executor.cancel();
      executor_thread.join();
      rclcpp::shutdown();
      return status;
    };

  MoveGroupInterface move_group_arm(move_group_arm_node, "arm");
  move_group_arm.setMaxVelocityScalingFactor(0.2);
  move_group_arm.setMaxAccelerationScalingFactor(0.2);

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_arm.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End-effector link: %s", move_group_arm.getEndEffectorLink().c_str());

  MoveGroupInterface move_group_gripper(move_group_gripper_node, "gripper");
  move_group_gripper.setMaxVelocityScalingFactor(0.2);
  move_group_gripper.setMaxAccelerationScalingFactor(0.2);
  auto gripper_joint_values = move_group_gripper.getCurrentJointValues();
  if (gripper_joint_values.empty()) {
    RCLCPP_ERROR(LOGGER, "No gripper joint state available; stopping episode.");
    return finish(1);
  }
  double GRIPPER_DEFAULT = 0.0;
  double GRIPPER_OPEN = to_radians(-30);
  double GRIPPER_CLOSE = to_radians(10);

  move_group_arm.setNamedTarget("vertical");
  if (!move_checked(move_group_arm, "Move to vertical")) {
    return finish(1);
  }

  gripper_joint_values[0] = GRIPPER_DEFAULT;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  if (!move_checked(move_group_gripper, "Initialize gripper")) {
    return finish(1);
  }

  // ----- Picking Preparation -----
  move_group_arm.setNamedTarget("home");
  if (!move_checked(move_group_arm, "Move to home")) {
    return finish(1);
  }

  gripper_joint_values[0] = GRIPPER_OPEN;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  if (!move_checked(move_group_gripper, "Open gripper")) {
    return finish(1);
  }

  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion q;
  target_pose.position.x = 0.0;
  target_pose.position.y = -0.09;
  target_pose.position.z = 0.17;
  q.setRPY(to_radians(0), to_radians(90), to_radians(-90));
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  if (!move_checked(move_group_arm, "Approach box")) {
    return finish(1);
  }

  q.setRPY(to_radians(0), to_radians(180), to_radians(-90));
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  if (!move_checked(move_group_arm, "Orient gripper for grasp")) {
    return finish(1);
  }

  target_pose.position.z = 0.14;
  move_group_arm.setPoseTarget(target_pose);
  if (!move_checked(move_group_arm, "Descend to grasp")) {
    return finish(1);
  }

  // Grasp
  gripper_joint_values[0] = GRIPPER_CLOSE;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  if (!move_checked(move_group_gripper, "Close gripper")) {
    return finish(1);
  }

  // Fixed-scene estimates: recalibrate if the car, box, or grasp changes.
  // The car: chassis: 5cm tall, center is 4cm above the car's origin -> chassis bottom = 0.04 - 0.025 = 0.015, chassis top = 0.04 + 0.0025 = 0.065
  // In gazebo, top surface = 1.015 + 0.065 = 1.080
  // In MoveIt (Movelt's zero is the robot based on the table) = 1.080 - 1.015 = 0.065
  // Resting box sitting on the roof, its center is half a cube higher: 0.065 + 0.025 = 0.090
  // Hand z = box center + 0.13 + 0.003 clearance = 0.090 + 0.13 + 0.003 = 0.223
  const double release_z = 0.223;
  const double transfer_clearance = 0.01;
  // Lift once to the minimum transfer height, rather than lift and raise again.
  target_pose = move_group_arm.getCurrentPose().pose;
  target_pose.position.z = std::max(
    target_pose.position.z, std::max(0.17, release_z + transfer_clearance));
  if (!cartesian_checked(move_group_arm, target_pose, "Lift to transfer height")) {
    return finish(1);
  }

  // Desired resting box center in MoveIt's coordinates:
  // (0.15, 0.10, 1.090 - 1.015) = (0.15, 0.10, 0.075)
  //
  // Estimated end-effector minus box-center offset at grasp:
  // (0, -0.09, 0.14) - (0, -0.10, 0.010)
  // = (0, 0.01, 0.13)
  //
  // Therefore estimated end-effector release position:
  // (0.15, 0.11, 0.205), plus 3 mm release clearance.
  // Change only x/y: maintain the measured hand height and orientation.
  geometry_msgs::msg::Pose place_pose = move_group_arm.getCurrentPose().pose;
  const double transfer_z = place_pose.position.z;
  if (transfer_z < release_z) {
    RCLCPP_ERROR(LOGGER, "Hand is below the estimated release height; stopping.");
    return finish(1);
  }
  place_pose.position.x = 0.15;
  place_pose.position.y = 0.11;
  if (!cartesian_checked(move_group_arm, place_pose, "Level transfer above car")) {
    return finish(1);
  }

  // Lower to the release position.
  place_pose.position.z = release_z;
  if (!cartesian_checked(move_group_arm, place_pose, "Lower onto car")) {
    return finish(1);
  }

  // Release.
  gripper_joint_values[0] = GRIPPER_OPEN;
  move_group_gripper.setJointValueTarget(gripper_joint_values);

  if (!move_checked(move_group_gripper, "Release box")) {
    return finish(1);
  }

  // Retreat upward.
  place_pose.position.z = transfer_z;
  if (!cartesian_checked(move_group_arm, place_pose, "Retreat from car")) {
    return finish(1);
  }
  return finish(0);
}
