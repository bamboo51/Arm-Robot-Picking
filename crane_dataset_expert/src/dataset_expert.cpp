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

// fraction of the joint velocity/acceleration limits for every motion
constexpr double SPEED = 0.5;

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

bool move_joint(MoveGroupInterface& group, size_t index, double value, const char* stage){
  auto joints = group.getCurrentJointValues();
  joints[index] = value;
  group.setJointValueTarget(joints);
  return move_checked(group, stage);
}

double param(const rclcpp::Node::SharedPtr& node, const char* name){
  return node->get_parameter(name).as_double();
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

  // Scene constants from config/scene.yaml (passed in by the launch file).
  const double approach_z = param(move_group_arm_node, "approach_z");
  const double grasp_z = param(move_group_arm_node, "grasp_z");
  const double hand_offset_y = param(move_group_arm_node, "hand_offset_y");
  const double release_clearance = param(move_group_arm_node, "release_clearance");
  const double transfer_clearance = param(move_group_arm_node, "transfer_clearance");
  const double car_roof_x = param(move_group_arm_node, "car_roof_x");
  const double car_roof_y = param(move_group_arm_node, "car_roof_y");
  const double car_roof_z = param(move_group_arm_node, "car_roof_z");
  const double box_x = param(move_group_arm_node, "box_x");
  const double box_y = param(move_group_arm_node, "box_y");

  // hand sits hand_offset_y closer to the base than the box, along the radial line
  const double roof_r = std::hypot(car_roof_x, car_roof_y);
  const double target_x = car_roof_x * (roof_r - hand_offset_y) / roof_r;
  const double target_y = car_roof_y * (roof_r - hand_offset_y) / roof_r;

  // same radical as the car, with the box
  const double box_r = std::hypot(box_x, box_y);
  const double pick_x = box_x * (box_r - hand_offset_y) / box_r;
  const double pick_y = box_y * (box_r - hand_offset_y) / box_r;
  // A 4-DOF arm can only point radially, so the hand yaw is the box's polar angle
  const double pick_yaw = std::atan2(pick_y, pick_x);

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
  move_group_arm.setMaxVelocityScalingFactor(SPEED);
  move_group_arm.setMaxAccelerationScalingFactor(SPEED);

  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_arm.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End-effector link: %s", move_group_arm.getEndEffectorLink().c_str());

  MoveGroupInterface move_group_gripper(move_group_gripper_node, "gripper");
  move_group_gripper.setMaxVelocityScalingFactor(SPEED);
  move_group_gripper.setMaxAccelerationScalingFactor(SPEED);
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
  target_pose.position.x = pick_x;
  target_pose.position.y = pick_y;
  target_pose.position.z = approach_z;
  q.setRPY(to_radians(0), to_radians(90), pick_yaw);
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  if (!move_checked(move_group_arm, "Approach box")) {
    return finish(1);
  }

  q.setRPY(to_radians(0), to_radians(180), pick_yaw);
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  if (!move_checked(move_group_arm, "Orient gripper for grasp")) {
    return finish(1);
  }

  target_pose.position.z = grasp_z;
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

  // The box origin is its bottom face, which rests at MoveIt z = 0 on the table.
  // So at grasp the hand is grasp_z above the box bottom, and releasing onto the
  // car roof needs the hand at roof + grasp_z (+ clearance). Default: 0.168.
  const double release_z = car_roof_z + grasp_z + release_clearance;
  // Lift once to the minimum transfer height, rather than lift and raise again.
  target_pose = move_group_arm.getCurrentPose().pose;
  target_pose.position.z = std::max(
    target_pose.position.z, std::max(approach_z, release_z + transfer_clearance));
  if (!cartesian_checked(move_group_arm, target_pose, "Lift to transfer height")) {
    return finish(1);
  }

  // Hand above the car roof center, shifted by the grasp's hand-to-box y offset.
  // Change only x/y: maintain the measured hand height and orientation.
  geometry_msgs::msg::Pose place_pose = move_group_arm.getCurrentPose().pose;
  const double transfer_z = place_pose.position.z;
  if (transfer_z < release_z) {
    RCLCPP_ERROR(LOGGER, "Hand is below the estimated release height; stopping.");
    return finish(1);
  }
  // turn the base toward the car
  const auto hand = move_group_arm.getCurrentPose().pose;
  const double turn = std::atan2(target_y, target_x) - std::atan2(hand.position.y, hand.position.x);
  auto joints = move_group_arm.getCurrentJointValues();
  joints[0] += turn;
  move_group_arm.setJointValueTarget(joints);
  if(!move_checked(move_group_arm, "Turn toward car")){
    return finish(1);
  }

  // Slide along the hand's actual direction to the target radius. A sideways move
  // with fixed orientation is impossible for this 4-DOF arm, so stay radial.
  place_pose = move_group_arm.getCurrentPose().pose;
  const double hand_r = std::hypot(place_pose.position.x, place_pose.position.y);
  const double target_r = std::hypot(target_x, target_y);
  place_pose.position.x *= target_r / hand_r;
  place_pose.position.y *= target_r / hand_r;
  if (std::abs(target_r - hand_r) > 0.002 &&
    !cartesian_checked(move_group_arm, place_pose, "Reach out above car"))
  {
    return finish(1);
  }

  // Lower to the release position.
  // place_pose.position.z = release_z;
  // if (!cartesian_checked(move_group_arm, place_pose, "Lower onto car")) {
  //   return finish(1);
  // }

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

  // Unfold one joint at a time (wrist -> elbow -> shoulder -> base) so the hand
  // only moves up and away from the car instead of sweeping across it.
  const auto vertical = move_group_arm.getNamedTargetValues("vertical");
  const auto names = move_group_arm.getJointNames();
  const char * stages[] = {"Unfold joint 1", "Unfold joint 2", "Unfold joint 3", "Unfold joint 4"};
  for (int i = 3; i >= 0; --i) {
    if (!move_joint(move_group_arm, i, vertical.at(names[i]), stages[i])) {
      return finish(1);
    }
  }
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  if(!move_checked(move_group_gripper, "Reset gripper")){
    return finish(1);
  }
  return finish(0);
}
