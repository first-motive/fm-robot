// Copyright 2026 First Motive
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
//
// pose_tracking_node — MoveIt Servo PoseTracking host for vision pose-mirroring teleop.
//
// The vision teleop path needs ABSOLUTE end-effector pose servoing, not the twist jogging the
// standalone servo_node does: the operator's hand drives an absolute EE target (computed in
// fm_teleop_vision/vision_source) and the arm must mirror it and HOLD. moveit_servo::PoseTracking
// provides exactly that — it owns its own Servo instance, subscribes to "target_pose", and runs a
// PID that drives the EE to the target. This node hosts PoseTracking and keeps it servoing
// continuously (PoseTracking::moveToPose is otherwise a one-shot "go to a waypoint" call).
//
// It REPLACES servo_node_main on the vision path (it embeds its own Servo — never run both against
// the same controller). Launched by fm_bringup/launch/pose_tracking.launch.py with the same MoveIt
// context servo.launch.py builds (robot_description + semantic + kinematics + joint_limits + the
// moveit_servo params, which here also carry the PoseTracking PID gains and command_in_type).
//
// SOLE-PUBLISHER contract: the ONLY publisher of "target_pose" is fm_teleop_vision/vision_source —
// it streams the mirror target while engaged and STOPS while disengaged. This node never publishes
// a target (an earlier design did, which fought vision_source on the topic and drove the arm into
// a singularity). When no target is streaming, moveToPose simply returns NO_RECENT_TARGET_POSE,
// Servo commands nothing, and the arm holds its last pose via the joint-trajectory controller.

#include <Eigen/Core>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/status_codes.h>

#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Plain node (like servo_node_main and the pose_tracking demo): makeServoParameters, the
  // planning-scene monitor's rdf_loader, the kinematics plugin loader, and PoseTracking's
  // readROSParams each declare the params they need, picking up the launch-provided override
  // values. (Do NOT auto-declare-from-overrides — that double-declares e.g. moveit_servo.use_gazebo
  // and makeServoParameters then throws ParameterAlreadyDeclaredException.)
  auto node = rclcpp::Node::make_shared("pose_tracking_node");
  const auto logger = node->get_logger();

  // Spin the node on a worker thread so the planning scene monitor, the target_pose subscription
  // (inside PoseTracking), and Servo's internal timer are all serviced while the supervisory loop
  // below drives moveToPose on the main thread.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() {executor.spin();});

  // Servo parameters from the "moveit_servo" namespace (set by the launch from servo.yaml +
  // pose_tracking.yaml). makeServoParameters returns nullptr if a required param is missing.
  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node);
  if (servo_parameters == nullptr) {
    RCLCPP_FATAL(logger, "Failed to load Servo parameters from the 'moveit_servo' namespace.");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  // Planning scene monitor on robot_description; start the joint-state monitor so Servo +
  // PoseTracking have a live robot state. check_collisions is false (servo.yaml), so no scene/
  // world geometry monitor is required.
  auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
    node,
    "robot_description");
  if (psm->getPlanningScene() == nullptr) {
    RCLCPP_FATAL(logger, "Planning scene not configured (is robot_description set?).");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  psm->startSceneMonitor();
  psm->startStateMonitor(servo_parameters->joint_topic);

  if (!psm->waitForCurrentRobotState(node->now(), 10.0)) {
    RCLCPP_FATAL(
      logger, "Timed out waiting for a current robot state on '%s'.",
      servo_parameters->joint_topic.c_str());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  // PoseTracking constructs AND starts its embedded Servo, subscribes to "target_pose"
  // (PoseStamped, planning frame), and loads its PID gains from moveit_servo.*_proportional_gain.
  moveit_servo::PoseTracking tracker(node, servo_parameters, psm);

  // moveToPose tolerances + timeout. Tiny linear tol -> it rarely "completes" mid-motion (so the
  // EE keeps chasing a moving target). Angular tol is wide (> pi) because orientation tracking is
  // disabled (pose_tracking.yaml angular gain 0, translation-only v1): the orientation error is
  // never corrected, so it must never gate completion. timeout 0.1 s == Servo's incoming_command_
  // timeout, so a disengage (target stream stops) returns NO_RECENT_TARGET_POSE within 0.1 s.
  const Eigen::Vector3d lin_tol(0.001, 0.001, 0.001);
  const double ang_tol = 3.2;  // > pi: effectively ignore orientation (angular gain is 0)
  const double target_timeout = 0.1;

  RCLCPP_INFO(
    logger, "pose_tracking_node ready; servoing the EE to /target_pose (frame '%s').",
    servo_parameters->planning_frame.c_str());

  // Supervisory loop: keep moveToPose running forever. While vision_source streams targets the EE
  // mirrors them; when the stream stops (disengage / tracking loss) moveToPose returns
  // NO_RECENT_TARGET_POSE, Servo commands nothing, and the arm holds via the JTC — then we
  // re-enter and wait for the next target. The arm is thus always tracking or holding, never
  // faulting on a command gap.
  while (rclcpp::ok()) {
    const moveit_servo::PoseTrackingStatusCode rc = tracker.moveToPose(
      lin_tol, ang_tol,
      target_timeout);

    if (rc == moveit_servo::PoseTrackingStatusCode::STOP_REQUESTED) {
      break;
    }

    if (rc == moveit_servo::PoseTrackingStatusCode::NO_RECENT_END_EFFECTOR_POSE) {
      psm->waitForCurrentRobotState(node->now(), 1.0);
    }

    // SUCCESS (operator paused; target stays latched) or NO_RECENT_TARGET_POSE (disengaged) —
    // either way just re-enter. resetTargetPose is intentionally NOT called: a stale latched
    // target is harmless (the EE is already at it), and re-latching on re-engage comes from
    // vision_source's first zero-delta target.
    std::this_thread::sleep_for(20ms);  // avoid a busy-spin; the gap is harmless (the arm holds)
  }

  executor.cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
