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

#ifndef FM_CONTROL__SO101_TASK_ENV_MARKERS_HPP_
#define FM_CONTROL__SO101_TASK_ENV_MARKERS_HPP_

#include <array>
#include <string>
#include <vector>

#include <rclcpp/duration.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace fm_control
{

struct MarkerColor
{
  double r;
  double g;
  double b;
  double a{1.0};
};

struct TaskEnvMarkerSpec
{
  std::string name;
  int marker_type;
  std::array<double, 3> position;
  std::array<double, 4> orientation_xyzw{0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> scale;
  MarkerColor color;
  std::string body_name;
};

inline std::vector<TaskEnvMarkerSpec> so101_task_env_marker_specs(const std::string & task_env)
{
  using visualization_msgs::msg::Marker;

  if (task_env == "table_reach") {
    return {
      {"table_top", Marker::CUBE, {0.22, 0.0, 0.19}, {0.0, 0.0, 0.0, 1.0}, {0.36, 0.48, 0.04},
        {0.62, 0.52, 0.42, 1.0}, "table"},
      {"reach_target", Marker::SPHERE, {0.25, 0.0, 0.255}, {0.0, 0.0, 0.0, 1.0},
        {0.036, 0.036, 0.036},
        {0.1, 0.8, 0.3, 1.0}, ""}
    };
  }

  if (task_env == "pick_place") {
    return {
      {"table_top", Marker::CUBE, {0.22, 0.0, 0.19}, {0.0, 0.0, 0.0, 1.0}, {0.36, 0.48, 0.04},
        {0.62, 0.52, 0.42, 1.0}, "table"},
      {"pickup_cube", Marker::CUBE, {0.22, -0.07, 0.225}, {0.0, 0.0, 0.0, 1.0}, {0.03, 0.03, 0.03},
        {0.88, 0.2, 0.2, 1.0}, "pickup_cube"},
      {"goal_pad", Marker::CYLINDER, {0.28, 0.09, 0.205}, {0.0, 0.0, 0.0, 1.0}, {0.06, 0.06, 0.006},
        {0.2, 0.45, 0.95, 1.0}, "goal_pad"}
    };
  }

  if (task_env == "bin_sort") {
    return {
      {"table_top", Marker::CUBE, {0.22, 0.0, 0.19}, {0.0, 0.0, 0.0, 1.0}, {0.36, 0.48, 0.04},
        {0.62, 0.52, 0.42, 1.0}, "table"},
      {"left_bin_base", Marker::CUBE, {0.27, -0.10, 0.215}, {0.0, 0.0, 0.0, 1.0},
        {0.09, 0.09, 0.02},
        {0.82, 0.25, 0.25, 0.45}, "left_bin"},
      {"right_bin_base", Marker::CUBE, {0.27, 0.10, 0.215}, {0.0, 0.0, 0.0, 1.0},
        {0.09, 0.09, 0.02},
        {0.22, 0.45, 0.88, 0.45}, "right_bin"},
      {"cube_yellow", Marker::CUBE, {0.19, -0.03, 0.225}, {0.0, 0.0, 0.0, 1.0},
        {0.028, 0.028, 0.028},
        {0.95, 0.82, 0.12, 1.0}, "cube_yellow_body"},
      {"cube_teal", Marker::CUBE, {0.19, 0.05, 0.225}, {0.0, 0.0, 0.0, 1.0}, {0.028, 0.028, 0.028},
        {0.12, 0.78, 0.78, 1.0}, "cube_teal_body"}
    };
  }

  return {};
}

inline visualization_msgs::msg::Marker marker_from_spec(
  const TaskEnvMarkerSpec & spec, const std::string & frame_id, int marker_id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = "task_env";
  marker.id = marker_id;
  marker.type = spec.marker_type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = spec.position[0];
  marker.pose.position.y = spec.position[1];
  marker.pose.position.z = spec.position[2];
  marker.pose.orientation.x = spec.orientation_xyzw[0];
  marker.pose.orientation.y = spec.orientation_xyzw[1];
  marker.pose.orientation.z = spec.orientation_xyzw[2];
  marker.pose.orientation.w = spec.orientation_xyzw[3];
  marker.scale.x = spec.scale[0];
  marker.scale.y = spec.scale[1];
  marker.scale.z = spec.scale[2];
  marker.color.r = spec.color.r;
  marker.color.g = spec.color.g;
  marker.color.b = spec.color.b;
  marker.color.a = spec.color.a;
  marker.lifetime.sec = 0;
  marker.lifetime.nanosec = 0;
  return marker;
}

template<typename PositionPtr, typename QuaternionPtr>
inline void apply_mujoco_pose(
  visualization_msgs::msg::Marker & marker, PositionPtr position, QuaternionPtr quaternion_wxyz)
{
  marker.pose.position.x = position[0];
  marker.pose.position.y = position[1];
  marker.pose.position.z = position[2];
  marker.pose.orientation.x = quaternion_wxyz[1];
  marker.pose.orientation.y = quaternion_wxyz[2];
  marker.pose.orientation.z = quaternion_wxyz[3];
  marker.pose.orientation.w = quaternion_wxyz[0];
}

}  // namespace fm_control

#endif  // FM_CONTROL__SO101_TASK_ENV_MARKERS_HPP_
