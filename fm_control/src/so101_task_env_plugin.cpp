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

#include "fm_control/so101_task_env_plugin.hpp"

#include <algorithm>
#include <utility>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/qos.hpp>

namespace fm_control
{

namespace
{
template<typename ValueT>
ValueT declare_or_get_parameter(
  const rclcpp::Node::SharedPtr & node, const std::string & name, const ValueT & default_value)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<ValueT>();
  }
  return node->declare_parameter<ValueT>(name, default_value);
}

std::string detect_task_env(const mjModel * model)
{
  if (model == nullptr) {
    return "default";
  }

  if (mj_name2id(model, mjOBJ_BODY, "pickup_cube") >= 0) {
    return "pick_place";
  }
  if (
    mj_name2id(model, mjOBJ_BODY, "cube_yellow_body") >= 0 ||
    mj_name2id(model, mjOBJ_BODY, "cube_teal_body") >= 0)
  {
    return "bin_sort";
  }
  if (mj_name2id(model, mjOBJ_BODY, "reach_target") >= 0) {
    return "table_reach";
  }

  return "default";
}
}  // namespace

bool So101TaskEnvPlugin::init(rclcpp::Node::SharedPtr node, const mjModel * model, mjData * data)
{
  node_ = std::move(node);
  logger_ = node_->get_logger();

  const std::string parameter_prefix = "mujoco_plugins.task_env_markers.";
  const auto robot = declare_or_get_parameter<std::string>(
    node_, parameter_prefix + "robot", "so101");
  const auto configured_task_env = declare_or_get_parameter<std::string>(
    node_, parameter_prefix + "task_env", "default");
  frame_id_ = declare_or_get_parameter<std::string>(
    node_, parameter_prefix + "frame_id", frame_id_);
  topic_ = declare_or_get_parameter<std::string>(
    node_, parameter_prefix + "topic", topic_);
  const double publish_rate_hz = declare_or_get_parameter<double>(
    node_, parameter_prefix + "publish_rate_hz", 20.0);
  task_env_ = robot == "so101" && configured_task_env != "default" ?
    configured_task_env : detect_task_env(model);

  RCLCPP_INFO(
    logger_,
    "SO101 task-env detection: pickup_cube=%d cube_yellow=%d cube_teal=%d reach_target=%d -> %s",
    model ? mj_name2id(model, mjOBJ_BODY, "pickup_cube") : -1,
    model ? mj_name2id(model, mjOBJ_BODY, "cube_yellow_body") : -1,
    model ? mj_name2id(model, mjOBJ_BODY, "cube_teal_body") : -1,
    model ? mj_name2id(model, mjOBJ_BODY, "reach_target") : -1,
    task_env_.c_str());

  if (publish_rate_hz <= 0.0) {
    RCLCPP_WARN(
      logger_,
      "Invalid task-env marker publish_rate_hz=%.3f; using 20 Hz",
      publish_rate_hz);
  }
  publish_period_seconds_ = publish_rate_hz > 0.0 ? 1.0 / publish_rate_hz : 0.05;

  const auto marker_specs = so101_task_env_marker_specs(task_env_);
  if (marker_specs.empty()) {
    RCLCPP_INFO(
      logger_,
      "No live task-env markers configured for task_env='%s'; plugin stays idle",
      task_env_.c_str());
    return true;
  }

  rclcpp::QoS qos(1);
  qos.reliable();
  qos.transient_local();
  marker_pub_raw_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic_, qos);
  marker_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<visualization_msgs::msg::MarkerArray>>(
    marker_pub_raw_);

  tracked_markers_.reserve(marker_specs.size());
  for (const auto & spec : marker_specs) {
    int body_id = -1;
    if (!spec.body_name.empty() && model != nullptr) {
      body_id = mj_name2id(model, mjOBJ_BODY, spec.body_name.c_str());
      if (body_id < 0) {
        RCLCPP_WARN(
          logger_,
          "MuJoCo body '%s' not found for task-env marker '%s'; using the configured fallback pose",
          spec.body_name.c_str(),
          spec.name.c_str());
      }
    }
    tracked_markers_.push_back({spec, body_id});
  }

  enabled_ = true;
  publish_markers(data);
  last_publish_sim_time_ = data ? data->time : 0.0;

  RCLCPP_INFO(
    logger_,
    "Publishing live MuJoCo task-env markers for %s on %s",
    task_env_.c_str(),
    topic_.c_str());
  return true;
}

void So101TaskEnvPlugin::update(const mjModel * /*model*/, mjData * data)
{
  if (!enabled_ || data == nullptr) {
    return;
  }

  if (
    last_publish_sim_time_ >= 0.0 &&
    (data->time - last_publish_sim_time_) < publish_period_seconds_)
  {
    return;
  }

  publish_markers(data);
  last_publish_sim_time_ = data->time;
}

void So101TaskEnvPlugin::cleanup()
{
  tracked_markers_.clear();
  marker_pub_.reset();
  marker_pub_raw_.reset();
  enabled_ = false;
}

void So101TaskEnvPlugin::publish_markers(const mjData * data)
{
  if (!enabled_ || data == nullptr || !marker_pub_ || !marker_pub_->trylock()) {
    return;
  }

  auto & msg = marker_pub_->msg_;
  msg.markers.clear();
  msg.markers.reserve(tracked_markers_.size());

  const auto stamp = rclcpp::Time(
    static_cast<rcl_time_point_value_t>(std::max(0.0, data->time) * 1e9),
    RCL_ROS_TIME);

  for (std::size_t i = 0; i < tracked_markers_.size(); ++i) {
    const auto & tracked = tracked_markers_[i];
    auto marker = marker_from_spec(tracked.spec, frame_id_, static_cast<int>(i));
    marker.header.stamp = stamp;

    if (tracked.body_id >= 0) {
      apply_mujoco_pose(
        marker,
        data->xpos + (tracked.body_id * 3),
        data->xquat + (tracked.body_id * 4));
    }

    msg.markers.push_back(std::move(marker));
  }

  marker_pub_->unlockAndPublish();
}

}  // namespace fm_control

PLUGINLIB_EXPORT_CLASS(
  fm_control::So101TaskEnvPlugin,
  mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
