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

#ifndef FM_CONTROL__SO101_TASK_ENV_PLUGIN_HPP_
#define FM_CONTROL__SO101_TASK_ENV_PLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "fm_control/so101_task_env_markers.hpp"
#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace fm_control
{

class So101TaskEnvPlugin : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
{
public:
  So101TaskEnvPlugin() = default;
  ~So101TaskEnvPlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel * model, mjData * data) override;
  void update(const mjModel * model, mjData * data) override;
  void cleanup() override;

private:
  struct TrackedMarker
  {
    TaskEnvMarkerSpec spec;
    int body_id{-1};
  };

  void publish_markers(const mjData * data);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("So101TaskEnvPlugin")};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_raw_;
  std::unique_ptr<realtime_tools::RealtimePublisher<visualization_msgs::msg::MarkerArray>>
  marker_pub_;
  std::vector<TrackedMarker> tracked_markers_;

  std::string frame_id_{"base_link"};
  std::string topic_{"/task_env_markers"};
  std::string task_env_{"default"};
  double publish_period_seconds_{0.05};
  double last_publish_sim_time_{-1.0};
  bool enabled_{false};
};

}  // namespace fm_control

#endif  // FM_CONTROL__SO101_TASK_ENV_PLUGIN_HPP_
