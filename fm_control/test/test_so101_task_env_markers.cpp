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
// Unit tests for the SO101 task-env marker helper logic (no ROS graph needed).
#include <gtest/gtest.h>

#include "fm_control/so101_task_env_markers.hpp"

using fm_control::apply_mujoco_pose;
using fm_control::marker_from_spec;
using fm_control::so101_task_env_marker_specs;

TEST(So101TaskEnvMarkers, PickPlaceIncludesLiveCubeBody)
{
  const auto specs = so101_task_env_marker_specs("pick_place");
  ASSERT_EQ(specs.size(), 3U);
  EXPECT_EQ(specs[1].name, "pickup_cube");
  EXPECT_EQ(specs[1].body_name, "pickup_cube");
  EXPECT_DOUBLE_EQ(specs[1].scale[0], 0.03);
}

TEST(So101TaskEnvMarkers, BinSortIncludesBothCubeBodies)
{
  const auto specs = so101_task_env_marker_specs("bin_sort");
  ASSERT_EQ(specs.size(), 5U);
  EXPECT_EQ(specs[3].body_name, "cube_yellow_body");
  EXPECT_EQ(specs[4].body_name, "cube_teal_body");
}

TEST(So101TaskEnvMarkers, MuJoCoQuaternionMapsToRosOrientation)
{
  const auto spec = so101_task_env_marker_specs("pick_place")[1];
  auto marker = marker_from_spec(spec, "base_link", 7);
  const double position[3] = {0.31, -0.04, 0.28};
  const double quaternion_wxyz[4] = {0.9, 0.1, 0.2, 0.3};

  apply_mujoco_pose(marker, position, quaternion_wxyz);

  EXPECT_DOUBLE_EQ(marker.pose.position.x, 0.31);
  EXPECT_DOUBLE_EQ(marker.pose.position.y, -0.04);
  EXPECT_DOUBLE_EQ(marker.pose.position.z, 0.28);
  EXPECT_DOUBLE_EQ(marker.pose.orientation.w, 0.9);
  EXPECT_DOUBLE_EQ(marker.pose.orientation.x, 0.1);
  EXPECT_DOUBLE_EQ(marker.pose.orientation.y, 0.2);
  EXPECT_DOUBLE_EQ(marker.pose.orientation.z, 0.3);
}
