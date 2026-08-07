// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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

#include "gtest/gtest.h"
#include "romea_common_utils/conversions/fsm_conversions.hpp"

TEST(TestFSMConversions, from_romea_to_ros_state_msg)
{
  romea::core::FSMState state;
  state.name = "RUNNING";
  state.id = 1;

  const auto msg = romea::ros2::to_ros_msg(state);

  EXPECT_EQ(msg.name, "RUNNING");
  EXPECT_EQ(msg.id, 1);
}

TEST(TestFSMConversions, from_ros_to_romea_state)
{
  romea_common_msgs::msg::FSMState msg;
  msg.name = "INIT";
  msg.id = 0;

  const auto state = romea::ros2::to_romea(msg);

  EXPECT_EQ(state.name, "INIT");
  EXPECT_EQ(state.id, 0);
}

TEST(TestFSMConversions, from_romea_to_ros_event_msg)
{
  romea::core::FSMEvent event;
  event.previous_state.name = "INIT";
  event.previous_state.id = 0;
  event.current_state.name = "RUNNING";
  event.current_state.id = 1;
  event.description = "INIT DONE, GO TO RUNNING MODE";

  const auto msg = romea::ros2::to_ros_msg(rclcpp::Time(42, 0), event);

  EXPECT_EQ(msg.header.stamp.sec, 42);
  EXPECT_TRUE(msg.header.frame_id.empty());
  EXPECT_EQ(msg.previous_state.name, "INIT");
  EXPECT_EQ(msg.previous_state.id, 0);
  EXPECT_EQ(msg.current_state.name, "RUNNING");
  EXPECT_EQ(msg.current_state.id, 1);
  EXPECT_EQ(msg.description, "INIT DONE, GO TO RUNNING MODE");
}

TEST(TestFSMConversions, from_romea_to_ros_event_msg_with_frame_id)
{
  romea::core::FSMEvent event;
  event.previous_state.name = "INIT";
  event.previous_state.id = 0;
  event.current_state.name = "RUNNING";
  event.current_state.id = 1;
  event.description = "INIT DONE, GO TO RUNNING MODE";

  romea_common_msgs::msg::FSMEvent msg;
  romea::ros2::to_ros_msg(rclcpp::Time(42, 0), "source", event, msg);

  EXPECT_EQ(msg.header.stamp.sec, 42);
  EXPECT_EQ(msg.header.frame_id, "source");
  EXPECT_EQ(msg.previous_state.name, "INIT");
  EXPECT_EQ(msg.current_state.name, "RUNNING");
  EXPECT_EQ(msg.description, "INIT DONE, GO TO RUNNING MODE");
}

TEST(TestFSMConversions, from_ros_to_romea_event)
{
  romea_common_msgs::msg::FSMEvent msg;
  msg.previous_state.name = "RUNNING";
  msg.previous_state.id = 1;
  msg.current_state.name = "INIT";
  msg.current_state.id = 0;
  msg.description = "UPDATE HAS FAILED, RESET AND GO TO INIT MODE";

  const auto event = romea::ros2::to_romea(msg);

  EXPECT_EQ(event.previous_state.name, "RUNNING");
  EXPECT_EQ(event.previous_state.id, 1);
  EXPECT_EQ(event.current_state.name, "INIT");
  EXPECT_EQ(event.current_state.id, 0);
  EXPECT_EQ(event.description, "UPDATE HAS FAILED, RESET AND GO TO INIT MODE");
}
