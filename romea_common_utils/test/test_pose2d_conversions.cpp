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



// std
#include <string>

// gtest
#include "gtest/gtest.h"

// local
#include "test_utils.hpp"
#include "romea_common_utils/conversions/pose2d_conversions.hpp"

//-----------------------------------------------------------------------------
class TestPose2DConversion : public ::testing::Test
{
public:
  TestPose2DConversion()
  : stamp(1000),
    frame_id("foo"),
    romea_pose2d(),
    ros_pose2d_msg()
  {
  }

  void SetUp()override
  {
    romea_pose2d.position.x() = 1;
    romea_pose2d.position.y() = 2;
    romea_pose2d.yaw = 3;
    fillEigenCovariance(romea_pose2d.covariance);
    romea::to_ros_msg(stamp, frame_id, romea_pose2d, ros_pose2d_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  romea::Pose2D romea_pose2d;
  romea_common_msgs::msg::Pose2DStamped ros_pose2d_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestPose2DConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::extract_time(ros_pose2d_msg).nanoseconds(), stamp.nanoseconds());
  EXPECT_STREQ(ros_pose2d_msg.header.frame_id.c_str(), frame_id.c_str());
  EXPECT_DOUBLE_EQ(ros_pose2d_msg.pose.position.x, romea_pose2d.position.x());
  EXPECT_DOUBLE_EQ(ros_pose2d_msg.pose.position.y, romea_pose2d.position.y());
  EXPECT_DOUBLE_EQ(ros_pose2d_msg.pose.yaw, romea_pose2d.yaw);
  isSame(ros_pose2d_msg.pose.covariance, romea_pose2d.covariance);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
