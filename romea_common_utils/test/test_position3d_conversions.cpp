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

// romea core
#include "romea_core_common/math/EulerAngles.hpp"

// local
#include "test_utils.hpp"
#include "romea_common_utils/conversions/position3d_conversions.hpp"

//-----------------------------------------------------------------------------
class TestPosition3DConversion : public ::testing::Test
{
public:
  TestPosition3DConversion()
  : stamp(1000),
    frame_id("foo"),
    child_frame_id("bar"),
    romea_position3d(),
    ros_position3d_msg()
  {
  }

  void SetUp()override
  {
    romea_position3d.position.x() = 1;
    romea_position3d.position.y() = 2;
    romea_position3d.position.z() = 3;
    fillEigenCovariance(romea_position3d.covariance);
    romea::ros2::to_ros_msg(stamp, frame_id, romea_position3d, ros_position3d_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  std::string child_frame_id;
  romea::core::Position3D romea_position3d;
  romea_common_msgs::msg::Position3DStamped ros_position3d_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestPosition3DConversion, from_romea_to_ros_msg)
{
  EXPECT_EQ(romea::ros2::extract_time(ros_position3d_msg).nanoseconds(), stamp.nanoseconds());
  EXPECT_STREQ(ros_position3d_msg.header.frame_id.c_str(), frame_id.c_str());
  EXPECT_DOUBLE_EQ(ros_position3d_msg.position.x, romea_position3d.position.x());
  EXPECT_DOUBLE_EQ(ros_position3d_msg.position.y, romea_position3d.position.y());
  EXPECT_DOUBLE_EQ(ros_position3d_msg.position.z, romea_position3d.position.z());
  isSame(ros_position3d_msg.position.covariance, romea_position3d.covariance);
}

//-----------------------------------------------------------------------------
TEST_F(TestPosition3DConversion, from_ros_msg_to_romea)
{
  romea::core::Position3D romea_position3d_bis = romea::ros2::to_romea(ros_position3d_msg.position);

  EXPECT_DOUBLE_EQ(romea_position3d_bis.position.x(), romea_position3d.position.x());
  EXPECT_DOUBLE_EQ(romea_position3d_bis.position.y(), romea_position3d.position.y());
  EXPECT_DOUBLE_EQ(romea_position3d_bis.position.z(), romea_position3d.position.z());
  isSame(romea_position3d_bis.covariance, romea_position3d.covariance);
}


//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
