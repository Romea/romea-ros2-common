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
#include "romea_common_utils/conversions/twist3d_conversions.hpp"


//-----------------------------------------------------------------------------
class TestTwist3DConversion : public ::testing::Test
{
public:
  TestTwist3DConversion()
  : stamp(1000),
    frame_id("foo"),
    romea_twist3d(),
    ros_twist3d_msg()
  {
  }

  void SetUp()override
  {
    romea_twist3d.linearSpeeds.x() = 1;
    romea_twist3d.linearSpeeds.y() = 2;
    romea_twist3d.linearSpeeds.z() = 3;
    romea_twist3d.angularSpeeds.z() = 4;
    romea_twist3d.angularSpeeds.x() = 5;
    romea_twist3d.angularSpeeds.z() = 6;
    fillEigenCovariance(romea_twist3d.covariance);
    romea::to_ros_msg(stamp, frame_id, romea_twist3d, ros_twist3d_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  romea::Twist3D romea_twist3d;
  geometry_msgs::msg::TwistWithCovarianceStamped ros_twist3d_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestTwist3DConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::extract_time(ros_twist3d_msg).nanoseconds(), stamp.nanoseconds());
  EXPECT_STREQ(ros_twist3d_msg.header.frame_id.c_str(), frame_id.c_str());
  EXPECT_DOUBLE_EQ(ros_twist3d_msg.twist.twist.linear.x, romea_twist3d.linearSpeeds.x());
  EXPECT_DOUBLE_EQ(ros_twist3d_msg.twist.twist.linear.y, romea_twist3d.linearSpeeds.y());
  EXPECT_DOUBLE_EQ(ros_twist3d_msg.twist.twist.linear.z, romea_twist3d.linearSpeeds.z());
  EXPECT_DOUBLE_EQ(ros_twist3d_msg.twist.twist.angular.x, romea_twist3d.angularSpeeds.x());
  EXPECT_DOUBLE_EQ(ros_twist3d_msg.twist.twist.angular.y, romea_twist3d.angularSpeeds.y());
  EXPECT_DOUBLE_EQ(ros_twist3d_msg.twist.twist.angular.z, romea_twist3d.angularSpeeds.z());
  isSame(ros_twist3d_msg.twist.covariance, romea_twist3d.covariance);
}

//-----------------------------------------------------------------------------
TEST_F(TestTwist3DConversion, fromRosMsgto_romea)
{
  romea::Twist3D romea_tsist3d_bis = romea::to_romea(ros_twist3d_msg.twist);
  EXPECT_DOUBLE_EQ(romea_tsist3d_bis.linearSpeeds.x(), romea_twist3d.linearSpeeds.x());
  EXPECT_DOUBLE_EQ(romea_tsist3d_bis.linearSpeeds.y(), romea_twist3d.linearSpeeds.y());
  EXPECT_DOUBLE_EQ(romea_tsist3d_bis.linearSpeeds.z(), romea_twist3d.linearSpeeds.z());
  EXPECT_DOUBLE_EQ(romea_tsist3d_bis.angularSpeeds.x(), romea_twist3d.angularSpeeds.x());
  EXPECT_DOUBLE_EQ(romea_tsist3d_bis.angularSpeeds.y(), romea_twist3d.angularSpeeds.y());
  EXPECT_DOUBLE_EQ(romea_tsist3d_bis.angularSpeeds.z(), romea_twist3d.angularSpeeds.z());
  isSame(romea_tsist3d_bis.covariance, romea_twist3d.covariance);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
