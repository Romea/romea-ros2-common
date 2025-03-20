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
#include "romea_common_utils/conversions/pose3d_conversions.hpp"

//-----------------------------------------------------------------------------
class TestPose3DConversion : public ::testing::Test
{
public:
  TestPose3DConversion()
  : stamp(1000),
    frame_id("foo"),
    child_frame_id("bar"),
    romea_pose3d(),
    quaternion(),
    ros_pose3d_msg()
  {
  }

  void SetUp()override
  {
    romea_pose3d.position.x() = 1;
    romea_pose3d.position.y() = 2;
    romea_pose3d.position.z() = 3;
    romea_pose3d.orientation.x() = 0.1;
    romea_pose3d.orientation.y() = 0.2;
    romea_pose3d.orientation.z() = 0.3;
    fillEigenCovariance(romea_pose3d.covariance);
    romea::ros2::to_ros_msg(stamp, frame_id, romea_pose3d, ros_pose3d_msg);
    quaternion = romea::core::eulerAnglesToRotation3D(romea_pose3d.orientation);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  std::string child_frame_id;
  romea::core::Pose3D romea_pose3d;
  Eigen::Quaterniond quaternion;
  geometry_msgs::msg::PoseWithCovarianceStamped ros_pose3d_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestPose3DConversion, from_romea_to_ros_msg)
{
  EXPECT_EQ(romea::ros2::extract_time(ros_pose3d_msg).nanoseconds(), stamp.nanoseconds());
  EXPECT_STREQ(ros_pose3d_msg.header.frame_id.c_str(), frame_id.c_str());
  EXPECT_DOUBLE_EQ(ros_pose3d_msg.pose.pose.position.x, romea_pose3d.position.x());
  EXPECT_DOUBLE_EQ(ros_pose3d_msg.pose.pose.position.y, romea_pose3d.position.y());
  EXPECT_DOUBLE_EQ(ros_pose3d_msg.pose.pose.position.z, romea_pose3d.position.z());
  EXPECT_DOUBLE_EQ(ros_pose3d_msg.pose.pose.orientation.x, quaternion.x());
  EXPECT_DOUBLE_EQ(ros_pose3d_msg.pose.pose.orientation.y, quaternion.y());
  EXPECT_DOUBLE_EQ(ros_pose3d_msg.pose.pose.orientation.z, quaternion.z());
  EXPECT_DOUBLE_EQ(ros_pose3d_msg.pose.pose.orientation.w, quaternion.w());
  isSame(ros_pose3d_msg.pose.covariance, romea_pose3d.covariance);
}

//-----------------------------------------------------------------------------
TEST_F(TestPose3DConversion, from_ros_msg_to_romea)
{
  romea::core::Pose3D romea_pose3d_bis = romea::ros2::to_romea(ros_pose3d_msg.pose);

  EXPECT_DOUBLE_EQ(romea_pose3d_bis.position.x(), romea_pose3d.position.x());
  EXPECT_DOUBLE_EQ(romea_pose3d_bis.position.y(), romea_pose3d.position.y());
  EXPECT_DOUBLE_EQ(romea_pose3d_bis.position.z(), romea_pose3d.position.z());
  EXPECT_DOUBLE_EQ(romea_pose3d_bis.orientation.x(), romea_pose3d.orientation.x());
  EXPECT_DOUBLE_EQ(romea_pose3d_bis.orientation.y(), romea_pose3d.orientation.y());
  EXPECT_DOUBLE_EQ(romea_pose3d_bis.orientation.z(), romea_pose3d.orientation.z());
  isSame(romea_pose3d_bis.covariance, romea_pose3d.covariance);
}

//-----------------------------------------------------------------------------
TEST_F(TestPose3DConversion, from_romea_to_ros_transform_msg)
{
  geometry_msgs::msg::TransformStamped tf_msg;

  romea::ros2::to_ros_transform_msg(stamp, romea_pose3d, frame_id, child_frame_id, tf_msg);

  EXPECT_EQ(romea::ros2::extract_time(tf_msg).nanoseconds(), stamp.nanoseconds());
  EXPECT_STREQ(tf_msg.header.frame_id.c_str(), frame_id.c_str());
  EXPECT_STREQ(tf_msg.child_frame_id.c_str(), child_frame_id.c_str());
  EXPECT_DOUBLE_EQ(tf_msg.transform.translation.x, romea_pose3d.position.x());
  EXPECT_DOUBLE_EQ(tf_msg.transform.translation.y, romea_pose3d.position.y());
  EXPECT_DOUBLE_EQ(tf_msg.transform.translation.z, romea_pose3d.position.z());
  EXPECT_DOUBLE_EQ(tf_msg.transform.rotation.x, quaternion.x());
  EXPECT_DOUBLE_EQ(tf_msg.transform.rotation.y, quaternion.y());
  EXPECT_DOUBLE_EQ(tf_msg.transform.rotation.z, quaternion.z());
  EXPECT_DOUBLE_EQ(tf_msg.transform.rotation.w, quaternion.w());
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
