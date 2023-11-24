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


// gtest
#include "gtest/gtest.h"

// local
#include "romea_common_utils/conversions/transform_conversions.hpp"
#include "romea_common_utils/conversions/geometry_conversions.hpp"
#include "romea_common_utils/conversions/diagnostic_conversions.hpp"

using GeometryMsgVector3 = geometry_msgs::msg::Vector3;
using GeometryMsgQuaternion = geometry_msgs::msg::Quaternion;
using GeometryMsgTransform = geometry_msgs::msg::Transform;
using DiagnosticMsgDiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;


//-----------------------------------------------------------------------------
TEST(TestRosConversions, testvector3Conversion)
{
  {
    Eigen::Vector3d vector3(1.2, -3.5, 10.);
    GeometryMsgVector3 vector3_msg;
    romea::ros2::to_ros_msg(vector3, vector3_msg);

    EXPECT_DOUBLE_EQ(vector3.x(), vector3_msg.x);
    EXPECT_DOUBLE_EQ(vector3.y(), vector3_msg.y);
    EXPECT_DOUBLE_EQ(vector3.z(), vector3_msg.z);
  }

  {
    GeometryMsgVector3 vector3_msg;
    vector3_msg.x = 13.3;
    vector3_msg.y = 6.8;
    vector3_msg.z = -6;

    Eigen::Vector3d vector3;
    romea::ros2::to_romea(vector3_msg, vector3);

    EXPECT_DOUBLE_EQ(vector3.x(), vector3_msg.x);
    EXPECT_DOUBLE_EQ(vector3.y(), vector3_msg.y);
    EXPECT_DOUBLE_EQ(vector3.z(), vector3_msg.z);
  }
}

//-----------------------------------------------------------------------------
TEST(TestRosConversions, testQuaternionConversion)
{
  double roll = 0.3;
  double pitch = -0.9;
  double yaw = -2.5;

  Eigen::Quaterniond quaternion =
    Eigen::AngleAxis<double>(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxis<double>(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxis<double>(roll, Eigen::Vector3d::UnitX());

  GeometryMsgQuaternion quaternion_msg;
  romea::ros2::to_ros_msg(quaternion, quaternion_msg);

  EXPECT_DOUBLE_EQ(quaternion.x(), quaternion_msg.x);
  EXPECT_DOUBLE_EQ(quaternion.y(), quaternion_msg.y);
  EXPECT_DOUBLE_EQ(quaternion.z(), quaternion_msg.z);
  EXPECT_DOUBLE_EQ(quaternion.w(), quaternion_msg.w);

  romea::ros2::to_romea(quaternion_msg, quaternion);
  EXPECT_DOUBLE_EQ(quaternion_msg.x, quaternion.x());
  EXPECT_DOUBLE_EQ(quaternion_msg.y, quaternion.y());
  EXPECT_DOUBLE_EQ(quaternion_msg.z, quaternion.z());
}

//-----------------------------------------------------------------------------
TEST(TestRosConversions, testTransformConversion)
{
  double x = -4.4;
  double y = 2.4;
  double z = 20.3;
  double roll = 0.4;
  double pitch = 1.2;
  double yaw = 3.2;

  Eigen::Quaterniond quaternion =
    Eigen::AngleAxis<double>(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxis<double>(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxis<double>(roll, Eigen::Vector3d::UnitX());

  Eigen::Matrix3d rotation = quaternion.toRotationMatrix();

  Eigen::Affine3d transform;
  transform.linear() = rotation;
  transform.translation() = Eigen::Vector3d(x, y, z);

  GeometryMsgTransform transform_msg;
  romea::ros2::to_ros_transform_msg(transform, transform_msg);
  EXPECT_DOUBLE_EQ(transform_msg.translation.x, x);
  EXPECT_DOUBLE_EQ(transform_msg.translation.y, y);
  EXPECT_DOUBLE_EQ(transform_msg.translation.z, z);
  EXPECT_DOUBLE_EQ(transform_msg.rotation.x, quaternion.x());
  EXPECT_DOUBLE_EQ(transform_msg.rotation.y, quaternion.y());
  EXPECT_DOUBLE_EQ(transform_msg.rotation.z, quaternion.z());
  EXPECT_DOUBLE_EQ(transform_msg.rotation.w, quaternion.w());

  romea::ros2::to_romea(transform_msg, transform);
  EXPECT_DOUBLE_EQ(transform.translation().x(), x);
  EXPECT_DOUBLE_EQ(transform.translation().y(), y);
  EXPECT_DOUBLE_EQ(transform.translation().z(), z);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      EXPECT_NEAR(transform.linear()(i, j), rotation(i, j), 0.00001);
    }
  }
}

//-----------------------------------------------------------------------------
TEST(TestRosConversions, testDiagnosticConversion)
{
  romea::core::DiagnosticReport report;
  report.diagnostics.push_back(
    romea::core::Diagnostic(romea::core::DiagnosticStatus::OK, "foo"));
  report.diagnostics.push_back(
    romea::core::Diagnostic(romea::core::DiagnosticStatus::ERROR, "bar"));
  report.info["foo"] = "valid";
  report.info["bar"] = "empty";

  DiagnosticMsgDiagnosticStatus status;
  romea::ros2::to_ros_diagnostic_msg("baz", "qux", report, status);

  EXPECT_STREQ(status.name.c_str(), "baz");
  EXPECT_STREQ(status.hardware_id.c_str(), "qux");
  EXPECT_EQ(status.level, DiagnosticMsgDiagnosticStatus::ERROR);
  EXPECT_STREQ(status.message.c_str(), "bar ");
  EXPECT_STREQ(status.values[0].key.c_str(), "bar");
  EXPECT_STREQ(status.values[0].value.c_str(), "empty");
  EXPECT_STREQ(status.values[1].key.c_str(), "foo");
  EXPECT_STREQ(status.values[1].value.c_str(), "valid");
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
