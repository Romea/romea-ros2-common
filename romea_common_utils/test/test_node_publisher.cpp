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
#include <memory>
#include <string>

// gtest
#include "gtest/gtest.h"

// romea
#include "test_node_publisher_utils.hpp"
#include "romea_common_utils/conversions/transform_conversions.hpp"
#include "romea_common_utils/conversions/diagnostic_conversions.hpp"
#include "romea_common_utils/conversions/time_conversions.hpp"
#include "romea_common_utils/publishers/data_publisher.hpp"
#include "romea_common_utils/publishers/stamped_data_publisher.hpp"
#include "romea_common_utils/publishers/odom_publisher.hpp"
#include "romea_common_utils/publishers/transform_publisher.hpp"
#include "romea_common_utils/publishers/diagnostic_publisher.hpp"

class TestNodePublisher : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("test_ros_publishers");
  }

  void SleedpAndSpinSome()
  {
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    rclcpp::spin_some(node);
  }

  std::shared_ptr<rclcpp::Node> node;
};


TEST_F(TestNodePublisher, testDataPublisher)
{
  auto pub = romea::make_data_publisher<std::string, std_msgs::msg::String>(node, "foo", 1, true);
  Subscription<std_msgs::msg::String> sub(node, "foo");

  pub->publish("bar");
  SleedpAndSpinSome();

  EXPECT_EQ(sub.get_publisher_count(), 1U);
  EXPECT_STREQ(sub.get_received_data().data.c_str(), "bar");
}

TEST_F(TestNodePublisher, testStampedDataPublisher)
{
  auto pub = romea::make_stamped_data_publisher<Eigen::Vector3d,
      geometry_msgs::msg::PointStamped>(node, "foo", "bar", 1, true);

  Subscription<geometry_msgs::msg::PointStamped> sub(node, "foo");

  rclcpp::Time t = node->get_clock()->now();
  pub->publish(t, Eigen::Vector3d(1, 2, 3));
  SleedpAndSpinSome();

  EXPECT_EQ(sub.get_publisher_count(), 1U);
  EXPECT_EQ(romea::extract_duration(sub.get_received_data()).count(), t.nanoseconds());
  EXPECT_STREQ(sub.get_received_data().header.frame_id.c_str(), "bar");
  EXPECT_DOUBLE_EQ(sub.get_received_data().point.x, 1);
  EXPECT_DOUBLE_EQ(sub.get_received_data().point.y, 2);
  EXPECT_DOUBLE_EQ(sub.get_received_data().point.z, 3);
}

TEST_F(TestNodePublisher, testOdomPublisher)
{
  auto pub =
    romea::make_odom_publisher<nav_msgs::msg::Odometry>(node, "odom", "foo", "bar", 1, true);
  Subscription<nav_msgs::msg::Odometry> sub(node, "odom");

  rclcpp::Time t = node->get_clock()->now();
  pub->publish(t, nav_msgs::msg::Odometry());
  SleedpAndSpinSome();

  EXPECT_EQ(sub.get_publisher_count(), 1U);
  EXPECT_EQ(romea::extract_duration(sub.get_received_data()).count(), t.nanoseconds());
  EXPECT_STREQ(sub.get_received_data().header.frame_id.c_str(), "foo");
  EXPECT_STREQ(sub.get_received_data().child_frame_id.c_str(), "bar");
}

TEST_F(TestNodePublisher, testDiagnosticPublisher)
{
  auto pub = romea::make_diagnostic_publisher<romea::DiagnosticReport>(node, "foo", 1.0);
  Subscription<diagnostic_msgs::msg::DiagnosticArray> sub(node, "/diagnostics");

  romea::DiagnosticReport report;
  report.diagnostics.push_back(romea::Diagnostic(romea::DiagnosticStatus::ERROR, "bar"));
  report.info["bar"] = "error";

  rclcpp::Time t = node->get_clock()->now();
  pub->publish(t, report);
  SleedpAndSpinSome();

  EXPECT_EQ(sub.get_publisher_count(), 1U);
  EXPECT_EQ(romea::extract_duration(sub.get_received_data()).count(), t.nanoseconds());
  EXPECT_STREQ(sub.get_received_data().status[0].name.c_str(), "foo");
  EXPECT_EQ(sub.get_received_data().status[0].level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_STREQ(sub.get_received_data().status[0].values[0].key.c_str(), "bar");
  EXPECT_STREQ(sub.get_received_data().status[0].values[0].value.c_str(), "error");
}

TEST_F(TestNodePublisher, testTransformPublisher)
{
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);
  auto pub = romea::make_transform_publisher<Eigen::Affine3d>(node, "foo", "bar", true);


  rclcpp::Time t = node->get_clock()->now();
  pub->publish(t - rclcpp::Duration(std::chrono::milliseconds(100)), Eigen::Affine3d::Identity());
  SleedpAndSpinSome();
  pub->publish(t + rclcpp::Duration(std::chrono::milliseconds(100)), Eigen::Affine3d::Identity());
  SleedpAndSpinSome();

  EXPECT_TRUE(tf_buffer.canTransform("bar", "foo", t, rclcpp::Duration(1, 0)));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
