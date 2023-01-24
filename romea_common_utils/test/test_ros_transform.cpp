// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <memory>

// gtest
#include "gtest/gtest.h"

// ros
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "rclcpp/rclcpp.hpp"

// local
#include "romea_common_utils/conversions/transform_conversions.hpp"
#include "romea_common_utils/conversions/time_conversions.hpp"


class TestRosTransform : public ::testing::Test
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
    node = std::make_shared<rclcpp::Node>("test_ros_transform");
  }

  void SleedpAndSpinSome()
  {
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    rclcpp::spin_some(node);
  }

  std::shared_ptr<rclcpp::Node> node;
};


TEST_F(TestRosTransform, DISABLED_lookupTransformOnce)
{
  tf2_ros::Buffer tf_buffer(node->get_clock());
  rclcpp::sleep_for(romea::durationFromSecond(1));

  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = node->get_clock()->now();
  msg.header.frame_id = "a";
  msg.child_frame_id = "b";
  msg.transform.translation.x = 1;
  msg.transform.translation.y = 2;
  msg.transform.translation.z = 3;
  msg.transform.rotation.x = 0;
  msg.transform.rotation.y = 0;
  msg.transform.rotation.z = 0;
  msg.transform.rotation.w = 1;

  tf2_ros::TransformBroadcaster transform_broadcaster(node);
  transform_broadcaster.sendTransform(msg);
  SleedpAndSpinSome();

  Eigen::Affine3d transform;
  EXPECT_NO_THROW(
    {transform = romea::lookupTransformOnce(
        node,
        tf_buffer,
        "a", "b",
        node->get_clock()->now(),
        rclcpp::Duration::from_seconds(10));});

  EXPECT_NEAR(transform.linear()(0, 0), 1., 0.000001);
  EXPECT_NEAR(transform.linear()(1, 1), 1., 0.000001);
  EXPECT_NEAR(transform.linear()(2, 2), 1., 0.000001);

  EXPECT_NEAR(transform.translation().x(), 1., 0.000001);
  EXPECT_NEAR(transform.translation().y(), 2., 0.000001);
  EXPECT_NEAR(transform.translation().z(), 3., 0.000001);
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
