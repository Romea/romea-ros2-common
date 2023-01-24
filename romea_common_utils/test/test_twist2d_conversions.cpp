// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license


// std
#include <string>

// gtest
#include "gtest/gtest.h"

// local
#include "test_utils.hpp"
#include "romea_common_utils/conversions/twist2d_conversions.hpp"

//-----------------------------------------------------------------------------
class TestTwist2DConversion : public ::testing::Test
{
public:
  TestTwist2DConversion()
  : stamp(1000),
    frame_id("foo"),
    romea_twist2d(),
    ros_twist2d_msg()
  {
  }

  void SetUp()override
  {
    romea_twist2d.linearSpeeds.x() = 1;
    romea_twist2d.linearSpeeds.y() = 2;
    romea_twist2d.angularSpeed = 3;
    fillEigenCovariance(romea_twist2d.covariance);
    romea::to_ros_msg(stamp, frame_id, romea_twist2d, ros_twist2d_msg);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  romea::Twist2D romea_twist2d;
  romea_common_msgs::msg::Twist2DStamped ros_twist2d_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestTwist2DConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::extract_time(ros_twist2d_msg).nanoseconds(), stamp.nanoseconds());
  EXPECT_STREQ(ros_twist2d_msg.header.frame_id.c_str(), frame_id.c_str());
  EXPECT_DOUBLE_EQ(ros_twist2d_msg.twist.linear_speeds.x, romea_twist2d.linearSpeeds.x());
  EXPECT_DOUBLE_EQ(ros_twist2d_msg.twist.linear_speeds.y, romea_twist2d.linearSpeeds.y());
  EXPECT_DOUBLE_EQ(ros_twist2d_msg.twist.angular_speed, romea_twist2d.angularSpeed);
  isSame(ros_twist2d_msg.twist.covariance, romea_twist2d.covariance);
}

//-----------------------------------------------------------------------------
TEST_F(TestTwist2DConversion, fromRosMsgto_romea)
{
  romea::Twist2D romea_tsist2d_bis = romea::to_romea(ros_twist2d_msg.twist);

  EXPECT_DOUBLE_EQ(romea_tsist2d_bis.linearSpeeds.x(), romea_twist2d.linearSpeeds.x());
  EXPECT_DOUBLE_EQ(romea_tsist2d_bis.linearSpeeds.y(), romea_twist2d.linearSpeeds.y());
  EXPECT_DOUBLE_EQ(romea_tsist2d_bis.angularSpeed, romea_twist2d.angularSpeed);
  isSame(romea_tsist2d_bis.covariance, romea_twist2d.covariance);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
