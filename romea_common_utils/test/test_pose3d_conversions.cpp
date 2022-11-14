// gtest
#include <gtest/gtest.h>
#include "test_utils.hpp"

//romea
#include <romea_core_common/math/EulerAngles.hpp>
#include "romea_common_utils/conversions/pose3d_conversions.hpp"

//-----------------------------------------------------------------------------
class TestPose3DConversion : public ::testing::Test
{
public :

  TestPose3DConversion():
    stamp(1000),
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
    romea::to_ros_msg(stamp, frame_id, romea_pose3d, ros_pose3d_msg);
    quaternion = romea::eulerAnglesToRotation3D(romea_pose3d.orientation);
  }

  rclcpp::Time stamp;
  std::string frame_id;
  std::string child_frame_id;
  romea::Pose3D romea_pose3d;
  Eigen::Quaterniond quaternion;
  geometry_msgs::msg::PoseWithCovarianceStamped ros_pose3d_msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestPose3DConversion, fromRomeato_ros_msg)
{
  EXPECT_EQ(romea::extract_time(ros_pose3d_msg).nanoseconds(), stamp.nanoseconds());
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
TEST_F(TestPose3DConversion, fromRosMsgto_romea)
{
  romea::Pose3D romea_pose3d_bis = romea::to_romea(ros_pose3d_msg.pose);

  EXPECT_DOUBLE_EQ(romea_pose3d_bis.position.x(), romea_pose3d.position.x());
  EXPECT_DOUBLE_EQ(romea_pose3d_bis.position.y(), romea_pose3d.position.y());
  EXPECT_DOUBLE_EQ(romea_pose3d_bis.position.z(), romea_pose3d.position.z());
  EXPECT_DOUBLE_EQ(romea_pose3d_bis.orientation.x(), romea_pose3d.orientation.x());
  EXPECT_DOUBLE_EQ(romea_pose3d_bis.orientation.y(), romea_pose3d.orientation.y());
  EXPECT_DOUBLE_EQ(romea_pose3d_bis.orientation.z(), romea_pose3d.orientation.z());
  isSame(romea_pose3d_bis.covariance, romea_pose3d.covariance);
}

//-----------------------------------------------------------------------------
TEST_F(TestPose3DConversion, fromRomeato_ros_transform_msg)
{
  geometry_msgs::msg::TransformStamped tf_msg;

  romea::to_ros_transform_msg(stamp, romea_pose3d, frame_id, child_frame_id, tf_msg);

  EXPECT_EQ(romea::extract_time(tf_msg).nanoseconds(), stamp.nanoseconds());
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
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
