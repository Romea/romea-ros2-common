//romea
#include "romea_common_utils/conversions/transform_conversions.hpp"
#include "romea_common_utils/conversions/geometry_conversions.hpp"

//ros
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

namespace romea {

//-----------------------------------------------------------------------------
void to_romea(const geometry_msgs::msg::Transform & tranform_msg,
             Eigen::Affine3d &eigen_transform)
{
    eigen_transform.translation() = Eigen::Vector3d(tranform_msg.translation.x,
                                                    tranform_msg.translation.y,
                                                    tranform_msg.translation.z);

    Eigen::Quaterniond q(tranform_msg.rotation.w,
                         tranform_msg.rotation.x,
                         tranform_msg.rotation.y,
                         tranform_msg.rotation.z);

    eigen_transform.linear() = q.toRotationMatrix();
}

//-----------------------------------------------------------------------------
void to_ros_transform_msg(const Eigen::Affine3d &eigen_transform,
                       geometry_msgs::msg::Transform & tranform_msg)
{
    to_ros_msg(eigen_transform.translation(), tranform_msg.translation);
    to_ros_msg(eigen_transform.linear(), tranform_msg.rotation);
}

//-----------------------------------------------------------------------------
Eigen::Affine3d lookupTransformOnce(std::shared_ptr<rclcpp::Node> node,
                                    tf2_ros::Buffer & tf_buffer,
                                    const std::string& target_frame,
                                    const std::string& source_frame,
                                    const rclcpp::Time& time,
                                    const rclcpp::Duration timeout)
{
//  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  std::cout << " coucou "<< std::endl;
  std::cout << "cache" <<tf_buffer.getCacheLength().count()<< std::endl;
  auto tranform_msg = tf_buffer.lookupTransform(target_frame,
                                                source_frame,
                                                time,
                                                timeout);

  std::cout << " coucou "<< std::endl;
  return tf2::transformToEigen(tranform_msg);
}

}  // namespce romea
