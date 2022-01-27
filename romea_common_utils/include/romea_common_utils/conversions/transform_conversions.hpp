#ifndef _romea_TransformConversions_hpp_
#define _romea_TransformConversions_hpp_

//ros
#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/transform.hpp>

//eigen
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
namespace romea {



void to_romea(const geometry_msgs::msg::Transform & tranform_msg,
              Eigen::Affine3d &eigen_transform);

void to_ros_transform_msg(const Eigen::Affine3d &eigen_transform,
                          geometry_msgs::msg::Transform & tranform_msg);

Eigen::Affine3d lookupTransformOnce(std::shared_ptr<rclcpp::Node> node,
                                    tf2_ros::Buffer &tf_buffer,
                                    const std::string & target_frame,
                                    const std::string & source_frame,
                                    const rclcpp::Time & time,
                                    const rclcpp::Duration timeout);


}

#endif
