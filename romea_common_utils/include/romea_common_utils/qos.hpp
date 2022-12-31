#ifndef ROMEA_COMMON_UTILS_QOS_HPP_
#define ROMEA_COMMON_UTILS_QOS_HPP_

#include <rclcpp/qos.hpp>

namespace  romea
{

rclcpp::QoS sensor_data_qos();

rclcpp::QoS best_effort(const size_t & history_size);

rclcpp::QoS reliable(const size_t & history_size);

rclcpp::QoS best_effort(const size_t & history_size, const rclcpp::Duration & timeout);

rclcpp::QoS reliable(const size_t & history_size, const rclcpp::Duration & timeout);

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS_QOS_HPP_
