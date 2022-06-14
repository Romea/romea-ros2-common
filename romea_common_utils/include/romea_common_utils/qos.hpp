#ifndef _romea_QOS_hpp_
#define _romea_QOS_hpp_

#include <rclcpp/qos.hpp>

namespace  romea {


rclcpp::QoS sensor_data_qos();

rclcpp::QoS best_effort(const size_t & history_size);

rclcpp::QoS reliable(const size_t & history_size);

rclcpp::QoS best_effort(const size_t & history_size, const rclcpp::Duration & timeout);

rclcpp::QoS reliable(const size_t & history_size, const rclcpp::Duration & timeout);


}


#endif
