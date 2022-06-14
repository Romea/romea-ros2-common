#include "romea_common_utils/qos.hpp"

namespace  romea {

//-----------------------------------------------------------------------------
rclcpp::QoS sensor_data_qos()
{
  return rclcpp::SensorDataQoS().reliable();
}

//-----------------------------------------------------------------------------
rclcpp::QoS best_effort(const size_t & history_size)
{
  return rclcpp::QoS(rclcpp::KeepLast(history_size))
      .best_effort().durability_volatile();
}

//-----------------------------------------------------------------------------
rclcpp::QoS reliable(const size_t & history_size)
{
  return rclcpp::QoS(rclcpp::KeepLast(history_size))
      .reliable().durability_volatile();
}

//-----------------------------------------------------------------------------
rclcpp::QoS best_effort(const size_t & history_size, const rclcpp::Duration & timeout)
{
  return best_effort(history_size).deadline(timeout);
}

//-----------------------------------------------------------------------------
rclcpp::QoS reliable(const size_t & history_size, const rclcpp::Duration & timeout)
{
  return reliable(history_size).deadline(timeout);
}


}
