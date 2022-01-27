#include "romea_common_utils/conversions/time_conversions.hpp"

namespace romea {

//-----------------------------------------------------------------------------
rclcpp::Time to_ros_time(const Duration & duration)
{
  return rclcpp::Time(durationToNanoSecond(duration));
}

//-----------------------------------------------------------------------------
Duration to_romea_duration(const rclcpp::Time & time)
{
  return Duration(time.nanoseconds());
}


}
