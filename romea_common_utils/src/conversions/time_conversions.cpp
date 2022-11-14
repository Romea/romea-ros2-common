#include "romea_common_utils/conversions/time_conversions.hpp"

namespace romea {

//-----------------------------------------------------------------------------
rclcpp::Time to_ros_time(const Duration & duration,
                         rcl_clock_type_t clock_type)
{
  return rclcpp::Time(durationToNanoSecond(duration), clock_type);
}

//-----------------------------------------------------------------------------
Duration to_romea_duration(const rclcpp::Time & time)
{
  return Duration(time.nanoseconds());
}

}  // namespace romea
