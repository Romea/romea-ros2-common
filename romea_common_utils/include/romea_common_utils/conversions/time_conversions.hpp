#ifndef _romea_TimeConversion_hpp_
#define _romea_TimeConversion_hpp_

//ros
#include <rclcpp/time.hpp>
#include <message_filters/message_traits.h>

//romea
#include <romea_core_common/time/Time.hpp>

namespace romea {


rclcpp::Time to_ros_time(const romea::Duration & duration);

romea::Duration to_romea_duration(const rclcpp::Time & time);

template <typename Msg>
rclcpp::Time extract_time(const Msg & msg)
{
   return message_filters::message_traits::TimeStamp<Msg>::value(msg);
}

template <typename Msg>
Duration extract_duration(const Msg & msg)
{
    return to_romea_duration(extract_time(msg));
}


}

#endif
