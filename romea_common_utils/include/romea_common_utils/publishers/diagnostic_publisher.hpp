#ifndef _romea_DiagnosticPublisher_hpp_
#define _romea_DiagnosticPublisher_hpp_

//ros
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

//romea
#include "../conversions/time_conversions.hpp"
#include "../conversions/diagnostic_conversions.hpp"

namespace romea
{

template <class DataType>
class DiagnosticPublisher
{

public :

    DiagnosticPublisher();

    DiagnosticPublisher(std::shared_ptr<rclcpp::Node> node,
                        const std::string & diagnostic_name,
                        const double & diagnostic_period,
                        const std::string & hardware_id_="",
                        const std::string & topic_name="/diagnostics");

public :

    void init(std::shared_ptr<rclcpp::Node> node,
              const std::string & diagnostic_name,
              const double & diagnostic_period,
              const std::string & hardware_id_="",
              const std::string & topic_name="/diagnostics");

    void publish(const Duration & duration,
                 const DataType &data);

    void publish(const rclcpp::Time & stamp,
                 const DataType &data);

private :

    void publish_(const rclcpp::Time & stamp,
                  const DataType &data);

private :

    std::string diagnostic_name_;
    std::string hardware_id_;
    std::shared_ptr<rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>> pub_;
    rclcpp::Time next_time_;
    rclcpp::Duration diagnostic_period_;
};

//-----------------------------------------------------------------------------
template <class DataType>
DiagnosticPublisher<DataType>::DiagnosticPublisher():
    diagnostic_name_(),
    hardware_id_(),
    pub_(),
    next_time_(),
    diagnostic_period_(0)
{

}

//-----------------------------------------------------------------------------
template <class DataType>
DiagnosticPublisher<DataType>::DiagnosticPublisher(std::shared_ptr<rclcpp::Node> node,
                                                   const std::string & diagnostic_name,
                                                   const double & diagnostic_period,
                                                   const std::string & hardware_id,
                                                   const std::string & topic_name):
    diagnostic_name_(),
    hardware_id_(),
    pub_(nullptr),
    next_time_(node->get_clock()->now()),
    diagnostic_period_(0)
{
    init(node,diagnostic_name,diagnostic_period,hardware_id,topic_name);
}

//-----------------------------------------------------------------------------
template <class DataType>
void DiagnosticPublisher<DataType>::init(std::shared_ptr<rclcpp::Node> node,
                                         const std::string & diagnostic_name,
                                         const double & diagnostic_period,
                                         const std::string & hardware_id,
                                         const std::string & topic_name)
{
    diagnostic_name_ = diagnostic_name;
    hardware_id_ = hardware_id;
    diagnostic_period_=rclcpp::Duration(romea::durationFromSecond(diagnostic_period));
    pub_= node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(topic_name,1);
}

//-----------------------------------------------------------------------------
template <class DataType>
void DiagnosticPublisher<DataType>::publish(const Duration & duration,
                                            const DataType &data)
{
    publish(to_ros_time(duration),data);
}

//-----------------------------------------------------------------------------
template <class DataType>
void DiagnosticPublisher<DataType>::publish(const rclcpp::Time & stamp,
                                            const DataType & data)
{
    if(stamp>next_time_)
    {
        publish_(stamp,data);
        next_time_ = stamp+ diagnostic_period_;
    }
}

//-----------------------------------------------------------------------------
template <class DataType>
void DiagnosticPublisher<DataType>::publish_(const rclcpp::Time & stamp,
                                             const DataType & data)
{
    auto msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
    msg->header.stamp=stamp;
    msg->status.push_back(diagnostic_msgs::msg::DiagnosticStatus());
    to_ros_diagnostic_msg(diagnostic_name_,hardware_id_,data,msg->status.back());
    pub_->publish(std::move(msg));
}

}


#endif
