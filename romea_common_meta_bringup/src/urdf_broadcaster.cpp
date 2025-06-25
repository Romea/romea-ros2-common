// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// std
#include <memory>
#include <string>
#include <utility>

// ros2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class URDFBroadcaster : public rclcpp::Node
{
public:
  URDFBroadcaster()
  : Node("robot_description_node")
  {
    std::string urdf_xml = this->declare_parameter("robot_description", std::string(""));
    if (urdf_xml.empty()) {
      throw std::runtime_error("robot_description parameter must not be empty");
    }

    auto timeout = this->declare_parameter("timeout", 10);

    description_pub_ = this->create_publisher<std_msgs::msg::String>(
      "robot_description", rclcpp::QoS(1).transient_local().reliable());

    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = urdf_xml;
    description_pub_->publish(std::move(msg));

    shutdown_timer_ = this->create_wall_timer(
      std::chrono::seconds(timeout),
      std::bind(&URDFBroadcaster::shutdown_node, this));
  }

private:
  void shutdown_node()
  {
    rclcpp::shutdown();
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<URDFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
