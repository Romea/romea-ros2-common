//ros
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

//romea
#include "../conversions/time_conversions.hpp"
#include "../conversions/transform_conversions.hpp"

namespace romea {

template <class DataType>
class TransformPublisher
{

public :

  TransformPublisher();

  TransformPublisher(std::shared_ptr<rclcpp::Node> node,
                     const std::string & frame_id,
                     const std::string & child_frame_id);

  virtual ~TransformPublisher()=default;

public :

  virtual void init(std::shared_ptr<rclcpp::Node> node,
                    const std::string & frame_id,
                    const std::string & child_frame_id);

  virtual void publish(const rclcpp::Time & stamp,
                       const DataType & data);

  virtual void publish(const Duration & duration,
                       const DataType & data);

protected :

  geometry_msgs::msg::TransformStamped transform_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};


//-----------------------------------------------------------------------------
template <class DataType>
TransformPublisher<DataType>::TransformPublisher():
  transform_(),
  broadcaster_(nullptr)
{

}

//-----------------------------------------------------------------------------
template <class DataType>
TransformPublisher<DataType>::TransformPublisher(std::shared_ptr<rclcpp::Node> node,
                                                 const std::string & frame_id,
                                                 const std::string & child_frame_id):
  transform_(),
  broadcaster_()
{
  init(node,frame_id,child_frame_id);
}

//-----------------------------------------------------------------------------
template <class DataType>
void TransformPublisher<DataType>::init(std::shared_ptr<rclcpp::Node> node,
                                        const std::string & frame_id,
                                        const std::string & child_frame_id)
{
  assert(!frame_id.empty());
  assert(!child_frame_id.empty());

  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  transform_.header.frame_id = frame_id;
  transform_.child_frame_id = child_frame_id;

  transform_.transform.translation.x=0;
  transform_.transform.translation.y=0;
  transform_.transform.translation.z=0;
  transform_.transform.rotation.x=0;
  transform_.transform.rotation.y=0;
  transform_.transform.rotation.z=0;
  transform_.transform.rotation.z=1;

}


//-----------------------------------------------------------------------------
template <class DataType>
void TransformPublisher<DataType>::publish(const rclcpp::Time & stamp,
                                           const DataType &data)
{
  transform_.header.stamp = stamp;
  to_ros_transform_msg(data,transform_.transform);
  broadcaster_->sendTransform(transform_);
}

//-----------------------------------------------------------------------------
template <class DataType>
void TransformPublisher<DataType>::publish(const romea::Duration & duration,
                                           const DataType & data)
{
  publish(to_ros_time(duration),data);
}

}
