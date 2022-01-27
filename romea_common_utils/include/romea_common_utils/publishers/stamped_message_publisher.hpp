//ros
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include "../conversions/time_conversions.hpp"

namespace romea {

template <class DataType, class MessageType>
class StampedMessagePublisher
{

public :

    StampedMessagePublisher();

    StampedMessagePublisher(std::shared_ptr<rclcpp::Node> node,
                            const std::string & topic_name,
                            const std::string & frame_id,
                            const size_t &queue_size);
public :

    void init(std::shared_ptr<rclcpp::Node> node,
              const std::string & topic_name,
              const std::string & frame_id,
              const size_t &queue_size);

    void publish(const rclcpp::Time & stamp,
                 const DataType & data);


    void publish(const romea::Duration & duration,
                 const DataType & data);

protected :

    std::string frame_id_;
    std::shared_ptr<rclcpp::Publisher<MessageType>> pub_;
};


//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
StampedMessagePublisher<DataType,MessageType>::StampedMessagePublisher():
    frame_id_(),
    pub_(nullptr)
{

}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
StampedMessagePublisher<DataType,MessageType>::StampedMessagePublisher(std::shared_ptr<rclcpp::Node> node,
                                                                       const std::string & topic_name,
                                                                       const std::string & frame_id,
                                                                       const size_t &queue_size):
    frame_id_(),
    pub_(nullptr)
{
    init(node,topic_name,frame_id,queue_size);
}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void StampedMessagePublisher<DataType,MessageType>::init(std::shared_ptr<rclcpp::Node> node,
                                                         const std::string & topic_name,
                                                         const std::string & frame_id,
                                                         const size_t &queue_size)
{
    assert(!frame_id.empty());
    frame_id_=frame_id;
    pub_=node->create_publisher<MessageType>(topic_name,queue_size);
}


//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void StampedMessagePublisher<DataType,MessageType>::publish(const rclcpp::Time & stamp,
                                                            const DataType &data)
{
    auto msg = std::make_unique<MessageType>();
    to_ros_msg(stamp,frame_id_,data,*msg.get());
    pub_->publish(std::move(msg));
}

//-----------------------------------------------------------------------------
template <class DataType, class MessageType>
void StampedMessagePublisher<DataType,MessageType>::publish(const romea::Duration & duration,
                                                            const DataType & data)
{
    publish(to_ros_time(duration),data);
}

}
