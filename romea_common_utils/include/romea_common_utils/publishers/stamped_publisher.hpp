#ifndef ROMEA_COMMON_UTILS_PUBLISHERS_STAMPED_PUBLISHER_HPP_
#define ROMEA_COMMON_UTILS_PUBLISHERS_STAMPED_PUBLISHER_HPP_

// std
#include <memory>
#include <string>
#include <utility>

// ros
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <romea_core_common/time/Time.hpp>

namespace romea {

template<typename DataType>
class StampedPublisherBase
{
 public:
  StampedPublisherBase() {}

  virtual ~StampedPublisherBase() = default;

  virtual std::string get_topic_name()const = 0;

  virtual void activate() = 0;

  virtual void deactivate() = 0;

  virtual bool is_activated() = 0;

  virtual void publish(const romea::Duration & stamp, const DataType & data) = 0;

  virtual void publish(const rclcpp::Time & stampe, const DataType & data) = 0;
};


template<typename DataType, typename MsgType, typename NodeType>
class StampedPublisher
{
};

template<typename DataType, typename MsgType>
class StampedPublisher<DataType, MsgType, rclcpp::Node> : public StampedPublisherBase<DataType>
{
 public:
  using Options = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;

 public:
  StampedPublisher(std::shared_ptr<rclcpp::Node> node,
                   const std::string & topic_name,
                   const rclcpp::QoS & qos,
                   const Options & options,
                   const bool & activated):
    is_activated_(activated),
    pub_(node->create_publisher<MsgType>(topic_name, qos, options)),
    should_log_(true),
    logger_(rclcpp::get_logger("Publisher"))
  {
  }

  virtual ~StampedPublisher() = default;

  std::string get_topic_name() const override
  {
    return pub_->get_topic_name();
  }

  void activate() override
  {
    is_activated_.store(true);
  }

  void deactivate() override
  {
    is_activated_.store(false);
  }

  bool is_activated() override
  {
    return is_activated_.load();
  }

 protected:
  void publish_message_(std::unique_ptr<MsgType> message)
  {
    if (!this->is_activated()) {
      log_publisher_not_enabled();
      return;
    }
    pub_->publish(std::move(message));
  }

  void publish_message_(const MsgType & message)
  {
    if (!this->is_activated()) {
      log_publisher_not_enabled();
      return;
    }

    pub_->publish(message);
  }

  void log_publisher_not_enabled()
  {
    // Nothing to do if we are not meant to log
    if (!should_log_) {
      return;
    }

    // Log the message
    RCLCPP_WARN(
          logger_,
          "Trying to publish message on the topic '%s', but the publisher is not activated",
          this->get_topic_name().c_str());

    // We stop logging until the flag gets enabled again
    should_log_ = false;
  }

 protected:
  std::atomic<bool> is_activated_;
  std::shared_ptr<rclcpp::Publisher<MsgType>> pub_;

  bool should_log_;
  rclcpp::Logger logger_;
};


template<typename DataType, typename MsgType>
class StampedPublisher<DataType, MsgType, rclcpp_lifecycle::LifecycleNode> : public StampedPublisherBase<DataType>
{
 public:
  using Options = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;

 public:
  StampedPublisher(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
                   const std::string & topic_name,
                   const rclcpp::QoS & qos,
                   const Options & options,
                   const bool & activated):
    pub_(node->create_publisher<MsgType>(topic_name, qos, options))
  {
    if (activated)
    {
      pub_->on_activate();
    }
  }

  virtual ~StampedPublisher() = default;


  std::string get_topic_name()const
  {
    return pub_->get_topic_name();
  }

  void activate() override
  {
    pub_->on_activate();
  }

  void deactivate() override
  {
    pub_->on_deactivate();
  }

  bool is_activated() override
  {
    return pub_->is_activated();
  }

protected:
  void publish_message_(std::unique_ptr<MsgType> message)
  {
    pub_->publish(std::move(message));
  }

  void publish_message_(const MsgType & message)
  {
    pub_->publish(message);
  }

protected :

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MsgType>> pub_;
};

}  // namespace romea

#endif  // ROMEA_COMMON_UTILS_PUBLISHERS_STAMPED_PUBLISHER_HPP_
