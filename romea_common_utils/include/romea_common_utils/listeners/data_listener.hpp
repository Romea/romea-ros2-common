#ifndef _romea_DataListener_hpp_
#define _romea_DataListener_hpp_

//ros
#include <rclcpp/rclcpp.hpp>

//romea
#include <romea_core_common/concurrency/SharedVariable.hpp>

namespace romea {

template<typename DataType>
class DataListenerBase
{

public:

  DataListenerBase(){};

  virtual ~DataListenerBase()=default;

  DataType get_data()const
  {

    std::lock_guard<std::mutex> lock(mutex_);
    return data_;
  }

  virtual std::string get_topic_name()const =0;

protected:

  mutable std::mutex mutex_;
  DataType data_;
};


template <typename DataType,typename MsgType>
class DataListener : public DataListenerBase<DataType>
{
public:

  DataListener(std::shared_ptr<rclcpp::Node> node,
               const std::string & topic_name,
               const size_t &queue_size)
  {
    auto callback = std::bind(&DataListener::callback_,this,std::placeholders::_1);
    data_sub_ = node->create_subscription<MsgType>(topic_name,queue_size,callback);
  }

  virtual std::string get_topic_name()const
  {
   return data_sub_->get_topic_name();
  }

  virtual ~DataListener() = default;

private:

  void callback_(typename MsgType::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    to_romea(*msg,this->data_);
  }

  std::shared_ptr<rclcpp::Subscription<MsgType>> data_sub_;
};

}

#endif
