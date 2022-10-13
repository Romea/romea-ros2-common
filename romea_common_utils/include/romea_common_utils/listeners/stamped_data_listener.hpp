#include <rclcpp/rclcpp.hpp>

template<typename DataType>
class DataListenerBase
{

public:

  DataListenerBase(){};

  virtual ~DataListenerBase()=default;

  const DataType & command()const
  {
    return data_;
  }

protected:

  DataType data_;
};


template <typename DataType,typename MsgType>
class DataListener : public DataListenerBase<DataType>
{
public:

  DataListener(std::shared_ptr<rclcpp::Node> node,
               const std::string & topic_name)
  {
    auto callback = std::bind(&DataListener::callback_,this,std::placeholders::_1);
    data_sub_ = node->create_subscription<MsgType>(topic_name,1,callback);
  }

private:

  virtual DataType to_cmd_(const MsgType & msg)=0;

  void callback_(typename MsgType::ConstSharedPtr msg)
  {
    std::cout << "calbback "<< std::endl;
    this->data_=to_cmd_(*msg);
  }

  std::shared_ptr<rclcpp::Subscription<MsgType>> data_sub_;
};
