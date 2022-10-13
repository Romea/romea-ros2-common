#ifndef _romea_DataSerialisation_hpp_
#define _romea_DataSerialization_hpp_

#include <rclcpp/serialization.hpp>

namespace romea {

template <typename DataType>
class DataSerializationBase
{
public:

  DataSerializationBase(){};

  virtual ~DataSerializationBase()=default;

  virtual void serialize(const DataType & data,rclcpp::SerializedMessage & serialized_message) =0;

  virtual void deserialize(const rclcpp::SerializedMessage & serialized_message, DataType & data) =0;

protected:

  std::unique_ptr<rclcpp::SerializationBase> rclcpp_serialization_;
};

template <typename DataType, typename MsgType>
class DataSerialization : public DataSerializationBase<DataType>
{
public:

  DataSerialization() :
    DataSerializationBase<DataType>()
  {
    this->rclcpp_serialization_=std::make_unique<rclcpp::Serialization<MsgType>>();
  }

  virtual ~DataSerialization()=default;

  void serialize(const DataType & data,rclcpp::SerializedMessage & serialized_message)override
  {
    MsgType msg;
    to_ros_msg(data,msg);
    this->rclcpp_serialization_->serialize_message(&msg,&serialized_message);

  }


  void deserialize(const rclcpp::SerializedMessage & serialized_message, DataType & data)override
  {
    // MsgType msg;
    // this->rclcpp_serialization_->deserialize_message(&serialized_message,&msg);
    // to_romea(data,msg);
  }
};

}

#endif
