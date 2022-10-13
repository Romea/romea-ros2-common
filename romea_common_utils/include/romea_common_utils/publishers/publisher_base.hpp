//#ifndef _romea_PublisherBase_hpp_
//#define _romea_PublisherBase_hpp_

//#include <rclcpp/node.hpp>
//#include <rclcpp/publisher.hpp>
//#include <rclcpp_lifecycle/lifecycle_node.hpp>
//#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

//namespace romea {

//template<typename DataType, typename NodeType>
//class PublisherBase
//{
//};

//template<typename DataType>
//class PublisherBase<DataType,rclcpp::Node>
//{
//public:
//  PublisherBase(){}

//  virtual ~PublisherBase()=default;

//  virtual std::string get_topic_name()const =0;
//};

//template<>
//class PublisherBase<rclcpp_lifecycle::LifecycleNode>
//{
//public:

//  PublisherBase(){}

//  virtual ~PublisherBase()=default;

//  virtual std::string get_topic_name()const =0;

//  virtual void on_activate()=0;

//  virtual void on_deactivate() =0;

//};

//using NodePublisherBase = PublisherBase<rclcpp::Node>;
//using LifecycleNodePublisherBase = PublisherBase<rclcpp_lifecycle::LifecycleNode>;

//}

//#endif
