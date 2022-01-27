#include <ros/ros.h>

namespace romea
{

using RosNodeHandle = ros::NodeHandle;

inline RosNodeHandle makeChildNode(const RosNodeHandle & nh,
                                   const std::string & ns)
{
   return RosNodeHandle(nh,ns);
}

}
