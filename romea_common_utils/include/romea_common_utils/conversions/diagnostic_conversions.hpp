#ifndef _romea_DiagnosticConversions_hpp_
#define _romea_DiagnosticConversions_hpp_

//romea
#include <romea_core_common/diagnostic/DiagnosticReport.hpp>

//ros
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

namespace romea
{

void to_ros_diagnostic_msg(const std::string & diagnostic_name,
                        const std::string & hardware_id,
                        const DiagnosticReport & report,
                        diagnostic_msgs::msg::DiagnosticStatus &msg);

}

#endif
