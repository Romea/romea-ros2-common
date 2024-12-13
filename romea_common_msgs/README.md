# romea_ros2_common_msgs

This package provides **ROS2 message definitions** .

This package provides a set of ROS2 message definitions similar to some messages offered by the `geometry_msgs` and `nav_msgs` packages but specifically adapted for 2D applications. The provided messages include:

- **Twist2D**: Represents 2D velocity information with its covariance.
- **Position2D**: Encodes 2D position data with its covariance.
- **Pose2D**: Combines position and orientation in 2D space with its covariance.
- **PoseAndTwist2D**: A 2D equivalent to `nav_msgs::Odometry`, combining pose and twist information.

These messages are primarily designed for and used by localization algorithms developed within the ROMEA ecosystem 

