# romea_ros2_common utils

This package provides a library designed to simplify ROS2 code development for developers working within the ROMEA ecosystem

## Utilities provided by the package

### Conversions

The package provides conversion helpers between ROMEA data structures and ROS2 messages:

- diagnostic conversions
- geometry conversions
- position 2D / 3D conversions
- pose 2D / 3D conversions
- twist 2D / 3D conversions
- pose and twist 2D / 3D conversions
- finite-state-machine conversions
- transform conversions
- time conversions

Typical functions:

- `to_ros_msg(...)`
- `to_romea(...)`
- `to_ros_transform_msg(...)`
- `to_ros_odom_msg(...)`
- `to_ros_time(...)`
- `to_romea_time(...)`

---

### Listeners

The package provides generic listener helpers to receive ROS2 messages and convert them into internal ROMEA data structures.

Available listeners:

- `DataListener`
- `StampedDataListener`

These helpers simplify subscription handling and data conversion.

---

### Publishers

The package provides generic publisher helpers to publish ROS2 messages from internal ROMEA data structures.

Available publishers:

- `DataPublisher`
- `StampedDataPublisher`
- `DiagnosticPublisher`
- `OdomPublisher`
- `TransformPublisher`

It also provides base publisher classes used to factorize common publishing behavior.

`StampedDataPublisher` can be used with an empty `frame_id` for stamped messages that carry time but are not attached to a spatial frame, such as `FSMEvent`.

---

### Logging

The package provides lightweight ROS2 logging helpers. `TopicLogger` publishes `romea_common_msgs/msg/DataLogger` debug data, commonly on `debug/<logger_name>` topics when the owning component enables debug logging. `ConsoleFSMLogger` prints finite-state-machine transitions consistently.

---

### Realtime publishers

The package provides realtime-compatible publisher helpers:

- `RealtimeMessagePublisher`
- `RealtimeStampedMessagePublisher`

These are useful when publishing data from control loops or realtime-sensitive components.

---

### Parameters

The package provides helpers to declare and retrieve ROS2 parameters consistently in Node.

Available parameter helpers include:

- node parameters
- algorithm parameters
- sensor parameters
- control parameters
- Eigen parameters
- geodesy parameters

Typical helpers:

- `declare_parameter(...)`
- `declare_parameter_with_default(...)`
- `get_parameter(...)`
- `get_parameter_or(...)`
- `declare_vector_parameter(...)`
- `get_vector_parameter(...)`

Specific helpers are also available for common parameters such as:

- `debug`
- `frame_id`
- `publish_rate`
- `base_footprint_frame_id`
- `odom_frame_id`
- `map_frame_id`

---

### Other utilities

The package also provides additional helpers for:

- QoS profiles
- ROS distro version handling
- parse and fill joint state messages
