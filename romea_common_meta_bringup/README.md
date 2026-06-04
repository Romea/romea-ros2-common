# romea_common_meta_bringup #

This package provides tools to build and orchestrate ROS 2 systems using a **meta-description** approach.

It offers:

* a structured way to describe devices (e.g. sensors, robots...) using YAML files
* helpers to generate and organize ROS2 python launch files
* utilities to write consistent ROS2 Python CLI scripts

It helps standardize bringup and system integration across projects.

---

## Device meta-description concept

A `device meta-description` is a YAML file used to describe a device at a high level.

It centralizes all information required to integrate a device into a robot system:

* its name and namespace
* its manufacturer, model and version
* its physical location on the robot
* its parent link, position and orientation
* its launch description (ROS2 YAML launch file)

The `MetaDescription` class loads this YAML file and provides consistent access to these fields.

It also automatically builds derived names such as:

* the full ROS namespace
* the URDF prefix
* the device link name
* the filename prefix

### Example

```yaml
name: front_sensor
namespace: sensors

configuration:
  manufacturer: foo
  model: bar
  version: "1.0"

location:
  parent_link: base_link
  xyz: [0.5, 0.0, 0.8]
  rpy: [0.0, 0.0, 0.0]

launch:
  - node:
      package: foo_driver
      executable: sensor_node
      name: driver
      output: screen
      parameters:
        - frame_id: front_sensor_link

  - include:
      file: other_package/launch/processing.launch.py
```

---

## LaunchFileGenerator

`LaunchFileGenerator` is provided by the `ros_launch` Python module of this package.

It generates a ROS 2 launch file in YAML format from the `launch` section of a meta-description.

During generation, it automatically:

- declares launch arguments, such as `mode` and `meta_description_file_path`, as well as any additional user-defined arguments  
- pushes the robot and device namespaces  
- injects resolved configuration values as `let` variables  
- groups all actions into a consistent launch structure  

This allows defining launch logic once in the meta-description and generating standardized launch files from it.

---

## ROS2 Python launch helpers

The package provides utilities (in the ros_launch module) to standardize how launch arguments are declared and accessed in ROS 2 Python launch files. The main supported arguments are:

* **mode** (execution mode)
→ allowed values: live, simulation, simulation_gazebo, simulation_isaac, simulation_4dv
* **robot_model**
→ default: None
* **robot_namespace**
→ default: None
* **meta_description_file_path**
→ default: None

These helpers provide:
* predefined defaults
* optional value restrictions (choices)

They ensure consistent argument handling across launch files and simplify reusable bringup design.

---

## ROS2 Script parameter helpers


The package also provides utilities (in the `script_parameters` module) to simplify CLI parameter handling in ROS 2 Python scripts. Parameters are defined once using helper descriptions, providing built-in parsing, default values, validation, and auto-completion.

### Supported parameters : 
* **mode** (execution mode)
→ choices: live, simulation, simulation_gazebo, simulation_isaac, simulation_4dv
→ default: None
* **robot_namespace**
→ default: None
* **meta_description_file_path**
→ default: None
* **configuration_file_path**
→ default: None
* **controllers_configuration_file_path**
→ default: None
* **ros_distro**
→ default: $ROS_DISTRO (environment variable)
* **extended**
→ default: "true"
→ choices: "true" / "false"
→ used when generating configuration files
* **standalone**
→ default: "true"
→ choices: "true" / "false"
→ used when generating URDF descriptions
* **generate_gazebo_tag**
→ default: "true"
→ choices: "true" / "false"
→ used when generating URDF descriptions
* **generate_ros2_control_tag**
→ default: "true"
→ choices: "true" / "false"
→ used when generating URDF descriptions

### CLI usage

Scripts using these helpers follow the standard ROS 2 CLI syntax:

```bash
parameter_name:=value
```

### Example
```bash
ros2 run my_pkg my_script mode:=live meta_description_file_path:=path/to/file.yaml
```

In this example:

* the script runs in live mode
* the device meta-description is loaded from the provided YAML file

All parameters are automatically handled by the helper system, so no manual parsing is required in the script.
