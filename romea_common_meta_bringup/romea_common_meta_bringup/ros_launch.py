# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import ExecutableInPackage

import yaml

from .utils import complete_mode


def declare_argument(description, default_value):
    if not default_value:
        return DeclareLaunchArgument(**description)
    else:
        return DeclareLaunchArgument(**description, default_value=default_value)


def declare_robot_namespace(default_value=None):
    return declare_argument(
        {
            "name": "robot_namespace",
            "description": "ROS namespace used for the robot without the '/' prefix",
        },
        default_value
    )


def declare_robot_model(choices, default_value=None):
    return declare_argument(
        {
            "name": "robot_model",
            "description": "Model of the robot",
            "choices": choices
        },
        default_value
    )


def declare_mode(default_value=None):
    return declare_argument(
        {
            "name": "mode",
            "description": "used to select the demo context",
            "choices": [
                "live",
                "simulation",
                "simulation_gazebo",
                "simulation_gazebo_classic",
                "simulation_isaac",
                "simulation_4dv",
                "replay",
            ],
        },
        default_value
    )


def generate_robot_description(bringup_pkg, what):
    return Command(
        [
            ExecutableInPackage(f"generate_{what}_description.py", bringup_pkg),
            " robot_namespace:",
            LaunchConfiguration("robot_namespace"),
            " robot_model:",
            LaunchConfiguration("robot_model", default="none"),
            " base_name:",
            LaunchConfiguration("base_name"),
            " mode:",
            LaunchConfiguration("mode"),
        ],
        on_stderr="ignore",
    )


def generate_robot_urdf_description(bringup_pkg):
    return generate_robot_description(bringup_pkg, "urdf")


def generate_robot_ros2_control_description(bringup_pkg):
    return generate_robot_description(bringup_pkg, "ros2_control")


def declare_robot_urdf_description(default_value=None):
    return declare_argument(
        {
            "name": "robot_urdf_description",
            "description": "Content of URDF xml description in string format",
        },
        default_value
    )


def declare_robot_ros2_control_description(default_value=None):
    return declare_argument(
        {
            "name": "robot_ros2_control_description",
            "description": "Content of ROS2 control xml description in string format",
        },
        default_value
    )


def get_mode(context):
    return complete_mode(LaunchConfiguration("mode").perform(context))


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_robot_model(context):
    return LaunchConfiguration("robot_model").perform(context)


def get_robot_urdf_description(context):
    return LaunchConfiguration("robot_urdf_description").perform(context)


def get_robot_ros2_control_description(context):
    return LaunchConfiguration("robot_ros2_control_description").perform(context)


class LaunchFileGenerator:
    def __init__(self, entity_type):
        self.__entity_namespace_name = f"{entity_type}_namespace"

    def generate(self, launch_file, launch_arguments, namespaces, configuration):

        launch = []
        for argument in launch_arguments:
            launch.append(self.__generate_argument(argument))

        actions = []
        for namespace in namespaces:
            actions.append(self.__generate_push_ros_namespace(namespace))
        for name, value in self.__flatten(configuration).items():
            actions.append(self.__generate_let(name, value))
        for action in launch_file:
            actions.append(action)

        launch.append({"group": actions})

        return yaml.dump(
            {"launch": launch},
            default_flow_style=False,
            allow_unicode=True,
            encoding=None,
            sort_keys=False,
        )

    def __generate_argument(self, argument):
        return {"arg": argument}

    def __generate_push_ros_namespace(self, namespace):
        return {"push-ros-namespace": {"namespace": namespace}}

    def __generate_let(self, name, value):
        return {"let": {"name": name, "value": value}}

    def __flatten(self, device_configuration, parent_key=""):
        items = {}
        for k, v in device_configuration.items():
            new_key = f"{parent_key}.{k}" if parent_key else k
            if isinstance(v, dict):
                items.update(self.__flatten(v, new_key))
            else:
                items[new_key] = str(v)
        return items
