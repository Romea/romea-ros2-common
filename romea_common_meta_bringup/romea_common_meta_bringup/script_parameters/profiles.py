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


from .descriptions import (
    controllers_configuration_file_path_parameter,
    extended_parameter,
    generate_gazebo_tag_parameter,
    generate_ros2_control_tag_parameter,
    meta_description_file_path_parameter,
    mode_parameter,
    robot_namespace_parameter,
    ros_distro_parameter,
    ScriptParameterDescription,
    standalone_parameter,
)


class ScriptParameterProfiles:
    @staticmethod
    def for_launch_file_generation(device_type: str) -> list[ScriptParameterDescription]:
        return [
            robot_namespace_parameter(),
            meta_description_file_path_parameter(device_type),
        ]

    @staticmethod
    def for_configuration_file_generation(device_type: str) -> list[ScriptParameterDescription]:
        return [
            meta_description_file_path_parameter(device_type),
            extended_parameter(),
        ]

    @staticmethod
    def for_controllers_configuration_file_generation(
        device_type: str,
    ) -> list[ScriptParameterDescription]:
        return [
            mode_parameter(),
            robot_namespace_parameter(),
            meta_description_file_path_parameter(device_type),
        ]

    @staticmethod
    def for_device_urdf_description_generation(
        sensor_type: str,
    ) -> list[ScriptParameterDescription]:
        return [
            mode_parameter(),
            robot_namespace_parameter(),
            meta_description_file_path_parameter(sensor_type),
            standalone_parameter(),
            ros_distro_parameter(),
        ]

    @staticmethod
    def for_robot_urdf_description_generation(
        robot_type: str,
    ) -> list[ScriptParameterDescription]:
        return [
            mode_parameter(),
            robot_namespace_parameter(),
            meta_description_file_path_parameter(robot_type),
            controllers_configuration_file_path_parameter(robot_type),
            generate_ros2_control_tag_parameter(),
            generate_gazebo_tag_parameter(),
            standalone_parameter(),
            ros_distro_parameter(),
        ]
