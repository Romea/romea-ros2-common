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

from typing import Optional

from romea_common_meta_bringup.utils import get_ros_distro


class ScriptParameterDescription:
    def __init__(
        self,
        name: str,
        values: Optional[list[str]] = None,
        default: Optional[str] = None,
        description: str = "",
    ):
        if not name:
            raise ValueError("Parameter description name cannot be empty.")

        self.name = name
        self.values = values
        self.default = default
        self.description = description

    def to_dict(self) -> dict:
        return {
            "values": self.values,
            "description": self.description,
        }


def parameter(
    name: str,
    description: str,
    *,
    values: list[str] | None = None,
    default: str | None = None,
) -> ScriptParameterDescription:
    return ScriptParameterDescription(
        name=name,
        values=values,
        default=default,
        description=description,
    )


def bool_parameter(
    name: str,
    description: str,
    *,
    default: str | None = None,
) -> ScriptParameterDescription:
    return parameter(
        name=name,
        values=["true", "false"],
        default=default,
        description=description,
    )


def configuration_file_path_parameter(
    device: str, name: str, kind: str
) -> ScriptParameterDescription:
    return parameter(
        name=name,
        description=f"Path to the {device} {kind} YAML file",
    )


def mode_parameter() -> ScriptParameterDescription:
    return parameter(
        name="mode",
        values=[
            "live",
            "simulation",
            "simulation_gazebo",
            "simulation_gazebo_classic",
            "simulation_gazebo_isaac",
            "simulation_4dv",
        ],
        description="Operating mode",
    )


def robot_namespace_parameter() -> ScriptParameterDescription:
    return parameter(
        name="robot_namespace",
        description="ROS namespace for the robot (without leading '/')",
    )


def meta_description_file_path_parameter(device: str) -> ScriptParameterDescription:
    return configuration_file_path_parameter(
        device=device,
        name="meta_description_file_path",
        kind="meta description",
    )


def controllers_configuration_file_path_parameter(device: str) -> ScriptParameterDescription:
    return configuration_file_path_parameter(
        device,
        "controllers_configuration_file_path",
        "controllers configuration",
    )


def extended_parameter() -> ScriptParameterDescription:
    return bool_parameter(
        name="extended",
        default="false",
        description="If true, generate the extended YAML format",
    )


def standalone_parameter() -> ScriptParameterDescription:
    return bool_parameter(
        name="standalone",
        default="true",
        description="If true, generate a standalone URDF (with a root link)",
    )


def ros_distro_parameter() -> ScriptParameterDescription:
    return parameter(
        name="ros_distro",
        default=get_ros_distro(),
        description="ROS distribution (default: ROS_DISTRO environment variable)",
    )


def generate_ros2_control_tag_parameter() -> ScriptParameterDescription:
    return bool_parameter(
        name="generate_ros2_control_tag",
        default="true",
        description="If true, include the ros2_control tag in the URDF",
    )


def generate_gazebo_tag_parameter() -> ScriptParameterDescription:
    return bool_parameter(
        name="generate_gazebo_tag",
        default="true",
        description="If true and gazebo in mode, include the Gazebo tags in the URDF",
    )
