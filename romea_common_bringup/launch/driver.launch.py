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

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes, PushRosNamespace, Node


def get_mode(context):
    return LaunchConfiguration("mode").perform(context)


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_driver_namespace(context):
    return LaunchConfiguration("driver_namespace").perform(context)


def get_driver_configuration_file(context):
    return LaunchConfiguration("driver_configuration_file_path").perform(context)


# def get_driver_configuration(context):
#     with open(LaunchConfiguration("driver_configuration_file_path").perform(context)) as f:
#         return yaml.safe_load(f)


def get_node_package(node_configuration):
    return node_configuration["package"]


def get_node_executable(node_configuration):
    return node_configuration.get("executable")


def get_node_plugin(node_configuration):
    return node_configuration.get("plugin")


def get_node_component_container(node_configuration):
    return node_configuration.get("component_container")


def full_name(namespace, name):
    return f"{namespace}/{name}" if namespace else name


def get_node_namespace(driver_namespace, node_configuration):
    node_namespace = node_configuration.get("namespace", "")

    if not driver_namespace:
        return node_namespace

    return full_name(driver_namespace, node_namespace)


def get_node_parameters(node_configuration):
    parameters = node_configuration.get("parameters")
    return [parameters] if parameters is not None else None


def get_node_remappings(node_configuration):
    remappings = node_configuration.get("remappings")
    return list(remappings.items()) if remappings is not None else None


def get_node_available_mode(node_configuration):
    return node_configuration.get("available_mode", "live")


def is_node_available(mode, node_name, node_configuration, configuration_file):
    executable = get_node_executable(node_configuration)
    plugin = get_node_plugin(node_configuration)
    component_container = get_node_component_container(node_configuration)

    if (executable and (plugin or component_container)) or (
        not executable and not (plugin and component_container)
    ):
        raise LookupError(
            f"Unable to launch driver called '{node_name}' because it can only have either "
            f"an executable or a plugin. Please check the configuration file {configuration_file}"
        )

    available_mode = get_node_available_mode(node_configuration)
    return available_mode in mode or available_mode == "all"


def launch_setup(context, *args, **kwargs):
    mode = get_mode(context)
    robot_namespace = get_robot_namespace(context)
    driver_namespace = get_driver_namespace(context)
    driver_configuration_file = get_driver_configuration_file(context)

    with open(driver_configuration_file) as f:
        driver_configuration = yaml.safe_load(f)

    actions = []
    actions.append(PushRosNamespace(robot_namespace))
    for node_name, node_configuration in driver_configuration.items():
        node_namespace = get_node_namespace(driver_namespace, node_configuration)
        full_node_name = full_name(robot_namespace, full_name(driver_namespace, node_name))
        if is_node_available(mode, full_node_name, node_configuration, driver_configuration_file):
            if get_node_executable(node_configuration):
                actions.append(
                    Node(
                        name=node_name,
                        namespace=node_namespace,
                        package=get_node_package(node_configuration),
                        executable=get_node_executable(node_configuration),
                        parameters=get_node_parameters(node_configuration),
                        remappings=get_node_remappings(node_configuration),
                    )
                )
            else:
                actions.append(
                    LoadComposableNodes(
                        target_container=full_name(
                            robot_namespace, get_node_component_container(node_configuration)
                        ),
                        composable_node_descriptions=[
                            ComposableNode(
                                name=node_name,
                                namespace=node_namespace,
                                package=get_node_package(node_configuration),
                                plugin=get_node_plugin(driver_configuration),
                                parameters=get_node_parameters(node_configuration),
                                remappings=get_node_remappings(node_configuration),
                            )
                        ],
                    )
                )

    return [GroupAction(actions)]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("driver_namespace"))

    declared_arguments.append(DeclareLaunchArgument("driver_configuration_file_path"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
