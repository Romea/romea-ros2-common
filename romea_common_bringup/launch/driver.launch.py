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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes, Node


def get_driver_namespace(context):
    return LaunchConfiguration("driver_namespace").perform(context)


def get_driver_configuration(context):
    with open(LaunchConfiguration("driver_configuration_file_path").perform(context)) as f:
        return yaml.safe_load(f)


def get_component_container(context):
    return LaunchConfiguration("component_container").perform(context)


def get_node_package(node_configuration):
    return node_configuration["package"]


def get_node_executable(node_configuration):
    return node_configuration["executable"]


def get_node_plugin(node_configuration):
    return node_configuration["plugin"]


def get_node_parameters(node_configuration):
    parameters = node_configuration.get("parameters")
    return [parameters] if parameters is not None else None


def get_node_remappings(node_configuration):
    remappings = node_configuration.get("remappings")
    return list(remappings.items()) if remappings is not None else None


def launch_setup(context, *args, **kwargs):
    driver_namespace = get_driver_namespace(context)
    driver_configuration = get_driver_configuration(context)
    component_container = get_component_container(context)

    nodes = []
    composable_nodes = []
    for node_name, node_configuration in driver_configuration.items():
        if not component_container or get_node_plugin(driver_configuration) is None:
            nodes.append(
                Node(
                    name=node_name,
                    namespace=driver_namespace,
                    package=get_node_package(node_configuration),
                    executable=get_node_executable(node_configuration),
                    parameters=get_node_parameters(node_configuration),
                    remappings=get_node_remappings(node_configuration),
                )
            )
        else:
            composable_nodes.append(
                ComposableNode(
                    name=node_name,
                    namespace=driver_namespace,
                    package=get_node_package(node_configuration),
                    plugin=get_node_plugin(driver_configuration),
                    parameters=get_node_parameters(node_configuration),
                    remappings=get_node_remappings(node_configuration),
                )
            )

    if composable_nodes:
        load_composable_nodes = (
            LoadComposableNodes(
                composable_node_descriptions=composable_nodes, target_container=component_container
            ),
        )

        return [*nodes, load_composable_nodes]
    else:
        return nodes


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("driver_namespace", default_value=""))

    declared_arguments.append(DeclareLaunchArgument("driver_configuration_file_path"))

    declared_arguments.append(DeclareLaunchArgument("component_container", default_value=""))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
