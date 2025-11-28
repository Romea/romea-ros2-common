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

import xml.etree.ElementTree as ET

from ament_index_python import get_package_share_directory, get_packages_with_prefixes


def robot_namespace(robot_name):
    if robot_name == "":
        return "/"
    else:
        return "/" + robot_name


def robot_prefix(robot_name):

    if robot_name == "":
        return "/"
    else:
        return "/" + robot_name + "/"


def device_namespace(robot_name, ros_namespace, device_name):
    if ros_namespace is not None:
        return robot_prefix(robot_name) + ros_namespace + "/" + device_name
    else:
        return robot_prefix(robot_name) + device_name


def device_prefix(robot_name, ros_namespace, device_name):
    if ros_namespace is not None:
        return robot_prefix(robot_name) + ros_namespace + "/" + device_name + "/"
    else:
        return robot_prefix(robot_name) + device_name + "/"


def robot_urdf_prefix(robot_name):

    if robot_name == "":
        return ""
    else:
        return robot_name + "_"


def device_urdf_prefix(robot_name, device_name):
    if device_name == "":
        return robot_urdf_prefix(robot_name)
    else:
        return robot_urdf_prefix(robot_name) + device_name + "_"


def device_link_name(robot_name, device_name):
    return robot_urdf_prefix(robot_name) + device_name + "_link"


def complete_mode(mode):
    if mode == "simulation":
        pkgs = get_packages_with_prefixes()
        if "gazebo_ros" in pkgs:
            mode += "_gazebo_classic"
        elif "ros_gz" in pkgs:
            mode += "_gazebo"
        else:
            raise ValueError("No Gazebo package found. Please install 'gazebo_ros' or 'ros_gz'.")
    return mode


def complete_robot_description(robot_urdf_description, additionnal_descriptions):

    full_description = ET.fromstring(robot_urdf_description)
    for additional_description in additionnal_descriptions:
        full_description.extend(ET.fromstring(additional_description))

    return ET.tostring(full_description, encoding="unicode")


# a oter
def get_file_path(file_configuration):
    pkg = file_configuration["pkg"]
    file = file_configuration["file"]
    return get_package_share_directory(pkg) + "/" + file
