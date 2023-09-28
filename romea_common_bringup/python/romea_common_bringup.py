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


# !/usr/bin/env python3
from ament_index_python import get_package_share_directory
import importlib
import yaml


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


# def temporary_file_path(robot_name, filename):
#     if robot_name == "":
#         return "/tmp/" + filename
#     else:
#         return "/tmp/" + robot_name + "_" + filename


def get_file_path(file_configuration):
    pkg = file_configuration["pkg"]
    file = file_configuration["file"]
    return get_package_share_directory(pkg) + "/" + file


def load_configuration(configuration_file_path):
    with open(configuration_file_path, 'r') as f:
        return yaml.safe_load(f)


def save_configuration(configuration, configuration_file_path):
    with open(configuration_file_path, 'w') as f:
        yaml.dump(configuration, f)


def meta_description_type(meta_description_file_path):
    return meta_description_file_path.split(".")[1]


def load_meta_description(meta_description_file_path, device_type=None):
    if device_type is None:
        device_type = meta_description_type(meta_description_file_path)
    bringup = importlib.import_module("romea_" + device_type + "_bringup")
    return bringup.load_meta_description(meta_description_file_path)


def load_meta_descriptions(meta_description_file_paths):
    return [
        load_meta_description(meta_description_file_path)
        for meta_description_file_path in meta_description_file_paths
    ]


def find_meta_description(meta_descriptions, device_name):
    meta_description = next(
        (
            meta_description
            for meta_description in meta_descriptions
            if meta_description.get_name() == device_name
        ),
        None,
    )

    if meta_description is None:

        raise LookupError(
            "Cannot find meta description for device called"
            + device_name
            + "into the provided meta descriptions list")

    return meta_description


def find_meta_descriptions(meta_descriptions, devices_names):
    return [
        find_meta_description(meta_descriptions, device_name)
        for device_name in devices_names
    ]


class MetaDescription:
    def __init__(self, description_type, meta_description_file_path):

        self.type = description_type
        # TODO assert description type when base metadescription
        # will be called like this base_config_filename.mobile_base.yaml
        self.filename = meta_description_file_path
        with open(meta_description_file_path) as f:
            self.data = yaml.safe_load(f)

    def exists(self, param, ns=None):
        if ns:
            return ns in self.data and param in self.data[ns]
        else:
            return param in self.data

    def get_or(self, param, ns=None, default=None):

        if ns is not None:
            try:
                return self.data[ns].get(param, default)
            except Exception:
                raise LookupError(
                    "Cannot get param "
                    + self.param_name_(param, ns)
                    + " from "
                    + self.type
                    + " description file "
                    + self.filename
                )
        else:
            return self.data.get(param, default)

    def get(self, param, ns=None):

        value = self.get_or(param, ns)

        if value is None:
            raise LookupError(
                "Cannot get param "
                + self.param_name_(param, ns)
                + " from "
                + self.type
                + " description file "
                + self.filename
            )

        return value

    def param_name_(self, param, ns):
        if ns:
            return ns + "." + param
        else:
            return param
