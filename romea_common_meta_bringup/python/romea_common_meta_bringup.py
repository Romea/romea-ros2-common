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

from ament_index_python import get_package_share_directory
from functools import reduce
from os.path import join
import importlib
import yaml
import os
import math


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


def device_configuration_filename(robot_name, device_name, filename):
    return f"{device_urdf_prefix(robot_name,device_name)}{filename}"


def save_temporary_file(configuration, filename):
    temporaty_filename = f"/tmp/{filename}"
    with open(temporaty_filename, 'w') as f:
        yaml.dump(configuration, f)
    return temporaty_filename


def get_file_path(file_configuration):
    pkg = file_configuration["pkg"]
    file = file_configuration["file"]
    return get_package_share_directory(pkg) + "/" + file


def load_configuration(configuration_file_path):
    with open(configuration_file_path, "r") as f:
        return yaml.safe_load(f)


def save_configuration(configuration, configuration_file_path):
    with open(configuration_file_path, "w") as f:
        yaml.dump(configuration, f)


def meta_description_type(meta_description_file_path):
    return meta_description_file_path.split(".")[1]


def load_meta_description(meta_description_file_path, device_type=None):
    if device_type is None:
        device_type = meta_description_type(meta_description_file_path)
    bringup = importlib.import_module("romea_" + device_type + "meta_bringup")
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
            + "into the provided meta descriptions list"
        )

    return meta_description


def find_meta_descriptions(meta_descriptions, devices_names):
    return [find_meta_description(meta_descriptions, device_name) for device_name in devices_names]


class MetaDescription:
    def __init__(self, description_type, meta_description_file_path):

        self.type = description_type
        # TODO assert description type when base metadescription
        # will be called like this base_config_filename.mobile_base.yaml
        self.filename = meta_description_file_path

        if not os.path.exists(meta_description_file_path):
            raise LookupError(
                description_type
                + "meta description file "
                + meta_description_file_path
                + " does not exists"
            )

        with open(meta_description_file_path) as f:
            self.data = yaml.safe_load(f)

        if "configuration" in self.data:
            if "type" in self.data["configuration"]:
                self.data["configuration"]["manufacturer"] = self.data["configuration"]["type"]
                del self.data["configuration"]["type"]

    def exists(self, param, ns=None):
        if ns:
            return ns in self.data and param in self.data[ns]
        else:
            return param in self.data

    def get_or(self, param, ns=None, default=None):

        if ns is not None:
            try:
                config = self.data
                for key in ns.split("."):
                    config = config[key]

                return config.get(param, default)
                # return self.data[ns].get(param, default)
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


class SensorMetaDescription:
    def __init__(self, description_type, meta_description_file_path, robot_name=None):
        self.__type = description_type
        self.__robot_name = str(robot_name or "")
        self.__filename = meta_description_file_path

        if not os.path.exists(meta_description_file_path):
            raise LookupError(
                description_type
                + "meta description file "
                + meta_description_file_path
                + " does not exists"
            )

        with open(meta_description_file_path) as f:
            self.__description = yaml.safe_load(f)

    def get_name(self):
        return self._get("name")

    def get_robot_name(self):
        return self.__robot_name

    def get_namespace(self):
        return self._get_or("namespace", None)

    def get_full_namespace(self):
        return device_namespace(self.get_robot_name(), self.get_namespace(), self.get_name())

    def get_filename_prefix(self):
        return device_urdf_prefix(self.get_robot_name(), self.get_name())

    def get_launch_file_configuration(self):
        return self._get_or("launch_file", None, {})

    def get_configuration(self):
        return self._get("configuration")

    def get_manufacturer(self):
        return self._get("manufacturer", "configuration")

    def get_model(self):
        return self._get("model", "configuration")

    def get_location(self):
        return self._get("location")

    def get_urdf_prefix(self):
        return robot_urdf_prefix(self.get_robot_name())

    def get_link(self):
        return device_link_name(self.get_robot_name(), self.get_name())

    def get_parent_link(self):
        return self._get("parent_link", "location")

    def get_xyz(self):
        return self._get("xyz", "location")

    def get_rpy(self):
        return self._get_or("rpy", "location", [0.0, 0.0, 0.0])

    def get_records(self):
        return self._get_or("records", None, {})

    def get_bridge(self):
        return self._get_or("bridge", None, {})

    def _get_or(self, param, ns=None, default=None):

        if ns is not None:
            try:
                config = self.__description
                for key in ns.split("."):
                    config = config[key]

                return config.get(param, default)
            except Exception:
                raise LookupError(
                    "Cannot get "
                    + self.__get_param_name(param, ns)
                    + " from "
                    + self.type
                    + " description file "
                    + self.filename
                )
        else:
            return self.__description.get(param, default)

    def _get(self, param, ns=None):

        value = self._get_or(param, ns)

        if value is None:
            raise LookupError(
                "Cannot get "
                + self.__get_param_name(param, ns)
                + " from "
                + self.type
                + " description file "
                + self.filename
            )

        return value

    def __get_param_name(self, param, ns):
        if ns:
            return ns + "." + param
        else:
            return param


class DriverLaunchFileProfile:
    def __init__(self, driver_profile_filename, configuration):

        self.configuration = configuration
        self.driver_profile_filename = driver_profile_filename

        if not os.path.exists(driver_profile_filename):
            raise FileNotFoundError(
                "driver profile file " + driver_profile_filename + " not found."
            )

        with open(driver_profile_filename, "r") as f:
            self.driver_profile = yaml.safe_load(f)

    def evaluate(self, mode, namespace=None):

        for node_name, node_configuration in self.driver_profile.items():
            if not self.__is_available_mode(mode, node_configuration):
                self.driver_profile.pop(node_name, None)
                continue

            self.__evaluate_component_container(node_name, node_configuration)
            self.__evaluate_plugin(node_configuration)
            self.__evaluate_executable(node_configuration)
            # self.__set_namespace(node_configuration, namespace)
            self.__evaluate_parameters(node_name, node_configuration)
            # self.__evaluate_remappings(node_name, node_configuration)

        return self.driver_profile

    def __is_available_mode(self, mode, node_configuration):
        available_mode = node_configuration.get("avaliable_mode", "live")
        return mode == "live" or available_mode == "all"

    def __evaluate_component_container(self, node_name, node_configuration):
        if isinstance(node_configuration.get("component_container", None), dict):

            configuration_source_name = node_configuration["component_container"].get("from")
            configuration = self.configuration.get(configuration_source_name)
            parameter_name = f"{node_name}.component_container"

            if configuration is None:
                raise ValueError(
                    f"No {configuration_source_name} is provided, unable to substitute parameter "
                    f"{parameter_name} in driver profile {self.driver_profile_filename}."
                )

            value = self.__get_parameter_value(configuration, parameter_name, None)

            if value:
                node_configuration["component_container"] = value
            else:
                node_configuration.pop("component_container", None)

    # def __set_namespace(self, node_configuration, namespace):
    #     node_configuration["namespace"] = namespace

    def __evaluate_plugin(self, node_configuration):
        if "plugin" in node_configuration:
            if node_configuration.get("component_container", None) is None:
                node_configuration.pop("plugin", None)

    def __evaluate_executable(self, node_configuration):
        if node_configuration.get("plugin", None):
            if node_configuration.get("component_container", None):
                node_configuration.pop("executable", None)

    def __evaluate_parameters(self, node_name, node_configuration):
        parameters = node_configuration.get("parameters")
        if parameters:
            for parameter_name, parameter in parameters.items():
                if isinstance(parameter, dict):
                    parameters[parameter_name] = self.__evaluate_parameter(
                        f"{node_name}.parameters.{parameter_name}", parameter
                    )

    def __evaluate_parameter(self, parameter_name, parameter_description):

        configuration_source_name = parameter_description.get("from")
        configuration = self.configuration.get(configuration_source_name)

        if configuration is None:
            raise ValueError(
                f"No {configuration_source_name} is provided, unable to substitute parameter "
                f"{parameter_name} in driver profile {self.driver_profile_filename}."
            )

        configuration_parameter_name = parameter_description.get("name", parameter_name)
        parameter_default_value = parameter_description.get("default", None)

        value = self.__get_parameter_value(
            configuration, configuration_parameter_name, parameter_default_value
        )

        if value is None:
            raise ValueError(
                f"Parameter {configuration_parameter_name} does not exists in "
                f"{configuration_source_name}, unable to substitute parmeter "
                f"{parameter_name} in driver profile {self.driver_profile_filename}."
            )

        if isinstance(value, (int, float)):
            scale = parameter_description.get("scale")
            if scale is not None:
                if isinstance(scale, str):
                    value *= eval(scale, {"__builtins__": {}}, {"pi": math.pi})
                else:
                    value *= scale

        return value

    def __get_parameter_value(
        self, configuration, configuration_parameter_name, parameter_default_value
    ):
        try:
            keys = configuration_parameter_name.split(".")
            value = reduce(
                lambda config, key: config[key] if isinstance(config, dict) else key,
                keys,
                configuration,
            )

            return value
        except KeyError:
            return parameter_default_value


class DriverLaunchFileConfiguration:
    def __init__(self, device_type, meta_bringup_package=None):
        self.__device_type = device_type
        if meta_bringup_package is not None:
            self.__meta_bringup_package = meta_bringup_package
        else:
            self.__meta_bringup_package = f"romea_{device_type}_meta_bringup"

    def evaluate(
            self, mode, launch_file_configuration, device_configuration, device_namespace
    ):
        configuration = {}

        for driver_name, driver_description in launch_file_configuration.items():
            driver_launch_file_configuration = DriverLaunchFileProfile(
                self.__get_profile_filename(driver_description),
                {
                    f"{self.__device_type}_configuration": device_configuration,
                    f"{driver_name}_configuration": driver_description.get(
                        "configuration", {}
                    ),
                },
            ).evaluate(mode, device_namespace)

            configuration.update(driver_launch_file_configuration)

        return configuration

    def __get_profile_filename(self, driver_configuration):
        pkg_path = get_package_share_directory(self.__meta_bringup_package)
        return join(pkg_path, "config", driver_configuration["profile"])
