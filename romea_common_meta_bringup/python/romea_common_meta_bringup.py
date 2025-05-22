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
    with open(temporaty_filename, "w") as f:
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


def load_meta_description(meta_description_file_path, entity_type=None):
    if entity_type is None:
        entity_type = meta_description_type(meta_description_file_path)
    bringup = importlib.import_module("romea_" + entity_type + "meta_bringup")
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

    def get_launch_file(self):
        return self._get_or("launch", None, {})

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
                    + self.__type
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
                + self.__type
                + " description file "
                + self.__filename
            )

        return value

    def __get_param_name(self, param, ns):
        if ns:
            return ns + "." + param
        else:
            return param


class NodeLaunchFileProfile:
    def __init__(self, node_profile_name, node_configuration, device_configuration):

        self.node_profile_name = node_profile_name
        self.device_configuration = device_configuration
        self.node_configuration = node_configuration

        if not os.path.exists(node_profile_name):
            raise FileNotFoundError("node profile file " + node_profile_name + " not found.")

        with open(node_profile_name, "r") as f:
            self.node_profile = yaml.safe_load(f)

    def evaluate(self, node_type):

        self.__check_pkg()
        if node_type == "node":
            self.__check_exec()
            self.__remove_plugin()
        else:
            self.__check_plugin()
            self.__remove_exec()

        self.__check_name()
        # self.__set_namespace()
        self.__evaluate_parameters()
        # self.__evaluate_remappings()

        return self.node_profile

    # def __set_namespace(self, node_configuration, namespace):
    #     node_configuration["namespace"] = namespace

    def __check_pkg(self):
        if "pkg" not in self.node_profile:
            raise LookupError("no pkg is given in" + self.node_profile_name)

    def __check_exec(self):
        if "exec" not in self.node_profile:
            raise LookupError("no exec is given in" + self.node_profile_name)

    def __check_plugin(self):
        if "exec" not in self.node_profile:
            raise LookupError("no plugin is given in" + self.node_profile_name)

    def __remove_plugin(self):
        self.node_profile.pop("plugin", None)

    def __remove_exec(self):
        self.node_profile.pop("exec", None)

    def __check_name(self):
        if "name" not in self.node_profile:
            raise LookupError("no name is given in" + self.node_profile_name)

    def __evaluate_parameters(self):
        for param in self.node_profile.get("param", []):
            if isinstance(param["value"], dict):
                param["value"] = self.__evaluate_parameter(param["name"], param["value"])

    def __evaluate_parameter(self, parameter_name, parameter_description):

        configuration_source_name = self.__get_configuration_source_name(
            parameter_name, parameter_description
        )

        configuration_parameter_name = self.__get_configuration_parameter_name(
            parameter_name, parameter_description
        )

        parameter_default_value = parameter_description.get("default", None)

        value = self.__get_parameter_value(
            parameter_name,
            configuration_source_name,
            configuration_parameter_name,
            parameter_default_value,
        )

        format_expression = parameter_description.get("format")
        if format_expression:
            value = self.__evaluate_parameter_format_expression(
                format_expression, {configuration_parameter_name: value}
            )

        if value is None:
            raise ValueError(
                f"Failed to evaluatue fomat expression {format_expression}, unable to substitute "
                f"parameter {parameter_name} in driver profile {self.driver_profile_filename}."
            )

        self.__check_parameter_value(parameter_name, parameter_description, value)
        return value

    def __get_parameter_value(
        self,
        parameter_name,
        configuration_source_name,
        configuration_parameter_name,
        parameter_default_value,
    ):
        # print("node_configuration", self.node_configuration)
        # print("device_configuration", self.device_configuration)
        if configuration_source_name == "node_configuration":
            params = {
                item["name"]: item["value"] for item in self.node_configuration.get("param", [])
            }
        else:
            params = self.device_configuration or {}

        try:
            return params[configuration_parameter_name]
        except KeyError:
            if parameter_default_value is None:
                raise ValueError(
                    f"Parameter {configuration_parameter_name} does not exists in "
                    f"{configuration_source_name}, unable to substitute parmeter "
                    f"{parameter_name} in driver profile {self.node_profile_name}."
                )
            return parameter_default_value

    def __evaluate_parameter_format_expression(self, expression, variables):
        try:
            safe_globals = {"__builtins__": None, "math": math, "pi": math.pi}
            safe_globals.update(variables)
            expression = expression.format(**variables)
            return eval(expression, safe_globals)
        except Exception:
            return None

    def __get_configuration_source_name(self, parameter_name, parameter_description):
        configuration_source_name = parameter_description.get("source", {}).get("from")
        if configuration_source_name is None:
            raise ValueError(
                f"No 'from' item is provided in source info, unable to substitute parameter "
                f"{parameter_name} in driver profile {self.node_profile_name}."
            )
        return configuration_source_name

    def __get_configuration_parameter_name(self, parameter_name, parameter_description):
        configuration_parameter_name = parameter_description.get("source", {}).get("param")
        if configuration_parameter_name is None:
            raise ValueError(
                f"No 'parameter' item is provided in source info, unable to substitute parameter "
                f"{parameter_name} in driver profile {self.node_profile_name}."
            )
        return configuration_parameter_name

    def __check_parameter_value(self, parameter_name, parameter_description, value):
        choices = parameter_description.get("choices")
        if choices and value not in choices:
            raise ValueError(
                f"Parameter value {value} does not belong to choices {choices}, "
                f"unable to substitute parameter {parameter_name} "
                f"in driver profile {self.node_profile_name}."
            )


class LaunchFileGenerator:
    def __init__(self, entity_type):
        self.__entity_namespace_name = f"{entity_type}_namespace"
        self.__meta_bringup_package = f"romea_{entity_type}_meta_bringup"

    def generate(self, mode, launch_file, device_configuration, robot_name, entity_name):

        actions = []
        actions.append(self.__generate_push_ros_namespace("robot_namespace"))
        actions.append(self.__generate_push_ros_namespace(self.__entity_namespace_name))
        for item in launch_file:
            if "node" in item:
                actions.append(self.__generate_node(
                    item["node"].copy(), device_configuration
                    )
                )
            elif "node_container" in item:
                actions.append(
                    self.__generate_node_container(
                        item["node_container"].copy(), device_configuration
                    )
                )
            elif "load_composable_node" in item:
                actions.append(
                    self.__generate_load_composable_node(
                        item["load_composable_node"].copy(), device_configuration
                    )
                )
            else:
                actions.append(item)

        launch = []
        launch.append(self.__generate_argument("mode", "live"))
        launch.append(self.__generate_argument("robot_namespace", robot_name))
        launch.append(self.__generate_argument(self.__entity_namespace_name, entity_name))
        launch.append({"group": actions})

        return yaml.dump(
            {"launch": launch},
            default_flow_style=False,
            allow_unicode=True,
            encoding=None,
            sort_keys=False,
        )

    def __generate_argument(self, name, default_value=None):
        return {"arg": {"name": name, "default": default_value}}

    def __generate_push_ros_namespace(self, namespace):
        return {"push-ros-namespace": {"namespace": f"$(var {namespace})"}}

    def __generate_node(self, node, device_configuration):
        if "profile" in node:
            file = self.__get_profile_filename(node)
            node_generator = NodeLaunchFileProfile(file, node, device_configuration)
            return {"node": node_generator.evaluate("node")}
        else:
            return {"node": node}

    def __generate_plugin(self, plugin, device_configuration):
        if "profile" in plugin:
            file = self.__get_profile_filename(plugin)
            node_generator = NodeLaunchFileProfile(file, plugin, device_configuration)
            return node_generator.evaluate("plugin")
        else:
            return plugin

    def __generate_plugins(self, plugins, device_configuration):
        return [self.__generate_plugin(plugin, device_configuration) for plugin in plugins]

    def __generate_node_container(self, node_container, device_configuration):
        node_container["composable_node"] = self.__generate_plugins(
            node_container["composable_node"], device_configuration
        )
        return {"node_container": node_container}

    def __generate_load_composable_node(self, load_composable_node, device_configuration):
        load_composable_node["composable_node"] = self.__generate_plugins(
            load_composable_node["composable_node"], device_configuration
        )
        return {"load_composable_node": load_composable_node}

    def __get_profile_filename(self, node_or_plugin_configuration):
        file = node_or_plugin_configuration.get("profile", None)
        if os.path.isabs(file):
            return file
        else:
            pkg = node_or_plugin_configuration.get("pkg", self.__meta_bringup_package)
            return os.path.join(get_package_share_directory(pkg), file)
