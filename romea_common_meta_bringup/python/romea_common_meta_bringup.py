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

        # if "configuration" in self.data:
        #     if "type" in self.data["configuration"]:
        #         self.data["configuration"]["manufacturer"] = self.data["configuration"]["type"]
        #         del self.data["configuration"]["type"]

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


class LaunchFileGenerator:
    def __init__(self, entity_type):
        self.__entity_namespace_name = f"{entity_type}_namespace"

    def generate(self, launch_file, device_configuration, robot_name, entity_name):

        launch = []
        launch.append(self.__generate_argument("mode", "live"))

        actions = []
        actions.append(self.__generate_push_ros_namespace(robot_name))
        actions.append(self.__generate_push_ros_namespace(entity_name))
        for name, value in self.__flatten(device_configuration).items():
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

    def __generate_argument(self, name, default_value=None):
        return {"arg": {"name": name, "default": default_value}}

    def __generate_push_ros_namespace(self, namespace):
        return {"push-ros-namespace": {"namespace": namespace}}

    def __generate_let(self, name, value):
        return {"let": {"name": name, "value": value}}

    def __flatten(self, device_configuration, parent_key=''):
        items = {}
        for k, v in device_configuration.items():
            new_key = f"{parent_key}.{k}" if parent_key else k
            if isinstance(v, dict):
                items.update(self.__flatten(v, new_key))
            else:
                items[new_key] = str(v)
        return items
