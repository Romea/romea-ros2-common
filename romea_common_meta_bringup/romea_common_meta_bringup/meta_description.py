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

import os

import yaml

from .utils import (
    device_link_name,
    device_namespace,
    device_urdf_prefix,
    robot_urdf_prefix
)


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

        if "configuration" in self.__description:
            if "version" not in self.__description["configuration"]:
                self.__description["configuration"]["version"] = ""
            if not isinstance(self.__description["configuration"]["version"], str):
                self.__description["configuration"]["version"] = (
                    str(self.__description["configuration"]["version"])
                )

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

    def get_version(self):
        return str(self._get_or("version", "configuration", ""))

    def get_location(self):
        return self._get("location")

    def get_simulation(self):
        return self._get_or("simulation", None, {})

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
