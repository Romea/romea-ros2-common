# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

# !/usr/bin/env python3
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


def robot_urdf_prefix(robot_namespace):

    if robot_namespace == "":
        return ""
    else:
        return robot_namespace + "_"


def device_urdf_prefix(robot_namespace, device_name):
    if device_name == "":
        return robot_urdf_prefix(robot_namespace)
    else:
        return robot_urdf_prefix(robot_namespace)+device_name+"_"


def device_link_name(robot_namespace, device_name):
    return robot_urdf_prefix(robot_namespace) + device_name + "_link"


class MetaDescription:
    def __init__(self, description_type, meta_description_file_path):

        self.type = description_type
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
