# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import re
import xml.dom

from ament_index_python.packages import get_package_share_directory

import xacro


def get_configuration_file_path(pkg, device_description, what, config_directory):
    model = device_description["model"]
    version = device_description["version"]
    manufacturer = device_description["manufacturer"]
    filename = f"{manufacturer}_{model}_{version}_{what}.yaml"
    directory_path = f"{get_package_share_directory(pkg)}/{config_directory}"

    confuration_files = [
        f for f in os.listdir(directory_path) if os.path.isfile(os.path.join(directory_path, f))
    ]

    for configuration_file in confuration_files:
        if len(configuration_file) != len(filename):
            continue

        if all(f == "x" or f == fn for f, fn in zip(configuration_file, filename)):
            return f"{directory_path}/{configuration_file}"

    raise RuntimeError(
        f"No {manufacturer} {model} {version} device is supported by {pkg} package."
        ' Please check your configuration or contribute to support this kind of device.'
    )


def get_specifications_file_path(pkg, device_description, config_directory="config"):
    return get_configuration_file_path(
        pkg, device_description, "specifications", config_directory
    )


def get_geometry_file_path(pkg, device_description, config_directory="config"):
    return get_configuration_file_path(
        pkg, device_description, "geometry", config_directory
    )


def generate_configuration_file(configuration, units, extended, depth=0):
    def get_unit(key, default=None):
        if key in units:
            return units[key]

        matches = []
        for pattern, value in units.items():
            if "*" in pattern:
                regex = "^" + re.escape(pattern).replace(r"\*", ".*") + "$"
                if re.match(regex, key):
                    matches.append((pattern, value))

        if matches:
            return max(matches, key=lambda x: len(x[0].replace("*", "")))[1]

        return default

    yaml_lines = []
    indent = "    " * depth
    for key, value in configuration.items():
        if isinstance(value, dict):
            yaml_lines.append(f"{indent}{key}:")
            yaml_lines.extend(
                generate_configuration_file(value, units, extended, depth + 1).splitlines()
            )
        else:
            unit = get_unit(key)
            if extended:
                yaml_lines.append(f"{indent}{key}:")
                yaml_lines.append(f"{indent}    value: {value}")
                if unit is not None:
                    yaml_lines.append(f"{indent}    unit: {unit}")
            else:
                if unit is None:
                    yaml_lines.append(f"{indent}{key}: {value}")
                else:
                    yaml_lines.append(f"{indent}{key}: {value}  # unit {unit}")

    return "\n".join(yaml_lines)


class DeviceConfiguration:
    def __init__(self, device_name, specifications, user_configuration, units={}):
        self.device_name = device_name
        self.specifications = specifications
        self.user_configuration = user_configuration
        self.units = units

    def get(self, parameter_name):

        user_value = self.user_configuration.get(parameter_name, None)
        specification = self.get_specification(parameter_name)

        if self._is_scalar_parameter(specification):
            return self.evaluate_scalar_parameter(parameter_name, specification, user_value)
        elif self._is_flat_dict_parameter(specification):
            return self.evaluate_flat_dict_parameter(parameter_name, specification, user_value)
        elif "list" in specification:
            return self.evaluate_parameter_from_list(parameter_name, specification, user_value)
        elif "range" in specification:
            return self.evaluate_parameter_from_range(parameter_name, specification, user_value)
        elif "dict" in specification:
            return self.evaluate_parameter_from_dict(parameter_name, specification, user_value)
        else:
            raise LookupError(
                "specification for " + parameter_name + " parameter is not well formed"
            )

    def get_specification(self, parameter_name):

        if parameter_name not in self.specifications:
            raise LookupError(
                parameter_name
                + " configuration does not exist in "
                + self.device_name
                + " specifications"
            )

        return self.specifications[parameter_name]

    def evaluate_parameter_from_list(self, parameter_name, specification, user_value):

        if user_value is None:
            if "default" in specification:
                return specification["default"]
            else:
                raise LookupError(
                    "no default "
                    + parameter_name
                    + " is provided for "
                    + self.device_name
                    + ", user must choose one these values: "
                    + str(specification["list"])
                )
        elif user_value in specification["list"]:
            return user_value
        else:
            raise ValueError(
                parameter_name
                + " value ("
                + str(user_value)
                + self.units.get(parameter_name, "")
                + ") provided by user is not available for "
                + self.device_name
                + ", it must be one of these values: "
                + str(specification["list"])
            )

    def evaluate_parameter_from_range(self, parameter_name, specification, user_value):

        if user_value is None:
            if "default" in specification:
                return specification["default"]
            else:
                raise LookupError(
                    "no default "
                    + parameter_name
                    + " is provided for "
                    + self.device_name
                    + ", user must choose a value between "
                    + str(specification["range"][0])
                    + " and "
                    + str(specification["range"][1])
                )
        elif user_value >= specification["range"][0] and user_value <= specification["range"][1]:
            return user_value
        else:
            raise ValueError(
                parameter_name
                + " value ("
                + str(user_value)
                + self.units.get(parameter_name, "")
                + ") provided by user is not available for "
                + self.device_name
                + ", it must be inside ["
                + str(specification["range"][0])
                + ", "
                + str(specification["range"][1])
                + "]"
            )

    def evaluate_parameter_from_dict(self, parameter_name, specification, user_value):

        if isinstance(specification["depend"], list):
            parameter = specification["dict"]
            for depend in specification["depend"]:
                parameter = parameter[self.get(depend)]
        else:
            parameter = specification["dict"][self.get(specification["depend"])]

        if isinstance(parameter, dict):
            if "list" in parameter:
                return self.evaluate_parameter_from_list(parameter_name, parameter, user_value)
            elif "range" in parameter:
                return self.evaluate_parameter_from_range(parameter_name, parameter, user_value)
            else:
                raise LookupError(
                    "specification for " + parameter_name + " parameter is not well formed"
                )

        else:
            if user_value is None:
                return parameter
            elif user_value == parameter:
                return user_value
            else:
                raise ValueError(
                    parameter_name
                    + " value ("
                    + str(user_value)
                    + self.units.get(parameter_name, "")
                    + ") provided by user is not available for this configuration of "
                    + self.device_name
                    + ", it must be equal to "
                    + str(parameter)
                )

    def evaluate_scalar_parameter(self, parameter_name, specification, user_value):

        if isinstance(specification, dict):
            value = specification.get("value", None)
            default_value = specification.get("default", None)
            has_value = "value" in specification
            has_default = "default" in specification
        else:
            value = specification
            default_value = None
            has_value = True
            has_default = False

        assert has_value != has_default

        if user_value is not None:
            if has_default:
                if type(user_value) is type(default_value):
                    return user_value
                else:
                    raise TypeError(
                        parameter_name
                        + " value ("
                        + str(user_value)
                        + self.units.get(parameter_name, "")
                        + ") provided by user is not available for "
                        + self.device_name
                        + ", it must be of type "
                        + type(default_value).__name__
                    )

            if user_value == value:
                return user_value
            else:
                raise ValueError(
                    parameter_name
                    + " value ("
                    + str(user_value)
                    + self.units.get(parameter_name, "")
                    + ") provided by user is not available for "
                    + self.device_name
                    + ", it must be equal to "
                    + str(value)
                )

        return default_value if has_default else value

    def evaluate_flat_dict_parameter(self, parameter_name, specification, user_value):

        exact_values = "values" in specification
        flat_dict = specification["values"] if exact_values else specification["defaults"]

        if user_value is None:
            return flat_dict

        if not isinstance(user_value, dict) or set(user_value.keys()) != set(flat_dict.keys()):
            raise TypeError(
                parameter_name
                + " must be a flat dictionary having the following keys "
                + str(list(flat_dict.keys()))
            )

        result = {}
        for key, value in flat_dict.items():
            child_specification = {"value": value} if exact_values else {"default": value}
            result[key] = self.evaluate_scalar_parameter(
                parameter_name + "." + str(key),
                child_specification,
                user_value[key],
            )

        return flat_dict if exact_values else result

    def _is_scalar(self, value):
        if isinstance(value, bool):
            return True
        return isinstance(value, (int, float, str))

    def _is_flat_dict(self, value):
        return isinstance(value, dict) and all(self._is_scalar(v) for v in value.values())

    def _is_scalar_parameter(self, specification):
        if self._is_scalar(specification):
            return True

        if isinstance(specification, dict):
            keys = set(specification.keys())
            if keys in ({"value"}, {"default"}):
                value = next(iter(specification.values()))
                return self._is_scalar(value)

        return False

    def _is_flat_dict_parameter(self, specification):
        if self._is_flat_dict(specification):
            return True

        if isinstance(specification, dict):
            keys = set(specification.keys())
            if keys in ({"values"}, {"defaults"}):
                value = next(iter(specification.values()))
                return self._is_flat_dict(value)

        return False


def generate_urdf_description_str(xacro_file, mappings):

    def remove_comments(node):
        for child in list(node.childNodes):
            if child.nodeType == xml.dom.Node.COMMENT_NODE:
                node.removeChild(child)
            else:
                remove_comments(child)

    urdf_xml = xacro.process_file(xacro_file, mappings=mappings)
    remove_comments(urdf_xml)
    return urdf_xml.toprettyxml()
