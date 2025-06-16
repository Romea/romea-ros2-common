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


def get_configuration_file(directory_path, filename):
    files = [
        f for f in os.listdir(directory_path) if os.path.isfile(os.path.join(directory_path, f))
    ]

    for file in files:
        if len(file) != len(filename):
            continue

        if all(f == "x" or f == fn for f, fn in zip(file, filename)):
            return f"{directory_path}/{file}"

    return None


def generate_configuration_file(configuration, units, extended, depth=0):
    yaml_lines = []
    indent = "    " * depth
    for key, value in configuration.items():
        if isinstance(value, dict):
            yaml_lines.append(f"{indent}{key}:")
            yaml_lines.extend(
                generate_configuration_file(value, units, extended, depth + 1).splitlines()
            )
        else:
            if extended:
                yaml_lines.append(f"{indent}{key}:")
                yaml_lines.append(f"{indent}    value: {value}")
                yaml_lines.append(f"{indent}    unit: {units.get(key, 'null')}")
            else:
                yaml_lines.append(f"{indent}{key}: {value}  # unit {units.get(key, 'none')}")

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

        if not isinstance(specification, dict):
            return self.evaluate_parameter(parameter_name, specification, user_value)
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

    def evaluate_parameter(self, parameter_name, specification, user_value):

        if user_value is None or specification == user_value:
            return specification
        else:
            raise ValueError(
                parameter_name
                + " value ("
                + str(user_value)
                + self.units.get(parameter_name, "")
                + ") provided by user is not available for "
                + self.device_name
                + ", it must be equal to "
                + str(specification)
            )
