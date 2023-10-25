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
import yaml

from ament_index_python.packages import get_package_share_directory


def raised_device_configuration_file_not_found(
    device_category, file, configuration_file_type, device_type, device_model
):
    raise LookupError(
        "No "
        + configuration_file_type
        + " "
        + file
        + " found in romea_"
        + device_category
        + "_description package for "
        + device_type
        + " "
        + device_model
        + " "
        + device_category
        + ". Please check you have selected good device type and/or model. Or add "
        + configuration_file_type
        + " file for this "
        + device_category
        + "."
    )


def get_device_configuration_file_path(device_category, configuration_file_type, device_type, device_model):

    file_path = (
        get_package_share_directory("romea_" + device_category + "_description")
        + "/config/"
        + device_type
        + "_"
        + device_model
        + "_"
        + configuration_file_type
        + ".yaml"
    )

    print(file_path)

    if not os.path.exists(file_path):
        raised_device_configuration_file_not_found(
            device_category, file_path, configuration_file_type, device_type, device_model
        )

    return file_path


def get_device_specifications_file_path(device_category, device_type, device_model):
    return get_device_configuration_file_path(device_category, "specifications", device_type, device_model)


def get_device_geometry_file_path(device_category, device_type, device_model):
    return get_device_configuration_file_path(device_category, "geometry", device_type, device_model)


def get_device_specifications(device_category, device_type, device_model):
    with open(get_device_specifications_file_path(device_category, device_type, device_model)) as f:
        return yaml.safe_load(f)


def get_device_geometry(device_category, device_type, device_model):
    with open(get_device_geometry_file_path(device_category, device_type, device_model)) as f:
        return yaml.safe_load(f)


def save_device_configuration_file(configuration_file_type, prefix, lidar_name, configuration):
    configuration_file_path = "/tmp/" + prefix + lidar_name + "_" + configuration_file_type + ".yaml"

    with open(configuration_file_path, "w") as f:
        yaml.dump(configuration, f)

    return configuration_file_path


def save_device_specifications_file(prefix, lidar_name, configuration):
    save_device_configuration_file("specifications", prefix, lidar_name, configuration)


def save_device_geometry_file(prefix, lidar_name, configuration):
    save_device_geometry_file("geometry", prefix, lidar_name, configuration)


def is_parameter_can_be_evaluated(device_category, specifications, user_configuration, parameter_name):

    if parameter_name not in user_configuration:
        raise LookupError(
            parameter_name.capitalize()
            + " does not exist in user "
            + device_category
            + " configuration. Parameters provided are : "
            + str(user_configuration)
            + "."
        )

    if parameter_name not in specifications:
        raise LookupError(
            parameter_name.capitalize()
            + " does not exist in"
            + user_configuration["device_type"]
            + " "
            + user_configuration["devide_model"]
            + " "
            + device_category
            + " specifications. Parameter provided are :"
            + str(specifications)
            + "."
        )

    return True


def evaluate_parameter(device_category, specifications, user_configuration, parameter_name):

    is_parameter_can_be_evaluated(device_category, specifications, user_configuration, parameter_name)

    user_value = user_configuration[parameter_name]
    if user_value is None or specifications[parameter_name] == user_value:
        return specifications[parameter_name]
    else:
        raise ValueError(
            parameter_name.capitalize()
            + " "
            + user_value
            + " is not available for "
            + user_configuration["device_type"]
            + " "
            + user_configuration["devide_model"]
            + " "
            + device_category
            + ". It must be equal to "
            + specifications[parameter_name]
            + "."
        )


def evaluate_parameter_from_list(device_category, specifications, user_configuration, parameter_name):
    user_value = user_configuration[parameter_name]
    if user_value is None:
        return specifications[parameter_name]["default"]
    elif user_value in specifications[parameter_name]["list"]:
        return user_value
    else:
        raise ValueError(
            parameter_name.capitalize()
            + " "
            + user_value
            + " is not available for "
            + user_configuration["device_type"]
            + " "
            + user_configuration["device_model"]
            + " "
            + device_category
            + ". It must be one of these values "
            + specifications[parameter_name]["list"]
            + "."
        )


def evaluate_parameter_from_range(device_type, specifications, user_configuration, parameter_name):
    user_value = user_configuration[parameter_name]
    if user_value is None:
        return specifications[parameter_name]["default"]
    elif (
        user_value >= specifications[parameter_name]["range"][0]
        and user_value <= specifications[parameter_name]["range"][1]
    ):
        return user_value
    else:
        raise ValueError(
            parameter_name.capitalize()
            + " "
            + user_value
            + " is not available for "
            + user_configuration["device_type"]
            + " "
            + user_configuration["devide_model"]
            + " "
            + device_type
            + " It must be inside ["
            + specifications[parameter_name]["range"][0]
            + ","
            + specifications[parameter_name]["range"][1]
            + "]."
        )
