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


def full_device_model_name(device_type, device_model):
    return device_type + " " + device_model


def full_device_category_name(device_category, device_type, device_model):
    return full_device_model_name(device_type, device_model) + " " + device_category


def raised_device_configuration_file_not_found(
    device_category, device_type, device_model, file, configuration_file_type
):
    raise LookupError(
        "No "
        + configuration_file_type
        + " "
        + file
        + " found in romea_"
        + device_category
        + "_description package for "
        + full_device_category_name(device_category, device_type, device_model)
        + ". Please check you have selected good "
        + device_category
        + " type and/or model. Or add "
        + configuration_file_type
        + " file for this "
        + device_category
        + "."
    )


def get_device_configuration_file_path(device_category, device_type, device_model, configuration_file_type):

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

    if not os.path.exists(file_path):
        raised_device_configuration_file_not_found(
            device_category, device_type, device_model, file_path, configuration_file_type
        )

    return file_path


def get_device_specifications_file_path(device_category, device_type, device_model):
    return get_device_configuration_file_path(device_category, device_type, device_model, "specifications")


def get_device_geometry_file_path(device_category, device_type, device_model):
    return get_device_configuration_file_path(device_category, device_type, device_model, "geometry")


def get_device_specifications(device_category, device_type, device_model):
    with open(get_device_specifications_file_path(device_category, device_type, device_model)) as f:
        return yaml.safe_load(f)


def get_device_geometry(device_category, device_type, device_model):
    with open(get_device_geometry_file_path(device_category, device_type, device_model)) as f:
        return yaml.safe_load(f)


def save_device_configuration_file(prefix, device_name, configuration, configuration_file_type):
    configuration_file_path = "/tmp/" + prefix + device_name + "_" + configuration_file_type + ".yaml"

    with open(configuration_file_path, "w") as f:
        yaml.dump(configuration, f)

    return configuration_file_path


def save_device_specifications_file(prefix, device_name, specifications):
    return save_device_configuration_file(prefix, device_name, specifications, "specifications")


def save_device_geometry_file(prefix, device_name, geometry):
    return save_device_geometry_file(prefix, device_name, geometry, "geometry")


def get_user_configuration_parameter(
    device_category, device_type, device_model, user_device_configuration, parameter_name
):

    if parameter_name not in user_device_configuration:
        raise LookupError(
            parameter_name
            + " parameter does not exist in user configuration of "
            + full_device_category_name(device_category, device_type, device_model)
            + ". Parameters provided are : "
            + str(user_device_configuration)
            + "."
        )

    return user_device_configuration[parameter_name]


def get_specification_parameter(
    device_category, device_type, device_model, device_specifications, parameter_name, key_value
):

    if parameter_name not in device_specifications:
        raise LookupError(
            parameter_name
            + " parameter does not exist in "
            + full_device_category_name(device_category, device_type, device_model)
            + " specifications. Parameter provided are :"
            + str(device_specifications)
            + "."
        )

    if key_value is None:
        return device_specifications[parameter_name]

    if key_value not in device_specifications[parameter_name]:

        raise LookupError(
            parameter_name
            + "["
            + key_value
            + "]"
            + " parameter does not exist in "
            + full_device_category_name(device_category, device_type, device_model)
            + " specifications. Parameter provided are :"
            + str(device_specifications)
            + "."
        )

    return device_specifications[parameter_name][key_value]


def evaluate_parameter(
    device_category,
    device_type,
    device_model,
    device_specifications,
    user_device_configuration,
    parameter_name,
    key_value=None,
):
    user_parameter = get_user_configuration_parameter(
        device_category, device_type, device_model, user_device_configuration, parameter_name
    )

    specification_parameter = get_specification_parameter(
        device_category, device_type, device_model, device_specifications, parameter_name, key_value
    )

    if user_parameter is None or specification_parameter == user_parameter:
        return specification_parameter
    else:
        raise ValueError(
            parameter_name
            + " parameter equal to "
            + str(user_parameter)
            + " is not available for "
            + full_device_category_name(device_category, device_type, device_model)
            + ". It must be equal to "
            + str(specification_parameter)
            + "."
        )


def evaluate_parameter_from_list(
    device_category,
    device_type,
    device_model,
    device_specifications,
    user_device_configuration,
    parameter_name,
    key_value=None,
):

    user_parameter = get_user_configuration_parameter(
        device_category, device_type, device_model, user_device_configuration, parameter_name
    )

    specification_parameter = get_specification_parameter(
        device_category, device_type, device_model, device_specifications, parameter_name, key_value
    )

    if user_parameter is None:
        return specification_parameter["default"]
    elif user_parameter in specification_parameter["list"]:
        return user_parameter
    else:
        raise ValueError(
            parameter_name
            + " parameter equal to"
            + str(user_parameter)
            + " is not available for "
            + full_device_category_name(device_category, device_type, device_model)
            + ". It must be one of these values "
            + str(specification_parameter["list"])
            + "."
        )


def evaluate_parameter_from_range(
    device_category,
    device_type,
    device_model,
    device_specifications,
    user_device_configuration,
    parameter_name,
    key_value=None,
):
    user_parameter = get_user_configuration_parameter(
        device_category, device_type, device_model, user_device_configuration, parameter_name
    )

    specification_parameter = get_specification_parameter(
        device_category, device_type, device_model, device_specifications, parameter_name, key_value
    )

    if user_parameter is None:
        return specification_parameter["default"]
    elif (
        user_parameter >= specification_parameter["range"][0] and user_parameter <= specification_parameter["range"][1]
    ):
        return user_parameter
    else:
        raise ValueError(
            parameter_name
            + " parameter equal to "
            + str(user_parameter)
            + " is not available for "
            + full_device_category_name(device_category, device_type, device_model)
            + " It must be inside ["
            + str(specification_parameter["range"][0])
            + ","
            + str(specification_parameter["range"][1])
            + "]."
        )
