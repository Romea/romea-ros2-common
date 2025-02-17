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
import pytest
from romea_common_meta_bringup import DriverLaunchFileConfiguration


@pytest.fixture
def profile_filename():
    return os.path.join(os.getcwd(), "test_driver_launch_file_profile.yaml")


@pytest.fixture
def configuration():
    config_filename = os.path.join(os.getcwd(), "test_driver_launch_file_configuration.yaml")
    with open(config_filename, "r") as f:
        return yaml.safe_load(f)


def test_get_complete_configuration(profile_filename, configuration):
    complete_configuration = DriverLaunchFileConfiguration(
        profile_filename, configuration
    ).evaluate()

    assert complete_configuration["driver1"]["parameters"]["p1"] == 1
    assert complete_configuration["driver1"]["parameters"]["p2"] == 2
    assert complete_configuration["driver1"]["parameters"]["p3"] == 3
    assert complete_configuration["driver1"]["parameters"]["p4"] == 4
    assert complete_configuration["driver2"]["parameters"]["p5"] == 5
    assert complete_configuration["driver2"]["parameters"]["p6"] == 6
    assert complete_configuration["driver2"]["parameters"]["p7"] == 7


def test_check_failed_when_driver_configuration_is_missing(profile_filename, configuration):

    del configuration["driver_configuration"]
    with pytest.raises(ValueError) as excinfo:
        DriverLaunchFileConfiguration(profile_filename, configuration).evaluate()

    assert (
        "No driver_configuration is provided, unable to substitute parameter driver1.parameters.p2"
        in str(excinfo.value)
    )


def test_check_failed_when_device_configuration_is_missing(profile_filename, configuration):

    del configuration["device_configuration"]
    with pytest.raises(ValueError) as excinfo:
        DriverLaunchFileConfiguration(profile_filename, configuration).evaluate()

    assert (
        "No device_configuration is provided, unable to substitute parameter driver1.parameters.p3"
        in str(excinfo.value)
    )


def test_check_failed_when_parameter_does_not_exists_in_device_configuration_(
    profile_filename, configuration
):

    del configuration["device_configuration"]["foo"]
    with pytest.raises(ValueError) as excinfo:
        DriverLaunchFileConfiguration(profile_filename, configuration).evaluate()

    assert (
        "Parameter foo does not exists in device_configuration, unable to substitute parmeter"
        in str(excinfo.value)
    )


def test_check_failed_when_parameter_does_not_exists_in_driver_configuration(
    profile_filename, configuration
):

    del configuration["driver_configuration"]["driver2"]["parameters"]["p6"]
    with pytest.raises(ValueError) as excinfo:
        DriverLaunchFileConfiguration(profile_filename, configuration).evaluate()

    assert (
        "Parameter driver2.parameters.p6 does not exists in driver_configuration, unable"
        in str(excinfo.value)
    )
