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


import pytest

from romea_common_description import (
    get_user_configuration_parameter,
    get_specification_parameter,
    evaluate_parameter,
    evaluate_parameter_from_list,
    evaluate_parameter_from_range,
    # get_device_geometry,
    # get_device_specifications,
)


# def get_specifications():
#     return get_device_specifications("common", "example_of", "device")


def test_get_user_configuration_parameter_ok():
    user_device_configuration = {"rate": 50}
    assert get_user_configuration_parameter("lidar", "sick", "lms1xx", user_device_configuration, "rate") == 50


def test_get_user_configuration_parameter_failed():
    user_device_configuration = {"rate": 50}
    with pytest.raises(LookupError) as excinfo:
        get_user_configuration_parameter("lidar", "sick", "lms1xx", user_device_configuration, "resolution")


def test_get_specification_parameter_ok():
    lidar_specification = {"minimal_range": 0.5, "maximal_range": {"lms10x": 20.0}}
    assert (get_specification_parameter("lidar", "sick", "lms1xx", lidar_specification, "minimal_range", None)) == 0.5


def test_get_specification_parameter_failed():
    lidar_specification = {"minimal_range": 0.5, "maximal_range": {"lms10x": 20.0}}
    with pytest.raises(LookupError) as excinfo:
        get_specification_parameter("lidar", "sick", "lms1xx", lidar_specification, "range_std", None)


def test_get_specification_parameter_with_key_ok():
    lidar_specification = {"minimal_range": 0.5, "maximal_range": {"lms10x": 20.0}}
    assert (
        get_specification_parameter("lidar", "sick", "lms1xx", lidar_specification, "maximal_range", "lms10x")
    ) == 20.0


def test_get_specification_parameter_with_key_failed():
    lidar_specification = {"minimal_range": 0.5, "maximal_range": {"lms10x": 20.0}}
    with pytest.raises(LookupError) as excinfo:
        get_specification_parameter("lidar", "sick", "lms1xx", lidar_specification, "maximal_range", "lms11x")


def test_evaluate_parameter():
    user_device_configuration = {"minimal_range": 0.5}
    lidar_specification = {"minimal_range": 0.5, "maximal_range": {"lms10x": 20.0}}
    assert (
        evaluate_parameter("lidar", "sick", "lms1xx", lidar_specification, user_device_configuration, "minimal_range")
        == 0.5
    )


def test_evaluate_parameter_default():
    user_device_configuration = {"minimal_range": None}
    lidar_specification = {"minimal_range": 0.5, "maximal_range": {"lms10x": 20.0}}
    assert (
        evaluate_parameter("lidar", "sick", "lms1xx", lidar_specification, user_device_configuration, "minimal_range")
        == 0.5
    )


def test_evaluate_parameter_failed():
    user_device_configuration = {"minimal_range": 1.0}
    lidar_specification = {"minimal_range": 0.5, "maximal_range": {"lms10x": 20.0}}
    with pytest.raises(ValueError) as excinfo:
        evaluate_parameter("lidar", "sick", "lms1xx", lidar_specification, user_device_configuration, "minimal_range")


def test_evaluate_parameter_with_key():
    user_device_configuration = {"maximal_range": 20.0}
    lidar_specification = {"minimal_range": 0.5, "maximal_range": {"lms10x": 20.0}}
    assert (
        evaluate_parameter(
            "lidar", "sick", "lms1xx", lidar_specification, user_device_configuration, "maximal_range", "lms10x"
        )
        == 20.0
    )


def test_evaluate_parameter_with_key_default():
    user_device_configuration = {"maximal_range": None}
    lidar_specification = {"minimal_range": 0.5, "maximal_range": {"lms10x": 20.0}}
    assert (
        evaluate_parameter(
            "lidar", "sick", "lms1xx", lidar_specification, user_device_configuration, "maximal_range", "lms10x"
        )
        == 20.0
    )


def test_evaluate_parameter_with_key_failed():
    user_device_configuration = {"maximal_range": 30.0}
    lidar_specification = {"minimal_range": 0.5, "maximal_range": {"lms10x": 20.0}}
    with pytest.raises(ValueError) as excinfo:
        evaluate_parameter(
            "lidar", "sick", "lms1xx", lidar_specification, user_device_configuration, "maximal_range", "lms10x"
        )


def test_evaluate_parameter_from_list():
    user_device_configuration = {"resolution": 0.5}
    lidar_specification = {"resolution": {"default": 0.25, "list": [0.25, 0.5]}}
    assert (
        evaluate_parameter_from_list(
            "lidar", "sick", "lms1xx", lidar_specification, user_device_configuration, "resolution"
        )
        == 0.5
    )

def test_evaluate_parameter_from_list_default():
    user_device_configuration = {"resolution": None}
    lidar_specification = {"resolution": {"default": 0.25, "list": [0.25, 0.5]}}
    assert (
        evaluate_parameter_from_list(
            "lidar", "sick", "lms1xx", lidar_specification, user_device_configuration, "resolution"
        )
        == 0.25
    )

def test_evaluate_parameter_from_list_failed():
    user_device_configuration = {"resolution": 0.75}
    lidar_specification = {"resolution": {"default": 0.25, "list": [0.25, 0.5]}}
    with pytest.raises(ValueError) as excinfo:
        evaluate_parameter_from_list(
            "lidar", "sick", "lms1xx", lidar_specification, user_device_configuration, "resolution"
        )

def test_evaluate_parameter_from_range():
    user_device_configuration = {"horizontal_fov": 27}
    lidar_specification = {"horizontal_fov": {"default": 72, "range": [27, 72]}}
    assert (
        evaluate_parameter_from_range(
            "camera", "axis", "p1346", lidar_specification, user_device_configuration, "horizontal_fov"
        )
        == 27
    )


def test_evaluate_parameter_from_range_default():
    user_device_configuration = {"horizontal_fov": None}
    lidar_specification = {"horizontal_fov": {"default": 72, "range": [27, 72]}}
    assert (
        evaluate_parameter_from_range(
            "camera", "axis", "p1346", lidar_specification, user_device_configuration, "horizontal_fov"
        )
        == 72
    )

def test_evaluate_parameter_from_range_failed():
    user_device_configuration = {"horizontal_fov": 100}
    lidar_specification = {"horizontal_fov": {"default": 72, "range": [27, 72]}}
    with pytest.raises(ValueError) as excinfo:
        evaluate_parameter_from_range(
            "camera", "axis", "p1346", lidar_specification, user_device_configuration, "horizontal_fov"
        )
