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

from romea_common_description import DeviceConfiguration


# def test_get_user_configuration_parameter_ok():
#     user_configuration = {"rate": 50}
#     specifications = {}
#     assert get_user_configuration_parameter("lidar", "sick", "lms1xx", user_device_configuration, "rate") == 50


# def test_get_user_configuration_parameter_failed():
#     user_device_configuration = {"rate": 50}
#     with pytest.raises(LookupError) as excinfo:
#         get_user_configuration_parameter("lidar", "sick", "lms1xx", user_device_configuration, "resolution")


def test_get_unkown_parameter_failed():
    lidar_specification = {"minimal_range": 0.5}
    configuration = DeviceConfiguration("lidar", lidar_specification, {})
    msg = "maximal_range configuration does not exist in lidar specifications"
    with pytest.raises(LookupError) as excinfo:
        configuration.get("maximal_range")
    assert excinfo.value.args[0] == msg


def test_get_parameter_without_user_configuration_ok():
    lidar_specification = {"minimal_range": 0.5}
    configuration = DeviceConfiguration("lidar", lidar_specification, {})
    assert configuration.get("minimal_range") == 0.5


def test_get_parameter_with_user_configuration_ok():
    lidar_specification = {"minimal_range": 0.5}
    user_configuration = {"minimal_range": 0.5}
    configuration = DeviceConfiguration("lidar", lidar_specification, user_configuration)
    assert configuration.get("minimal_range") == 0.5


def test_get_parameter_with_bad_user_configuration_failed():
    lidar_specification = {"minimal_range": 0.5}
    user_configuration = {"minimal_range": 0.1}
    configuration = DeviceConfiguration("lidar", lidar_specification, user_configuration)
    msg = (
        "minimal_range value (0.1) provided by user is not available for lidar, "
        + "it must be equal to 0.5"
    )
    with pytest.raises(ValueError) as excinfo:
        configuration.get("minimal_range")
    assert excinfo.value.args[0] == msg


def test_get_default_parameter_from_list_ok():
    user_configuration = {}
    camera_specification = {
        "resolution": {
            "default": "1280x720",
            "list": ["1280x720", "800x600", "640x480"],
        }
    }
    configuration = DeviceConfiguration("camera", camera_specification, user_configuration)
    assert configuration.get("resolution") == "1280x720"


def test_get_default_parameter_from_list_failed():
    user_configuration = {}
    camera_specification = {
        "resolution": {
            "list": ["1280x720", "800x600", "640x480"],
        }
    }
    configuration = DeviceConfiguration("camera", camera_specification, user_configuration)
    msg = (
        "no default resolution is provided for camera, "
        + "user must choose one these values: ['1280x720', '800x600', '640x480']"
    )
    with pytest.raises(LookupError) as excinfo:
        configuration.get("resolution")
    assert excinfo.value.args[0] == msg


def test_get_parameter_from_list_ok():
    user_configuration = {"resolution": "640x480"}
    camera_specification = {
        "resolution": {
            "default": "1280x720",
            "list": ["1280x720", "800x600", "640x480"],
        }
    }
    configuration = DeviceConfiguration("camera", camera_specification, user_configuration)
    assert configuration.get("resolution") == "640x480"


def test_get_parameter_from_list_failed():
    user_configuration = {"resolution": "480x640"}
    camera_specification = {
        "resolution": {
            "default": "1280x720",
            "list": ["1280x720", "800x600", "640x480"],
        }
    }
    configuration = DeviceConfiguration("camera", camera_specification, user_configuration)
    msg = (
        "resolution value (480x640) provided by user is not available "
        + "for camera, it must be one of these values: ['1280x720', '800x600', '640x480']"
    )
    with pytest.raises(ValueError) as excinfo:
        configuration.get("resolution")
    assert excinfo.value.args[0] == msg


def test_get_default_parameter_from_range_ok():
    user_configuration = {}
    camera_specification = {
        "brightness": {
            "default": 50,
            "range": [0, 255],
        }
    }
    configuration = DeviceConfiguration("camera", camera_specification, user_configuration)
    assert configuration.get("brightness") == 50


def test_get_default_parameter_from_range_failed():
    user_configuration = {}
    camera_specification = {
        "brightness": {
            "range": [0, 255],
        }
    }
    configuration = DeviceConfiguration("camera", camera_specification, user_configuration)
    msg = (
        "no default brightness is provided for camera, "
        + "user must choose a value between 0 and 255"
    )
    with pytest.raises(LookupError) as excinfo:
        configuration.get("brightness")
    assert excinfo.value.args[0] == msg


def test_get_parameter_from_range_ok():
    user_configuration = {"brightness": 70}
    camera_specification = {
        "brightness": {
            "range": [0, 255],
        }
    }

    configuration = DeviceConfiguration("camera", camera_specification, user_configuration)
    assert configuration.get("brightness") == 70


def test_get_parameter_from_range_failed():
    user_configuration = {"brightness": -1}
    camera_specification = {
        "brightness": {
            "range": [0, 255],
        }
    }
    configuration = DeviceConfiguration("camera", camera_specification, user_configuration)
    msg = (
        "brightness value (-1) provided by user is not available "
        + "for camera, it must be inside [0, 255]"
    )
    with pytest.raises(ValueError) as excinfo:
        configuration.get("brightness")
    assert excinfo.value.args[0] == msg


def test_get_parameter_from_dict_ok():
    user_configuration = {"model": "os1_32"}
    lidar_specification = {
        "model": {
            "list": ["os1_32", "os1_64", "os1_128"],
        },
        "lasers": {
            "depend": "model",
            "dict": {
                "os1_32": 32,
                "os1_64": 64,
                "os1_128": 128,
            },
        },
    }

    configuration = DeviceConfiguration("lidar", lidar_specification, user_configuration)
    assert configuration.get("lasers") == 32


def test_get_parameter_from_dict_failed_because_depend_configuration_is_not_defined():
    lidar_specification = {
        "model": {
            "list": ["os1_32", "os1_64", "os1_128"],
        },
        "lasers": {
            "depend": "model",
            "dict": {
                "os1_32": 32,
                "os1_64": 64,
                "os1_128": 128,
            },
        },
    }

    configuration = DeviceConfiguration("lidar", lidar_specification, {})
    msg = (
        "no default model is provided for lidar, user must choose one "
        + "these values: ['os1_32', 'os1_64', 'os1_128']"
    )
    with pytest.raises(LookupError) as excinfo:
        configuration.get("lasers")
    assert excinfo.value.args[0] == msg


def test_get_parameter_from_dict_with_iterative_depends_ok():
    user_configuration = {"model": "lms100"}
    lidar_specification = {
        "model": {
            "list": ["lms100", "lms101", "lms102"],
        },
        "model_family": {
            "depend": "model",
            "dict": {
                "lms100": "lms10x",
                "lms101": "lms10x",
                "lms102": "lms10x",
            },
        },
        "maximal_range": {
            "depend": "model_family",
            "dict": {
                "lms10x": 20.0,
                "lms14x": 40.0,
                "lms15x": 50.0,
            },
        },
    }

    configuration = DeviceConfiguration("lidar", lidar_specification, user_configuration)
    assert configuration.get("maximal_range") == 20.0


def test_get_parameter_from_dict_with_multiple_depends_ok():

    user_configuration = {
        "model": "landmark40",
        "acceleration_range": 19.62
    }

    imu_specification = {
        "model": {
            "list": ["landmark10", "landmark20", "landmark30", "landmark40"],
        },
        "acceleration_range": {
            "default": 98.1,
            "list": [19.62, 98.1],
        },
        "acceleration_noise_density": {
            "depend": ["model", "acceleration_range"],
            "dict": {
                "landmark40": {
                    19.62: 0.39240e-03,
                    98.1: 1.1772e-03,
                }
            }
        }
    }

    configuration = DeviceConfiguration("lidar", imu_specification, user_configuration)
    assert configuration.get("acceleration_noise_density") == 0.0003924


def test_get_parameter_from_list_into_a_dict_ok():

    user_configuration = {
        "resolution": "1920x1080",
        "frame_rate": 30
    }

    camera_specification = {
        "resolution": {
            "default": "1280x720",
            "list": ["2208x1242", "1920x1080", "1280x720", "672x376"]
        },
        "frame_rate": {
            "depend": "resolution",
            "dict": {
                "2208x1242": 15,
                "1920x1080": {
                    "default": 30,
                    "list": [15, 30],
                },
                "1280x720": {
                    "default": 30,
                    "list": [15, 30, 60],
                },
                "672x376": {
                    "default": 30,
                    "list": [15, 30, 60, 100]
                }
            }
        }
    }

    configuration = DeviceConfiguration("camera", camera_specification, user_configuration)
    assert configuration.get("frame_rate") == 30
