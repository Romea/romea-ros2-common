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
import pytest
from romea_common_meta_bringup import NodeLaunchFileProfile


@pytest.fixture
def profile_filename():
    return os.path.join(os.getcwd(), "test_node_profile.yaml")


@pytest.fixture
def device_configuration():
    return {"foo": 3, "bar": 2}


@pytest.fixture
def node_configuration():
    return {"param": [{"name": "p2", "value": 2}, {"name": "p6", "value": 6}]}


def test_generate_node(profile_filename, node_configuration, device_configuration):

    node = NodeLaunchFileProfile(
        profile_filename, node_configuration, device_configuration
    ).evaluate("node")

    assert "exec" in node
    assert "plugin" not in node
    assert node["param"][0]["name"] == "p1"
    assert node["param"][0]["value"] == 1
    assert node["param"][1]["name"] == "p2"
    assert node["param"][1]["value"] == 2
    assert node["param"][2]["name"] == "p3"
    assert node["param"][2]["value"] == 3
    assert node["param"][3]["name"] == "p4"
    assert node["param"][3]["value"] == 4
    assert node["param"][4]["name"] == "p5"
    assert node["param"][4]["value"] == 5
    assert node["param"][5]["name"] == "p6"
    assert node["param"][5]["value"] == 6
    assert node["param"][6]["name"] == "p7"
    assert node["param"][6]["value"] == 7


def test_generate_plugin(profile_filename, node_configuration, device_configuration):

    node = NodeLaunchFileProfile(
        profile_filename, node_configuration, device_configuration
    ).evaluate("plugin")

    assert "exec" not in node
    assert "plugin" in node
    assert node["param"][0]["name"] == "p1"
    assert node["param"][0]["value"] == 1
    assert node["param"][1]["name"] == "p2"
    assert node["param"][1]["value"] == 2
    assert node["param"][2]["name"] == "p3"
    assert node["param"][2]["value"] == 3
    assert node["param"][3]["name"] == "p4"
    assert node["param"][3]["value"] == 4
    assert node["param"][4]["name"] == "p5"
    assert node["param"][4]["value"] == 5
    assert node["param"][5]["name"] == "p6"
    assert node["param"][5]["value"] == 6
    assert node["param"][6]["name"] == "p7"
    assert node["param"][6]["value"] == 7


def test_check_failed_when_parameter_does_not_exists_in_device_configuration_(
    profile_filename, node_configuration, device_configuration
):

    del device_configuration["foo"]
    with pytest.raises(ValueError) as excinfo:
        NodeLaunchFileProfile(
            profile_filename, node_configuration, device_configuration
        ).evaluate("node")

    assert (
        "Parameter foo does not exists in device_configuration, unable to substitute parmeter"
        in str(excinfo.value)
    )


def test_check_failed_when_parameter_does_not_exists_in_driver_configuration(
    profile_filename, node_configuration, device_configuration
):

    del node_configuration["param"][1]
    with pytest.raises(ValueError) as excinfo:
        NodeLaunchFileProfile(
            profile_filename, node_configuration, device_configuration
        ).evaluate("node")

    assert "Parameter p6 does not exists in node_configuration, unable" in str(excinfo.value)


def test_check_failed_when_parameter_value_does_not_belong_in_choices(
    profile_filename, node_configuration, device_configuration
):

    node_configuration["param"][1]["value"] = 7
    with pytest.raises(ValueError) as excinfo:
        NodeLaunchFileProfile(
            profile_filename, node_configuration, device_configuration
        ).evaluate("node")
    assert "Parameter value 7 does not belong to choices [6, 10], unable to " in str(excinfo.value)
