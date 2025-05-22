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
import yaml
from romea_common_meta_bringup import LaunchFileGenerator


def change_profile_value(d):
    for key, value in d.items():
        if key == "profile":
            d[key] = os.path.join(os.getcwd(), d[key])
        elif isinstance(value, dict):
            change_profile_value(value)
        elif isinstance(value, list):
            for item in value:
                if isinstance(item, dict):
                    change_profile_value(item)
    return d


@pytest.fixture
def launch_file():
    launch_file_filename = os.path.join(os.getcwd(), "test_launch_file.yaml")
    print("launch_file_filename", launch_file_filename)
    with open(launch_file_filename, 'r') as file:
        return change_profile_value(yaml.safe_load(file))


@pytest.fixture
def device_configuration():
    return {"foo": 3, "bar": 2}


def test_get_complete_launch_file(launch_file, device_configuration):

    print("launch file", launch_file)
    launch_file_generator = LaunchFileGenerator("common")
    launch_file = yaml.safe_load(launch_file_generator.generate(
         "all", launch_file["launch"], device_configuration, "robot", "ns"
        )
    )

    items = launch_file["launch"]
    assert "arg" in items[0]
    assert "mode" == items[0]["arg"]["name"]
    assert "arg" in items[1]
    assert "robot_namespace" == items[1]["arg"]["name"]
    assert "arg" in items[2]
    assert "common_namespace" == items[2]["arg"]["name"]
    assert "group" in items[3]
    group_items = items[3]["group"]
    assert "push-ros-namespace" in group_items[0]
    assert "$(var robot_namespace)" == group_items[0]["push-ros-namespace"]["namespace"]
    assert "push-ros-namespace" in group_items[1]
    assert "$(var common_namespace)" == group_items[1]["push-ros-namespace"]["namespace"]
    assert "node" in group_items[2]
    assert "pkg" in group_items[2]["node"]
    assert "node_container" in group_items[3]
    assert "pkg" in group_items[3]["node_container"]["composable_node"][0]
    assert "load_composable_node" in group_items[4]
    assert "pkg" in group_items[4]["load_composable_node"]["composable_node"][0]
