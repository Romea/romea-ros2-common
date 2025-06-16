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


import yaml
from romea_common_meta_bringup import LaunchFileGenerator


def test_get_complete_launch_file():

    device_configuration = {"foo": 2.0, "bar": "toto"}
    launch = [{"node": {"pkg": "pkg1", "exec": "exec1"}}]
    launch_arguments = [{"name": "mode", "default": "live"}]

    launch_file_generator = LaunchFileGenerator("common")
    launch_file = yaml.safe_load(launch_file_generator.generate(
          launch, launch_arguments, device_configuration, "robot", "ns"
        )
    )

    print("launch_file")
    print(launch_file)

    items = launch_file["launch"]
    assert "arg" in items[0]
    assert "mode" == items[0]["arg"]["name"]
    assert "group" in items[1]
    group_items = items[1]["group"]
    assert "push-ros-namespace" in group_items[0]
    assert "robot" == group_items[0]["push-ros-namespace"]["namespace"]
    assert "push-ros-namespace" in group_items[1]
    assert "ns" == group_items[1]["push-ros-namespace"]["namespace"]
    assert "let" in group_items[2]
    assert "foo" == group_items[2]["let"]["name"]
    assert "2.0" == group_items[2]["let"]["value"]
    assert "let" in group_items[3]
    assert "bar" == group_items[3]["let"]["name"]
    assert "toto" == group_items[3]["let"]["value"]
    assert "node" in group_items[4]
    assert "pkg" in group_items[4]["node"]
