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
from ament_index_python import get_package_share_directory


from romea_common_description import (
    get_device_specifications,
    get_device_geometry,
    evaluate_parameter,
    evaluate_parameter_from_list,
    evaluate_parameter_from_range,
)

def get_specifications():
    return get_device_specifications('common', 'example_of', 'device')
  

def test_evaluate_foo_parameter_ok():
    configuration = {'foo': 'bar'}
    specifications = get_specifications()
    assert evaluate_parameter(specifications, configuration, 'foo') == 'bar'

def test_evaluate_foo_parameter_failed():
    configuration = {'foo': 'thud'}
    specifications = get_specifications()
    # with pytest.raises(LookupError) as excinfo:
    assert evaluate_parameter(specifications, configuration, 'foo') == 'bar'
    # assert 

# def test_get_baz_foo(meta_description):
#     with pytest.raises(LookupError) as excinfo:
#         meta_description.get("baz", "foo")
#     assert "romea_common_bringup/test_common_bringup.yaml" in str(excinfo.value)
#     assert "Cannot get param foo.baz" in str(excinfo.value)


# def test_robot_namespace():
#     assert robot_namespace("") == "/"
#     assert robot_namespace("robot") == "/robot"


# def test_robot_prefix():
#     assert robot_prefix("") == "/"
#     assert robot_prefix("robot") == "/robot/"


# def test_device_namespace():
#     assert device_namespace("robot", None, "device") == "/robot/device"
#     assert device_namespace("robot", "ns", "device") == "/robot/ns/device"


# def test_device_prefix():
#     assert device_prefix("robot", None, "device") == "/robot/device/"
#     assert device_prefix("robot", "ns", "device") == "/robot/ns/device/"


# def test_robot_urdf_prefix():
#     assert robot_urdf_prefix("") == ""
#     assert robot_urdf_prefix("robot") == "robot_"


# def test_device_urdf_prefix():
#     assert device_urdf_prefix("", "") == ""
#     assert device_urdf_prefix("robot", "") == "robot_"
#     assert device_urdf_prefix("robot", "device") == "robot_device_"


# def test_device_link_name():
#     assert device_link_name("", "device") == "device_link"
#     assert device_link_name("robot", "device") == "robot_device_link"


# @pytest.fixture(scope="module")
# def meta_description():
#     meta_description_file_path = os.path.join(os.getcwd(), "test_common_bringup.yaml")
#     return MetaDescription("common", meta_description_file_path)


# def test_foo_exists(meta_description):
#     assert meta_description.exists("foo") is True


# def test_qux_baz_exists(meta_description):
#     assert meta_description.exists("qux", "baz") is True


# def test_bar_does_not_exists(meta_description):
#     assert meta_description.exists("bar") is False


# def test_foo_thud_does_not_exists(meta_description):
#     assert meta_description.exists("foo", "thud") is False


# def test_baz_foo_does_not_exists(meta_description):
#     assert meta_description.exists("baz", "foo") is False


# def test_get_or_foo(meta_description):
#     assert meta_description.get_or("foo") == "bar"


# def test_get_or_qux_baz(meta_description):
#     assert meta_description.get_or("qux", "baz") == "thud"


# def test_get_or_bar(meta_description):
#     assert meta_description.get_or("bar") is None


# def test_get_or_foo_baz(meta_description):
#     assert meta_description.get_or("foo", "baz") is None


# def test_get_or_foo_thud(meta_description):
#     with pytest.raises(LookupError) as excinfo:
#         meta_description.get_or("foo", "thud")
#     assert "romea_common_bringup/test_common_bringup.yaml" in str(excinfo.value)
#     assert "Cannot get param thud.foo" in str(excinfo.value)


# def test_get_or_baz_foo(meta_description):
#     with pytest.raises(LookupError) as excinfo:
#         meta_description.get_or("baz", "foo")
#     assert "romea_common_bringup/test_common_bringup.yaml" in str(excinfo.value)
#     assert "Cannot get param foo.baz" in str(excinfo.value)


# def test_get_foo(meta_description):
#     assert meta_description.get("foo") == "bar"


# def test_get_qux_baz(meta_description):
#     assert meta_description.get("qux", "baz") == "thud"


# def test_get_bar(meta_description):
#     with pytest.raises(LookupError) as excinfo:
#         assert meta_description.get("bar")
#     assert "romea_common_bringup/test_common_bringup.yaml" in str(excinfo.value)
#     assert "Cannot get param bar" in str(excinfo.value)


# def test_get_foo_baz(meta_description):
#     with pytest.raises(LookupError) as excinfo:
#         assert meta_description.get("foo", "baz")
#     assert "romea_common_bringup/test_common_bringup.yaml" in str(excinfo.value)
#     assert "Cannot get param baz.foo" in str(excinfo.value)


# def test_get_foo_thud(meta_description):
#     with pytest.raises(LookupError) as excinfo:
#         meta_description.get_or("foo", "thud")
#     assert "romea_common_bringup/test_common_bringup.yaml" in str(excinfo.value)
#     assert "Cannot get param thud.foo" in str(excinfo.value)


# def test_get_baz_foo(meta_description):
#     with pytest.raises(LookupError) as excinfo:
#         meta_description.get("baz", "foo")
#     assert "romea_common_bringup/test_common_bringup.yaml" in str(excinfo.value)
#     assert "Cannot get param foo.baz" in str(excinfo.value)


# def test_get_file(meta_description):
#     file_path = get_file_path(meta_description.get("file"))
#     assert file_path == get_package_share_directory("romea_common_bringup")+"/config/foo.yaml"
