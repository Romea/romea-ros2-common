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
    is_parameter_can_be_evaluated,
    evaluate_parameter,
    # evaluate_parameter_from_list,
    # evaluate_parameter_from_range,
    # get_device_geometry,
    get_device_specifications,
)


def get_specifications():
    return get_device_specifications("common", "example_of", "device")


def test_check_parameter_can_be_evaluated():
    configuration = {"foo": "bar"}
    specifications = get_specifications()
    assert is_parameter_can_be_evaluated("common", specifications, configuration, "foo") is True

def test_check_parameter_can_be_evaluated_not_exist_in_configuration():
    configuration = {"bar": "foo"}
    specifications = get_specifications()
    with pytest.raises(LookupError) as excinfo:
        is_parameter_can_be_evaluated("common", specifications, configuration, "foo")

def test_check_parameter_can_be_evaluated_not_exist_in_specifications():
    configuration = {"bar": "foo"}
    specifications = get_specifications()
    with pytest.raises(LookupError) as excinfo:
        is_parameter_can_be_evaluated("common", specifications, configuration, "bar")


def test_evaluate_foo_parameter_ok():
    configuration = {"foo": "bar"}
    specifications = get_specifications()
    print(configuration)
    print(specifications)
    evaluate_parameter("common", specifications, configuration, "foo")


# def test_evaluate_foo_parameter_note_exist_in_configuration():
#     configuration = {'bar': 'foo'}
#     specifications = get_specifications()
#     print(configuration)
#     print(specifications)
#     evaluate_parameter('common',specifications, configuration, 'foo')


# def test_get_baz_foo(meta_description):
#     with pytest.raises(LookupError) as excinfo:
#         meta_description.get('baz', 'foo')
#     assert 'romea_common_bringup/test_common_bringup.yaml' in str(excinfo.value)
#     assert 'Cannot get param foo.baz' in str(excinfo.value)
