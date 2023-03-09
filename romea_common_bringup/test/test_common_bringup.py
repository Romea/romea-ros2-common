# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

import os
import pytest

from romea_common_bringup import (
    robot_prefix,
    robot_namespace,
    device_prefix,
    device_namespace,
    robot_urdf_prefix,
    device_urdf_prefix,
    device_link_name,
    MetaDescription,
)


def test_robot_namespace():
    assert robot_namespace("") == "/"
    assert robot_namespace("robot") == "/robot"


def test_robot_prefix():
    assert robot_prefix("") == "/"
    assert robot_prefix("robot") == "/robot/"


def test_device_namespace():
    assert device_namespace("robot", None, "device") == "/robot/device"
    assert device_namespace("robot", "ns", "device") == "/robot/ns/device"


def test_device_prefix():
    assert device_prefix("robot", None, "device") == "/robot/device/"
    assert device_prefix("robot", "ns", "device") == "/robot/ns/device/"


def test_robot_urdf_prefix():
    assert robot_urdf_prefix("") == ""
    assert robot_urdf_prefix("robot") == "robot_"


def test_device_urdf_prefix():
    assert device_urdf_prefix("", "") == ""
    assert device_urdf_prefix("robot", "") == "robot_"
    assert device_urdf_prefix("robot", "device") == "robot_device_"


def test_device_link_name():
    assert device_link_name("", "device") == "device_link"
    assert device_link_name("robot", "device") == "robot_device_link"


@pytest.fixture(scope="module")
def meta_description():
    meta_description_filename = os.path.join(os.getcwd(), "test_common_bringup.yaml")
    return MetaDescription("common", meta_description_filename)


def test_foo_exists(meta_description):
    assert meta_description.exists("foo") is True


def test_qux_baz_exists(meta_description):
    assert meta_description.exists("qux", "baz") is True


def test_bar_does_not_exists(meta_description):
    assert meta_description.exists("bar") is False


def test_foo_thud_does_not_exists(meta_description):
    assert meta_description.exists("foo", "thud") is False


def test_baz_foo_does_not_exists(meta_description):
    assert meta_description.exists("baz", "foo") is False


def test_get_or_foo(meta_description):
    assert meta_description.get_or("foo") == "bar"


def test_get_or_qux_baz(meta_description):
    assert meta_description.get_or("qux", "baz") == "thud"


def test_get_or_bar(meta_description):
    assert meta_description.get_or("bar") is None


def test_get_or_foo_baz(meta_description):
    assert meta_description.get_or("foo", "baz") is None


def test_get_or_foo_thud(meta_description):
    with pytest.raises(LookupError) as excinfo:
        meta_description.get_or("foo", "thud")
    assert "romea_common_bringup/test_common_bringup.yaml" in str(excinfo.value)
    assert "Cannot get param thud.foo" in str(excinfo.value)


def test_get_or_baz_foo(meta_description):
    with pytest.raises(LookupError) as excinfo:
        meta_description.get_or("baz", "foo")
    assert "romea_common_bringup/test_common_bringup.yaml" in str(excinfo.value)
    assert "Cannot get param foo.baz" in str(excinfo.value)


def test_get_foo(meta_description):
    assert meta_description.get("foo") == "bar"


def test_get_qux_baz(meta_description):
    assert meta_description.get("qux", "baz") == "thud"


def test_get_bar(meta_description):
    with pytest.raises(LookupError) as excinfo:
        assert meta_description.get("bar")
    assert "romea_common_bringup/test_common_bringup.yaml" in str(excinfo.value)
    assert "Cannot get param bar" in str(excinfo.value)


def test_get_foo_baz(meta_description):
    with pytest.raises(LookupError) as excinfo:
        assert meta_description.get("foo", "baz")
    assert "romea_common_bringup/test_common_bringup.yaml" in str(excinfo.value)
    assert "Cannot get param baz.foo" in str(excinfo.value)


def test_get_foo_thud(meta_description):
    with pytest.raises(LookupError) as excinfo:
        meta_description.get_or("foo", "thud")
    assert "romea_common_bringup/test_common_bringup.yaml" in str(excinfo.value)
    assert "Cannot get param thud.foo" in str(excinfo.value)


def test_get_baz_foo(meta_description):
    with pytest.raises(LookupError) as excinfo:
        meta_description.get("baz", "foo")
    assert "romea_common_bringup/test_common_bringup.yaml" in str(excinfo.value)
    assert "Cannot get param foo.baz" in str(excinfo.value)
