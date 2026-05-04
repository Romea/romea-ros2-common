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

from argparse import Namespace

import pytest

from romea_common_meta_bringup.script_parameters import (
    ScriptParameterDescription,
    ScriptParameters,
    ScriptParametersCompleter,
)


@pytest.mark.parametrize(
    "argument, key, expected",
    [
        ("foo:bar", "foo", "bar"),
        ("foo:=bar", "foo", "bar"),
    ],
)
def test_accepts_valid_string_arguments(argument, key, expected):
    params = ScriptParameters([argument])
    assert params.pop_str(key) == expected


@pytest.mark.parametrize("argument", ["foobar", "foo=bar", "foo->bar"])
def test_rejects_invalid_format(argument):
    with pytest.raises(ValueError, match="Expected format"):
        ScriptParameters([argument])


@pytest.mark.parametrize(
    "argument",
    [
        "foo: bar",
        "foo :bar",
        "foo :=bar",
        "foo:= bar",
        "foo : bar",
        "foo\t:bar",
        "foo:\tbar",
    ],
)
def test_rejects_arguments_with_spaces(argument):
    with pytest.raises(ValueError, match="Spaces are not allowed"):
        ScriptParameters([argument])


@pytest.mark.parametrize("argument", [":bar", ":=bar"])
def test_rejects_empty_name(argument):
    with pytest.raises(ValueError, match="Argument name cannot be empty"):
        ScriptParameters([argument])


@pytest.mark.parametrize("argument", ["foo:", "foo:="])
def test_rejects_empty_value(argument):
    with pytest.raises(ValueError, match="Argument value cannot be empty"):
        ScriptParameters([argument])


def test_pop_str_returns_existing_value():
    params = ScriptParameters(["foo:=bar"])
    assert params.pop_str("foo") == "bar"


def test_pop_str_returns_default_for_missing_key():
    params = ScriptParameters([])
    assert params.pop_str("foo", "bar") == "bar"


def test_pop_str_returns_none_for_missing_key_without_default():
    params = ScriptParameters([])
    assert params.pop_str("foo") is None


def test_pop_str_accepts_existing_required_key():
    params = ScriptParameters(["foo:=bar"])
    assert params.pop_str("foo", required=True) == "bar"


def test_pop_str_rejects_missing_required_key():
    params = ScriptParameters([])

    with pytest.raises(ValueError, match="Missing required parameter: foo"):
        params.pop_str("foo", required=True)


def test_pop_str_does_not_reject_missing_optional_key():
    params = ScriptParameters([])
    assert params.pop_str("foo", required=False) is None


@pytest.mark.parametrize(
    "argument",
    ["flag:=true", "flag:=True", "flag:=1", "flag:=yes", "flag:=on"],
)
def test_pop_bool_normalizes_true_values(argument):
    params = ScriptParameters([argument])
    assert params.pop_bool("flag") == "true"


@pytest.mark.parametrize(
    "argument",
    ["flag:=false", "flag:=False", "flag:=0", "flag:=no", "flag:=off"],
)
def test_pop_bool_normalizes_false_values(argument):
    params = ScriptParameters([argument])
    assert params.pop_bool("flag") == "false"


def test_pop_bool_returns_default_for_missing_key():
    params = ScriptParameters([])
    assert params.pop_bool("flag", "true") == "true"


def test_pop_bool_accepts_existing_required_key():
    params = ScriptParameters(["flag:=true"])
    assert params.pop_bool("flag", required=True) == "true"


def test_pop_bool_rejects_missing_required_key():
    params = ScriptParameters([])

    with pytest.raises(ValueError, match="Missing required parameter: flag"):
        params.pop_bool("flag", required=True)


def test_pop_bool_rejects_invalid_value():
    params = ScriptParameters(["flag:maybe"])

    with pytest.raises(
        ValueError,
        match=r"Invalid boolean value for 'flag': 'maybe'",
    ):
        params.pop_bool("flag")


def test_conditional_required_parameter_can_be_managed_by_user_code():
    params = ScriptParameters(["foo:=true"])

    foo = params.pop_bool("foo", default="true")

    with pytest.raises(
        ValueError,
        match="Missing required parameter: bar",
    ):
        params.pop_str(
            "bar",
            required=foo == "true",
        )


def test_conditional_required_parameter_can_be_optional_by_user_code():
    params = ScriptParameters(["foo:=false"])

    foo = params.pop_bool("foo", default="true")

    assert (
        params.pop_str(
            "bar",
            required=foo == "true",
        )
        is None
    )


def test_remaining_returns_unconsumed_parameters():
    params = ScriptParameters(["foo:bar", "baz:qux", "quux:corge"])

    assert params.pop_str("foo") == "bar"

    assert params.remaining() == {
        "baz": "qux",
        "quux": "corge",
    }


def test_remaining_returns_a_copy():
    params = ScriptParameters(["foo:bar"])

    remaining = params.remaining()
    remaining["foo"] = "changed"

    assert params.pop_str("foo") == "bar"


def test_duplicate_key_keeps_last_value():
    params = ScriptParameters(["foo:bar", "foo:baz"])
    assert params.pop_str("foo") == "baz"


@pytest.mark.parametrize(
    "argument, key, expected",
    [
        ("foo:bar:baz", "foo", "bar:baz"),
        ("foo:=bar:=baz", "foo", "bar:=baz"),
    ],
)
def test_value_can_contain_separator_after_first_split(argument, key, expected):
    params = ScriptParameters([argument])
    assert params.pop_str(key) == expected


def test_completer_suggests_parameter_names():
    completer = ScriptParametersCompleter(
        [
            ScriptParameterDescription("foo"),
            ScriptParameterDescription("bar"),
        ]
    )

    assert completer("", Namespace(parameters=[])) == ["foo:=", "bar:="]


def test_completer_filters_parameter_names_from_prefix():
    completer = ScriptParametersCompleter(
        [
            ScriptParameterDescription("foo"),
            ScriptParameterDescription("bar"),
        ]
    )

    assert completer("f", Namespace(parameters=[])) == ["foo:="]


def test_completer_does_not_suggest_already_used_parameters():
    completer = ScriptParametersCompleter(
        [
            ScriptParameterDescription("foo"),
            ScriptParameterDescription("bar"),
        ]
    )

    assert completer("", Namespace(parameters=["foo:=value"])) == ["bar:="]


def test_completer_suggests_values_for_known_parameter():
    completer = ScriptParametersCompleter(
        [
            ScriptParameterDescription("foo", values=["true", "false"]),
        ]
    )

    assert completer("foo:=", Namespace(parameters=[])) == [
        "foo:=true",
        "foo:=false",
    ]


def test_completer_filters_values_from_prefix():
    completer = ScriptParametersCompleter(
        [
            ScriptParameterDescription("foo", values=["true", "false"]),
        ]
    )

    assert completer("foo:=t", Namespace(parameters=[])) == ["foo:=true"]


def test_completer_returns_empty_for_unknown_parameter_value_completion():
    completer = ScriptParametersCompleter(
        [
            ScriptParameterDescription("foo", values=["true", "false"]),
        ]
    )

    assert completer("bar:=", Namespace(parameters=[])) == []


def test_completer_supports_custom_argument_name():
    completer = ScriptParametersCompleter(
        [ScriptParameterDescription("foo")],
        argument_name="custom_parameters",
    )

    assert completer("", Namespace(custom_parameters=[])) == ["foo:="]


def test_completer_supports_custom_separator():
    completer = ScriptParametersCompleter(
        [ScriptParameterDescription("foo", values=["bar"])],
        separator=":",
    )

    assert completer("foo:", Namespace(parameters=[])) == ["foo:bar"]
