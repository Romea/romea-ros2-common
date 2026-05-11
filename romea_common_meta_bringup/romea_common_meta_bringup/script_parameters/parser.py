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

import argparse
import re
from typing import Optional, Sequence

import argcomplete

from .descriptions import ScriptParameterDescription
from .profiles import ScriptParameterProfiles


class CompactHelpFormatter(argparse.RawTextHelpFormatter):
    def __init__(self, *args, **kwargs):
        kwargs["max_help_position"] = 4
        kwargs["width"] = 100
        super().__init__(*args, **kwargs)


class ScriptParametersCompleter:
    def __init__(
        self,
        parameters: list[ScriptParameterDescription],
        argument_name: str = "parameters",
        separator: str = ":=",
    ):
        self._parameters = parameters
        self._argument_name = argument_name
        self._separator = separator
        self._known_params = {parameter.name: parameter.to_dict() for parameter in parameters}

    def __call__(self, prefix, parsed_args, **kwargs):
        results = []

        used_args = getattr(parsed_args, self._argument_name, []) or []
        used_keys = {
            arg.split(self._separator, 1)[0] for arg in used_args if self._separator in arg
        }

        if self._separator in prefix:
            key, _ = prefix.split(self._separator, 1)
            parameter = self._known_params.get(key)
            values = parameter.get("values") if parameter else None

            if values:
                for value in values:
                    candidate = f"{key}{self._separator}{value}"
                    if candidate.startswith(prefix):
                        results.append(candidate)

            return results

        for parameter in self._parameters:
            if parameter.name in used_keys:
                continue

            candidate = f"{parameter.name}{self._separator}"
            if candidate.startswith(prefix):
                results.append(candidate)

        return results

    @classmethod
    def build_parser(
        cls,
        parameters: list[ScriptParameterDescription],
        argument_name: str = "parameters",
        separator: str = ":=",
        prog: Optional[str] = None,
        description: Optional[str] = None,
        epilog: Optional[str] = None,
    ) -> argparse.ArgumentParser:
        parser = argparse.ArgumentParser(
            prog=prog,
            description=description,
            epilog=epilog,
            formatter_class=CompactHelpFormatter,
        )

        help_lines = [f"<key>{separator}<value>", ""]

        for parameter in parameters:
            help_lines.append(f"  - {parameter.name}")

            if parameter.description:
                help_lines.append(f"    Description: {parameter.description}")

            if parameter.values:
                help_lines.append(f"    Choices:     {' | '.join(parameter.values)}")

            if parameter.default is not None:
                help_lines.append(f"    Default:     {parameter.default}")

            help_lines.append("")

        arg = parser.add_argument(
            argument_name,
            nargs="*",
            help="\n".join(help_lines),
        )
        arg.completer = cls(parameters, argument_name, separator)

        argcomplete.autocomplete(parser)
        return parser


class ScriptParameters:
    def __init__(
        self,
        argv: list[str],
        descriptions: Optional[list[ScriptParameterDescription]] = None,
        separators: Sequence[str] = (":=", ":"),
    ):
        if not separators:
            raise ValueError("At least one separator must be provided.")

        self._separators = tuple(separators)
        self._params: dict[str, str] = {}
        self._descriptions = {
            description.name: description for description in (descriptions or [])
        }

        for argument in argv:
            name, value = self._split_argument(argument)

            if not name:
                raise ValueError(f"Invalid argument '{argument}'. Argument name cannot be empty.")

            if not value:
                raise ValueError(f"Invalid argument '{argument}'. Argument value cannot be empty.")

            self._params[name] = value

        self._apply_defaults()
        self._validate_values()

    def _split_argument(self, argument: str) -> tuple[str, str]:
        if re.search(r"\s", argument):
            raise ValueError(f"Invalid argument '{argument}'. Spaces are not allowed.")

        for separator in self._separators:
            if separator in argument:
                name, value = argument.split(separator, 1)
                return name, value

        expected_formats = " or ".join(f"name{separator}value" for separator in self._separators)
        raise ValueError(f"Invalid argument '{argument}'. Expected format: {expected_formats}")

    def _apply_defaults(self):
        for name, description in self._descriptions.items():
            if name in self._params:
                continue

            if description.default is not None:
                self._params[name] = description.default

    def _validate_values(self):
        for name, value in self._params.items():
            description = self._descriptions.get(name)

            if description and description.values and value not in description.values:
                allowed = " | ".join(description.values)
                raise ValueError(
                    f"Invalid value for parameter '{name}': '{value}'. "
                    f"Allowed values: {allowed}"
                )

    def pop(self, key: str, default=None):
        return self._params.pop(key, default)

    def pop_str(
        self,
        key: str,
        default: Optional[str] = None,
        *,
        required: bool = False,
    ) -> Optional[str]:
        value = self._params.pop(key, default)

        if value is None:
            if required:
                raise ValueError(f"Missing required parameter: {key}")
            return default

        if value == "":
            raise ValueError(f"Parameter '{key}' cannot be empty.")

        return value

    def pop_bool(
        self,
        key: str,
        default: Optional[str] = None,
        *,
        required: bool = False,
    ) -> Optional[str]:
        value = self.pop_str(key, default, required=required)

        if value is None:
            return default

        v = value.lower()

        if v in {"true", "1", "yes", "on"}:
            return "true"

        if v in {"false", "0", "no", "off"}:
            return "false"

        raise ValueError(f"Invalid boolean value for '{key}': '{value}'")

    def remaining(self) -> dict[str, str]:
        return dict(self._params)

    @classmethod
    def from_cli(
        cls,
        descriptions: list[ScriptParameterDescription],
        argument_name: str = "parameters",
        separator: str = ":=",
        **parser_kwargs,
    ) -> "ScriptParameters":
        parser = ScriptParametersCompleter.build_parser(
            parameters=descriptions,
            argument_name=argument_name,
            separator=separator,
            **parser_kwargs,
        )

        parsed_args = parser.parse_args()
        argv = getattr(parsed_args, argument_name)

        return cls(
            argv=argv,
            descriptions=descriptions,
            separators=(separator,),
        )


def configuration_file_generation_parameters_from_cli(device_type: str) -> ScriptParameters:
    return ScriptParameters.from_cli(
        ScriptParameterProfiles.for_configuration_file_generation(device_type),
        prog=f"generate-{device_type}-configuration-file",
        description=f"Generate {device_type} YAML configuration file.",
    )


def controllers_configuration_file_generation_parameters_from_cli(
    device_type: str,
) -> ScriptParameters:
    return ScriptParameters.from_cli(
        ScriptParameterProfiles.for_controllers_configuration_file_generation(device_type),
        prog=f"generate-{device_type}-controllers-configuration-file",
        description=f"Generate {device_type} YAML controllers configuration file.",
    )


def launch_file_generation_parameters_from_cli(device_type: str) -> ScriptParameters:
    return ScriptParameters.from_cli(
        ScriptParameterProfiles.for_launch_file_generation(device_type),
        prog=f"generate-{device_type}-launch-file",
        description=f"Generate {device_type} YAML launch file.",
    )


def robot_urdf_description_generation_parameters_from_cli(device_type: str) -> ScriptParameters:
    return ScriptParameters.from_cli(
        ScriptParameterProfiles.for_robot_urdf_description_generation(device_type),
        prog=f"generate-{device_type}-urdf-description",
        description=f"Generate {device_type} URDF description.",
    )


def device_urdf_description_generation_parameters_from_cli(device_type: str) -> ScriptParameters:
    return ScriptParameters.from_cli(
        ScriptParameterProfiles.for_device_urdf_description_generation(device_type),
        prog=f"generate-{device_type}-urdf-description",
        description=f"Generate {device_type} URDF description.",
    )
