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

import os
import re
import tempfile


def render_template_file(file_path, context):
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()

    def replace(match):
        key = match.group(1)

        if key not in context:
            raise KeyError(f"Missing template variable: '{key}'")

        return str(context[key])

    return re.sub(r"\$\(var\s+([a-zA-Z_][a-zA-Z0-9_]*)\)", replace, content)


def save_temporary_file(file_name=None, content=""):
    if file_name:
        file_name = os.path.basename(file_name)
        file_path = os.path.join("/tmp", file_name)

        with open(file_path, "w", encoding="utf-8") as f:
            f.write(content)

        return file_path

    # Si aucun nom fourni → fichier temporaire auto
    with tempfile.NamedTemporaryFile(
        mode="w",
        suffix=".yaml",
        delete=False,
        encoding="utf-8"
    ) as f:
        f.write(content)
        return f.name
