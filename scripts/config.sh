#!/bin/bash
#
# Copyright 2024 The Cloud Robotics Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Escapes the input "foo bar" -> "foo\ bar".
function escape {
  sed 's/[^a-zA-Z0-9,._+@%/-]/\\&/g' <<< "$@"
}

# Escapes the input twice "foo bar" -> "foo\\\ bar"
function double_escape {
  sed 's/[^a-zA-Z0-9,._+@%/-]/\\\\\\&/g' <<< "$@"
}

# Creates a substitution pattern for sed using an unprintable char as seperator.
# This allows the user to use any normal char in the input.
function sed_pattern {
  local regexp="$1"
  local replacement="$2"
  echo s$'\001'${regexp}$'\001'${replacement}$'\001'
}

# Sets the given variable in config.sh. If $value is empty, the variable
# assignement is commented out in config.sh.
function save_variable {
  local config_file="$1"
  local name="$2"
  local value="$3"

  if [[ -z "${value}" ]]; then
    sed -i "s/^\(${name}=.*\)$/#\1/" "${config_file}"
  elif grep -q "^\(# *\)\{0,1\}${name}=" "${config_file}"; then
    value=$( double_escape ${value} )
    sed -i "$( sed_pattern "^\(# *\)\{0,1\}${name}=.*$" "${name}=${value}" )" "${config_file}"
  else
    value=$( escape ${value} )
    echo >>"${config_file}"
    echo "${name}=${value}" >>"${config_file}"
  fi
}
