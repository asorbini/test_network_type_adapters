# Copyright 2021 Open Source Robotics Foundation, Inc.
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

from itertools import chain

import launch
from launch_ros.actions import Node

from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  share_dir = Path(get_package_share_directory("test_network_type_adapters"))
  config_file = str(share_dir / "connext" / "shape_transform.yml")
  print(f"CONFIG_FILE: {config_file}")
  return launch.LaunchDescription([
    Node(
      name=f"shape_talker",
      namespace='',
      package='test_network_type_adapters',
      executable='shape_talker',
      output='both',
      additional_env={
        "RMW_CONNEXT_CONFIG" : config_file
      },
    ),
    # Node(
    #   name=f"listener",
    #   namespace='',
    #   package='demo_nodes_cpp',
    #   executable='listener',
    #   output='both',
    #   additional_env={
    #     "RMW_CONNEXT_CONFIG" : str(share_dir / "shape_transform.yml")
    #   },
    # ),
  ])
