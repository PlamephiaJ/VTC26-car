# Copyright (c) 2026 Yuhao Chen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('motion_planning')
    common_config = os.path.join(
        package_share, 'config', 'rrt_common.yaml')
    real_config = os.path.join(package_share, 'config', 'rrt_real.yaml')
    generic_launch = os.path.join(package_share, 'launch', 'rrt.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'common_config',
            default_value=common_config,
            description='Shared planning configuration.'),
        DeclareLaunchArgument(
            'config',
            default_value=real_config,
            description='Real-car environment configuration.'),
        DeclareLaunchArgument(
            'waypoint_file',
            default_value='',
            description='Required real-car waypoint CSV.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(generic_launch),
            launch_arguments={
                'common_config': LaunchConfiguration('common_config'),
                'config': LaunchConfiguration('config'),
                'waypoint_file': LaunchConfiguration('waypoint_file'),
            }.items()),
    ])
