from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'max_speed', default_value='4.0',
            description='Maximum commanded speed in m/s.'),
        DeclareLaunchArgument(
            'max_deceleration', default_value='2.5',
            description='Maximum allowed deceleration in m/s^2.'),
        DeclareLaunchArgument(
            'prefer_left_at_fork', default_value='true',
            description='Prefer the left viable branch.'),
        DeclareLaunchArgument(
            'left_branch_depth_ratio', default_value='0.65',
            description='Minimum left-gap depth relative to deepest gap.'),
        DeclareLaunchArgument(
            'minimum_gap_angle', default_value='0.12',
            description='Minimum navigable gap width in radians.'),
        Node(
            package='gap_follow',
            executable='reactive_node',
            name='reactive_follow_gap_node',
            output='screen',
            parameters=[{
                'max_speed': ParameterValue(
                    LaunchConfiguration('max_speed'), value_type=float),
                'max_deceleration': ParameterValue(
                    LaunchConfiguration('max_deceleration'), value_type=float),
                'prefer_left_at_fork': ParameterValue(
                    LaunchConfiguration('prefer_left_at_fork'),
                    value_type=bool),
                'left_branch_depth_ratio': ParameterValue(
                    LaunchConfiguration('left_branch_depth_ratio'),
                    value_type=float),
                'minimum_gap_angle': ParameterValue(
                    LaunchConfiguration('minimum_gap_angle'),
                    value_type=float),
            }],
        ),
    ])
