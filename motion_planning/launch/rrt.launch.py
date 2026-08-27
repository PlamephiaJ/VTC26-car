import os
from copy import deepcopy

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_config_path(package_share, config_path):
    if not os.path.isabs(config_path):
        config_path = os.path.join(package_share, 'config', config_path)
    return os.path.abspath(config_path)


def _load_config(config_path):
    if not os.path.isfile(config_path):
        raise RuntimeError(f'Configuration file does not exist: {config_path}')
    with open(config_path, 'r') as config_file:
        config = yaml.safe_load(config_file)
    if not isinstance(config, dict):
        raise RuntimeError(
            f'Configuration root must be a mapping in {config_path}.')
    return config


def _deep_merge(base, override):
    """Return base recursively updated by override without mutating either."""
    result = deepcopy(base)
    for key, value in override.items():
        if (key in result and isinstance(result[key], dict) and
                isinstance(value, dict)):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def _launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory('motion_planning')
    common_config_path = _resolve_config_path(
        package_share,
        LaunchConfiguration('common_config').perform(context))
    environment_config_path = _resolve_config_path(
        package_share, LaunchConfiguration('config').perform(context))
    common_config = _load_config(common_config_path)
    environment_config = _load_config(environment_config_path)
    config = _deep_merge(common_config, environment_config)
    waypoint_override = LaunchConfiguration('waypoint_file').perform(context)

    launch_config = config.get('launch', {})
    common_node_config = config.get(
        'rrt_node', {}).get('ros__parameters', {})
    if not isinstance(common_node_config, dict):
        raise RuntimeError(
            'Merged rrt_node.ros__parameters must be a mapping in '
            f'{environment_config_path}.')

    # A vehicles list starts one independently configured RRT process per car.
    # Keep accepting the original launch.namespace form for existing single-car
    # configuration files.
    vehicles = launch_config.get('vehicles')
    if vehicles is None:
        vehicles = [{
            'namespace': launch_config.get('namespace', 'ego_racecar'),
        }]
    if not isinstance(vehicles, list) or not vehicles:
        raise RuntimeError(
            'launch.vehicles must be a non-empty list in '
            f'{environment_config_path}.')
    for index, vehicle in enumerate(vehicles):
        if not isinstance(vehicle, dict):
            raise RuntimeError(
                f'launch.vehicles[{index}] must be a mapping in '
                f'{environment_config_path}.')

    enabled_vehicles = [
        vehicle for vehicle in vehicles
        if vehicle.get('enabled', True)
    ]
    vehicle_mode = launch_config.get('vehicle_mode')
    if vehicle_mode is not None:
        if (isinstance(vehicle_mode, bool) or
                not isinstance(vehicle_mode, int) or
                vehicle_mode not in (1, 2)):
            raise RuntimeError(
                f'launch.vehicle_mode must be the integer 1 or 2 in '
                f'{environment_config_path}.')
        if len(enabled_vehicles) < vehicle_mode:
            raise RuntimeError(
                f'launch.vehicle_mode is {vehicle_mode}, but only '
                f'{len(enabled_vehicles)} enabled vehicle configuration(s) '
                f'exist in {environment_config_path}.')
        vehicles = enabled_vehicles[:vehicle_mode]

    actions = []
    node_fully_qualified_names = set()
    for index, vehicle in enumerate(vehicles):
        if not isinstance(vehicle, dict):
            raise RuntimeError(
                f'launch.vehicles[{index}] must be a mapping in '
                f'{environment_config_path}.')
        if not vehicle.get('enabled', True):
            continue

        namespace = str(vehicle.get('namespace', '')).strip('/')
        node_name = str(vehicle.get('node_name', 'rrt_node')).strip('/')
        if not node_name:
            raise RuntimeError(
                f'launch.vehicles[{index}] needs a non-empty node_name in '
                f'{environment_config_path}.')

        instance_overrides = vehicle.get('ros__parameters', {})
        if not isinstance(instance_overrides, dict):
            raise RuntimeError(
                f'launch.vehicles[{index}].ros__parameters must be a mapping '
                f'in {environment_config_path}.')
        node_config = dict(common_node_config)
        node_config.update(instance_overrides)
        if waypoint_override:
            node_config['waypoint_file_path'] = waypoint_override

        waypoint_file = node_config.get('waypoint_file_path', '')
        if not waypoint_file:
            raise RuntimeError(
                f'waypoint_file_path is empty for vehicle '
                f'{namespace or "<root>"}. Set it in '
                f'{environment_config_path} or pass waypoint_file:=PATH.')
        if not os.path.isabs(waypoint_file):
            waypoint_file = os.path.join(
                os.path.dirname(environment_config_path), waypoint_file)
        waypoint_file = os.path.abspath(waypoint_file)
        if not os.path.isfile(waypoint_file):
            raise RuntimeError(
                f'Waypoint file for vehicle {namespace or "<root>"} does '
                f'not exist: {waypoint_file}. Override it with '
                f'waypoint_file:=PATH if this workspace uses another map.')
        node_config['waypoint_file_path'] = waypoint_file

        fully_qualified_name = (
            f'/{namespace}/{node_name}' if namespace else f'/{node_name}')
        if fully_qualified_name in node_fully_qualified_names:
            raise RuntimeError(
                f'Duplicate RRT node name {fully_qualified_name} in '
                f'{environment_config_path}.')
        node_fully_qualified_names.add(fully_qualified_name)

        actions.append(Node(
            package='motion_planning',
            executable='rrt_node_sim',
            name=node_name,
            namespace=namespace,
            output='screen',
            parameters=[node_config],
        ))

    if not actions:
        raise RuntimeError(
            'All entries in launch.vehicles are disabled in '
            f'{environment_config_path}.')
    return actions


def generate_launch_description():
    package_share = get_package_share_directory('motion_planning')

    return LaunchDescription([
        DeclareLaunchArgument(
            'common_config',
            default_value=os.path.join(
                package_share, 'config', 'rrt_common.yaml'),
            description=(
                'Shared planning configuration: an absolute path or a file '
                'in config/.'),
        ),
        DeclareLaunchArgument(
            'config',
            default_value=os.path.join(
                package_share, 'config', 'rrt_sim.yaml'),
            description=(
                'Environment configuration merged over common_config: an '
                'absolute path or a file in config/.'),
        ),
        DeclareLaunchArgument(
            'waypoint_file',
            default_value='',
            description=(
                'Optional absolute waypoint CSV override for every launched '
                'vehicle.'),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
