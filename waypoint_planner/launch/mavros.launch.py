"""MAVROS for the UAV, including isolation of its internal MAVLink ROS bus."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _start_mavros(context):
    robot_ns = '/' + LaunchConfiguration('robot_ns').perform(context).strip('/')
    if robot_ns == '/':
        raise ValueError('robot_ns must name a robot to isolate its MAVLink bus')
    ids = {name: int(LaunchConfiguration(name).perform(context))
           for name in ('system_id', 'tgt_system', 'tgt_component')}
    if any(not 1 <= value <= 255 for value in ids.values()):
        raise ValueError('MAVLink system/component IDs must be between 1 and 255')
    # mavros_node hardcodes /uas<tgt_system>, independent of its ROS namespace.
    # Both the Router and UAS inherit these remappings from the same process.
    bus = '/uas' + str(ids['tgt_system'])
    isolated_bus = robot_ns + '/mavros/internal' + bus
    return [Node(
        package='mavros', executable='mavros_node', output='screen',
        namespace=robot_ns + '/mavros',
        parameters=[LaunchConfiguration('pluginlists_yaml'), LaunchConfiguration('config_yaml'), {
            'fcu_url': LaunchConfiguration('fcu_url'),
            **ids,
            # UAS reads these names; the all-in-one launcher reads tgt_* above.
            'target_system_id': ids['tgt_system'],
            'target_component_id': ids['tgt_component'],
        }],
        remappings=[(bus + '/' + topic, isolated_bus + '/' + topic)
                    for topic in ('mavlink_source', 'mavlink_sink')],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', default_value='uav'),
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyUSB0:921600'),
        DeclareLaunchArgument('system_id', default_value='21', description='MAVROS sender system ID'),
        DeclareLaunchArgument('tgt_system', default_value='1', description='Actual FCU MAVLink system ID'),
        DeclareLaunchArgument('tgt_component', default_value='1', description='Actual FCU component ID'),
        DeclareLaunchArgument('pluginlists_yaml', default_value=PathJoinSubstitution([
            FindPackageShare('waypoint_planner'), 'px4_pluginlists.yaml'])),
        DeclareLaunchArgument('config_yaml', default_value=PathJoinSubstitution([
            FindPackageShare('waypoint_planner'), 'px4_config.yaml'])),
        OpaqueFunction(function=_start_mavros),
    ])
