from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    yaml_filename = LaunchConfiguration('yaml_filename')
    battery_threshold = LaunchConfiguration('battery_threshold')
    memlog_src = LaunchConfiguration('memlog_src')
    log_src = LaunchConfiguration('log_src')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace to apply to node(s)'
        ),
        DeclareLaunchArgument(
            'use_namespace',
            default_value='false',
            description='Whether to apply the namespace'
        ),
        
        DeclareLaunchArgument(
            'yaml_filename',
            default_value='/sdcard/mmcblk2p1/soundarya/FILES_OF_ROVER/CORRIDOR/corridor_waypoints/corridor_starting_pt.yaml',
            description='Waypoint YAML file - for rover to be placed in the starting place correctly in case of low charge'
        ),
        
        DeclareLaunchArgument(
            'memlog_src',
            default_value='/sdcard/mmcblk2p1/soundarya/FILES_OF_ROVER/rover4_memlog.txt',
            description='Source path for rover memlog to backup'
        ),
        
        DeclareLaunchArgument(
            'log_src',
            default_value='/sdcard/mmcblk2p1/soundarya/FILES_OF_ROVER/rover4_loggg.txt',
            description='Source path for rover log to backup'
        ),
        
        DeclareLaunchArgument(
            'battery_threshold',
            default_value='0.2',
            description='Battery percentage threshold to trigger navigation (0.0 to 1.0)'
        ),

        GroupAction([
            PushRosNamespace(
                condition=IfCondition(use_namespace),
                namespace=namespace
            ),
            Node(
                package='econlidar',
                executable='battery_monitor_and_navigate_docking',
                name='battery_monitor_docking',
                parameters=[{
                    'yaml_filename': yaml_filename,
                    'battery_threshold': battery_threshold,
                    'memlog_src': LaunchConfiguration('memlog_src'),
                    'log_src': LaunchConfiguration('log_src')
                }]
            )
        ])
    ])
