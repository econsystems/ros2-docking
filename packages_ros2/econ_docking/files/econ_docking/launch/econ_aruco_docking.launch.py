import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    aruco_calib = LaunchConfiguration('aruco_calib')
    show_image = LaunchConfiguration('show_image')
    image_topic = LaunchConfiguration('image_topic')
    aruco_dict = LaunchConfiguration('aruco_dictionary_name')
    aruco_size = LaunchConfiguration('aruco_marker_side_length')
    target_marker_id = LaunchConfiguration('target_marker_id')
    process_every_nth_frame = LaunchConfiguration('process_every_nth_frame')
    battery_max_threshold = LaunchConfiguration('battery_max_threshold')
    battery_min_threshold = LaunchConfiguration('battery_min_threshold')
    undocking_distance = LaunchConfiguration('undocking_distance')
    approach_distance = LaunchConfiguration('approach_distance')
    offset_search_tolerance = LaunchConfiguration('offset_search_tolerance')
    angular_velocity_search = LaunchConfiguration('angular_velocity_search')

    default_calib = PathJoinSubstitution([FindPackageShare('econ_docking'), 'config', 'calibration_chessboard.yaml'])
    default_staging_yaml = PathJoinSubstitution([FindPackageShare('econ_docking'), 'config', 'staging_waypoints.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=TextSubstitution(text=''), description='Top-level namespace to apply to node(s)'),
        DeclareLaunchArgument('use_namespace', default_value=TextSubstitution(text='false'), description='Whether to apply the namespace (true/false)'),
        DeclareLaunchArgument('aruco_calib', default_value=default_calib, description='Path to camera calibration YAML for ArUco'),
        DeclareLaunchArgument('show_image', default_value=TextSubstitution(text='false'), description='Whether to show debug GUI window'),
        DeclareLaunchArgument('image_topic', default_value=TextSubstitution(text='rgb/image_raw'), description='Camera image topic'),
        DeclareLaunchArgument('target_marker_id', default_value=TextSubstitution(text='0'), description='Target ArUco marker ID'),
        DeclareLaunchArgument('process_every_nth_frame', default_value=TextSubstitution(text='4'), description='Process every Nth frame to reduce CPU'),
        DeclareLaunchArgument('aruco_dictionary_name', default_value=TextSubstitution(text='DICT_APRILTAG_36h11'), description='ArUco dictionary'),
        DeclareLaunchArgument('aruco_marker_side_length', default_value=TextSubstitution(text='0.09'), description='ArUco marker side length (meters)'),
        DeclareLaunchArgument('staging_waypoints_yaml', default_value=default_staging_yaml, description='YAML file with staging/docking waypoints'),
        DeclareLaunchArgument('battery_max_threshold', default_value=TextSubstitution(text='0.93'), description='Max battery threshold to stop docking'),
        DeclareLaunchArgument('battery_min_threshold', default_value=TextSubstitution(text='0.27'), description='Min battery threshold to start docking'),
        DeclareLaunchArgument('undocking_distance', default_value=TextSubstitution(text='0.40'), description='Backup distance during undocking (meters)'),
        DeclareLaunchArgument('approach_distance', default_value=TextSubstitution(text='0.25'), description='Approach distance during docking (meters)'),
        DeclareLaunchArgument('offset_search_tolerance', default_value=TextSubstitution(text='50'), description='Offset search tolerance (pixels)'),
        DeclareLaunchArgument('angular_velocity_search', default_value=TextSubstitution(text='0.32'), description='Angular velocity during search (rad/s)'),

        GroupAction([
            # push namespace only if use_namespace is true
            PushRosNamespace(namespace=namespace, condition=IfCondition(use_namespace)),

            Node(
                package='econ_docking',
                executable='aruco_marker_detector',
                name='aruco_marker_detector',
                output='screen',
                parameters=[{
                    'aruco_dictionary_name': aruco_dict,
                    'aruco_marker_side_length': aruco_size,
                    'camera_calibration_parameters_filename': aruco_calib,
                    'image_topic': image_topic,
                    'target_marker_id': target_marker_id,
                    'process_every_nth_frame': process_every_nth_frame,
                    'aruco_marker_name': 'aruco_marker',
                    # 'show_image': show_image,
                }]
            ),

            # Node(
            #     package='econ_docking',
            #     executable='aruco_marker_pose_estimation_tf',
            #     name='aruco_marker_pose_estimation_tf',
            #     output='screen',
            #     # remap the global /tf and /tf_static into the (namespaced) tf topics
            #     remappings=[
            #         ('/tf', 'tf'),
            #         ('/tf_static', 'tf_static'),
            #     ],
            #     parameters=[{
            #         'aruco_dictionary_name': aruco_dict,
            #         'aruco_marker_side_length': aruco_size,
            #         'camera_calibration_parameters_filename': aruco_calib,
            #         'image_topic': image_topic,
            #         'aruco_marker_name': 'aruco_marker',
            #         'show_image': TextSubstitution(text='false'),
            #     }]
            # ),

            Node(
                package='econ_docking',
                executable='navigate_to_charging_dock_no_nav2',
                name='navigate_to_charging_dock_no_nav2',
                output='screen',
                parameters=[{
                    'target_battery': battery_max_threshold,
                    'low_battery_min_threshold': battery_min_threshold,
                    'undocking_distance': undocking_distance,
                    'approach_distance': approach_distance,
                    'offset_search_tolerance': offset_search_tolerance,
                    'angular_velocity_search': angular_velocity_search,
                }]
            ),
        ])
    ])