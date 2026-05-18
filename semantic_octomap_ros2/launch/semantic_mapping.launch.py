from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('semantic_octomap_ros2')
    default_rviz = os.path.join(pkg_share, 'rviz', 'semantic_mapping.rviz')

    return LaunchDescription([
        # Use CycloneDDS — FastDDS on Jazzy has a wire-format bug serializing
        # geometry_msgs/Point[] between rclpy and rclcpp (causes GetRLE to
        # receive empty/shifted requests). CycloneDDS handles it correctly.
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        # Unset FastDDS profile in case it's set by the parent shell.
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', ''),

        DeclareLaunchArgument('pointcloud_topic',
                              default_value='/nbv/plant_pointcloud'),
        DeclareLaunchArgument('semantic_pointcloud_topic',
                              default_value='/nbv/semantic_pointcloud'),
        DeclareLaunchArgument('world_frame_id', default_value='world'),
        DeclareLaunchArgument('resolution',     default_value='0.05'),
        DeclareLaunchArgument('max_range',      default_value='10.0'),
        DeclareLaunchArgument('raycast_range',  default_value='10.0'),
        DeclareLaunchArgument('psi',            default_value='0.3'),
        DeclareLaunchArgument('phi',            default_value='-0.1'),
        DeclareLaunchArgument('rviz',           default_value='true'),
        DeclareLaunchArgument('rviz_config',    default_value=default_rviz),

        # PTv3 segmentation node (Python 3.12 bridge → Python 3.8 worker subprocess)
        Node(
            package='ptv3_segmentation_ros2',
            executable='ptv3_node',
            name='ptv3_segmentation_node',
            output='screen',
            remappings=[
                ('/nbv/plant_pointcloud',
                 LaunchConfiguration('pointcloud_topic')),
            ],
            parameters=[{
                'num_classes': 6,
                'conda_python': '/home/fondecyt/anaconda3/envs/pointcept/bin/python3',
            }],
        ),

        # Semantic OctoMap node (C++) — namespace: semantic_octomap
        # Service: /semantic_octomap/query_rle
        Node(
            package='semantic_octomap_ros2',
            executable='semantic_octomap_node',
            name='semantic_octomap_node',
            namespace='semantic_octomap',
            output='screen',
            parameters=[{
                'pointcloud_topic': LaunchConfiguration('semantic_pointcloud_topic'),
                'world_frame_id':   LaunchConfiguration('world_frame_id'),
                'resolution':       LaunchConfiguration('resolution'),
                'max_range':        LaunchConfiguration('max_range'),
                'raycast_range':    LaunchConfiguration('raycast_range'),
                'psi':              LaunchConfiguration('psi'),
                'phi':              LaunchConfiguration('phi'),
                'publish_2d_map':   True,
                'use_sim_time':     False,
            }],
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
