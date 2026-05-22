from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def _octomap_nodes(context, *args, **kwargs):
    """Create one semantic_octomap_node per env (resolved at launch time)."""
    num_envs     = int(context.launch_configurations.get('num_envs', '1'))
    world_frame  = context.launch_configurations.get('world_frame_id', 'world')
    resolution   = context.launch_configurations.get('resolution', '0.05')
    max_range    = context.launch_configurations.get('max_range', '10.0')
    raycast_range= context.launch_configurations.get('raycast_range', '10.0')
    psi          = context.launch_configurations.get('psi', '0.3')
    phi          = context.launch_configurations.get('phi', '-0.1')

    nodes = []
    for i in range(num_envs):
        nodes.append(Node(
            package='semantic_octomap_ros2',
            executable='semantic_octomap_node',
            name=f'semantic_octomap_node_{i}',
            namespace=f'semantic_octomap_{i}',
            output='screen',
            parameters=[{
                'pointcloud_topic': f'/nbv/semantic_pointcloud_{i}',
                'world_frame_id':   world_frame,
                'resolution':       float(resolution),
                'max_range':        float(max_range),
                'raycast_range':    float(raycast_range),
                'psi':              float(psi),
                'phi':              float(phi),
                'publish_2d_map':   False,
                'use_sim_time':     False,
            }],
        ))
    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('semantic_octomap_ros2')
    default_rviz = os.path.join(pkg_share, 'rviz', 'semantic_mapping.rviz')

    return LaunchDescription([
        # Use CycloneDDS — FastDDS on Jazzy has a wire-format bug serializing
        # geometry_msgs/Point[] between rclpy and rclcpp (causes GetRLE to
        # receive empty/shifted requests). CycloneDDS handles it correctly.
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', ''),

        DeclareLaunchArgument('num_envs',       default_value='1',
                              description='Number of independent octree instances (one per RL env)'),
        DeclareLaunchArgument('pointcloud_topic',
                              default_value='/nbv/plant_pointcloud'),
        DeclareLaunchArgument('world_frame_id', default_value='world'),
        DeclareLaunchArgument('resolution',     default_value='0.05'),
        DeclareLaunchArgument('max_range',      default_value='10.0'),
        DeclareLaunchArgument('raycast_range',  default_value='10.0'),
        DeclareLaunchArgument('psi',            default_value='0.3'),
        DeclareLaunchArgument('phi',            default_value='-0.1'),
        DeclareLaunchArgument('rviz',           default_value='true'),
        DeclareLaunchArgument('rviz_config',    default_value=default_rviz),
        DeclareLaunchArgument('use_ptv3_node',  default_value='false'),

        # PTv3 segmentation node (external source only)
        Node(
            package='ptv3_segmentation_ros2',
            executable='ptv3_node',
            name='ptv3_segmentation_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_ptv3_node')),
            remappings=[('/nbv/plant_pointcloud', LaunchConfiguration('pointcloud_topic'))],
            parameters=[{
                'num_classes': 6,
                'conda_python': '/home/fondecyt/anaconda3/envs/pointcept/bin/python3',
            }],
        ),

        # N semantic_octomap_node instances — one per env (OpaqueFunction reads num_envs at runtime)
        OpaqueFunction(function=_octomap_nodes),

        # RViz2 — shows env 0 data (/nbv/semantic_pointcloud_0, markers from semantic_octomap_0)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', default_rviz],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
