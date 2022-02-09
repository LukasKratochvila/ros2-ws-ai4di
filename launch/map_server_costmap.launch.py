#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    #os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message}'
    
    # Map server parameters
    resources_dir = get_package_share_directory('navigation_bringup')

    autostart = LaunchConfiguration('autostart')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    scan_topic = LaunchConfiguration('scan_topic')##
    
    # Lifecycle parameters
    node_names = ['map_server', 'controller_server']##'amcl','nav2_costmap_2d', 'nav2_costmap_2d', 'recoveries_server', 'planner_server'


    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    # TODO: Are these actually necessary?
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'yaml_filename': map_yaml_file,
        'scan_topic': scan_topic,##
        'topic': scan_topic}##

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True)


    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the map server node.')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true.')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(resources_dir, 'params', 'lukas_params.yaml'), #lukas_params nav2_params
        description='Full path to the map_server parameters file.')

    declare_map_yaml_file_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(resources_dir, 'maps', 'ricaip_hall_map3.yaml'),
        description='Full path to the map parameters file.')
        
    declare_scan_topic_cmd = DeclareLaunchArgument(##
        'scan_topic',
        default_value='/cloud',#'/loki_1/sensors/velodyne_1/velodyne_points',
        description='Topic from which the Navigation2 stack nodes source sensor data.')##
    map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['1','2','0','0','0','0','map','loki_1_foot_link']
        )
    base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0.2','0','0','0','loki_1_foot_link','loki_1_base_link']#'1','2','0.2','0','0','0.2','map','loki_1_base_link'
        )
    top_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0.1','0','0','0','loki_1_base_link','loki_1_top_plate_link'] 
        )
    cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.2','-0.1','0','0','0','0','loki_1_top_plate_link','image'] # loki_1_camera
        )
    
    pcl_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.030','-0.045','0','0','{}'.format(5/180*3.14),'0','image','cloud']
        )
        
    start_amcl_cmd = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            emulate_tty=True,
            parameters=[configured_params],
            remappings=remappings)
    
    start_planner_server_cmd = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            emulate_tty=True,
            parameters=[configured_params],
            remappings=remappings)
    
    start_controller_server_cmd = Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            emulate_tty=True,
            parameters=[configured_params],
            remappings=remappings)
    
    start_recoveries_server_cmd = Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            emulate_tty=True,
            parameters=[configured_params],
            remappings=remappings)
    
    start_map_server_cmd = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            emulate_tty=True,
            parameters=[configured_params],
            remappings=remappings)
            
    start_pcl_pub_cmd = Node(##
            package='pcl_pub_py',
            executable='pcl_pub_node',
            name='pcl_pub_node',
            output='screen',
            emulate_tty=True,
            parameters=[configured_params],
            remappings=remappings)##
    
    start_costmap_2d_markers_cmd = Node(##
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d_markers',
            name='nav2_costmap_2d',
            output='screen',
            emulate_tty=True,
            parameters=[configured_params],
            remappings=[("voxel_grid", "costmap/voxel_grid")])##
            
    start_costmap_2d_cmd = Node(##
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='nav2_costmap_2d',
            output='screen',
            emulate_tty=True,
            parameters=[configured_params],
            remappings=remappings)##

    start_map_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,
            parameters=[{'autostart': autostart},
                        {'use_sim_time': use_sim_time},
                        {'node_names': node_names}])

    ld = LaunchDescription()

    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_yaml_file_cmd)
    ld.add_action(declare_scan_topic_cmd)##
    
    ld.add_action(map_tf)
    ld.add_action(base_tf)
    ld.add_action(top_tf)
    ld.add_action(cam_tf)
    ld.add_action(pcl_tf)
    #ld.add_action(start_amcl_cmd)
    #ld.add_action(start_planner_server_cmd)
    ld.add_action(start_controller_server_cmd)
    #ld.add_action(start_recoveries_server_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_pcl_pub_cmd)##
    #ld.add_action(start_costmap_2d_markers_cmd)##
    #ld.add_action(start_costmap_2d_cmd)##
    ld.add_action(start_map_lifecycle_manager_cmd)

    return ld
