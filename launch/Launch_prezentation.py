#from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import os


def generate_launch_description():
    workspace_dir = os.path.join(os.path.dirname(__file__),os.pardir)
    params_file = os.path.join(workspace_dir,"params/launch_params.yaml")
    rviz_config_path = os.path.join(workspace_dir, 'params/prezentation.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config', rviz_config_path]
        )
        
    param_substitutions = {}
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True)        
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    tf_map_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['1','2','0.2','0','0','0','map','loki_1_base_link']#'1','2','0.2','0','0','0.2','map','loki_1_base_link'
        )
    tf_top_plate_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0.1','0','0','0','loki_1_base_link','loki_1_top_plate_link'])
    
    tf_top_plate_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0.1','0','0','0','loki_1_base_footprint','loki_1_top_plate_link'])
    tf_cloud_to_top_plate = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.2','-0.1','0','0','0','0','loki_1_top_plate_link','cloud'] # loki_1_camera
        )
    tf_image_to_cloud = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.030','0.045','0','0','{}'.format(-5/180*3.14),'0','cloud','image'])
    tf_for_local_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['28.5','-18','0','{}'.format(0/180*3.14),'0','0','map','mapOrigin'])

    cam_image_node = Node(
        package='image_tools_custom',
        executable='cam2image',
        name='cam_image_node',
        parameters=[configured_params],
        remappings=remappings)
    livox_node = Node(
        package='livox_ros2_driver',
        executable='livox_ros2_driver_node',
        name='livox_node',
        parameters=[configured_params],
        remappings=remappings)
        
    decompressor_node = Node(
        package='detection_visualizer',
        executable='decompressor_node',
        name='decompressor_node',
        parameters=[configured_params],
        remappings=remappings)

    writer_node = Node(
        package='writer_cpp',
        executable='writer_cpp_node',
        name='writer_node',
        parameters=[configured_params],
        remappings=remappings)
    pcl_pub_node = Node(
        package='pcl_pub_cpp',
        executable='pcl_pub_cpp_node',
        name='pcl_pub_node',
        parameters=[configured_params],
        remappings=remappings)
    img_pub_node = Node(
        package='img_pub_py',
        executable='file_pub',
        name='img_pub_node',
        parameters=[configured_params],
        remappings=remappings)
    video_pub_node = Node(
        package='img_pub_py',
        executable='img_pub',
        name='video_pub_node',
        parameters=[configured_params],
        remappings=remappings)

    pcl_preprocessing_node = Node(
        package='pcl_preprocessing',
        executable='pcl_preprocessing_node',
        name='pcl_preprocessing_node',
        parameters=[configured_params],
        remappings=remappings)
    clustering_node = Node(
        package='clustering',
        executable='clustering',
        name='clustering_node',
        parameters=[configured_params],
        remappings=remappings)
    map_lookup_node = Node(
        package='map_lookup',
        executable='map_lookup_node',
        name='map_lookup_node',
        parameters=[configured_params],
        remappings=remappings)
    projection_node = Node(
        package='projection',
        executable='projection',
        name="projection_node",
        parameters=[configured_params],
        remappings=remappings)
    detector_node = Node(
        package='openrobotics_darknet_ros',
        executable='detector_node',
        name = 'detector_node',
        parameters=[configured_params],
        remappings=remappings)
    detection_matcher_node = Node(
        package='detection_matcher_py',
        executable='detection_matcher_node',
        name="detection_matcher_node",
        parameters=[configured_params],
        remappings=remappings)
        
    tracker_2d_node = Node(
        package='tracker',
        executable='tracker_node',
        name='tracker_2d_node',
        parameters=[configured_params],
        remappings=remappings)
    tracker_3d_node = Node(
        package='tracker',
        executable='tracker_node',
        name='tracker_3d_node',
        parameters=[configured_params],
        remappings=remappings)
    
    viz_2d_yolo = Node(
        package='detection_visualizer',
        executable='detection_visualizer',
        name='viz_2d_yolo',
        parameters=[configured_params],
        remappings=remappings)
    viz_2d_projection = Node(
        package='detection_visualizer',
        executable='detection_visualizer',
        name='viz_2d_projection',
        parameters=[configured_params],
        remappings=remappings)
    viz_2d_tracker = Node(
        package='detection_visualizer',
        executable='detection_visualizer',
        name='viz_2d_tracker',
        parameters=[configured_params],
        remappings=remappings)
        
    viz_3d_clusters = Node(
        package='detection_visualizer',
        executable='det3d_viz_node',
        name="viz_3d_clusters",
        parameters=[configured_params],
        remappings=remappings)
    viz_3d_matcher = Node(
        package='detection_visualizer',
        executable='det3d_viz_node',
        name="viz_3d_matcher",
        parameters=[configured_params],
        remappings=remappings)
    viz_3d_tracker = Node(
        package='detection_visualizer',
        executable='det3d_viz_node',
        name="viz_3d_tracker",
        parameters=[configured_params],
        remappings=remappings)

    node_names = ['map_server','pcl_preprocessing_node'] #  for lifecycle_manager

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        emulate_tty=True,
        parameters=[configured_params],
        remappings=remappings)
    map_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,
            parameters=[{'autostart': True},
                        {'use_sim_time': True},
                        {'node_names': node_names}])

    ld = LaunchDescription()
    ld.add_action(rviz)


    # Nodes for data without odometry
    #ld.add_action(tf_map_to_base_link)
    #ld.add_action(tf_top_plate_to_base_link)
    
    ld.add_action(tf_top_plate_to_base_footprint)
    ld.add_action(tf_cloud_to_top_plate)
    ld.add_action(tf_image_to_cloud)
    
    #ld.add_action(tf_for_local_map)

    #ld.add_action(cam_image_node)
    #ld.add_action(livox_node)
    
    ld.add_action(decompressor_node)

    #ld.add_action(writer_node)
    #ld.add_action(pcl_pub_node)
    #ld.add_action(img_pub_node)
    #ld.add_action(video_pub_node)

    ld.add_action(pcl_preprocessing_node)
    ld.add_action(clustering_node)
    #ld.add_action(map_lookup_node)
    #ld.add_action(projection_node)
    ld.add_action(detector_node)
    #ld.add_action(detection_matcher_node)
    
    #ld.add_action(tracker_2d_node)
    #ld.add_action(tracker_3d_node)

    ld.add_action(viz_2d_yolo)
    #ld.add_action(viz_2d_projection)
    #ld.add_action(viz_2d_tracker)
    
    ld.add_action(viz_3d_clusters)
    #ld.add_action(viz_3d_matcher)
    #ld.add_action(viz_3d_tracker)
    
    ld.add_action(map_server)
    ld.add_action(map_lifecycle_manager_cmd)

    return ld
    
'''
TODO:
- doladit tracking
- odriznuti v preprocessingu podle mapy
- diagram - popsat uzly
'''
