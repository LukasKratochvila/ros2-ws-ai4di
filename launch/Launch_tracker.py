#from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import os


def generate_launch_description():
    workspace_dir = os.path.join(os.path.dirname(__file__),os.pardir)
    params_file = os.path.join(workspace_dir,"params","launch_params.yaml")
        
    declare_map_yaml_file_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(workspace_dir, 'maps', 'SE1_102_map_new.yaml'),
        description='Full path to the map parameters file.')
    
    map_yaml_file = LaunchConfiguration('map')
        
    param_substitutions = {'yaml_filename': map_yaml_file}
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True)        
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['1','2','0.2','0','0','0','map','loki_1_base_link']#'1','2','0.2','0','0','0.2','map','loki_1_base_link'
        )
    base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0.2','0','0','0','loki_1_foot_link','loki_1_base_link']#'1','2','0.2','0','0','0.2','map','loki_1_base_link'
        )
    top_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0.1','0','0','0','loki_1_base_footprint','loki_1_top_plate_link']) # '0','0','0.1','0','0','0','loki_1_base_link','loki_1_top_plate_link'
    cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.2','-0.1','0','0','0','0','loki_1_top_plate_link','cloud'] # loki_1_camera
        )
    pcl_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.030','0.045','0','0','{}'.format(-5/180*3.14),'0','cloud','image'])
    o_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['28.5','-18','0','{}'.format(0/180*3.14),'0','0','map','mapOrigin'])
    
    decompress = Node(
        package='detection_visualizer',
        executable='decompressor_node',
        name='decompressor',
        parameters=[configured_params],
        remappings=remappings)
        
        
    
    preprocessing = Node(
        package='pcl_preprocessing',
        executable='pcl_preprocessing_node',
        name='pcl_preprocessing_node',
        parameters=[configured_params],
        remappings=remappings)
    cluster = Node(
        package='clustering',
        executable='clustering',
        parameters=[configured_params],
        remappings=remappings)
        
    
    tracker = Node(
        package='tracker',
        executable='tracker_node',
        name='tracker',
        parameters=[configured_params],
        remappings=remappings)
    detection_matcher_viz2 = Node(
        package='detection_visualizer',
        executable='det3d_viz_node',
        name="detection_matcher_viz_kalman",
        parameters=[configured_params],
        remappings=remappings)
        
    #uncompress = Node(
    #    package='image_transport',
    #    executable='republish',
    #    arguments=['compressed', 'in:=/image', 'raw', 'out:=/republish_image'],
    #    remappings=[('/in/compressed', '/image/compressed')])
    #    #arguments=['compressed', 'raw'], #, 'in:=/image/compresed', 'in:=/image/compressed', 'out:=/republish_image'
    #    #remappings=[('/in', '/image/compressed'),
    #    #          ('/out', '/uncompressed')])
        
    node_names = ['map_server','tracker'] #  for lifecycle_manager

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

    ld.add_action(declare_map_yaml_file_cmd)
    
    #ld.add_action(map_tf)
    #ld.add_action(base_tf)
    ld.add_action(top_tf)
    ld.add_action(cam_tf)
    ld.add_action(pcl_tf)
    #ld.add_action(o_tf)
    
    ld.add_action(decompress)
    
    ld.add_action(detection_matcher_viz2)
    ld.add_action(tracker)
    
    ld.add_action(map_server)
    ld.add_action(map_lifecycle_manager_cmd)
    
    ld.add_action(preprocessing)
    ld.add_action(cluster)

    return ld
