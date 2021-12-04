#from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
#from nav2_common.launch import RewrittenYaml
#import os


def generate_launch_description():


    #resources_dir = get_package_share_directory('navigation_bringup')
    #params_file = os.path.join(resources_dir, 'params', 'lukas_params.yaml')
    #print(params_file)
    
    #param_substitutions = {
    #    'yaml_filename': os.path.join(resources_dir, 'maps', 'ricaip_hall_map3.yaml')}
    #print(param_substitutions)
    
    #configured_params = RewrittenYaml(
    #    source_file=params_file,
    #    param_rewrites=param_substitutions,
    #    convert_types=True)
        
    #node_names = ['map_server']
        
    #remappings = [('/tf', 'tf'),
    #              ('/tf_static', 'tf_static')]
                  
    #map_server = Node(
    #    package='nav2_map_server',
    #    executable='map_server',
    #    name='map_server',
    #    output='screen',
    #    emulate_tty=True,
    #    parameters=[param_substitutions],
    #    remappings=remappings)
    
    #pcl_pub = Node(
    #    package='pcl_pub_py',
    #    executable='pcl_pub_node'
    #    )
    #img_pub = Node(
    #    package='img_pub_py',
    #    executable='file_pub'
    #    )
    lidar_grabber = Node(
        package='livox_ros2_driver',
        executable='livox_ros2_driver'
    )
    preprocessing = Node(
        package='pcl_preprocessing',
        executable='pcl_preprocessing_node',
        remappings=[('livox/lidar','/cloud')]
        )
    cluster = Node(
        package='clustering',
        executable='clustering',
        remappings=[('/cloud','/filteredPcl')]
        )
    #map_lookup = Node(
    #    package='map_lookup',
    #    executable='map_lookup_node'
    #    )
    #project = Node(
    #    package='projection',
    #    executable='projection',
    #    remappings=[('/detections','/unknownObstacles')]
    #    )
    #viz = Node(
    #    package='detection_visualizer',
    #    executable='detection_visualizer',
    #    remappings=[('/detector_node/detections','/cluster_det')]
    #    )
        
    #map_tf = Node(
    #    package='tf2_ros',
    #    executable='static_transform_publisher',
    #    arguments=['1','2','0.2','0','0','0','map','loki_1_base_link']#'1','2','0.2','0','0','0.2','map','loki_1_base_link'
    #    )
    #top_tf = Node(
    #    package='tf2_ros',
    #    executable='static_transform_publisher',
    #    arguments=['0','0','0.1','0','0','0','loki_1_base_link','loki_1_top_plate_link'] 
    #    )
    cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.2','-0.1','0','0','0','0','loki_1_top_plate_link','cloud'] # loki_1_camera
        )
    
    pcl_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.030','0.045','0','0','{}'.format(-5/180*3.14),'0','cloud','image']
        )
        
    o_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['28.5','-18','0','{}'.format(0/180*3.14),'0','0','map','mapOrigin']
        )
    #map_lifecycle_manager_cmd = Node(
    #        package='nav2_lifecycle_manager',
    #        executable='lifecycle_manager',
    #        name='lifecycle_manager',
    #        output='screen',
    #        emulate_tty=True,
    #        parameters=[{'autostart': True},
    #                    {'use_sim_time': True},
    #                    {'node_names': node_names}])
    

    return LaunchDescription([
	lidar_grabber,
        #pcl_pub,
        preprocessing,
        cluster,
        #img_pub,
        #map_lookup,
        #project,
        #viz,
        #map_server,
        
        #map_tf,
        #top_tf,
        cam_tf,
        pcl_tf,
        o_tf,
        #map_lifecycle_manager_cmd
    ])
