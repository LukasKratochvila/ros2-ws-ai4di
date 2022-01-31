#from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
#import os


def generate_launch_description():
    params_file = "launch_params.yaml"
    
    declare_debug_cmd = DeclareLaunchArgument(
        'debug',
        default_value='False',
        description='Debugging')
    
    debug = LaunchConfiguration('debug')
        
    param_substitutions = {"debug": debug}
    
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
    top_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0.1','0','0','0','loki_1_base_link','loki_1_top_plate_link'])
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

    cam_grabber = Node(
        package='image_tools',
        executable='cam2image',
        name='cam_image',
        parameters=[configured_params],
        remappings=remappings)
    livox_grabber = Node(
        package='livox_ros2_driver',
        executable='livox_ros2_driver_node',
        name='livox',
        parameters=[configured_params],
        remappings=remappings)

    writer_cpp = Node(
        package='writer_cpp',
        executable='writer_cpp_node',
        name='writer_cpp',
        parameters=[configured_params],
        remappings=remappings)
    #writer_py = Node(
    #    package='writer',
    #    executable='writer_node',
    #    name='writer_py',
    #    parameters=[configured_params],
    #    remappings=remappings)
    pcl_pub = Node(
        package='pcl_pub_cpp',
        executable='pcl_pub_cpp_node',
        name='pcl_pub_cpp_node',
        parameters=[configured_params],
        remappings=remappings)
    #pcl_pub_py = Node(
    #    package='pcl_pub_py',
    #    executable='pcl_pub_node',
    #    name='pcl_pub_py',
    #    parameters=[configured_params],
    #    remappings=remappings)
    img_pub = Node(
        package='img_pub_py',
        executable='file_pub',
        name='img_pub_py',
        parameters=[configured_params],
        remappings=remappings)
    video_pub = Node(
        package='img_pub_py',
        executable='img_pub',
        name='video_pub',
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
    border_checker = Node(
        package='border_checker',
        executable='border_checker',
        name='border_checker',
        parameters=[configured_params],
        remappings=remappings)
    map_lookup = Node(
        package='map_lookup',
        executable='map_lookup_node',
        parameters=[configured_params],
        remappings=remappings)
    project = Node(
        package='projection',
        executable='projection',
        name="projection",
        parameters=[configured_params],
        remappings=remappings)
    yolo = Node(
        package='openrobotics_darknet_ros',
        executable='detector_node',
        name = 'detector_node',
        parameters=[configured_params],
        remappings=remappings)

    viz = Node(
        package='detection_visualizer',
        executable='detection_visualizer',
        name='viz',
        parameters=[configured_params],
        remappings=remappings)

    node_names = ['map_server'] #  for lifecycle_manager

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        emulate_tty=True,
        parameters=[param_substitutions],
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
    
    ld.add_action(declare_debug_cmd)
    
    ld.add_action(map_tf)
    #ld.add_action(base_tf)
    ld.add_action(top_tf)
    ld.add_action(cam_tf)
    ld.add_action(pcl_tf)
    ld.add_action(o_tf)

    #ld.add_action(cam_grabber)
    #ld.add_action(livox_grabber)

    #ld.add_action(writer_py)
    #ld.add_action(writer_cpp)
    ld.add_action(pcl_pub)
    #ld.add_action(pcl_pub_py)
    ld.add_action(img_pub)
    #ld.add_action(video_pub)

    ld.add_action(preprocessing)
    ld.add_action(cluster)
    #ld.add_action(border_checker)
    #ld.add_action(map_lookup)
    ld.add_action(project)
    #ld.add_action(yolo)

    ld.add_action(viz)
    
    #ld.add_action(map_server)
    #ld.add_action(map_lifecycle_manager_cmd)

    return ld
