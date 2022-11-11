#from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    workspace_dir = os.path.join(os.path.dirname(__file__),os.pardir)
    params_file = os.path.join(workspace_dir,"params/launch_params.yaml")   
    param_substitutions = {}

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='/loki_1',
        description='Set robot name for all topics. (default: /loki_1)')
    robot_name = LaunchConfiguration('robot_name')
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True)        
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    cam_image_node = Node(
        package='image_tools_custom',
        executable='cam2image',
        name='cam_image_node',
        namespace=robot_name,
        parameters=[configured_params],
        remappings=remappings)
    
    map_lookup_node = Node(
        package='map_lookup',
        executable='map_lookup_node',
        name='map_lookup_node',
        namespace=robot_name,
        parameters=[configured_params],
        remappings=[('/loki_1/tf', '/tf'),
         ('/loki_1/tf_static', '/tf_static')])
    projection_node = Node(
        package='projection',
        executable='projection',
        name="projection_node",
        namespace=robot_name,
        parameters=[configured_params],
        remappings=[('/loki_1/tf', '/tf'),
         ('/loki_1/tf_static', '/tf_static')])
    detection_matcher_node = Node(
        package='detection_matcher_py',
        executable='detection_matcher_node',
        name="detection_matcher_node",
        namespace=robot_name,
        parameters=[configured_params],
        remappings=remappings)
        
    viz_3d_matcher = Node(
        package='detection_visualizer',
        executable='det3d_viz_node',
        name="viz_3d_matcher",
        namespace=robot_name,
        parameters=[configured_params],
        remappings=remappings)

    ld = LaunchDescription()

    ld.add_action(declare_robot_name_cmd)

    ld.add_action(cam_image_node)
    
    #ld.add_action(map_lookup_node)
    ld.add_action(projection_node)
    
    ld.add_action(detection_matcher_node)
    
    ld.add_action(viz_3d_matcher)

    return ld
