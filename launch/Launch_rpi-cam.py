#from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import os


def generate_launch_description():
    workspace_dir = os.path.join(os.path.dirname(__file__),os.pardir)
    params_file = os.path.join(workspace_dir,"params/launch_params.yaml")   
    param_substitutions = {}
    
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
    detection_matcher_node = Node(
        package='detection_matcher_py',
        executable='detection_matcher_node',
        name="detection_matcher_node",
        parameters=[configured_params],
        remappings=remappings)
        
    viz_3d_matcher = Node(
        package='detection_visualizer',
        executable='det3d_viz_node',
        name="viz_3d_matcher",
        parameters=[configured_params],
        remappings=remappings)

    ld = LaunchDescription()

    ld.add_action(cam_image_node)
    
    #ld.add_action(map_lookup_node)
    ld.add_action(projection_node)
    
    ld.add_action(detection_matcher_node)
    
    ld.add_action(viz_3d_matcher)

    return ld
