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
    param_substitutions = {}
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True)        
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
        
    decompressor_node = Node(
        package='detection_visualizer',
        executable='decompressor_node',
        name='decompressor_node',
        parameters=[configured_params],
        remappings=remappings)

    detector_node = Node(
        package='openrobotics_darknet_ros',
        executable='detector_node',
        name = 'detector_node',
        parameters=[configured_params],
        remappings=remappings)
    detectorv5_node = Node(
        package='yolov5_ros',
        executable='yolov5_ros',
        name = 'detectorv5_node',
        parameters=[configured_params],
        remappings=remappings)

    ld = LaunchDescription()
    #ld.add_action(decompressor_node)

    #ld.add_action(detector_node)
    ld.add_action(detectorv5_node)

    return ld
