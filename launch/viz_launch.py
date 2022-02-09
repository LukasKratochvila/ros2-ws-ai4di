#from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
#import os


def generate_launch_description():
    workspace_dir = os.path.join(os.path.dirname(__file__),os.pardir)
    params_file = os.path.join(workspace_dir,"params/launch_params.yaml")
    
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

    viz = Node(
        package='detection_visualizer',
        executable='detection_visualizer',
        name='viz',
        parameters=[configured_params],
        remappings=remappings)

    ld = LaunchDescription()
    
    ld.add_action(declare_debug_cmd)
    ld.add_action(viz)

    return ld
