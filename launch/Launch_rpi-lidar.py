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
    
    livox_node = Node(
        package='livox_ros2_driver',
        executable='livox_ros2_driver_node',
        name='livox_node',
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
    map_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,
            parameters=[{'autostart': True},
                        {'use_sim_time': True},
                        {'node_names': pcl_preprocessing_node}])

    ld = LaunchDescription()

    ld.add_action(livox_node)

    ld.add_action(pcl_preprocessing_node)
    ld.add_action(clustering_node)

    ld.add_action(map_lifecycle_manager_cmd)

    return ld
