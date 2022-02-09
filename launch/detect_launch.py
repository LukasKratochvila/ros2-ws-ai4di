from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    img_pub = Node(
        package='img_pub_py',
        executable='file_pub'
        )
    detect = Node(
        package='openrobotics_darknet_ros',
        executable='detector_node',
        parameters=['detector_node_params.yaml'],
        )
    viz = Node(
        package='detection_visualizer',
        executable='detection_visualizer'
        )

    return LaunchDescription([
        img_pub,
        detect,
        viz
    ])
