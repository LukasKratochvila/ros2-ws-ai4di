from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    webcam_grabber = Node(
        package='image_tools',
        executable='cam2image'
        )
    lidar_grabber = Node(
        package='livox_ros2_driver',
        executable='livox_ros2_driver_node'
        )
        
    img_saver = Node(
        package='writer_cpp',
        executable='writer_cpp_node',
        arguments=['--ros-args', 'img']
        )

    pcl_saver = Node(
        package='writer_cpp',
        executable='writer_cpp_node',
        arguments=['--ros-args', 'pcl']
        )

    return LaunchDescription([
        lidar_grabber,
        webcam_grabber,
        pcl_saver,
        img_saver,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])
