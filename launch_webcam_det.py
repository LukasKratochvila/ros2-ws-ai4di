import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

# load config for rviz2
cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
rviz_config_path = os.path.join(cur_path, 'webcam.rviz')


def generate_launch_description():
    webcam_grabber = Node(
        package='image_tools',
        executable='cam2image'
        )
#    webcam_grabber = Node(
#        package='v4l2_camera',
#        executable='v4l2_camera_node',
#        remappings=[("image_raw", "image")]
#        )
    yolo_detector = Node(
        package='openrobotics_darknet_ros',
        executable='detector_node',
        arguments=['--ros-args', '--params-file', 'detector_node_params.yaml']
        )
        
    visualizer = Node(
        package='detection_visualizer',
        executable='detection_visualizer'
        )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config', rviz_config_path]
        )

    return LaunchDescription([
        webcam_grabber,
        yolo_detector,
        visualizer,
        rviz,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])
