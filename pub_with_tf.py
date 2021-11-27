from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pcl_pub = Node(
        package='pcl_pub_py',
        executable='pcl_pub_node'
        )
    img_pub = Node(
        package='img_pub_py',
        executable='file_pub'
        )
        
    pcl_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','cloud','map'] # -0.03 0.02 0.03 0 0 0 robot_top_plate
        )
    
    degree = 5/180*3.14
    img_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.030','-0.045','0','0','{}'.format(degree),'0','image','cloud']
        )
    

    return LaunchDescription([
        pcl_pub,
        img_pub,

        pcl_tf,
        img_tf        
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])
