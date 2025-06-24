# Step 4: This code does the following:
# Creates a launch file to launch the subscriber nodes first (depth, fusion).
# Waits 5 seconds, then launches the image publisher.

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start stereo_depth_node immediately
        Node(
            package='perception_pipeline',
            executable='stereo_depth_node',
            name='stereo_depth_node',
            output='screen'
        ),
        # Start object_fusion_warning_node immediately
        Node(
            package='perception_pipeline',
            executable='object_fusion_warning_node',
            name='object_fusion_warning_node',
            output='screen'
        ),
        # Delay the stereo_image_publisher node by 2 seconds
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='perception_pipeline',
                    executable='stereo_image_publisher',
                    name='stereo_image_publisher_node',
                    output='screen'
                )
            ]
        )
    ])
