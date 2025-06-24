# Step 4: Launch file to run all 3 nodes together

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception_pipeline',
            executable='stereo_image_publisher',
            name='stereo_image_publisher_node',
            output='screen'
        ),
        Node(
            package='perception_pipeline',
            executable='stereo_depth_node',
            name='stereo_depth_node',
            output='screen'
        ),
        Node(
            package='perception_pipeline',
            executable='object_fusion_warning_node',
            name='object_fusion_warning_node',
            output='screen'
        )
    ])
