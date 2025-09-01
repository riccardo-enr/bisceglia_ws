from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ROS<->Gazebo odometry bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/model/x500_0/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
            output='screen'
        ),
        # Mocap node
        Node(
            package='mocap_bridge',
            executable='mocap_node',
            output='screen'
        )
    ])
