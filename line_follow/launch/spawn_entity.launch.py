import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='-4.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.01')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='1.57')
    
    spawn_entity = Node(
        package = "ros_gz_sim",
        executable = "create",
        arguments=[
            "-name","robot",
            "-topic","robot_description",
            "-x", LaunchConfiguration('x'),
            "-y", LaunchConfiguration('y'),
            "-z", LaunchConfiguration('z'),
            "-Y", LaunchConfiguration('yaw')
        ],
        parameters=[
            {'use_sim_time': True}
        ],
        output="screen"
    )


    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/model/robot/tf', '/tf'), 
        ]
    )


    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        spawn_entity,
        bridge
    ])