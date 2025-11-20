from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, EmitEvent, ExecuteProcess, TimerAction
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
import os

def generate_launch_description():
    line_follow_dir = get_package_share_directory('line_follow')
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    world_path = os.path.join(line_follow_dir, 'worlds', 'line_and_points.world')
    rviz_config_path = os.path.join(line_follow_dir, 'rviz', 'tb3.rviz')

    maps_dir = os.path.join(line_follow_dir, 'maps')
    map_save_path = os.path.join(maps_dir, 'line_map') 
    saved_map_yaml = os.path.join(maps_dir, 'line_map.yaml')

    slam_config_path = os.path.join(
        get_package_share_directory('line_follow'),
        'config',
        'slam_params.yaml'
    )
    
    nav2_config_path = os.path.join(
        get_package_share_directory('line_follow'),
        'config',
        'nav2_params.yaml'
    )
    

    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim,"launch","gz_sim.launch.py")
        ),
        launch_arguments ={"gz_args": ["-s -r -v1 ", world_path]}.items()
    )

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim,"launch","gz_sim.launch.py")    
        ),
        launch_arguments = {"gz_args": ["-g -v1"]}.items()
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(line_follow_dir,"launch","robot_state_publisher.launch.py")
        )
    )

    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                os.path.join(line_follow_dir,"launch","spawn_entity.launch.py")
            ),
            launch_arguments={
                'x': "-3.97",
                'y': "-3.8",
                'z': "0.01",
                'yaw': "1.57"
            }.items()
    )

    line_follower_node = Node(
        package='line_follow', 
        executable='line_follow_node',
        name='line_follower',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    return LaunchDescription([
        gzserver_launch,
        gzclient_launch,
        robot_state_publisher,
        spawn_entity,
        line_follower_node
    ])