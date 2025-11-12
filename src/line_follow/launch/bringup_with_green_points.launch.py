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
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    turtlebot3_desc_pkg = FindPackageShare('turtlebot3_description') 
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
    
    urdf_file = PathJoinSubstitution([
        turtlebot3_desc_pkg,
        'urdf',
        'turtlebot3_waffle_pi.urdf'
    ])
    sdf_path = os.path.join(line_follow_dir, 'models', 'turtlebot3_waffle_pi', 'model.sdf')  

    robot_description_content = Command(['cat ', urdf_file])

    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(line_follow_dir, 'models') + ':/opt/ros/humble/share/turtlebot3_gazebo/models'
    )

    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'true'
        }.items()
    )

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={
            'verbose': 'true'
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_content
        }]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_waffle_pi',
            '-file', sdf_path,
            '-x', '-6.05', '-y', '-6.0', '-z', '0.0',
            '-Y','1.57'
        ],
        output='screen',
    )
    
    line_follower_node = Node(
        package='line_follow', 
        executable='line_follow_node',
        name='line_follower',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_path, 
            {'use_sim_time': True}
        ],
    )

    map_saver_process = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli', 
             '-f', map_save_path, 
             '--ros-args', 
             '-p', 'use_sim_time:=true', 
             '-p', 'map_subscribe_transient_local:=true'
            ],
        output='screen',
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': saved_map_yaml
        }]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='log',
        parameters=[nav2_config_path]
    )

    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl', 'controller_server', 'planner_server', 'behavior_server', 'bt_navigator']
        }]
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_config_path]
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_config_path]
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_config_path]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_config_path]
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_config_path]
    )

    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_config_path]
    )

    green_point_navigator_node = Node(
        package='line_follow',
        executable='green_point_navigator_node',
        name='green_point_navigator',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    start_navigation_action = TimerAction(
        period=5.0,  
        actions=[
            map_server_node,
            amcl_node,
            controller_server_node,
            planner_server_node,
            behavior_server_node,
            bt_navigator_node,
            waypoint_follower_node,
            velocity_smoother_node,
            lifecycle_manager_nav,
            TimerAction(
                period=10.0,
                actions=[green_point_navigator_node]
            )
        ]
    )

    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=line_follower_node,
            on_exit=[
                map_saver_process,
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=map_saver_process,
                        on_exit=[
                            start_navigation_action
                        ]
                    )
                )
            ]
        )
    )

    return LaunchDescription([
        gazebo_model_path,
        gzserver_launch,
        gzclient_launch,
        robot_state_publisher,
        spawn_entity,
        rviz_node,
        line_follower_node,
        slam_node,
        shutdown_handler
    ])