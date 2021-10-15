import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers.on_process_exit import OnProcessExit
from launch.events.process.process_exited import ProcessExited
from launch.launch_context import LaunchContext
from launch.actions import RegisterEventHandler

import launch_ros
import os
import yaml

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def generate_launch_description():
    config_path = "/Configs/Ros"
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_config = os.path.join(config_path, 'config', 'robot_configs', 'columbus_2_config.yaml')
    with open(robot_config, 'r') as f:
        params = yaml.safe_load(f)['odom_publisher']['ros__parameters']
    
    map_saver_server = launch_ros.actions.Node(
        package='ros2_com',
        executable='map_saver',
        name='map_saver_server',
        parameters=[params]
    )

    slam_toolbox_node = launch_ros.actions.Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[os.path.join(config_path, 'config', 'mapper_params_online_async.yaml'), 
            {'use_sim_time': use_sim_time}]
    )

    localization_node = launch_ros.actions.Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(config_path, 'config', 'localization_params.yaml'),
            {"use_sim_time" : use_sim_time}],
    )


    def shutdown_all(event:ProcessExited, context:LaunchContext):
        if event.returncode != 0:
            print(f"{bcolors.FAIL}[ERROR] {event.action.name} node exited with status code {event.returncode}, shutting down all mapping nodes{bcolors.ENDC}")
            return launch.actions.EmitEvent(event=launch.events.Shutdown())
        
    event_hand = RegisterEventHandler(event_handler=OnProcessExit(on_exit=shutdown_all))


    return launch.LaunchDescription([      
    launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                        description='Flag to enable use_sim_time'),                               
    map_saver_server,
    slam_toolbox_node,
    event_hand,
    ])
    
    
