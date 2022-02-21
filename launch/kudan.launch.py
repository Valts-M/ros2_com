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
    config_path = "/configs/ros"
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_model = os.getenv('ROBOT_MODEL')

    robot_model_config = os.path.join(config_path, 'config', 'robot_model_configs', robot_model + '_config.yaml')
    with open(robot_model_config, 'r') as f:
        params = yaml.safe_load(f)['model_params']

    kdlidar_config = os.path.join(
        config_path, 'config', 'kudan.yaml'
    )

    license_file_path = os.path.join(
        config_path, 'config', '2023-02-14.kdlicense'
    )

    kudan_node = launch_ros.actions.Node(
        package='kdlidar_ros2',
        executable='kdlidar_ros2_node',
        name='kdlidar_node',
        parameters = ["/configs/ros/config/kudan.yaml",
                      {"license_file" : license_file_path},
                      {"respawn" : True}],
        remappings=[
            ('/sensor/points', '/points'),
            ('/sensor/imu', '/imu')
        ]
    )

    def restart_on_exit(event:ProcessExited, context:LaunchContext):
        if event.returncode != 0:
            print(f"{bcolors.FAIL}[ERROR] {event.action.name} node exited with status code {event.returncode}, restarting node{bcolors.ENDC}")
            return kudan_node
        
    event_hand = RegisterEventHandler(event_handler=OnProcessExit(on_exit=restart_on_exit))


    return launch.LaunchDescription([      
    launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                        description='Flag to enable use_sim_time'),                               
    kudan_node,
    event_hand,
    ])