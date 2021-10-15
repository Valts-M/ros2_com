import launch
from launch.actions import RegisterEventHandler
from launch.event_handlers.on_process_exit import OnProcessExit
from launch.events.process.process_exited import ProcessExited
from launch.launch_context import LaunchContext
from launch.actions import RegisterEventHandler

import launch_ros

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
    clock_server = launch_ros.actions.Node(
        package='ros2_com',
        executable='clock_server',
        name='clock_server'
    )

    rosbag_node = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '--compression-mode','file', 
            '--compression-format', 'zstd'],
        output='screen'
    )

    def shutdown_all(event:ProcessExited, context:LaunchContext):
        if event.returncode != 0:
            print(f"{bcolors.FAIL}[ERROR] {event.action.name} node exited with status code {event.returncode}, shutting down recording nodes{bcolors.ENDC}")
            return launch.actions.EmitEvent(event=launch.events.Shutdown())
        
    event_hand = RegisterEventHandler(event_handler=OnProcessExit(on_exit=shutdown_all))


    return launch.LaunchDescription([      
    clock_server,
    rosbag_node,
    event_hand
    ])
    
    
