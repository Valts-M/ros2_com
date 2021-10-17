import launch
import sys
import os,signal
from launch.substitutions import  LaunchConfiguration
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, ExecuteProcess
from launch.launch_context import LaunchContext
from launch.events.process.process_exited import ProcessExited

from launch_ros.actions import Node
from launch.event_handlers.on_process_exit import OnProcessExit

import yaml

ld = launch.LaunchDescription()

save_dir = LaunchConfiguration('save_dir')

slam_params_file = LaunchConfiguration('slam_params_file')
bag_file = LaunchConfiguration('bag_file')
config_path = "/configs/ros"

robot_config = os.path.join(config_path, 'config', 'robot_configs', 'stolzenberg_1_config.yaml')
with open(robot_config, 'r') as f:
    params = yaml.safe_load(f)['odom_publisher']['ros__parameters']

rosbag_node = ExecuteProcess(
    name='rosbag',
    cmd=['ros2', 'bag', 'play', 
    '--read-ahead-queue-size', '200000', 
    '-r', '5', 
    bag_file,
    '--remap', 'tf:=pre_filter_tf', 'tf_static:=pre_filter_tf',
    '--topics', '/clock', '/tf', '/tf_static', '/scan', '/odom_publisher/path']
)
tf_filter = Node(
    package='ros2_com',
    executable='tf_filter',
    name='tf_filter'
)
slam_toolbox_node = Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    parameters=[slam_params_file, 
        {'use_sim_time': True}]
)
map_saver_server = Node(
    package='ros2_com',
    executable='map_saver',
    name='map_saver_server',
    parameters=[params]
)

declare_params = DeclareLaunchArgument(name='slam_params_file',
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
declare_bag = DeclareLaunchArgument(name='bag_file', description='Full path to bag file')
declare_save =DeclareLaunchArgument(name='save_dir', description='Directory to save bin file in')


def save_map():
    return ExecuteProcess(
        name='save_map',
        cmd=['ros2', 'service', 'call', '/ros2_com/save_map', 'ros2_com/srv/SaveMap', save_dir]
    )

def on_exit_save(event:ProcessExited, context:LaunchContext):
    if event.returncode == 0 and 'rosbag' in event.action.name:
        return save_map()
    else:
        return launch.actions.EmitEvent(event=launch.events.Shutdown())
    
event_hand = RegisterEventHandler(event_handler=OnProcessExit(on_exit=on_exit_save))

def generate_launch_description():
    
    ld.add_action(declare_params)
    ld.add_action(declare_bag)
    ld.add_action(declare_save)
    ld.add_action(map_saver_server)
    ld.add_action(slam_toolbox_node)
    ld.add_action(tf_filter)
    ld.add_action(rosbag_node)
    ld.add_action(event_hand)

    return ld