import launch
from launch.launch_description import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent, DeclareLaunchArgument
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo
from launch.events import matches_action
from launch.event_handlers.on_shutdown import OnShutdown
from launch.event_handlers.on_process_exit import OnProcessExit
from launch.events.process.process_exited import ProcessExited
from launch.launch_context import LaunchContext
from launch.actions import RegisterEventHandler

import launch_ros
import lifecycle_msgs.msg
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
    robot_model = os.getenv('ROBOT_MODEL')
    robot_number = os.getenv('ROBOT_NUMBER')

    robot_urdf_path = os.path.join(config_path, 'descriptions', robot_model + '_description.urdf')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_config_path = os.path.join(config_path, 'config', 'robot_configs', robot_model + '_' + robot_number + '_config.yaml')
    robot_model_config_path = os.path.join(config_path, 'config', 'robot_model_configs', robot_model + '_config.yaml')
    print(bcolors.OKCYAN, 'Robot config = ', robot_config_path, bcolors.ENDC)
    print(bcolors.OKCYAN, 'Robot model config = ', robot_model_config_path, bcolors.ENDC)

    with open(robot_config_path, 'r') as f:
        robot_config = yaml.safe_load(f)
    lidar_params = robot_config['lidar_params']
    robot_calibrations = robot_config['robot_calibrations']

    with open(robot_model_config_path, 'r') as f:
        model_params = yaml.safe_load(f)['model_params']

    if lidar_params['lidar_model'] == 'ouster':
        lidar_config_path = os.path.join(config_path, 'config', 'ouster_config.yaml')
        with open(lidar_config_path, 'r') as f:
            lidar_config = yaml.safe_load(f)['ouster_driver']['ros__parameters']
        lidar_config['lidar_ip'] = lidar_params['lidar_ip']
        lidar_config['computer_ip'] = lidar_params['robot_ip']
    elif lidar_params['lidar_model'] == 'velodyne':
        lidar_config_path = os.path.join(config_path, 'config', 'velodyne_config.yaml')
        with open(lidar_config_path, 'r') as f:
            lidar_config = yaml.safe_load(f)['velodyne_driver_node']['ros__parameters']
        lidar_config['device_ip'] = lidar_params['lidar_ip']

    model_params['lidar_height'] += lidar_config['lidar_z_offset']
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')]),
            'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', 'ERROR'],

    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    clock_server = launch_ros.actions.Node(
        package='ros2_com',
        executable='clock_server',
        name='clock_server'
    )

    velodyne_driver_node = launch_ros.actions.Node(package='velodyne_driver',
        executable='velodyne_driver_node',
        output='screen',
        parameters=[lidar_config],
    )

    velodyne_convert_node = launch_ros.actions.Node(package='velodyne_pointcloud',
        executable='velodyne_convert_node',
        output='screen',
        parameters=[os.path.join(config_path, 'config', 'velodyne_converter_config.yaml')]
    )

    velodyne_laserscan_node = launch_ros.actions.Node(package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        output='screen',
        parameters=[os.path.join(config_path, 'config', 'velodyne_laserscan_config.yaml')]
    )

    odom_publisher_node = launch_ros.actions.Node(
        package='ros2_com',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
        parameters=[robot_calibrations,
            {'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    ouster_node = LifecycleNode(package='ros2_ouster',
                                executable='ouster_driver',
                                name="ouster_driver",
                                output='screen',
                                emulate_tty=True,
                                parameters=[lidar_config],
                                arguments=['--ros-args', '--log-level', 'INFO'],
                                namespace='/',
                                )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(ouster_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=ouster_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] Ouster driver node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(ouster_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # TODO make lifecycle transition to shutdown before SIGINT
    shutdown_event = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                EmitEvent(event=ChangeState(
                  lifecycle_node_matcher=matches_node_name(node_name='ouster_driver'),
                  transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN,
                )),
                LogInfo(
                    msg="[LifecycleLaunch] Ouster driver node is exiting."),
            ],
        )
    )
    
    pose_listener_node = launch_ros.actions.Node(
        package='ros2_com',
        executable='pose_listener',
        name='pose_listener',
        output='screen'
    )

    with open(os.path.join(config_path, 'config', 'point2block_params.yaml'), 'r') as f:
        point2block_params = yaml.safe_load(f)['point2block_params']
    point2block_node = launch_ros.actions.Node(
        package='ros2_com',
        executable='point2block',
        name='point2block',
        output='screen',
        parameters=[model_params, point2block_params]
    )

    urdf_model = DeclareLaunchArgument(name='model', default_value=robot_urdf_path,
                                        description='Absolute path to robot urdf file')

    use_sim_time_arg = DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                        description='Flag to enable use_sim_time')

    
    def shutdown_all(event:ProcessExited, context:LaunchContext):
        if event.returncode != 0:
            print(f"{bcolors.FAIL}[ERROR] {event.action.name} node exited with status code {event.returncode}, shutting down all common nodes{bcolors.ENDC}")
            return launch.actions.EmitEvent(event=launch.events.Shutdown())
        
    crash_event_hand = RegisterEventHandler(event_handler=OnProcessExit(on_exit=shutdown_all))

    ld = LaunchDescription()

    ld.add_action(urdf_model)
    ld.add_action(use_sim_time_arg)
    ld.add_action(clock_server)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(pose_listener_node)
    ld.add_action(odom_publisher_node)
    ld.add_action(point2block_node)
    ld.add_action(crash_event_hand)

    if lidar_params['lidar_model'] == 'ouster':
        ld.add_action(ouster_node)
        ld.add_action(activate_event)
        ld.add_action(configure_event)
        ld.add_action(shutdown_event)
    elif lidar_params['lidar_model'] == 'velodyne':
        ld.add_action(velodyne_driver_node)
        ld.add_action(velodyne_convert_node)
        ld.add_action(velodyne_laserscan_node)

    return ld
    
    
