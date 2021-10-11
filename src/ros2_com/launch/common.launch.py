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
    pkg_share = launch_ros.substitutions.FindPackageShare(package='ros2_com').find('ros2_com')
    default_model_path = os.path.join(pkg_share, 'descriptions/columbus_description.urdf')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_config = os.path.join(pkg_share, 'config', 'robot_configs', 'columbus_2_config.yaml')
    with open(robot_config, 'r') as f:
        lidar_model = yaml.safe_load(f)['odom_publisher']['ros__parameters']['lidar_model']
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')]),
            'use_sim_time': use_sim_time}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    velodyne_driver_node = launch_ros.actions.Node(package='velodyne_driver',
        executable='velodyne_driver_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/velodyne_config.yaml')],
    )

    convert_params_file = os.path.join(pkg_share, 'config', 'velodyne_converter_config.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_convert_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(pkg_share, 'config', 'VLP16db.yaml')

    velodyne_convert_node = launch_ros.actions.Node(package='velodyne_pointcloud',
        executable='velodyne_convert_node',
        output='screen',
        parameters=[convert_params]
    )

    velodyne_laserscan_node = launch_ros.actions.Node(package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'velodyne_laserscan_config.yaml')]
    )


    odom_publisher_node = launch_ros.actions.Node(
        package='ros2_com',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
        parameters=[robot_config,
            {'use_sim_time': use_sim_time}],
    )

    ouster_node = LifecycleNode(package='ros2_ouster',
                                executable='ouster_driver',
                                name="ouster_driver",
                                output='screen',
                                emulate_tty=True,
                                parameters=[os.path.join(pkg_share, 'config/ouster_config.yaml'), {'use_sim_time': use_sim_time}],
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

    point2block_node = launch_ros.actions.Node(
        package='ros2_com',
        executable='point2block',
        name='point2block',
        output='screen'
    )

    urdf_model = DeclareLaunchArgument(name='model', default_value=default_model_path,
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
    ld.add_action(robot_state_publisher_node)
    ld.add_action(pose_listener_node)
    ld.add_action(odom_publisher_node)
    # ld.add_action(point2block_node)
    ld.add_action(crash_event_hand)

    if lidar_model == 'ouster':
        ld.add_action(ouster_node)
        ld.add_action(activate_event)
        ld.add_action(configure_event)
        ld.add_action(shutdown_event)
    elif lidar_model == 'velodyne':
        ld.add_action(velodyne_driver_node)
        ld.add_action(velodyne_convert_node)
        ld.add_action(velodyne_laserscan_node)

    return ld
    
    
