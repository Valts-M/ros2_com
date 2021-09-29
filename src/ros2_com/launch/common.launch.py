import launch
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo
from launch.events import matches_action
from launch.event_handlers.on_shutdown import OnShutdown

import launch_ros
import lifecycle_msgs.msg
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='ros2_com').find('ros2_com')
    default_model_path = os.path.join(pkg_share, 'descriptions/columbus_description.urdf')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
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
    clock_server = launch_ros.actions.Node(
        package='ros2_com',
        executable='clock_server',
        name='clock_server'
    )
    map_saver_server = launch_ros.actions.Node(
        package='ros2_com',
        executable='map_saver',
        name='map_saver_server'
    )
    cloud_filter = launch_ros.actions.Node(
        package='ros2_com',
        executable='point2block',
        name='point2block'
    )
    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), 
            {'use_sim_time': use_sim_time}]
	)
    slam_toolbox_node = launch_ros.actions.Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[os.path.join(pkg_share, 'config/mapper_params_online_async.yaml'), 
            {'use_sim_time': use_sim_time}]
    )

    localization_node = launch_ros.actions.Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/localization_params.yaml'),
            {"use_sim_time" : use_sim_time}],
    )

    odom_publisher_node = launch_ros.actions.Node(
        package='ros2_com',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
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

    return launch.LaunchDescription([      
    launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                        description='Absolute path to robot urdf file'),
    launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                        description='Flag to enable use_sim_time'),                               
    # map_saver_server,
    # clock_server,
    robot_state_publisher_node,
    #slam_toolbox_node,
    # localization_node,
    pose_listener_node,
    odom_publisher_node,
    ouster_node,
    activate_event,
    configure_event,
    shutdown_event,
    cloud_filter
    ])
    
    
