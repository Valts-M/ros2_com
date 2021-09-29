import launch
from launch.substitutions import  LaunchConfiguration
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, ExecuteProcess

from launch_ros.actions import Node

launch.Event

def generate_launch_description():
    slam_params_file = LaunchConfiguration('slam_params_file')
    bag_file = LaunchConfiguration('bag_file')

    rosbag_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-r', '4', bag_file, '--remap', 'tf:=pre_filter_tf', 'tf_static:=pre_filter_tf',
        '--topics', '/clock', '/tf', '/tf_static', '/scan', '/ros2_com/path']
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
        name='map_saver_server'
    )
   

    return launch.LaunchDescription([      
    DeclareLaunchArgument(name='slam_params_file',
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'),
    DeclareLaunchArgument(name='bag_file', description='Full path to bag file'), 
    slam_toolbox_node,
    map_saver_server,
    rosbag_node,
    tf_filter
    ])
    
    
