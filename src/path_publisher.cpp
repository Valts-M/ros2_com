#include "path_publisher.hpp"

using namespace std::chrono_literals;

namespace ros2_com
{
  PathPublisher::PathPublisher() : rclcpp::Node("pose_listener"), 
    m_map_frame("map"), m_odom_frame("odom"), m_lidar_frame("laser_sensor_frame"),
    m_base_frame("base_footprint")
  {
    geometry_msgs::msg::TransformStamped tempTf{};
    tempTf.header.frame_id = m_map_frame;
    tempTf.child_frame_id = m_lidar_frame;
    PathTracker* tempPtr = &m_paths[alias::lidarMap];

    tempPtr->child_frame = tempTf.child_frame_id;
    tempPtr->parent_frame = tempTf.header.frame_id;
    tempPtr->currTf = tempTf;
    tempPtr->prevTf = tempTf;
    tempPtr->param_name = "publish_lidar_map_path";
    tempPtr->pathPublisher = create_publisher<nav_msgs::msg::Path>("lidar_map_path", 10);
    declare_parameter(tempPtr->param_name , false);
    tempPtr->enabled = get_parameter(tempPtr->param_name).as_bool();

    tempTf.header.frame_id = m_odom_frame;
    tempPtr = &m_paths[alias::lidarOdom];

    tempPtr->child_frame = tempTf.child_frame_id;
    tempPtr->parent_frame = tempTf.header.frame_id;
    tempPtr->currTf = tempTf;
    tempPtr->prevTf = tempTf;
    tempPtr->param_name = "publish_lidar_odom_path";
    tempPtr->pathPublisher = create_publisher<nav_msgs::msg::Path>("lidar_odom_path", 10);
    declare_parameter(tempPtr->param_name , false);
    tempPtr->enabled = get_parameter(tempPtr->param_name).as_bool();

    tempTf.child_frame_id = m_base_frame;
    tempPtr = &m_paths[alias::robotOdom];

    tempPtr->child_frame = tempTf.child_frame_id;
    tempPtr->parent_frame = tempTf.header.frame_id;
    tempPtr->currTf = tempTf;
    tempPtr->prevTf = tempTf;
    tempPtr->param_name = "publish_robot_odom_path";
    tempPtr->pathPublisher = create_publisher<nav_msgs::msg::Path>("robot_odom_path", 10);
    declare_parameter(tempPtr->param_name , false);
    tempPtr->enabled = get_parameter(tempPtr->param_name).as_bool();

    tempTf.header.frame_id = m_map_frame;
    tempPtr = &m_paths[alias::robotMap];

    tempPtr->child_frame = tempTf.child_frame_id;
    tempPtr->parent_frame = tempTf.header.frame_id;
    tempPtr->currTf = tempTf;
    tempPtr->prevTf = tempTf;
    tempPtr->param_name = "publish_robot_map_path";
    tempPtr->pathPublisher = create_publisher<nav_msgs::msg::Path>("robot_map_path", 10);
    declare_parameter(tempPtr->param_name , false);
    tempPtr->enabled = get_parameter(tempPtr->param_name).as_bool();

    m_tfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);

    m_timer = create_wall_timer(100ms, std::bind(&PathPublisher::timerCallback, this));
  }

  void PathPublisher::timerCallback()
  {
    updateParams();
    updatePaths();
  }

  void PathPublisher::updatePaths()
  {
    for(int i = 0; i < alias::count; ++i)
    {
      if(m_paths[i].enabled)
      {
        try {
          m_paths[i].currTf = m_tfBuffer->lookupTransform(
            m_paths[i].parent_frame, m_paths[i].child_frame, tf2::TimePointZero);
          updatePath(i);
        } catch (const tf2::TransformException & ex) {}
      }
    }
  }

  void PathPublisher::updatePath(int i)
  {
    if(m_paths[i].path.poses.empty())
    {
      m_paths[i].prevTf = m_paths[i].currTf;
      m_paths[i].path.header = m_paths[i].currTf.header;
      m_paths[i].path.poses.push_back(toPose(m_paths[i].currTf));
      m_paths[i].pathPublisher->publish(m_paths[i].path);

    }
    else
    {
      if((m_paths[i].currTf.transform.translation - m_paths[i].prevTf.transform.translation) > m_pathUpdateDist)
      {
        m_paths[i].prevTf = m_paths[i].currTf;
        m_paths[i].path.header = m_paths[i].currTf.header;
        m_paths[i].path.poses.push_back(toPose(m_paths[i].currTf));
        m_paths[i].pathPublisher->publish(m_paths[i].path);
      }
    }
  }

  geometry_msgs::msg::PoseStamped PathPublisher::toPose(geometry_msgs::msg::TransformStamped& tf)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = tf.header;
    pose.pose.orientation = tf.transform.rotation;
    pose.pose.position.x = tf.transform.translation.x;
    pose.pose.position.y = tf.transform.translation.y;
    pose.pose.position.z = tf.transform.translation.z;
    return pose;
  }

  void PathPublisher::updateParams()
  {
    for(int i = 0; i < alias::count; i++)
    {
      m_paths[i].enabled = get_parameter(m_paths[i].param_name).as_bool();
    }
  }

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto path_publisher_node = std::make_shared<ros2_com::PathPublisher>();
  rclcpp::spin(path_publisher_node);
  rclcpp::shutdown();
  return 0;
}