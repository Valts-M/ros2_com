#include "odom_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"


using namespace std::chrono_literals;

namespace ros2_com
{
OdometryPublisher::OdometryPublisher()
: Node("odom_publisher"), m_count(0), m_tfBroadcaster(this)
{
  m_odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("encoder/odom", 10);
  m_pathPublisher = this->create_publisher<nav_msgs::msg::Path>("encoder/path", 10);

  //service = this->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);
  
  allocateShmem();
  //TODO: get form config
  m_odomMsg.header.frame_id = "odom";
  m_odomMsg.child_frame_id = "base_link";
  m_pathMsg.header.frame_id = m_odomMsg.header.frame_id;

  m_odomMsg.pose.covariance =
  {
    0.008, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.008, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.016
  };

  m_odomMsg.twist.covariance =
  {
    0.004, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.016
  };

  m_tfMsg.header.frame_id = m_odomMsg.header.frame_id;
  m_tfMsg.child_frame_id = m_odomMsg.child_frame_id;

  m_rosTimer = this->create_wall_timer(
    10ms,
    std::bind(&OdometryPublisher::updateHandler, this));
}

OdometryPublisher::~OdometryPublisher()
{
  deallocateShmem();
  m_rosTimer.reset();
  m_odomPublisher.reset();
  std::cout << "\nDestructed\n";
}

void OdometryPublisher::updateOdom()
{
  m_odomMsg.header.stamp = this->get_clock()->now();
  m_kinematics.calcPosAndVelocity(m_reactdLog, m_odomMsg);

  m_tfMsg.header = m_odomMsg.header;
  m_tfMsg.transform.rotation = m_odomMsg.pose.pose.orientation;
  m_tfMsg.transform.translation.x = m_odomMsg.pose.pose.position.x;
  m_tfMsg.transform.translation.y = m_odomMsg.pose.pose.position.y;
}

void OdometryPublisher::updatePath()
{
  geometry_msgs::msg::PoseStamped temp;
  temp.header = m_odomMsg.header;
  temp.pose = m_odomMsg.pose.pose;
  m_pathMsg.poses.push_back(temp);
}

void OdometryPublisher::updateHandler()
{
  if (needAllocateShmem()) {allocateShmem();}
  if(!getPoseAndVelocity()) return;
  updateOdom();
  updatePath();

  m_odomPublisher->publish(m_odomMsg);
  m_pathPublisher->publish(m_pathMsg);
  m_tfBroadcaster.sendTransform(m_tfMsg);
  std::cout << m_odomMsg.pose.pose.position.x << "\t" << m_odomMsg.pose.pose.position.y << '\n';
}

bool OdometryPublisher::getPoseAndVelocity()
{
  try {
    if (!m_poseConsumer->consumerSize()) {return false;}
    m_reactdLog = m_poseConsumer->getAndPop();
  } catch (std::exception & e) {
    // std::cout << std::flush << "Failed to get data: " << e.what() << '\n';
    return false;
  }
  return true;
}

bool OdometryPublisher::needAllocateShmem()
{
  return !m_poseConsumer.get();
}

void OdometryPublisher::allocateShmem()
{
  if (!m_poseConsumer.get()) {
    //TODO: get from config
    m_poseConsumer = std::make_unique<ShmemPoseConsumer>(
      "RobotKinematics", "KinematicsOutput",
      "m_uniqueName");
    m_poseConsumer->start();
  }
}

void OdometryPublisher::deallocateShmem()
{
  m_poseConsumer.reset();
}

void OdometryPublisher::stopShmem()
{
  if (m_poseConsumer.get()) {m_poseConsumer->stop();}
}

void OdometryPublisher::startShmem()
{
  if (m_poseConsumer.get()) {m_poseConsumer->stop();}
}

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_com::OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}
