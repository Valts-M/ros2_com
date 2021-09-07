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
  allocateShmem();
  //TODO: get form config
  m_odomMsg.header.frame_id = "odom";
  m_odomMsg.child_frame_id = "base_link";

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


void OdometryPublisher::updateHandler()
{
  if (needAllocateShmem()) {allocateShmem();}
  if(!getPoseAndVelocity()) return;
  updateOdom();

  m_odomPublisher->publish(m_odomMsg);
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
      "ROS2", "KinematicsInput",
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
  // if consumers and producers are alot, then create std::vector<ShmemBase> and start, stop with iterators
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
