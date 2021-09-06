#include "odom_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

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
  m_odomMsg.twist.twist.angular.z = m_poseVelocity.angularVelocity;
  m_odomMsg.twist.twist.linear.x = m_poseVelocity.linearVelocity;
  m_odomMsg.pose.pose.position.x = m_poseVelocity.robotPose.x;
  m_odomMsg.pose.pose.position.y = m_poseVelocity.robotPose.y;

  tf2::Quaternion temp;
  temp.setRPY(0, 0, m_poseVelocity.robotPose.angle);
  tf2::convert(temp, m_odomMsg.pose.pose.orientation);

  m_tfMsg.header = m_odomMsg.header;
  m_tfMsg.transform.rotation = m_odomMsg.pose.pose.orientation;
  m_tfMsg.transform.translation.x = m_odomMsg.pose.pose.position.x;
  m_tfMsg.transform.translation.y = m_odomMsg.pose.pose.position.y;
}


void OdometryPublisher::updateHandler()
{
  if (needAllocateShmem()) {allocateShmem();}
  getPoseAndVelocity();
  updateOdom();

  m_odomPublisher->publish(m_odomMsg);
  m_tfBroadcaster.sendTransform(m_tfMsg);
  std::cout << m_poseVelocity.robotPose.ts << '\n' << ++m_count << '\n';
}

bool OdometryPublisher::getPoseAndVelocity()
{
  try {
    if (!m_poseConsumer->consumerSize()) {return false;}
    m_poseVelocity = m_poseConsumer->getAndPop();
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
