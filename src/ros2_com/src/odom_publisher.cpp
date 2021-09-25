#include "odom_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace ros2_com
{

OdometryPublisher::OdometryPublisher() : OdometryPublisher(rclcpp::NodeOptions()){}

OdometryPublisher::OdometryPublisher(const rclcpp::NodeOptions & options)
: Node("ros2_com", options), m_count(0), m_tfBroadcaster(this)
{
  m_paused = this->declare_parameter(
    "/ros2_com/paused_new_measurements", m_paused);
  m_odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("ros2_com/odom", 10);
  m_pathPublisher = this->create_publisher<nav_msgs::msg::Path>("ros2_com/path", 10);
  m_pauseOdomService = this->create_service<ros2_com::srv::PauseOdom>("ros2_com/pause_odom", 
    std::bind(&OdometryPublisher::pauseToggle, this, _1, _2));
  m_resetOdomService = this->create_service<ros2_com::srv::ResetOdom>("ros2_com/reset_odom", 
    std::bind(&OdometryPublisher::resetOdom, this, _1, _2));

  allocateShmem();
  //TODO: get form config
  initMsgs();
  
  m_rosTimer = this->create_wall_timer(
    10ms,
    std::bind(&OdometryPublisher::updateHandler, this));
}

OdometryPublisher::~OdometryPublisher()
{
  deallocateShmem();
}

void OdometryPublisher::initMsgs()
{
  m_odomMsg.header.frame_id = "odom";
  m_odomMsg.child_frame_id = "base_footprint";
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
}

void OdometryPublisher::pauseToggle(const std::shared_ptr<ros2_com::srv::PauseOdom::Request> request,
          std::shared_ptr<ros2_com::srv::PauseOdom::Response> response)
{
  m_paused = !m_paused;
  response->paused=m_paused;
}

void OdometryPublisher::resetOdom(const std::shared_ptr<ros2_com::srv::ResetOdom::Request> request,
          std::shared_ptr<ros2_com::srv::ResetOdom::Response> response)
{
  m_odomMsg.pose.pose.position.x = 0.0;
  m_odomMsg.pose.pose.position.y = 0.0;
  m_kinematics.m_yaw = 0.0;
  m_odomMsg.pose.pose.orientation = geometry_msgs::msg::Quaternion();
  m_tfMsg.transform.rotation = m_odomMsg.pose.pose.orientation;
  m_tfMsg.transform.translation.x = m_odomMsg.pose.pose.position.x;
  m_tfMsg.transform.translation.y = m_odomMsg.pose.pose.position.y;
  m_pathMsg.poses.clear();
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

  if(!m_paused)
  {
    updateOdom();
    updatePath();

    m_odomPublisher->publish(m_odomMsg);
    m_pathPublisher->publish(m_pathMsg);
    m_tfBroadcaster.sendTransform(m_tfMsg);
    // std::cout << m_odomMsg.pose.pose.position.x << "\t" << m_odomMsg.pose.pose.position.y << '\t' << m_kinematics.m_yaw << '\n';
    RCLCPP_DEBUG(this->get_logger(), 
      "x=%f, y=%f, yaw=%f", 
      m_odomMsg.pose.pose.position.x, 
      m_odomMsg.pose.pose.position.y, 
      m_kinematics.m_yaw);
  }
}

bool OdometryPublisher::getPoseAndVelocity()
{
  try 
  {
    if (!m_poseConsumer->isConsumerReferenced() || !m_poseConsumer->consumerSize()) {return false;}
    m_reactdLog = m_poseConsumer->getAndPop();
  } 
  catch (std::exception & e) 
  {
    std::cout << std::flush << "Failed to get data: " << e.what() << '\n';
    return false;
  }
  return true;
}

bool OdometryPublisher::needAllocateShmem()
{
  // return !m_poseConsumer.get();
  bool needAllocate = false;
  if(!m_poseConsumer) needAllocate = true;
  else if (m_poseConsumer->isInternalError())
  {
    needAllocate = true;
    m_poseConsumer->stop();
    m_poseConsumer.reset();
  }
  return needAllocate;
}

void OdometryPublisher::allocateShmem()
{
  if (!m_poseConsumer.get()) {
    //TODO: get from config
    m_poseConsumer = std::make_unique<ShmemPoseConsumer>(
      "MsgRawStatus", "MsgRawStatus",
      "MsgRawStatusConsumer");
      startShmem();

  }
}

void OdometryPublisher::deallocateShmem()
{
  stopShmem();
  m_poseConsumer.reset();
}

void OdometryPublisher::stopShmem()
{
  if (m_poseConsumer.get()) {m_poseConsumer->stop();}
}

void OdometryPublisher::startShmem()
{
  if (m_poseConsumer.get()) {m_poseConsumer->start();}
}

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto odom_node = std::make_shared<ros2_com::OdometryPublisher>();
  rclcpp::spin(odom_node);
  rclcpp::shutdown();
  return 0;
}
