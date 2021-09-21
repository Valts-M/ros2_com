#include "pose_listener.hpp"
#include "helper.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"

namespace ros2_com
{
  PoseListener::PoseListener() : rclcpp::Node("pose_listener"), 
    m_count(0), m_map_frame("map"), m_odom_frame("odom"),
    m_mapPose(0.0, 0.0, 0.0), m_odomPose(0.0, 0.0, 0.0)
  {
    allocateShmem();

    this->declare_parameter<std::string>("target_frame", "laser_sensor_frame");
    this->get_parameter("target_frame", m_target_frame);

    m_mapLidarTfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_odomLidarTfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    m_mapLidarListener = std::make_shared<tf2_ros::TransformListener>(*m_mapLidarTfBuffer);
    m_odomLidarListener = std::make_shared<tf2_ros::TransformListener>(*m_odomLidarTfBuffer);

    m_timer = this->create_wall_timer(50ms, std::bind(&PoseListener::timerCallback, this));
  }
  
  PoseListener::~PoseListener()
  {
    deallocateShmem();
  }

  void PoseListener::timerCallback()
  {
    if(needAllocateShmem())
      allocateShmem();
    m_ts = Helper::getTimeStamp();

    try {
      m_odomLidarMsg = m_odomLidarTfBuffer->lookupTransform(
        m_odom_frame, m_target_frame,
        tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        m_odom_frame.c_str(), m_target_frame.c_str(), ex.what());
      return;
    }

    tf2::Quaternion tempQuat;
    tf2::convert(m_odomLidarMsg.transform.rotation, tempQuat);
    tf2::Matrix3x3 tempMatrix(tempQuat);
    double roll, pitch, yaw;
    tempMatrix.getEulerYPR(roll, pitch, yaw);

    m_odomPose.x() = m_odomLidarMsg.transform.translation.x;
    m_odomPose.y() = m_odomLidarMsg.transform.translation.y;
    m_odomPose.yaw() = yaw;

    try {
      m_mapLidarMsg = m_mapLidarTfBuffer->lookupTransform(
        m_map_frame, m_target_frame,
        tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        m_map_frame.c_str(), m_target_frame.c_str(), ex.what());
      return;
    }

    // RCLCPP_INFO(this->get_logger(), "Odom: x='%f', y='%f'", m_odomPose.x(), m_odomPose.y());
    m_odomPoseProducer->append(m_odomPose, m_ts);

    tf2::convert(m_mapLidarMsg.transform.rotation, tempQuat);
    tempMatrix.setRotation(tempQuat);
    tempMatrix.getEulerYPR(roll, pitch, yaw);

    m_mapPose.x() = m_mapLidarMsg.transform.translation.x;
    m_mapPose.y() = m_mapLidarMsg.transform.translation.y;
    m_mapPose.yaw() = yaw;

    // RCLCPP_INFO(this->get_logger(), "Map: x='%f', y='%f'", m_odomPose.x(), m_odomPose.y());
    m_mapPoseProducer->append(m_mapPose, m_ts);
  }

bool PoseListener::needAllocateShmem()
{
  return !m_odomPoseProducer.get() && !m_mapPoseProducer.get();
}

void PoseListener::allocateShmem()
{
  if (!m_odomPoseProducer.get()) {
    //TODO: get from config
    m_odomPoseProducer = std::make_unique<ShmemPoseProducer>("RosOdomPoses", "OdomPose", 1024U, 1024U * sizeof(RobotPose) + 10240U);
  }
  if (!m_mapPoseProducer.get()) {
    //TODO: get from config
    m_mapPoseProducer = std::make_unique<ShmemPoseProducer>("RosMapPoses", "MapPose", 1024U, 1024U * sizeof(RobotPose) + 10240U);
  }
  startShmem();
}

void PoseListener::deallocateShmem()
{
  stopShmem();
  m_odomPoseProducer.reset();
  m_mapPoseProducer.reset();
}

void PoseListener::stopShmem()
{
  if (m_odomPoseProducer.get()) {m_odomPoseProducer->stop();}
  if (m_mapPoseProducer.get()) {m_mapPoseProducer->stop();}
}

void PoseListener::startShmem()
{
  if (m_odomPoseProducer.get()) {m_odomPoseProducer->start();}
  if (m_mapPoseProducer.get()) {m_mapPoseProducer->start();}
}
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto pose_listener_node = std::make_shared<ros2_com::PoseListener>();
  rclcpp::spin(pose_listener_node);
  rclcpp::shutdown();
  return 0;
}