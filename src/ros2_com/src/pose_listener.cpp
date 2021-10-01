#include "pose_listener.hpp"
#include "helper.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace ros2_com
{
  PoseListener::PoseListener() : rclcpp::Node("pose_listener"), 
    m_count(0), m_map_frame("map"), m_odom_frame("odom"),
    m_mapPose(0.0, 0.0, 0.0), m_odomPose(0.0, 0.0, 0.0)
  {
    
    m_shmemUtil = std::make_unique<ShmemUtility>(std::vector<ConsProdNames>{ConsProdNames::p_MapPose, ConsProdNames::p_OdomPose});
    m_shmemUtil->start();

    this->declare_parameter<std::string>("target_frame", "base_footprint");
    this->get_parameter("target_frame", m_target_frame);

    m_sendPoseService = this->create_service<ros2_com::srv::SendInitialPose>("ros2_com/send_initial_pose", 
    std::bind(&PoseListener::sendInitialPose, this, _1, _2));

    m_savePoseService = this->create_service<ros2_com::srv::SaveInitialPose>("ros2_com/save_initial_pose", 
    std::bind(&PoseListener::saveInitialPose, this, _1, _2)); 

    m_initialPosePublisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

    m_tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);

    m_timer = this->create_wall_timer(50ms, std::bind(&PoseListener::timerCallback, this));
  }
  
  PoseListener::~PoseListener()
  {
    m_shmemUtil->stop();
    m_shmemUtil.reset();
  }

  void PoseListener::sendInitialPose(
    const std::shared_ptr<ros2_com::srv::SendInitialPose::Request> request,
    std::shared_ptr<ros2_com::srv::SendInitialPose::Response> response)
  {
    if(m_initialPosePublisher->get_subscription_count() > 0)
    {
      m_initialPose.header.stamp = this->now();
      m_initialPosePublisher->publish(m_initialPose);
      response->success=true;
    }
    else
      response->success=false;
  }

  void PoseListener::saveInitialPose(
    const std::shared_ptr<ros2_com::srv::SaveInitialPose::Request> request,
    std::shared_ptr<ros2_com::srv::SaveInitialPose::Response> response)
  {
    try
    {
      geometry_msgs::msg::TransformStamped mapBaseLinkTransform = 
      m_tfBuffer->lookupTransform(m_map_frame, "base_footprint", tf2::TimePointZero);

      m_initialPose.header = mapBaseLinkTransform.header;
      m_initialPose.pose.pose.position.x = mapBaseLinkTransform.transform.translation.x;
      m_initialPose.pose.pose.position.y = mapBaseLinkTransform.transform.translation.y;
      m_initialPose.pose.pose.position.z = mapBaseLinkTransform.transform.translation.z;
      m_initialPose.pose.pose.orientation = mapBaseLinkTransform.transform.rotation;

      response->success=true;
    }
    catch(const tf2::TransformException & ex)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000, "Could not transform %s to base_footprint: %s",
        m_map_frame.c_str(), ex.what());
      response->success=false;
    }
  }

  void PoseListener::timerCallback()
  {
    // if(needAllocateShmem())
    //   allocateShmem();
      auto m_odomPoseProducer = m_shmemUtil->getShmem<PositionProducer>(ConsProdNames::p_OdomPose);
      auto m_mapPoseProducer = m_shmemUtil->getShmem<PositionProducer>(ConsProdNames::p_MapPose);
    
    if (!m_odomPoseProducer ) return;
    if (!m_odomPoseProducer->isObjectReferenced()) return;
    m_ts = Helper::getTimeStamp();

    try {
      m_odomLidarMsg = m_tfBuffer->lookupTransform(
        m_odom_frame, m_target_frame,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000, "Could not transform %s to base_footprint: %s",
        m_map_frame.c_str(), ex.what());
      return;
    }

    tf2::Quaternion tempQuat;
    tf2::convert(m_odomLidarMsg.transform.rotation, tempQuat);
    // RCLCPP_INFO(this->get_logger(), "x=%f, y=%f, z=%f, w=%f", tempQuat.x(), tempQuat.y(), tempQuat.z(), tempQuat.w());
    tf2::Matrix3x3 tempMatrix(tempQuat);
    // RCLCPP_INFO(this->get_logger(), "%f %f %f\n%f %f %f\n%f %f %f", 
    // tempMatrix[0][0], tempMatrix[0][1], tempMatrix[0][2],
    // tempMatrix[1][0], tempMatrix[1][1], tempMatrix[1][2],
    // tempMatrix[2][0], tempMatrix[2][1], tempMatrix[2][2]);
    double roll, pitch, yaw;
    tempMatrix.getEulerYPR(yaw, pitch, roll);

    m_odomPose.x() = m_odomLidarMsg.transform.translation.x;
    m_odomPose.y() = m_odomLidarMsg.transform.translation.y;
    m_odomPose.yaw() = yaw;
    m_odomPoseProducer->append(m_odomPose, m_ts);

    RCLCPP_DEBUG(this->get_logger(), "Odom: x='%f', y='%f'", m_odomPose.x(), m_odomPose.y());
    RCLCPP_DEBUG(this->get_logger(), "roll=%f, pitch=%f, yaw=%f", roll, pitch, yaw);
    if (!m_mapPoseProducer ) return;
    if (!m_mapPoseProducer->isObjectReferenced()) return;
    try {
      m_mapLidarMsg = m_tfBuffer->lookupTransform(
        m_map_frame, m_target_frame,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000, "Could not transform %s to base_footprint: %s",
        m_map_frame.c_str(), ex.what());
      return;
    }

    tf2::convert(m_mapLidarMsg.transform.rotation, tempQuat);
    tempMatrix.setRotation(tempQuat);
    tempMatrix.getEulerYPR(yaw, pitch, roll);

    m_mapPose.x() = m_mapLidarMsg.transform.translation.x;
    m_mapPose.y() = m_mapLidarMsg.transform.translation.y;
    m_mapPose.yaw() = yaw;

    RCLCPP_DEBUG(this->get_logger(), "Map: x='%f', y='%f'", m_odomPose.x(), m_odomPose.y());
    RCLCPP_DEBUG(this->get_logger(), "roll=%f, pitch=%f, yaw=%f", roll, pitch, yaw);
    m_mapPoseProducer->append(m_mapPose, m_ts);
  }

// bool PoseListener::needAllocateShmem()
// {
//   if(!m_odomPoseProducer || !m_mapPoseProducer)
//     return true;
//   if(m_odomPoseProducer->isInternalError())
//   {
//     m_odomPoseProducer->stop();
//     m_odomPoseProducer.reset();
//     return true;
//   }
//   if(m_mapPoseProducer->isInternalError())
//   {
//     m_mapPoseProducer->stop();
//     m_mapPoseProducer.reset();
//     return true;
//   }
//   return false;
// }

// void PoseListener::allocateShmem()
// {
//   if (!m_odomPoseProducer) {
//     //TODO: get from config
//     m_odomPoseProducer = std::make_unique<ShmemPoseProducer>("RosOdomPoses", "RosOdomPoses", 1024U, 1024U * sizeof(RobotPose) + 10240U);
//   }
//   if (!m_mapPoseProducer) {
//     //TODO: get from config
//     m_mapPoseProducer = std::make_unique<ShmemPoseProducer>("RosMapPoses", "RosMapPoses", 1024U, 1024U * sizeof(RobotPose) + 10240U);
//   }
//   startShmem();
// }

// void PoseListener::deallocateShmem()
// {
//   stopShmem();
//   m_odomPoseProducer.reset();
//   m_mapPoseProducer.reset();
//   RCLCPP_INFO(this->get_logger(), "Destructed");
// }

// void PoseListener::stopShmem()
// {
//   if (m_odomPoseProducer) {m_odomPoseProducer->stop();}
//   if (m_mapPoseProducer) {m_mapPoseProducer->stop();}
// }

// void PoseListener::startShmem()
// {
//   if (m_odomPoseProducer && !m_odomPoseProducer->isInternalError()) {m_odomPoseProducer->start();}
//   if (m_mapPoseProducer&& !m_mapPoseProducer->isInternalError()) {m_mapPoseProducer->start();}
// }
 }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto pose_listener_node = std::make_shared<ros2_com::PoseListener>();
  rclcpp::spin(pose_listener_node);
  rclcpp::shutdown();
  return 0;
}