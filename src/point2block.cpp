#include "point2block.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/common/transforms.h>

namespace ros2_com
{
  
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

Point2Block::Point2Block() : Node("point2block"),
m_unfilteredCloud(new pcl::PointCloud<pcl::PointXYZ>),
m_filteredCloud(new pcl::PointCloud<pcl::PointXYZ>),
m_rotatedCloud(new pcl::PointCloud<pcl::PointXYZ>)
{
  initParams();
  m_clearMap = cv::Mat(m_rows, m_cols, CV_8U);
  m_obstacleMap = cv::Mat(m_rows, m_cols, CV_8U);
  m_clearMap.setTo(127U);
  m_obstacleMap.setTo(0U);

  x_offset = m_rows * m_mapResolutionCm / 2;
  y_offset = m_cols * m_mapResolutionCm / 2;
  
  m_shmemUtil = std::make_unique<ShmemUtility>(std::vector<ConsProdNames>{ConsProdNames::p_LocaldMap});
  m_shmemUtil->start();

  m_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);

  m_tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);

  m_pcSubscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>
    ("points", 10, std::bind(&Point2Block::pcTopicCallback, this, _1));
  m_lsSubscriber = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&Point2Block::lsTopicCallback, this, _1));
}

Point2Block::~Point2Block()
{
  m_shmemUtil->stop();
  m_shmemUtil.reset();
}

void Point2Block::pcTopicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const double scanTs = Helper::getTimeStamp();
  auto localdMapProducer = m_shmemUtil->getShmem<shmem::RawProducer<LocaldMap>>(ConsProdNames::p_LocaldMap);
  if (!localdMapProducer)
  {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Locald map producer not available!");
      return;
  }
  
  try 
  {
    m_mapLidarMsg = m_tfBuffer->lookupTransform(
      "map", "laser_sensor_frame",
      tf2::TimePointZero);
  } 
  catch (const tf2::TransformException & ex) 
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Could not transform map to laser_sensor_frame: %s", ex.what());
    return;
  }

  if(msg->data.empty())
    RCLCPP_ERROR(this->get_logger(), "EMPTY POINT CLOUD");

  geometry_msgs::msg::Quaternion* quat = &m_mapLidarMsg.transform.rotation;

  pcl::fromROSMsg(*msg, *m_unfilteredCloud);

  for(size_t i = 0; i < m_unfilteredCloud->points.size(); ++i)
  {
    pcl::PointXYZ point = m_unfilteredCloud->points.at(i);

    if(std::abs(point.x) < 4.2426)
      if(std::abs(point.y) < 4.2426)
        m_filteredCloud->points.push_back(point);
  }

  pcl::transformPointCloud(*m_filteredCloud, *m_rotatedCloud, Eigen::Vector3f::Zero(), 
    Eigen::Quaternionf(quat->w, quat->x, quat->y, quat->z));

  // RCLCPP_WARN(this->get_logger(), "msg:%d, filtered:%d, unfiltered:%d, rotated:%d",
  //   msg->data.size(), m_filteredCloud->size(), m_unfilteredCloud->size(), m_rotatedCloud->points.size());

  m_filteredCloud->points.clear();
  int zeroes = 0;

  for (size_t i = 0; i < m_rotatedCloud->points.size(); ++i)
  {
      pcl::PointXYZ point = m_rotatedCloud->points.at(i);

      if (std::abs(point.x) < m_lidarBlindRadius && std::abs(point.y) < m_lidarBlindRadius)
      {
          ++zeroes;
          continue;
      }

      if (std::abs(point.x) < 3.0)
          if (std::abs(point.y) < 3.0)
          {
              m_filteredCloud->points.push_back(point);

              updateObstacleImage(point);
          }
  }

  // RCLCPP_ERROR(this->get_logger(), "msg:%d, filtered:%d, unfiltered:%d, rotated:%d, zeroes: %d",
  //   msg->data.size(), m_filteredCloud->size(), m_unfilteredCloud->size(), m_rotatedCloud->points.size(), zeroes);

  makeClearImage();

  //pcl::toROSMsg(*m_filteredCloud, m_filteredCloudMsg);
  //m_filteredCloudMsg.header = msg->header;
  //// m_filteredCloudMsg.header.stamp = this->now();
  //m_publisher->publish(m_filteredCloudMsg);

  tf2::Quaternion tempQuat;
  tf2::convert(m_mapLidarMsg.transform.rotation, tempQuat);
  tf2::Matrix3x3 tempMatrix(tempQuat);
  double roll, pitch, yaw;
  tempMatrix.getEulerYPR(yaw, pitch, roll);

  m_pose.x() = m_mapLidarMsg.transform.translation.x;
  m_pose.y() = m_mapLidarMsg.transform.translation.y;
  m_pose.yaw() = yaw;
  const double msgTs = static_cast<double>(m_mapLidarMsg.header.stamp.sec) + 1e-9 * static_cast<double>(m_mapLidarMsg.header.stamp.nanosec);

  //RCLCPP_INFO(this->get_logger(), "scanTs=%.5f, msgTs=%.5f, tsDiff=%.5f, x=%.2f, y=%.2f, yaw=%.2f", scanTs, msgTs, scanTs - msgTs, m_pose.x(), m_pose.y(), m_pose.yawDeg());

  try
  {
    localdMapProducer->copyUpdate(LocaldMap{ msgTs, m_clearMap.data, m_obstacleMap.data, m_rows, m_cols, PoseRaw(m_pose) });
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Shmem not working: %s", e.what());
    return;
  }

  // cv::imwrite("/code/RobotV3/ros/src/ros2_com/obstacles.png", m_obstacleMap);
  // cv::imwrite("/code/RobotV3/ros/src/ros2_com/map.png", m_clearMap);
    
  m_clearMap.setTo(127);
  m_obstacleMap.setTo(0U);
  m_filteredCloud->clear();
  m_rotatedCloud->clear();
}

void Point2Block::initParams()
{
  this->declare_parameter("lidar_height");
  this->declare_parameter("robot_height");
  this->declare_parameter("floor_tolerance");
  this->declare_parameter("robot_height_tolerance");
  this->declare_parameter("image_height");
  this->declare_parameter("image_width");
  this->declare_parameter("map_resolution_cm");
  this->declare_parameter("lidar_blind_radius");

  m_lidarHeight = this->get_parameter("lidar_height").as_double();
  m_robotHeight = this->get_parameter("robot_height").as_double();
  m_floorTolerance = this->get_parameter("floor_tolerance").as_double();
  m_robotHeightTolerance = this->get_parameter("robot_height_tolerance").as_double();
  m_rows = this->get_parameter("image_height").as_int();
  m_cols = this->get_parameter("image_width").as_int();
  m_mapResolutionCm = this->get_parameter("map_resolution_cm").as_int();
  m_lidarBlindRadius = this->get_parameter("lidar_blind_radius").as_double();

}

void Point2Block::lsTopicCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  m_laserScan = msg;
}

void Point2Block::makeClearImage()
{
  const cv::Point startPoint{m_rows / 2, m_cols / 2};

  tf2::Quaternion tempQuat;
  tf2::convert(m_mapLidarMsg.transform.rotation, tempQuat);
  const tf2::Matrix3x3 tempMatrix(tempQuat);
  double roll, pitch, yaw;
  tempMatrix.getEulerYPR(yaw, pitch, roll);

  for(size_t i = 0; i < m_laserScan->ranges.size(); ++i)
  {
      float range = m_laserScan->ranges.at(i) * 100 / m_mapResolutionCm; //in px
      const float angle = m_laserScan->angle_min + i * m_laserScan->angle_increment;
      if(m_laserScan->ranges.at(i) < m_lidarBlindRadius  || m_laserScan->ranges.at(i) == INFINITY)
        if(angle > -M_PI_2 && angle < M_PI_2)
          range = 420;     
        else 
          continue;
      const int x = std::rint(std::cos(angle + yaw) * range) + m_rows/2;
      const int y = -std::rint(std::sin(angle + yaw) * range) + m_cols/2;
      const cv::Point endPoint{x, y};
      cv::line(m_clearMap, startPoint, endPoint, 255U, 1);
  }
}

void Point2Block::updateObstacleImage(const pcl::PointXYZ& t_point)
{
  const cv::Point endPoint{static_cast<int>(t_point.x * 100 + x_offset) / m_mapResolutionCm, 
    static_cast<int>(-t_point.y * 100 + y_offset) / m_mapResolutionCm};

  if(t_point.z <= -(m_lidarHeight + m_floorTolerance))
    m_obstacleMap.at<unsigned char>(endPoint) |= ProjectionTypes::FALL;

  else if(t_point.z > -(m_lidarHeight + m_floorTolerance) && t_point.z <= -(m_lidarHeight - m_floorTolerance))
    m_obstacleMap.at<unsigned char>(endPoint) |= ProjectionTypes::FLOOR;

  else if(t_point.z > -(m_lidarHeight - m_floorTolerance) && t_point.z <= m_robotHeight + m_robotHeightTolerance)
    m_obstacleMap.at<unsigned char>(endPoint) |= ProjectionTypes::OBSTACLE;
  else
    m_obstacleMap.at<unsigned char>(endPoint) |= ProjectionTypes::TOO_HIGH;
}

}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_com::Point2Block>());
  rclcpp::shutdown();
  return 0;
}