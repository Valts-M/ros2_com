#include "point_cloud_stitcher.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/common/transforms.h>

namespace ros2_com
{
  
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

PointCloudStitcher::PointCloudStitcher() : Node("point_cloud_stitcher"),
m_lidarCloud(new pcl::PointCloud<pcl::PointXYZI>),
m_cameraCloud(new pcl::PointCloud<pcl::PointXYZI>),
m_stitchedCloud(new pcl::PointCloud<pcl::PointXYZI>)
{
  // initParams();
  
  m_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("stitched_points", 10);

  m_tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);

  m_lidarSubscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>
    ("points", 10, std::bind(&PointCloudStitcher::lidarTopicCallback, this, _1));
  m_cameraSubscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>
    ("camera/depth/color/points", 10, std::bind(&PointCloudStitcher::cameraTopicCallback, this, _1));
}

void PointCloudStitcher::cameraTopicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  m_cameraCloudMsg = msg;
}

void PointCloudStitcher::lidarTopicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if(msg->data.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "EMPTY LIDAR POINT CLOUD");
    return;
  }

  if(m_cameraCloudMsg)
  {
    if(getTimeDiff(msg, m_cameraCloudMsg) < 0.03)
    {
      pcl::fromROSMsg(*m_cameraCloudMsg, *m_cameraCloud);
      pcl::fromROSMsg(*msg, *m_lidarCloud);
      geometry_msgs::msg::TransformStamped camToLidar;
      try 
      {
        camToLidar = m_tfBuffer->lookupTransform(
          m_cameraCloudMsg->header.frame_id, msg->header.frame_id,
          msg->header.stamp);
      } 
      catch (const tf2::TransformException & ex) 
      {
        RCLCPP_WARN(
          this->get_logger(), "Could not transform %s to %s: %s", 
            m_cameraCloudMsg->header.frame_id.c_str(), msg->header.frame_id.c_str(),  ex.what());
        return;
      }

      Eigen::Affine3d temp;
      tf2::fromMsg(camToLidar, temp);
      pcl::transformPointCloud(*m_cameraCloud, *m_cameraCloud, temp);
      *m_lidarCloud += *m_cameraCloud;
      publishCloud(*m_lidarCloud);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Time diff is too large!");
      publishCloud(msg);
    }
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "No data from camera");
    publishCloud(msg);
  }
}

void PointCloudStitcher::publishCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  m_publisher->publish(*msg);
}

double PointCloudStitcher::getTimeDiff(const sensor_msgs::msg::PointCloud2::SharedPtr a, 
  const sensor_msgs::msg::PointCloud2::SharedPtr b)
{
  return abs(static_cast<double>(a->header.stamp.sec - b->header.stamp.sec) + 
    1E-9 * static_cast<double>(a->header.stamp.nanosec - b->header.stamp.nanosec));
}

// void PointCloudStitcher::initParams()
// {
//   declare_parameter("lidar_height");
//   declare_parameter("robot_height");
//   declare_parameter("floor_tolerance");
//   declare_parameter("robot_height_tolerance");
//   declare_parameter("image_height");
//   declare_parameter("image_width");
//   declare_parameter("map_resolution_cm");
//   declare_parameter("lidar_blind_radius");

//   m_lidarHeight = get_parameter("lidar_height").as_double();
//   m_robotHeight = get_parameter("robot_height").as_double();
//   m_floorTolerance = get_parameter("floor_tolerance").as_double();
//   m_robotHeightTolerance = get_parameter("robot_height_tolerance").as_double();
//   m_rows = get_parameter("image_height").as_int();
//   m_cols = get_parameter("image_width").as_int();
//   m_mapResolutionCm = get_parameter("map_resolution_cm").as_int();
//   m_lidarBlindRadius = get_parameter("lidar_blind_radius").as_double();

// }

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_com::PointCloudStitcher>());
  rclcpp::shutdown();
  return 0;
}