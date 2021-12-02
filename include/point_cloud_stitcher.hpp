#ifndef POINT_CLOUD_STITCHER_h
#define POINT_CLOUD_STITCHER_h

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

//ros
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>

//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

namespace pcl 
{
  template <typename PointT> void 
  transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, 
                        pcl::PointCloud<PointT> &cloud_out, 
                        const Eigen::Affine3d &transform,
                        bool copy_all_fields = true)
  {
    return (transformPointCloud<PointT, double> (cloud_in, cloud_out, transform, copy_all_fields));
  }
}

namespace tf2 
{
  /** \brief Convert a Pose message transform type to a Eigen Affine3d.
   * This function is a specialization of the toMsg template defined in tf2/convert.h.
   * \param msg The Pose message to convert.
   * \param out The pose converted to a Eigen Affine3d.
   */
  inline
  void fromMsg(const geometry_msgs::msg::TransformStamped& msg, Eigen::Affine3d& out) {
    out = Eigen::Affine3d(
        Eigen::Translation3d(msg.transform.translation.x, 
          msg.transform.translation.y, msg.transform.translation.z) *
        Eigen::Quaterniond(msg.transform.rotation.w,
                          msg.transform.rotation.x,
                          msg.transform.rotation.y,
                          msg.transform.rotation.z));
  }
}

namespace ros2_com
{

class PointCloudStitcher : public rclcpp::Node
{

public:
  PointCloudStitcher();

private:

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_lidarSubscriber{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_cameraSubscriber{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_publisher{nullptr};
  sensor_msgs::msg::PointCloud2 m_stitchedCloudMsg;
  sensor_msgs::msg::PointCloud2::SharedPtr m_cameraCloudMsg;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_lidarCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_cameraCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_stitchedCloud;
  
  std::shared_ptr<tf2_ros::TransformListener> m_tfListener{nullptr};
  geometry_msgs::msg::TransformStamped m_mapLidarMsg;
  std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;

  // void initParams();

  double getTimeDiff(const sensor_msgs::msg::PointCloud2::SharedPtr a, 
    const sensor_msgs::msg::PointCloud2::SharedPtr b);

  void publishCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
  template <typename PointT> void 
  publishCloud(const pcl::PointCloud<PointT>& cloud)
  {
    pcl::toROSMsg(cloud, m_stitchedCloudMsg);
    m_publisher->publish(m_stitchedCloudMsg);
  }

  double m_lidarHeight;
  double m_robotHeight;
  double m_floorTolerance;
  double m_robotHeightTolerance;

  int m_rows;
  int m_cols;
  int m_mapResolutionCm;
  double m_lidarBlindRadius;
  int x_offset;
  int y_offset;

  void lidarTopicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void cameraTopicCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

};
}
#endif //POINT_CLOUD_STITCHER_h