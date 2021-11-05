#ifndef MAP_LOADER_H
#define MAP_LOADER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

//ros
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "ros2_com/srv/create_map_img.hpp"

#include "shmem_util.hpp"
#include <opencv2/opencv.hpp>

namespace ros2_com
{

/**
 * @brief Node for loading a map from the server
 * 
 */
class MapLoader : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Map Saver object
   * 
   */
  MapLoader();

private:

  rclcpp::Service<ros2_com::srv::CreateMapImg>::SharedPtr m_createMapImgService{nullptr};

  double m_lidarOffset{-9999999.0};


  cv::Mat m_mapImage;

  void bin2img(const std::shared_ptr<ros2_com::srv::CreateMapImg::Request> request,
          std::shared_ptr<ros2_com::srv::CreateMapImg::Response> response);

  /**
   * @brief Map object
   * 
   */
  nav_msgs::msg::OccupancyGrid m_map{};

};
}
#endif //MAP_LOADER_H