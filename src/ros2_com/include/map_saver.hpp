#ifndef MAP_SAVER_H
#define MAP_SAVER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

//ros
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "ros2_com/srv/save_map.hpp"

#include "shmem_util.hpp"
#include <opencv2/opencv.hpp>

namespace ros2_com
{

/**
 * @brief Node for saving the map in both a binary and pgm file
 * 
 */
class MapSaver : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Map Saver object
   * 
   */
  MapSaver();
  /**
   * @brief Destroy the Map Saver object
   * 
   */
  ~MapSaver();

private:
  /**
   * @brief Subscriber to the /map topic
   * 
   */
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_subscriber{nullptr};
  /**
   * @brief Map saver service
   * 
   */
  rclcpp::Service<ros2_com::srv::SaveMap>::SharedPtr m_saveMapService{nullptr};
  /**
   * @brief Shmem util
   * 
   */
  std::unique_ptr<ShmemUtility> m_shmemUtil;
  
  const double m_occupiedThreashold = 0.65;

  const double m_freeThreashold = 0.25;

  /**
   * @brief Callback for the save map service. Will attemt to save the map as a bin and pgm file.
   * 
   * @param t_request Contains a std::string of the full path minus the extention of where to
   * save the map and with what name
   * @param t_response Contains the success statuss
   */
  void saveMapHandler(const std::shared_ptr<ros2_com::srv::SaveMap::Request> t_request,  
          std::shared_ptr<ros2_com::srv::SaveMap::Response> t_response);
  /**
   * @brief Will attempt to save the current map
   * 
   * @param t_path to where to save the map and with what name minus the extention
   * @param t_saveImage bool of whether or not to save the map in the pgm format as well
   * @return int 
   */
  int saveMap(const std::string &t_path, const bool t_saveImage);

  cv::Mat m_mapImage;

  void updateImage(const size_t& i);

  bool saveMapYamlFile(const std::string& t_path);

  /**
   * @brief Pointer to the map message
   * 
   */
  nav_msgs::msg::OccupancyGrid::SharedPtr m_map{nullptr};
  /**
   * @brief Subscription callback to the /map topic. Will save the current map as a temporary
   * file to send to the server
   * 
   * @param msg 
   */
  void topicCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  bool m_savingMap{false};
};
}
#endif //MAP_SAVER_H