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

namespace ros2_com
{

class MapSaver : public rclcpp::Node
{
public:
  MapSaver();

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_subscriber{nullptr};
  size_t m_count;
  rclcpp::Service<ros2_com::srv::SaveMap>::SharedPtr m_saveMapService{nullptr};
  void saveMap(const std::shared_ptr<ros2_com::srv::SaveMap::Request> t_request,  
          std::shared_ptr<ros2_com::srv::SaveMap::Response> t_response);
  nav_msgs::msg::OccupancyGrid::SharedPtr m_map{nullptr};
  void topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
};
}
#endif //MAP_SAVER_H