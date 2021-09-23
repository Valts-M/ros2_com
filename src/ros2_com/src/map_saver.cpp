#include "map_saver.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace ros2_com
{
MapSaver::MapSaver() : Node("map_saver_server"), m_count(0)
{
  m_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>
    ("map", 10, std::bind(&MapSaver::topicCallback, this, _1));
  m_saveMapService = this->create_service<ros2_com::srv::SaveMap>
    ("ros2_com/save_map", std::bind(&MapSaver::saveMap, this, _1, _2));
}

void MapSaver::topicCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  m_map = msg;
}

void MapSaver::saveMap(const std::shared_ptr<ros2_com::srv::SaveMap::Request> request,
          std::shared_ptr<ros2_com::srv::SaveMap::Response> response)
{
  if(m_map == nullptr)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "ros2_com: Haven't gotten any map data yet");
    response->success=-1;
    return;
  }

  std::string path = request->filename;
  if(path.empty() || path == "") path = "/home/RobotV3/slam_maps/map";

  std::ofstream wf(path + ".bin", std::ios::out | std::ios::binary);
  if(!wf) 
  {
    RCLCPP_WARN(
      this->get_logger(),
      "ros2_com: Failed to save map as %s.bin, can't open file",
      path.c_str());
      response->success=0;
      return;
  }

  wf.write((char *) &m_map->info, sizeof(m_map->info));

  for(uint32_t i=0U; i<m_map->info.height*m_map->info.width; i++)
  {
    wf.write((char *) &m_map->data[i], sizeof(int8_t));
  }

  wf.close();
  if(wf.good())
    RCLCPP_INFO(this->get_logger(), "Map bin file Saved");
  else
  {
    RCLCPP_WARN(this->get_logger(), "Error while saving map");
    response->success=0;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "ros2_com: Saving map as %s.", path.c_str());
  int rc = system(("ros2 run nav2_map_server map_saver_cli -f " + path  + " --ros-args -p map_subscribe_transient_local:=true").c_str());
  rclcpp::sleep_for(std::chrono::seconds(1));
  response->success=1;
}
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_com::MapSaver>());
  rclcpp::shutdown();
  return 0;
}