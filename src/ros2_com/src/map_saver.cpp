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
    ("ros2_com/save_map", std::bind(&MapSaver::saveMapHandler, this, _1, _2));
  m_shmemUtil = std::make_unique<ShmemUtility>(std::vector<ConsProdNames>{ConsProdNames::c_MsgRawStatus});
  m_shmemUtil->start();
}
MapSaver::~MapSaver()
{
  m_shmemUtil->stop();
  m_shmemUtil.reset();
}

void MapSaver::topicCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  m_map = msg;
  saveMap("/home/RobotV3/slam_maps/tmp/map", false);
}

void MapSaver::saveMapHandler(const std::shared_ptr<ros2_com::srv::SaveMap::Request> request,
          std::shared_ptr<ros2_com::srv::SaveMap::Response> response)
{
  response->success = saveMap(request->filename, true);
}

int MapSaver::saveMap(const std::string &path, const bool saveImage)
{
  if(m_map == nullptr)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Haven't gotten any map data yet");
    return -1;
  }

  if(path.empty() || path == "")
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "No map save path specified");
    return 0;
  }

  std::ofstream wf(path + ".bin", std::ios::out | std::ios::binary);
  if(!wf) 
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to save map as %s.bin, can't open/create file",
      path.c_str());
      return 0;
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
    RCLCPP_ERROR(this->get_logger(), "Error while saving map");
    return 0;
  }

  if(saveImage)
  {
    RCLCPP_INFO(this->get_logger(), "Saving map as %s.pgm", path.c_str());
    int rc = system(("ros2 run nav2_map_server map_saver_cli -f " + path  + " --ros-args -p map_subscribe_transient_local:=true").c_str());
    // rclcpp::sleep_for(std::chrono::seconds(1));
  }
  
  auto p = m_shmemUtil->getShmem<RawProducer<TextualInfo>>(ConsProdNames::p_MapPath);
  if(!p) return -2;
  try
  {
    if (!p->isObjectReferenced()) return -2;
    p->copyUpdate(TextualInfo{(path + ".bin").c_str()});
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Shmem not working");
    return -2;
  }
    
  return 1;
}

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_com::MapSaver>());
  rclcpp::shutdown();
  return 0;
}