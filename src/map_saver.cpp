#include "map_saver.hpp"

#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "map_img_creator.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace ros2_com
{
MapSaver::MapSaver() : Node("map_saver_server")
{
  this->declare_parameter("lidar_x_offset");
  m_lidarOffset = this->get_parameter("lidar_x_offset").as_double();
  m_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>
    ("map", 10, std::bind(&MapSaver::topicCallback, this, _1));
  m_saveMapService = this->create_service<ros2_com::srv::SaveMap>
    ("map_saver/save_map", std::bind(&MapSaver::saveMapHandler, this, _1, _2));
  m_shmemUtil = std::make_unique<ShmemUtility>(std::vector<ConsProdNames>{ConsProdNames::p_MapPath});
  m_shmemUtil->start();
  m_map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
}

MapSaver::~MapSaver()
{
  m_shmemUtil->stop();
  m_shmemUtil.reset();
}

void MapSaver::topicCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if(!m_savingMap) //check if we're not in the progress of saving a map
  {
    m_map = msg;
    saveMap("/home/RobotV3/slam_maps/tmp/map", false);
    // saveMap("/workspaces/RobotV3/ros/src/ros2_com/test", true);
  }
  m_savingMap = false; //should be done by next topic callback
}

void MapSaver::saveMapHandler(const std::shared_ptr<ros2_com::srv::SaveMap::Request> request,
          std::shared_ptr<ros2_com::srv::SaveMap::Response> response)
{
  response->success = saveMap(request->filename, true);
  // m_savingMap = false;
}

int MapSaver::saveMap(const std::string &path, const bool saveImage)
{
  m_savingMap = true;
  if(m_map == nullptr)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Haven't gotten any map data yet");
    m_savingMap = false;
    return -1;
  }

  if(path.empty() || path == "")
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "No map save path specified");
    m_savingMap = false;
    return 0;
  }

  if(saveImage)
    m_mapImage = cv::Mat(m_map->info.height, m_map->info.width, CV_8U, 205);

  std::ofstream binWriter(path + ".bin", std::ios::out | std::ios::binary);
  if(!binWriter) 
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to save map as %s.bin, can't open/create file",
      path.c_str());
    m_savingMap = false;
    return 0;
  }

  if(saveImage)
    if(!MapImageCreator::saveMapYamlFile(path, m_map->info, this))
    {
      m_savingMap = false;
      return -2;
    }

  {
    //write map info
    auto mapInfo = m_map->info;
    mapInfo.origin.position.x -= m_lidarOffset;
    binWriter.write((char *) &mapInfo, sizeof(mapInfo));
  }

  for(size_t i = 0U; i < m_map->info.height * m_map->info.width; ++i)
  {
    binWriter.write((char *) &m_map->data[i], sizeof(int8_t));

    if(saveImage)
      try
      {
        MapImageCreator::updateImage(i, &m_mapImage, m_map.get());
      }
      catch(const std::exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), e.what());
        m_savingMap = false;
        return -1;
      }
  }

  binWriter.close();
  if(binWriter.good())
    RCLCPP_INFO(this->get_logger(), "Map bin file Saved");
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Error while saving map");
    m_savingMap = false;
    return 0;
  }

  if(saveImage)
  {
    RCLCPP_INFO(this->get_logger(), "Saving map as %s.pgm", path.c_str());
    cv::imwrite(path + ".pgm", m_mapImage);
    // cv::imwrite(path + ".png", m_mapImage);
  }
  
  auto p = m_shmemUtil->getShmem<shmem::RawProducer<TextualInfo>>(ConsProdNames::p_MapPath);
  if (!p)
  {
    m_savingMap = false;
    return -2;
  }
  try
  {
    p->copyUpdate(path + ".bin");
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Shmem not working");
    m_savingMap = false;
    return -2;
  }
  
  // m_savingMap = false;
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