#include "map_saver.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
}

void MapSaver::saveMapHandler(const std::shared_ptr<ros2_com::srv::SaveMap::Request> request,
          std::shared_ptr<ros2_com::srv::SaveMap::Response> response)
{
  response->success = saveMap(request->filename, true);
}

int MapSaver::saveMap(const std::string &path, const bool saveImage)
{
  m_savingMap = true;
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

  if(saveImage)
    m_mapImage = cv::Mat(m_map->info.height, m_map->info.width, CV_8U, 205);

  std::ofstream binWriter(path + ".bin", std::ios::out | std::ios::binary);
  if(!binWriter) 
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to save map as %s.bin, can't open/create file",
      path.c_str());
    return 0;
  }

  if(saveImage)
    if(!saveMapYamlFile(path))
      return -2;

  //write map info
  m_map->info.origin.position.x -= m_lidarOffset;
  binWriter.write((char *) &m_map->info, sizeof(m_map->info));

  for(size_t i = 0U; i < m_map->info.height * m_map->info.width; ++i)
  {
    binWriter.write((char *) &m_map->data[i], sizeof(int8_t));

    if(saveImage)
      try
      {
        updateImage(i);
      }
      catch(const std::exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), e.what());
        return -1;
      }
      
  }

  binWriter.close();
  if(binWriter.good())
    RCLCPP_INFO(this->get_logger(), "Map bin file Saved");
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Error while saving map");
    return 0;
  }

  if(saveImage)
  {
    RCLCPP_INFO(this->get_logger(), "Saving map as %s.pgm", path.c_str());
    cv::imwrite(path + ".pgm", m_mapImage);
    // cv::imwrite(path + ".png", m_mapImage);
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
  
  m_savingMap = false;
  return 1;
}

void MapSaver::updateImage(const size_t& i)
{
    const size_t row = m_map->info.height - i / m_map->info.width - 1;
    const size_t col = i % m_map->info.width;
    const int8_t map_cell = m_map->data[i];
    // if(map_cell == 0)
    //   RCLCPP_INFO(this->get_logger(), "%d %d %d", x ,y ,map_cell);
    
    if (map_cell >= 0 && map_cell <= 100) 
    {
      if (map_cell <= m_freeThreashold) 
      {
        m_mapImage.at<unsigned char>(row, col) = 254;
      } 
      else if (map_cell >= m_occupiedThreashold) 
      {
        m_mapImage.at<unsigned char>(row, col) = 0;
      }
    }
}

bool MapSaver::saveMapYamlFile(const std::string& t_path)
{
  std::string mapmetadatafile = t_path+ ".yaml";
  RCLCPP_INFO(this->get_logger(), "Writing map metadata to %s", mapmetadatafile.c_str());

  geometry_msgs::msg::Quaternion orientation = m_map->info.origin.orientation;
  tf2::Matrix3x3 mat(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);

  YAML::Emitter e;
  e << YAML::Precision(3);
  e << YAML::BeginMap;
  e << YAML::Key << "image" << YAML::Value << t_path + ".pgm";
  e << YAML::Key << "mode" << YAML::Value << "trinary";
  e << YAML::Key << "resolution" << YAML::Value << m_map->info.resolution;
  e << YAML::Key << "origin" << YAML::Flow << YAML::BeginSeq << m_map->info.origin.position.x <<
    m_map->info.origin.position.y << yaw << YAML::EndSeq;
  e << YAML::Key << "negate" << YAML::Value << 0;
  e << YAML::Key << "occupied_thresh" << YAML::Value << m_occupiedThreashold/100;
  e << YAML::Key << "free_thresh" << YAML::Value << m_freeThreashold/100;

  if (!e.good()) 
  {
    RCLCPP_WARN(this->get_logger(), "YAML writer failed with an error %s", 
      e.GetLastError());
    return false;
  }

  try
  {
    std::ofstream yaml(mapmetadatafile);
    yaml << e.c_str();
    yaml.close();

    if(yaml.good())
      return true;
    else
      return false;
  }
  catch(std::exception e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error while writing map metadata to yaml file: %s", e.what());
    return false;
  }

}

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_com::MapSaver>());
  rclcpp::shutdown();
  return 0;
}