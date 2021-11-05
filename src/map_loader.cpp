#include "map_loader.hpp"

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

MapLoader::MapLoader() : Node("map_loader")
{
  this->declare_parameter("lidar_x_offset");
  m_lidarOffset = this->get_parameter("lidar_x_offset").as_double();
  m_createMapImgService = this->create_service<ros2_com::srv::CreateMapImg>
    ("map_saver/create_map_img", std::bind(&MapLoader::bin2img, this, _1, _2));
  m_mapImage = cv::Mat(m_map.info.height, m_map.info.width, CV_8U, 205);
}

void MapLoader::bin2img(const std::shared_ptr<ros2_com::srv::CreateMapImg::Request> request,
          std::shared_ptr<ros2_com::srv::CreateMapImg::Response> response)
{
  std::ifstream binReader(request->path, std::ios::in | std::ios::binary);
  if(!binReader) 
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to read %s",
      request->path.c_str());
      response->success=false;
      return;
  }

  //read map info
  binReader.read((char *) &m_map.info, sizeof(m_map.info));
  m_map.info.origin.position.x += m_lidarOffset;
  m_map.data.clear();
  m_map.data.resize(m_map.info.height * m_map.info.width);

  for(size_t i = 0U; i < m_map.info.height * m_map.info.width; ++i)
  {
    binReader.read((char *) &m_map.data[i], sizeof(int8_t));
    try
    {
      MapImageCreator::updateImage(i, &m_mapImage, &m_map);
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), e.what());
      response->success = false;
      return;
    }
  }

  binReader.close();
  if(binReader.good())
    RCLCPP_INFO(this->get_logger(), "Map bin file read successfully");
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Error while reading bin file");
    response->success = false;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Saving map as /home/RobotV3/slam_maps/server_map/map.pgm");
  MapImageCreator::saveMapYamlFile("/home/RobotV3/slam_maps/server_map/map", m_map.info, this);
  cv::imwrite("/home/RobotV3/slam_maps/server_map/map.pgm", m_mapImage);
  response->success = true;
  m_mapImage.setTo(205);
}

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_com::MapLoader>());
  rclcpp::shutdown();
  return 0;
}