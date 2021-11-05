#ifndef MAP_IMG_CREATOR_H
#define MAP_IMG_CREATOR_H

#include <string>
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include "rclcpp/rclcpp.hpp"

#include "map_img_creator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "yaml-cpp/yaml.h"

namespace ros2_com
{
  namespace MapImageCreator
  {
    inline static constexpr double m_occupiedThreashold{65.0};

    inline static constexpr double m_freeThreashold{25.0};

    void updateImage(const size_t i, cv::Mat* const t_img, const nav_msgs::msg::OccupancyGrid* t_mapData)
    {
      const size_t row = t_mapData->info.height - i / t_mapData->info.width - 1;
      const size_t col = i % t_mapData->info.width;
      
      if (t_mapData->data[i] >= 0 && t_mapData->data[i] <= 100) 
      {
        if (t_mapData->data[i] <= ros2_com::MapImageCreator::m_freeThreashold) 
        {
          t_img->at<unsigned char>(row, col) = 254;
        } 
        else if (t_mapData->data[i] >= ros2_com::MapImageCreator::m_occupiedThreashold) 
        {
          t_img->at<unsigned char>(row, col) = 0;
        }
      }
    }

    bool saveMapYamlFile(const std::string& t_path, 
      const nav_msgs::msg::MapMetaData& t_mapInfo,
      const rclcpp::Node* t_node)
    {
      std::string mapmetadatafile = t_path+ ".yaml";
      RCLCPP_INFO(t_node->get_logger(), "Writing map metadata to %s", mapmetadatafile.c_str());

      geometry_msgs::msg::Quaternion orientation = t_mapInfo.origin.orientation;
      tf2::Matrix3x3 mat(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
      double yaw, pitch, roll;
      mat.getEulerYPR(yaw, pitch, roll);

      YAML::Emitter e;
      e << YAML::Precision(3);
      e << YAML::BeginMap;
      e << YAML::Key << "image" << YAML::Value << t_path + ".pgm";
      e << YAML::Key << "mode" << YAML::Value << "trinary";
      e << YAML::Key << "resolution" << YAML::Value <<  t_mapInfo.resolution;
      e << YAML::Key << "origin" << YAML::Flow << YAML::BeginSeq <<  t_mapInfo.origin.position.x <<
        t_mapInfo.origin.position.y << yaw << YAML::EndSeq;
      e << YAML::Key << "negate" << YAML::Value << 0;
      e << YAML::Key << "occupied_thresh" << YAML::Value << m_occupiedThreashold/100;
      e << YAML::Key << "free_thresh" << YAML::Value << m_freeThreashold/100;

      if (!e.good()) 
      {
        RCLCPP_WARN(t_node->get_logger(), "YAML writer failed with an error %s", 
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
      catch(const std::exception& e)
      {
        RCLCPP_ERROR(t_node->get_logger(), "Error while writing map metadata to yaml file: %s", e.what());
        return false;
      }
    }
  }
}

#endif //MAP_IMG_CREATOR_H