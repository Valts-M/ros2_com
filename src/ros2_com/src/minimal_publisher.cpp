#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include<iostream>
#include<fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>

using namespace std::chrono_literals;
using std::placeholders::_1;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0), tfBroad_(this)
    {
      tfMsg_.child_frame_id = "base_link";
      tfMsg_.header.frame_id = "odom";
      tfMsg_.transform.rotation.x = 0.0;
      tfMsg_.transform.rotation.y = 0.0;
      tfMsg_.transform.rotation.z = 0.0;
      tfMsg_.transform.rotation.w = 1.0;
      tfMsg_.transform.translation.x = 0.0;
      tfMsg_.transform.translation.y = 0.0;
      tfMsg_.transform.translation.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Esda");

      //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 10, std::bind(&MinimalPublisher::topic_callback, this, _1));
      // timer_ = this->create_wall_timer(
      // 50ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      tfMsg_.header.stamp = this->get_clock()->now();
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
      tfBroad_.sendTransform(tfMsg_);
    }

    void topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) const
    {
        std::ofstream wf("map.bin", std::ios::out | std::ios::binary);
        if(!wf) 
        {
          std::cout << "Cannot open file!" << '\n';
          return;
        }

        nav_msgs::msg::MapMetaData info = msg->info;
        RCLCPP_INFO(this->get_logger(), "height: '%d'", info.height);
        RCLCPP_INFO(this->get_logger(), "width: '%d'", info.width);
        RCLCPP_INFO(this->get_logger(), "sec: '%d'", info.map_load_time.sec);
        RCLCPP_INFO(this->get_logger(), "nanosec: '%u'", info.map_load_time.nanosec);
        RCLCPP_INFO(this->get_logger(), "resolution: '%f'", info.resolution);
        RCLCPP_INFO(this->get_logger(), "x: '%f'", info.origin.position.x);
        RCLCPP_INFO(this->get_logger(), "y: '%f'", info.origin.position.y);
        RCLCPP_INFO(this->get_logger(), "z: '%f'", info.origin.position.z);
        RCLCPP_INFO(this->get_logger(), "qx: '%f'", info.origin.orientation.x);
        RCLCPP_INFO(this->get_logger(), "qy: '%f'", info.origin.orientation.y);
        RCLCPP_INFO(this->get_logger(), "qz: '%f'", info.origin.orientation.z);
        RCLCPP_INFO(this->get_logger(), "qw: '%f'", info.origin.orientation.w);

        wf.write((char *) &info, sizeof(info));

        for(int i=0; i<info.height*info.width; i++)
        {
          wf.write((char *) &msg->data[i], sizeof(int8_t));
        }

        wf.close();
        if(wf.good())
          RCLCPP_INFO(this->get_logger(), "saved");
        else
          RCLCPP_INFO(this->get_logger(), "Error saving map");
        
        RCLCPP_INFO(this->get_logger(), "Error saving map");
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscriber_;
    size_t count_;
    tf2_ros::TransformBroadcaster tfBroad_;
    geometry_msgs::msg::TransformStamped tfMsg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}