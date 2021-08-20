#include "odom_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

namespace ros2_com
{
    OdometryPublisher::OdometryPublisher()
    : Node("odom_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&OdometryPublisher::timer_callback, this));
    }

    void OdometryPublisher::timerCallback()
    {
      if (!reconnect()) return;

      {
          std::lock_guard<std::mutex> lock(m_data.mtx);
          if (!m_data.newData)return;
          std::swap(m_data.front, m_data.back);
          m_data.newData = false;
      }

      auto message = std_msgs::msg::String();
      message.data = m_data.front;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      m_publisher->publish(message);

    }

  bool OdometryPublisher::reconnect()
	{
		if (!m_odomConsumer.get())
		{
			m_odomConsumer = std::make_unique<ShmemConsumer<OdomData, Storage>>("ros_odometry", "robot_odometry", "");
			return false;
		}

		if (!m_odomConsumer->isObjectFound()) m_odomConsumer->findObject();

		//odomLogTask();
		return true;
	}

    // bool OdometryPublisher::odomLogTask()
    // {
    // 	if (!m_odomConsumer.get()) return false;
    // 	if (m_odomConsumer->isAsyncWaiting()) return true;

    // 	const bool waiting = m_odomConsumer->asyncWait(
    // 		[&](const int32_t& t_input)
    // 		{
    // 			std::lock_guard<std::mutex> lock(m_data.mtx);
    // 			m_data.back = t_input;
    // 			m_data.newData = true;
    // 		});
    // 	return m_odomConsumer->isAsyncWaiting();
    // }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_com::OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}
