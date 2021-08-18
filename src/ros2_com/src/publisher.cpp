#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "shmemconsumer.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "publisher.hpp"

using namespace std::chrono_literals;

namespace ros2_com{

    OdometryPublisher::OdometryPublisher() : Node("odometry_publisher")
    {
        m_publisher = this->create_publisher<std_msgs::msg::String>("odom", 10);
        m_timer = this->create_wall_timer(
        500ms, std::bind(&OdometryPublisher::timerCallback, this));
        m_odomConsumer = std::make_unique<ShmemConsumer<OdomData, Storage>>("ros_odometry", "robot_odometry", "");
        m_log.init("");
        m_log.readScan();
    }

    OdometryPublisher::~OdometryPublisher()
    {
        m_publisher.reset();
        m_timer.reset();
        m_odomConsumer.reset();
    }

    void OdometryPublisher::timerCallback()
    {
        // if (!reconnect()) return;

        // {
        //     std::lock_guard<std::mutex> lock(m_data.mtx);
        //     if (!m_data.newData)return;
        //     std::swap(m_data.front, m_data.back);
        //     m_data.newData = false;
        // }

        // auto message = std_msgs::msg::String();
        // message.data = m_data.front;
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        // m_publisher->publish(message);

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

    int main(int argc, char * argv[])
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<OdometryPublisher>());
        rclcpp::shutdown();
        return 0;
    }
}
