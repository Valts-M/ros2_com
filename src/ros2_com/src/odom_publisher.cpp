#include "odom_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <math.h>

using namespace std::chrono_literals;

namespace ros2_com
{
  OdometryPublisher::OdometryPublisher()
  : Node("odom_publisher"), m_count(0)
  {
    m_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    allocateShmem();
    Run();
  }

  OdometryPublisher::~OdometryPublisher()
  {
    deallocateShmem();
    m_publisher.reset();
  }

  bool OdometryPublisher::getPoseAndVelocity()
	{
		try
		{
			if (!m_poseConsumer->consumerSize()) return false;
			m_poseVelocity = m_poseConsumer->getAndPop();
			//std::cout << std::flush << m_reactdRec.id << '\n';
		}
		catch (std::exception& e)
		{
			std::cout << std::flush << "Failed to get data: " << e.what() << '\n';
			return false;
		}
		return true;
	}

	bool OdometryPublisher::needAllocateShmem()
	{
		return !m_poseConsumer.get();
	}
	
	void OdometryPublisher::allocateShmem()
	{
		if (!m_poseConsumer.get())
		{
      //TODO: get from config
			m_poseConsumer = std::make_unique<ShmemPoseConsumer>("ROS2", "KinematicsOutput", "m_uniqueName");
			m_poseConsumer->start();
		}
	}
	
	void OdometryPublisher::deallocateShmem()
	{
		m_poseConsumer.reset();
	}
	
	void OdometryPublisher::stopShmem()
	{
		// if consumers and producers are alot, then create std::vector<ShmemBase> and start, stop with iterators
		if (m_poseConsumer.get()) m_poseConsumer->stop();
	}
	
	void OdometryPublisher::startShmem()
	{
		if (m_poseConsumer.get()) m_poseConsumer->stop();
	}

  double OdometryPublisher::getDistanceTraveled()
  {
    return m_poseVelocity.robotPose.angle;
  }

  nav_msgs::msg::Odometry OdometryPublisher::createOdomMsg()
  { 
    const double distanceTraveled = getDistanceTraveled();

    const double linSpeed = distanceTraveled / m_poseVelocity.robotPose.ts;
    const double angSpeed = m_poseVelocity.robotPose.angle / m_poseVelocity.robotPose.ts;

    nav_msgs::msg::Odometry message{};

    message.header.set__stamp(rclcpp::Time(m_ts));
    message.twist.twist.angular.z = angSpeed;
    message.twist.twist.linear.x = linSpeed;
    message.pose.pose.position.x = m_poseVelocity.robotPose.x;
    message.pose.pose.position.y = m_poseVelocity.robotPose.y;

    tf2::Quaternion temp;
    temp.setRPY(0, 0, m_poseVelocity.robotPose.angle);
    tf2::convert(temp, message.pose.pose.orientation);

    return message;
  }

 
  void OdometryPublisher::Run()
  {
    nav_msgs::msg::Odometry odomMsg{};
    std::chrono::seconds const waitTime = std::chrono::seconds(100U);

    while(true)
    {
      //if no data in 100 seconds break the loop
      if (needAllocateShmem()) allocateShmem();
      if(!getPoseAndVelocity()) break;
      m_ts += m_poseVelocity.robotPose.ts;

      odomMsg = createOdomMsg();

      m_publisher->publish(odomMsg);
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_com::OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}
