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
    m_odomConsumer = std::make_unique<ShmemConsumer<ReactdLog, Storage>>("LogReader", "RosModuleInput", "asd");
    Run();
  }

  bool OdometryPublisher::noMovement(ReactdLog &t_reacdtdLog)
	{
		return t_reacdtdLog.leftEncoder < 10 && t_reacdtdLog.rightEncoder < 10;
  }

  OdometryPublisher::~OdometryPublisher()
  {
    m_publisher.reset();
    m_odomConsumer.reset();
  }

  double OdometryPublisher::getDistanceTraveled(ReactdLog &t_reactdLog)
  {
    return (t_reactdLog.leftEncoder + t_reactdLog.rightEncoder) * m_encoderScale_2;
  }

  nav_msgs::msg::Odometry OdometryPublisher::createOdomMsg(ReactdLog &t_reactdLog)
  { 
    const double distanceTraveled = getDistanceTraveled(t_reactdLog);
    const double angle = m_gyroScale * (t_reactdLog.gyroZ - m_gyroCorrection);

    m_robotAngle += angle;
    m_robotPosX += distanceTraveled * std::cos(angle);
    m_robotPosY += distanceTraveled * std::sin(angle);

    const double linSpeed = distanceTraveled / t_reactdLog.ts;
    const double angSpeed = angle / t_reactdLog.ts;

    nav_msgs::msg::Odometry message{};

    message.header.set__stamp(rclcpp::Time(m_ts));
    message.twist.twist.angular.z = angSpeed;
    message.twist.twist.linear.x = linSpeed;
    message.pose.pose.position.x = m_robotPosX;
    message.pose.pose.position.y = m_robotPosY;

    tf2::Quaternion temp;
    temp.setRPY(0, 0, angle);
    tf2::convert(temp, message.pose.pose.orientation);

    return message;
  }

  bool OdometryPublisher::reconnect()
  {
    if (!m_odomConsumer.get())
		{
      m_odomConsumer = std::make_unique<ShmemConsumer<ReactdLog, Storage>>("LogReader", "RosModuleInput", "asd");
			return false;
		}

    if(!m_odomConsumer->isMemoryOpen())
    

		if (!m_odomConsumer->isObjectFound()) 
      return m_odomConsumer->findObject();

		return true;
  }

  void OdometryPublisher::Run()
  {
    nav_msgs::msg::Odometry odomMsg{};
    ReactdLog reactdLog{};
    std::chrono::seconds const waitTime = std::chrono::seconds(100U);

    //wait till connected to the other module
    while(!reconnect()) std::cout << "a";
    if(!m_odomConsumer->waitCopy(reactdLog, waitTime)) return;
    m_ts = reactdLog.ts;

    while(true)
    {
      //if no data in 100 seconds break the loop
      if(!m_odomConsumer->waitCopy(reactdLog, waitTime)) break;
      m_ts += reactdLog.ts;

      if(noMovement(reactdLog))
      {
        m_gyroSum += reactdLog.gyroZ;
        m_gyroCorrection = m_gyroSum / ++m_noMovementCount;
      }

      odomMsg = createOdomMsg(reactdLog);

      std::cout << reactdLog.id << "\n";
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
