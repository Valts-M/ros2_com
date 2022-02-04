#include "kinematics.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"

namespace ros2_com
{ 

  Kinematics::Kinematics() : Kinematics::Kinematics(RobotConfig()){}

  Kinematics::Kinematics(const RobotConfig& config) : 
    m_leftEncScale(config.leftEncScale), 
    m_rightEncScale(config.rightEncScale), 
    m_leftGyroScale(config.leftGyroScale), 
    m_rightGyroScale(config.rightGyroScale),
    m_robotCalibration(config.robotCalibration){}

  double Kinematics::getYaw()
  {
    return m_yaw;
  }

  void Kinematics::setYaw(const double value)
  {
    m_yaw = value;
  }

  void Kinematics::resetTicCounts()
  {
    m_leftEncTicCount = 0;
    m_rightEncTicCount = 0;
    m_gyroTicCount = 0;
  }

  void Kinematics::calcPosAndVelocity(const zbot::MsgRawStatus& input, nav_msgs::msg::Odometry& output)
  {
    const double leftDistance{input.encoders[0] * m_leftEncScale};
    const double rightDistance{input.encoders[1] * m_rightEncScale};
    const double distanceTraveled{(leftDistance + rightDistance) / 2};

    //const double deltaAngle{(rightDistance - leftDistance) / m_wheelDistance};
    const int trueGyroZ = input.gyroDelta - m_gyroBias;
    const double deltaAngle{ trueGyroZ < 0 ? trueGyroZ * m_leftGyroScale : trueGyroZ * m_rightGyroScale};

    //if moving update message
    if(std::fabs(input.encoders[0]) > 10 || std::fabs(input.encoders[1]) > 10)
    {
      m_yaw += deltaAngle;
      output.pose.pose.position.x += distanceTraveled * cos(m_yaw);
      output.pose.pose.position.y += distanceTraveled * sin(m_yaw);

      //check if orientation is within bounds
      if( m_yaw >= M_PI)
        m_yaw -= 2 * M_PI;
      else if( m_yaw <= -M_PI)
        m_yaw += 2 * M_PI;
      
      tf2::Quaternion temp;
      temp.setRPY(0, 0, m_yaw);
      tf2::convert(temp, output.pose.pose.orientation);

      output.twist.twist.linear.x = distanceTraveled / input.time;
      output.twist.twist.angular.z = deltaAngle / input.time;

      if(m_robotCalibration)
      {
        m_gyroTicCount += input.gyroDelta;
        m_leftEncTicCount += input.encoders[0];
        m_rightEncTicCount += input.encoders[1];

        std::cout << "LeftEncTicCount = " << m_leftEncTicCount
                  << "RightEncTicCount = " << m_rightEncTicCount
                  << "GyroTicSum = " << m_gyroTicCount;
      }
    }
    else //if not moving recalculate gyro bias
    {
      gyroBuf.push_back(input.gyroDelta);
      m_gyroTicSum += input.gyroDelta;
      if(gyroBuf.full())
      {
        m_gyroTicSum -= gyroBuf.front();
      }
      else 
      {
        ++m_noMovementCount;
      }
      m_gyroBias = m_gyroTicSum / m_noMovementCount;
    }
  }
}