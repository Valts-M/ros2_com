#include "kinematics.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"

namespace ros2_com
{ 

  Kinematics::Kinematics() : Kinematics::Kinematics(RobotConfig()){}

  Kinematics::Kinematics(const RobotConfig& config) : m_leftEncScale(config.leftEncScale), 
    m_rightEncScale(config.rightEncScale), 
    m_leftGyroScale(config.leftGyroScale), 
    m_rightGyroScale(config.RightGyroScale){}

  void Kinematics::calcPosAndVelocity(zbot::MsgRawStatus& input, nav_msgs::msg::Odometry& output)
  {
    const double leftDistance{input.encoders[0] * m_leftEncScale};
    const double rightDistance{input.encoders[1] * m_rightEncScale};
    const double distanceTraveled{(leftDistance + rightDistance) / 2};

    //const double deltaAngle{(rightDistance - leftDistance) / m_wheelDistance};
    const double trueGyroZ = input.gyroDelta - m_gyroBias;
    const double deltaAngle{ trueGyroZ < 0 ? trueGyroZ * m_leftGyroScale : trueGyroZ * m_rightGyroScale};

    //if moving update message
    if(std::fabs(input.encoders[0]) > 10 || std::fabs(input.encoders[0]) > 10)
    {
      output.pose.pose.position.x += distanceTraveled * cos(m_yaw + deltaAngle / 2);
      output.pose.pose.position.y += distanceTraveled * sin(m_yaw + deltaAngle / 2);
      m_yaw += deltaAngle;

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
    }
    else //if not moving recalculate gyro bias
    {
      //TODO: replaces with something better. If robot stands still for an hour might run into
      // a large number division problem
      m_gyroTicCount += input.gyroDelta;
      m_gyroBias = m_gyroTicCount / ++m_noMovementCount;
    }
  }
}