#include "kinematics.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h"

namespace ros2_com
{ 
  void Kinematics::calcPosAndVelocity(zbot::ReactdLog& input, nav_msgs::msg::Odometry& output)
  {
    const double leftDistance{input.leftEncoder * m_leftEncScale};
    const double rightDistance{input.rightEncoder * m_rightEncScale};
    const double distanceTraveled{(leftDistance + rightDistance) / 2};

    //const double deltaAngle{(rightDistance - leftDistance) / m_wheelDistance};
    const double deltaAngle{m_gyroScale * (input.gyroZ - m_gyroBias)};

    //if moving update message
    if(std::fabs(input.leftEncoder) > 10 || std::fabs(input.rightEncoder) > 10)
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

      output.twist.twist.linear.x = distanceTraveled / input.ts;
      output.twist.twist.angular.z = deltaAngle / input.ts;
    }
    else
    {
      m_gyroTicCount += input.gyroZ;
      m_gyroBias = m_gyroTicCount / ++m_noMovementCount;
    }
  }
}