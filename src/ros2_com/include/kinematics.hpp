#ifndef KINEMATICS_H
#define KINEMATICS_H

#define COLUMBUS
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <math.h>
#include "data_structures/log_data_structures.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace ros2_com
{
class Kinematics
{
public:
  /**
   * @brief Calculates the position and linear and angular velocities nad updates the odometry message
   * 
   * @param input Reactd log input
   * @param output Odometry message to update
   */
  void calcPosAndVelocity(zbot::ReactdLog& input, nav_msgs::msg::Odometry& output);
  double m_yaw{0.0}; //rad

private:

  //TODO: get from config
  // const double m_wheelEncScale{3.4708331619197432411444141689373E-06}; //encTicks * scale = distance
  const double m_leftEncScale{3.446982481197425E-06};
  const double m_rightEncScale{3.451556213079376E-06};
  const double m_wheelDistance{0.60842}; //m
  // const double m_gyroScale{-3.644141592438521E-07};
  const double m_rightGyroScale{-3.615086195934141E-07};
  const double m_leftGyroScale{-3.641771822801296E-07};
  uint32_t m_noMovementCount{0};

  double m_gyroTicCount{0.0};
  double m_gyroBias{0.0};
};
}

#endif //KINEMATICS_H
