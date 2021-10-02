#ifndef KINEMATICS_H
#define KINEMATICS_H

#define COLUMBUS
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <math.h>

#include "data_structures/protocol.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "robot_config.hpp"

namespace ros2_com
{
class Kinematics
{
public:

  Kinematics();
  Kinematics(const RobotConfig& config);

  /**
   * @brief Calculates the position and linear and angular velocities nad updates the odometry message
   * 
   * @param input Reactd log input
   * @param output Odometry message to update
   */
  void calcPosAndVelocity(zbot::MsgRawStatus& input, nav_msgs::msg::Odometry& output);
  double m_yaw{0.0}; //rad

private:

  const double m_leftEncScale;
  const double m_rightEncScale;
  const double m_rightGyroScale;
  const double m_leftGyroScale;
  uint32_t m_noMovementCount{0};

  double m_gyroTicCount{0.0};
  double m_gyroBias{0.0};
};
}

#endif //KINEMATICS_H
