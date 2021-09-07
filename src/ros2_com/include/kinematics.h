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
  void calcPosAndVelocity(zbot::ReactdLog& input, nav_msgs::msg::Odometry& output);

private:

  //TODO: get from config
  const double m_wheelEncScale{3.4708331619197432411444141689373E-06}; //encTicks * scale = distance
  const double m_wheelDistance{0.60842}; //m
  double m_yaw{0.0}; //rad
};
}

#endif //KINEMATICS_H
