#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

namespace ros2_com
{
  struct RobotConfig
  {
    double leftEncScale;
    double rightEncScale;
    double leftGyroScale;
    double RightGyroScale;
  };
}


#endif