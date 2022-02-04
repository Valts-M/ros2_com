#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

namespace ros2_com
{
  /**
   * @brief Struct that holds scale factors for robot kinematics
   * 
   */
  struct RobotConfig
  {
    double leftEncScale;
    double rightEncScale;
    double leftGyroScale;
    double rightGyroScale;
    bool robotCalibration;
  };
}


#endif