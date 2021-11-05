#ifndef ROS_MANAGER_MAPS_H
#define ROS_MANAGER_MAPS_H

#include <map>
#include <data_structures/ros_flags.hpp>

/**
 * @brief Usefull maps used by ros manager
 * 
 */

namespace ros2_com
{

using namespace zbot;

std::map<zbot::processId, pid_t> m_pidMap
{
  {odom, 0},
  {localization, 0},
  {mapping, 0},
  {logging, 0}
};

std::map<zbot::processId, const char*> m_commandMap
{
  {odom, "common.launch.py"},
  {localization, "localization.launch.py"},
  {mapping, "mapping.launch.py"},
  {logging, "recording.launch.py"}
};

std::map<zbot::processId, int8_t> m_stopCountMap
{
  {odom, 0},
  {localization, 0},
  {mapping, 0},
  {logging, 0}
};

std::map<zbot::processId, bool> m_flagMap
{
  {odom, true},
  {localization, true},
  {mapping, false},
  {logging, false}
};

std::map<zbot::processId, bool> m_restartMap
{
    {odom, false},
    {localization, false},
    {mapping, false},
    {logging, false}
};
}

#endif //ROS_MANAGER_MAPS_H