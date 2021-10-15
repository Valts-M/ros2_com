#ifndef COLOR_CODES_H
#define COLOR_CODES_H

#include <map>

namespace ros2_com
{
  enum Color
  {
    green,
    red,
    yellow,
    blue,
    endColor
  };

  std::map<ros2_com::Color, const char*> m_colorMap
  {
    {green, "\033[92m"},
    {red, "\033[91m"},
    {yellow, "\033[93m"},
    {blue, "\033[94m"},
    {endColor, "\033[0m"}
  };
}

#endif //COLOR_CODES_H