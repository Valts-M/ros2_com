#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

namespace ros2_com
{
class OdometryPublisher : public rclcpp::Node
{
public:
  OdometryPublisher();

private:
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  size_t count_;

  /*!
  * @brief Data structure for containing reactd log data
  */
  struct ReactdLog
  {
    /*!
     * @brief Timestamp that corresponds to the measurements
    */
    double ts{0.0};
    /*!
     * @brief Gyroscope measurement about Z axis
    */
    int gyroZ{0};
    /*!
     * @brief VectorNAV gyroscope measurement (not used)
    */
    double vGyroZ{0.0};
    /*!
     * @brief Left encoder sensor's measurement
    */
    int leftEncoder{0};
    /*!
     * @brief Right encoder sensor's measurement
    */
    int rightEncoder{0};
    /*!
     * @brief The id of the measurements
    */
    size_t id{0U};
  };
};
}
