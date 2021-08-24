#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "shmem/shmem_consumer.hpp"

using namespace std::chrono_literals;

namespace ros2_com
{
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

using Storage = boost::interprocess::managed_shared_memory;

class OdometryPublisher : public rclcpp::Node
{
public:
  OdometryPublisher();
  ~OdometryPublisher();
  void Run();

private:
  nav_msgs::msg::Odometry createOdomMsg(ReactdLog &t_reactdLog);
  double estWheelTravel(int &t_encoderVal);
  double getDistanceTraveled(ReactdLog &t_reactdLog);
  double calcEncScale();
  bool noMovement(ReactdLog &t_reactdLog);

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_publisher;
  std::unique_ptr<ShmemConsumer<ReactdLog, Storage>> m_odomConsumer;
  size_t m_count;

  double m_robotPosX{0.0}; //m
  double m_robotPosY{0.0}; //m
  double m_robotAngle{0.0}; //rad
  double m_ts{0.0};

  //For gyro drift correction
  double m_gyroSum{0.0};
  int m_noMovementCount{0};
  double m_gyroCorrection{0.0};


  //TODO: Get these from config later
  const double m_wheelRadius{0.25};
  const double m_wheelDistance{1.2};
  const int m_encoderPPR{2048 * 4}; //encoder pulses per rotation TODO: check if accurate
  const double m_encoderScale{2 * M_PI * m_wheelRadius * m_encoderPPR};
  const double m_encoderScale_2{m_encoderScale / 2};
  const double m_gyroScale{3.733262196325799e-07};

};
}
