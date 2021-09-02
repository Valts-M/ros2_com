#include <chrono>
#include <functional>
#include <memory>
#include <string>

//ros
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

//robotv3
#include <shmem/shmem_cb_consumer.hpp>
#include "data_structures/log_data_structures.hpp"
#include "/workspaces/RobotV3/ModuleGroups/VilnisGroup/Kinematics/include/kinematics_data_structures.hpp"
#include <robot_pose.hpp>

namespace ros2_com
{

using namespace zbot;

class OdometryPublisher : public rclcpp::Node
{

using Storage = boost::interprocess::managed_shared_memory;
using ShmemPoseConsumer = shmem::ShmemCBConsumer<PoseAndVelocity, shmem::PolicyFifo, Storage>;

public:
  OdometryPublisher();
  ~OdometryPublisher();
  void updateHandler();

private:

  /*!
    * @brief Creates a nav_msg odometry message from the received lidar packets and stores it in m_odomMsg
  */
  void updateOdomMsg();

  /*!
    * @brief Shared pointer to the ros odometry message publisher
  */
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_publisher;

  /*!
    * @brief Unique pointer to the shared memory pose consumer
  */
	std::unique_ptr<ShmemPoseConsumer> m_poseConsumer;

  /*!
    * @brief Stores how many ros messages have been published
  */
  size_t m_count;

  /*!
    * @brief Stores the PoseAndVelocity data from shared memory
  */
  PoseAndVelocity m_poseVelocity{};

  /*!
    * @brief Stores the ros2 odometry mesage
    * @details http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html
  */
  nav_msgs::msg::Odometry m_odomMsg{};

  /*!
    * @brief Stores the timestamp
  */
  double m_ts{0.0};

  /*!
    * @brief Stores the X position of the previous message (m)
  */
  double m_previousX{0.0};

  /*!
    * @brief Stores the Y position of the previous message (m)
  */
  double m_previousY{0.0};

  /*!
    * @brief Stores the angle from the previous message (rad)
  */
  double m_previousAngle{0.0};

  rclcpp::Clock m_rosClock;
  rclcpp::TimerBase::SharedPtr m_rosTimer;

  /*!
    * @brief Gets data from the kinematics producer
    * @return True if the data was successfully recieved, otherwise returns false
  */
  bool getPoseAndVelocity();

  /*!
    * @brief Allocates and starts the shmem smart pointers
  */
  void allocateShmem();
  /*!
    * @brief Deallocates the shmem smart pointers
  */
  void deallocateShmem();
  /*!
    * @brief Starts the shmem smart pointers
  */
  void startShmem();
  /*!
    * @brief Stops the shmem smart pointers
  */
  void stopShmem();
  /*!
    * @brief Checks if the shemem smart pointers are allocated
  */
  bool needAllocateShmem();

};
}
