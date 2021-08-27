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
#include "/workspaces/RobotV3/ModuleGroups/VilnisGroup/LogReader/include/logdatastructures.hpp"
#include <robotpose.hpp>
#include "/workspaces/RobotV3/ModuleGroups/VilnisGroup/Kinematics/include/kinematicsdatastructures.hpp"

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
  void Run();

private:
  nav_msgs::msg::Odometry createOdomMsg();
  double getDistanceTraveled();

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_publisher;
	std::unique_ptr<ShmemPoseConsumer> m_poseConsumer;
  size_t m_count;
  PoseAndVelocity m_poseVelocity{};
  double m_ts{0.0};

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
