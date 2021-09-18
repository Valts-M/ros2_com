#include "ros_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace ros2_com
{

RosManager::RosManager() : RosManager(rclcpp::NodeOptions()){}

RosManager::RosManager(const rclcpp::NodeOptions & t_options)
: Node("ros_manager", t_options), m_count(0)
{
  allocateShmem();
  startShmem();
  
  m_rosTimer = this->create_wall_timer(
    10ms,
    std::bind(&RosManager::updateHandler, this));
}

RosManager::~RosManager()
{
  deallocateShmem();
  RCLCPP_INFO(this->get_logger(), "Destructed");
}

void RosManager::updateHandler()
{
  if (needAllocateShmem()) {allocateShmem();}

  
}


bool RosManager::needAllocateShmem()
{
  return !m_poseConsumer.get();
}

void RosManager::allocateShmem()
{
  if (!m_poseConsumer.get()) {
    //TODO: get from config
    m_poseConsumer = std::make_unique<ShmemPoseConsumer>(
      "GGKReactdLog", "GGKReactdLog",
      "m_uniqueName");
  }
}

void RosManager::deallocateShmem()
{
  stopShmem();
  m_poseConsumer.reset();
}

void RosManager::stopShmem()
{
  if (m_poseConsumer.get()) {m_poseConsumer->stop();}
}

void RosManager::startShmem()
{
  if (m_poseConsumer.get()) {m_poseConsumer->start();}
}

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto manager_node = std::make_shared<ros2_com::RosManager>();
  rclcpp::spin(manager_node);
  rclcpp::shutdown();
  return 0;
}
