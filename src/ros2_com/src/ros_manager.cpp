#include "ros_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include <signal.h>
#include <sys/wait.h>

using namespace std::chrono_literals;

namespace ros2_com
{

RosManager::RosManager() : RosManager(rclcpp::NodeOptions()){}

RosManager::RosManager(const rclcpp::NodeOptions & t_options)
: Node("ros_manager", t_options)
{
  allocateShmem();
  startShmem();

  m_mapSaver = this->create_client<ros2_com::srv::SaveMap>("/ros2_com/save_map");
  
  m_rosTimer = this->create_wall_timer(
    100ms,
    std::bind(&RosManager::updateHandler, this));
}

RosManager::~RosManager()
{
  deallocateShmem();
  killAll();
  RCLCPP_INFO(this->get_logger(), "Destructed");
}

void RosManager::updateHandler()
{
  if (needAllocateShmem()) allocateShmem();
  getRosFlags();
  updateProcessStates();
  RCLCPP_INFO(this->get_logger(), std::to_string(static_cast<int>(m_saveMapFlag)));

  // if(m_currFlags.saveMap)
    // saveMap();
}

void RosManager::updateProcessStates()
{
  for(size_t i = 0; i < m_flagMap.size(); ++i)
  {
    updateProcessState(static_cast<processId>(i));
  }
}

void RosManager::updateProcessState(const processId & t_processId)
{
  //check if restart requested
  if(m_restartMap[t_processId])
  {
    //if restarting means something's not working right so sending SIGKILL
    sendKill(t_processId);
    if(!isProcessRunning(t_processId))
    {
      m_flagMap[t_processId] = true; //setting flag to true so process gets started
      m_restartMap[t_processId] = false; //reset restart flag
      m_pidMap[t_processId] = 0; //reset process pid
    }
    else return; //if process hasn't died yet safer to try again
  }
  //if is supposed to be running and is running do nothing
  if(m_flagMap[t_processId] && isProcessRunning(t_processId)) return;

  //if isn't supposed to be running and isn't running reset the stop count
  if(!m_flagMap[t_processId] && !isProcessRunning(t_processId))
  {
    m_stopCountMap[t_processId] = 0;
    return;
  }
  
  //if is supposed to be running and doesn't have a valid pid (therefor not running) start process
  if(m_flagMap[t_processId] && !isProcessRunning(t_processId)) 
  {
    startProcess(t_processId);
  }
  //if not supposed to be running, but is running stop process
  else
  {
    if(m_stopCountMap[t_processId] > 10)
      sendKill(t_processId);
    else
      sendStop(t_processId);
  }
}

void RosManager::getRosFlags()
{
  try 
  {
    if (!m_flagConsumer->consumerSize()) return;
    m_currFlags = m_flagConsumer->getAndPop();

    for(size_t i = 0; i < processId::count; ++i)
    {
      processId id = static_cast<processId>(i);
      if(m_currFlags.flagMap[id].second)
      {
        if(m_currFlags.flagMap[id].second > 0) m_flagMap[id] = true;
        else m_flagMap[id] = false;
      }

      if(m_currFlags.restartMap[id].second)
        m_restartMap[id] = true;

      if(m_currFlags.saveMap)
        m_saveMapFlag = true;
    }
  } 
  catch (std::exception & e){}
}

void RosManager::killAll()
{
  for(size_t i = 0; i < m_pidMap.size(); ++i)
  {
    sendKill(static_cast<processId>(i));
  }
}

void RosManager::sendKill(const processId & t_processId)
{
  int status = kill(m_pidMap[t_processId], SIGKILL);
  if(status == 0)
    RCLCPP_INFO(this->get_logger(), "Successfully sent SIGKILL signal");
  else
    RCLCPP_INFO(this->get_logger(), "Error while sending SIGKILL signal");
}

bool RosManager::incompatibleProcesses(const processId & t_processId)
{
  //check if starting localization or mapping
  if(t_processId == processId::localization && isProcessRunning(processId::mapping))
  {
    m_flagMap[processId::mapping] = false; //set mapping flag to false to shut down
    return true; //shouldn't start localization before mapping has shut down
  }
  else if(t_processId == processId::mapping && isProcessRunning(processId::localization))
  {
    m_flagMap[processId::localization] = false; //set localization flag to false to shut down
    return true; //shouldn't start mapping before localization has shut down
  }
  return false;
}

void RosManager::startProcess(const processId & t_processId)
{
  //if trying to launch slam and localization at the same time return;
  if(incompatibleProcesses(t_processId))
    return;

  m_pidMap[t_processId] = fork();

  if (m_pidMap[t_processId] < 0) 
  {
    RCLCPP_INFO(this->get_logger(), "Failed to fork process, trying again");
  } 
  else if (m_pidMap[t_processId] == 0) 
  {
    setsid();
    execl("/bin/sh", "sh", "-c", m_commandMap[t_processId], NULL);
    _exit(1);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Successacefully forked process with pid %d", m_pidMap[t_processId]);
  }
}

void RosManager::sendStop(const processId & t_processId)
{

  int status = kill(m_pidMap[t_processId], SIGINT);
  if(status == 0)
  {
    RCLCPP_INFO(this->get_logger(), "Successfully sent SIGINT signal");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Error while sending SIGINT signal");
  }
  ++m_stopCountMap[t_processId];
}

bool RosManager::isProcessRunning(const processId & t_processId)
{
  //check if valid pid
  if(m_pidMap[t_processId] > 0)
  {
    //check if process running
    if(waitpid(m_pidMap[t_processId], nullptr, WNOHANG) == 0)
      return true;
    else return false;
  }
  else return false;
}

bool RosManager::needAllocateShmem()
{
  return !m_flagConsumer.get();
}

void RosManager::allocateShmem()
{
  if (!m_flagConsumer.get()) {
    //TODO: get from config
    m_flagConsumer = std::make_unique<ShmemFlagConsumer>("RosFlags", "RosFlags", "RosFlagConsumer");
  }
}

void RosManager::deallocateShmem()
{
  stopShmem();
  m_flagConsumer.reset();
}

void RosManager::stopShmem()
{
  if (m_flagConsumer.get()) {m_flagConsumer->stop();}
}

void RosManager::startShmem()
{
  if (m_flagConsumer.get()) {m_flagConsumer->start();}
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
