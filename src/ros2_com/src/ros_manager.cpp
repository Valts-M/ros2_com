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

  if(m_currFlags.saveMap)
    saveMap();
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
  
  //if is supposed to be running and doesn't have a valid pid (therefor not running) start process
  if(m_flagMap[t_processId]) 
  {
    startProcess(t_processId);
  }
  //if not supposed to be running, but is running stop process
  else
  {
    stopProcess(t_processId);
  }
}

void RosManager::stopProcess(const processId & t_processId)
{
  if(m_stopCountMap[t_processId] < 20)
    sendStop(t_processId);  
  else
    sendKill(t_processId);
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
  if(!isProcessRunning(t_processId))
  {
    RCLCPP_INFO(this->get_logger(), "Process %d NOT running, nothing to kill", t_processId);
    return;
  }
  else
  {
    int status = kill(m_pidMap[t_processId], SIGKILL);
    if(status == 0)
      RCLCPP_INFO(this->get_logger(), "Successfully sent SIGKILL to %d", t_processId);
    else
      RCLCPP_WARN(this->get_logger(), "Error while sending SIGKILL to %d", t_processId);
  }
}

bool RosManager::incompatibleProcesses(const processId & t_processId)
{
  //check if starting localization or mapping
  if(t_processId == processId::localization && isProcessRunning(processId::mapping))
  {
    RCLCPP_WARN(this->get_logger(), 
      "Trying to launch localization while mapping is still active! Shutting down mapping");

    m_flagMap[processId::mapping] = false; //set mapping flag to false to shut down
    return true; //shouldn't start localization before mapping has shut down
  }
  else if(t_processId == processId::mapping && isProcessRunning(processId::localization))
  {
    RCLCPP_WARN(this->get_logger(), 
      "Trying to launch mapping while localization is still active! Shutting down localization");

    m_flagMap[processId::localization] = false; //set localization flag to false to shut down
    return true; //shouldn't start mapping before localization has shut down
  }
  return false;
}

void RosManager::startProcess(const processId & t_processId)
{
  if(isProcessRunning(t_processId))
  {
    RCLCPP_INFO(this->get_logger(), "Process %d running", t_processId);
    return;
  }
  else
  {
    //if trying to launch slam and localization at the same time return;
    if(incompatibleProcesses(t_processId))
      return;

    m_pidMap[t_processId] = fork();

    if (m_pidMap[t_processId] < 0) 
    {
      RCLCPP_INFO(this->get_logger(), "Failed to fork process %d, trying again", t_processId);
    } 
    else if (m_pidMap[t_processId] == 0) 
    {
      setsid();
      execl("/bin/sh", "sh", "-c", m_commandMap[t_processId], NULL);
      _exit(1);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Successacefully forked process %d with pid %d", t_processId, m_pidMap[t_processId]);
    }
  }
}

void RosManager::sendStop(const processId & t_processId)
{
  if(!isProcessRunning(t_processId))
  {
    RCLCPP_INFO(this->get_logger(), "Process %d NOT running", t_processId);
    m_stopCountMap[t_processId] = 0;
    return;
  }
  else
  {
    int status = kill(m_pidMap[t_processId], SIGINT);
    if(status == 0)
    {
      RCLCPP_INFO(this->get_logger(), "Successfully sent SIGINT to %d", t_processId);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Error while sending SIGINT to %d", t_processId);
    }
    ++m_stopCountMap[t_processId];
  }
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

void RosManager::saveMap()
{
  if(!isProcessRunning(processId::mapping))
  {
    RCLCPP_WARN(this->get_logger(), "Can't save map, mapper process is not active");
  }
  else if (m_mapSaver->wait_for_service())
  {
    RCLCPP_WARN(this->get_logger(), "Can't save map, service is not active yet");
  }
  else
  {
    auto request = std::make_shared<ros2_com::srv::SaveMap_Request>();

    auto mapServiceCallback = [&,this](rclcpp::Client<ros2_com::srv::SaveMap>::SharedFuture future)
    { 
      auto result = future.get();
      if(result->success)
        RCLCPP_INFO(this->get_logger(), "Map saved successacefully!");
      else
        RCLCPP_WARN(this->get_logger(), "Map saving unsuccessful!");
    };
    auto result = m_mapSaver->async_send_request(request, mapServiceCallback);
    
    //TODO: figure out how the fuck am I supposed to reset this flag from the callback above
    m_saveMapFlag = false;
  }
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
    startShmem();
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
