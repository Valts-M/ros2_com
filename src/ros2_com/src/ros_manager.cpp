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

  m_mapSaver = this->create_client<ros2_com::srv::SaveMap>("ros2_com/save_map");
  
  m_rosTimer = this->create_wall_timer(
    500ms,
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

  if(getRosFlags())
    setLocalFlags();

  updateProcessStates();

  if(m_saveMapFlag)
    saveMap();

  RCLCPP_INFO(this->get_logger(), "\n\n*********************************************************************************************************************\n");
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
    stopProcess(t_processId);
    if(!isProcessRunning(t_processId))
    {
      m_flagMap[t_processId] = true; //setting flag to true so process gets started
      m_restartMap[t_processId] = false; //reset restart flag
    }
    else return; //if process hasn't stopped yet, try again
  }
  
  if(m_flagMap[t_processId]) 
    startProcess(t_processId);
  else
    stopProcess(t_processId);
}

void RosManager::stopProcess(const processId & t_processId)
{
  //TODO: replace with timeout
  if(m_stopCountMap[t_processId] < 20)
    sendStop(t_processId);  
  else
    sendKill(t_processId);
}

bool RosManager::getRosFlags()
{
  try 
  {
    if (!m_flagConsumer->consumerSize()) return false;
    m_latestFlags = m_flagConsumer->getAndPop();

    return true;
  } 
  catch (std::exception & e)
  {
    return false;
  }
}

void RosManager::setLocalFlags()
{
  for(size_t i = 0; i < processId::count; ++i)
  {
    processId id = static_cast<processId>(i);

    setStateFlag(id);

    if(m_latestFlags.restartMap[id].second)
    {
      m_restartMap[id] = true;
      if(id == processId::mapping)
        m_saveMapFlag = true;
    }

    RCLCPP_INFO(this->get_logger(), "Process %d received %d, set to %d", 
      id, m_latestFlags.flagMap[id].second, m_flagMap[id]);
  }
  if(m_latestFlags.saveMap)
      m_saveMapFlag = true;
}

void RosManager::setStateFlag(const processId & t_processId)
{
  //if != 0; 0 == no change
  if(m_latestFlags.flagMap[t_processId].second)
  {
    if(m_latestFlags.flagMap[t_processId].second > 0)
    {
      //set to active
      m_flagMap[t_processId] = true;

      //if localization set to active, turn off mapping and save map
      if(t_processId == processId::localization)
      {
        m_flagMap[processId::mapping] = false;
        m_saveMapFlag = true;
      }
      //if mapping set to active, turn off localization
      else if(t_processId == processId::mapping)
        m_flagMap[processId::localization] = false;
    }
    else 
    {
      //set to shutdown
      m_flagMap[t_processId] = false;

      //if shutting down mapping, save map
      if(t_processId == processId::mapping)
        m_saveMapFlag = true;
    }
  }
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
    int status = kill(m_pidMap[t_processId], SIGINT);
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
      execl("/bin/python3", "python3", "/opt/ros/foxy/bin/ros2", "launch", "-n", "ros2_com", m_commandMap[t_processId], NULL);
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
  else if(t_processId == processId::mapping && m_saveMapFlag)
  {
    RCLCPP_WARN(this->get_logger(), "Can't stop mapping, map save pending");
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
  else if (!m_mapSaver->wait_for_service())
  {
    RCLCPP_WARN(this->get_logger(), "Can't save map, service is not active yet");
  }
  else if(m_mapSavePending)
  {
    RCLCPP_WARN(this->get_logger(), "Can't save map, a map save is still pending");
  }
  else
  {
    m_mapSavePending = true;
    auto request = std::make_shared<ros2_com::srv::SaveMap_Request>();
    request->filename = m_mapSavePath;

    auto mapServiceCallback = [&](rclcpp::Client<ros2_com::srv::SaveMap>::SharedFuture future)
    { 
      m_mapSavePending = false;
      auto result = future.get();
      if(result->success == 1)
      {
        RCLCPP_INFO(this->get_logger(), "Map saved successacefully!");
        m_slamPathProducer->copyUpdate(m_text);
        m_saveMapFlag = false;
      }
      else if(result->success == 0)
      {
        RCLCPP_WARN(this->get_logger(), "Map saving unsuccessful!");
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Map saving unsuccessful! No map has been created yet!");
        m_saveMapFlag = false;
      }
    };
    auto result = m_mapSaver->async_send_request(request, mapServiceCallback);
  }
}

bool RosManager::needAllocateShmem()
{
  return !m_flagConsumer || !m_slamPathProducer;
}

void RosManager::allocateShmem()
{
  if (!m_flagConsumer) {
    //TODO: get from config
    m_flagConsumer = std::make_unique<ShmemFlagConsumer>("RosFlags", "RosFlags", "RosFlagConsumer");
  }
  if(!m_slamPathProducer)
  {
    m_slamPathProducer = std::make_unique<ShmemSlamMapPathProducer>("SlamMapPath", "SlamMapPath", 1024U * 10U);
  }
  startShmem();
}

void RosManager::deallocateShmem()
{
  stopShmem();
  m_flagConsumer.reset();
  m_slamPathProducer.reset();
}

void RosManager::stopShmem()
{
  if (m_flagConsumer.get()) {m_flagConsumer->stop();}
  if (m_slamPathProducer.get()) {m_slamPathProducer->stop();}
}

void RosManager::startShmem()
{
  if (m_flagConsumer.get()) {m_flagConsumer->start();}
  if (m_slamPathProducer.get()) {m_slamPathProducer->start();}
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
