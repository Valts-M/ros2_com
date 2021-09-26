#include "ros_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include <signal.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/types.h>

using namespace std::chrono_literals;

namespace ros2_com
{

RosManager::RosManager() : RosManager(rclcpp::NodeOptions()){}

RosManager::RosManager(const rclcpp::NodeOptions & t_options)
: Node("ros_manager", t_options)
{
  allocateShmem();

  m_latestMapsPath = initLatestMapPath();

  m_mapSaver = this->create_client<ros2_com::srv::SaveMap>("ros2_com/save_map");
  m_odomResetter = this->create_client<ros2_com::srv::ResetOdom>("ros2_com/reset_odom");
  m_initialPoseSender = this->create_client<ros2_com::srv::SendInitialPose>("ros2_com/send_initial_pose");
  m_initialPoseSaver = this->create_client<ros2_com::srv::SaveInitialPose>("ros2_com/save_initial_pose");
  
  m_rosTimer = this->create_wall_timer(
    500ms,
    std::bind(&RosManager::updateHandler, this));
}

RosManager::~RosManager()
{
  deallocateShmem();
  stopAll();
  RCLCPP_INFO(this->get_logger(), "Destructed");
}

void RosManager::updateHandler()
{
  if (needAllocateShmem()) allocateShmem();

  if(getRosFlags())
    setLocalFlags();

  updateProcessStates();

  if(m_saveInitialPose)
    saveInitialPose();

  if(m_resetOdomFlag)
    resetOdom();

  if(m_saveMapFlag)
    saveMap();

  if(m_sendInitialPose)
    sendInitialPose();

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
        m_resetOdomFlag = true;
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

      //if localization set to active, turn off mapping, save map and send initial pose
      if(t_processId == processId::localization)
      {
        m_flagMap[processId::mapping] = false;
        m_saveMapFlag = true;
        m_saveInitialPose = true;
        m_sendInitialPose = true;
        m_resetOdomFlag = true;
      }
      //if mapping set to active, turn off localization, reset odometry
      else if(t_processId == processId::mapping)
      {
        m_flagMap[processId::localization] = false;
        m_resetOdomFlag = true;
      }
    }
    else 
    {
      //set to shutdown
      m_flagMap[t_processId] = false;

      //if shutting down mapping, save map
      if(t_processId == processId::mapping)
      {
        m_saveMapFlag = true;
        m_saveInitialPose = true;
      }
    }
  }
}

void RosManager::stopAll()
{
  for(size_t i = 0; i < m_pidMap.size(); ++i)
  {
    sendStop(static_cast<processId>(i));
  }
}

void RosManager::stopProcess(const processId & t_processId)
{
  //TODO: replace with timeout
  if(m_stopCountMap[t_processId] < 20)
    sendStop(t_processId);  
  else
    sendKill(t_processId);
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
      switch(t_processId)
      {
        case(processId::odom):
          execl("/bin/python3", "python3", "/opt/ros/foxy/bin/ros2", "launch", "-n", "ros2_com", "common.launch.py", NULL);
          break;
        case(processId::mapping):
          execl("/bin/python3", "python3", "/opt/ros/foxy/bin/ros2", "launch", "-n", "ros2_com", "mapping.launch.py", NULL);
          break;
        case(processId::localization):
          execl("/bin/python3", "python3", "/opt/ros/foxy/bin/ros2", "launch", "-n", "ros2_com", "localization.launch.py", m_latestMapsPath.c_str(), NULL);
          break;
        case(processId::logging):
          execl("/bin/python3", "python3", "/opt/ros/foxy/bin/ros2", "launch", "-n", "ros2_com", "recording.launch.py", NULL);
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown process id, exiting subprocess");
          exit(EXIT_FAILURE);
      }
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
  else if(t_processId == processId::mapping && m_saveInitialPose)
  {
    RCLCPP_WARN(this->get_logger(), "Can't stop mapping, save initial pose pending");
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
    RCLCPP_WARN(this->get_logger(), "Save map: FAILED (Process not active)");
    m_saveMapFlag = false;
  }
  else if (!m_mapSaver->wait_for_service())
  {
    RCLCPP_WARN(this->get_logger(), "Save map: FAILED (Service not active)");
    m_saveMapFlag = false;
  }
  else if(m_mapSavePending)
  {
    RCLCPP_WARN(this->get_logger(), "Save map: FAILED (Pending response)");
  }
  else
  {

    std::string path = createMapSavePath() + "/map";

    m_mapSavePending = true;

    auto request = std::make_shared<ros2_com::srv::SaveMap_Request>();
    request->filename = path;

    auto mapServiceCallback = [&, path](rclcpp::Client<ros2_com::srv::SaveMap>::SharedFuture future)
    { 
      m_mapSavePending = false;
      auto result = future.get();
      if(result->success == 1)
      {
        RCLCPP_INFO(this->get_logger(), "Save map: SUCCESS");
        TextualInfo info{(path + ".bin").c_str()};
        m_slamPathProducer->copyUpdate(info);
        m_latestMapsPath = "map:=" + path + ".yaml";
        RCLCPP_INFO(this->get_logger(), m_latestMapsPath);

        m_saveMapFlag = false;
      }
      else if(result->success == 0)
      {
        RCLCPP_WARN(this->get_logger(), "Save map: FAILED (Unknown error)");
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Save map: FAILED (Map hasn't been created)");
        m_saveMapFlag = false;
      }
    };
    auto result = m_mapSaver->async_send_request(request, mapServiceCallback);
  }
}

std::string RosManager::createMapSavePath()
{
  std::string numFilePath = m_slamMapsDir + "/num.txt";
  std::ifstream fileReader(numFilePath);
  if(!fileReader.is_open())
  {
    RCLCPP_WARN(this->get_logger(), 
      "Couldn't open file %s, saving in %s",
       numFilePath, m_slamMapsDir);
    return m_slamMapsDir;
  }
  int num;
  fileReader >> num;
  fileReader.clear();
  fileReader.close();

  std::string saveDir = m_slamMapsDir + "/" + std::to_string(++num);
  if(mkdir(saveDir.c_str(), 0777) == -1)
  {
    RCLCPP_WARN(this->get_logger(), 
      "Couldn't create directory %s/%d, saving in %s",
       m_slamMapsDir, num, m_slamMapsDir);
    return m_slamMapsDir;
  }

  std::ofstream fileWriter(numFilePath);
  if(!fileWriter.is_open())
  {
    RCLCPP_WARN(this->get_logger(), 
      "Couldn't open file %s for write, saving in %s",
       numFilePath, m_slamMapsDir);
    return m_slamMapsDir;
  }
  fileWriter << num;
  fileWriter.flush();
  fileWriter.close();
  return saveDir;
}

std::string RosManager::initLatestMapPath()
{
  std::string numFilePath = m_slamMapsDir + "/num.txt";
  std::ifstream fileReader(numFilePath);
  if(!fileReader.is_open())
  {
    RCLCPP_WARN(this->get_logger(), 
      "Couldn't open file %s",
       numFilePath, m_slamMapsDir);
    return m_slamMapsDir;
  }

  int num;
  fileReader >> num;
  fileReader.clear();
  fileReader.close();

  return "map:=" + m_slamMapsDir + "/" + std::to_string(num) + "/map.yaml";
}

void RosManager::resetOdom()
{
  if(!isProcessRunning(processId::odom))
  {
    RCLCPP_WARN(this->get_logger(), "Odometry reset: FAILED (Process not active)");
  }
  else if (!m_odomResetter->wait_for_service())
  {
    RCLCPP_WARN(this->get_logger(), "Odometry reset: FAILED (Service not active)");
  }
  else
  {
    auto result = m_odomResetter->async_send_request(std::make_shared<ros2_com::srv::ResetOdom_Request>());
    RCLCPP_INFO(this->get_logger(), "Odometry reset: SUCCESS");
  }
  m_resetOdomFlag = false;
}

void RosManager::sendInitialPose()
{
  if(!isProcessRunning(processId::odom))
  {
    RCLCPP_WARN(this->get_logger(), "Send initial pose: FAILED (Process not active)");
    m_sendInitialPose = false;
  }
  else if (!m_odomResetter->wait_for_service())
  {
    RCLCPP_WARN(this->get_logger(), "Send initial pose: FAILED (Service not active)");
    m_sendInitialPose = false;
  }
  else if(m_saveInitialPose)
  {
    RCLCPP_WARN(this->get_logger(), "Send initial pose: FAILED (Pending initial pose save)");
  }
  else if(m_sendInitialPosePending)
  {
    RCLCPP_WARN(this->get_logger(), "Send initial pose: FAILED (Pending response)");
  }
  else
  {
    auto request = std::make_shared<ros2_com::srv::SendInitialPose_Request>();
    m_sendInitialPosePending = true;

    auto sendInitialPoseServiceCallback = [&](rclcpp::Client<ros2_com::srv::SendInitialPose>::SharedFuture future)
    { 
      m_sendInitialPosePending = false;
      auto result = future.get();
      if(result->success)
      {
        RCLCPP_INFO(this->get_logger(), "Send initial pose: SUCCESS");
        m_sendInitialPose = false;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Send initial pose: FAILED (Localization not fully active yet)");
      }
    };
    auto result = m_initialPoseSender->async_send_request(request, sendInitialPoseServiceCallback);
  }
}

void RosManager::saveInitialPose()
{
  if(!isProcessRunning(processId::odom))
  {
    RCLCPP_WARN(this->get_logger(), "Save initial pose: FAILED (Process not active)");
    m_saveInitialPose = false;
  }
  else if (!m_initialPoseSaver->wait_for_service())
  {
    RCLCPP_WARN(this->get_logger(), "Save initial pose: FAILED (Service not active)");
    m_saveInitialPose = false;
  }
  else if(m_saveInitialPosePending)
  {
    RCLCPP_WARN(this->get_logger(), "Save initial pose: FAILED (Pending response)");
  }
  else
  {
    auto request = std::make_shared<ros2_com::srv::SaveInitialPose_Request>();
    m_saveInitialPosePending = true;

    auto saveInitialPoseServiceCallback = [&](rclcpp::Client<ros2_com::srv::SaveInitialPose>::SharedFuture future)
    { 
      m_saveInitialPosePending = false;
      auto result = future.get();
      if(result->success)
      {
        RCLCPP_INFO(this->get_logger(), "Save initial pose: SUCCESS");
        m_saveInitialPose = false;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Save initial pose: FAILED (Unknown error)");
      }
    };
    auto result = m_initialPoseSaver->async_send_request(request, saveInitialPoseServiceCallback);
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
