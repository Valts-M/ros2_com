#include "slam_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include <signal.h>
#include <filesystem>
#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "color_codes.hpp"

using namespace std::chrono_literals;

namespace ros2_com
{

SlamController::SlamController() : Node("slam_controller")
{
  m_shmemUtil = std::make_unique<ShmemUtility>(std::vector<ConsProdNames>{ConsProdNames::c_RosFlags, ConsProdNames::c_MapAndPose});
  m_shmemUtil->start();

  createMapSaveDirStructure();
  m_latestMapPath = getLatestMapPath();

  m_mapSaver = this->create_client<ros2_com::srv::SaveMap>("map_saver/save_map");
  
  m_rosTimer = this->create_wall_timer(
    500ms,
    std::bind(&SlamController::updateHandler, this));
}

SlamController::~SlamController()
{
  m_shmemUtil->stop();
  m_shmemUtil.reset();
  RCLCPP_INFO(this->get_logger(), "Destructed");
}

void SlamController::createMapSaveDirStructure()
{
  if(!std::filesystem::exists(m_slamMapsDir))
  {
    if(!std::filesystem::create_directories(m_slamMapsDir))
    {
      RCLCPP_FATAL(this->get_logger(), "Couldn't create map save directory: %s", m_slamMapsDir.c_str());
      exit(EXIT_FAILURE);
    }
  }

  std::filesystem::path tmpSavePath{m_slamMapsDir / "tmp"};
  if(!std::filesystem::exists(tmpSavePath))
  {
    if(!std::filesystem::create_directory(tmpSavePath))
    {
      RCLCPP_FATAL(this->get_logger(), "Couldn't create tmp map save directory: %s", tmpSavePath.c_str());
      exit(EXIT_FAILURE);
    }
  }

  std::filesystem::path numFilePath{m_slamMapsDir / "num.txt"};
  if(!std::filesystem::exists(numFilePath))
  {
     std::ofstream fileWriter(numFilePath);
    if(!fileWriter.is_open())
    {
      RCLCPP_FATAL(this->get_logger(), "Couldn't open file %s for write", numFilePath.c_str());
      exit(EXIT_FAILURE);
    }
    fileWriter << 0;
    fileWriter.flush();
    fileWriter.close();
  }
}

void SlamController::updateHandler()
{
  if(!getSlamMode())
    return;

  if(getMapFromServer() && isSlamRunning())
    stopSlam();

  startSlam();
}

void SlamController::createServerMapDir()
{
  std::filesystem::path serverMapPath{m_slamMapsDir / "server_map"};
  if(std::filesystem::exists(serverMapPath))
    std::filesystem::remove_all(serverMapPath);

  if(!std::filesystem::create_directory(serverMapPath))
  {
    RCLCPP_FATAL(this->get_logger(), "Couldn't create server map save directory: %s", serverMapPath.c_str());
    exit(EXIT_FAILURE);
  }
}

bool SlamController::getMapFromServer()
{
  // RobotPose tmp;
  // if(m_shmemUtil->getMapAndPose(&m_pathToBin, &tmp))
  // {
  //   RCLCPP_INFO(this->get_logger(), "%sCreating img from bin%s", 
  //     m_colorMap[Color::green], m_colorMap[Color::endColor]);
  //   createServerMapDir();

   
  //   return true;
  // }
  return false;
}

bool SlamController::getSlamMode()
{
  auto m_flagConsumer = m_shmemUtil->getShmem<shmem::CBConsumer<SlamMode>>(ConsProdNames::c_RosFlags);
  if(!m_flagConsumer) return false;
  try 
  {
    slamMode = m_flagConsumer->getAndPop();

    return true;
  } 
  catch (std::exception & e)
  {
    return false;
  }
}

void SlamController::startSlam()
{
  slamPID = fork();

  if (slamPID < 0) 
  {
    RCLCPP_WARN(this->get_logger(), "Failed to fork process, trying again");
  } 
  else if (slamPID == 0) 
  {
    setsid();
    switch(slamMode)
    {
      case(SlamMode::localization):
        execl("/bin/python3", "python3", "/opt/ros/foxy/bin/ros2", "launch", "-n", "ros2_com", 
          "kudan.launch.py", NULL);
        break;
      case(SlamMode::mapping):
        {
          const std::string mapParam = "map:=" + m_latestMapPath.string();
          execl("/bin/python3", "python3", "/opt/ros/foxy/bin/ros2", "launch", "-n",
            "ros2_com", "kudan.launch.py", mapParam.c_str(), NULL);
        }
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown process id, exiting subprocess");
        exit(EXIT_FAILURE);
    }
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "%sSuccessacefully forked process with pid %d%s", 
      m_colorMap[Color::green], slamPID, m_colorMap[Color::endColor]);
  }
}

void SlamController::stopSlam()
{
  if(!isSlamRunning())
  {
    return;
  }

  if(slamMode == SlamMode::mapping)
  {
    saveMap();
  }

  int status = kill(slamPID, SIGINT);
  if(status == 0)
  {
    RCLCPP_INFO(this->get_logger(), "%sSuccessfully sent SIGINT to %s%s", 
      m_colorMap[Color::green], toString(t_processId).c_str(), m_colorMap[Color::endColor]);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Error while sending SIGINT to %s", toString(t_processId).c_str());
  }

  while(isSamWorking())
  {
    RCLCPP_INFO(this->get_logger(), "%sWaiting for SLAM to shut down%s", 
      m_colorMap[Color::yellow], toString(t_processId).c_str(), m_colorMap[Color::endColor]);
  }
}

bool SlamController::isSlamRunning()
{
  //check if valid pid
  if(slamPID > 0)
  {
    //check if process running
    if(waitpid(slamPID, nullptr, WNOHANG) == 0)
    {
      return true;
    }
  }
  return false;
}

void SlamController::saveMap()
{
  if (!m_mapSaver->service_is_ready())
  {
    RCLCPP_ERROR(this->get_logger(), "Save map: FAILED (Service not active)");
  }
  else
  {
    std::filesystem::path mapPath = createMapSavePath();
    mapPath.append("map");

    auto request = std::make_shared<ros2_com::srv::SaveMap_Request>();
    request->filename = mapPath;

    auto mapServiceCallback = [&, mapPath](rclcpp::Client<ros2_com::srv::SaveMap>::SharedFuture future)
    { 
      // m_mapSavePending = false;
      auto result = future.get();
      if(result->success == 1)
      {
        RCLCPP_INFO(this->get_logger(), "%sSave map: SUCCESS%s", 
          m_colorMap[Color::green], m_colorMap[Color::endColor]);
        std::filesystem::path mapYamlPath{mapPath};
        mapYamlPath += ".yaml";
        m_latestMapPath = mapYamlPath;
        RCLCPP_INFO(this->get_logger(), "Map saved to %s", m_latestMapPath.c_str());
        
        //remove last loaded server map
        if(std::filesystem::exists(m_slamMapsDir / "server_map"))
          std::filesystem::remove_all(m_slamMapsDir / "server_map");

      }
      else if(result->success == 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Save map: FAILED (Unknown error)");
      }
      else if (result->success == -1)
      {
        RCLCPP_ERROR(this->get_logger(), "Save map: FAILED (Map hasn't been created)");
        // m_saveMapFlag = false;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Save map: FAILED (Shmem not working)");
        // m_saveMapFlag = false;
      }
    };
    auto result = m_mapSaver->async_send_request(request, mapServiceCallback);
  }
}

std::string SlamController::createMapSavePath()
{
  std::filesystem::path numFilePath = m_slamMapsDir;
  numFilePath.append("num.txt");

  std::ifstream fileReader(numFilePath);
  if(!fileReader.is_open())
  {
    RCLCPP_WARN(this->get_logger(), 
      "Couldn't open file %s, saving in %s",
       numFilePath.c_str(), m_slamMapsDir.c_str());
    return m_slamMapsDir;
  }
  int num;
  fileReader >> num;
  fileReader.clear();
  fileReader.close();

  std::filesystem::path saveDir = m_slamMapsDir;
  saveDir.append(std::to_string(++num));

  if(!std::filesystem::create_directory(saveDir))
  {
    RCLCPP_FATAL(this->get_logger(), "Couldn't create map save directory: %s", saveDir.c_str());
    exit(EXIT_FAILURE);
  }

  std::ofstream fileWriter(numFilePath);
  if(!fileWriter.is_open())
  {
    RCLCPP_WARN(this->get_logger(), 
      "Couldn't open file %s for write, saving in %s",
       numFilePath.c_str(), m_slamMapsDir.c_str());
    return m_slamMapsDir;
  }
  fileWriter << num;
  fileWriter.flush();
  fileWriter.close();
  return saveDir;
}

std::filesystem::path SlamController::getLatestMapPath()
{
  std::filesystem::path serverMapPath{m_slamMapsDir / "server_map" / "map.kdlm"};
  std::filesystem::path numFilePath{m_slamMapsDir / "num.txt"};

  if(std::filesystem::exists(serverMapPath))
  {
    return serverMapPath;
  }
  else if(std::filesystem::exists(numFilePath))
  {
    std::ifstream fileReader(numFilePath);
    if(!fileReader.is_open())
    {
      RCLCPP_FATAL(this->get_logger(), 
        "Couldn't open file %s",
        numFilePath.c_str());
      exit(EXIT_FAILURE);
    }

    std::string num;
    fileReader >> num;
    fileReader.clear();
    fileReader.close();

    std::filesystem::path mapPath{m_slamMapsDir / num / "map.kdlm"};
    if(std::filesystem::exists(mapPath))
    {
      RCLCPP_WARN(this->get_logger(), 
        "One or more map files missing at :\"%s\"!",
        mapPath.c_str());
      return "";
    }

    return mapPath / "map.yaml";
  }
  else
  {
    RCLCPP_FATAL(
      this->get_logger(), 
      "The file \"%s\" has been deleted durring runtime",
      numFilePath.c_str());

    exit(EXIT_FAILURE);
  }
}
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto manager_node = std::make_shared<ros2_com::SlamController>();
  rclcpp::spin(manager_node);
  rclcpp::shutdown();
  return 0;
}
