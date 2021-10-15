#ifndef LIDAR_LOG_READER_H
#define LIDAR_LOG_READER_H

#include "ouster/os1_datatypes.h"
#include "ouster/os1_packet.h"
#include <fstream>

namespace ros2_com
{
class LidarLogReader
{
public:
  LidarLogReader();

  /*!
      * @brief Opens up the log filestream and reads the header
      * @param path The full path to the lidar log file in binary format
  */
  LidarLogReader(std::string path);

  /*!
      * @brief Initialized the LidarLogReader, in case the default constructor is used
      * @param path The full path to the lidar log file in binary format
  */
  void init(std::string path);
  ~LidarLogReader();

  /*!
      * @brief Reads a lidar scan into m_lidarBuffer
  */
  void readScan();

private:
  /*!
      * @brief Filestream to the logfile
  */
  std::ifstream m_logFile;

  /*!
      * @brief Stores the last received lidar packet
  */
  ouster::LidarPacket m_packet;

  /*!
      * @brief Stores the log header information
  */
  ouster::OS1Header m_header;

  /*!
      * @brief A buffer for lidar and imu packets
  */
  uint8_t m_lidarBuffer[ouster::lidar_packet_bytes + 1], imuBuffer[ouster::imu_packet_bytes + 1];

  /*!
      * @brief Stores the incremeting scan number
  */
  uint64_t m_scanNo;

  /*!
      * @brief Stores the width of the lidar scan (512, 1024, 2048)
  */
  uint32_t m_width;

  /*!
      * @brief Stores the height of the lidar scan (16, 32, 64, 128)
  */
  uint32_t m_height;

  /*!
      * @brief Reads the header of the log file, parses it and stores it into m_header
  */
  void readHeader();
};
}

#endif //LIDAR_LOG_READER_H
