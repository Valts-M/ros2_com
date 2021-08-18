#ifndef LIDAR_LOG_READER_H
#define LIDAR_LOG_READER_H

#include "os1_datatypes.h"
#include "os1_lidar_scan.h"
#include <fstream>

namespace ros2_com
{
    class LidarLogReader
    {
        public:
            LidarLogReader() = default;
            LidarLogReader(std::string path);
            void init(std::string path);
            ~LidarLogReader();
            ouster::OS1Header readHeader();
            ouster::LidarScan readScan();
        private:
            std::ifstream m_logFile;

            ouster::LidarScan m_scan;
            ouster::LidarPacket m_packet;
            ouster::OS1Header m_header;

            uint8_t m_lidarBuffer[ouster::lidar_packet_bytes + 1], imuBuffer[ouster::imu_packet_bytes + 1];

            uint64_t m_scanNo;
            uint32_t m_width, m_height;
            
            uint8_t m_altitudeIndex;
    };
}

#endif //LIDAR_LOG_READER_H