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
            LidarLogReader(std::string path);
            void init(std::string path);
            ~LidarLogReader();
            void readScan();
        private:
            std::ifstream m_logFile;

            ouster::LidarPacket m_packet;
            ouster::OS1Header m_header;

            uint8_t m_lidarBuffer[ouster::lidar_packet_bytes + 1], imuBuffer[ouster::imu_packet_bytes + 1];

            uint64_t m_scanNo;
            uint32_t m_width, m_height;
            
            uint8_t m_altitudeIndex;

            void readHeader();
    };
}

#endif //LIDAR_LOG_READER_H