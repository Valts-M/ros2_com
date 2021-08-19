#include "lidar_log_reader.hpp"

namespace ros2_com
{
    LidarLogReader::LidarLogReader(){}

    LidarLogReader::LidarLogReader(std::string path)
    {
        m_logFile.open(path, std::ios::in | std::ios::binary);
        readHeader();
    }

    void LidarLogReader::init(std::string path)
    {
        m_logFile.open(path, std::ios::in | std::ios::binary);
        readHeader();
    }

    LidarLogReader::~LidarLogReader()
    {
        m_logFile.close();
    }

    void LidarLogReader::readHeader()
    {
        m_logFile.read((char*)&m_header, ouster::headerBytes);
    }

    void LidarLogReader::readScan()
    {
        ouster::LidarPacket scan[ouster::packetsPerScan+1]{};
        double ts[ouster::packetsPerScan+1]{};
        uint8_t lidarBuffer[ouster::lidarPacketBytes+1]{};

        m_logFile.read((char*)&scan[0], ouster::lidarPacketBytes);
        m_logFile.read((char*)&ts[0], sizeof(double));
        u_int16_t current_frame_id = scan[0].azimuths[0].frameId;

        for(int i{1}; i<ouster::packetsPerScan-1; ++i)
        {
            m_logFile.read((char*)&scan[i], ouster::lidarPacketBytes);
            m_logFile.read((char*)&ts[i], sizeof(double));
            if(scan[i].azimuths[0].frameId != current_frame_id) break;
        }
    }
} // namespace ros2_com