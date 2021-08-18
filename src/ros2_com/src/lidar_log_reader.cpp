#include "lidar_log_reader.hpp"

namespace ros2_com
{
    LidarLogReader::LidarLogReader(){};

    LidarLogReader::LidarLogReader(std::string path)
    {
        m_logFile.open(path, std::ios::in | std::ios::binary);
    }

    void LidarLogReader::init(std::string path)
    {
        m_logFile.open(path, std::ios::in | std::ios::binary);
    }

    LidarLogReader::~LidarLogReader()
    {
        m_logFile.close();
    }

    ouster::OS1Header LidarLogReader::readHeader()
    {
        m_logFile.read((char*)&m_header, ouster::headerBytes);
        
        ouster::OS1Mode &mode = m_header.mode;
        m_height = m_header.mode.columnCount;
        m_width = m_header.mode.frequency;
    }

    ouster::LidarScan LidarLogReader::readScan()
    {
        const int packetsPerScan{64};
        ouster::LidarPacket scan[packetsPerScan+1]{};
        double ts[packetsPerScan+1]{};
        uint8_t lidarBuffer[ouster::lidarPacketBytes+1]{};

        m_logFile.read((char*)&scan[0], ouster::lidarPacketBytes);
        m_logFile.read((char*)&ts[0], sizeof(double));
        u_int16_t current_frame_id = scan[0].azimuths[0].frameId;

        for(int i{1}; i<packetsPerScan-1; ++i)
        {
            m_logFile.read((char*)&scan[i], ouster::lidarPacketBytes);
            m_logFile.read((char*)&ts[i], sizeof(double));
            if(scan[i].azimuths[0].frameId != current_frame_id) break;
        }
    }
} // namespace ros2_com
