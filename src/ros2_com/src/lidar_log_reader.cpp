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

        for(size_t i{0}; i<ouster::packetsPerScan; ++i)
        {
            m_logFile.read((char*)&ts[i], sizeof(double));
            m_logFile.read((char*)&scan[i], ouster::lidarPacketBytes);
            for(int j=0; j<16; ++j)
            {
                std::cout << scan[i].azimuths[j].measurementId << std::endl;
            }
            // if(scan[i].azimuths[0].frameId != current_frame_id) break;
        }
    }
} // namespace ros2_com