#ifndef OS1_DATATYPES_H
#define OS1_DATATYPES_H

#include <iostream>
#include <cstdint>
#include <math.h>


namespace ouster
{

  enum class PacketType: uint8_t
  {
    LiDAR = 0U,
    IMU = 1U,
    UNKNOWN = 2U
  };

  static std::string to_string(PacketType t_type)
  {
    switch (t_type) {
      case PacketType::LiDAR: return std::string("LiDAR");
      case PacketType::IMU: return std::string("IMU");
      default: return std::string("UNKNOWN");
    }
  }
#pragma pack(1)
  struct OS1Version
  {
    uint8_t major {0U};
    uint8_t minor {0U};
    uint8_t patch {0U};
    inline bool isValid() {return major == 1 && minor == 13;}
  };
  struct OS1Mode
  {
    uint16_t columnCount {0U};
    uint8_t frequency {0U};
    inline bool isValid()
    {
      // possible modes are: 512x10; 512x20; 1024x10; 1024x20; 2048x10
      if (columnCount != 512U && columnCount != 1024U && columnCount != 2048U) {return false;}
      if (frequency != 10U && frequency != 20U) {return false;}
      if (frequency == 20U && columnCount == 2048) {return false;}
      return true;
    }
    constexpr double AzimuthDeltaTimestap()
    {
      return 1.0 / static_cast < double > (frequency) / static_cast < double > (columnCount);
    }
  };
  struct OS1Header
  {
    char hostname[255];
    char serial_number[255];
    OS1Version firmware;
    OS1Mode mode;
    uint8_t radians {1U};
    double azimuth_angles[64];
    double atitude_angles[64];
    double imu_to_sensor[16];
    double lidar_to_sensor[16];
    PacketType packetType {PacketType::UNKNOWN};

    inline bool isValidHostname() {return true;}               // how to validate this?
    inline bool isValidSerialNumber() {return true;}               // how to validate this?
    inline bool isValidFirmware() {return firmware.isValid();}
    inline bool isValidMode() {return mode.isValid();}
    inline bool isValidAzimuthAndAtitudeAngles()
    {
      const double toRad = M_PI / 180.0;
      auto isValidAngle = [&toRad](const double & angle, const bool & radians)
      {
        if (radians) {return (-90.0) * toRad <= angle && angle <= 90.0 * toRad;} else {
          return (-90.0) <= angle && angle <= 90.0;
        }
      };
      const bool rads = static_cast < bool > (radians);
      for (auto & azimuth : azimuth_angles) {
        if (!isValidAngle(azimuth, rads)) {return false;}}
      for (auto & atitude : atitude_angles) {
        if (!isValidAngle(atitude, rads)) {return false;}}
      return true;
    }
    inline bool isValidImuAndLidarMatrices()
    {
      if (!(imu_to_sensor[0] == 1.0 && imu_to_sensor[5] == 1.0 &&
        imu_to_sensor[10] == 1.0 && imu_to_sensor[15] == 1.0)) {return false;}

      if (!(lidar_to_sensor[0] == -1.0 && lidar_to_sensor[5] == -1.0 &&
        lidar_to_sensor[10] == 1.0 && lidar_to_sensor[15] == 1.0)) {return false;}

      if (!(lidar_to_sensor[1] == 0.0 && lidar_to_sensor[2] == 0.0 && lidar_to_sensor[3] == 0.0 &&
        lidar_to_sensor[4] == 0.0 && lidar_to_sensor[6] == 0.0 && lidar_to_sensor[7] == 0.0 &&
        lidar_to_sensor[8] == 0.0 && lidar_to_sensor[9] == 0.0 &&
        lidar_to_sensor[12] == 0.0 && lidar_to_sensor[13] == 0.0 && lidar_to_sensor[14] == 0.0))
      {
        return false;
      }

      return true;
    }
    inline bool isPacketTypeValid()
    {
      return packetType == PacketType::LiDAR || packetType == PacketType::IMU;
    }
    inline bool isValid()
    {
      if (!isValidHostname()) {
        std::cout << "OS1Header is not valid: bad hostname\n";
        return false;
      }
      if (!isValidSerialNumber()) {
        std::cout << "OS1Header is not valid: bad serial number\n";
        return false;
      }
      if (!isValidFirmware()) {
        std::cout << "OS1Header is not valid: bad firmware\n";
        return false;
      }
      if (!isValidMode()) {
        std::cout << "OS1Header is not valid: bad LiDAR mode\n";
        return false;
      }
      if (!isValidAzimuthAndAtitudeAngles()) {
        std::cout << "OS1Header is not valid: bad intrinsic angles\n";
        return false;
      }
      if (!isValidImuAndLidarMatrices()) {
        std::cout << "OS1Header is not valid: bad IMU/LiDAR transformation matrices\n";
        return false;
      }
      if (!isPacketTypeValid()) {
        std::cout << "OS1Header is not valid: bad packet type\n";
        return false;
      }

      return true;
    }
  };
  const size_t headerBytes = sizeof(OS1Header);       // 1798
  const size_t lidar_packet_bytes = 12608;
  const size_t imu_packet_bytes = 48;
  const size_t packetsPerScan = 64;

  struct LidarPacketChannel
  {
    uint32_t range;
    uint16_t signal;
    uint16_t reflectivity;
    uint16_t empty;
    uint16_t noise;
  };

  struct LidarPacketAzimuth
  {
    uint64_t timestamp;
    uint16_t measurementId;
    uint16_t frameId;
    uint32_t encoderCount;
    LidarPacketChannel channels[64];
    uint32_t status;
  };
  struct LidarPacket
  {
    LidarPacketAzimuth azimuths[16];
  };

  const size_t lidarPacketChannelBytes = sizeof(LidarPacketChannel);
  const size_t LidarPacketAzimuthBytes = sizeof(LidarPacketAzimuth);
  const size_t lidarPacketBytes = sizeof(LidarPacket);

  struct ImuPacketAcceleration
  {
    float x;
    float y;
    float z;
  };
  struct ImuPacketAngularVelocity
  {
    float x;
    float y;
    float z;
  };
  struct ImuPacket
  {
    uint64_t timestampImu;
    uint64_t timestampAcc;
    uint64_t timestampGyro;
    ImuPacketAcceleration acceleration;
    ImuPacketAngularVelocity angularVelocity;
  };

  const size_t imuPacketAccelerationBytes = sizeof(ImuPacketAcceleration);
  const size_t imuPacketAngularVelocityBytes = sizeof(ImuPacketAngularVelocity);
  const size_t imuPacketBytes = sizeof(ImuPacket);

#pragma pack()

}

#endif // !OS1_DATATYPES_H
