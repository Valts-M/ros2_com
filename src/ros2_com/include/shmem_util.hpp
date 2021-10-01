#ifndef SHMEM_UTILITY_H
#define SHMEM_UTILITY_H

#include <string>

// #include <helper.hpp>
#include <shmem/shmem_container.hpp>
#include <shmem/shmem_raw_producer.hpp>
#include <shmem/shmem_raw_consumer.hpp>
#include <shmem/shmem_position_producer.hpp>
#include <shmem/shmem_position_consumer.hpp>
#include <shmem/shmem_cb_producer.hpp>
#include <shmem/shmem_cb_consumer.hpp>
#include <shmem/shmem_timeline_producer.hpp>
#include <shmem/shmem_timeline_consumer.hpp>
#include <data_structures/protocol.hpp>
#include <data_structures/common_data_structures.hpp>

namespace ros2_com
{

using namespace zbot;
  
enum class ConsProdNames
{
  c_MsgRawStatus,
  c_RosFlags,

  p_MapPath,
  p_OdomPose,
  p_MapPose
};

constexpr int toMask(const ConsProdNames& t_enum)
{
  return 1 << static_cast<int>(t_enum);
}

template<class DataType>
using RawConsumer = shmem::ShmemRawConsumer<DataType, shmem::Storage>;
template<class DataType>
using RawProducer = shmem::ShmemRawProducer<DataType, shmem::Storage>;

template<class DataType, class Policy = shmem::PolicyFifo>
using CBConsumer = shmem::ShmemCBConsumer<DataType, Policy, shmem::Storage>;
template<class DataType, class Policy = shmem::PolicyFifo>
using CBProducer = shmem::ShmemCBProducer<DataType, Policy, shmem::Storage>;

using PositionConsumer = shmem::ShmemPositionConsumer<shmem::Storage>;
using PositionProducer = shmem::ShmemPositionProducer<shmem::Storage>;

class ShmemUtility : public shmem::ShmemContainer<shmem::Storage>
{
  using BaseShmemClass = shmem::ShmemContainer<shmem::Storage>;
  using BaseConsProdClass = BaseShmemClass::ConsumerProducerHelper;

public:
  ShmemUtility(const std::vector<ConsProdNames>& shmemEnums);
  ~ShmemUtility();

private:
  inline static const MyConsProdDescriptions m_consProdDescriptions =
  {
			{static_cast<int>(ConsProdNames::c_MsgRawStatus), ConsProdDescription("MsgRawStatus", "ShmemUtility")},
      {static_cast<int>(ConsProdNames::c_RosFlags), ConsProdDescription("RosFlags", "ShmemUtility")},
      {static_cast<int>(ConsProdNames::p_MapPath), ConsProdDescription("SlamMapPath")},
      {static_cast<int>(ConsProdNames::p_OdomPose), ConsProdDescription("RosOdomPoses", 1024U, 1024U * sizeof(RobotPose) + 10240U)},
      {static_cast<int>(ConsProdNames::p_MapPose), ConsProdDescription("RosMapPoses", 1024U, 1024U * sizeof(RobotPose) + 10240U)}
  };

  const MyConsProdDescriptions & getDescriptions() const override
  {
    return ShmemUtility::m_consProdDescriptions;
  }

  void allocateShmem() override;
  int m_shmemMask{0};

};
}

#endif // !SHMEM_UTILITY_H
