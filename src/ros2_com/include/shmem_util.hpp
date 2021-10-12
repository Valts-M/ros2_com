#ifndef SHMEM_UTILITY_H
#define SHMEM_UTILITY_H

#include <string>

#include <shmem/shmem_container.hpp>
#include <data_structures/common_data_structures.hpp>
#include <data_structures/locald_data.hpp>
#include <data_structures/ros_flags.hpp>

namespace ros2_com
{

using namespace zbot;
using Storage = boost::interprocess::managed_shared_memory;

enum class ConsProdNames
{
  c_MsgRawStatus,
  c_RosFlags,

  p_MapPath,
  p_OdomPose,
  p_MapPose,
  p_LocaldMap
};

//constexpr int toMask(const ConsProdNames& t_enum)
//{
//  return 1 << static_cast<int>(t_enum);
//}

//template<class DataType>
//using RawConsumer = shmem::ShmemRawConsumer<DataType, shmem::Storage>;
//template<class DataType>
//using RawProducer = shmem::ShmemRawProducer<DataType, shmem::Storage>;
//
//template<class DataType, class Policy = shmem::PolicyFifo>
//using CBConsumer = shmem::ShmemCBConsumer<DataType, Policy, shmem::Storage>;
//template<class DataType, class Policy = shmem::PolicyFifo>
//using CBProducer = shmem::ShmemCBProducer<DataType, Policy, shmem::Storage>;
//
//using PositionConsumer = shmem::ShmemPositionConsumer<shmem::Storage>;
//using PositionProducer = shmem::ShmemPositionProducer<shmem::Storage>;

class ShmemUtility : public BaseThread, public ConsumerProducerHelper
{
  using ShmemContainer = shmem::ShmemContainer<Storage>;
  using ShmemType = shmem::ShmemType;
  using MyShmemPointers = shmem::MyShmemPointers;
public:
  ShmemUtility(const std::vector<ConsProdNames>& t_shmemEnums);
  ~ShmemUtility();

  template <
	  class ShmemObject,
	  typename ConsProdEnum = size_t,
	  typename = typename std::enable_if<std::is_scalar<ConsProdEnum>::value, ConsProdEnum>::type
  >
	  ShmemObject* const getShmem(const ConsProdEnum& t_cpName)
  {
	  if (!m_shmems) return nullptr;
	  return std::forward<ShmemObject* const>(m_shmems->getShmem<ShmemObject>(std::forward<const ConsProdEnum&>(t_cpName)));
  }

private:
  inline static const MyConsProdDescriptions m_consProdDescriptions =
  {
      {static_cast<size_t>(ConsProdNames::c_MsgRawStatus), { shmem::createCBConsumer<MsgRawStatus>, ConsProdDescription("MsgRawStatus", "ShmemUtility")}},
      {static_cast<size_t>(ConsProdNames::c_RosFlags), { shmem::createCBConsumer<RosFlags>, ConsProdDescription("RosFlags", "ShmemUtility")}},
      {static_cast<size_t>(ConsProdNames::p_MapPath), { shmem::createRawProducer<TextualInfo>, ConsProdDescription("SlamMapPath")}},
      {static_cast<size_t>(ConsProdNames::p_OdomPose), { shmem::createPositionProducer, ConsProdDescription("RosOdomPoses", 1024U, 1024U * sizeof(RobotPose) + 10240U)}},
      {static_cast<size_t>(ConsProdNames::p_MapPose), { shmem::createPositionProducer, ConsProdDescription("RosMapPoses", 1024U, 1024U * sizeof(RobotPose) + 10240U)}},
      {static_cast<size_t>(ConsProdNames::p_LocaldMap), { shmem::createRawProducer<LocaldMap>, ConsProdDescription("LocaldMap", sizeof(LocaldMap) * 10U)}}
  };

  const MyConsProdDescriptions & getDescriptions() const override
  {
    return ShmemUtility::m_consProdDescriptions;
  }

  bool isConsProdSet(const ConsProdNames& t_enum) const
  {
      return std::find(m_shmemEnums.cbegin(), m_shmemEnums.cend(), t_enum) != m_shmemEnums.cend();
  }

  void run() override;
  void onStart() override;
  void onStop() override;

  void processShmemData();

  std::vector<ConsProdNames> m_shmemEnums;
  std::unique_ptr<ShmemContainer> m_shmems;
};
}

#endif // !SHMEM_UTILITY_H
