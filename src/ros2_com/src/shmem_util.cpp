#include <data_structures/ros_flags.hpp>

#include "shmem_util.hpp"

namespace ros2_com
{
  ShmemUtility::ShmemUtility(const std::vector<ConsProdNames>& shmemEnums)
  : shmem::ShmemContainer<shmem::Storage>()
  {
    for(auto& shmemEnum: shmemEnums) m_shmemMask |= toMask(shmemEnum);
    allocateShmem();
  }

  ShmemUtility::~ShmemUtility()
  {
		BaseShmemClass::deallocateShmem();
  }
  
  void ShmemUtility::allocateShmem()
  {
    if(m_shmemMask & toMask(ConsProdNames::c_MsgRawStatus)) BaseShmemClass::allocateShmem<CBConsumer<MsgRawStatus>>(ConsProdNames::c_MsgRawStatus, false);
    if(m_shmemMask & toMask(ConsProdNames::c_RosFlags)) BaseShmemClass::allocateShmem<CBConsumer<RosFlags>>(ConsProdNames::c_RosFlags, false);
    if(m_shmemMask & toMask(ConsProdNames::p_MapPath)) BaseShmemClass::allocateShmem<RawProducer<TextualInfo>>(ConsProdNames::p_MapPath, false);
    if(m_shmemMask & toMask(ConsProdNames::p_OdomPose)) BaseShmemClass::allocateShmem<PositionProducer, true>(ConsProdNames::p_OdomPose, false);
    if(m_shmemMask & toMask(ConsProdNames::p_MapPose)) BaseShmemClass::allocateShmem<PositionProducer, true>(ConsProdNames::p_MapPose, false);
  }
}