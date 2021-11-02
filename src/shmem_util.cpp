#include <data_structures/ros_flags.hpp>

#include "shmem_util.hpp"

namespace ros2_com
{
  ShmemUtility::ShmemUtility(const std::vector<ConsProdNames>& t_shmemEnums)
  : BaseThread("RosShmemUtil"), ConsumerProducerHelper(), m_shmemEnums(t_shmemEnums)
  {}

  ShmemUtility::~ShmemUtility()
  {
      if (m_shmems)
      {
          m_shmems->stop();
          m_shmems->deallocateShmem();
      }
      m_shmems.reset();
  }
  
  void ShmemUtility::onStart()
  {
      if (m_shmems)
      {
          m_shmems->stop();
          m_shmems.reset();
      }
      m_shmems = std::make_unique<ShmemContainer>("RosShmemContainer", &getDescriptions(),
                                                  [&](const size_t& t_enum)->bool
                                                  {
                                                      return isConsProdSet(static_cast<ConsProdNames>(t_enum));
                                                  });
      m_shmems->allocateShmem();
      m_shmems->start();
  }

  void ShmemUtility::onStop()
  {
      if (!m_shmems) return;
      m_shmems->stop();
      m_shmems->deallocateShmem();
  }

  void ShmemUtility::run()
  {
      while (isWorkerEnabled())
      {
          sleepFor(10ms);
      }
  }

  bool ShmemUtility::getForcePose(RobotPose* t_pose)
  {
      auto p = m_shmems->getShmem<shmem::RawConsumer<ForcePose>>(ConsProdNames::c_ForcePoseROS);
      if (!p || !t_pose) return false;
      try
      {
          if (!p->isConsumerReferenced()) return false;
          ForcePose data;
          if (!p->pollCopy(data)) return false;
          *t_pose = data.getRobotPose();
          return true;
      }
      catch (const std::exception& stde)
      {
          std::cerr << "getForcePose failed: " << stde.what() << '\n';
      }
      return false;
  }
}