/*!
* @file
* @ingroup CoreLibGroup
* @ingroup SharedMemory
*/
#ifndef SHMEM_POSITION_CONSUMER_HPP
#define SHMEM_POSITION_CONSUMER_HPP

#include "shmem_consumer_base.hpp"
#include "shmem_position.hpp"

namespace zbot::shmem
{
	/*!
	 * @brief Developer interface for easier manipulation with the position buffer as a consumer
	 * @tparam StorageType Only bip::managed_shared_memory and bip::managed_mapped_file types are supported
	*/
	template <class StorageType>
	class ShmemPositionConsumer : public ShmemConsumerBase<SharedPositionObject<StorageType>, StorageType>
	{
	public:
		using SharedObject = SharedPositionObject<StorageType>;
		using ConsumerBase = ShmemConsumerBase<SharedObject, StorageType>;

		using Time = typename SharedObject::Time;
		using TimeSeries = typename SharedObject::TimeSeries;
		using time_type = typename SharedObject::time_type;
		using size_type = typename SharedObject::size_type;
		using capacity_type = typename SharedObject::capacity_type;
		using RobotPoseTime = typename SharedObject::RobotPoseTime;

		/*!
		 * @brief Initializes the base class and some other variables
		 * @param t_producerName The name of the producer that created the shared object
		 * @param t_memoryName The name of the shared memory where the shared object is placed
		 * @param t_consumerName The name of the consumer
		*/
		ShmemPositionConsumer(const std::string& t_producerName, const std::string& t_memoryName, const std::string t_consumerName)
			: ConsumerBase(t_producerName, t_memoryName, t_consumerName)
		{}
		/*!
		 * @brief The capacity of the buffer
		*/
		capacity_type capacity() const
		{
			checkObject();
			auto sharedPtr = SharedObjectSharedPtr(boost::move(referenceObject()));
			return sharedPtr->capacity();
		}
		/*!
		 * @brief The size of the buffer
		*/
		size_type size() const
		{
			checkObject();
			auto sharedPtr = SharedObjectSharedPtr(boost::move(referenceObject()));
			return sharedPtr->size();
		}
		/*!
		 * @brief Is the buffer empty
		*/
		bool empty() const
		{
			checkObject();
			auto sharedPtr = SharedObjectSharedPtr(boost::move(referenceObject()));
			return sharedPtr->empty();
		}
		/*!
		 * @brief Is the buffer full
		*/
		bool full() const
		{
			checkObject();
			auto sharedPtr = SharedObjectSharedPtr(boost::move(referenceObject()));
			return sharedPtr->full();
		}
		/*!
		 * @brief Get the latest robot pose with its coressponding timestamp
		*/
		RobotPoseTime latest() const
		{
			checkObject();
			auto sharedPtr = SharedObjectSharedPtr(boost::move(referenceObject()));
			return sharedPtr->latest();
		}
		/*!
		 * @brief Get the latest robot pose
		*/
		RobotPose latestPose() const
		{
			checkObject();
			auto sharedPtr = SharedObjectSharedPtr(boost::move(referenceObject()));
			return sharedPtr->latestPose();
		}
		/*!
		 * @brief Get the latest timestamp
		*/
		time_type latestTimestamp() const
		{
			checkObject();
			auto sharedPtr = SharedObjectSharedPtr(boost::move(referenceObject()));
			return sharedPtr->latestTimestamp();
		}
		/*!
		 * @brief Get the robot pose that corresponds to the timestamp @p t_ts
		 * @return Robot pose corresponding to the timestamp @p t_ts
		 *
		 * @details If the timeline does not contain an element with a timestamp
		 * that is equal with the @p t_ts then a linear interpolation between
		 * two of the closest elements is calculated as the returning pose.
		*/
		RobotPose getLinearlyInterpolated(time_type t_ts) const
		{
			checkObject();
			auto sharedPtr = SharedObjectSharedPtr(boost::move(referenceObject()));
			return sharedPtr->getLinearlyInterpolated(t_ts);
		}
	};
}
#endif // !SHMEM_POSITION_CONSUMER_HPP