/*!
* @file
* @ingroup CoreLibGroup
* @ingroup SharedMemory
* @ingroup DataStructures
*/

#ifndef SHMEM_POSITION_H
#define SHMEM_POSITION_H

#include <string>
#include <exception>

#include "position.hpp"
#include "shmem_base_new.hpp"

namespace zbot::shmem
{
	/*!
	 * @brief Shared object container for position buffer
	 * @tparam StorageType Only bip::managed_shared_memory and bip::managed_mapped_file types are supported
	*/
	template <class StorageType>
	class SharedPositionObject : public SharedObjectBase<PoseRaw, ConsumerBase<StorageType>, StorageType>
	{
	public:
		using this_type = SharedPositionObject<StorageType>;

		using base_type = SharedObjectBase<PoseRaw, ConsumerBase<StorageType>, StorageType>;
		using typename base_type::consumer_type;
		using typename base_type::ConsumerWeakPtr;
		using typename base_type::ConsumerSharedPtr;

		using Allocator = CustomAllocator<TimeContainer<PoseRaw>, StorageType>;
		using ShmemPosition = Position<Allocator>;

		using Time = typename ShmemPosition::Time;
		using TimeSeries = typename ShmemPosition::TimeSeries;
		using time_type = typename ShmemPosition::time_type;
		using size_type = typename ShmemPosition::size_type;
		using capacity_type = typename ShmemPosition::capacity_type;
		using RobotPoseTime = typename ShmemPosition::RobotPoseTime;

		/*!
		 * @brief Creates an empty position buffer with the specified @p t_allocator allocator with the capacity of @p t_bufferCapacity
		*/
		SharedPositionObject(VoidAllocator<StorageType>& t_allocator, capacity_type t_bufferCapacity)
			: base_type(t_allocator), m_buffer(t_bufferCapacity, t_allocator),
			m_bufferCapacityInput(t_bufferCapacity ? t_bufferCapacity : 1U)
		{}
		/*!
		 * @brief Adds a consumer reference
		 * @param t_consumerName The name of the consumer
		 * @param t_storage Reference to the shared memory segment where the consumer is created
		 * @param[out] t_out Weak pointer reference to the consumer
		 * @return True if the consumer was successfully created, otherwise returns false
		*/
		bool addConsumer(ShmemString<StorageType>&& t_consumerName, StorageType& t_storage, ConsumerWeakPtr& t_out)
		{
			ScopedSharableLock lock(m_mtx);
			try
			{
				// construct element
				auto objTemp = t_storage.construct<consumer_type>(t_consumerName.c_str())(
					boost::forward<ShmemString<StorageType>>(t_consumerName));
				// make shared ptr
				auto objTempSharedPtr = bip::make_managed_shared_ptr<consumer_type>(objTemp, t_storage);
				// push back
				m_consumers.push_back(boost::move(objTempSharedPtr));
				t_out = ConsumerWeakPtr(m_consumers.back());
				return true;
			}
			catch (bip::interprocess_exception& ex)
			{
				std::cout << "SharedTimelineObject failed to add consumer: '" << t_consumerName
					<< "'; Err.: " << ex.get_error_code() << "; " << ex.what() << '\n';
			}
			catch (std::exception& ex)
			{
				std::cout << "SharedTimelineObject failed to add consumer: '" << t_consumerName
					<< "'; Err.: " << ex.what() << '\n';
			}
			return false;
		}
		/*!
		 * @brief The capacity of the buffer
		*/
		capacity_type capacity() const { return m_buffer.capacity(); }
		/*!
		 * @brief The size of the buffer
		*/
		size_type size() const { return m_buffer.size(); }
		/*!
		 * @brief Is the buffer empty
		*/
		bool empty() const { return m_buffer.empty(); }
		/*!
		 * @brief Is the buffer full
		*/
		bool full() const { return m_buffer.full(); }
		/*!
		 * @brief Appends a later robot pose
		 * @param t_pose The pose that is inserted
		 * @param t_tsThe timestamp that corresponds to the pose
		*/
		void append(const RobotPose& t_pose, const time_type& t_time)
		{
			ScopedSharableLock lock(m_mtx);
			m_buffer.append(t_pose, t_time);
		}
		/*!
		 * @brief Get the latest robot pose with its coressponding timestamp
		*/
		RobotPoseTime latest() const
		{
			ScopedSharableLock lock(m_mtx);
			return m_buffer.latest();
		}
		/*!
		 * @brief Get the latest robot pose
		*/
		RobotPose latestPose() const
		{
			ScopedSharableLock lock(m_mtx);
			return m_buffer.latestPose();
		}
		/*!
		 * @brief Get the latest timestamp
		*/
		time_type latestTimestamp() const
		{
			ScopedSharableLock lock(m_mtx);
			return m_buffer.latestTimestamp();
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
			ScopedSharableLock lock(m_mtx);
			return m_buffer.getLinearlyInterpolated(t_ts);
		}

	private:
		/*!
		 * @brief The capacity of the buffer
		*/
		capacity_type m_bufferCapacityInput{ 0U };
		/*!
		 * @brief The underlaying buffer
		*/
		ShmemPosition m_buffer;
	};
}

#endif // !SHMEM_TIMELINE_H