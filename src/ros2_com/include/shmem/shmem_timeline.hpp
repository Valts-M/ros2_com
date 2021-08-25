/*!
* @file
* @ingroup CoreLibGroup
* @ingroup SharedMemory
* @ingroup DataStructures
*/

#ifndef SHMEM_TIMELINE_H
#define SHMEM_TIMELINE_H

#include <string>
#include <exception>

#include "timeline.hpp"
#include "shmem_base_new.hpp"

namespace zbot::shmem
{
	/*!
	 * @brief Shared object container for timeline buffer
	 * @tparam T The raw data type for every timeline buffer element
	 * @tparam StorageType Only bip::managed_shared_memory and bip::managed_mapped_file types are supported
	*/
	template <class T, class StorageType>
	class SharedTimelineObject : public SharedObjectBase<T, ConsumerBase<StorageType>, StorageType>
	{
	public:
		using Allocator = CustomAllocator<TimeContainer<T>, StorageType>;
		using ShmemTimeline = Timeline<T, Allocator>;

		using param_type = typename ShmemTimeline::param_type;
		using reference_param_type = typename ShmemTimeline::reference_param_type;
		using const_reference_param_type = typename ShmemTimeline::const_reference_param_type;
		using rvalue_reference_param_type = typename ShmemTimeline::rvalue_reference_param_type;

		using TimeSeries = typename ShmemTimeline::TimeSeries;
		using time_type = typename ShmemTimeline::time_type;
		using value_type = typename ShmemTimeline::value_type;
		using reference_type = typename ShmemTimeline::reference_type;
		using const_reference_type = typename ShmemTimeline::const_reference_type;
		using rvalue_reference = typename ShmemTimeline::rvalue_reference;
		using size_type = typename ShmemTimeline::size_type;
		using capacity_type = typename ShmemTimeline::capacity_type;
		using index_type = typename ShmemTimeline::index_type;

		using this_type = SharedTimelineObject<T, StorageType>;

		using base_type = SharedObjectBase<T, ConsumerBase<StorageType>, StorageType>;
		using typename base_type::consumer_type;
		using typename base_type::ConsumerWeakPtr;
		using typename base_type::ConsumerSharedPtr;

		/*!
		 * @brief Creates an empty timeline buffer with the specified @p t_allocator allocator with the capacity of @p t_bufferCapacity
		*/
		SharedTimelineObject(VoidAllocator<StorageType>& t_allocator, capacity_type t_bufferCapacity)
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
		 * @brief Pushes the @p t_item with its coresponding timestamp @p t_time to the back of the buffer
		 * @details Pushes as a later data
		*/
		void pushLater(const_reference_param_type t_item, const time_type& t_time)
		{
			ScopedSharableLock lock(m_mtx);
			m_buffer.pushLater(t_item, t_time);
		}
		/*!
		 * @brief Pushes the @p t_item with its coresponding timestamp to the back of the buffer
		 * @details Pushes as a later data
		*/
		void pushLater(const_reference_type t_item)
		{
			ScopedSharableLock lock(m_mtx);
			m_buffer.pushLater(t_item);
		}
		/*!
		 * @brief Pushes the @p t_item with its coresponding timestamp @p t_time to the front of the buffer
		 * @details Pushes as an older data
		*/
		void pushOlder(const_reference_param_type t_item, const time_type& t_time)
		{
			ScopedSharableLock lock(m_mtx);
			m_buffer.pushOlder(t_item, t_time);
		}
		/*!
		 * @brief Pushes the @p t_item with its coresponding timestamp  to the front of the buffer
		 * @details Pushes as an older data
		*/
		void pushOlder(const_reference_type t_item)
		{
			ScopedSharableLock lock(m_mtx);
			m_buffer.pushOlder(t_item);
		}
		/*!
		 * @brief Gets latest data
		*/
		reference_type latest()
		{
			ScopedSharableLock lock(m_mtx);
			return m_buffer.latest();
		}
		/*!
		 * @brief Gets latest data
		*/
		const_reference_type latest() const
		{
			ScopedSharableLock lock(m_mtx);
			return m_buffer.latest();
		}
		/*!
		 * @brief Gets the oldest data
		*/
		reference_type oldest()
		{
			ScopedSharableLock lock(m_mtx);
			return m_buffer.oldest();
		}
		/*!
		 * @brief Gets the oldest data
		*/
		const_reference_type oldest() const
		{
			ScopedSharableLock lock(m_mtx);
			return m_buffer.oldest();
		}
		/*!
		 * @brief Gets series of data that corresponds to the @p t_time timestamp
		 * @return A vector of up to two elements
		 * 
		 * @details Searches the buffer from later data to older
		*/
		TimeSeries getLaterToOlder(const time_type& t_time) const
		{
			ScopedSharableLock lock(m_mtx);
			return m_buffer.getLaterToOlder(t_time);
		}
		/*!
		 * @brief Gets series of data that corresponds to the @p t_time timestamp
		 * @return A vector of up to two elements
		 *
		 * @details Searches the buffer from older data to later
		*/
		TimeSeries getOlderToLater(const time_type& t_time) const
		{
			ScopedSharableLock lock(m_mtx);
			return m_buffer.getOlderToLater(t_time);
		}
		/*!
		 * @brief Gets series of data that corresponds to the time interval from @p t_min to @p t_max
		 * @return A vector of multiple elements
		 *
		 * @details Searches the buffer from later data to older
		*/
		TimeSeries getLaterToOlder(time_type t_min, time_type t_max) const
		{
			ScopedSharableLock lock(m_mtx);
			return m_buffer.getLaterToOlder(t_min, t_max);
		}
		/*!
		 * @brief Gets series of data that corresponds to the time interval from @p t_min to @p t_max
		 * @return A vector of multiple elements
		 *
		 * @details Searches the buffer from older data to later
		*/
		TimeSeries getOlderToLater(time_type t_min, time_type t_max) const
		{
			ScopedSharableLock lock(m_mtx);
			return m_buffer.getOlderToLater(t_min, t_max);
		}

	private:
		/*!
		 * @brief The capacity of the buffer
		*/
		capacity_type m_bufferCapacityInput{ 0U };
		/*!
		 * @brief The underlaying timeline buffer
		*/
		ShmemTimeline m_buffer;
	};
}

#endif // !SHMEM_TIMELINE_H