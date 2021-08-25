/*!
* @file
* @ingroup CoreLibGroup
* @ingroup SharedMemory
*/
#ifndef SHMEM_TIMELINE_PRODUCER_HPP
#define SHMEM_TIMELINE_PRODUCER_HPP

#include "shmem_producer_base.hpp"
#include "shmem_timeline.hpp"

namespace zbot::shmem
	/*!
	 * @brief Developer interface for easier manipulation with the timeline buffer as a producer
	 * @tparam T The data type of a single timeline buffer element
	 * @tparam StorageType Only bip::managed_shared_memory and bip::managed_mapped_file types are supported
	*/
	template <class T, class StorageType>
	class ShmemTimelineProducer : public ShmemProducerBase<SharedTimelineObject<T, StorageType>, StorageType, VoidAllocator<StorageType>, size_t>
	{
	public:
		using SharedObject = SharedTimelineObject<T, StorageType>;
		using ProducerBase = ShmemProducerBase<SharedObject, StorageType, VoidAllocator<StorageType>, size_t>;
		
		using TimeSeries = typename SharedObject::TimeSeries;
		using time_type = typename SharedObject::time_type;

		using param_type = typename SharedObject::param_type;
		using reference_param_type = typename SharedObject::reference_param_type;
		using const_reference_param_type = typename SharedObject::const_reference_param_type;
		using rvalue_reference_param_type = typename SharedObject::rvalue_reference_param_type;

		using value_type = typename SharedObject::value_type;
		using reference_type = typename SharedObject::reference_type;
		using const_reference_type = typename SharedObject::const_reference_type;
		using rvalue_reference = typename SharedObject::rvalue_reference;
		using size_type = typename SharedObject::size_type;
		using capacity_type = typename SharedObject::capacity_type;
		using index_type = typename SharedObject::index_type;

		/*!
		 * @brief Initializes the base class and other variables
		 * @param t_producerName The name of the producer that is used for creating and accessing the buffer
		 * @param t_memoryName The name of the shared memory segment where the shared object is placed
		 * @param t_capacity The desired capacity of the buffer
		 * @param t_shmemSize The size of the shared memory segment that needs to be allocated for the buffer
		*/
		ShmemTimelineProducer(const std::string& t_producerName, const std::string& t_memoryName, capacity_type t_capacity, size_t t_shmemSize = bip::mapped_region::get_page_size())
			: ProducerBase(t_producerName, t_memoryName, t_shmemSize), m_bufferCapacity(t_capacity)
		{}

		/*!
		 * @brief The capacity of the buffer
		*/
		capacity_type capacity() const { checkObject(); return m_ownerSharedPtr->sharedData->capacity(); }
		/*!
		 * @brief The size of the buffer
		*/
		size_type size() const { checkObject(); return m_ownerSharedPtr->sharedData->size(); }
		/*!
		 * @brief Is the buffer empty
		*/
		bool empty() const { checkObject(); return m_ownerSharedPtr->sharedData->empty(); }
		/*!
		 * @brief Is the buffer full
		*/
		bool full() const { checkObject(); return m_ownerSharedPtr->sharedData->full(); }
		/*!
		 * @brief Pushes the @p t_item with its coresponding timestamp @p t_time to the back of the buffer
		 * @details Pushes as a later data
		*/
		void pushLater(const_reference_param_type t_item, const time_type& t_time)
		{
			checkObject();
			m_ownerSharedPtr->sharedData->pushLater(t_item, t_time);
		}
		/*!
		 * @brief Pushes the @p t_item with its coresponding timestamp to the back of the buffer
		 * @details Pushes as a later data
		*/
		void pushLater(const_reference_type t_item)
		{
			checkObject();
			m_ownerSharedPtr->sharedData->pushLater(t_item);
		}
		/*!
		 * @brief Pushes the @p t_item with its coresponding timestamp @p t_time to the front of the buffer
		 * @details Pushes as an older data
		*/
		void pushOlder(const_reference_param_type t_item, const time_type& t_time)
		{
			checkObject();
			m_ownerSharedPtr->sharedData->pushOlder(t_item, t_time);
		}
		/*!
		 * @brief Pushes the @p t_item with its coresponding timestamp to the front of the buffer
		 * @details Pushes as an older data
		*/
		void pushOlder(const_reference_type t_item)
		{
			checkObject();
			m_ownerSharedPtr->sharedData->pushOlder(t_item);
		}
		/*!
		 * @brief Get the latest data
		*/
		value_type latest() const
		{
			checkObject();
			return m_ownerSharedPtr->sharedData->latest();
		}
		/*!
		 * @brief Get the oldest data
		 * @return 
		*/
		value_type oldest() const
		{
			checkObject();
			return m_ownerSharedPtr->sharedData->oldest();
		}
		/*!
		 * @brief Gets series of data that corresponds to the @p t_time timestamp
		 * @return A vector of up to two elements
		 *
		 * @details Searches the buffer from later data to older
		*/
		TimeSeries getLaterToOlder(const time_type& t_time) const
		{
			checkObject();
			return m_ownerSharedPtr->sharedData->getLaterToOlder(t_time);
		}
		/*!
		 * @brief Gets series of data that corresponds to the @p t_time timestamp
		 * @return A vector of up to two elements
		 *
		 * @details Searches the buffer from older data to later
		*/
		TimeSeries getOlderToLater(const time_type& t_time) const
		{
			checkObject();
			return m_ownerSharedPtr->sharedData->getOlderToLater(t_time);
		}
		/*!
		 * @brief Gets series of data that corresponds to the time interval from @p t_min to @p t_max
		 * @return A vector of multiple elements
		 *
		 * @details Searches the buffer from later data to older
		*/
		TimeSeries getLaterToOlder(time_type t_min, time_type t_max) const
		{
			checkObject();
			return m_ownerSharedPtr->sharedData->getLaterToOlder(t_min, t_max);
		}
		/*!
		 * @brief Gets series of data that corresponds to the time interval from @p t_min to @p t_max
		 * @return A vector of multiple elements
		 *
		 * @details Searches the buffer from older data to later
		*/
		TimeSeries getOlderToLater(time_type t_min, time_type t_max) const
		{
			checkObject();
			return m_ownerSharedPtr->sharedData->getOlderToLater(t_min, t_max);
		}
		
	private:
		/*!
		 * @brief The capacity of the buffer
		*/
		capacity_type m_bufferCapacity{ 0U };
		/*!
		 * @brief Creates a tuple of the arguments that are passed in the shared object constructor
		 * @return A tuple with two elements: allocator and buffer capacity
		*/
		std::tuple<VoidAllocator<StorageType>, size_t> getSharedObjectConstructorArguments() override
		{
			size_t tempSize = static_cast<size_t>(m_bufferCapacity);
			return std::make_tuple<VoidAllocator<StorageType>, size_t>(static_cast<VoidAllocator<StorageType>>(m_shmemManager.get_segment_manager()), boost::move(tempSize));
		}
	};
}
#endif // !SHMEM_TIMELINE_PRODUCER_HPP