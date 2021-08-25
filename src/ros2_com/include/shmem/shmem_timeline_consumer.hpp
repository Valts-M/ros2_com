/*!
* @file
* @ingroup CoreLibGroup
* @ingroup SharedMemory
*/
#ifndef SHMEM_TIMELINE_CONSUMER_HPP
#define SHMEM_TIMELINE_CONSUMER_HPP

#include "shmem_consumer_base.hpp"
#include "shmem_timeline.hpp"

namespace zbot::shmem
{
	/*!
	 * @brief Developer interface for easier manipulation with the timeline buffer as a consumer
	 * @tparam T The data type of a single timeline buffer element
	 * @tparam StorageType Only bip::managed_shared_memory and bip::managed_mapped_file types are supported
	*/
	template <class T, class StorageType>
	class ShmemTimelineConsumer : public ShmemConsumerBase<SharedTimelineObject<T, StorageType>, StorageType>
	{
	public:
		using SharedObject = SharedTimelineObject<T, StorageType>;
		using ConsumerBase = ShmemConsumerBase<SharedObject, StorageType>;

		using TimeSeries = typename SharedObject::TimeSeries;
		using time_type = typename SharedObject::time_type;

		using value_type = typename SharedObject::value_type;
		using reference_type = typename SharedObject::reference_type;
		using const_reference_type = typename SharedObject::const_reference_type;
		using rvalue_reference = typename SharedObject::rvalue_reference;
		using size_type = typename SharedObject::size_type;
		using capacity_type = typename SharedObject::capacity_type;
		using index_type = typename SharedObject::index_type;

		/*!
		 * @brief Initializes the base class and some other variables
		 * @param t_producerName The name of the producer that created the shared object
		 * @param t_memoryName The name of the shared memory where the shared object is placed
		 * @param t_consumerName The name of the consumer
		*/
		ShmemTimelineConsumer(const std::string& t_producerName, const std::string& t_memoryName, const std::string t_consumerName)
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
		 * @brief Get the latest data
		*/
		value_type latest() const
		{
			checkObject();
			auto sharedPtr = SharedObjectSharedPtr(boost::move(referenceObject()));
			return sharedPtr->latest();
		}
		/*!
		 * @brief Get the oldest data
		*/
		value_type oldest() const
		{
			checkObject();
			auto sharedPtr = SharedObjectSharedPtr(boost::move(referenceObject()));
			return sharedPtr->oldest();
		}
		/*!
		/*!
		 * @brief Gets series of data that corresponds to the @p t_time timestamp
		 * @return A vector of up to two elements
		 *
		 * @details Searches the buffer from later data to older
		*/
		TimeSeries getLaterToOlder(const time_type& t_time) const
		{
			checkObject();
			auto sharedPtr = SharedObjectSharedPtr(boost::move(referenceObject()));
			return sharedPtr->getLaterToOlder(t_time);
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
			auto sharedPtr = SharedObjectSharedPtr(boost::move(referenceObject()));
			return sharedPtr->getOlderToLater(t_time);
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
			auto sharedPtr = SharedObjectSharedPtr(boost::move(referenceObject()));
			return sharedPtr->getLaterToOlder(t_min, t_max);
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
			auto sharedPtr = SharedObjectSharedPtr(boost::move(referenceObject()));
			return sharedPtr->getOlderToLater(t_min, t_max);
		}
	};
}
#endif // !SHMEM_TIMELINE_CONSUMER_HPP