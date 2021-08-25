/*!
* @file
* @ingroup CoreLibGroup
* @ingroup SharedMemory
*/
#ifndef SHMEM_CB_CONSUMER_HPP
#define SHMEM_CB_CONSUMER_HPP

#include "shmem_consumer_base.hpp"
#include "shmem_circular_buffer.hpp"

namespace zbot::shmem
{
	/*!
	 * @brief Developer interface for easier manipulation with the circular buffer as a consumer
	 * @tparam T The data type of a single ring buffer element
	 * @tparam Policy The IO policy (FIFO or LIFO)
	 * @tparam StorageType Only bip::managed_shared_memory and bip::managed_mapped_file types are supported
	*/
	template <class T, class Policy, class StorageType>
	class ShmemCBConsumer : public ShmemConsumerBase<SharedCBObject<T, Policy, StorageType>, StorageType>
	{
	public:
		using SharedObject= SharedCBObject<T, Policy, StorageType>;
		using ConsumerBase = ShmemConsumerBase<SharedObject, StorageType>;
		using SharedObjectSharedPtr = typename ConsumerBase::SharedObjectSharedPtr;

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
		ShmemCBConsumer(const std::string& t_producerName, const std::string& t_memoryName, const std::string t_consumerName) 
			: ConsumerBase(t_producerName, t_memoryName, t_consumerName)
		{}
		/*!
		 * @brief The capacity of the buffer
		*/
		capacity_type capacity() const
		{
			ConsumerBase::checkObject();
			auto sharedPtr = SharedObjectSharedPtr(boost::move(ConsumerBase::referenceObject()));
			return sharedPtr->capacity();
		}
		/*!
		 * @brief The size of the buffer for the specified consumer
		*/
		size_type consumerSize() const
		{
			ConsumerBase::checkObject();
			auto sharedPtr = SharedObjectSharedPtr(boost::move(ConsumerBase::referenceObject()));
			return sharedPtr->size(ConsumerBase::m_consumer);
		}
		/*!
		 * @brief The size of the ring buffer
		*/
		size_type size() const
		{
			ConsumerBase::checkObject();
			auto sharedPtr = SharedObjectSharedPtr(boost::move(ConsumerBase::referenceObject()));
			return sharedPtr->size();
		}
		/*!
		 * @brief Is the buffer empty
		*/
		bool empty() const
		{
			ConsumerBase::checkObject();
			auto sharedPtr = SharedObjectSharedPtr(boost::move(ConsumerBase::referenceObject()));
			return sharedPtr->empty();
		}
		/*!
		 * @brief Is the buffer full
		*/
		bool full() const
		{
			ConsumerBase::checkObject();
			auto sharedPtr = SharedObjectSharedPtr(boost::move(ConsumerBase::referenceObject()));
			return sharedPtr->full();
		}
		/*!
		 * @brief Pops the element from the buffer according to the IO policy
		*/
		void pop()
		{
			ConsumerBase::checkObject();
			auto sharedPtr = SharedObjectSharedPtr(boost::move(ConsumerBase::referenceObject()));
			sharedPtr->pop(ConsumerBase::m_consumer);
		}
		/*!
		 * @brief Get the data according to the IO policy
		*/
		value_type get()
		{
			ConsumerBase::checkObject();
			auto sharedPtr = SharedObjectSharedPtr(boost::move(ConsumerBase::referenceObject()));
			return sharedPtr->get(ConsumerBase::m_consumer);
		}

		/*!
		 * @brief Get the data according to the IO policy, pops the corresponding element
		*/
		value_type getAndPop()
		{
			ConsumerBase::checkObject();
			auto sharedPtr = SharedObjectSharedPtr(boost::move(ConsumerBase::referenceObject()));
			return sharedPtr->getAndPop(ConsumerBase::m_consumer);
		}
	};
}
#endif // !SHMEM_CB_PRODUCER_HPP