/*!
* @file
* @ingroup CoreLibGroup
* @ingroup SharedMemory
*/
#ifndef SHMEM_CB_PRODUCER_HPP
#define SHMEM_CB_PRODUCER_HPP

#include "shmem_producer_base.hpp"
#include "shmem_circular_buffer.hpp"

namespace zbot::shmem
{
	/*!
	 * @brief Developer interface for easier manipulation with the ring buffer as a producer
	 * @tparam T The data type of a single ring buffer element
	 * @tparam Policy The IO policy (FIFO or LIFO)
	 * @tparam StorageType Only bip::managed_shared_memory and bip::managed_mapped_file types are supported
	*/
	template <class T, class Policy, class StorageType>
	class ShmemCBProducer : public ShmemProducerBase<SharedCBObject<T, Policy, StorageType>, StorageType, VoidAllocator<StorageType>, size_t>
	{
	public:
		using SharedObject = SharedCBObject<T, Policy, StorageType>;
		using ProducerBase = ShmemProducerBase<SharedObject, StorageType, VoidAllocator<StorageType>, size_t>;
		
		using value_type = typename SharedObject::value_type;
		using reference_type = typename SharedObject::reference_type;
		using const_reference_type = typename SharedObject::const_reference_type;
		using rvalue_reference = typename SharedObject::rvalue_reference;
		using size_type = typename SharedObject::size_type;
		using capacity_type = typename SharedObject::capacity_type;
		using index_type = typename SharedObject::index_type;

		/*!
		 * @brief Initializes the base class and other variables
		 * @param t_producerName The name of the producer that is used for creating and accessing the ring buffer
		 * @param t_memoryName The name of the shared memory segment where the shared object is placed
		 * @param t_capacity The desired capacity of the ring buffer
		 * @param t_shmemSize The size of the shared memory segment that needs to be allocated for the ring buffer
		*/
		ShmemCBProducer(const std::string& t_producerName, const std::string& t_memoryName, capacity_type t_capacity, size_t t_shmemSize = bip::mapped_region::get_page_size()) 
			: ProducerBase(t_producerName, t_memoryName, t_shmemSize), m_bufferCapacity(t_capacity)
		{}
		/*!
		 * @brief The capacity of the buffer
		*/
		capacity_type capacity() const { ProducerBase::checkObject(); return ProducerBase::m_ownerSharedPtr->sharedData->capacity(); }
		/*!
		 * @brief The size of the buffer
		*/
		size_type size() const { ProducerBase::checkObject(); return ProducerBase::m_ownerSharedPtr->sharedData->size(); }
		/*!
		 * @brief Is the buffer empty
		*/
		bool empty() const { ProducerBase::checkObject(); return ProducerBase::m_ownerSharedPtr->sharedData->empty(); }
		/*!
		 * @brief Is the buffer full
		*/
		bool full() const { ProducerBase::checkObject(); return ProducerBase::m_ownerSharedPtr->sharedData->full(); }
		/*!
		 * @brief Pops the element from the buffer according to the IO policy
		*/
		void pop()
		{
			ProducerBase::checkObject();
			ProducerBase::m_ownerSharedPtr->sharedData->pop();
		}
		/*!
		 * @brief Push the data @p t_item to the buffer
		*/
		void push(const_reference_type t_item)
		{
			ProducerBase::checkObject();
			ProducerBase::m_ownerSharedPtr->sharedData->push(t_item);
		}
		/*!
		 * @brief Get the data according to the IO policy
		*/
		value_type get()
		{
			ProducerBase::checkObject();
			return ProducerBase::m_ownerSharedPtr->sharedData->get();
		}
		/*!
		 * @brief Get the data according to the IO policy, pops the corresponding element
		*/
		value_type getAndPop()
		{
			ProducerBase::checkObject();
			return ProducerBase::m_ownerSharedPtr->sharedData->getAndPop();
		}

	private:
		/*!
		 * @brief The buffer capacity
		*/
		capacity_type m_bufferCapacity{ 0U };

		/*!
		 * @brief Creates a tuple of the arguments that are passed in the shared object constructor
		 * @return A tuple with two elements: allocator and ring buffer capacity
		*/
		std::tuple<VoidAllocator<StorageType>, size_t> getSharedObjectConstructorArguments() override
		{
			size_t tempSize = static_cast<size_type>(m_bufferCapacity);
			return std::make_tuple< VoidAllocator<StorageType>, size_t>(static_cast<VoidAllocator<StorageType>>(ProducerBase::m_shmemManager.get_segment_manager()), boost::move(tempSize));
		}
	};
}
#endif // !SHMEM_CB_PRODUCER_HPP