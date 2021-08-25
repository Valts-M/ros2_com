/*!
* @file
* @ingroup CoreLibGroup
* @ingroup SharedMemory
*/
#ifndef SHMEM_RAW_PRODUCER_HPP
#define SHMEM_RAW_PRODUCER_HPP

#include "shmem_producer_base.hpp"
#include "shmem_raw_data.hpp"

namespace zbot::shmem
{
	/*!
	 * @brief Shared object wrapper for writing/reading shared data
	 * @tparam T Data type that is stored in the shared memory
	 * @tparam StorageType The shared memory storage. Two possible storages:
	 * 'boost::interprocess::managed_shared_memory' or 'boost::interprocess::managed_mapped_file'
	*/
	template <class T, class StorageType>
	class ShmemRawProducer : public ShmemProducerBase<SharedRawObject<T, StorageType>, StorageType, VoidAllocator<StorageType>>
	{
	public:
		using SharedObject = SharedRawObject<T, StorageType>;
		using ProducerBase = ShmemProducerBase<SharedObject, StorageType, VoidAllocator<StorageType>>;

		using value_type = typename SharedObject::value_type;
		using reference_type = typename SharedObject::reference_type;
		using const_reference_type = typename SharedObject::const_reference_type;
		using rvalue_reference_type = typename SharedObject::rvalue_reference_type;
		/*!
		 * @brief Initializes the base class and other variables
		 * @param t_producerName The name of the producer that is usde for creating and accessing the ring buffer
		 * @param t_memoryName The name of the shared memory segment where the shared object is placed
		 * @param t_shmemSize The size of the shared memory segment that needs to be allocated for the ring buffer
		*/
		ShmemRawProducer(const std::string& t_producerName, const std::string& t_memoryName, const size_t t_shmemSize = bip::mapped_region::get_page_size()) 
			: ProducerBase(t_producerName, t_memoryName, t_shmemSize)
		{}

		/*!
		 * @brief Update the shared data wih a custom function
		 * @tparam CustomFunc A function with signature <i>void(T&)</i>
		 * @param t_update Function that is called with reference to the
		 * shared data <i>t_update(static_cast<T&>(data))</i>
		*/
		template <typename CustomFunc>
		void customUpdate(CustomFunc&& t_update)
		{
			checkObject();
			SharedObjectSharedPtr& sharedObj = m_ownerSharedPtr->sharedData;
			sharedObj->customUpdate(std::forward<CustomFunc>(t_update));
		}
		/*!
		 * @brief Update the shared data wih a copy-assignment
		 * @param t_data Process-scoped data that is copied to the shared data
		*/
		void copyUpdate(const_reference_type t_data)
		{
			checkObject();
			SharedObjectSharedPtr& sharedObj = m_ownerSharedPtr->sharedData;
			*sharedObj = t_data;
		}
	private:

		/*!
		 * @brief Creates a tuple of the arguments that are passed in the shared object constructor
		 * @return A tuple with a single element: allocator
		*/
		std::tuple<VoidAllocator<StorageType>> getSharedObjectConstructorArguments() override
		{
			return std::make_tuple<VoidAllocator<StorageType>>(static_cast<VoidAllocator<StorageType>>(m_shmemManager.get_segment_manager()));
		}
	};
}
#endif // !SHMEM_RAW_PRODUCER_HPP