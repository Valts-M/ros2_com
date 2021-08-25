/*!
* @file
* @ingroup CoreLibGroup
* @ingroup SharedMemory
*/
#ifndef SHMEM_CONSUMER_BASE_HPP
#define SHMEM_CONSUMER_BASE_HPP

#include "shmem_base_new.hpp"

namespace zbot::shmem
{
	/*!
	 * @brief Shared memory consumer base class
	 * @tparam StorageType Only bip::managed_shared_memory and bip::managed_mapped_file types are supported
	 * @tparam SharedData It's supposed to be SharedObject<T, StorageType>
	*/
	template <class SharedObject, class StorageType>
	class ShmemConsumerBase : public ShmemBase<StorageType>
	{
	public:
		using SharedObjectWeakPtr = ManagedWeakPtr<SharedObject, StorageType>;
		using SharedObjectSharedPtr = ManagedSharedPtr<SharedObject, StorageType>;

		using ObjectOwner = SharedObjectOwner<SharedObject, StorageType>;
		using consumer_type = typename SharedObject::consumer_type;
		using ConsumerWeakPtr = typename SharedObject::ConsumerWeakPtr;
		using ConsumerSharedPtr = typename SharedObject::ConsumerSharedPtr;
		
		/*!
		 * @brief Initializes base class and other varialbes
		 * @param t_producerName The name of the producer that creates the object, form which recieve data
		 * @param t_memoryName The shared memory segment where the shared object is placed
		 * @param t_consumerName The name of the consumer
		*/
		ShmemConsumerBase(const std::string& t_producerName, const std::string& t_memoryName, const std::string& t_consumerName) 
			: ShmemBase<StorageType>("defaultype", t_producerName, t_memoryName), m_consumerName(t_consumerName)
		{}

		/*!
		 * @brief Deallocates the smart pointers of the shared object and the consumer
		*/
		~ShmemConsumerBase() override 
		{
			m_objectWeakPtr.reset();
			m_consumer.reset();
		};
		/*!
		 * @brief Placeholder
		*/
		bool isTypeOf(DataStructType t_id) const override { return true; }

		/*!
		 * @brief Opens the shared memory segment
		*/
		void openMemory() override
		{
			try
			{
				ShmemBase<StorageType>::m_shmemManager = StorageType(bip::open_only, ShmemBase<StorageType>::getMemoryName().c_str());
			}
			catch (bip::interprocess_exception& ex)
			{
				std::cout << std::flush << "ShmemConsumerBase: memory failed to open: '" << ShmemBase<StorageType>::getMemoryName()
					<< "'; Err: " << ex.get_error_code() << ", " << ex.what() << '\n';
			}
			catch (std::exception& stdEx)
			{
				std::cout << std::flush << "ShmemConsumerBase: memory failed to open: '" << ShmemBase<StorageType>::getMemoryName()
					<< "'; Err: " << stdEx.what() << '\n';
			}
		}
		/*!
		 * @brief Checks if the shared object is referenced
		*/
		bool isObjectReferenced() const override { return m_objectWeakPtr.expired() ? false : true; }

		/*!
		 * @brief Finds the shared object's owner and adds a consumer reference
		 * @return True if the object was found, otherwise returns false
		*/
		bool findObject()
		{
			if (isObjectReferenced()) return true;
			if (!ShmemBase<StorageType>::isMemoryOpen()) return false;
			try
			{
				auto owner = ShmemBase<StorageType>::m_shmemManager.template find<ObjectOwner>(ShmemBase<StorageType>::getOwnerName().c_str());
				if (owner.second != 1)
				{
					std::cout << std::flush << "ShmemConsumerBase: findObject failed: '" << ShmemBase<StorageType>::getProducerName()
						<< "' in memory: '" << ShmemBase<StorageType>::getMemoryName() << "'\n";
					return false;
				}
				else
				{
					m_objectWeakPtr = SharedObjectWeakPtr(owner.first->sharedData);
				}
			}
			catch (bip::interprocess_exception& ex)
			{
				std::cout << std::flush << "ShmemConsumerBase: findObject failed: '" << ShmemBase<StorageType>::getProducerName()
					<< "' in memory: '" << ShmemBase<StorageType>::getMemoryName()
					<< "'; Err: " << ex.get_error_code() << ", " << ex.what() << '\n';
				return false;
			}
			catch (std::exception& stdEx)
			{
				std::cout << std::flush << "ShmemConsumerBase: findObject failed: '" << ShmemBase<StorageType>::getProducerName()
					<< "' in memory: '" << ShmemBase<StorageType>::getMemoryName()
					<< "'; Err: " << stdEx.what() << '\n';
				return false;
			}

			//std::cout << std::flush << "ShmemConsumer: object found: '" << getProducerName() << "'\n";
			addConsumer();
			return true;
		}

	protected:
		/*!
		 * @brief Reference to the shared object in the shared memory
		*/
		SharedObjectWeakPtr m_objectWeakPtr;
		/*!
		 * @brief Reference to the consumer manager in the shared memory
		*/
		ConsumerWeakPtr m_consumer;
		/*!
		 * @brief Get shared pointer to the	shared object
		*/
		SharedObjectSharedPtr referenceObject() const
		{
			SharedObjectSharedPtr ptr = m_objectWeakPtr.lock();
			return ptr;
		}
		/*!
		 * @brief Monitors shared memory segment and object 
		 * @return True if the memory is opened and the object is referenced
		 * 
		 * @details Opens memory if necessary and finds the shared object if necessary
		*/
		bool monitorMemory() override
		{
			if (!ShmemBase<StorageType>::isMemoryOpen()) openMemory();
			if (ShmemBase<StorageType>::isMemoryOpen() && !isObjectReferenced()) findObject();
			return ShmemBase<StorageType>::isMemoryOpen() && isObjectReferenced();
		}

	private:
		/*!
		 * @brief The name of the consumer
		*/
		std::string m_consumerName;
		/*!
		 * @brief Adds a reference to the consumer manager
		*/
		void addConsumer()
		{
			if (!isObjectReferenced()) return;
			if (!ShmemBase<StorageType>::isMemoryOpen()) return;
			auto sharedPtr = m_objectWeakPtr.lock();
			ShmemString<StorageType> consumerName = ShmemString<StorageType>(m_consumerName.c_str(), ShmemBase<StorageType>::getAllocator());
			sharedPtr->addConsumer(boost::move(consumerName), ShmemBase<StorageType>::m_shmemManager, m_consumer);
		}
	};

}

#endif // !SHMEM_CONSUMER_BASE_HPP