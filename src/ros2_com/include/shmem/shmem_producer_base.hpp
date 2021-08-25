/*!
* @file
* @ingroup CoreLibGroup
* @ingroup SharedMemory
*/
#ifndef SHMEM_PRODUCER_BASE_HPP
#define SHMEM_PRODUCER_BASE_HPP

#include "shmem_base_new.hpp"

namespace zbot::shmem
{

	/*!
	 * @brief Shared memory producer base class
	 * @tparam SharedData It's supposed to be the shared object type with template parameters T, StorageType (e.g. SharedRawObject<T, StorageType>)
	 * @tparam StorageType Only bip::managed_shared_memory and bip::managed_mapped_file types are supported
	 * @tparam ...ArgsA A list of arguments that are passed int the shared object object for SharedData
	*/
	template <class SharedObject, class StorageType, typename... ArgsA>
	class ShmemProducerBase : public ShmemBase<StorageType>
	{
	public:
		using SharedObjectWeakPtr = ManagedWeakPtr<SharedObject, StorageType>;
		using SharedObjectSharedPtr = ManagedSharedPtr<SharedObject, StorageType>;

		using ObjectOwner = SharedObjectOwner<SharedObject, StorageType>;
		using ObjectOwnerSharedPtr = ManagedSharedPtr<ObjectOwner, StorageType>;

		using ShmemRemover = SharedMemoryRemover<StorageType>;
		/*!
		 * @brief Initializes base class and some variables
		 * @param t_producerName The name of the producer that will be used for shared object access
		 * @param t_memoryName The name of the shared memory segment
		 * @param t_shmemSize The size for the shared memory segment
		 * 
		 * @todo Calculate shared memory size based on the data type and the data structure used (ring buffer, timeline, raw etc.)
		*/
		ShmemProducerBase(const std::string& t_producerName, const std::string& t_memoryName, const size_t t_shmemSize = bip::mapped_region::get_page_size()) 
			: ShmemBase<StorageType>("defaultype", t_producerName, t_memoryName),
			m_shmemSize(t_shmemSize < bip::mapped_region::get_page_size() ? bip::mapped_region::get_page_size() : t_shmemSize),
			m_remover(ShmemBase<StorageType>::getMemoryName())
		{}
		/*!
		 * @brief Clears the smart pointer of the owner
		*/
		~ShmemProducerBase() override { m_ownerSharedPtr.reset(); };
		/*!
		 * @brief Placeholder
		*/
		bool isTypeOf(DataStructType t_id) const override { return true; }
		/*!
		 * @brief Creates the shared memory segment
		 * 
		 * @details If a conflict occurs (a memory with the same name already exists) then an exception is thrown.
		*/
		void openMemory() override 
		{
			if (isMemoryConflict()) throw std::domain_error("Shared memory conflict!");
			try
			{
				// mapped file is created in the local binary folder if the path is not global
				ShmemBase<StorageType>::m_shmemManager = StorageType(bip::create_only, ShmemBase<StorageType>::getMemoryName().c_str(), m_shmemSize);
				//std::cout << std::flush << "ShmemProducer: memory opened: '" << ShmemBase<StorageType>::getMemoryName() << "'\n";
			}
			catch (bip::interprocess_exception& ex)
			{
				std::cout << std::flush << "ShmemProducerBase: memory failed to open: '" << ShmemBase<StorageType>::getMemoryName()
					<< "'; Err: " << ex.get_error_code() << "; " << ex.what() << '\n';
			}
			catch (std::exception& stdEx)
			{
				std::cout << std::flush << "ShmemProducerBase: memory failed to open: '" << ShmemBase<StorageType>::getMemoryName()
					<< "'; Err: " << stdEx.what() << '\n';
			}
		}
		/*!
		 * @brief Checks if the shared object is referenced
		*/
		bool isObjectReferenced() const override { return m_ownerSharedPtr.get() ? true : false; }
		
		/*!
		 * @brief Creates a raw object in the shared memory
		 * @tparam T The type of the object to create. 
		 * @param[out] t_success Outputs whether the creation was successfull
		 * @param ...t_pack The arguments for the object's constructor
		 * @return Pointer to the created object. If the object was not created, then a nullptr is returned.
		*/
		template<class T, typename... Args>
		T* createObject(bool& t_success, const std::string& t_name, Args&&... t_pack)
		{
			t_success = false;
			if (!ShmemBase<StorageType>::isMemoryOpen()) return nullptr; // memory is not opened
			try
			{
				T* data = ShmemBase<StorageType>::m_shmemManager.template construct<T>(t_name.c_str())(t_pack...);
				t_success = true;
				return data;
			}
			catch (bip::interprocess_exception& ex)
			{
				std::cout << std::flush << "ShmemProducerBase: createObject failed: '" << ShmemBase<StorageType>::getProducerName()
					<< "' in memory: '" << ShmemBase<StorageType>::getMemoryName()
					<< "'; Err: " << ex.get_error_code() << ", " << ex.what() << '\n';
			}
			catch (std::exception& stdEx)
			{
				std::cout << std::flush << "ShmemProducerBase: createObject failed: '" << ShmemBase<StorageType>::getProducerName()
					<< "' in memory: '" << ShmemBase<StorageType>::getMemoryName()
					<< "'; Err: " << stdEx.what() << '\n';
			}
			return nullptr;
		}

		/*!
		 * @brief Creates the shared object with owner, reference is stored in the local variable
		 * @tparam ...Args Argument list data types for the shared object constructor
		 * @param ...t_pack Argument list for the shared object construcor
		 * @return True if the shared object and owner was successfully created, otherwise returns false
		*/
		template <typename... Args>
		bool createSharedObject(Args&&... t_pack)
		{
			if (isObjectReferenced()) return true; // already created it
			if (!ShmemBase<StorageType>::isMemoryOpen()) return false; // memory is not opened
			try
			{
				// construct the shared object
				SharedObject* sharedObject = ShmemBase<StorageType>::m_shmemManager.template construct<SharedObject>(ShmemBase<StorageType>::getObjectName().c_str())(boost::forward<Args>(t_pack)...);
				// make shared pointer to the shared object
				SharedObjectSharedPtr sharedObjectPtr = bip::make_managed_shared_ptr<SharedObject>(sharedObject, ShmemBase<StorageType>::m_shmemManager);

				// create owner of the shared object pointer
				ObjectOwner* owner = ShmemBase<StorageType>::m_shmemManager.template construct<ObjectOwner>(ShmemBase<StorageType>::getOwnerName().c_str())(sharedObjectPtr);
				m_ownerSharedPtr = bip::make_managed_shared_ptr<ObjectOwner>(owner, ShmemBase<StorageType>::m_shmemManager);
				return true;
			}
			catch (bip::interprocess_exception& ex)
			{
				std::cout << std::flush << "ShmemProducerBase: createSharedObject failed: '" << ShmemBase<StorageType>::getProducerName()
					<< "' in memory: '" << ShmemBase<StorageType>::getMemoryName()
					<< "'; Err: " << ex.get_error_code() << ", " << ex.what() << '\n';
			}
			catch (std::exception& stdEx)
			{
				std::cout << std::flush << "ShmemProducerBase: createSharedObject failed: '" << ShmemBase<StorageType>::getProducerName()
					<< "' in memory: '" << ShmemBase<StorageType>::getMemoryName()
					<< "'; Err: " << stdEx.what() << '\n';
			}
			return false;
		}

	protected: 
		/*!
		 * @brief Reference to the shared object's owner in the shared memory
		*/
		ObjectOwnerSharedPtr m_ownerSharedPtr;
		/*!
		 * @brief Monitors the memory
		 * @return True if memory is open and the object is referenced
		 * 
		 * @details Creates the shared memory segment if necessary, also creates the shared object 
		 * and its owner if necessary
		*/
		bool monitorMemory() override
		{
			if (!ShmemBase<StorageType>::isMemoryOpen()) openMemory();
			if (ShmemBase<StorageType>::isMemoryOpen() && !isObjectReferenced())
			{
				auto args = getSharedObjectConstructorArguments();
				
				std::apply([&](auto&&... ar)
						   {
							   createSharedObject(ar...);
						   }, getSharedObjectConstructorArguments());
			}
			return ShmemBase<StorageType>::isMemoryOpen() && isObjectReferenced();
		}

	private:
		/*!
		 * @brief The size of the shared memory
		*/
		size_t m_shmemSize{ 0U };
		/*!
		 * @brief Shared memory segment deallocator for automatic deallocation on destruction
		*/
		ShmemRemover m_remover;
		/*!
		 * @brief Checks if a shared memory segment with the same name already exists
		*/
		bool isMemoryConflict()
		{
			if (ShmemBase<StorageType>::isMemoryOpen()) return false; // if this producer has acquired the memory, then there is no conflict
			try
			{
				auto tempManager = StorageType(bip::open_only, ShmemBase<StorageType>::getMemoryName().c_str());
				std::cout << std::flush << "ShmemProducerBase: memory conflict: '" << ShmemBase<StorageType>::getMemoryName() << "'\n";
				if (tempManager.get_segment_manager()) return true;
			}
			catch (bip::interprocess_exception& ex)
			{
				if (ex.get_error_code() != 7)
					std::cout << std::flush << "ShmemProducerBase: isMemoryConflict failed: '" << ShmemBase<StorageType>::getMemoryName()
					<< "' Err: " << ex.get_error_code() << "; " << ex.what() << '\n';
			}
			catch (std::exception& stdEx)
			{
				std::cout << std::flush << "ShmemProducerBase: isMemoryConflict failed: '" << ShmemBase<StorageType>::getMemoryName()
					<< "'; Err: " << stdEx.what() << '\n';
			}
			return false;
		}

		/*!
		 * @brief Implementation should return a tuple of rvalue references of the shared object constructor arguments
		*/
		virtual std::tuple<ArgsA...> getSharedObjectConstructorArguments() = 0;

	};

}

#endif // !SHMEM_PRODUCER_BASE_HPP