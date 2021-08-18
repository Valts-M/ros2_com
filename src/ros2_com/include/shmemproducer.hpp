/*!
* @file
*/

#ifndef SHMEM_PRODUCER_H
#define SHMEM_PRODUCER_H

#include "shmembase.hpp"

namespace ros2_com
{
	/*!
	 * @brief RAII for removing shared memory
	 * @tparam StorageType The shared memory storage. Two possibile storages:
	 * 'boost::interprocess::managed_shared_memory' or 'boost::interprocess::managed_mapped_file'
	*/
	template <class StorageType = bip::managed_shared_memory>
	struct SharedMemoryRemover
	{
		static_assert(std::is_same<StorageType, bip::managed_mapped_file>::value ||
					  std::is_same<StorageType, bip::managed_shared_memory>::value,
					  "SharedMemoryRemover: StorageType must be of type 'boost::interprocess::managed_shared_memory' or 'boost::interprocess::managed_mapped_file'");

		const std::string shmemName;
		SharedMemoryRemover(const std::string& t_shmemName) :shmemName(t_shmemName)
		{
			removeMemory();
			//std::cout << "Removed memory \"" << shmemName << "\" on construction!\n";
		}
		~SharedMemoryRemover()
		{
			removeMemory();
			//std::cout << "Removed memory \"" << shmemName << "\" on destruction!\n";
		}
		void removeMemory()
		{
			if (shmemName.empty())return;
			if constexpr (std::is_same<StorageType, bip::managed_mapped_file>::value)
			{
				bip::file_mapping::remove(shmemName.c_str());
			}
			else if constexpr (std::is_same<StorageType, bip::managed_shared_memory>::value)
			{
				bip::shared_memory_object::remove(shmemName.c_str());
			}

		}
	};

	/*!
	 * @brief Shared memory producer that writes to the shared data
	 * @tparam T Data type that is stored in the shared memory
	 * @tparam StorageType The shared memory storage. Two possibile storages:
	 * 'boost::interprocess::managed_shared_memory' or 'boost::interprocess::managed_mapped_file'
	 * 
	 * Restrictions (see boost interprocess shared memory limitations)\n
	 * - do not insert objects with raw pointers, use boost::offset_ptr
	 * - do not use references, use boost::offset_ptr
	 * - do not use virtuality (inheritance)
	 * - do not use static members/function
	*/
	template <class T, class StorageType = bip::managed_shared_memory>
	class ShmemProducer : public ShmemBase
	{
		static_assert(std::is_same<StorageType, bip::managed_mapped_file>::value ||
					  std::is_same<StorageType, bip::managed_shared_memory>::value,
					  "ShmemProducer: StorageType must be of type 'boost::interprocess::managed_shared_memory' or 'boost::interprocess::managed_mapped_file'");

	public:
		/*!
		 * @brief Constructs the shared memory producer and tries to open the shared memory segment
		 * @param t_producerName The name of the producer that creates the shared object
		 * @param t_memoryName The name of the shared memory segment where the shared object is placed 
		 * @param t_shmemSize The size of the shared memery segment that needs to be allocated
		*/
		ShmemProducer(const std::string& t_producerName, const std::string& t_memoryName, const size_t t_shmemSize = bip::mapped_region::get_page_size());
		~ShmemProducer();

		bool isTypeOf(DataStructType t_id) override;

		/*!
		 * @brief Checks if the shared memory segment is successfully created
		 * @return
		*/
		bool isMemoryOpen();
		/*!
		 * @brief Tries to create the shared memory segment
		*/
		void memoryOpen();


		/*!
		 * @brief Creates shared object in the shared memory
		 * @tparam ...Args Arguments' types for shared data constructor (may be empty)
		 * @param ...t_pack Arguments for shared data constructor (may be empty)
		 * @return True if the shared object is successfully created, otherwise false
		 * 
		 * Tries to create the shared object in the shared memory. Multiple step process:
		 * - creates the shared data (passes the input \p t_args to the shared data constructor)
		 * - creates <i>SharedObject</i> in the shared memory and moves the created shared data to it
		 * - creates a shared pointer to the <i>SharedObject</i>
		 * - creates <i>SharedObjectOwner</i> with the previously created shared object shared pointer
		 * 
		 * Returns true if all of the steps where successfull, otherwise returns false.
		*/
		template<typename... Args>
		bool createObject(Args&&... t_pack);

		/*!
		 * @brief Creates shared object in the shared memory
		 * @tparam ...Args Arguments' types for shared data constructor (may be empty)
		 * @param ...t_pack Arguments for shared data constructor (may be empty)
		 * @return True if the shared object is successfully created, otherwise false
		 * 
		 * Calls createObject with the first argument as the shared memory allocator 
		 * and forwards the rest of the arguments
		*/
		template <typename... Args>
		bool createObjectWithAllocator(Args&&... t_pack);
		/*!
		 * @brief Checks if the shared object is already created
		*/
		bool isObjectCreated();
		/*!
		 * @brief Updates shared data with a function
		 * @tparam CustomFunc A function with signature void(T&)
		 * @param t_update A function that is called with the reference to the shared data
		*/
		template <typename CustomFunc>
		void customUpdate(CustomFunc&& t_update);
		/*!
		 * @brief Copies \p t_data to the shared data
		*/
		void copyUpdate(const T& t_data);
		auto getSegmentManager();
		size_t getSegmentSize();

	private:
		/*!
		 * @brief Shared pointer to the owner of the shared object 
		*/
		ObjSharedPtr<SharedObjectOwner<T, StorageType>, StorageType> m_ownerSharedPtr;
		/*!
		 * @brief The size of the shared memory that needs to be allocated
		*/
		size_t m_shmemSize{ 0 };
		/*!
		 * @brief The shared memory segment manager
		*/
		StorageType m_shmemManager;
		/*!
		 * @brief The shared memory deallocator
		*/
		SharedMemoryRemover<StorageType> m_remover;

		/*!
		 * @brief Checks if the shared object is already created by some other producer
		*/
		bool isObjectConflict();
		/*!
		 * @brief Checks if a memory with the same name already exists
		*/
		bool isMemoryConflict();
	};

	template <class T, class StorageType>
	bool ShmemProducer<T, StorageType>::isMemoryConflict()
	{
		if (isMemoryOpen()) return false; // if this producer has acquired the memory, then there is no conflict
		try
		{
			auto tempManager = StorageType(bip::open_only, getMemoryName().c_str());
			std::cout << std::flush << "ShmemProducer: memory conflict: '" << getMemoryName() << "'\n";
			if (tempManager.get_segment_manager()) return true;
		}
		catch (bip::interprocess_exception& ex)
		{
			if (ex.get_error_code() != 7)
				std::cout << std::flush << "ShmemProducer: isMemoryConflict failed: '" << getMemoryName()
				<< "' Err: " << ex.get_error_code() << "; " << ex.what() << '\n';
		}
		catch (std::exception& stdEx)
		{
			std::cout << std::flush << "ShmemProducer: isMemoryConflict failed: '" << getMemoryName()
				<< "'; Err: " << stdEx.what() << '\n';
		}
		return false;
	}

	template <class T, class StorageType>
	bool ShmemProducer<T, StorageType>::isMemoryOpen()
	{
		return m_shmemManager.get_segment_manager() ? true : false;
	}

	template <class T, class StorageType>
	void ShmemProducer<T, StorageType>::memoryOpen()
	{
		try
		{
			// mapped file is created in the local binary folder if the path is not global
			m_shmemManager = StorageType(bip::create_only, getMemoryName().c_str(), m_shmemSize);
			//std::cout << std::flush << "ShmemProducer: memory opened: '" << getMemoryName() << "'\n";
		}
		catch (bip::interprocess_exception& ex)
		{
			std::cout << std::flush << "ShmemProducer: memory failed to open: '" << getMemoryName() 
				<< "'; Err: " << ex.get_error_code() << "; " << ex.what() << '\n';
		}
		catch (std::exception& stdEx)
		{
			std::cout << std::flush << "ShmemProducer: memory failed to open: '" << getMemoryName()
				<< "'; Err: " << stdEx.what() << '\n';
		}
	}

	template <class T, class StorageType>
	bool ShmemProducer<T, StorageType>::isObjectCreated()
	{
		return m_ownerSharedPtr.get() ? true : false;
	}

	template <class T, class StorageType>
	bool ShmemProducer<T, StorageType>::isObjectConflict()
	{
		if (isObjectCreated()) return false; // if this producer has acquired the memory, then there is no conflict
		try
		{
			auto tempObj = m_shmemManager.find<ObjSharedPtr<T, StorageType>>(getProducerName().c_str());
			if (tempObj.second)
			{
				std::cout << std::flush << "ShmemProducer: object conflict: '" << getProducerName() 
					<< "' in memory: '" << getMemoryName() << "'\n";
				return true;
			}
			// if another producer has already created the object then there IS a conflict
		}
		catch (bip::interprocess_exception& ex)
		{
			if (ex.get_error_code() == 7) return false;
			std::cout << std::flush << "ShmemProducer: is objectConflict failed: '" << getProducerName()
				<< "' in memory: '" << getMemoryName()
				<< "' Err: " << ex.get_error_code() << "; " << ex.what() << '\n';
			return true;

		}
		//std::cout << std::flush << "ShmemProducer: object created: '" << getProducerName() << "' in memory: '" << getMemoryName() << "'\n";
		return false;
	}


	template <class T, class StorageType>
	ShmemProducer<T, StorageType>::ShmemProducer(const std::string& t_producerName, const std::string& t_memoryName, const size_t t_shmemSize)
		: ShmemBase("mytype", t_producerName, t_memoryName), 
		m_shmemSize(t_shmemSize < bip::mapped_region::get_page_size() ? bip::mapped_region::get_page_size() : t_shmemSize),
		m_remover(getMemoryName())
	{
		if (isMemoryConflict()) return;
		memoryOpen();
		//if (isMemoryOpen()) std::cout << std::flush << "Memory was successfully opened for: " << getMemoryName() << '\n';
	}

	template <class T, class StorageType>
	ShmemProducer<T, StorageType>::~ShmemProducer()
	{
		m_ownerSharedPtr.reset();
		//std::cout << std::flush << "***********************************ShmemProducer destructed\n";
	}

	template <class T, class StorageType>
	bool ShmemProducer<T, StorageType>::isTypeOf(DataStructType t_id)
	{
		return true;
	}

	template <class T, class StorageType>
	template <typename... Args>
	bool ShmemProducer<T, StorageType>::createObject(Args&&... t_pack)
	{
		if (isObjectCreated()) return true; // already created it
		if (!isMemoryOpen()) return false; // memory is not opened
		try
		{
			// create data
			T* data = m_shmemManager.construct<T>(getDataName().c_str())(t_pack...);
			VoidAllocator<StorageType> allocInst = m_shmemManager.get_segment_manager();
			// create SharedObject
			SharedObject<T, StorageType>* sharedObject =
				m_shmemManager.construct<SharedObject<T, StorageType>>(getObjectName().c_str())(boost::move(*data), allocInst);
			// create shared ptr
			ObjSharedPtr<SharedObject<T, StorageType>, StorageType> sharedObjectPtr = bip::make_managed_shared_ptr<SharedObject<T, StorageType>>(sharedObject, m_shmemManager);
			// create owner
			SharedObjectOwner<T, StorageType>* owner = m_shmemManager.construct<SharedObjectOwner<T, StorageType>>(getOwnerName().c_str())(sharedObjectPtr);
			m_ownerSharedPtr = bip::make_managed_shared_ptr< SharedObjectOwner<T, StorageType>>(owner, m_shmemManager);
			return true;
		}
		catch (bip::interprocess_exception& ex)
		{
			std::cout << std::flush << "ShmemProducer: createObject failed: '" << getProducerName() 
				<< "' in memory: '" << getMemoryName() 
				<< "'; Err: " << ex.get_error_code() << ", " << ex.what() << '\n';
		}
		catch (std::exception& stdEx)
		{
			std::cout << std::flush << "ShmemProducer: createObject failed: '" << getProducerName()
				<< "' in memory: '" << getMemoryName() 
				<< "'; Err: " << stdEx.what() << '\n';
		}

		return false;
	}

	template <class T, class StorageType>
	template <typename... Args>
	bool ShmemProducer<T, StorageType>::createObjectWithAllocator(Args&&... t_pack)
	{
		return createObject(static_cast<VoidAllocator<StorageType>>(m_shmemManager.get_segment_manager()), t_pack...);
	}

	template <class T, class StorageType>
	template <typename CustomFunc>
	void ShmemProducer<T, StorageType>::customUpdate(CustomFunc&& t_update)
	{
		if (!isObjectCreated()) return;
		ObjSharedPtr<SharedObject<T, StorageType>, StorageType>& sharedObj = m_ownerSharedPtr->sharedData;
		sharedObj->customUpdate(std::forward<CustomFunc>(t_update));
	}

	template <class T, class StorageType>
	void ShmemProducer<T, StorageType>::copyUpdate(const T& t_data)
	{
		if (!isObjectCreated()) return;
		ObjSharedPtr<SharedObject<T, StorageType>, StorageType>& sharedObj = m_ownerSharedPtr->sharedData;
		*sharedObj = t_data;
	}


	template <class T, class StorageType>
	auto ShmemProducer<T, StorageType>::getSegmentManager()
	{
		return m_shmemManager.get_segment_manager();
	}

	template <class T, class StorageType>
	size_t ShmemProducer<T, StorageType>::getSegmentSize()
	{
		return m_shmemManager.get_size();
	}
}

#endif // !SHMEM_PRODUCER_H