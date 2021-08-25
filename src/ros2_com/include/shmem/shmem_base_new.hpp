/*!
* @file
* @ingroup CoreLibGroup
* @ingroup SharedMemory
*/
#ifndef SHMEM_BASE_NEW_HPP
#define SHMEM_BASE_NEW_HPP

#include <string>
#include <exception>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/managed_mapped_file.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/file_mapping.hpp>

#include <boost/interprocess/sync/sharable_lock.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_sharable_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition_any.hpp>

#include <boost/interprocess/smart_ptr/shared_ptr.hpp>
#include <boost/interprocess/smart_ptr/weak_ptr.hpp>

#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/containers/list.hpp>

namespace zbot::shmem
{
	namespace bip = boost::interprocess;

	template <typename StorageType> using ManagedSharedMemory =
		typename std::enable_if_t<std::is_same<StorageType, bip::managed_mapped_file>::value ||
		std::is_same<StorageType, bip::managed_shared_memory>::value, StorageType>;

	template <class StorageType> using SegmentManager = typename ManagedSharedMemory<StorageType>::segment_manager;
	template <class StorageType> using VoidAllocator = bip::allocator<void, SegmentManager<StorageType>>;

	template <class T, class StorageType> using CustomAllocator = bip::allocator<T, SegmentManager<StorageType>>;

	template <class StorageType> using ShmemCharAllocator = bip::allocator<char, SegmentManager<StorageType>>;
	template <class StorageType> using ShmemString = bip::basic_string<char, std::char_traits<char>, ShmemCharAllocator<StorageType>>;
	template <class StorageType> using ShmemStringAllocator = bip::allocator<ShmemString<StorageType>, SegmentManager<StorageType>>;

	template <class T, class StorageType> using ManagedSharedPtr = typename bip::managed_shared_ptr<T, ManagedSharedMemory<StorageType>>::type;
	template <class T, class StorageType> using ManagedWeakPtr = typename bip::managed_weak_ptr<T, ManagedSharedMemory<StorageType>>::type;

	using sharable_mutex = bip::interprocess_sharable_mutex;
	using ScopedSharableLock = bip::sharable_lock<sharable_mutex>;

	/*!
	 * @brief A simple struct that asserts if the StorageType is correct
	 * @tparam StorageType Currently two types are supported: bip::managed_mapped_file and bip::managed_shared_memory
	*/
	template <class StorageType>
	struct StorageAssert
	{
		static_assert(std::is_same<StorageType, bip::managed_mapped_file>::value ||
					  std::is_same<StorageType, bip::managed_shared_memory>::value,
					  "StorageType must be of type 'boost::interprocess::managed_shared_memory' or 'boost::interprocess::managed_mapped_file'");
		using type = StorageType;
	};

	/*!
	 * @brief A simple struct that does the raw data type checks for the shared memory
	*/
	template <typename T>
	struct RawDataTypeAsserter
	{
		static_assert(std::is_default_constructible<T>::value, "Shared memory RawDataTypeAsserter: input data type must be default constructible!");
		static_assert(std::is_copy_constructible<T>::value, "Shared memory RawDataTypeAsserter: input data type must be copy constructible!");
		static_assert(std::is_move_constructible<T>::value, "Shared memory RawDataTypeAsserter: input data type must be move constructible!");
		static_assert(std::is_copy_assignable<T>::value, "Shared memory RawDataTypeAsserter: input data type must be copy assignable!");
		static_assert(std::is_move_assignable<T>::value, "Shared memory RawDataTypeAsserter: input data type must be move assignable!");
		static_assert(!std::is_void<T>::value, "Shared memory RawDataTypeAsserter: input data type cannot be void!");
		static_assert(!std::is_function<T>::value, "Shared memory RawDataTypeAsserter: input data type cannot be a function!");
		static_assert(!std::is_pointer<T>::value, "Shared memory RawDataTypeAsserter: input data type cannot be a pointer!");
		static_assert(!std::is_member_pointer<T>::value, "Shared memory RawDataTypeAsserter: input data type cannot be a pointer!");
		static_assert(!std::is_reference<T>::value, "Shared memory RawDataTypeAsserter: input data type cannot be a reference!");
		static_assert(!std::is_polymorphic<T>::value, "Shared memory RawDataTypeAsserter: input data type cannot be polymorphic!");
		using type = T;
	};
	/*!
	 * @brief Empty placeholder
	*/
	struct DataStructType {};
	/*!
	 * @brief Base class for shared memory producers and consumers
	 * @tparam StorageType Only bip::managed_shared_memory and bip::managed_mapped_file types are supported
	 *
	 * @details
	 * Holds shared data:
	 * - type (string representation),
	 * - producer name,
	 * - object name,
	 * - data name,
	 * - owner name,
	 * - memory name
	 * 
	 * Multiple helper functions:
	 * - get allocater
	 * - shared memory manager
	 * - checks if the memory is open
	 * - checks if the shared object is referenced
	 * 
	 * Has incorporated monitoring tool that:
	 * - opens memory if necessary
	 * - references the shared object if necessary
	*/
	template <class StorageType>
	class ShmemBase : public BaseThread
	{
		using StorageAsserter = typename StorageAssert<StorageType>::type;
	public:
		/*!
		 * @brief Initializes the names of the shared data
		 * @param t_myType The type of the shared data
		 * @param t_producerName The name of the producer
		 * @param t_memoryName The name of the shared memory segment
		*/
		ShmemBase(const std::string& t_myType, const std::string& t_producerName, const std::string& t_memoryName)
			: BaseThread("ShmemBase"),
			m_myType(t_myType), m_producerName(t_producerName), m_objectName(t_producerName + "_Object"),
			m_dataName(t_producerName + "_Data"), m_ownerName(t_producerName + "_Owner"),
			m_memoryName(t_memoryName + "_Memory")
		{};
		virtual ~ShmemBase() {};

		/*!
		 * @brief A placeholder for determening if the instance is of type \p t_id
		 * @param t_id Somekind of a type identificator
		 * @return True if the instance is of the same type, otherwise returns false
		*/
		virtual bool isTypeOf(DataStructType t_id) const = 0;

		const std::string& getType() const { return m_myType; }
		const std::string& getProducerName() const { return m_producerName; }
		const std::string& getObjectName() const { return m_objectName; }
		const std::string& getDataName() const { return m_dataName; }
		const std::string& getOwnerName() const { return m_ownerName; }
		const std::string& getMemoryName() const { return m_memoryName; }

		/*!
		 * @brief Gets the shared memory allocator
		*/
		auto getAllocator() { return m_shmemManager.get_segment_manager(); }
		/*!
		 * @brief Gets the shared memory allocator
		*/
		const auto getAllocator() const { return m_shmemManager.get_segment_manager(); }
		/*!
		 * @brief Gets the reference of the shared memory manager
		*/
		StorageType& getShmemManagerRef() { return m_shmemManager; }
		/*!
		 * @brief Gets the reference of the shared memory manager
		*/
		const StorageType& getShmemManagerRef() const { return m_shmemManager; }
		/*!
		 * @brief Gets the pointer to the shared memory manager
		*/
		StorageType* getShmemManagerPtr() { return &m_shmemManager; }
		/*!
		 * @brief Gets the pointer to the shared memory manager
		*/
		const StorageType* getShmemManagerPtr() const { return &m_shmemManager; }
		/*!
		 * @brief Checks if the memory is open
		*/
		const bool isMemoryOpen() const { return getAllocator() ? true : false; }
		/*!
		 * @brief Implementation shoudl open/create the shared memory
		*/
		virtual void openMemory() = 0;
		/*!
		 * @brief Implementation should check if a reference to the shared object is acquired
		*/
		virtual bool isObjectReferenced() const = 0;
		/*!
		 * @brief Gets the size of the shared memory segment
		*/
		size_t getSegmentSize() const
		{
			if (isMemoryOpen()) return m_shmemManager.get_size();
			return 0U;
		}
	protected:
		/*!
		 * @brief The shared memory segment manager
		*/
		StorageType m_shmemManager;
		/*!
		 * @brief Checks if the shared object is referenced, if not then an exception is thrown
		*/
		void checkObject() const
		{
			if (isObjectReferenced()) return;
			throw std::invalid_argument("ShmemBase failed: Object is not referenced!");
		}
		/*!
		 * @brief Implementation should monitor the memory and object reference
		 * @return True if the memory is open and the object is referenced, otherwise returns false
		*/
		virtual bool monitorMemory() = 0;
		/*!
		 * @brief Changes the monitoring thread delay amount
		*/
		template <typename Rep, typename Period>
		constexpr void setThreadDelay(const std::chrono::duration<Rep, Period>& t_newDelay)
		{
			m_delay = std::chrono::duration_cast<std::chrono::microseconds>(t_newDelay);
		}
		/*!
		 * @brief Empty
		*/
		void onStart() override {}
		/*!
		 * @brief Empty
		*/
		void onStop() override {}
		/*!
		 * @brief Monitors the memory
		*/
		void run() override
		{
			while (isWorkerEnabled())
			{
				sleepFor(m_delay);
				monitorMemory();
			}
		}

	private:
		/*!
		 * @brief The string interpretation of the data type that is stored in the shared memory
		*/
		std::string m_myType;
		/*!
		 * @brief The name of the producer that created the shared object
		*/
		std::string m_producerName;
		/*!
		 * @brief The name that is used to create the <i>SharedObject</i> in the shared memory
		 * Equal to m_producerName + "_Object"
		*/
		std::string m_objectName;
		/*!
		 * @brief The name that is used to create the shared data in the shared memory
		 * Equal to m_producerName + "_Data"
		 */
		std::string m_dataName;
		/*!
		 * @brief The name that used to create the shared object owner in the shared memory
		 * Equal to m_producerName + "_Owner"
		 */
		std::string m_ownerName;
		/*!
		 * @brief The name that used to create the shared memory segment
		 * Equal to m_producerName + "_Memory"
		 */
		std::string m_memoryName;
		std::chrono::microseconds m_delay{ 100000 }; // 100 ms
	};

	/*!
	 * @brief Automatically removes the shared memory on creation and destruction
	 * @tparam StorageType Only bip::managed_shared_memory and bip::managed_mapped_file types are supported
	 *
	 * @details Use only in producer, because the producer is responsible for creating and destructing the memory
	*/
	template <class StorageType>
	struct SharedMemoryRemover
	{
		using StorageAsserter = typename StorageAssert<StorageType>::type;
		/*!
		 * @brief The name of the shared memory
		*/
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
		/*!
		 * @brief Removes the shared memory
		*/
		void removeMemory()
		{
			if (shmemName.empty()) return;
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
	 * @brief Shared object owner that is placed in the shared memoyr for automatic unreferencing of the object on destruction
	 * @tparam SharedData equals to SharedObject<T, StorageType>
	 * @tparam StorageType Only bip::managed_shared_memory and bip::managed_mapped_file types are supported
	*/
	template <class SharedData, class StorageType>
	struct SharedObjectOwner
	{
		using StorageAsserter = typename StorageAssert<StorageType>::type;
		SharedObjectOwner(const ManagedSharedPtr<SharedData, StorageType>& t_otherPtr) : sharedData(t_otherPtr) {};
		SharedObjectOwner(const SharedObjectOwner<SharedData, StorageType>& t_otherOwner) : sharedData(t_otherOwner.t_otherPtr) {};
		~SharedObjectOwner() = default;
		ManagedSharedPtr<SharedData, StorageType> sharedData;
	};
	/*!
	 * @brief Consumer base class for storing assertions, aliases and some variables
	*/
	template <class StorageType>
	struct ConsumerBase
	{
		ConsumerBase(ShmemString<StorageType>&& t_consumerName) 
			: name(boost::forward<ShmemString<StorageType>>(t_consumerName)) 
		{}
		using StorageAsserter = typename StorageAssert<StorageType>::type;
		/*!
		 * @brief The name of the consumer
		*/
		ShmemString<StorageType> name;
	};
	/*!
	 * @brief Shared object base class for storing assertions, aliases and some variables
	*/
	template <class T, class ConsumerType, class StorageType>
	class SharedObjectBase
	{
	public:
		static_assert(std::is_base_of<ConsumerBase<StorageType>, ConsumerType>::value 
					  || std::is_same<ConsumerBase<StorageType>, ConsumerType>::value, 
					  "SharedObjectBase: invalid ConsumerType");

		using value_type = T;

		using DataTypeAsserter = typename RawDataTypeAsserter<value_type>::type;
		using StorageAsserter = typename StorageAssert<StorageType>::type;
		using consumer_type = /*typename*/ ConsumerType;

		using ConsumerSharedPtr = ManagedSharedPtr<consumer_type, StorageType>;
		using ConsumerWeakPtr = ManagedWeakPtr<consumer_type, StorageType>;

		using ConsumerAllocator = bip::allocator<ConsumerSharedPtr, SegmentManager<StorageType>>;
		using ConsumerList = bip::list<ConsumerSharedPtr, ConsumerAllocator>;
		using ConsumerListAllocator = bip::allocator<ConsumerList, SegmentManager<StorageType>>;
		/*!
		 * @brief Constructs the base object with the specified @p t_allocator allocator
		*/
		SharedObjectBase(VoidAllocator<StorageType>& t_allocator) 
			: m_consumers(t_allocator) 
		{}
		/*!
		 * @brief Removes the consumer reference
		 * @param t_consumer The weak pointer of the ConsumerReference that needs to be removed
		 *
		 * Resets the \p t_consumer weak pointer.
		*/
		void removeConsumer(ConsumerWeakPtr& t_consumer)
		{
			if (t_consumer.expired())
			{
				t_consumer.reset();
				return;
			}
			ScopedSharableLock lock(m_mtx);
			auto consumerSharedPtr = t_consumer.lock();
			const ShmemString<StorageType> consumerName = consumerSharedPtr->name;
			auto c = m_consumers.begin();
			for (; c != m_consumers.end(); ++c)
			{
				if (consumerName == c->name) break;
			}
			if (c == m_consumers.end()) return;
			m_consumers.erase(c);
			t_consumer.reset();
		}

	protected:
		/*!
		 * @brief Mutex for exclusive/sharable access to the shared data
		*/
		mutable sharable_mutex m_mtx;
		/*!
		 * @brief A list of all the consumers that are reading the shared data
		*/
		ConsumerList m_consumers;
	};

}

#endif // !SHMEM_BASE_NEW_HPP