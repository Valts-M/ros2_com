#ifndef SHMEM_BASE_H
#define SHMEM_BASE_H

#include <string>
#include <cassert>
#include <iostream>
#include <chrono>

#include <boost/interprocess/sync/sharable_lock.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_sharable_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition_any.hpp>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/managed_mapped_file.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/file_mapping.hpp>

#include <boost/interprocess/smart_ptr/shared_ptr.hpp>
#include <boost/interprocess/smart_ptr/weak_ptr.hpp>

#include <boost/interprocess/containers/list.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/string.hpp>

#include <boost/date_time.hpp>
#include <boost/atomic/ipc_atomic_flag.hpp>

namespace ros2_com
{
	namespace bip = boost::interprocess;

	/*!
	 * @brief A simple convertor from <i>std::chrono::duration</i> to <i>boost::posix_time::microseconds</i>
	 * @param t_chronoTime Time duration in <i>std::chrono</i> format
	 * @return The corresponding <i>boost::posix::microseconds</i> value
	*/
	template <typename _Rep, typename _Period>
	constexpr boost::posix_time::microseconds usChronoToPosix(const std::chrono::duration<_Rep, _Period>& t_chronoTime)
	{
		return boost::posix_time::microseconds(std::chrono::duration_cast<std::chrono::microseconds>(t_chronoTime).count());
	}


	/*ALIASES*/
	template <typename StorageType> using ManagedSharedMemory =
		typename std::enable_if_t<std::is_same<StorageType, bip::managed_mapped_file>::value ||
		std::is_same<StorageType, bip::managed_shared_memory>::value, StorageType>;

	template <class StorageType> using SegmentManager = typename ManagedSharedMemory<StorageType>::segment_manager;
	template <class StorageType> using VoidAllocator = bip::allocator<void, SegmentManager<StorageType>>;

	template <class T, class StorageType> using CustomAllocator = bip::allocator<T, SegmentManager<StorageType>>;

	template <class StorageType> using ShmemCharAllocator = bip::allocator<char, SegmentManager<StorageType>>;
	template <class StorageType> using ShmemString = bip::basic_string<char, std::char_traits<char>, ShmemCharAllocator<StorageType>>;
	template <class StorageType> using ShmemStringAllocator = bip::allocator<ShmemString<StorageType>, SegmentManager<StorageType>>;

	template <class T, class StorageType> using ObjSharedPtr = typename bip::managed_shared_ptr<T, ManagedSharedMemory<StorageType>>::type;
	template <class T, class StorageType> using ObjWeakPtr = typename bip::managed_weak_ptr<T, ManagedSharedMemory<StorageType>>::type;


	template <class StorageType>
	struct ConsumerReference;

	template <class StorageType> using ConRefSharedPtr = ObjSharedPtr<ConsumerReference<StorageType>, StorageType>;
	template <class StorageType> using ConRefWeakPtr = ObjWeakPtr<ConsumerReference<StorageType>, StorageType>;

	template <class StorageType> using ConsumerReferenceAllocator = bip::allocator<ConRefSharedPtr<StorageType>, SegmentManager<StorageType>>;
	template <class StorageType> using ConsumerReferenceList = bip::list<ConRefSharedPtr<StorageType>, ConsumerReferenceAllocator<StorageType>>;
	template <class StorageType> using ConsumerReferenceListAllocator = bip::allocator<ConsumerReferenceList<StorageType>, SegmentManager<StorageType>>;

	/*-------*/


	struct DataStructType {};
	/*!
	 * @brief Base class for shared memory producers and consumers
	 * 
	 * Holds shared data:
	 * - type (string representation),
	 * - producer name,
	 * - object name,
	 * - data name,
	 * - owner name,
	 * - memory name
	*/
	class ShmemBase
	{
	public:
		/*!
		 * @brief Initializes the names of the shared data
		 * @param t_myType The type of the shared data
		 * @param t_producerName The name of the producer
		 * @param t_memoryName The name of the shared memory segment
		*/
		ShmemBase(const std::string& t_myType, const std::string& t_producerName, const std::string& t_memoryName)
			: m_myType(t_myType), m_producerName(t_producerName), m_objectName(t_producerName + "_Object"),
			m_dataName(t_producerName + "_Data"), m_ownerName(t_producerName + "_Owner"), 
			m_memoryName(t_memoryName + "_Memory")
		{};
		virtual ~ShmemBase() {};

		/*!
		 * @brief A placeholder for determening if the instance is of type \p t_id
		 * @param t_id Somekind of a type identificator
		 * @return True if the instance is of the same type, otherwise returns false
		*/
		virtual bool isTypeOf(DataStructType t_id) = 0;

		const std::string& getType() { return m_myType; }
		const std::string& getType() const { return m_myType; }
		const std::string& getProducerName() { return m_producerName; }
		const std::string& getProducerName() const { return m_producerName; }
		const std::string& getObjectName() { return m_objectName; }
		const std::string& getObjectName() const { return m_objectName; }
		const std::string& getDataName() { return m_dataName; }
		const std::string& getDataName() const { return m_dataName; }
		const std::string& getOwnerName() { return m_ownerName; }
		const std::string& getOwnerName() const { return m_ownerName; }
		const std::string& getMemoryName() { return m_memoryName; }
		const std::string& getMemoryName() const { return m_memoryName; }

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
	};

	template <class T, class StorageType>
	class ShmemProducer;

	/*!
	 * @brief Reference to the shared memory consumer
	 * @tparam StorageType The shared memory storage. Two possible storages:
	 * 'boost::interprocess::managed_shared_memory' or 'boost::interprocess::managed_mapped_file'
	*/
	template <class StorageType>
	struct ConsumerReference
	{
		static_assert(std::is_same<StorageType, bip::managed_mapped_file>::value ||
					  std::is_same<StorageType, bip::managed_shared_memory>::value,
					  "ConsumerReference: StorageType must be of type 'boost::interprocess::managed_shared_memory' or 'boost::interprocess::managed_mapped_file'");

		/*!
		 * @brief Creates consumer reference with the name \p t_consumerName
		*/
		ConsumerReference(ShmemString<StorageType>&& t_consumerName) : 
			name(boost::forward<ShmemString<StorageType>>(t_consumerName)) {}
		~ConsumerReference() { 
			//std::cout << std::flush << "***********************************ConsumerReference destructed\n";
		}
		/*!
		 * @brief Flag that corresponds whether there is new shared data
		*/
		bool newData{ false };

		
		/*!
		 * @brief The name of the consumer
		*/
		ShmemString<StorageType> name;
	};

	/*!
	 * @brief The object that is placed in the shared memory
	 * @tparam T Data type that is stored in the shared memory
	 * @tparam StorageType The shared memory storage. Two possible storages:
	 * 'boost::interprocess::managed_shared_memory' or 'boost::interprocess::managed_mapped_file'
	 * 
	 * This class has the shared data and multiple functions for reading from/writing to the shared data
	*/
	template <class T, class StorageType>
	class SharedObject
	{
	public:
		static_assert(std::is_same<StorageType, bip::managed_mapped_file>::value ||
					  std::is_same<StorageType, bip::managed_shared_memory>::value,
					  "SharedObject: StorageType must be of type 'boost::interprocess::managed_shared_memory' or 'boost::interprocess::managed_mapped_file'");

		/*!
		 * @brief Constructs shared object
		 * @param t_data rvalue reference to the shared data that is constructed in the shared memory
		 * @param t_allocator Allocator for allocating memory in the shared memory for consumer list 
		*/
		SharedObject(T&& t_data, VoidAllocator<StorageType>& t_allocator) :
			data(boost::forward<T>(t_data)), consumers(t_allocator)
		{}
		~SharedObject() 
		{
			//std::cout << std::flush << "***********************************SharedObject destructed\n";
		}
		/*!
		 * @brief Adds consumer reference
		 * @param t_consumerName The name of the consumer
		 * @param t_storage The shared memory manager where the shared object is placed
		 * @param t_out[out] Outputs weak pointer to the ConsumerReference
		 * @return Return true if ConsumerReference was successfully created, otherwise
		 * returns false.
		*/
		bool addConsumer(ShmemString<StorageType>&& t_consumerName, StorageType& t_storage, ConRefWeakPtr<StorageType>& t_out)
		{
			bip::scoped_lock<bip::interprocess_sharable_mutex> lock(mtx);
			try
			{
				consumers.push_back(
					bip::make_managed_shared_ptr<ConsumerReference<StorageType>>(
					t_storage.template construct<ConsumerReference<StorageType>>(t_consumerName.c_str())(
					boost::forward<ShmemString<StorageType>>(t_consumerName)),
					t_storage));
				t_out = ConRefWeakPtr<StorageType>(consumers.back());
				return true;
			}
			catch (bip::interprocess_exception& ex)
			{
				std::cout << "SharedObject failed to add consumer: '" << t_consumerName 
					<< "'; Err.: " << ex.get_error_code() << "; " << ex.what() << '\n';
			}
			catch (std::exception& ex)
			{
				std::cout << "SharedObject failed to add consumer: '" << t_consumerName 
					<< "'; Err.: " << ex.what() << '\n';
			}
			return false;
		}
		/*!
		 * @brief Removes the consumer reference
		 * @param t_consumer The weak pointer of the ConsumerReference that needs to be removed
		 * 
		 * Resets the \p t_consumer weak pointer.
		*/
		void removeConsumer(ConRefWeakPtr<StorageType>& t_consumer)
		{
			if (t_consumer.expired())
			{
				t_consumer.reset();
				return;
			}
			bip::scoped_lock<bip::interprocess_sharable_mutex> lock(mtx);
			auto consumerSharedPtr = t_consumer.lock();
			const ShmemString<StorageType> consumerName = consumerSharedPtr->name;
			auto c = consumers.begin();
			for (; c != consumers.end(); ++c)
			{
				if (consumerName == c->name) break;
			}
			if (c == consumers.end()) return;
			c->killAsync();
			consumers.erase(c);
			t_consumer.reset();
		}

		/*!
		 * @brief Update the shared data wih a custom function
		 * @tparam CustomUpdateFunc A function with signature <i>void(T&)</i>
		 * @param t_update Function that is called with reference to the 
		 * shared data <i>t_update(static_cast<T&>(data))</i>
		*/
		template <typename CustomUpdateFunc>
		void customUpdate(CustomUpdateFunc&& t_update)
		{
			bip::scoped_lock<bip::interprocess_sharable_mutex> lock(mtx);
			t_update(static_cast<T&>(data));
			setNewData();
		}

		/*!
		 * @brief Update the shared data wih a copy-assignment
		 * @param t_data Process-scoped data that is copied to the shared data
		*/
		SharedObject<T, StorageType>& operator=(const T& t_data)
		{
			bip::scoped_lock<bip::interprocess_sharable_mutex> lock(mtx);
			data = t_data;
			setNewData();			
			return *this;
		}

		/*!
		 * @brief Polls shared data from the shared object
		 * @tparam PollCallback A function with signature <i>void(const T&)</i>
		 * @param t_consumer Reference to the consumer that is waiting for the shared data
		 * @param t_get A function that will be called with the shared data as the argument
		 * @return True if new data was recieved, otherwise false
		 *
		 * Polls for new shared data. If there is no new data then false is returned.
		 * Otherwise calls the function with the shared data <i>t_get(static_cast<const T&>(sharedData))</i>
		*/
		template <typename PollCallback>
		bool poll(ConRefWeakPtr<StorageType>& t_consumer, PollCallback&& t_get)
		{
			if (t_consumer.expired()) return false;
			bip::sharable_lock<bip::interprocess_sharable_mutex> lock(mtx);
			ConRefSharedPtr<StorageType> tmpShared = t_consumer.lock();
			if (!tmpShared->newData) return false;
			t_get(static_cast<const T&>(data));
			tmpShared->newData = false;
			return true;
		}

		/*!
		 * @brief Polls shared data from the shared object
		 * @param t_consumer Reference to the consumer that is waiting for the shared data
		 * @param t_dataOut Reference of the process-scoped object where the shared data is copied to
		 * @return True if new data was recieved, otherwise false
		 *
		 * Polls for new shared data. If there is no new data then false is returned.
		 * Otherwise calls copy-assignment with the shared data <i>t_dataOut = sharedData</i>
		*/
		bool pollCopy(ConRefWeakPtr<StorageType>& t_consumer, T& t_dataOut)
		{
			if (t_consumer.expired()) return false;
			bip::sharable_lock<bip::interprocess_sharable_mutex> lock(mtx);
			ConRefSharedPtr<StorageType> tmpShared = t_consumer.lock();
			if (!tmpShared->newData) return false;
			t_dataOut = data;
			tmpShared->newData = false;
			return true;
		}

		/*!
		 * @brief Waits for new shared data in the shared object for a specific amount of time
		 * @param t_consumer Reference to the consumer that is waiting for the shared data
		 * @param t_timeout The amount of time to wait for new shared data in the shared object
		 * @param t_dataOut Reference of the process-scoped object where the shared data is copied to
		 * @return True if a value was recieved. False if the timeout occured.
		 *
		 * Waits for new shared data using condition variable. If timeout occurs then the function 
		 * returns false. If the producer notifies consumer then the copy-assignment
		 * is called with the shared data <i>t_dataOut = sharedData</i> and true is returned.
		*/
		template <class _Rep, class _Period>
		bool waitCopy(ConRefWeakPtr<StorageType>& t_consumer, const std::chrono::duration<_Rep, _Period>& t_timeout, T& t_dataOut)
		{
			if (t_consumer.expired()) return false;
			
			bip::scoped_lock<bip::interprocess_sharable_mutex> lock(mtx);
			ConRefSharedPtr<StorageType> tmpShared = t_consumer.lock();

			using clock = boost::posix_time::microsec_clock;
			auto usTimeout = usChronoToPosix(t_timeout);
			
			if (!condition.timed_wait(lock, clock::universal_time() + usTimeout, [&] { return tmpShared->newData; }))
			{
				//std::cout << "SharedObject: waitCopy timed out\n";
				return false;
			}
			t_dataOut = data;
			tmpShared->newData = false;
			//std::cout << "SharedObject: waitCopy was successfull\n";
			return true;
		}

		/*!
		 * @brief Waits for new shared data in the shared object for a specific amount of time
		 * @tparam RecieveCallback A function with a signature of <i>void(const T&)</i>
		 * @param t_consumer Reference to the consumer that is waiting for the shared data
		 * @param t_timeout The amount of time to wait for the input from the producer
		 * @param t_get A function that will be called with the shared data as the argument
		 * @return True if a value was recieved. False if the timeout occured.
		 *
		 * This function may be used when a simple copy-assignment is not eligible.
		 * Waits for new shared data using condition variable. If timeout occurs then the function 
		 * returns false. If the producer notifies consumer then the custom function \p t_get 
		 * is called with the shared data  <i>t_get(static_cast<const T&>(sharedData))</i> 
		 * and true is returned.
		*/
		template <class _Rep, class _Period, typename RecieveCallback>
		bool wait(ConRefWeakPtr<StorageType>& t_consumer, const std::chrono::duration<_Rep, _Period>& t_timeout,  RecieveCallback&& t_get)
		{
			if (t_consumer.expired()) return false;

			bip::scoped_lock<bip::interprocess_sharable_mutex> lock(mtx);
			ConRefSharedPtr<StorageType> tmpShared = t_consumer.lock();

			using clock = boost::posix_time::microsec_clock;
			auto usTimeout = usChronoToPosix(t_timeout);

			if (!condition.timed_wait(lock, clock::universal_time() + usTimeout, [&] { return tmpShared->newData; }))
			{
				//std::cout << "SharedObject: wait timed out\n";
				return false;
			}
			t_get(static_cast<const T&>(data));
			tmpShared->newData = false;
			//std::cout << "SharedObject: wait was successfull\n";
			return true;
		}
		/*!
		 * @brief Removes the references to the consumers
		 * 
		 * Consumers are referenced with shared pointers, so if no other reference to the 
		 * consumer is active then the consumer is destroyed
		*/
		void removeConsumers()
		{
			for (auto it = consumers.begin(); it != consumers.end(); ++it)
				(*it).reset();
		}

	private:
		/*!
		 * @brief Notifies consumers that there is new shared data 
		*/
		void setNewData()
		{
			for (auto it = consumers.begin(); it != consumers.end(); ++it)
				(*it)->newData = true;
			condition.notify_all();
		}
		
		/*!
		 * @brief Mutex for exclusive/sharable access to the shared data
		*/
		bip::interprocess_sharable_mutex mtx;
		/*!
		 * @brief Condition variable that is used to notify consumers that there is new shared data
		*/
		bip::interprocess_condition_any condition;
		/*!
		 * @brief A list of all the consumers that are reading the shared data
		*/
		ConsumerReferenceList<StorageType> consumers;
		/*!
		 * @brief The shared data
		*/
		T data;
	};

	/*!
	 * @brief Shared object owner
	 * @tparam T Data type that is stored in the shared memory
	 * @tparam StorageType The shared memory storage. Two possible storages:
	 * 'boost::interprocess::managed_shared_memory' or 'boost::interprocess::managed_mapped_file'
	 * 
	 * This is used for RAII, so that when this is destroyed, then <i>SharedObject</i> with <i>ConsumerReference</i> is also destroyed
	*/
	template <class T, class StorageType>
	struct SharedObjectOwner
	{
		SharedObjectOwner(const ObjSharedPtr<SharedObject<T, StorageType>, StorageType>& t_otherPtr) : sharedData(t_otherPtr) {};
		SharedObjectOwner(const SharedObjectOwner<T, StorageType>& t_otherOwner) : sharedData(t_otherOwner.t_otherPtr) {};
		~SharedObjectOwner()
		{
			//std::cout << std::flush << "***********************************SharedObjectOwner destructed\n";
		}
		ObjSharedPtr<SharedObject<T, StorageType>, StorageType> sharedData;
	};
}
#endif // SHMEM_BASE_H