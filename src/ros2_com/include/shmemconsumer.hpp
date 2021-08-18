/*!
* @file
*/

#ifndef SHMEM_CONSUMER_H
#define SHMEM_CONSUMER_H

#include <functional>
#include <thread>
#include <memory>
#include <mutex>

#include "shmembase.hpp"

namespace ros2_com
{

	/*!
	 * @brief Class that is meant for running seperate thread for asynchronous shared data reading
	 * @tparam T Data type that is stored in the shared memory
	 * @tparam StorageType The shared memory storage. Two possible storages:
	 * 'boost::interprocess::managed_shared_memory' or 'boost::interprocess::managed_mapped_file'
	*/
	template <class T, class StorageType>
	class AsyncWaiter
	{
		static_assert(std::is_same<StorageType, bip::managed_mapped_file>::value ||
					  std::is_same<StorageType, bip::managed_shared_memory>::value,
					  "AsyncWaiter: StorageType must be of type 'boost::interprocess::managed_shared_memory' or 'boost::interprocess::managed_mapped_file'");
	public:
		/*!
		 * @brief Constructs asynchronous waiter
		 * @tparam OnRecieveCallback A function with signature <i>void(const T&)</i>
		 * @param t_sharedObj Shared pointer to the shared object
		 * @param t_consumerRef Weak pointer to the consumer reference
		 * @param t_onRecieve Function that is called when new shared data has arrived
		 * 
		 * Creates a seperate thread that continuously waits for a notification from the shared producer
		*/
		template <typename OnRecieveCallback>
		AsyncWaiter(ObjSharedPtr<SharedObject<T, StorageType>, StorageType>& t_sharedObj, ConRefWeakPtr<StorageType>& t_consumerRef, OnRecieveCallback&& t_onRecieve)
			: m_sharedObj(t_sharedObj), m_consumerRef(t_consumerRef)
		{
			auto waiterFunc = [&](OnRecieveCallback&& t_callback) mutable
			{
				//std::cout << std::flush << "AsyncWaiter started!\n";
				while (!isStop())
				{
					if (!m_sharedObj.get() || m_consumerRef.expired()) break;
					if (!m_sharedObj->wait(m_consumerRef, std::chrono::milliseconds(100), std::forward<OnRecieveCallback>(t_callback))) continue;
					//std::cout << std::flush << "AsyncWaiter new recieved something!\n";
				}
				std::lock_guard<std::mutex> lock(m_mtx);
				b_stop = true;
				//std::cout << std::flush << "AsyncWaiter stopped!\n";
			};
			try
			{
				m_asyncThread = std::thread(waiterFunc, std::forward<OnRecieveCallback>(t_onRecieve));
				m_threadRunning = true;
			}
			catch (std::system_error& ex)
			{
				std::cout << std::flush << "AsyncWaiter: Thread execute failed: " << ex.code() << ", " << ex.what() << '\n';
			}
		}
		
		~AsyncWaiter()
		{
			stopThread();
		}
		/*!
		 * @brief Checks if the stop flag is set
		*/
		bool isStop()
		{
			std::lock_guard<std::mutex> lock(m_mtx);
			return b_stop;
		}

		/*!
		 * @brief Tries to stop and join the thread
		*/
		void stopThread()
		{
			{
				std::lock_guard<std::mutex> lock(m_mtx);
				b_stop = true;
			}
			if (!m_asyncThread.joinable())
			{
				m_threadRunning = false;
				return;
			}
			try
			{
				m_asyncThread.join();
				m_threadRunning = false;
			}
			catch (std::system_error& ex)
			{
				std::cout << std::flush << "AsyncWaiter: Thread join failed: " << ex.code() << ", " << ex.what() << '\n';
			}
		}
		/*!
		 * @brief Checks if the thread is running
		*/
		const bool& isThreadRunning() { return m_threadRunning; }
		/*!
		 * @brief Checks if the thread is running
		*/
		const bool& isThreadRunning() const { return m_threadRunning; }
	private:
		/*!
		 * @brief Thread handler
		*/
		std::thread m_asyncThread;
		/*!
		 * @brief Mutex for exclusive stop flag usage
		*/
		std::mutex m_mtx;
		/*!
		 * @brief Flag that triggers the thread to stop
		*/
		bool b_stop{ false };
		/*!
		 * @brief Flag that corresponds to the current state of the thread (running / not running)
		*/
		bool m_threadRunning{ false };
		/*!
		 * @brief Shared pointer to the shared object
		*/
		ObjSharedPtr<SharedObject<T, StorageType>, StorageType> m_sharedObj;
		/*!
		 * @brief Weak pointer to the consumer reference
		*/
		ConRefWeakPtr<StorageType> m_consumerRef;
	};

	/*!
	 * @brief Shared object wrapper for reading shared data
	 * @tparam T Data type that is stored in the shared memory
	 * @tparam StorageType The shared memory storage. Two possible storages: 
	 * 'boost::interprocess::managed_shared_memory' or 'boost::interprocess::managed_mapped_file'
	 * 
	 * Restrictions (see boost interprocess shared memory limitations)\n
	 * - do not insert objects with raw pointers, use boost::offset_ptr
	 * - do not use references, use boost::offset_ptr
	 * - do not use virtuality (inheritance)
	 * - do not use static members/function
	*/
	template <class T, class StorageType = bip::managed_shared_memory>
	class ShmemConsumer: public ShmemBase
	{
		static_assert(std::is_same<StorageType, bip::managed_mapped_file>::value ||
					  std::is_same<StorageType, bip::managed_shared_memory>::value,
					  "ShmemConsumer: StorageType must be of type 'boost::interprocess::managed_shared_memory' or 'boost::interprocess::managed_mapped_file'");

	public:
		/*!
		 * @brief Constructs the shared memory consumer and opens the shared memory segment
		 * @param t_producerName The name of the producer that created the shared object
		 * @param t_memoryName The name of the memory segment where the shared object is located
		 * @param t_consumerName The name of the current shared memory consumer instance
		 * 
		 * Tries to open the shared memory segment of name \p t_memoryName.
		*/
		ShmemConsumer(const std::string& t_producerName, const std::string& t_memoryName, const std::string& t_consumerName);
		~ShmemConsumer();

		bool isTypeOf(DataStructType t_id) override;

		/*!
		 * @brief Checks whether the shared memory is opened
		*/
		bool isMemoryOpen();
		/*!
		 * @brief Tries to open the shared memory
		*/
		void memoryOpen();

		/*!
		 * @brief Tries to find the shared object and create a reference to it
		 * @return True if the shared object has been found, otherwise returns fales
		*/
		bool findObject();
		/*!
		 * @brief Checks if the reference to the shared object is created
		*/
		bool isObjectFound();

		
		/*!
		 * @brief Polls data from the shared object
		 * @tparam PollCallback A function with signature <i>void(const T&)</i>
		 * @param t_get A function that will be called with the shared data as the argument
		 * @return True if a new data was recieved, otherwise false
		 * 
		 * Polls for new shared data in the shared object. If there is no new data then false is returned.
		 * Otherwise calls the function with the shared data <i>t_get(static_cast<const T&>(sharedData))</i>
		*/
		template <typename PollCallback>
		bool poll(PollCallback&& t_get);

		/*!
		 * @brief Polls shared data from the shared object
		 * @param t_dataOut Reference of the process-scoped object where the shared data is copied to
		 * @return True if new data was recieved, otherwise false
		 * 
		 * Polls for new shared data in the shared object. If there is no new data then false is returned.
		 * Otherwise calls copy-assignment with the shared data <i>t_dataOut = sharedData</i>
		*/
		bool pollCopy(T& t_dataOut);

		/*!
		 * @brief Waits for new shared data in the shared object for a specific amount of time
		 * @param t_dataOut Reference of the process-scoped object where the shared data is copied to
		 * @param t_timeout The amount of time to wait for new shared data in the shared object
		 * @return True if a value was recieved. False if the timeout occured.
		 * 
		 * Waits for new shared data in the shared object using condition variable. If timeout occurs
		 * then the function returns false. If the producer notifies consumer then the copy-assignment
		 * is called with the shared data <i>t_dataOut = sharedData</i> and true is returned.
		*/
		template <class _Rep, class _Period>
		bool waitCopy(T& t_dataOut, const std::chrono::duration<_Rep, _Period>& t_timeout);

		/*!
		 * @brief Waits for new shared data in the shared object for a specific amount of time
		 * @tparam RecieveCallback A function with a signature of <i>void(const T&)</i>
		 * @param t_timeout The amount of time to wait for the input from the producer
		 * @param t_get A function that will be called with the shared data as the argument
		 * @return True if a value was recieved. False if the timeout occured.
		 * 
		 * This function may be used when a simple copy-assignment is not eligible.
		 * Waits for new shared data in the shared object using condition variable. If timeout occurs
		 * then the function returns false. If the producer notifies consumer then the custom
		 * function \p t_get is called with the shared data 
		 * <i>t_get(static_cast<const T&>(sharedData))</i> and true is returned.
		*/
		template <class _Rep, class _Period, typename RecieveCallback>
		bool wait(const std::chrono::duration<_Rep, _Period>& t_timeout, RecieveCallback&& t_get);

		/*!
		 * @brief Start a thread that starts an asynchronous wait
		 * @tparam RecieveCallback A function with a signature of <i>void(const T&)<\i>
		 * @param t_get A function that will be called with the shared data as the argument
		 * @return True if the thread is running, otherwise- false
		 * 
		 * Starts a thread with a loop, in which it waits for new shared data in the shared object.
		 * When the producer notifies the consumer of the new shared data, then the custom \p t_get
		 * function is called <i>t_get(static_cast<const T&>(sharedData))</i>. If the thread was
		 * succesfully started then true is returned, if something failed then false ir returned.
		*/
		template <typename RecieveCallback>
		bool asyncWait(RecieveCallback&& t_get);

		/*!
		 * @brief Stop the asynchronous waiting thread
		*/
		void stopAsync();
		/*!
		 * @brief Checks whether the asynchronous waiting thread is running
		*/
		bool isAsyncWaiting();
		
	private:
		/*!
		 * @brief Pointer to the shared object (weak pointer)
		*/
		ObjWeakPtr<SharedObject<T, StorageType>, StorageType> m_objectWeakPtr;

		/*!
		 * @brief The shared memory manager
		*/
		StorageType m_shmemManager;
		/*!
		 * @brief The name of the consumer
		*/
		std::string m_consumerName;
		/*!
		 * @brief Reference to the consumer in the shared memory
		*/
		ConRefWeakPtr<StorageType> m_consumer;
		/*!
		 * @brief Asynchronous thread that waits for the shared data from the producer
		*/
		std::unique_ptr<AsyncWaiter<T, StorageType>> m_asyncWaiter;
		
		/*!
		 * @brief Adds a consumer reference to the shared object
		*/
		void addConsumer();
	};

	
	template <class T, class StorageType>
	bool ShmemConsumer<T, StorageType>::isMemoryOpen()
	{
		return m_shmemManager.get_segment_manager() ? true : false;
	}

	template <class T, class StorageType>
	void ShmemConsumer<T, StorageType>::memoryOpen()
	{
		try
		{
			m_shmemManager = StorageType(bip::open_only, getMemoryName().c_str());
			//std::cout << std::flush << "ShmemConsumer: memory opened: '" << getMemoryName() << "'\n";
		}
		catch (bip::interprocess_exception& ex)
		{
			std::cout << std::flush << "ShmemConsumer: memory failed to open: '" << getMemoryName()
				<< "'; Err: " << ex.get_error_code() << ", " << ex.what() << '\n';
		}
		catch (std::exception& stdEx)
		{
			std::cout << std::flush << "ShmemConsumer: memory failed to open: '" << getMemoryName()
				<< "'; Err: " << stdEx.what() << '\n';
		}
	}

	template <class T, class StorageType>
	bool ShmemConsumer<T, StorageType>::isObjectFound()
	{
		return m_objectWeakPtr.expired() ? false : true;
	}

	template <class T, class StorageType>
	bool ShmemConsumer<T, StorageType>::findObject()
	{
		if (isObjectFound()) return true;
		if (!isMemoryOpen()) return false;
		try
		{
			auto owner = m_shmemManager.find< SharedObjectOwner<T, StorageType>>(getOwnerName().c_str());
			if (owner.second != 1)
			{
				std::cout << std::flush << "ShmemConsumer: findObject failed: '" << getProducerName() 
					<< "' in memory: '" << getMemoryName() << "'\n";
				return false;
			}
			else
			{
				m_objectWeakPtr = ObjWeakPtr<SharedObject<T, StorageType>, StorageType>(owner.first->sharedData);
			}
		}
		catch (bip::interprocess_exception& ex)
		{
			std::cout << std::flush << "ShmemConsumer: findObject failed: '" << getProducerName()
				<< "' in memory: '" << getMemoryName()
				<< "'; Err: " << ex.get_error_code() << ", " << ex.what() << '\n';
			return false;
		}
		catch (std::exception& stdEx)
		{
			std::cout << std::flush << "ShmemConsumer: findObject failed: '" << getProducerName()
				<< "' in memory: '" << getMemoryName() 
				<< "'; Err: " << stdEx.what() << '\n';
			return false;
		}

		//std::cout << std::flush << "ShmemConsumer: object found: '" << getProducerName() << "'\n";
		addConsumer();
		return true;
	}


	template <class T, class StorageType>
	ShmemConsumer<T, StorageType>::ShmemConsumer(const std::string& t_producerName, const std::string& t_memoryName, const std::string& t_consumerName)
		: ShmemBase("mytype", t_producerName, t_memoryName), m_consumerName(t_consumerName)
	{
		memoryOpen();
		//if (isMemoryOpen()) std::cout << std::flush << "Memory was successfully opened for: " << getMemoryName() << '\n';
	}

	template <class T, class StorageType>
	ShmemConsumer<T, StorageType>::~ShmemConsumer()
	{
		m_objectWeakPtr.reset();
		m_asyncWaiter.reset();
		//std::cout << std::flush << "***********************************ShmemConsumer destructed\n";
	}

	template <class T, class StorageType>
	bool ShmemConsumer<T, StorageType>::isTypeOf(DataStructType t_id)
	{
		return true;
	}

	template <class T, class StorageType>
	void ShmemConsumer<T, StorageType>::addConsumer()
	{
		if (!isObjectFound()) return;
		auto sharedPtr = m_objectWeakPtr.lock();
		ShmemString<StorageType> consumerName = ShmemString<StorageType>(m_consumerName.c_str(), m_shmemManager.get_segment_manager());
		sharedPtr->addConsumer(boost::move(consumerName), m_shmemManager, m_consumer);
	}

	template <class T, class StorageType>
	template <typename PollCallback>
	bool ShmemConsumer<T, StorageType>::poll(PollCallback&& t_get)
	{
		if (!isObjectFound()) return false;
		ObjSharedPtr<SharedObject<T, StorageType>, StorageType> objectSharedPtr = m_objectWeakPtr.lock();
		return objectSharedPtr->poll(m_consumer, std::forward<PollCallback>(t_get));
	}

	template <class T, class StorageType>
	bool ShmemConsumer<T, StorageType>::pollCopy(T& t_dataOut)
	{
		if (!isObjectFound()) return false;
		ObjSharedPtr<SharedObject<T, StorageType>, StorageType> objectSharedPtr = m_objectWeakPtr.lock();
		return objectSharedPtr->pollCopy(m_consumer, std::forward<T>(t_dataOut));
	}

	template <class T, class StorageType>
	template <class _Rep, class _Period>
	bool ShmemConsumer<T, StorageType>::waitCopy(T& t_dataOut, const std::chrono::duration<_Rep, _Period>& t_timeout)
	{
		if (!isObjectFound()) return false;
		ObjSharedPtr<SharedObject<T, StorageType>, StorageType> objectSharedPtr = m_objectWeakPtr.lock();
		return objectSharedPtr->waitCopy(m_consumer, std::forward<std::chrono::duration<_Rep, _Period>>(t_timeout), std::forward<T>(t_dataOut));
	}

	template <class T, class StorageType>
	template <class _Rep, class _Period, typename RecieveCallback>
	bool ShmemConsumer<T, StorageType>::wait(const std::chrono::duration<_Rep, _Period>& t_timeout, RecieveCallback&& t_get)
	{
		if (!isObjectFound()) return false;
		ObjSharedPtr<SharedObject<T, StorageType>, StorageType> objectSharedPtr = m_objectWeakPtr.lock();
		return objectSharedPtr->wait(m_consumer, std::forward<std::chrono::duration<_Rep, _Period>>(t_timeout), std::forward<RecieveCallback>(t_get));
	}

	template <class T, class StorageType>
	template <typename RecieveCallback>
	bool ShmemConsumer<T, StorageType>::asyncWait(RecieveCallback&& t_get)
	{
		if (!isObjectFound()) return false;
		m_asyncWaiter.reset();
		ObjSharedPtr<SharedObject<T, StorageType>, StorageType> objectSharedPtr = m_objectWeakPtr.lock();
		m_asyncWaiter = std::make_unique<AsyncWaiter<T, StorageType>>(objectSharedPtr, m_consumer, std::forward<RecieveCallback>(t_get));
		if (m_asyncWaiter->isThreadRunning()) return true;
		m_asyncWaiter.reset();
		return false;
	}

	template <class T, class StorageType>
	void ShmemConsumer<T, StorageType>::stopAsync()
	{
		m_asyncWaiter.reset();
	}

	template <class T, class StorageType>
	bool ShmemConsumer<T, StorageType>::isAsyncWaiting()
	{
		return m_asyncWaiter.get() ? m_asyncWaiter->isThreadRunning() : false;
	}

}

#endif // !SHMEM_CONSUMER_H