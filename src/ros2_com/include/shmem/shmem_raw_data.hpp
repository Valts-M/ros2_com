/*!
* @file
* @ingroup CoreLibGroup
* @ingroup SharedMemory
* @ingroup DataStructures
*/

#ifndef SHMEM_RAW_DATA_HPP
#define SHMEM_RAW_DATA_HPP

#include <boost/date_time.hpp>
#include <boost/interprocess/containers/list.hpp>

#include "shmem_base_new.hpp"

namespace zbot::shmem
{
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

	/*!
	 * @brief Simple data structure for referencing the consumer
	 * @tparam StorageType Only bip::managed_shared_memory and bip::managed_mapped_file types are supported
	*/
	template <class StorageType>
	struct ConsumerReference : public ConsumerBase<StorageType>
	{
		/*!
		 * @brief Creates consumer reference with the name \p t_consumerName
		*/
		ConsumerReference(ShmemString<StorageType>&& t_consumerName) 
			: ConsumerBase<StorageType>(boost::forward<ShmemString<StorageType>>(t_consumerName))
		{}
		/*!
		 * @brief Flag that corresponds whether there is new shared data
		*/
		bool newData{ false };
	};

	/*!
	 * @brief The object that is placed in the shared memory
	 * @tparam T Data type that is stored in the shared memory
	 * @tparam StorageType Only bip::managed_shared_memory and bip::managed_mapped_file types are supported
	 *
	 * This class has the shared data and multiple functions for reading from/writing to the shared data
	*/
	template <class T, class StorageType>
	class SharedRawObject : public SharedObjectBase<T, ConsumerReference<StorageType>, StorageType>
	{
	public:
		using this_type = SharedRawObject<T, StorageType>;
		using base_type = SharedObjectBase<T, ConsumerReference<StorageType>, StorageType>;
		using typename base_type::consumer_type;
		using typename base_type::ConsumerWeakPtr;
		using typename base_type::ConsumerSharedPtr;

		using value_type = T;
		using reference_type = value_type&;
		using const_reference_type = const value_type&;
		using rvalue_reference_type = value_type&&;

		/*!
		 * @brief Constructs shared object
		 * @param t_data rvalue reference to the shared data that is constructed in the shared memory
		 * @param t_allocator Allocator for allocating memory in the shared memory for consumer list
		*/
		SharedRawObject(VoidAllocator<StorageType>& t_allocator)
			: base_type(t_allocator), m_data()
		{}
		/*!
		 * @brief Adds consumer reference
		 * @param t_consumerName The name of the consumer
		 * @param t_storage The shared memory manager where the shared object is placed
		 * @param t_out[out] Outputs weak pointer to the ConsumerReference
		 * @return Return true if ConsumerReference was successfully created, otherwise
		 * returns false.
		*/
		bool addConsumer(ShmemString<StorageType>&& t_consumerName, StorageType& t_storage, ConsumerWeakPtr& t_out)
		{
			ScopedSharableLock lock(m_mtx);
			try
			{
				auto objTemp = t_storage.construct<consumer_type>(t_consumerName.c_str())(
					boost::forward<ShmemString<StorageType>>(t_consumerName));
				// make shared ptr
				auto objTempSharedPtr = bip::make_managed_shared_ptr<consumer_type>(objTemp, t_storage);
				// push back
				m_consumers.push_back(boost::move(objTempSharedPtr));
				t_out = ConsumerWeakPtr(m_consumers.back());
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
		 * @brief Update the shared data wih a custom function
		 * @tparam CustomUpdateFunc A function with signature <i>void(T&)</i>
		 * @param t_update Function that is called with reference to the
		 * shared data <i>t_update(static_cast<T&>(data))</i>
		*/
		template <typename CustomUpdateFunc>
		void customUpdate(CustomUpdateFunc&& t_update)
		{
			ScopedSharableLock lock(m_mtx);
			t_update(static_cast<reference_type>(m_data));
			setNewData();
		}

		/*!
		 * @brief Update the shared data wih a copy-assignment
		 * @param t_data Process-scoped data that is copied to the shared data
		*/
		this_type& operator=(const_reference_type t_data)
		{
			ScopedSharableLock lock(m_mtx);
			m_data = t_data;
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
		bool poll(ConsumerWeakPtr& t_consumer, PollCallback&& t_get)
		{
			if (t_consumer.expired()) return false;
			ScopedSharableLock lock(m_mtx);
			auto tmpShared = t_consumer.lock();
			if (!tmpShared->newData) return false;
			t_get(static_cast<const_reference_type>(m_data));
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
		bool pollCopy(ConsumerWeakPtr& t_consumer, reference_type t_dataOut)
		{
			if (t_consumer.expired()) return false;
			ScopedSharableLock lock(m_mtx);
			auto tmpShared = t_consumer.lock();
			if (!tmpShared->newData) return false;
			t_dataOut = m_data;
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
		bool waitCopy(ConsumerWeakPtr& t_consumer, const std::chrono::duration<_Rep, _Period>& t_timeout, reference_type t_dataOut)
		{
			if (t_consumer.expired()) return false;

			ScopedSharableLock lock(m_mtx);
			auto tmpShared = t_consumer.lock();

			using clock = boost::posix_time::microsec_clock;
			auto usTimeout = usChronoToPosix(t_timeout);

			if (!m_condition.timed_wait(lock, clock::universal_time() + usTimeout, [&] { return tmpShared->newData; }))
			{
				//std::cout << "SharedObject: waitCopy timed out\n";
				return false;
			}
			t_dataOut = m_data;
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
		bool wait(ConsumerWeakPtr& t_consumer, const std::chrono::duration<_Rep, _Period>& t_timeout, RecieveCallback&& t_get)
		{
			if (t_consumer.expired()) return false;

			ScopedSharableLock lock(m_mtx);
			auto tmpShared = t_consumer.lock();

			using clock = boost::posix_time::microsec_clock;
			auto usTimeout = usChronoToPosix(t_timeout);

			if (!m_condition.timed_wait(lock, clock::universal_time() + usTimeout, [&] { return tmpShared->newData; }))
			{
				//std::cout << "SharedObject: wait timed out\n";
				return false;
			}
			t_get(static_cast<const_reference_type>(m_data));
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
			for (auto it = m_consumers.begin(); it != m_consumers.end(); ++it)
				(*it).reset();
		}

	private:
		/*!
		 * @brief Notifies consumers that there is new shared data
		*/
		void setNewData()
		{
			for (auto it = m_consumers.begin(); it != m_consumers.end(); ++it)
				(*it)->newData = true;
			m_condition.notify_all();
		}

		/*!
		 * @brief Condition variable that is used to notify consumers that there is new shared data
		*/
		bip::interprocess_condition_any m_condition;
		/*!
		 * @brief The shared data
		*/
		value_type m_data;
	};
}

#endif // SHMEM_RAW_DATA_HPP