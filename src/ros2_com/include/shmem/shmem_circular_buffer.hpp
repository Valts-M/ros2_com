/*!
* @file
* @ingroup CoreLibGroup
* @ingroup SharedMemory
* @ingroup DataStructures
*/

#ifndef SHMEM_CIRCULAR_BUFFER_H
#define SHMEM_CIRCULAR_BUFFER_H

#include <string>
#include <exception>

#include <boost/circular_buffer.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/containers/list.hpp>

#include "circular_buffer.hpp"
#include "shmem_base_new.hpp"

namespace zbot::shmem
{
	/*!
	 * @brief First in first out (FIFO) policy for the circular buffer
	*/
	struct PolicyFifo {};
	/*!
	 * @brief Last in first out (LIFO) policy for the circular buffer
	*/
	struct PolicyLifo {};

	/*!
	 * @brief Circular buffer consumer reference for indexing circular data
	 * @details Uses boost::circular_buffer for storing indices of the shared data. 
	*/
	template <class StorageType>
	struct ConsumerCBReference : public ConsumerBase<StorageType>
	{
		using index_type = uint64_t;
		using IndexAllocator = CustomAllocator<index_type, StorageType>;
		using CircularBufferIndices = boost::circular_buffer<index_type, IndexAllocator>;

		/*!
		 * @brief Creates consumer reference with the name \p t_consumerName
		*/
		ConsumerCBReference(ShmemString<StorageType>&& t_consumerName, size_t t_bufferSize, const VoidAllocator<StorageType>& t_allocator) 
			: ConsumerBase<StorageType>(boost::forward<ShmemString<StorageType>&&>(t_consumerName)),
			references(CircularBufferIndices(t_bufferSize, static_cast<const IndexAllocator&>(t_allocator)))
		{}
		/*!
		 * @brief Circular buffer that stores the indices of the circular buffer data
		*/
		CircularBufferIndices references;
		/*!
		 * @brief Pushes an index to the back of the circular buffer
		*/
		inline void pushBack(const index_type t_pos) { references.push_back(t_pos); }
		/*!
		 * @brief Pushes an index to the front of the circular buffer
		*/
		inline void pushFront(const index_type t_pos) { references.push_front(t_pos); }
		/*!
		 * @brief Returns the index at the back of the buffer
		*/
		inline index_type back() const
		{ 
			checkEmpty();
			return references.back();
		}
		/*!
		 * @brief Returns the index at the front of the buffer
		*/
		inline index_type front() const
		{ 
			checkEmpty();
			return references.front();
		}
		/*!
		 * @brief Pops the index at the back of the buffer
		*/
		inline void popBack() 
		{ 
			checkEmpty();
			references.pop_back();
		}
		/*!
		 * @brief Pops the index at the front of the buffer
		*/
		inline void popFront()
		{ 
			checkEmpty();
			references.pop_front();
		}
		/*!
		 * @brief Returns the size of the buffer
		*/
		inline size_t size() const { return references.size(); }
		/*!
		 * @brief Returns the capacity of the buffer
		*/
		inline size_t capacity() const { return references.capacity(); }

		/*!
		 * @brief Checks if the buffer is empty
		 * 
		 * @details If the buffer is empty, an exception is thrown
		*/
		void checkEmpty() const
		{
			if (!references.empty()) return;
			throw std::out_of_range("Consumer Circular buffer indexed get out of range");
		}

	};

	/*!
	 * @brief Shared object container for circular buffer
	 * @tparam T The raw data type for every circular buffer element
	 * @tparam Policy The IO policy type (FIFO or LIFO)
	 * @tparam StorageType Only bip::managed_shared_memory and bip::managed_mapped_file types are supported
	*/
	template <class T, class Policy, class StorageType>
	class SharedCBObject : public SharedObjectBase<T, ConsumerCBReference<StorageType>, StorageType>
	{
	public:
		static_assert(std::is_same<Policy, PolicyFifo>::value || std::is_same<Policy, PolicyLifo>::value,
					  "SharedCBObject: Policy must be of type FIFO or LIFO");
		using this_type = SharedCBObject<T, Policy, StorageType>;

		using base_type = SharedObjectBase<T, ConsumerCBReference<StorageType>, StorageType>;
		using typename base_type::consumer_type;
		using typename base_type::ConsumerWeakPtr;
		using typename base_type::ConsumerSharedPtr;
		
		using Allocator =  CustomAllocator<T, StorageType>;
		using ShmemCircularBuffer = CircularBuffer<T, Allocator>;

		using value_type = typename ShmemCircularBuffer::value_type;
		using reference_type = typename ShmemCircularBuffer::reference_type;
		using const_reference_type = typename ShmemCircularBuffer::const_reference_type;
		using rvalue_reference = typename ShmemCircularBuffer::rvalue_reference;
		using size_type = typename ShmemCircularBuffer::size_type;
		using capacity_type = typename ShmemCircularBuffer::capacity_type;
		using index_type = typename ShmemCircularBuffer::index_type;

		/*!
		 * @brief Constructs an empty container with the specified @p t_allocator allocator
		 * @param t_allocator Allocator for allocating the variables in the shared memory
		 * @param t_bufferCapacity The capacity of the circular buffer
		*/
		SharedCBObject(VoidAllocator<StorageType>& t_allocator, capacity_type t_bufferCapacity) 
			: base_type(t_allocator), m_buffer(t_bufferCapacity, t_allocator),
			m_bufferCapacityInput(t_bufferCapacity ? t_bufferCapacity : 1U)
		{}

		/*!
		 * @brief Adds a consumer reference
		 * @param t_consumerName The name of the consumer
		 * @param t_storage Reference to the shared memory segment where the consumer is created
		 * @param[out] t_out Weak pointer reference to the consumer
		 * @return True if the consumer was successfully created, otherwise returns false
		*/
		bool addConsumer(ShmemString<StorageType>&& t_consumerName, StorageType& t_storage, ConsumerWeakPtr& t_out)
		{
			ScopedSharableLock lock(base_type::m_mtx);
			try
			{
				// construct element
				auto objTemp = t_storage.template construct<consumer_type>(t_consumerName.c_str())(
					boost::forward<ShmemString<StorageType>>(t_consumerName),
					m_buffer.capacity(),
					static_cast<VoidAllocator<StorageType>>(t_storage.get_segment_manager()));

				// make shared ptr
				auto objTempSharedPtr = bip::make_managed_shared_ptr<consumer_type>(objTemp, t_storage);
				// push back
				base_type::m_consumers.push_back(boost::move(objTempSharedPtr));
				t_out = ConsumerWeakPtr(base_type::m_consumers.back());
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
		 * @brief The capacity of the buffer
		*/
		capacity_type capacity() const { return m_buffer.capacity(); }
		/*!
		 * @brief The size of the global buffer
		*/
		size_type size() const { return m_buffer.size(); }
		/*!
		 * @brief The size of the buffer for the specified @p t_consumer consumer
		*/
		size_type size(const ConsumerWeakPtr& t_consumer) const
		{ 
			ScopedSharableLock lock(base_type::m_mtx);
			auto consumerShared = t_consumer.lock();
			return consumerShared->size();
		}
		/*!
		 * @brief Is the buffer empty
		*/
		bool empty() const { return m_buffer.empty(); }
		/*!
		 * @brief Is the buffer full
		*/
		bool full() const { return m_buffer.full(); }

		/*!
		 * @brief Pushes data to the back of the global buffer, pushes index to the consumers
		 * @param t_item The data to push into the buffer
		*/
		void push(const_reference_type t_item)
		{
			ScopedSharableLock lock(base_type::m_mtx);
			m_buffer.pushBack(t_item);
			pushBackConsumers(m_buffer.m_last); // TODO
		}
		/*!
		 * @brief Pops the element according to the IO policy
		*/
		void pop()
		{
			ScopedSharableLock lock(base_type::m_mtx);
			if constexpr (std::is_same<Policy, PolicyFifo>::value)
				m_buffer.popFront();
			else if constexpr (std::is_same<Policy, PolicyLifo>::value)
				m_buffer.popBack();
			else throw std::invalid_argument("SharedCBObject: pop policy ill-defined");
		}
		/*!
		 * @brief Pops the element according to the IO policy from the global buffer and the consumer 
		*/
		void pop(const ConsumerWeakPtr& t_consumer)
		{
			ScopedSharableLock lock(base_type::m_mtx);
			auto consumerShared = t_consumer.lock();
			if constexpr (std::is_same<Policy, PolicyFifo>::value)
			{
				m_buffer.popFront();
				consumerShared->popFront();
			}
			else if constexpr (std::is_same<Policy, PolicyLifo>::value)
			{
				m_buffer.popBack();
				consumerShared->popBack();
			}
			else
				throw std::invalid_argument("SharedCBObject: pop policy ill-defined");
		}

		/*!
		 * @brief Gets data from the global buffer
		 *
		 * @details This does not update consumer indices
		*/
		reference_type get()
		{
			ScopedSharableLock lock(base_type::m_mtx);
			if constexpr (std::is_same<Policy, PolicyFifo>::value)
				return m_buffer.front();
			else if constexpr (std::is_same<Policy, PolicyLifo>::value)
				return m_buffer.back();
			else
				throw std::invalid_argument("SharedCBObject: get policy ill-defined");
		}
		/*!
		 * @brief Gets data from the global buffer
		 *
		 * @details This does not update consumer indices
		*/
		const_reference_type get() const
		{
			ScopedSharableLock lock(base_type::m_mtx);
			if constexpr (std::is_same<Policy, PolicyFifo>::value)
				return m_buffer.front();
			else if constexpr (std::is_same<Policy, PolicyLifo>::value)
				return m_buffer.back();
			else
				throw std::invalid_argument("SharedCBObject: get policy ill-defined");
		}
		
		/*!
		 * @brief Get the data according to the consumer
		 * @param t_consumer Reference to the consumer
		*/
		reference_type get(const ConsumerWeakPtr& t_consumer) 
		{
			ScopedSharableLock lock(base_type::m_mtx);
			auto consumerShared = t_consumer.lock();
			index_type idx = m_buffer.size();
			if constexpr (std::is_same<Policy, PolicyFifo>::value)
				idx = consumerShared->front();
			else if constexpr (std::is_same<Policy, PolicyLifo>::value)
				idx = consumerShared->back();
			else throw std::invalid_argument("SharedCBObject: get policy ill-defined");
			return m_buffer.get(idx);
		}
		/*!
		 * @brief Get the data according to the consumer
		 * @param t_consumer Reference to the consumer
		*/
		const_reference_type get(const ConsumerWeakPtr& t_consumer) const
		{
			ScopedSharableLock lock(base_type::m_mtx);
			auto consumerShared = t_consumer.lock();
			index_type idx = m_buffer.size();
			if constexpr (std::is_same<Policy, PolicyFifo>::value)
				idx = consumerShared->front();
			else if constexpr (std::is_same<Policy, PolicyLifo>::value)
				idx = consumerShared->back();
			else throw std::invalid_argument("SharedCBObject: get policy ill-defined");
			return m_buffer.get(idx);
		}

		/*!
		 * @brief Gets data and pops the data from the global buffer
		 *
		 * @details This does not update consumer indices
		*/
		value_type getAndPop()
		{
			ScopedSharableLock lock(base_type::m_mtx);
			value_type temp;
			if constexpr (std::is_same<Policy, PolicyFifo>::value)
			{
				temp = m_buffer.front();
				m_buffer.popFront();
			}
			else if constexpr (std::is_same<Policy, PolicyLifo>::value)
			{
				temp = m_buffer.back();
				m_buffer.popBack();
			}
			else throw std::invalid_argument("SharedCBObject: get policy ill-defined");
			return temp;
		}
		
		/*!
		 * @brief Gets data and pops the data from the global buffer and the consumer buffer
		*/
		value_type getAndPop(const ConsumerWeakPtr& t_consumer)
		{
			ScopedSharableLock lock(base_type::m_mtx);
			auto consumerShared = t_consumer.lock();
			index_type idx = m_buffer.size();
			if constexpr (std::is_same<Policy, PolicyFifo>::value)
			{
				idx = static_cast<index_type>(consumerShared->front());
				m_buffer.decrement(idx);
				consumerShared->popFront();
			}
			else if constexpr (std::is_same<Policy, PolicyLifo>::value)
			{
				idx = static_cast<index_type>(consumerShared->back());
				m_buffer.decrement(idx);
				consumerShared->popBack();
			}
			else throw std::invalid_argument("SharedCBObject: get policy ill-defined");
			return m_buffer.get(idx);
		}

	private:
		/*!
		 * @brief Buffer capacity that was passed into the constructor
		*/
		capacity_type m_bufferCapacityInput{ 0U };
		/*!
		 * @brief The underlaying circular buffer
		*/
		ShmemCircularBuffer m_buffer;

		/*!
		 * @brief Pushes the index @p t_pos at the back of the consumers' buffers
		 * @param t_item The new index to insert
		*/
		void pushBackConsumers(index_type t_pos)
		{
			if (!base_type::m_consumers.size()) return;
			for (auto c : base_type::m_consumers) c->pushBack(t_pos);
		}
		/*!
		 * @brief Pushes the index @p t_pos at the front of the consumers' buffers
		 * @param t_item The new index to insert
		*/
		void pushFrontConsumers(index_type t_pos)
		{
			if (!base_type::m_consumers.size()) return;
			for (auto c : base_type::m_consumers) c->pushFront(t_pos);
		}
	};
}

#endif // SHMEM_CIRCULAR_BUFFER_H