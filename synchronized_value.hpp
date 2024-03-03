#pragma once

#include <cassert>
#include <chrono>
#include <concepts>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <type_traits>
#include <utility>


namespace detail {

template<typename Mutex>
concept mutex = requires(Mutex mut) {
	mut.lock();
	mut.unlock();
};

template<typename Mutex>
concept timed_mutex = requires(Mutex mut) {
	requires mutex<Mutex>;
	mut.try_lock_for(std::chrono::seconds{});
	mut.try_lock_until(std::chrono::steady_clock::time_point{});
};

template<typename Mutex>
concept shared_mutex = requires(Mutex mut) {
	requires mutex<Mutex>;
	mut.lock_shared();
	mut.unlock_shared();
};

template<typename Mutex>
concept shared_timed_mutex = requires(Mutex mut) {
	requires shared_mutex<Mutex>;
	requires timed_mutex<Mutex>;
	mut.try_lock_shared_for(std::chrono::seconds{});
	mut.try_lock_shared_until(std::chrono::steady_clock::time_point{});
};


template<typename Lock, typename Mutex>
concept lockable = requires {
	requires mutex<Mutex> || shared_mutex<Mutex>;
	requires std::constructible_from<Lock, Mutex&>;
	requires std::constructible_from<Lock, Mutex&, std::try_to_lock_t>;
	requires std::constructible_from<Lock, Mutex&, std::defer_lock_t>;
};

template<typename Lock, typename Mutex>
concept timed_lockable = requires {
	requires timed_mutex<Mutex> || shared_timed_mutex<Mutex>;
	requires std::constructible_from<Lock, Mutex&, std::chrono::seconds>;
	requires std::constructible_from<Lock, Mutex&, std::chrono::steady_clock::time_point>;
};


template<typename LockPolicy>
concept locking_policy = requires {
	typename LockPolicy::mutex_type;
	typename LockPolicy::read_lock_type;
	typename LockPolicy::write_lock_type;

	requires lockable<typename LockPolicy::read_lock_type, typename LockPolicy::mutex_type>;
	requires lockable<typename LockPolicy::write_lock_type, typename LockPolicy::mutex_type>;
};


class dummy_mutex {
public:
	auto lock() -> void {
	}

	auto try_lock() -> bool {
		return true;
	}

	template<typename Rep, typename Period>
	auto try_lock_for([[maybe_unused]] std::chrono::duration<Rep, Period> const& timeout_duration) -> bool {
		return true;
	}

	template<typename Clock, typename Duration>
	auto try_lock_until([[maybe_unused]] std::chrono::time_point<Clock, Duration> const& timeout_time) -> bool {
		return true;
	}

	auto unlock() -> void {
	}
};

} //namespace detail


/**
 * @brief A policy which results in no synchronization of the value
 * @details A regular, unwrapped value should be used instead of a synchronized_value with this policy, but it is
 *          provided for completeness.
 */
struct unsynchronized_lock_policy {
	using mutex_type = detail::dummy_mutex;
	using read_lock_type = std::unique_lock<mutex_type>;
	using write_lock_type = std::unique_lock<mutex_type>;
};

/// A policy which allows only a single reader or writer at a time
template<detail::mutex Mutex = std::timed_mutex>
struct exclusive_lock_policy {
	using mutex_type = Mutex;
	using read_lock_type = std::unique_lock<mutex_type>;
	using write_lock_type = std::unique_lock<mutex_type>;
};

/// A policy which allows a single writer or multiple readers
template<detail::shared_mutex Mutex = std::shared_timed_mutex>
struct shared_lock_policy {
	using mutex_type = Mutex;
	using read_lock_type = std::shared_lock<mutex_type>;
	using write_lock_type = std::unique_lock<mutex_type>;
};


template<typename T, detail::locking_policy LockPolicy = shared_lock_policy<>>
class [[nodiscard]] synchronized_value {
public:
	/// A type with smart-pointer-like semantics which maintains a lock on a value as long as it is in scope.
	template<typename ValueT, typename Lock>
	class [[nodiscard]] locked_value {
	public:
		locked_value() = default;

		template<typename Mutex, typename... ArgsT>
		locked_value(ValueT& value, Mutex& mutex, ArgsT&&... args) :
			lock(mutex, std::forward<ArgsT>(args)...),
			ref(value) {
		}

		[[nodiscard]]
		auto operator*() noexcept -> ValueT& {
			assert(valid());
			return ref;
		}

		[[nodiscard]]
		auto operator*() const noexcept -> const ValueT& {
			assert(valid());
			return ref;
		}

		[[nodiscard]]
		auto operator->() noexcept -> ValueT* {
			assert(valid());
			return std::addressof(ref);
		}

		[[nodiscard]]
		auto operator->() const noexcept -> const ValueT* {
			assert(valid());
			return std::addressof(ref);
		}

		[[nodiscard]]
		explicit operator bool() const noexcept {
			return valid();
		}

		[[nodiscard]]
		auto valid() const noexcept -> bool {
			return bool(lock);
		}

		[[nodiscard]]
		auto get() noexcept -> ValueT* {
			return std::addressof(ref);
		}

		[[nodiscard]]
		auto get() const noexcept -> const ValueT* {
			return std::addressof(ref);
		}

	private:
		Lock lock;
		ValueT& ref;
	};


	using value_type = T;
	using lock_policy = LockPolicy;
	using mutex_type = typename LockPolicy::mutex_type;
	using read_lock_type = typename LockPolicy::read_lock_type;
	using write_lock_type = typename LockPolicy::write_lock_type;
	using read_locked_value = locked_value<T const, typename LockPolicy::read_lock_type>;
	using write_locked_value = locked_value<T, typename LockPolicy::write_lock_type>;


	synchronized_value()
		noexcept(std::is_nothrow_default_constructible_v<T>)
		requires std::default_initializable<T>
		: value() {
	}

	explicit synchronized_value(T const& val)
		noexcept(std::is_nothrow_copy_constructible_v<T>)
		requires std::copy_constructible<T>
		: value(val) {
	}

	explicit synchronized_value(T&& val)
		noexcept(std::is_nothrow_move_constructible_v<T>)
		requires std::move_constructible<T>
		: value(std::move(val)) {
	}

	template<typename... ArgsT> requires std::constructible_from<T, ArgsT...>
	explicit synchronized_value(std::in_place_t, ArgsT&&... args)
		noexcept(std::is_nothrow_constructible_v<T, ArgsT...>)
		: value(std::forward<ArgsT>(args)...) {
	}

	synchronized_value(synchronized_value const&) = delete;
	synchronized_value(synchronized_value&&) = delete;

	~synchronized_value() = default;

	auto operator=(synchronized_value const&) -> synchronized_value& = delete;
	auto operator=(synchronized_value&&) -> synchronized_value& = delete;

	template<typename U = T> requires std::is_assignable_v<T&, U>
	auto operator=(U&& val) -> synchronized_value& {
		set(std::forward<U>(val));
		return *this;
	}


	[[nodiscard]]
	auto unsafe_access() -> T& {
		return value;
	}

	[[nodiscard]]
	auto unsafe_access() const -> T const& {
		return value;
	}


	[[nodiscard]]
	auto get() const -> T requires std::copyable<T> {
		auto lock = read_lock_type{mutex};
		return value;
	}

	template<typename U = T> requires std::is_assignable_v<T&, U>
	auto set(U&& val) -> void {
		auto lock = write_lock_type{mutex};
		value = std::forward<U>(val);
	}

	[[nodiscard]]
	auto lock_read() const -> read_locked_value {
		return read_locked_value{value, mutex};
	}

	[[nodiscard]]
	auto lock_write() -> write_locked_value {
		return write_locked_value{value, mutex};
	}


	[[nodiscard]]
	auto try_get() const -> std::optional<T> requires std::copyable<T> {
		if (auto lock = read_lock_type{mutex, std::try_to_lock}) {
			return value;
		}
		return std::nullopt;
	}

	template<typename U = T> requires std::is_assignable_v<T&, U>
	auto try_set(U&& val) -> bool {
		auto lock = write_lock_type{mutex, std::try_to_lock};
		if (lock) {
			value = std::forward<U>(val);
		}
		return lock.owns_lock();
	}


	[[nodiscard]]
	auto try_lock_read() const -> read_locked_value {
		return read_locked_value{value, mutex, std::try_to_lock};
	}

	[[nodiscard]]
	auto try_lock_write() -> write_locked_value {
		return write_locked_value{value, mutex, std::try_to_lock};
	}


	template<typename Rep, typename Period>
	requires std::copyable<T> && detail::timed_lockable<read_lock_type, mutex_type>
	[[nodiscard]]
	auto try_get_for(std::chrono::duration<Rep, Period> const& duration) const -> std::optional<T> {
		if (auto lock = read_lock_type{mutex, duration}) {
			return value;
		}
		return std::nullopt;
	}

	template<typename U = T, typename Rep, typename Period>
	requires std::is_assignable_v<T&, U> && detail::timed_lockable<write_lock_type, mutex_type>
	auto try_set_for(U&& val, std::chrono::duration<Rep, Period> const& duration) -> bool {
		auto lock = write_lock_type{mutex, duration};
		if (lock) {
			value = std::forward<U>(val);
		}
		return lock.owns_lock();
	}


	template<typename Rep, typename Period>
	requires detail::timed_lockable<read_lock_type, mutex_type>
	[[nodiscard]]
	auto try_lock_read_for(std::chrono::duration<Rep, Period> const& duration) const -> read_locked_value {
		return read_locked_value{value, mutex, duration};
	}

	template<typename Rep, typename Period>
	requires detail::timed_lockable<write_locked_value, mutex_type>
	[[nodiscard]]
	auto try_lock_write_for(std::chrono::duration<Rep, Period> const& duration) const -> write_locked_value {
		return write_locked_value{value, mutex, duration};
	}


	template<typename Clock, typename Duration>
	requires std::copyable<T> && detail::timed_lockable<read_lock_type, mutex_type>
	[[nodiscard]]
	auto try_get_until(std::chrono::time_point<Clock, Duration> const& time) const -> std::optional<T> {
		if (auto lock = read_lock_type{mutex, time}) {
			return value;
		}
		return std::nullopt;
	}

	template<typename U, typename Clock, typename Duration>
	requires std::is_assignable_v<T&, U> && detail::timed_lockable<write_lock_type, mutex_type>
	auto try_set_until(U&& val, std::chrono::time_point<Clock, Duration> const& time) -> bool {
		auto lock = write_lock_type{mutex, time};
		if (lock) {
			value = std::forward<U>(val);
		}
		return lock.owns_lock();
	}


	template<typename Clock, typename Duration>
	requires detail::timed_lockable<read_lock_type, mutex_type>
	[[nodiscard]]
	auto try_lock_read_until(std::chrono::time_point<Clock, Duration> const& time) const -> read_locked_value {
		return read_locked_value{value, mutex, time};
	}

	template<typename Clock, typename Duration>
	requires detail::timed_lockable<write_lock_type, mutex_type>
	[[nodiscard]]
	auto try_lock_write_until(std::chrono::time_point<Clock, Duration> const& time) const -> write_locked_value {
		return write_locked_value{value, mutex, time};
	}


	template<std::invocable<T const&> Function>
	auto with_read_lock(Function&& func) const -> std::invoke_result_t<Function, T const&> {
		auto lock = read_lock_type{mutex};
		return std::forward<Function>(func)(value);
	}

	template<std::invocable<T&> Function>
	auto with_write_lock(Function&& func) -> std::invoke_result_t<Function, T&> {
		auto lock = write_lock_type{mutex};
		return std::forward<Function>(func)(value);
	}

private:
	value_type value;
	mutable mutex_type mutex;
};
