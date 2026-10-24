#pragma once
#include <array>
#include <atomic>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <utility>

namespace flyarm_mpc_cpp {
// Exactly one producer and one consumer. A busy consumer cannot spuriously reject
// a record: only all Capacity owned slots occupied is a full queue.
template<class T,std::size_t Capacity> class BoundedRecordQueue {
  static_assert(Capacity>0);
  static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
  static_assert(std::is_nothrow_move_constructible_v<T> && std::is_nothrow_move_assignable_v<T>);
 public:
  bool push(T&& value) noexcept {
    const auto write=write_.load(std::memory_order_relaxed);
    if(write-read_.load(std::memory_order_acquire)>=Capacity)return false;
    slots_[write%Capacity].emplace(std::move(value));
    write_.store(write+1,std::memory_order_release);return true;
  }
  bool pop(T& value) noexcept {
    const auto read=read_.load(std::memory_order_relaxed);
    if(read==write_.load(std::memory_order_acquire))return false;
    value=std::move(*slots_[read%Capacity]);slots_[read%Capacity].reset();
    read_.store(read+1,std::memory_order_release);return true;
  }
 private:
  std::array<std::optional<T>,Capacity> slots_;
  alignas(64) std::atomic<std::uint64_t> read_{};
  alignas(64) std::atomic<std::uint64_t> write_{};
};
}
