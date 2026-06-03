/**
 * @file spsc_ring_buffer.hpp
 * @brief Small bounded SPSC queue helper for RGB-D frame handoff.
 * @author WenSheng Xu
 * @date 2026-06-03
 * @version 0.1
 * @copyright Copyright (c) 2026, WenSheng Xu. All rights reserved.
 */

#pragma once
#ifndef ORB_SLAM3_BRIDGE_SPSC_RING_BUFFER_HPP
#define ORB_SLAM3_BRIDGE_SPSC_RING_BUFFER_HPP

#include <atomic>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace orbslam3_ros {

    template<typename T>
    class SpscRingBuffer {
    public:
        struct PushResult {
            bool pushed{false};
            bool dropped_oldest{false};
        };

        // Construct a bounded single-producer, single-consumer ring buffer.
        explicit SpscRingBuffer(std::size_t capacity)
        : capacity_(capacity),
          slot_count_(capacity + 1),
          slots_(slot_count_) {
            if (capacity_ == 0) {
                throw std::invalid_argument("SpscRingBuffer capacity must be greater than zero");
            }
        }

        SpscRingBuffer(const SpscRingBuffer&) = delete;
        SpscRingBuffer& operator=(const SpscRingBuffer&) = delete;
        SpscRingBuffer(SpscRingBuffer&&) = delete;
        SpscRingBuffer& operator=(SpscRingBuffer&&) = delete;

        // Push a value into the buffer, dropping the oldest item if needed.
        PushResult Push(std::shared_ptr<T> value) {
            PushResult result;

            const std::size_t head = head_.load(std::memory_order_relaxed);
            const std::size_t next_head = Next(head);
            std::size_t tail = tail_.load(std::memory_order_acquire);

            if (next_head == tail) {
                result.dropped_oldest = true;
                tail = Next(tail);
                tail_.store(tail, std::memory_order_release);
            }

            std::atomic_store_explicit(&slots_[head], std::move(value), std::memory_order_release);
            head_.store(next_head, std::memory_order_release);
            result.pushed = true;
            return result;
        }

        template<typename... Args>
        // Construct an item in place and push it into the buffer.
        PushResult Emplace(Args&&... args) {
            return Push(std::make_shared<T>(std::forward<Args>(args)...));
        }

        // Pop the next value from the buffer.
        bool Pop(std::shared_ptr<T>& value) {
            const std::size_t tail = tail_.load(std::memory_order_relaxed);
            if (tail == head_.load(std::memory_order_acquire)) {
                return false;
            }

            value = std::atomic_load_explicit(&slots_[tail], std::memory_order_acquire);
            std::atomic_store_explicit(&slots_[tail], std::shared_ptr<T>{}, std::memory_order_release);
            tail_.store(Next(tail), std::memory_order_release);
            return static_cast<bool>(value);
        }

        // Return the configured capacity.
        [[nodiscard]] std::size_t Capacity() const noexcept {
            return capacity_;
        }

        // Return the current number of buffered items.
        [[nodiscard]] std::size_t Size() const noexcept {
            const std::size_t head = head_.load(std::memory_order_acquire);
            const std::size_t tail = tail_.load(std::memory_order_acquire);
            if (head >= tail) {
                return head - tail;
            }
            return slot_count_ - (tail - head);
        }

        // Return true when the buffer has no items.
        [[nodiscard]] bool Empty() const noexcept {
            return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
        }

        // Remove every buffered item.
        void Clear() {
            std::shared_ptr<T> discarded;
            while (Pop(discarded)) {
            }
        }

    private:
        [[nodiscard]] std::size_t Next(std::size_t index) const noexcept {
            return (index + 1) % slot_count_;
        }

        const std::size_t capacity_;
        const std::size_t slot_count_;
        std::vector<std::shared_ptr<T>> slots_;
        std::atomic<std::size_t> head_{0};
        std::atomic<std::size_t> tail_{0};
    };

}  // namespace orbslam3_ros

#endif  // ORB_SLAM3_BRIDGE_SPSC_RING_BUFFER_HPP
