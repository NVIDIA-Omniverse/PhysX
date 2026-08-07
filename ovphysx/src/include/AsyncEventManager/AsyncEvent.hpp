// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//


#ifndef ASYNC_EVENT_HPP
#define ASYNC_EVENT_HPP

#include <chrono>
#include <thread>
#include "ovphysx/ovphysx_export.h"
#include "AsyncEventManager/AsyncEventManager.hpp"

namespace omni
{
namespace sdk
{
namespace async
{

/**
 * @brief RAII wrapper for async event handles
 * 
 * Automatically manages the lifecycle of async event handles, ensuring
 * cleanup happens even in the presence of exceptions.
 * 
 * Event objects are returned from asynchronous operations and provide
 * methods to poll for completion or wait with a timeout.
 * 
 * @par Usage Example:
 * @code
 * // Async pattern - poll manually
 * auto event = sdk.loadUsd("scene.usda");
 * while (event.poll() == ASYNC_STATUS_PENDING) {
 *     // Do other work...
 *     std::this_thread::sleep_for(std::chrono::milliseconds(10));
 * }
 * 
 * // Blocking pattern - wait with timeout
 * auto event = sdk.step(0.016f);
 * if (event.waitFor(std::chrono::seconds(5))) {
 *     std::cout << "Step completed successfully\n";
 * } else {
 *     std::cout << "Step failed or timed out\n";
 * }
 * 
 * // Automatic cleanup when event goes out of scope
 * @endcode
 * 
 * @note Move-only type - events represent unique resources
 * 
 * @threadsafety poll() and waitFor() can be called from any thread.
 *               However, each Event should only be accessed by one thread at a time.
 *               The underlying async operation may be thread-safe depending on the
 *               operation type (see specific operation documentation).
 */
class OVPHYSX_API Event
{
public:
    /**
     * @brief Construct an Event from a raw handle
     * @param handle Raw async event handle (takes ownership)
     */
    explicit Event(async_event_handle_t handle) : m_handle(handle) {}
    
    /**
     * @brief Destructor - automatically cleans up the event
     */
    ~Event() noexcept
    {
        if (m_handle != 0) {
            AsyncEventManager::cleanup_event(m_handle);
        }
    }
    
    // Delete copy operations - events are unique resources
    Event(const Event&) = delete;
    Event& operator=(const Event&) = delete;
    
    /**
     * @brief Move constructor
     * @param other Event to move from (will be left in valid but empty state)
     */
    Event(Event&& other) noexcept : m_handle(other.m_handle)
    {
        // Clear other's handle to prevent double-cleanup.
        other.m_handle = 0;
    }
    
    /**
     * @brief Move assignment operator
     * @param other Event to move from
     * @return Reference to this
     */
    Event& operator=(Event&& other) noexcept
    {
        if (this != &other) {
            // Clean up our current resource before taking ownership of other's
            if (m_handle != 0) {
                AsyncEventManager::cleanup_event(m_handle);
            }
            
            // Clear other's handle to prevent double-cleanup.
            m_handle = other.m_handle;
            other.m_handle = 0;
        }
        return *this;
    }
    
    /**
     * @brief Poll the event for completion status
     * @return Current status (PENDING, COMPLETED, or FAILED)
     */
    async_status_t poll() const noexcept
    {
        return AsyncEventManager::poll_event(m_handle);
    }
    
    /**
     * @brief Wait for event completion with timeout
     * @param timeout Maximum time to wait
     * @param pollInterval Time between polls (default: 1ms)
     * @return true if completed successfully, false if failed or timed out
     * 
     * @note This is a blocking operation
     */
    bool waitFor(std::chrono::milliseconds timeout, 
                 std::chrono::milliseconds pollInterval = std::chrono::milliseconds(1)) const
    {
        auto start = std::chrono::steady_clock::now();
        async_status_t status;
        
        while ((status = poll()) == ASYNC_STATUS_PENDING) {
            if (std::chrono::steady_clock::now() - start > timeout) {
                return false; // Timeout
            }
            std::this_thread::sleep_for(pollInterval);
        }
        
        return status == ASYNC_STATUS_COMPLETED;
    }
    
    /**
     * @brief Get the underlying raw handle
     * @return Raw async event handle
     * @note For advanced usage only - prefer using the Event methods
     */
    async_event_handle_t handle() const noexcept { return m_handle; }
    
    /**
     * @brief Check if this Event owns a valid handle
     * @return true if valid, false if moved-from or default-constructed
     */
    bool valid() const noexcept { return m_handle != 0; }

private:
    async_event_handle_t m_handle;
};

} // namespace async
} // namespace sdk
} // namespace omni

#endif // ASYNC_EVENT_HPP
