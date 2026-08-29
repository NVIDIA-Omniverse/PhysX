// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>
#include <stdint.h>
#include <stdbool.h>
#include <thread>
#include <chrono>
#include <vector>
#include <atomic>
#include "AsyncEventManager/AsyncEventManager.h"

// Creates an event that auto-completes after delay_ms on a background thread.
static async_event_handle_t async_create_event_with_delay(uint64_t delay_ms) {
    uint64_t event_handle = async_create_event();
    if (event_handle > 0 && delay_ms > 0) {
        std::thread([event_handle, delay_ms]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
            async_complete_event(event_handle, true, nullptr);
        }).detach();
    }
    return event_handle;
}

TEST(AsyncEventManagerC, BasicEventLifecycle) {
    async_cleanup_all_events();

    uint64_t handle = async_create_event();
    EXPECT_GT(handle, 0);
    EXPECT_TRUE(async_is_valid_event(handle));

    EXPECT_EQ(async_poll_event(handle), ASYNC_STATUS_PENDING);

    async_complete_event(handle, true, NULL);
    EXPECT_EQ(async_poll_event(handle), ASYNC_STATUS_COMPLETED);

    async_cleanup_event(handle);
    EXPECT_FALSE(async_is_valid_event(handle));
}

TEST(AsyncEventManagerC, EventWithDelay) {
    async_cleanup_all_events();

    uint64_t handle = async_create_event_with_delay(50);
    EXPECT_GT(handle, 0);

    EXPECT_EQ(async_poll_event(handle), ASYNC_STATUS_PENDING);

    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    EXPECT_EQ(async_poll_event(handle), ASYNC_STATUS_COMPLETED);

    async_cleanup_event(handle);
}

TEST(AsyncEventManagerC, EventFailure) {
    async_cleanup_all_events();

    uint64_t handle = async_create_event();
    EXPECT_EQ(async_poll_event(handle), ASYNC_STATUS_PENDING);

    async_complete_event(handle, false, "Test error");
    EXPECT_EQ(async_poll_event(handle), ASYNC_STATUS_FAILED);

    async_cleanup_event(handle);
}

TEST(AsyncEventManagerC, InvalidEventHandling) {
    async_cleanup_all_events();

    EXPECT_EQ(async_poll_event(99999), ASYNC_STATUS_FAILED);
    EXPECT_FALSE(async_is_valid_event(99999));

    // Cleanup of an invalid handle must not crash
    async_cleanup_event(99999);
}

TEST(AsyncEventManagerC, ConcurrentAccess) {
    async_cleanup_all_events();

    const int num_threads = 5;
    const int events_per_thread = 100;
    std::vector<std::thread> threads;
    std::vector<int> success_counts(num_threads, 0);

    for (int i = 0; i < num_threads; ++i) {
        threads.emplace_back([&success_counts, i, events_per_thread]() {
            for (int j = 0; j < events_per_thread; ++j) {
                uint64_t handle = async_create_event();
                if (handle > 0 && async_is_valid_event(handle)) {
                    async_complete_event(handle, true, NULL);
                    if (async_poll_event(handle) == ASYNC_STATUS_COMPLETED) {
                        success_counts[i]++;
                    }
                    async_cleanup_event(handle);
                }
            }
        });
    }

    for (auto& thread : threads) {
        thread.join();
    }

    int total_success = 0;
    for (int i = 0; i < num_threads; ++i) {
        total_success += success_counts[i];
    }

    EXPECT_EQ(total_success, num_threads * events_per_thread);
    EXPECT_EQ(async_get_active_event_count(), 0);
}

TEST(AsyncEventManagerC, EventCleanup) {
    async_cleanup_all_events();

    const int num_events = 100;
    uint64_t handles[num_events];

    for (int i = 0; i < num_events; ++i) {
        handles[i] = async_create_event();
    }

    EXPECT_EQ(async_get_active_event_count(), num_events);

    for (int i = 0; i < num_events / 2; ++i) {
        async_cleanup_event(handles[i]);
    }

    EXPECT_EQ(async_get_active_event_count(), num_events / 2);

    async_cleanup_all_events();
    EXPECT_EQ(async_get_active_event_count(), 0);
}

//==============================================================================
// C++ Interface Tests (using std::thread, std::vector, etc.)
//==============================================================================

TEST(AsyncEventManager, BasicEventLifecycle) {
    async_cleanup_all_events();

    auto handle = async_create_event();
    EXPECT_GT(handle, 0);
    EXPECT_TRUE(async_is_valid_event(handle));

    EXPECT_EQ(async_poll_event(handle), ASYNC_STATUS_PENDING);

    async_complete_event(handle, true, nullptr);
    EXPECT_EQ(async_poll_event(handle), ASYNC_STATUS_COMPLETED);

    async_cleanup_event(handle);
    EXPECT_FALSE(async_is_valid_event(handle));
}

TEST(AsyncEventManager, EventWithDelay) {
    async_cleanup_all_events();

    auto handle = async_create_event_with_delay(50);
    EXPECT_GT(handle, 0);

    EXPECT_EQ(async_poll_event(handle), ASYNC_STATUS_PENDING);

    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    EXPECT_EQ(async_poll_event(handle), ASYNC_STATUS_COMPLETED);

    async_cleanup_event(handle);
}

TEST(AsyncEventManager, EventFailure) {
    async_cleanup_all_events();

    auto handle = async_create_event();
    EXPECT_EQ(async_poll_event(handle), ASYNC_STATUS_PENDING);

    async_complete_event(handle, false, "Test error");
    EXPECT_EQ(async_poll_event(handle), ASYNC_STATUS_FAILED);

    async_cleanup_event(handle);
}

TEST(AsyncEventManager, InvalidEventHandling) {
    async_cleanup_all_events();

    EXPECT_EQ(async_poll_event(99999), ASYNC_STATUS_FAILED);
    EXPECT_FALSE(async_is_valid_event(99999));

    // Cleanup of an invalid handle must not crash
    async_cleanup_event(99999);
}

TEST(AsyncEventManager, ConcurrentAccess) {
    async_cleanup_all_events();

    const int num_threads = 5;
    const int events_per_thread = 100;
    std::vector<std::thread> threads;
    std::atomic<int> success_count{0};
    std::atomic<int> error_count{0};

    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back([&]() {
            for (int i = 0; i < events_per_thread; ++i) {
                try {
                    auto handle = async_create_event();
                    if (async_is_valid_event(handle)) {
                        async_complete_event(handle, true, nullptr);
                        if (async_poll_event(handle) == ASYNC_STATUS_COMPLETED) {
                            success_count++;
                        }
                        async_cleanup_event(handle);
                    } else {
                        error_count++;
                    }
                } catch (const std::exception& e) {
                    std::cerr << "[StressTest] Exception in thread: " << e.what() << std::endl;
                    error_count++;
                } catch (...) {
                    std::cerr << "[StressTest] Unknown exception in thread" << std::endl;
                    error_count++;
                }
            }
        });
    }

    for (auto& thread : threads) {
        thread.join();
    }

    EXPECT_EQ(success_count.load(), num_threads * events_per_thread);
    EXPECT_EQ(error_count.load(), 0);
    EXPECT_EQ(async_get_active_event_count(), 0);
}

TEST(AsyncEventManager, EventCleanup) {
    async_cleanup_all_events();

    const int num_events = 100;
    std::vector<uint64_t> handles;

    for (int i = 0; i < num_events; ++i) {
        auto handle = async_create_event();
        handles.push_back(handle);
    }

    EXPECT_EQ(async_get_active_event_count(), num_events);

    for (int i = 0; i < num_events / 2; ++i) {
        async_cleanup_event(handles[i]);
    }

    EXPECT_EQ(async_get_active_event_count(), num_events / 2);

    async_cleanup_all_events();
    EXPECT_EQ(async_get_active_event_count(), 0);
}
