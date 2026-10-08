// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include "AsyncEventManager/AsyncEventManager.h"
#include "global_test_environment.h"

TEST_F(PhysXTestFixture, HandleLeakDetection_CleanShutdown) {
    async_event_handle_t event1 = async_create_event();
    async_event_handle_t event2 = async_create_event();
    
    EXPECT_NE(event1, 0);
    EXPECT_NE(event2, 0);
    EXPECT_EQ(async_get_active_event_count(), 2);
    
    async_cleanup_event(event1);
    async_cleanup_event(event2);
    
    EXPECT_EQ(async_get_active_event_count(), 0);
    
    async_shutdown();
}

TEST_F(PhysXTestFixture, HandleLeakDetection_LeakyShutdown) {
    EXPECT_EQ(async_get_active_event_count(), 0);
    async_event_handle_t event1 = async_create_event();
    async_event_handle_t event2 = async_create_event();
    async_event_handle_t event3 = async_create_event();
    
    EXPECT_NE(event1, 0);
    EXPECT_NE(event2, 0);
    EXPECT_NE(event3, 0);
    EXPECT_EQ(async_get_active_event_count(), 3);
    
    // Only clean up one event (leak the other two)
    async_cleanup_event(event1);
    
    EXPECT_EQ(async_get_active_event_count(), 2);
    
    async_shutdown();
    
    EXPECT_EQ(async_get_active_event_count(), 0);
}

TEST_F(PhysXTestFixture, HandleLeakDetection_GlobalShutdown) {
    // Previous tests may have left events behind.
    size_t initial_count = async_get_active_event_count();
    
    // These events are deliberately left for the global shutdown.
    async_event_handle_t event1 = async_create_event();
    async_event_handle_t event2 = async_create_event();
    
    EXPECT_NE(event1, 0);
    EXPECT_NE(event2, 0);
    EXPECT_EQ(async_get_active_event_count(), initial_count + 2);
    
    // Global shutdown must clean up all events, including the leftovers.
    async_shutdown();
    
    EXPECT_EQ(async_get_active_event_count(), 0);
}
