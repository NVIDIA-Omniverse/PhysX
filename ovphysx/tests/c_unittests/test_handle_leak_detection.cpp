// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include <gtest/gtest.h>
#include "AsyncEventManager/AsyncEventManager.h"
#include "global_test_environment.h"

// Test 1: Clean shutdown (no leaks)
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

// Test 2: Leaky shutdown (with leaks)
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

// Test 3: PhysX integration - REMOVED
// This test was specific to the old dm_* registration API which used async_event_handle_t.
// The new API uses ovphysx_op_index_t for operation tracking instead of AsyncEventManager events.
// The AsyncEventManager is tested sufficiently by Tests 1, 2, and 4.

// Test 4: Global shutdown
TEST_F(PhysXTestFixture, HandleLeakDetection_GlobalShutdown) {
    // Record initial event count (may have leftovers from previous tests)
    size_t initial_count = async_get_active_event_count();
    
    // Create some events that won't be cleaned up
    async_event_handle_t event1 = async_create_event();
    async_event_handle_t event2 = async_create_event();
    
    EXPECT_NE(event1, 0);
    EXPECT_NE(event2, 0);
    EXPECT_EQ(async_get_active_event_count(), initial_count + 2);
    
    // Global shutdown should clean up ALL events (including leftovers)
    async_shutdown();
    
    EXPECT_EQ(async_get_active_event_count(), 0);
}
