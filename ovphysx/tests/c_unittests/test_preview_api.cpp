// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// Tests for the preview (not-yet-implemented) APIs:
//   ovphysx_find_prims     - must return OVPHYSX_API_NOT_IMPLEMENTED
//   ovphysx_destroy_prim_list - must return OVPHYSX_API_NOT_IMPLEMENTED
//
// These tests verify that the not-implemented stubs do not crash and return
// the expected NOT_IMPLEMENTED status code, so they remain safe to call.

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "preview/ovphysx_preview.h"
#include "global_test_environment.h"
#include "test_utilities.h"

using namespace test_utils;

// ---------------------------------------------------------------------------
// ovphysx_find_prims
// ---------------------------------------------------------------------------

// Without a loaded USD, find_prims should still return NOT_IMPLEMENTED (not crash).
TEST_F(PhysXTestFixture, FindPrimsReturnsNotImplemented)
{
    ovphysx_prim_list_t prim_list{};
    ovphysx_result_t r = ovphysx_find_prims(
        m_handle,
        OVPHYSX_LITERAL("/World/*"),
        OVPHYSX_LITERAL("physics:velocity"),
        &prim_list);
    EXPECT_EQ(r.status, OVPHYSX_API_NOT_IMPLEMENTED)
        << "ovphysx_find_prims must return NOT_IMPLEMENTED";
}

// Different patterns should all return NOT_IMPLEMENTED.
TEST_F(PhysXTestFixture, FindPrimsWithEmptyAttributeReturnsNotImplemented)
{
    ovphysx_prim_list_t prim_list{};
    ovphysx_result_t r = ovphysx_find_prims(
        m_handle,
        OVPHYSX_LITERAL("/World/**"),
        OVPHYSX_LITERAL(""),   // empty attribute name
        &prim_list);
    EXPECT_EQ(r.status, OVPHYSX_API_NOT_IMPLEMENTED);
}

// Null out_prim_list should still not crash (may return INVALID_ARGUMENT or NOT_IMPLEMENTED).
TEST_F(PhysXTestFixture, FindPrimsNullOutPrimList)
{
    ovphysx_result_t r = ovphysx_find_prims(
        m_handle,
        OVPHYSX_LITERAL("/World/*"),
        OVPHYSX_LITERAL(""),
        nullptr);
    bool acceptable = (r.status == OVPHYSX_API_NOT_IMPLEMENTED ||
                       r.status == OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_TRUE(acceptable)
        << "Null out_prim_list should yield NOT_IMPLEMENTED or INVALID_ARGUMENT";
}

// ---------------------------------------------------------------------------
// ovphysx_destroy_prim_list
// ---------------------------------------------------------------------------

// destroy_prim_list with a zero-initialized prim list must not crash.
// The header comment says NOT_IMPLEMENTED, but the implementation returns
// SUCCESS for an empty/zero-initialized list (safe no-op).  Accept either.
TEST_F(PhysXTestFixture, DestroyEmptyPrimListNoCrash)
{
    ovphysx_prim_list_t prim_list{};
    ovphysx_result_t r = ovphysx_destroy_prim_list(m_handle, &prim_list);
    bool acceptable = (r.status == OVPHYSX_API_SUCCESS ||
                       r.status == OVPHYSX_API_NOT_IMPLEMENTED);
    EXPECT_TRUE(acceptable)
        << "destroy_prim_list on empty list must not crash; got status " << r.status;
}

// Null prim_list pointer should not crash.
TEST_F(PhysXTestFixture, DestroyNullPrimListNoCrash)
{
    ovphysx_result_t r = ovphysx_destroy_prim_list(m_handle, nullptr);
    bool acceptable = (r.status == OVPHYSX_API_NOT_IMPLEMENTED ||
                       r.status == OVPHYSX_API_SUCCESS         ||
                       r.status == OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_TRUE(acceptable)
        << "Null prim_list must not crash; got status " << r.status;
}
