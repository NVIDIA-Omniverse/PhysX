// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// C-boundary tests for the PhysX debug-render API (ovphysx_debug_render_*).
//
// These cover the input-validation hardening + the cached getters added in review
// (MR !7493): the argument checks all run BEFORE the stage/forward, and the getters
// read an ovphysx-side cache, so a bare instance (no attached USD stage) is enough to
// exercise every check here. (The actual draw-buffer population is covered end-to-end
// by the BlokyNext consumer suite, which needs a stepped scene.)

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_types.h"

#include <cmath>
#include <limits>

namespace
{
// Bare instance (no stage). The debug-render arg validation + cached getters do not
// require an attached stage.
struct DebugRenderTest : public ::testing::Test
{
    ovphysx_handle_t h = 0;
    void SetUp() override
    {
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
        ovphysx_result_t r = ovphysx_create_instance(&args, &h);
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
        ASSERT_NE(h, 0u);
    }
    void TearDown() override
    {
        if (h)
            ovphysx_destroy_instance(h);
    }
};

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();
} // namespace

// set_parameter must reject NONE (0) and out-of-range BEFORE forwarding -- omni.physx
// does `visMask |= (1ull << param)` unchecked (param >= 64 is UB) and param 0 collides
// with the eSCALE slot in the enable loop.
TEST_F(DebugRenderTest, SetParameterRejectsNoneAndOutOfRange)
{
    EXPECT_EQ(ovphysx_debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_NONE, true).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_COUNT, true).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_set_parameter(h, 64u, true).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_set_parameter(h, 1000000u, true).status, OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(DebugRenderTest, GetParameterRejectsBadArgs)
{
    bool on = true;
    EXPECT_EQ(ovphysx_debug_render_get_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_NONE, &on).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_get_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_COUNT, &on).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_get_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, nullptr).status,
              OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(DebugRenderTest, SetScaleRejectsNonFiniteAndNegative)
{
    EXPECT_EQ(ovphysx_debug_render_set_scale(h, kNaN).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_set_scale(h, kInf).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_set_scale(h, -kInf).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_set_scale(h, -1.0f).status, OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(DebugRenderTest, GetScaleRejectsNull)
{
    EXPECT_EQ(ovphysx_debug_render_get_scale(h, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);
}

// culling box: NULL, non-finite, or min > max on any axis -> INVALID_ARGUMENT (omni.physx
// would silently drop an invalid box via bounds.isValid() and still return SUCCESS).
TEST_F(DebugRenderTest, SetCullingBoxRejectsBadArgs)
{
    const float mn[3] = { 0.f, 0.f, 0.f };
    const float mx[3] = { 1.f, 1.f, 1.f };
    EXPECT_EQ(ovphysx_debug_render_set_culling_box(h, nullptr, mx).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_set_culling_box(h, mn, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);
    const float nan3[3] = { 0.f, kNaN, 0.f };
    EXPECT_EQ(ovphysx_debug_render_set_culling_box(h, nan3, mx).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_set_culling_box(h, mn, nan3).status, OVPHYSX_API_INVALID_ARGUMENT);
    const float inverted[3] = { 2.f, 2.f, 2.f }; // min > max (mx = 1,1,1) on every axis
    EXPECT_EQ(ovphysx_debug_render_set_culling_box(h, inverted, mx).status, OVPHYSX_API_INVALID_ARGUMENT);
}

// The buffer getters must reject a NULL TYPED out-pointer (previously the shared check
// guarded an internal local, so a NULL caller pointer slipped through as SUCCESS).
TEST_F(DebugRenderTest, GettersRejectNullOutPointers)
{
    const ovphysx_debug_point_t* pts = nullptr;
    const ovphysx_debug_line_t* lns = nullptr;
    const ovphysx_debug_triangle_t* tris = nullptr;
    uint32_t n = 0u;
    EXPECT_EQ(ovphysx_debug_render_get_points(h, nullptr, &n).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_get_points(h, &pts, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_get_lines(h, nullptr, &n).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_get_lines(h, &lns, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_get_triangles(h, nullptr, &n).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_get_triangles(h, &tris, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);
}

// The cached getters round-trip the last value requested through ovphysx (set/get
// pairing). Each assertion sets the state it checks, so it is independent of the
// process-global cache's prior contents / test ordering.
TEST_F(DebugRenderTest, ParameterAndScaleGettersRoundTrip)
{
    bool on = false;

    ovphysx_debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, true);
    ASSERT_EQ(ovphysx_debug_render_get_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, &on).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(on);

    // A parameter that was never set this run reads back off (bit-independent).
    ovphysx_debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_SDF, false);
    ASSERT_EQ(ovphysx_debug_render_get_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_SDF, &on).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(on);
    // ... while BODY_AXES is still on (independent bits).
    ASSERT_EQ(ovphysx_debug_render_get_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, &on).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(on);

    // Toggle BODY_AXES back off.
    ovphysx_debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, false);
    ASSERT_EQ(ovphysx_debug_render_get_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, &on).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(on);

    // Scale round-trips, including the valid boundary value 0.0.
    float scale = -1.0f;
    ovphysx_debug_render_set_scale(h, 2.5f);
    ASSERT_EQ(ovphysx_debug_render_get_scale(h, &scale).status, OVPHYSX_API_SUCCESS);
    EXPECT_FLOAT_EQ(scale, 2.5f);

    ovphysx_debug_render_set_scale(h, 0.0f);
    ASSERT_EQ(ovphysx_debug_render_get_scale(h, &scale).status, OVPHYSX_API_SUCCESS);
    EXPECT_FLOAT_EQ(scale, 0.0f);
}

// Stageless contract: with no USD stage attached, the validate-then-forward calls return
// OVPHYSX_API_ERROR ("no USD stage loaded") AFTER their argument checks pass. This is the
// other half of the doc contract the cached getters exercise: interface/fn-ptr unavailable
// -> no-op SUCCESS, but no stage attached -> ERROR. (set_parameter/set_scale still write
// the ovphysx-side cache before the stage check, which is why the round-trip getters above
// read SUCCESS on the same bare instance.)
TEST_F(DebugRenderTest, ValidCallsReturnErrorWithoutStage)
{
    EXPECT_EQ(ovphysx_debug_render_enable(h, true).status, OVPHYSX_API_ERROR);
    EXPECT_EQ(ovphysx_debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_WORLD_AXES, true).status,
              OVPHYSX_API_ERROR);
    EXPECT_EQ(ovphysx_debug_render_set_scale(h, 1.0f).status, OVPHYSX_API_ERROR);
    const float mn[3] = { 0.f, 0.f, 0.f };
    const float mx[3] = { 1.f, 1.f, 1.f };
    EXPECT_EQ(ovphysx_debug_render_set_culling_box(h, mn, mx).status, OVPHYSX_API_ERROR);
}

// Every visualization parameter must round-trip independently through the cached mask
// (bit 1<<param). ParameterAndScaleGettersRoundTrip only covers two bits, so exercise the
// whole range PARAM_WORLD_AXES (1) .. PARAM_SDF (27 == COUNT-1): enable each and confirm it
// reads back without disturbing the others, then clear each and confirm the not-yet-cleared
// parameters stay set. Catches a bad shift, a bit collision, or an off-by-one at the top of
// the uint32 mask. (set_parameter returns ERROR on this stageless instance but writes the
// cache before the stage check, so the cached getter still round-trips.)
TEST_F(DebugRenderTest, AllVisualizationParameterBitsRoundTripIndependently)
{
    const uint32_t first = OVPHYSX_DEBUG_RENDER_PARAM_WORLD_AXES;
    const uint32_t last = OVPHYSX_DEBUG_RENDER_PARAM_COUNT - 1; // PARAM_SDF (27)

    // Enable every parameter; each must read back set.
    for (uint32_t p = first; p <= last; ++p)
    {
        ovphysx_debug_render_set_parameter(h, (ovphysx_debug_render_parameter_t)p, true);
        bool on = false;
        ASSERT_EQ(ovphysx_debug_render_get_parameter(h, (ovphysx_debug_render_parameter_t)p, &on).status,
                  OVPHYSX_API_SUCCESS);
        EXPECT_TRUE(on) << "parameter " << p << " did not read back as set";
    }

    // Independence: with all bits set, every parameter still reads on (no bit clobbered another).
    for (uint32_t p = first; p <= last; ++p)
    {
        bool on = false;
        ovphysx_debug_render_get_parameter(h, (ovphysx_debug_render_parameter_t)p, &on);
        EXPECT_TRUE(on) << "parameter " << p << " was clobbered while setting another bit";
    }

    // Clear each in ascending order; every not-yet-cleared (higher) parameter must stay set.
    for (uint32_t p = first; p <= last; ++p)
    {
        ovphysx_debug_render_set_parameter(h, (ovphysx_debug_render_parameter_t)p, false);
        bool on = true;
        ovphysx_debug_render_get_parameter(h, (ovphysx_debug_render_parameter_t)p, &on);
        EXPECT_FALSE(on) << "parameter " << p << " did not clear";
        for (uint32_t q = p + 1; q <= last; ++q)
        {
            bool other = false;
            ovphysx_debug_render_get_parameter(h, (ovphysx_debug_render_parameter_t)q, &other);
            EXPECT_TRUE(other) << "clearing parameter " << p << " disturbed parameter " << q;
        }
    }
}
