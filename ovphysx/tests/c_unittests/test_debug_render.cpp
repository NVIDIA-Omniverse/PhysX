// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// C-boundary tests for the PhysX debug-render API (ovphysx_debug_render_*).
//
// These cover the input-validation hardening + the cached getters added in review:
// the argument checks all run BEFORE the stage/forward, and the getters
// read an ovphysx-side cache, so a bare instance (no attached USD stage) is enough to
// exercise every check here. (The actual draw-buffer population is covered end-to-end
// by the BlokyNext consumer suite, which needs a stepped scene.)

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_types.h"
#include "ovphysxTestHelpers.h"
#include "global_test_environment.h"

#include <atomic>
#include <cmath>
#include <limits>

namespace
{
std::atomic<int> g_scopeCalls{0};
std::atomic<uint32_t> g_lastScopeCount{0};
std::atomic<bool> g_lastScopeWasNull{false};

void recordScopeCall(const ovx_primpath_t* tokens, uint32_t count)
{
    g_scopeCalls.fetch_add(1, std::memory_order_relaxed);
    g_lastScopeCount.store(count, std::memory_order_relaxed);
    g_lastScopeWasNull.store(tokens == nullptr, std::memory_order_relaxed);
}

bool scopeSuccess(const ovx_primpath_t* tokens, uint32_t count)
{
    recordScopeCall(tokens, count);
    return true;
}

bool scopeFalse(const ovx_primpath_t* tokens, uint32_t count)
{
    recordScopeCall(tokens, count);
    return false;
}

ovphysx_result_t failingUserTask(ovphysx_handle_t, ovphysx_op_index_t, void*)
{
    return {OVPHYSX_API_ERROR};
}

struct SidecarSlotsGuard
{
    ovphysx_test_set_viz_scope_tokens_fn previousScope;

    explicit SidecarSlotsGuard(ovphysx_test_set_viz_scope_tokens_fn scope)
        : previousScope(ovphysx_exchange_set_viz_scope_tokens_internal(scope))
    {
        g_scopeCalls.store(0, std::memory_order_relaxed);
        g_lastScopeCount.store(0, std::memory_order_relaxed);
        g_lastScopeWasNull.store(false, std::memory_order_relaxed);
    }

    ~SidecarSlotsGuard()
    {
        ovphysx_exchange_set_viz_scope_tokens_internal(previousScope);
    }
};

struct OvstageAttachmentStateGuard
{
    ovphysx_handle_t handle;
    bool configured;

    explicit OvstageAttachmentStateGuard(ovphysx_handle_t h, int64_t stageId)
        : handle(h)
        , configured(ovphysx_set_ovstage_attachment_state_internal(h, true, stageId))
    {
    }

    ~OvstageAttachmentStateGuard()
    {
        if (configured)
            ovphysx_set_ovstage_attachment_state_internal(handle, false, 0);
    }
};

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
        {
            test_utils::destroy_ovstage_test_attachments(h);
            ovphysx_destroy_instance(h);
        }
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
    EXPECT_EQ(ovphysx_debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_NONE, 1.0f).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_COUNT, 1.0f).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_set_parameter(h, 64u, 1.0f).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_set_parameter(h, 1000000u, 1.0f).status, OVPHYSX_API_INVALID_ARGUMENT);
    // The value must be finite and non-negative.
    EXPECT_EQ(ovphysx_debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, -1.0f).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, kNaN).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, kInf).status,
              OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(DebugRenderTest, GetParameterRejectsBadArgs)
{
    float value = 1.0f;
    EXPECT_EQ(ovphysx_debug_render_get_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_NONE, &value).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_debug_render_get_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_COUNT, &value).status,
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
    float value = -1.0f;

    // A distinct non-default value proves the float path (not a bool round-trip).
    ovphysx_debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, 2.5f);
    ASSERT_EQ(ovphysx_debug_render_get_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, &value).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_FLOAT_EQ(value, 2.5f);

    // A parameter that was never set this run reads back off (entry-independent).
    ovphysx_debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_SDF, 0.0f);
    ASSERT_EQ(ovphysx_debug_render_get_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_SDF, &value).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_FLOAT_EQ(value, 0.0f);
    // ... while BODY_AXES keeps its value (independent entries).
    ASSERT_EQ(ovphysx_debug_render_get_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, &value).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_FLOAT_EQ(value, 2.5f);

    // Back off.
    ovphysx_debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, 0.0f);
    ASSERT_EQ(ovphysx_debug_render_get_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, &value).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_FLOAT_EQ(value, 0.0f);

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
    EXPECT_EQ(ovphysx_debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_WORLD_AXES, 1.0f).status,
              OVPHYSX_API_ERROR);
    EXPECT_EQ(ovphysx_debug_render_set_scale(h, 1.0f).status, OVPHYSX_API_ERROR);
    const float mn[3] = { 0.f, 0.f, 0.f };
    const float mx[3] = { 1.f, 1.f, 1.f };
    EXPECT_EQ(ovphysx_debug_render_set_culling_box(h, mn, mx).status, OVPHYSX_API_ERROR);
}

TEST_F(DebugRenderTest, InvalidHandleAndWaitFailureDoNotForwardTokenScope)
{
    SidecarSlotsGuard slots(&scopeSuccess);
    ovx_primpath_t token = 0x1234u;
    const ovphysx_handle_t invalidHandle = static_cast<ovphysx_handle_t>(~0ull);
    EXPECT_EQ(ovphysx_debug_render_set_scope_tokens(invalidHandle, &token, 1u).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(g_scopeCalls.load(std::memory_order_relaxed), 0);

    ovphysx_user_task_desc_t task{};
    task.run = &failingUserTask;
    const ovphysx_enqueue_result_t queued = ovphysx_add_user_task(h, &task);
    ASSERT_EQ(queued.status, OVPHYSX_API_ERROR);
    ASSERT_NE(queued.op_index, 0u);
    EXPECT_EQ(ovphysx_debug_render_set_scope_tokens(h, &token, 1u).status, OVPHYSX_API_ERROR);
    EXPECT_EQ(g_scopeCalls.load(std::memory_order_relaxed), 0);
}

TEST_F(DebugRenderTest, TokenScopeValidatesBeforeForwardAndRequiresOvstage)
{
    SidecarSlotsGuard slots(&scopeSuccess);
    const ovx_primpath_t valid = 0x1234u;
    const ovx_primpath_t invalid = OVX_INVALID_PRIMPATH;
    EXPECT_EQ(ovphysx_debug_render_set_scope_tokens(h, nullptr, 0u).status, OVPHYSX_API_ERROR);
    EXPECT_EQ(ovphysx_debug_render_set_scope_tokens(h, &valid, 1u).status, OVPHYSX_API_ERROR);
    EXPECT_EQ(ovphysx_debug_render_set_scope_tokens(h, &invalid, 1u).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(g_scopeCalls.load(std::memory_order_relaxed), 0);
}

TEST_F(DebugRenderTest, TokenScopeFailsClosedOnMissingOrFalseSidecar)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(h, "tests/data/minimal_scene.usda"));
    ovx_primpath_t token = 0x1234u;

    {
        SidecarSlotsGuard slots(nullptr);
        EXPECT_EQ(ovphysx_debug_render_set_scope_tokens(h, nullptr, 0u).status, OVPHYSX_API_ERROR);
        EXPECT_EQ(ovphysx_debug_render_set_scope_tokens(h, &token, 1u).status, OVPHYSX_API_ERROR);
    }
    {
        SidecarSlotsGuard slots(&scopeFalse);
        EXPECT_EQ(ovphysx_debug_render_set_scope_tokens(h, nullptr, 0u).status, OVPHYSX_API_ERROR);
        EXPECT_EQ(ovphysx_debug_render_set_scope_tokens(h, &token, 1u).status, OVPHYSX_API_ERROR);
        EXPECT_EQ(g_scopeCalls.load(std::memory_order_relaxed), 2);
    }
}

TEST_F(DebugRenderTest, TokenScopePropagatesSuccessfulSidecarStatus)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(h, "tests/data/minimal_scene.usda"));
    SidecarSlotsGuard slots(&scopeSuccess);
    ovx_primpath_t token = 0x1234u;
    EXPECT_EQ(ovphysx_debug_render_set_scope_tokens(h, &token, 1u).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_debug_render_set_scope_tokens(h, nullptr, 0u).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(g_scopeCalls.load(std::memory_order_relaxed), 2);
}

TEST_F(DebugRenderTest, TokenScopeAcceptsOvstageAttachmentWithZeroUsdStageId)
{
    SidecarSlotsGuard slots(&scopeSuccess);
    OvstageAttachmentStateGuard attachment(h, 0);
    ASSERT_TRUE(attachment.configured);

    const ovx_primpath_t token = 0x1234u;
    EXPECT_EQ(ovphysx_debug_render_set_scope_tokens(h, &token, 1u).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_debug_render_set_scope_tokens(h, nullptr, 0u).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(g_scopeCalls.load(std::memory_order_relaxed), 2);
    EXPECT_EQ(g_lastScopeCount.load(std::memory_order_relaxed), 0u);
    EXPECT_TRUE(g_lastScopeWasNull.load(std::memory_order_relaxed));
}

TEST_F(DebugRenderTest, DebugRenderAcceptsOvstageAttachmentWithZeroUsdStageId)
{
    OvstageAttachmentStateGuard attachment(h, 0);
    ASSERT_TRUE(attachment.configured);

    EXPECT_EQ(ovphysx_debug_render_enable(h, true).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_debug_render_enable(h, false).status, OVPHYSX_API_SUCCESS);
}

TEST_F(DebugRenderTest, DestroyClearsTokenScopeAfterPendingOperationFails)
{
    SidecarSlotsGuard slots(&scopeSuccess);
    ASSERT_TRUE(ovphysx_set_ovstage_attachment_state_internal(h, true, 0));

    const ovx_primpath_t token = 0x1234u;
    ASSERT_EQ(ovphysx_debug_render_set_scope_tokens(h, &token, 1u).status,
              OVPHYSX_API_SUCCESS);

    ovphysx_user_task_desc_t task{};
    task.run = &failingUserTask;
    const ovphysx_enqueue_result_t queued = ovphysx_add_user_task(h, &task);
    ASSERT_EQ(queued.status, OVPHYSX_API_ERROR);
    ASSERT_NE(queued.op_index, 0u);

    EXPECT_EQ(ovphysx_destroy_instance(h).status, OVPHYSX_API_SUCCESS);
    h = 0;
    EXPECT_EQ(g_scopeCalls.load(std::memory_order_relaxed), 2);
    EXPECT_EQ(g_lastScopeCount.load(std::memory_order_relaxed), 0u);
    EXPECT_TRUE(g_lastScopeWasNull.load(std::memory_order_relaxed));
}

// Every visualization parameter must round-trip independently through the cached
// values. ParameterAndScaleGettersRoundTrip only covers two entries, so exercise
// the whole range PARAM_WORLD_AXES (1) .. COUNT-1 with a per-parameter distinct
// value: set each and confirm it reads back without disturbing the others, then
// clear each and confirm the not-yet-cleared parameters keep their values.
// (set_parameter returns ERROR on this stageless instance but writes the cache
// before the stage check, so the cached getter still round-trips.)
TEST_F(DebugRenderTest, AllVisualizationParameterValuesRoundTripIndependently)
{
    const uint32_t first = OVPHYSX_DEBUG_RENDER_PARAM_WORLD_AXES;
    const uint32_t last = OVPHYSX_DEBUG_RENDER_PARAM_COUNT - 1;

    // Set every parameter to a distinct value; each must read back exactly.
    for (uint32_t p = first; p <= last; ++p)
    {
        ovphysx_debug_render_set_parameter(h, (ovphysx_debug_render_parameter_t)p, float(p));
        float value = -1.0f;
        ASSERT_EQ(ovphysx_debug_render_get_parameter(h, (ovphysx_debug_render_parameter_t)p, &value).status,
                  OVPHYSX_API_SUCCESS);
        EXPECT_FLOAT_EQ(value, float(p)) << "parameter " << p << " did not read back";
    }

    // Independence: with all entries set, every parameter still reads its own value.
    for (uint32_t p = first; p <= last; ++p)
    {
        float value = -1.0f;
        ovphysx_debug_render_get_parameter(h, (ovphysx_debug_render_parameter_t)p, &value);
        EXPECT_FLOAT_EQ(value, float(p)) << "parameter " << p << " was clobbered while setting another";
    }

    // Clear each in ascending order; every not-yet-cleared (higher) parameter keeps its value.
    for (uint32_t p = first; p <= last; ++p)
    {
        ovphysx_debug_render_set_parameter(h, (ovphysx_debug_render_parameter_t)p, 0.0f);
        float value = -1.0f;
        ovphysx_debug_render_get_parameter(h, (ovphysx_debug_render_parameter_t)p, &value);
        EXPECT_FLOAT_EQ(value, 0.0f) << "parameter " << p << " did not clear";
        for (uint32_t q = p + 1; q <= last; ++q)
        {
            float other = -1.0f;
            ovphysx_debug_render_get_parameter(h, (ovphysx_debug_render_parameter_t)q, &other);
            EXPECT_FLOAT_EQ(other, float(q)) << "clearing parameter " << p << " disturbed parameter " << q;
        }
    }
}
