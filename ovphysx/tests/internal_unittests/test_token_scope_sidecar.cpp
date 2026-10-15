// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "internal/sidecar/ovphysxInternal.h"
#include "internal/sidecar/ovphysxInternalInterop.h"

#include <omni/physx/IPhysxVisualization.h>
#include <omni/physx/IOvxPhysicsRead.h>
#include <omni/physx/IOvxPhysicsWrite.h>

#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

static_assert(OVPHYSX_INTERNAL_INTERFACE_VERSION == 17,
              "sidecar loader ABI version must move with the sidecar export set");

namespace
{
omni::physx::IPhysxVisualization* g_visualization = nullptr;

omni::physx::IPhysxVisualization* fakeGetVisualization()
{
    return g_visualization;
}

bool CARB_ABI pluginFalse(omni::physics::AttachHandle, const ovx_primpath_t*, uint32_t)
{
    return false;
}

bool CARB_ABI pluginThrow(omni::physics::AttachHandle, const ovx_primpath_t*, uint32_t)
{
    throw std::runtime_error("plugin failure");
}

struct AccessorGuard
{
    AccessorGuard()
    {
        g_visualization = nullptr;
        ovphysx_internal_set_physx_runtime_accessors(nullptr, nullptr);
    }

    ~AccessorGuard()
    {
        ovphysx_internal_set_physx_runtime_accessors(nullptr, nullptr);
        g_visualization = nullptr;
    }
};

constexpr uint64_t kReadHandle = 17;
constexpr uint64_t kWriteHandle = 18;
size_t g_readReleaseCalls = 0;
size_t g_writeReleaseCalls = 0;

omni::physx::OvxReadHandle fakeRead(uint64_t, const ovx_string_or_token_t*, size_t,
                                 omni::physx::OvxReadResult* summary)
{
    static const omni::physx::OvxReadAttributeResult attribute{101, 1, 0};
    if (summary)
        *summary = {1, 1, &attribute};
    return kReadHandle;
}

omni::physx::OvxReadStatus fakeFetchRead(uint64_t, ovstage_read_group_t* group)
{
    *group = {};
    group->read_group_id = 23;
    return omni::physx::kOvxReadStatusOk;
}

bool fakeReleaseRead(uint64_t)
{
    ++g_readReleaseCalls;
    return true;
}

omni::physx::OvxWriteHandle fakeWrite(uint64_t, const ovx_string_or_token_t*, size_t,
                                   omni::physx::OvxWriteResult* summary)
{
    static const omni::physx::OvxWriteAttributeResult attribute{102, 1};
    if (summary)
        *summary = {1, 1, &attribute};
    return kWriteHandle;
}

bool fakeReleaseWrite(uint64_t)
{
    ++g_writeReleaseCalls;
    return true;
}

class SidecarSessionIdentity : public ::testing::Test
{
protected:
    void SetUp() override
    {
        g_readReleaseCalls = 0;
        g_writeReleaseCalls = 0;
        ovphysx_internal_set_ovx_read_accessors(nullptr, nullptr, reinterpret_cast<void*>(&fakeRead),
            reinterpret_cast<void*>(&fakeFetchRead), nullptr, reinterpret_cast<void*>(&fakeReleaseRead), nullptr);
        ovphysx_internal_set_ovx_write_accessors(reinterpret_cast<void*>(&fakeWrite), nullptr, nullptr,
            reinterpret_cast<void*>(&fakeReleaseWrite));
    }

    void TearDown() override
    {
        // Clear the fixed mock identities even after an assertion aborts a test.
        ovphysx_internal_release_read(kReadHandle);
        ovphysx_internal_release_write(kWriteHandle);
        ovphysx_internal_set_ovx_read_accessors(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
        ovphysx_internal_set_ovx_write_accessors(nullptr, nullptr, nullptr, nullptr);
    }
};
} // namespace

TEST(SidecarAbi, VersionAndLoaderSourceMoveTogether)
{
    EXPECT_EQ(ovphysx_plugin_version(), 17u);

    std::ifstream header("src/include/internal/sidecar/ovphysxInternal.h");
    std::ifstream loader("src/ovphysx/ovphysxSidecarLoader.cpp");
    ASSERT_TRUE(header);
    ASSERT_TRUE(loader);
    const std::string headerText((std::istreambuf_iterator<char>(header)), {});
    const std::string loaderText((std::istreambuf_iterator<char>(loader)), {});
    EXPECT_NE(headerText.find("10 -> 11: removed the USD-linked SdfPath encoder and UsdUtilsStageCache erase exports"),
              std::string::npos);
    EXPECT_NE(headerText.find("11 -> 12: replaced path-based articulation FK with query-selected FK"),
              std::string::npos);
    EXPECT_NE(headerText.find("13 -> 14: debug-render apply and copy sidecar exports."), std::string::npos);
    EXPECT_NE(headerText.find("14 -> 15: added the contact query export and its owner-side accessor setter"),
              std::string::npos);
    EXPECT_NE(headerText.find("15 -> 16: session creation summaries and multi-attribute write ABI"),
              std::string::npos);
    EXPECT_NE(headerText.find("16 -> 17: object-change subscribe takes a lifetime_keep so unsubscribe can"),
              std::string::npos);
    EXPECT_NE(headerText.find("#define OVPHYSX_INTERNAL_INTERFACE_VERSION 17"), std::string::npos);
    const size_t versionCheck = loaderText.find("if (version != OVPHYSX_INTERNAL_INTERFACE_VERSION)");
    const size_t tokenResolve = loaderText.find("ovphysx_internal_set_visualization_scope_tokens");
    ASSERT_NE(versionCheck, std::string::npos);
    ASSERT_NE(tokenResolve, std::string::npos);
    EXPECT_LT(versionCheck, tokenResolve);
    EXPECT_EQ(loaderText.find("ovphysx_internal_intern_paths"), std::string::npos);
    // The sidecar no longer links OpenUSD, so the loader must not resolve the
    // removed USD-backed exports.
    EXPECT_EQ(loaderText.find("ovphysx_encode_sdf_path"), std::string::npos);
    EXPECT_EQ(loaderText.find("ovphysx_close_usd_stage"), std::string::npos);
}

TEST(SidecarTokenScope, MissingInjectedInterfaceFails)
{
    AccessorGuard guard;
    const ovx_primpath_t token = 1u;
    EXPECT_FALSE(ovphysx_internal_set_visualization_scope_tokens(1u, &token, 1u));
}

TEST(SidecarTokenScope, NullAppendedSlotFails)
{
    AccessorGuard guard;
    omni::physx::IPhysxVisualization visualization{};
    g_visualization = &visualization;
    ovphysx_internal_set_physx_runtime_accessors(nullptr, &fakeGetVisualization);
    const ovx_primpath_t token = 1u;
    EXPECT_FALSE(ovphysx_internal_set_visualization_scope_tokens(1u, &token, 1u));
}

TEST(SidecarTokenScope, PluginFalsePropagates)
{
    AccessorGuard guard;
    omni::physx::IPhysxVisualization visualization{};
    visualization.setVisualizationScopeTokens = &pluginFalse;
    g_visualization = &visualization;
    ovphysx_internal_set_physx_runtime_accessors(nullptr, &fakeGetVisualization);
    const ovx_primpath_t token = 1u;
    EXPECT_FALSE(ovphysx_internal_set_visualization_scope_tokens(1u, &token, 1u));
}

TEST(SidecarTokenScope, ThrowingPluginFails)
{
    AccessorGuard guard;
    omni::physx::IPhysxVisualization visualization{};
    visualization.setVisualizationScopeTokens = &pluginThrow;
    g_visualization = &visualization;
    ovphysx_internal_set_physx_runtime_accessors(nullptr, &fakeGetVisualization);
    const ovx_primpath_t token = 1u;
    EXPECT_FALSE(ovphysx_internal_set_visualization_scope_tokens(1u, &token, 1u));
}

// @implements REQ-CAPI-SESSION-001
// @covers AC-2 AC-4
TEST_F(SidecarSessionIdentity, DuplicateReadPreservesExistingSession)
{
    const ovx_string_or_token_t attribute{101, {nullptr, 0}};
    ovphysx_read_result_t original{};
    ASSERT_EQ(ovphysx_internal_read_outputs(1, &attribute, 1, &original), kReadHandle);
    ASSERT_NE(original.attributes, nullptr);
    const ovstage_read_group_t* group = nullptr;
    ASSERT_EQ(ovphysx_internal_fetch_read_next(kReadHandle, &group), 1);
    ASSERT_NE(group, nullptr);

    ovphysx_read_result_t duplicate{};
    EXPECT_EQ(ovphysx_internal_read_outputs(1, &attribute, 1, &duplicate), 0u);
    EXPECT_EQ(duplicate.attributes, nullptr);
    ASSERT_EQ(g_readReleaseCalls, 0u);
    EXPECT_EQ(original.attributes[0].attribute, 101u);
    EXPECT_EQ(original.attributes[0].group_count, 1u);
    EXPECT_EQ(group->read_group_id, 23u);
    EXPECT_EQ(ovphysx_internal_fetch_read_next(kReadHandle, &group), 1);
}

// @implements REQ-CAPI-SESSION-001
// @covers AC-3 AC-4
TEST_F(SidecarSessionIdentity, DuplicateWritePreservesExistingSummary)
{
    const ovx_string_or_token_t attribute{102, {nullptr, 0}};
    ovphysx_write_result_t original{};
    ASSERT_EQ(ovphysx_internal_write_attribute(1, &attribute, 1, &original), kWriteHandle);
    ASSERT_NE(original.attributes, nullptr);

    ovphysx_write_result_t duplicate{};
    EXPECT_EQ(ovphysx_internal_write_attribute(1, &attribute, 1, &duplicate), 0u);
    EXPECT_EQ(duplicate.attributes, nullptr);
    ASSERT_EQ(g_writeReleaseCalls, 0u);
    EXPECT_EQ(original.attributes[0].attribute, 102u);
    EXPECT_EQ(original.attributes[0].group_count, 1u);
}
