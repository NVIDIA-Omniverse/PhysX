// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include "internal/sidecar/ovphysxInternal.h"
#include "internal/sidecar/ovphysxInternalInterop.h"

#include <pxr/usd/sdf/path.h>
#include <omni/physx/IPhysxVisualization.h>

#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

static_assert(OVPHYSX_INTERNAL_INTERFACE_VERSION == 10,
              "sidecar loader ABI version must move with token scope");

namespace
{
omni::physx::IPhysxVisualization* g_visualization = nullptr;

omni::physx::IPhysxVisualization* fakeGetVisualization()
{
    return g_visualization;
}

bool CARB_ABI pluginFalse(const ovx_primpath_t*, uint32_t)
{
    return false;
}

bool CARB_ABI pluginThrow(const ovx_primpath_t*, uint32_t)
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
} // namespace

TEST(SidecarAbi, VersionAndLoaderSourceMoveTogether)
{
    EXPECT_EQ(ovphysx_plugin_version(), 10u);

    std::ifstream header("src/include/internal/sidecar/ovphysxInternal.h");
    std::ifstream loader("src/ovphysx/ovphysxSidecarLoader.cpp");
    ASSERT_TRUE(header);
    ASSERT_TRUE(loader);
    const std::string headerText((std::istreambuf_iterator<char>(header)), {});
    const std::string loaderText((std::istreambuf_iterator<char>(loader)), {});
    EXPECT_NE(headerText.find("9 -> 10: added the checked tokenized visualization-scope sidecar export"),
              std::string::npos);
    EXPECT_NE(headerText.find("#define OVPHYSX_INTERNAL_INTERFACE_VERSION 10"), std::string::npos);
    const size_t versionCheck = loaderText.find("if (version != OVPHYSX_INTERNAL_INTERFACE_VERSION)");
    const size_t tokenResolve = loaderText.find("ovphysx_internal_set_visualization_scope_tokens");
    ASSERT_NE(versionCheck, std::string::npos);
    ASSERT_NE(tokenResolve, std::string::npos);
    EXPECT_LT(versionCheck, tokenResolve);
    EXPECT_EQ(loaderText.find("ovphysx_internal_intern_paths"), std::string::npos);
}

TEST(SidecarTokenScope, MissingInjectedInterfaceFails)
{
    AccessorGuard guard;
    const ovx_primpath_t token = 1u;
    EXPECT_FALSE(ovphysx_internal_set_visualization_scope_tokens(&token, 1u));
}

TEST(SidecarTokenScope, NullAppendedSlotFails)
{
    AccessorGuard guard;
    omni::physx::IPhysxVisualization visualization{};
    g_visualization = &visualization;
    ovphysx_internal_set_physx_runtime_accessors(nullptr, &fakeGetVisualization);
    const ovx_primpath_t token = 1u;
    EXPECT_FALSE(ovphysx_internal_set_visualization_scope_tokens(&token, 1u));
}

TEST(SidecarTokenScope, PluginFalsePropagates)
{
    AccessorGuard guard;
    omni::physx::IPhysxVisualization visualization{};
    visualization.setVisualizationScopeTokens = &pluginFalse;
    g_visualization = &visualization;
    ovphysx_internal_set_physx_runtime_accessors(nullptr, &fakeGetVisualization);
    const ovx_primpath_t token = 1u;
    EXPECT_FALSE(ovphysx_internal_set_visualization_scope_tokens(&token, 1u));
}

TEST(SidecarTokenScope, ThrowingPluginFails)
{
    AccessorGuard guard;
    omni::physx::IPhysxVisualization visualization{};
    visualization.setVisualizationScopeTokens = &pluginThrow;
    g_visualization = &visualization;
    ovphysx_internal_set_physx_runtime_accessors(nullptr, &fakeGetVisualization);
    const ovx_primpath_t token = 1u;
    EXPECT_FALSE(ovphysx_internal_set_visualization_scope_tokens(&token, 1u));
}
