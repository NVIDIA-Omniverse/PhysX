// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "common/TestHelpers.h"

#include <filesystem>
#include <pxr/base/plug/registry.h>

#include <carb/settings/ISettings.h>
#include <carb/settings/SettingsUtils.h>

#include <omni/physx/IPhysxCooking.h>
#include <private/omni/physx/IPhysxCookingServicePrivate.h>

#include "CookingTestTools.h"

namespace
{
    // Cooking-related plugins only — omni.physx.plugin is NOT loaded.
    // Order matters: dependencies must be loaded before dependents.
    const std::vector<const char*> kCookingPlugins = {
        // Carbonite infrastructure plugins (found in CARB_SDK_SEARCH_PATH)
        "carb.tasking.plugin",
        // RTX plugins (found in RTX_PLUGINS_SEARCH_PATH)
        "carb.stats.plugin",
        // Ujitso dependencies (carb.datasource-file from CARB_SDK, datastore+blobkey from RTX)
        "carb.datasource-file.plugin",
        "carb.datastore.plugin",
        "omni.blobkey.plugin",
        // Ujitso plugins (found in RTX_UJITSO_AGENT_SEARCH_PATH / RTX_UJITSO_DEFAULT_SEARCH_PATH)
        "carb.ujitsoagent.plugin",
        "carb.ujitso.default.plugin",
        // TBB global control (found in RTX_USD_SEARCH_PATH, required by omni.fabric)
        "omni.tbb.globalcontrol.plugin",
        // Fabric plugin (found in RTX_FABRIC_SEARCH_PATH)
        "omni.fabric.plugin",
        // USDRT plugins (found in RTX_SCENEGRAPH_SEARCH_PATH / RTX_USDRT_SEARCH_PATH)
        // These are hybrid Carbonite+ONI plugins: omniCarbStartup registers both sides.
        "usdrt.scenegraph.plugin",
        "usdrt.hierarchy.plugin",
        "usdrt.population.plugin",
        // Our built plugins (found in "." relative to executable)
        "omni.convexdecomposition.plugin",
        "omni.usdphysics.plugin",
        "omni.physx.gpu.plugin",
        "omni.physx.foundation.plugin",
        "omni.physx.cooking.plugin",
    };
}

static CookingTest* gCookingTests = nullptr;

CookingTest* CookingTest::getCookingTests()
{
    if (!gCookingTests)
    {
        gCookingTests = new CookingTest();
    }

    return gCookingTests;
}

CookingTest::CookingTest()
{
    mApp = new carb::AppScoped();
    mApp->startupEmpty();

    // Register physxSchema and physxSchemaAddition with the USD plug registry.
    // Without this, USD schema types are unknown and schema-based APIs fail silently.
    // On Windows the path is relative to the exe dir (portable across CI agents);
    // resolve to absolute so USD's PlugRegistry can find the plugInfo.json files.
#if defined(USD_EXT_PHYSICS_PLUGIN_PATH)
    {
        std::filesystem::path schemaPluginDir =
            std::filesystem::absolute(std::filesystem::path(USD_EXT_PHYSICS_PLUGIN_PATH));
        std::string schemaPluginDirStr = schemaPluginDir.string();
        PXR_NS::PlugRegistry& plugRegistry = PXR_NS::PlugRegistry::GetInstance();
        plugRegistry.RegisterPlugins(schemaPluginDirStr + "/PhysxSchema/resources");
        plugRegistry.RegisterPlugins(schemaPluginDirStr + "/PhysxSchemaAddition/resources");
    }
#endif

    carb::FrameworkScoped& framework = *mApp;

    // Load cooking plugins directly via Carbonite (no Kit extension manager)
    framework.loadPlugins(kCookingPlugins);
}

void CookingTest::release()
{
    delete gCookingTests;
    gCookingTests = nullptr;
}

CookingTest::~CookingTest()
{
    delete mApp;
    mApp = nullptr;
}

omni::physx::IPhysxCooking* CookingTest::acquireCookingInterface()
{
    carb::FrameworkScoped& framework = *mApp;
    return framework->acquireInterface<omni::physx::IPhysxCooking>();
}

omni::physx::IPhysxCookingServicePrivate* CookingTest::acquireCookingServicePrivateInterface()
{
    carb::FrameworkScoped& framework = *mApp;
    return framework->acquireInterface<omni::physx::IPhysxCookingServicePrivate>();
}

omni::physx::IPhysxCookingService* CookingTest::acquireCookingServiceInterface()
{
    carb::FrameworkScoped& framework = *mApp;
    return framework->acquireInterface<omni::physx::IPhysxCookingService>();
}
