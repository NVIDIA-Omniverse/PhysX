// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include <filesystem>
#include <pxr/base/plug/registry.h>

#include <carb/settings/ISettings.h>
#include <carb/settings/SettingsUtils.h>

#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxCooking.h>
#include <private/omni/physx/IPhysxCookingServicePrivate.h>
#include <private/omni/physx/IPhysxTests.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxVehicle.h>
#include <omni/physx/IPhysxSettings.h>
#include <omni/physx/IPhysxSceneQuery.h>
#include <private/omni/physx/IPhysxPrivate.h>
#include <omni/physx/IPhysxReplicator.h>
#include <omni/physx/IPhysxFabric.h>

#include "PhysicsTools.h"

namespace
{
    // All physics plugins needed by the test suite, loaded via Carbonite directly (no Kit extension manager).
    // Order matters: dependencies must be loaded before dependents.
    const std::vector<const char*> kPhysicsPlugins = {
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
        // Cubric adapter (found in RTX_CUBRIC_SEARCH_PATH, required by omni.physx.fabric.plugin)
        "omni.cubric.plugin",
        // GPU compute CUDA backend (found in RTX_GPUCOMPUTE_SEARCH_PATH, required by fabric GPU compute)
        "omni.gpucompute-cuda.plugin",
        // Our built plugins (found in "." relative to executable)
        "omni.convexdecomposition.plugin",
        "omni.usdphysics.plugin",
        "omni.physx.gpu.plugin",
        "omni.physx.foundation.plugin",
        "omni.physx.cooking.plugin",
        "omni.physx.plugin",
    };
}

static PhysicsTest* gPhysicsTests = nullptr;

PhysicsTest* PhysicsTest::getPhysicsTests()
{
    if (!gPhysicsTests)
    {
        gPhysicsTests = new PhysicsTest();
    }

    return gPhysicsTests;
}

PhysicsTest::PhysicsTest()
{
    mApp = new carb::AppScoped();
    mApp->startupEmpty();

    // Threading limits (match Kit CI defaults, OMPE-59303).
    // Without these, TBB and carb.tasking create threads based on hardware_concurrency()
    // which can be very high on CI runners, causing hangs.
    {
        carb::settings::ISettings* settings = mApp->getFramework()->acquireInterface<carb::settings::ISettings>();
        settings->setDefaultInt("/plugins/carb.tasking.plugin/threadCount", 16);
        settings->setDefaultInt("/plugins/omni.tbb.globalcontrol/maxThreadCount", 16);
    }

    // Register physxSchema and physxSchemaAddition with the USD plug registry.
    // Without this, USD schema types (e.g. PhysxArticulationAPI) are unknown and
    // schema-based APIs fail silently. The path is set by CMake from usd_ext_physics.
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
        plugRegistry.RegisterPlugins(schemaPluginDirStr + "/OmniUsdPhysicsDeformableSchema/resources");
    }
#endif

    // Register Newton USD schemas (newton-usd-schemas pip package).
#if defined(NEWTON_SCHEMA_PLUGIN_PATH)
    {
        std::filesystem::path newtonSchemaDir =
            std::filesystem::absolute(std::filesystem::path(NEWTON_SCHEMA_PLUGIN_PATH));
        PXR_NS::PlugRegistry::GetInstance().RegisterPlugins(newtonSchemaDir.string());
    }
#endif

    // Register Kit prim metadata fields (hide_in_stage_window, no_delete) so that
    // production plugin code calling setHideInStageWindow/setNoDelete does not emit
    // USD Coding Errors about unregistered metadata fields. In the full Kit environment
    // these fields are registered by a Kit extension; here we register them explicitly.
#if defined(UNIT_TEST_RESOURCES_PATH)
    PXR_NS::PlugRegistry::GetInstance().RegisterPlugins(UNIT_TEST_RESOURCES_PATH);
#endif

    carb::FrameworkScoped& framework = *mApp;

    // Load physics plugins directly via Carbonite (no Kit extension manager)
    framework.loadPlugins(kPhysicsPlugins);

    omni::physx::IPhysx* iPhysx = framework->acquireInterface<omni::physx::IPhysx>();
    if (iPhysx)
    {
        mEventStreamPtr = iPhysx->getErrorEventStream();
        mSubscriptionPtr = mEventStreamPtr->createSubscriptionToPop(&mErrorListener, 0);

        mErrorListener.setDict(framework->acquireInterface<carb::dictionary::IDictionary>());
    }
}

void PhysicsTest::release()
{
    if (mSubscriptionPtr)
    {
        mSubscriptionPtr->unsubscribe();
        mSubscriptionPtr = nullptr;
    }
    mEventStreamPtr = nullptr;

    delete gPhysicsTests;
    gPhysicsTests = nullptr;
}

PhysicsTest::~PhysicsTest()
{
    delete mApp;
    mApp = nullptr;
}

std::string PhysicsTest::getDataDirectory()
{
    carb::filesystem::IFileSystem* fs = mApp->getFramework()->acquireInterface<carb::filesystem::IFileSystem>();

    return std::string(fs->getAppDirectoryPath()) + "/../../../../ovexts/data/";
}

std::string PhysicsTest::getUnitTestsDataDirectory()
{
    return getDataDirectory() + "usd/tests/Physics/Unit_Tests/";
}

omni::physx::IPhysx* PhysicsTest::acquirePhysxInterface()
{
    carb::FrameworkScoped& framework = *mApp;
    omni::physx::IPhysx* iPhysx = framework->acquireInterface<omni::physx::IPhysx>();    
    return iPhysx;
}

omni::physx::IPhysxCooking* PhysicsTest::acquirePhysxCookingInterface()
{
    carb::FrameworkScoped& framework = *mApp;
    omni::physx::IPhysxCooking* iPhysx = framework->acquireInterface<omni::physx::IPhysxCooking>();
    return iPhysx;
}

omni::physx::IPhysxCookingServicePrivate* PhysicsTest::acquirePhysxCookingServicePrivateInterface()
{
    carb::FrameworkScoped& framework = *mApp;
    return framework->acquireInterface<omni::physx::IPhysxCookingServicePrivate>();
}

omni::physx::IPhysxCookingService* PhysicsTest::acquirePhysxCookingServiceInterface()
{
    carb::FrameworkScoped& framework = *mApp;
    return framework->acquireInterface<omni::physx::IPhysxCookingService>();
}

omni::physx::IPhysxUnitTests* PhysicsTest::acquirePhysxUnitTestInterface()
{
    carb::FrameworkScoped& framework = *mApp;
    return framework->acquireInterface<omni::physx::IPhysxUnitTests>();    
}

omni::physx::IPhysxSimulation* PhysicsTest::acquirePhysxSimulationInterface()
{
    carb::FrameworkScoped& framework = *mApp;
    return framework->acquireInterface<omni::physx::IPhysxSimulation>();    
}

omni::physx::IPhysxSceneQuery* PhysicsTest::acquirePhysxSceneQueryInterface()
{
    carb::FrameworkScoped& framework = *mApp;
    return framework->acquireInterface<omni::physx::IPhysxSceneQuery>();
}

omni::physx::IPhysxPrivate* PhysicsTest::acquirePhysxPrivateInterface()
{
    carb::FrameworkScoped& framework = *mApp;
    return framework->acquireInterface<omni::physx::IPhysxPrivate>();
}

omni::physx::IPhysxReplicator* PhysicsTest::acquirePhysxReplicatorInterface()
{
    carb::FrameworkScoped& framework = *mApp;
    return framework->acquireInterface<omni::physx::IPhysxReplicator>();
}

carb::settings::ISettings* PhysicsTest::acquireSettingsInterface()
{
    carb::FrameworkScoped& framework = *mApp;
    return framework->acquireInterface<carb::settings::ISettings>();
}

void PhysicsTest::enablePVD(bool enable)
{
    carb::FrameworkScoped& framework = *mApp;
    auto settings = framework->acquireInterface<carb::settings::ISettings>();
    settings->setBool(omni::physx::kSettingPVDEnabled, enable);
}


PXR_NS::UsdGeomMesh createUsdGeomMesh(PXR_NS::UsdStageWeakPtr stage, PXR_NS::SdfPath path,
    const PXR_NS::VtArray<PXR_NS::GfVec3f>& points, const PXR_NS::VtArray<PXR_NS::GfVec3f>& normals,
    const PXR_NS::VtArray<int>& indices, const PXR_NS::VtArray<int>& vertexCounts)
{
    PXR_NS::UsdGeomMesh mesh = PXR_NS::UsdGeomMesh::Define(stage, path);

    mesh.CreateFaceVertexCountsAttr().Set(vertexCounts);
    mesh.CreateFaceVertexIndicesAttr().Set(indices);
    mesh.CreatePointsAttr().Set(points);
    mesh.CreateDoubleSidedAttr().Set(false);
    mesh.CreateNormalsAttr().Set(normals);

    return mesh;
};

PXR_NS::UsdGeomMesh createConcaveMesh(PXR_NS::UsdStageWeakPtr stage, PXR_NS::SdfPath path, float halfSize, float ZOffset)
{
    PXR_NS::VtArray<PXR_NS::GfVec3f> points =
    {
            PXR_NS::GfVec3f(halfSize, -halfSize, -halfSize + ZOffset),
            PXR_NS::GfVec3f(halfSize, halfSize, -halfSize + ZOffset),
            PXR_NS::GfVec3f(halfSize, halfSize, halfSize + ZOffset),
            PXR_NS::GfVec3f(halfSize, -halfSize, halfSize + ZOffset),
            PXR_NS::GfVec3f(0.0f, -halfSize, halfSize * 0.2f + ZOffset),
            PXR_NS::GfVec3f(0.0f, halfSize, halfSize * 0.2f + ZOffset),
            PXR_NS::GfVec3f(-halfSize, -halfSize, -halfSize + ZOffset),
            PXR_NS::GfVec3f(-halfSize, halfSize, -halfSize + ZOffset),
            PXR_NS::GfVec3f(-halfSize, halfSize, halfSize + ZOffset),
            PXR_NS::GfVec3f(-halfSize, -halfSize, halfSize + ZOffset),
    };

    PXR_NS::VtArray<PXR_NS::GfVec3f> normals =
    {
            PXR_NS::GfVec3f(1, 0, 0), PXR_NS::GfVec3f(1, 0, 0), PXR_NS::GfVec3f(1, 0, 0), PXR_NS::GfVec3f(1, 0, 0), PXR_NS::GfVec3f(0, 0, 1),
            PXR_NS::GfVec3f(0, 0, 1), PXR_NS::GfVec3f(-1, 0, 0), PXR_NS::GfVec3f(-1, 0, 0), PXR_NS::GfVec3f(-1, 0, 0), PXR_NS::GfVec3f(-1, 0, 0)
    };

    PXR_NS::VtArray<int> indices =
    {
            0, 1, 2, 3, 1, 7, 8, 5, 2, 3, 2, 5, 4, 4, 5, 8, 9, 9, 8, 7, 6, 0, 6, 7, 1, 0, 3, 4, 9, 6
    };

    PXR_NS::VtArray<int> vertexCounts = { 4, 5, 4, 4, 4, 4, 5 };

    return createUsdGeomMesh(stage, path, points, normals, indices, vertexCounts);
}


ScopedFabricActivation::ScopedFabricActivation()
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

    carb::Framework* framework = physicsTests.getApp()->getFramework();
    auto settings = framework->acquireInterface<carb::settings::ISettings>();
    settings->setBool("/app/settings/fabricConnectivityWithoutFSD", true);

    // Load the fabric plugin and its dependencies directly via Carbonite.
    // omni.cubric.plugin provides omni::cubric::IAdapter required by omni.physx.fabric.plugin.
    // usdrt plugins are loaded in kPhysicsPlugins (always available).
    static const std::vector<const char*> kFabricPlugins = {
        "omni.cubric.plugin",
        "omni.physx.fabric.plugin",
    };
    physicsTests.getApp()->loadPlugins(kFabricPlugins);

    omni::physx::IPhysxFabric* iPhysxFabric = framework->tryAcquireInterface<omni::physx::IPhysxFabric>();
    REQUIRE(iPhysxFabric);

    settings->setBool(omni::physx::kSettingFabricEnabled, true);

    // store for external use
    mIPhysxFabric = iPhysxFabric;
}

ScopedFabricActivation::~ScopedFabricActivation()
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

    carb::Framework* framework = physicsTests.getApp()->getFramework();
    auto settings = framework->acquireInterface<carb::settings::ISettings>();
    settings->setBool(omni::physx::kSettingFabricEnabled, false);
}

ScopedOmniPhysicsActivation::ScopedOmniPhysicsActivation()
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

    // Load the omni.physics.physx plugin directly via Carbonite
    static const std::vector<const char*> kOmniPhysicsPlugins = { "omni.physics.physx.plugin" };
    physicsTests.getApp()->loadPlugins(kOmniPhysicsPlugins);
}

ScopedOmniPhysicsActivation::~ScopedOmniPhysicsActivation()
{
}


std::ostream& PXR_INTERNAL_NS::operator<<(std::ostream& os, const UsdStageRefPtr& value)
{
    os << "UsdStageRefPtr";
    return os;
}
