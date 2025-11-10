// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include <omni/ext/IExtensions.h>
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

    const std::vector<const char*> kPhysicsPlugins = {
        "omni.physx.plugin", "omni.physx.foundation.plugin",
    };

    const std::vector<const char*> kEventsPlugins = {
        "carb.events.plugin",     "omni.ext.plugin",      "carb.scripting-python.plugin",
        "carb.dictionary.plugin", "carb.settings.plugin", "carb.dictionary.serializer-toml.plugin",
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

    mApp->loadPlugins(kEventsPlugins);
    carb::FrameworkScoped& framework = *mApp;

    mManager = mApp->getIApp()->getExtensionManager();

    auto settings = framework->acquireInterface<carb::settings::ISettings>();
    std::vector<std::string> foldersNew = carb::settings::getStringArray(settings, "/app/exts/foldersCore");
    carb::extras::Path basePath = mApp->getFileSystem()->getExecutableDirectoryPath();
    foldersNew.push_back((basePath /= "/../extsPhysics").getString());
    foldersNew.push_back((basePath /= "/../exts").getString());

    for (const std::string& folder : foldersNew)
    {
        mManager->addPath(folder.c_str(), omni::ext::ExtensionPathType::eCollection);        
    }

    // Before loading physx, load usdrt and ujitso. These are required for the tests.
    // Not being able to have ujitso enabled is not an error and can occur on non-GPU machines.
    mManager->setExtensionEnabled("usdrt.scenegraph", true);
    mManager->setExtensionEnabled("omni.ujitso.default", true);
    mManager->processAndApplyAllChanges();

    // Load physx *after* ujitso, to ensure it is available when physx.cooking is loaded
    // in order to avoid loading it "immediate" as part of that carb plugin loading which
    // can have side-effects.
    mManager->setExtensionEnabled("omni.physx", true);
    mManager->processAndApplyAllChanges();

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
    mManager = nullptr;
    delete mApp;
    mApp = nullptr;
}

std::string PhysicsTest::getDataDirectory()
{
    carb::filesystem::IFileSystem* fs = mApp->getFramework()->acquireInterface<carb::filesystem::IFileSystem>();

    return std::string(fs->getAppDirectoryPath()) + "/../../../../data/";
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


pxr::UsdGeomMesh createUsdGeomMesh(pxr::UsdStageWeakPtr stage, pxr::SdfPath path,
    const pxr::VtArray<pxr::GfVec3f>& points, const pxr::VtArray<pxr::GfVec3f>& normals,
    const pxr::VtArray<int>& indices, const pxr::VtArray<int>& vertexCounts)
{
    pxr::UsdGeomMesh mesh = pxr::UsdGeomMesh::Define(stage, path);

    mesh.CreateFaceVertexCountsAttr().Set(vertexCounts);
    mesh.CreateFaceVertexIndicesAttr().Set(indices);
    mesh.CreatePointsAttr().Set(points);
    mesh.CreateDoubleSidedAttr().Set(false);
    mesh.CreateNormalsAttr().Set(normals);

    return mesh;
};

pxr::UsdGeomMesh createConcaveMesh(pxr::UsdStageWeakPtr stage, pxr::SdfPath path, float halfSize, float ZOffset)
{
    pxr::VtArray<pxr::GfVec3f> points =
    {
            pxr::GfVec3f(halfSize, -halfSize, -halfSize + ZOffset),
            pxr::GfVec3f(halfSize, halfSize, -halfSize + ZOffset),
            pxr::GfVec3f(halfSize, halfSize, halfSize + ZOffset),
            pxr::GfVec3f(halfSize, -halfSize, halfSize + ZOffset),
            pxr::GfVec3f(0.0f, -halfSize, halfSize * 0.2f + ZOffset),
            pxr::GfVec3f(0.0f, halfSize, halfSize * 0.2f + ZOffset),
            pxr::GfVec3f(-halfSize, -halfSize, -halfSize + ZOffset),
            pxr::GfVec3f(-halfSize, halfSize, -halfSize + ZOffset),
            pxr::GfVec3f(-halfSize, halfSize, halfSize + ZOffset),
            pxr::GfVec3f(-halfSize, -halfSize, halfSize + ZOffset),
    };

    pxr::VtArray<pxr::GfVec3f> normals =
    {
            pxr::GfVec3f(1, 0, 0), pxr::GfVec3f(1, 0, 0), pxr::GfVec3f(1, 0, 0), pxr::GfVec3f(1, 0, 0), pxr::GfVec3f(0, 0, 1),
            pxr::GfVec3f(0, 0, 1), pxr::GfVec3f(-1, 0, 0), pxr::GfVec3f(-1, 0, 0), pxr::GfVec3f(-1, 0, 0), pxr::GfVec3f(-1, 0, 0)
    };

    pxr::VtArray<int> indices =
    {
            0, 1, 2, 3, 1, 7, 8, 5, 2, 3, 2, 5, 4, 4, 5, 8, 9, 9, 8, 7, 6, 0, 6, 7, 1, 0, 3, 4, 9, 6
    };

    pxr::VtArray<int> vertexCounts = { 4, 5, 4, 4, 4, 4, 5 };

    return createUsdGeomMesh(stage, path, points, normals, indices, vertexCounts);
}


ScopedFabricActivation::ScopedFabricActivation()
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

    omni::ext::ExtensionManager* extManager = physicsTests.getExtensionManager();
    REQUIRE(extManager);

    carb::Framework* framework = physicsTests.getApp()->getFramework();
    auto settings = framework->acquireInterface<carb::settings::ISettings>();
    settings->setBool("/app/settings/fabricConnectivityWithoutFSD", true);

    extManager->setExtensionEnabled("omni.physx.fabric", true);
    extManager->processAndApplyAllChanges();

    omni::physx::IPhysxFabric* iPhysxFabric = framework->tryAcquireInterface<omni::physx::IPhysxFabric>();
    REQUIRE(iPhysxFabric);

    // store for external use
    mIPhysxFabric = iPhysxFabric;
}

ScopedFabricActivation::~ScopedFabricActivation()
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

    omni::ext::ExtensionManager* extManager = physicsTests.getExtensionManager();
    REQUIRE(extManager);

    extManager->setExtensionEnabled("omni.physx.fabric", false);
    extManager->processAndApplyAllChanges();
}

ScopedPopulationActivation::ScopedPopulationActivation()
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

    omni::ext::ExtensionManager* extManager = physicsTests.getExtensionManager();
    REQUIRE(extManager);

    extManager->setExtensionEnabled("omni.hydra.usdrt_delegate", true);
    extManager->processAndApplyAllChanges();
}

ScopedPopulationActivation::~ScopedPopulationActivation()
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    omni::ext::ExtensionManager* extManager = physicsTests.getExtensionManager();
    REQUIRE(extManager);

    extManager->setExtensionEnabled("omni.hydra.usdrt_delegate", false);
    extManager->processAndApplyAllChanges();
}

ScopedOmniPhysicsActivation::ScopedOmniPhysicsActivation()
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    omni::ext::ExtensionManager* extManager = physicsTests.getExtensionManager();
    REQUIRE(extManager);

    extManager->setExtensionEnabled("omni.physics", true);
    extManager->processAndApplyAllChanges();
}

ScopedOmniPhysicsActivation::~ScopedOmniPhysicsActivation()
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    omni::ext::ExtensionManager* extManager = physicsTests.getExtensionManager();
    REQUIRE(extManager);

    extManager->setExtensionEnabled("omni.physics", false);
    extManager->processAndApplyAllChanges();
}


std::ostream& PXR_INTERNAL_NS::operator<<(std::ostream& os, const UsdStageRefPtr& value)
{
    os << "UsdStageRefPtr";
    return os;
}
