// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// Regression coverage for OMPE-95128 (NVBug 6322107): setupGPU() must not release/recreate a
// PxCudaContextManager that live PhysX scenes still cache (raw pointer) -> use-after-free crash
// on scene teardown. Back-ported to release/omni/110.1 from trunk commit baa46e5d.

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include "PhysicsTools.h"

#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxFoundation.h>
#include <omni/physx/IPhysxSettings.h>
#include <private/omni/physx/IPhysxStageUpdate.h>
#include <private/omni/physx/IPhysxTests.h>

#include <carb/settings/ISettings.h>

using namespace PXR_NS;
using namespace omni::physx;

// ---------------------------------------------------------------------------
// OMPE-95128: NpScene caches the PxCudaContextManager* at construction; setupGPU()
// used to release/replace it on a CUDA-setting change while scenes were live ->
// dangling pointer / crash. The fix defers the refresh and warns instead. Driven
// via the stage-update resume path; GPU-only (setupGPU() no-ops on CPU).
// ---------------------------------------------------------------------------

// Z-up in-memory stage with a PhysicsScene and a dynamic sphere at z=4; returns its cache id.
static long makeLiveSphereStage(UsdStageRefPtr& outStage)
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    UsdGeomSetStageMetersPerUnit(stage, 0.01);
    UsdPrim defaultPrim = stage->DefinePrim(SdfPath("/World"));
    stage->SetDefaultPrim(defaultPrim);
    UsdPhysicsScene::Define(stage, SdfPath("/World/physicsScene"));

    UsdGeomSphere sphere = UsdGeomSphere::Define(stage, SdfPath("/sphereActor"));
    sphere.CreateRadiusAttr().Set(1.0);
    UsdGeomXformable(sphere.GetPrim()).AddTranslateOp().Set(GfVec3d(0.0, 0.0, 4.0));
    UsdPhysicsCollisionAPI::Apply(sphere.GetPrim());
    UsdPhysicsRigidBodyAPI::Apply(sphere.GetPrim());

    UsdUtilsStageCache::Get().Insert(stage);
    outStage = stage;
    return UsdUtilsStageCache::Get().GetId(stage).ToLongInt();
}

// True if a usable CUDA device is present; the deferral path only runs on GPU.
static bool cudaDeviceAvailable()
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysxFoundation* foundation =
        physicsTests.getApp()->getFramework()->acquireInterface<IPhysxFoundation>();
    return foundation && foundation->cudaDeviceCheck();
}

// Restores the setting, detaches, and erases the stage on scope exit, so a throw
// inside the stage-update calls cannot leak global state into other test cases.
struct CudaSettingTestGuard
{
    IPhysxStageUpdate* stageUpdate;
    carb::settings::ISettings* settings;
    bool originalSyncLaunches;
    UsdStageRefPtr stage;
    ~CudaSettingTestGuard()
    {
        stageUpdate->onDetach();
        settings->setBool(kSettingSynchronousKernelLaunches, originalSyncLaunches);
        UsdUtilsStageCache::Get().Erase(stage);
    }
};


TEST_CASE("Simulation Extended - CUDA setting change with live scene defers (OMPE-95128)",
    "[omniphysics][component=OmniPhysics][owner=snie][priority=mandatory]")
{
    if (!cudaDeviceAvailable())
    {
        MESSAGE("Skipping: requires a CUDA device; the deferral path is GPU-only");
        return;
    }

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysxStageUpdate* stageUpdate =
        physicsTests.getApp()->getFramework()->acquireInterface<IPhysxStageUpdate>();
    REQUIRE(stageUpdate);
    IPhysxUnitTests* physxUT = physicsTests.acquirePhysxUnitTestInterface();
    REQUIRE(physxUT);
    carb::settings::ISettings* settings =
        physicsTests.getApp()->getFramework()->acquireInterface<carb::settings::ISettings>();
    REQUIRE(settings);

    const bool original = settings->getAsBool(kSettingSynchronousKernelLaunches);

    UsdStageRefPtr stage;
    const long stageId = makeLiveSphereStage(stage);
    const SdfPath spherePath("/sphereActor");
    CudaSettingTestGuard cleanup{ stageUpdate, settings, original, stage };

    // Known starting value so the later flip is a genuine change.
    settings->setBool(kSettingSynchronousKernelLaunches, false);

    // Attach + resume: create the manager and the live scene that caches it.
    stageUpdate->onAttach(stageId);
    stageUpdate->onResume(0.0f);
    for (int i = 0; i < 3; ++i)
        stageUpdate->onUpdate(static_cast<float>(i) * 0.02f, 0.02f, true);
    CHECK(physxUT->getPhysicsStats().numDynamicRigids == 1);

    // Baseline height; the sphere must keep falling after the deferral (sim ran, not no-op).
    const float heightBeforeFlip = getPhysicsPrimPos(stage, spherePath)[2];

    // Flip the CUDA-affecting setting while the scene is still live.
    settings->setBool(kSettingSynchronousKernelLaunches, true);

    // Next resume re-runs setupGPU(): must defer + warn, not release the cached manager (the UAF).
    physxUT->startLoggerCheck("change deferred until next attach", true, true);
    stageUpdate->onResume(0.0f);
    for (int i = 3; i < 6; ++i)
        stageUpdate->onUpdate(static_cast<float>(i) * 0.02f, 0.02f, true);
    CHECK(physxUT->endLoggerCheck());

    // Simulation kept running on the original manager and actually advanced.
    CHECK(physxUT->getPhysicsStats().numDynamicRigids == 1);
    CHECK(getPhysicsPrimPos(stage, spherePath)[2] < heightBeforeFlip);
    // cleanup guard restores the setting, detaches, and erases the stage.
}


// Covers the sticky guard: the deferral warning fires exactly once and is then suppressed
// while the change stays pending under a live scene.
//
// Deliberately does NOT exercise the detach -> re-attach re-arm path. Doing so forces an
// extra PxCudaContextManager release+recreate; running that inside the shared unit-test
// process churns CUDA state and corrupts a later GPU test (USD Load SDF), which then
// crashes (SIGABRT/SIGSEGV) and hangs the process on exit -> CI timeout (OMPE-95128 CI run).
// The re-arm-on-release logic is still covered indirectly by the per-attach behaviour above;
// do not add a detach/re-attach leg here without isolating it into its own process.
TEST_CASE("Simulation Extended - CUDA deferral warning is sticky (OMPE-95128)",
    "[omniphysics][component=OmniPhysics][owner=snie][priority=mandatory]")
{
    if (!cudaDeviceAvailable())
    {
        MESSAGE("Skipping: requires a CUDA device; the deferral path is GPU-only");
        return;
    }

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysxStageUpdate* stageUpdate =
        physicsTests.getApp()->getFramework()->acquireInterface<IPhysxStageUpdate>();
    REQUIRE(stageUpdate);
    IPhysxUnitTests* physxUT = physicsTests.acquirePhysxUnitTestInterface();
    REQUIRE(physxUT);
    carb::settings::ISettings* settings =
        physicsTests.getApp()->getFramework()->acquireInterface<carb::settings::ISettings>();
    REQUIRE(settings);

    const bool original = settings->getAsBool(kSettingSynchronousKernelLaunches);
    const char* const warning = "change deferred until next attach";

    UsdStageRefPtr stage;
    const long stageId = makeLiveSphereStage(stage);
    CudaSettingTestGuard cleanup{ stageUpdate, settings, original, stage };

    // Baseline + live scene that caches the manager.
    settings->setBool(kSettingSynchronousKernelLaunches, false);
    stageUpdate->onAttach(stageId);
    stageUpdate->onResume(0.0f);
    stageUpdate->onUpdate(0.0f, 0.02f, true);

    // First deferral: flipping under a live scene warns exactly once.
    settings->setBool(kSettingSynchronousKernelLaunches, true);
    physxUT->startLoggerCheck(warning, true, true);
    stageUpdate->onResume(0.0f);
    stageUpdate->onUpdate(0.02f, 0.02f, true);
    CHECK(physxUT->endLoggerCheck());

    // Still changed + still live: sticky guard suppresses the warning (expectedResult=false asserts absence).
    physxUT->startLoggerCheck(warning, false, true);
    stageUpdate->onResume(0.0f);
    stageUpdate->onUpdate(0.04f, 0.02f, true);
    CHECK(physxUT->endLoggerCheck());
    // cleanup guard restores the setting, detaches, and erases the stage.
}


// NVBug 6511059: changing OmniPVD output used to recreate PxPhysics while live scenes
// still belonged to it, leaving their PxScene pointers dangling.
struct PvdSettingTestGuard
{
    IPhysxStageUpdate* stageUpdate;
    carb::settings::ISettings* settings;
    bool originalOmniPvdOutputEnabled;
    UsdStageRefPtr stage;
    ~PvdSettingTestGuard()
    {
        stageUpdate->onDetach();
        settings->setBool(kOmniPvdOutputEnabled, originalOmniPvdOutputEnabled);
        UsdUtilsStageCache::Get().Erase(stage);
    }
};


TEST_CASE("Simulation Extended - OmniPVD setting change with live scene defers (NVBug 6511059)",
    "[omniphysics][component=OmniPhysics][owner=malesiani][priority=mandatory]")
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysxStageUpdate* stageUpdate =
        physicsTests.getApp()->getFramework()->acquireInterface<IPhysxStageUpdate>();
    REQUIRE(stageUpdate);
    IPhysxUnitTests* physxUT = physicsTests.acquirePhysxUnitTestInterface();
    REQUIRE(physxUT);
    carb::settings::ISettings* settings =
        physicsTests.getApp()->getFramework()->acquireInterface<carb::settings::ISettings>();
    REQUIRE(settings);

    const bool original = settings->getAsBool(kOmniPvdOutputEnabled);
    settings->setBool(kOmniPvdOutputEnabled, false);

    UsdStageRefPtr stage;
    const long stageId = makeLiveSphereStage(stage);
    const SdfPath spherePath("/sphereActor");
    PvdSettingTestGuard cleanup{ stageUpdate, settings, original, stage };

    stageUpdate->onAttach(stageId);
    stageUpdate->onResume(0.0f);
    for (int i = 0; i < 3; ++i)
        stageUpdate->onUpdate(static_cast<float>(i) * 0.02f, 0.02f, true);
    CHECK(physxUT->getPhysicsStats().numDynamicRigids == 1);
    const float heightBeforeFlip = getPhysicsPrimPos(stage, spherePath)[2];

    physxUT->startLoggerCheck("PVD settings change deferred until next attach", true, true);
    settings->setBool(kOmniPvdOutputEnabled, true);
    CHECK(physxUT->endLoggerCheck());

    for (int i = 3; i < 6; ++i)
        stageUpdate->onUpdate(static_cast<float>(i) * 0.02f, 0.02f, true);

    CHECK(physxUT->getPhysicsStats().numDynamicRigids == 1);
    CHECK(getPhysicsPrimPos(stage, spherePath)[2] < heightBeforeFlip);
    // cleanup guard restores the setting, detaches, and erases the stage.
}
