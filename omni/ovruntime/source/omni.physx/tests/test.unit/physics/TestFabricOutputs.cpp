// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"
#include "../../common/PhysicsChangeTemplate.h"

#include "PhysicsTools.h"

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxSettings.h>
#include <omni/physx/PhysxTokens.h>
#include <omni/physx/IPhysxFabric.h>

#include <omni/fabric/SimStageWithHistory.h>


using namespace PXR_NS;
using namespace omni::physx;
using namespace ::physx;
using namespace carb;
using namespace omni::fabric;
//-----------------------------------------------------------------------------
// fabric joint state

template <typename JointType, typename TestParameters>
void testArticulationJointState(ScopedFabricActivation& fabricEnable)
{
    TestParameters testParams;

    FabricChange changeTemplate;
    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    PXR_NS::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = PXR_NS::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));

    PXR_NS::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = PXR_NS::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath articulationPrimPath = SdfPath("/World");
    UsdGeomXform articulationPrim = UsdGeomXform::Define(stage, articulationPrimPath);
    UsdPhysicsArticulationRootAPI::Apply(articulationPrim.GetPrim());

    const SdfPath physicsScenePath = articulationPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

    // create rigid body - rootLink
    addRigidBox(stage, "/World/rootLink", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);

    const SdfPath rootLinkPath = SdfPath("/World/rootLink");
    UsdPrim rootLinkPrim = stage->GetPrimAtPath(rootLinkPath);

    const SdfPath fixedJointPath = SdfPath("/World/baseFixedJoint");
    UsdPhysicsFixedJoint fixedJoint = UsdPhysicsFixedJoint::Define(stage, fixedJointPath);
    fixedJoint.GetBody1Rel().AddTarget(rootLinkPath);

    // create rigid body - dynamicLink
    addRigidBox(stage, "/World/dynamicLink", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);

    const SdfPath dynamicLinkPath = SdfPath("/World/dynamicLink");
    UsdPrim dynamicLinkPrim = stage->GetPrimAtPath(dynamicLinkPath);


    const SdfPath jointPath = SdfPath("/World/joint");
    JointType usdJoint = JointType::Define(stage, jointPath);
    usdJoint.GetBody0Rel().AddTarget(rootLinkPath);
    usdJoint.GetBody1Rel().AddTarget(dynamicLinkPath);
    usdJoint.GetLowerLimitAttr().Set(-90.0f);
    usdJoint.GetUpperLimitAttr().Set(90.0f);

    PhysxSchemaPhysxJointAPI physxJointAPI = PhysxSchemaPhysxJointAPI::Apply(usdJoint.GetPrim());
    PhysxSchemaPhysxLimitAPI limitAPI =
        PhysxSchemaPhysxLimitAPI::Apply(usdJoint.GetPrim(), TfToken(testParams.jointTypeAttributeText));
    PhysxSchemaJointStateAPI jointStateAPI =
        PhysxSchemaJointStateAPI::Apply(usdJoint.GetPrim(), TfToken(testParams.jointTypeAttributeText));

    // This creates the fabric stage
    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);
    fabricEnable.mIPhysxFabric->attachStage(stageId);

    const TfToken jointStatePositionToken(testParams.jointStateAttributeText);

    float value = changeTemplate.template getAttributeValue<float>(jointPath, jointStatePositionToken);
    // check default value
    CHECK(fabsf(value) < epsilon);

    value = testParams.startJointStateValue;
    changeTemplate.setAttributeValue(jointPath, jointStatePositionToken, value);
    physxSim->simulate(1.0f / 60, 0.0f);
    physxSim->fetchResults();
    fabricEnable.mIPhysxFabric->update(1.0f / 60, 0.0f);

    omni::fabric::IStageReaderWriter* iSip = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    SUBCASE("Position")
    {
        omni::fabric::StageReaderWriter stage = iSip->get(stageId);
        const omni::fabric::Token positionToken(jointStatePositionToken.GetText());
        omni::fabric::Path fabricPath = omni::fabric::convertToPathType<omni::fabric::Path>(stage.getFabricId(), jointPath);
        REQUIRE(stage.attributeExists(fabricPath, positionToken));
        const float jointState = *stage.getAttributeRd<float>(fabricPath, positionToken);
        CHECK(fabsf(jointState - value) < epsilon);
    }

    SUBCASE("Velocity")
    {
        UsdPhysicsDriveAPI driveAPI =
            UsdPhysicsDriveAPI::Apply(usdJoint.GetPrim(), TfToken(testParams.jointTypeAttributeText));
        driveAPI.GetStiffnessAttr().Set(1000.0f);
        driveAPI.GetDampingAttr().Set(100.0f);
        const TfToken jointDriveToken(testParams.jointDriveAttributeText);
        value = testParams.driveTargetValue;
        changeTemplate.setAttributeValue(jointPath, jointDriveToken, value);

        for (int i = 0; i < 5; ++i)
        {
            physxSim->simulate(1.0f / 60, 0.0f);
            physxSim->fetchResults();
            fabricEnable.mIPhysxFabric->update(1.0f / 60, 0.0f);
        }
        omni::fabric::StageReaderWriter stage = iSip->get(stageId);
        const omni::fabric::Token velocityToken(testParams.jointVelocityAttributeText);
        omni::fabric::Path fabricPath = omni::fabric::convertToPathType<omni::fabric::Path>(stage.getFabricId(), jointPath);
        REQUIRE(stage.attributeExists(fabricPath, velocityToken));
        const float jointState = *stage.getAttributeRd<float>(fabricPath, velocityToken);
        CHECK(fabsf(jointState - testParams.jointStateVelocityValue) < 0.01f);
    }

    // Common post-test actions
    fabricEnable.mIPhysxFabric->detachStage(); 
    physxSim->detachStage();
    changeTemplate.destroy();

    PXR_NS::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
struct ArticulationJointStateLinearTestParams
{
    const char* jointTypeAttributeText = "linear";
    const char* jointStateAttributeText = "state:linear:physics:position";
    const char* jointDriveAttributeText = "drive:linear:physics:targetPosition";
    const char* jointVelocityAttributeText = "state:linear:physics:velocity";
    const float startJointStateValue = 10.0f;
    const float driveTargetValue = 20.0f;
    const float jointStateVelocityValue = 0.828f;
};

struct ArticulationJointStateAngularTestParams
{
    const char* jointTypeAttributeText = "angular";
    const char* jointStateAttributeText = "state:angular:physics:position";
    const char* jointDriveAttributeText = "drive:angular:physics:targetPosition";
    const char* jointVelocityAttributeText = "state:angular:physics:velocity";
    const float startJointStateValue = 10.0f;
    const float driveTargetValue = 20.0f;
    const float jointStateVelocityValue = 0.02859f;
};

void testTransformationUpdate(ScopedFabricActivation& fabricEnable)
{
    FabricChange changeTemplate;
    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    PXR_NS::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = PXR_NS::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));

    PXR_NS::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = PXR_NS::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    // This creates the fabric stage
    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    const SdfPath rootPrimPath = SdfPath("/World");
    UsdGeomXform::Define(stage, rootPrimPath);

    const SdfPath physicsScenePath = rootPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene::Define(stage, physicsScenePath);

    addRigidBox(stage, "/World/body", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);

    const SdfPath bodyPath = SdfPath("/World/body");
    UsdPrim bodyPrim = stage->GetPrimAtPath(bodyPath);
    CHECK(bodyPrim);

    SUBCASE("Kinematic Body")
    {
        UsdPhysicsRigidBodyAPI rboApi(bodyPrim);
        rboApi.CreateKinematicEnabledAttr().Set(true);

        physxSim->attachStage(stageId);
        fabricEnable.mIPhysxFabric->attachStage(stageId);

        for (int i = 0; i < 5; ++i)
        {
            physxSim->simulate(1.0f / 60, 0.0f);
            physxSim->fetchResults();
            fabricEnable.mIPhysxFabric->update(1.0f / 60, 0.0f);
        }

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(bodyPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK((basePtr->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) == true);
        PxRigidDynamic* dynamicBody = basePtr->is<PxRigidDynamic>();

        const PxTransform tr = dynamicBody->getGlobalPose();
        const GfTransform gftr = changeTemplate.getTransformation(bodyPath);

        compare(tr.p, GfVec3f(gftr.GetTranslation()), epsilon);
    }

    SUBCASE("Dynamic Body")
    {
        physxSim->attachStage(stageId);
        fabricEnable.mIPhysxFabric->attachStage(stageId);
        
        for (int i = 0; i < 5; ++i)
        {
            physxSim->simulate(1.0f / 60, 0.0f);
            physxSim->fetchResults();
            fabricEnable.mIPhysxFabric->update(1.0f / 60, 0.0f);
        }

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(bodyPath, ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidDynamic>());
        CHECK((basePtr->is<PxRigidDynamic>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) == false);
        PxRigidDynamic* dynamicBody = basePtr->is<PxRigidDynamic>();

        const PxTransform tr = dynamicBody->getGlobalPose();
        const GfTransform gftr = changeTemplate.getTransformation(bodyPath);

        compare(tr.p, GfVec3f(gftr.GetTranslation()), epsilon);
    }

    // Common post-test actions
    fabricEnable.mIPhysxFabric->detachStage();
    physxSim->detachStage();
    changeTemplate.destroy();

    PXR_NS::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

// Two dynamic rigid bodies where the second is nested under the first in the USD hierarchy
// (e.g. /World/parentBody/childBody). PhysX simulates both independently in world space, so on
// stop (with resetOnStop disabled) FabricManager::saveToUsd must write the child's local
// transform relative to the parent's *newly saved* world pose, not a stale/pre-simulation one -
// otherwise the child ends up offset by however far the parent moved.
void testNestedBodyTransformSaveOnStop(ScopedFabricActivation& fabricEnable)
{
    const float epsilon = 0.01f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);
    carb::settings::ISettings* settings = physicsTests.acquireSettingsInterface();

    const bool resetOnStop = settings->getAsBool(kSettingResetOnStop);
    settings->setBool(kSettingResetOnStop, false);
    ExecOnDestruction restoreResetOnStop([&] { settings->setBool(kSettingResetOnStop, resetOnStop); });

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    PXR_NS::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    PXR_NS::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));

    PXR_NS::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = PXR_NS::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    FabricChange changeTemplate;
    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    const SdfPath rootPrimPath = SdfPath("/World");
    UsdGeomXform::Define(stage, rootPrimPath);

    const SdfPath physicsScenePath = rootPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene::Define(stage, physicsScenePath);

    // size=1 avoids compounding the parent's scale into the child's local offset, which would
    // otherwise make the world-space arithmetic below harder to reason about.
    // Both bodies start with a non-identity orientation so that a parent-before-child propagation
    // regression in rotation (not just translation) would also be caught below.
    const GfQuatf parentOrientation(GfRotation(GfVec3d(0.0, 0.0, 1.0), 30.0).GetQuat());
    const GfQuatf childOrientation(GfRotation(GfVec3d(1.0, 0.0, 0.0), 20.0).GetQuat());

    const SdfPath parentPath = SdfPath("/World/parentBody");
    addRigidBox(stage, parentPath.GetString(), GfVec3f(1.f), GfVec3f(0.0f, 0.0f, 500.0f), parentOrientation, GfVec3f(0.7f), 0.001f);

    // childBody is nested under parentBody in the USD hierarchy, but it is an independent
    // dynamic rigid body - physics simulates it in world space regardless of USD parenting. It is
    // given a lateral velocity the parent doesn't have, so their relative offset actually changes
    // over the simulation instead of staying constant (which a stale/never-updated child local
    // transform could otherwise coincidentally still reproduce under uniform gravity alone).
    const SdfPath childPath = parentPath.AppendChild(TfToken("childBody"));
    addRigidBox(stage, childPath.GetString(), GfVec3f(1.f), GfVec3f(300.0f, 0.0f, 800.0f), childOrientation, GfVec3f(0.7f), 0.001f,
        GfVec3f(50.0f, 0.0f, 0.0f));

    UsdPrim parentPrim = stage->GetPrimAtPath(parentPath);
    UsdPrim childPrim = stage->GetPrimAtPath(childPath);
    REQUIRE(parentPrim);
    REQUIRE(childPrim);
    CHECK(childPrim.GetParent() == parentPrim);

    bool physxAttached = false;
    bool fabricAttached = false;
    // Registered as soon as the stages are attached so a fatal REQUIRE below still detaches them
    // and erases the stage cache entry instead of leaking attached state.
    ExecOnDestruction detachStages([&] {
        if (fabricAttached)
        {
            fabricEnable.mIPhysxFabric->detachStage();
        }
        if (physxAttached)
        {
            physxSim->detachStage();
        }
        PXR_NS::UsdUtilsStageCache::Get().Erase(stage);
    });

    physxSim->attachStage(stageId);
    physxAttached = true;
    fabricEnable.mIPhysxFabric->attachStage(stageId);
    fabricAttached = true;

    for (int i = 0; i < 30; ++i)
    {
        physxSim->simulate(1.0f / 60, 0.0f);
        physxSim->fetchResults();
        fabricEnable.mIPhysxFabric->update(1.0f / 60, 0.0f);
    }

    // Ground truth: read the exact same fabric attribute that FabricManager::saveToUsd() reads
    // ("omni:fabric:worldMatrix"), captured before stop() runs.
    omni::fabric::IStageReaderWriter* iSip = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    const omni::fabric::Token tokenWorldMatrix("omni:fabric:worldMatrix");
    GfMatrix4d parentGroundTruth;
    GfMatrix4d childGroundTruth;
    {
        omni::fabric::StageReaderWriter fabricStage = iSip->get(stageId);
        const omni::fabric::Path parentFabricPath =
            omni::fabric::convertToPathType<omni::fabric::Path>(fabricStage.getFabricId(), parentPath);
        const omni::fabric::Path childFabricPath =
            omni::fabric::convertToPathType<omni::fabric::Path>(fabricStage.getFabricId(), childPath);
        REQUIRE(fabricStage.attributeExists(parentFabricPath, tokenWorldMatrix));
        REQUIRE(fabricStage.attributeExists(childFabricPath, tokenWorldMatrix));
        parentGroundTruth = *fabricStage.getAttributeRd<GfMatrix4d>(parentFabricPath, tokenWorldMatrix);
        childGroundTruth = *fabricStage.getAttributeRd<GfMatrix4d>(childFabricPath, tokenWorldMatrix);
    }

    // Triggers FabricManager::stop(); with resetOnStop disabled this must save the current
    // fabric-driven transforms back to USD, honoring the parent/child hierarchy.
    fabricEnable.mIPhysxFabric->detachStage();
    fabricAttached = false;

    // Reimport the root layer into a plain, Fabric-unattached stage so that the transforms read
    // back below reflect exactly what was authored to USD - not values live-bridged from Fabric.
    std::string usda;
    stage->GetRootLayer()->ExportToString(&usda);
    UsdStageRefPtr verifyStage = UsdStage::CreateInMemory();
    verifyStage->GetRootLayer()->ImportFromString(usda);

    UsdPrim verifyParentPrim = verifyStage->GetPrimAtPath(parentPath);
    UsdPrim verifyChildPrim = verifyStage->GetPrimAtPath(childPath);
    REQUIRE(verifyParentPrim);
    REQUIRE(verifyChildPrim);

    UsdGeomXformCache xformCache;
    const GfMatrix4d parentWorld = xformCache.GetLocalToWorldTransform(verifyParentPrim);
    const GfMatrix4d childWorld = xformCache.GetLocalToWorldTransform(verifyChildPrim);

    compare(GfVec3f(parentGroundTruth.ExtractTranslation()), GfVec3f(parentWorld.ExtractTranslation()), epsilon);
    compare(GfVec3f(childGroundTruth.ExtractTranslation()), GfVec3f(childWorld.ExtractTranslation()), epsilon);

    const GfQuatf parentWorldRotation(parentWorld.ExtractRotationQuat());
    const GfQuatf childWorldRotation(childWorld.ExtractRotationQuat());
    const GfQuatf parentGroundTruthRotation(parentGroundTruth.ExtractRotationQuat());
    const GfQuatf childGroundTruthRotation(childGroundTruth.ExtractRotationQuat());
    compare(parentGroundTruthRotation, parentWorldRotation, epsilon);
    compare(childGroundTruthRotation, childWorldRotation, epsilon);

    // Common post-test actions
    physxSim->detachStage();
    physxAttached = false;
    changeTemplate.destroy();
    stage = nullptr;
}

// Same premise as testNestedBodyTransformSaveOnStop(), except childBody sits a level further
// down the hierarchy behind a plain (non-rigid-body) Xform prim: /World/parentBody/mid/childBody.
// FabricManager::saveToUsd() must still express childBody's saved local transform relative to
// parentBody's freshly saved world pose - not a stale, pre-simulation one - even though parentBody
// is not childBody's direct USD parent.
void testNestedBodyTransformSaveOnStopIntermediateXform(ScopedFabricActivation& fabricEnable)
{
    const float epsilon = 0.01f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);
    carb::settings::ISettings* settings = physicsTests.acquireSettingsInterface();

    const bool resetOnStop = settings->getAsBool(kSettingResetOnStop);
    settings->setBool(kSettingResetOnStop, false);
    ExecOnDestruction restoreResetOnStop([&] { settings->setBool(kSettingResetOnStop, resetOnStop); });

    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    PXR_NS::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f;
    PXR_NS::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));

    PXR_NS::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = PXR_NS::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    FabricChange changeTemplate;
    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    const SdfPath rootPrimPath = SdfPath("/World");
    UsdGeomXform::Define(stage, rootPrimPath);

    const SdfPath physicsScenePath = rootPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene::Define(stage, physicsScenePath);

    const SdfPath parentPath = SdfPath("/World/parentBody");
    addRigidBox(stage, parentPath.GetString(), GfVec3f(1.f), GfVec3f(0.0f, 0.0f, 500.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);

    // A plain Xform between two rigid bodies: it is not itself a rigid body, so it must be
    // skipped over when looking for the nearest updating ancestor.
    const SdfPath midPath = parentPath.AppendChild(TfToken("mid"));
    UsdGeomXform::Define(stage, midPath);

    const SdfPath childPath = midPath.AppendChild(TfToken("childBody"));
    addRigidBox(stage, childPath.GetString(), GfVec3f(1.f), GfVec3f(300.0f, 0.0f, 800.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f,
        GfVec3f(50.0f, 0.0f, 0.0f));

    UsdPrim parentPrim = stage->GetPrimAtPath(parentPath);
    UsdPrim childPrim = stage->GetPrimAtPath(childPath);
    REQUIRE(parentPrim);
    REQUIRE(childPrim);
    CHECK(childPrim.GetParent().GetPrimPath() == midPath);

    bool physxAttached = false;
    bool fabricAttached = false;
    ExecOnDestruction detachStages([&] {
        if (fabricAttached)
        {
            fabricEnable.mIPhysxFabric->detachStage();
        }
        if (physxAttached)
        {
            physxSim->detachStage();
        }
        PXR_NS::UsdUtilsStageCache::Get().Erase(stage);
    });

    physxSim->attachStage(stageId);
    physxAttached = true;
    fabricEnable.mIPhysxFabric->attachStage(stageId);
    fabricAttached = true;

    for (int i = 0; i < 30; ++i)
    {
        physxSim->simulate(1.0f / 60, 0.0f);
        physxSim->fetchResults();
        fabricEnable.mIPhysxFabric->update(1.0f / 60, 0.0f);
    }

    omni::fabric::IStageReaderWriter* iSip = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    const omni::fabric::Token tokenWorldMatrix("omni:fabric:worldMatrix");
    GfMatrix4d childGroundTruth;
    {
        omni::fabric::StageReaderWriter fabricStage = iSip->get(stageId);
        const omni::fabric::Path childFabricPath =
            omni::fabric::convertToPathType<omni::fabric::Path>(fabricStage.getFabricId(), childPath);
        REQUIRE(fabricStage.attributeExists(childFabricPath, tokenWorldMatrix));
        childGroundTruth = *fabricStage.getAttributeRd<GfMatrix4d>(childFabricPath, tokenWorldMatrix);
    }

    fabricEnable.mIPhysxFabric->detachStage();
    fabricAttached = false;

    std::string usda;
    stage->GetRootLayer()->ExportToString(&usda);
    UsdStageRefPtr verifyStage = UsdStage::CreateInMemory();
    verifyStage->GetRootLayer()->ImportFromString(usda);

    UsdPrim verifyChildPrim = verifyStage->GetPrimAtPath(childPath);
    REQUIRE(verifyChildPrim);

    UsdGeomXformCache xformCache;
    const GfMatrix4d childWorld = xformCache.GetLocalToWorldTransform(verifyChildPrim);

    compare(GfVec3f(childGroundTruth.ExtractTranslation()), GfVec3f(childWorld.ExtractTranslation()), epsilon);
    compare(GfQuatf(childGroundTruth.ExtractRotationQuat()), GfQuatf(childWorld.ExtractRotationQuat()), epsilon);

    physxSim->detachStage();
    physxAttached = false;
    changeTemplate.destroy();
    stage = nullptr;
}

// Same premise as testNestedBodyTransformSaveOnStop(), except childBody has resetXformStack set:
// its world transform is authored as absolute (it does not compose through parentBody's
// transform), so FabricManager::saveToUsd() must not apply any parent-relative correction to it,
// even though it is nested under a moving rigid-body parent in the USD hierarchy.
void testNestedBodyTransformSaveOnStopResetXformStack(ScopedFabricActivation& fabricEnable)
{
    const float epsilon = 0.01f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);
    carb::settings::ISettings* settings = physicsTests.acquireSettingsInterface();

    const bool resetOnStop = settings->getAsBool(kSettingResetOnStop);
    settings->setBool(kSettingResetOnStop, false);
    ExecOnDestruction restoreResetOnStop([&] { settings->setBool(kSettingResetOnStop, resetOnStop); });

    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    PXR_NS::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f;
    PXR_NS::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));

    PXR_NS::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = PXR_NS::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    FabricChange changeTemplate;
    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    const SdfPath rootPrimPath = SdfPath("/World");
    UsdGeomXform::Define(stage, rootPrimPath);

    const SdfPath physicsScenePath = rootPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene::Define(stage, physicsScenePath);

    const SdfPath parentPath = SdfPath("/World/parentBody");
    addRigidBox(stage, parentPath.GetString(), GfVec3f(1.f), GfVec3f(0.0f, 0.0f, 500.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);

    const SdfPath childPath = parentPath.AppendChild(TfToken("childBody"));
    addRigidBox(stage, childPath.GetString(), GfVec3f(1.f), GfVec3f(300.0f, 0.0f, 800.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f,
        GfVec3f(50.0f, 0.0f, 0.0f));

    UsdPrim parentPrim = stage->GetPrimAtPath(parentPath);
    UsdPrim childPrim = stage->GetPrimAtPath(childPath);
    REQUIRE(parentPrim);
    REQUIRE(childPrim);
    CHECK(childPrim.GetParent() == parentPrim);

    UsdGeomXform(childPrim).SetResetXformStack(true);

    bool physxAttached = false;
    bool fabricAttached = false;
    ExecOnDestruction detachStages([&] {
        if (fabricAttached)
        {
            fabricEnable.mIPhysxFabric->detachStage();
        }
        if (physxAttached)
        {
            physxSim->detachStage();
        }
        PXR_NS::UsdUtilsStageCache::Get().Erase(stage);
    });

    physxSim->attachStage(stageId);
    physxAttached = true;
    fabricEnable.mIPhysxFabric->attachStage(stageId);
    fabricAttached = true;

    for (int i = 0; i < 30; ++i)
    {
        physxSim->simulate(1.0f / 60, 0.0f);
        physxSim->fetchResults();
        fabricEnable.mIPhysxFabric->update(1.0f / 60, 0.0f);
    }

    omni::fabric::IStageReaderWriter* iSip = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    const omni::fabric::Token tokenWorldMatrix("omni:fabric:worldMatrix");
    GfMatrix4d childGroundTruth;
    {
        omni::fabric::StageReaderWriter fabricStage = iSip->get(stageId);
        const omni::fabric::Path childFabricPath =
            omni::fabric::convertToPathType<omni::fabric::Path>(fabricStage.getFabricId(), childPath);
        REQUIRE(fabricStage.attributeExists(childFabricPath, tokenWorldMatrix));
        childGroundTruth = *fabricStage.getAttributeRd<GfMatrix4d>(childFabricPath, tokenWorldMatrix);
    }

    fabricEnable.mIPhysxFabric->detachStage();
    fabricAttached = false;

    std::string usda;
    stage->GetRootLayer()->ExportToString(&usda);
    UsdStageRefPtr verifyStage = UsdStage::CreateInMemory();
    verifyStage->GetRootLayer()->ImportFromString(usda);

    UsdPrim verifyChildPrim = verifyStage->GetPrimAtPath(childPath);
    REQUIRE(verifyChildPrim);

    // childBody has resetXformStack, so its computed world transform equals its local transform
    // regardless of parentBody's authored pose.
    UsdGeomXformCache xformCache;
    const GfMatrix4d childWorld = xformCache.GetLocalToWorldTransform(verifyChildPrim);

    compare(GfVec3f(childGroundTruth.ExtractTranslation()), GfVec3f(childWorld.ExtractTranslation()), epsilon);
    compare(GfQuatf(childGroundTruth.ExtractRotationQuat()), GfQuatf(childWorld.ExtractRotationQuat()), epsilon);

    physxSim->detachStage();
    physxAttached = false;
    changeTemplate.destroy();
    stage = nullptr;
}

TEST_CASE("Fabric Outputs Tests",
          "[omniphysics]"
          "[component=OmniPhysics][owner=scristiano][priority=mandatory]")
{
#if !CARB_AARCH64 // OM-111136: Fabric Crash on ARM64 during physics unit test "Fabric Outputs Tests" (TestFabricOutputs.cpp)
    ScopedFabricActivation fabricEnable;
    SUBCASE("Articulation Prismatic Joint State")
    {
        testArticulationJointState<UsdPhysicsPrismaticJoint, ArticulationJointStateLinearTestParams>(fabricEnable);
    }
    SUBCASE("Articulation Angular Joint State")
    {
        testArticulationJointState<UsdPhysicsRevoluteJoint, ArticulationJointStateAngularTestParams>(fabricEnable);
    }
    SUBCASE("Transformation Update")
    {
        testTransformationUpdate(fabricEnable);
    }
    SUBCASE("Nested Body Transform Save On Stop")
    {
        testNestedBodyTransformSaveOnStop(fabricEnable);
    }
    SUBCASE("Nested Body Transform Save On Stop - Intermediate Xform")
    {
        testNestedBodyTransformSaveOnStopIntermediateXform(fabricEnable);
    }
    SUBCASE("Nested Body Transform Save On Stop - ResetXformStack Child")
    {
        testNestedBodyTransformSaveOnStopResetXformStack(fabricEnable);
    }
#endif
}
