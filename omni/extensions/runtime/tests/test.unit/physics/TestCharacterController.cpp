// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include "PhysicsTools.h"

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxTests.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxCct.h>

using namespace pxr;
using namespace omni::physx;
using namespace ::physx;

const float frameTime = 1.0f / 60.0f;

//-----------------------------------------------------------------------------
// Init test
TEST_CASE("CCT Tests",
          "[omniphysics]"
          "[component=OmniPhysics][owner=mhapala][priority=mandatory]")
{
    // constants for setup
    const GfVec3f gravityDirection(0.0f, 0.0f, -1.0f); // use z-up
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const float gravityMagnitude = 10.0f / metersPerStageUnit;
    const float density = 1000.f * metersPerStageUnit * metersPerStageUnit * metersPerStageUnit;

    // setup common to all subcases
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);
    scene.GetGravityMagnitudeAttr().Set(gravityMagnitude);
    scene.GetGravityDirectionAttr().Set(gravityDirection);

    // setup CCT:
    const SdfPath cctPath = defaultPrimPath.AppendChild(TfToken("cctShape"));
    UsdGeomCapsule capsule = UsdGeomCapsule::Define(stage, cctPath);
    capsule.AddScaleOp().Set(GfVec3d(1.0)); // TODO test with nonunit scale and different up axes
    PhysxSchemaPhysxCharacterControllerAPI physxCctAPI = PhysxSchemaPhysxCharacterControllerAPI::Apply(capsule.GetPrim());

    SUBCASE("BasicParsingAndUpdates")
    {
        // initial values for parsing test:
        float slopeLimit = 0.98357f;
        double capsuleHeight = 0.237;
        double capsuleRadius = 0.54;
        float contactOffset = 0.44f;
        float stepOffset = 0.567f;

        capsule.CreateHeightAttr().Set(capsuleHeight);
        capsule.CreateRadiusAttr().Set(capsuleRadius);
        physxCctAPI.CreateSlopeLimitAttr().Set(slopeLimit);
        physxCctAPI.CreateContactOffsetAttr().Set(contactOffset);
        physxCctAPI.CreateStepOffsetAttr().Set(stepOffset);
        physxCctAPI.CreateNonWalkableModeAttr().Set(PhysxSchemaTokens->preventClimbing);
        physxCctAPI.CreateUpAxisAttr().Set(PhysxSchemaTokens->Z);
        physxCctAPI.CreateClimbingModeAttr().Set(PhysxSchemaTokens->easy);

        // NOTE: untested, no setters/getters for these at the moment and can't be changed at runtime
        // float jumpHeight = 0.34f;
        // float scaleCoef = 0.644f;
        // float volumeGrowth = 1.4f;
        // physxCctAPI.CreateVolumeGrowthAttr().Set(volumeGrowth);
        // physxCctAPI.CreateMaxJumpHeightAttr().Set(jumpHeight);
        // physxCctAPI.CreateScaleCoeffAttr().Set(scaleCoef);
        // physxCctAPI.CreateScaleCoeffAttr().Set(scaleCoef);

        // NOTE: untested, would need cct plugin dep, test in python however
        // GfVec3f moveTarget = GfVec3f(1.0f, 2.0f, 3.0f);
        // physxCctAPI.CreateMoveTargetAttr().Set(moveTarget);

        // parse
        physxSim->attachStage(stageId);

        // get physx cct
        ::physx::PxCapsuleController* physxCCTcontroller = reinterpret_cast<::physx::PxCapsuleController*>(physx->getPhysXPtr(cctPath, omni::physx::PhysXType::ePTCct));
        REQUIRE(physxCCTcontroller);

        // check parsing
        CHECK(doctest::Approx(capsuleHeight) == physxCCTcontroller->getHeight());
        CHECK(doctest::Approx(capsuleRadius) == physxCCTcontroller->getRadius());
        CHECK(doctest::Approx(slopeLimit) == physxCCTcontroller->getSlopeLimit());
        CHECK(doctest::Approx(contactOffset) == physxCCTcontroller->getContactOffset());
        CHECK(doctest::Approx(stepOffset) == physxCCTcontroller->getStepOffset());
        CHECK_EQ(PxControllerNonWalkableMode::ePREVENT_CLIMBING, physxCCTcontroller->getNonWalkableMode());
        compare(physxCCTcontroller->getUpDirection(), GfVec3f(0.0f, 0.0f, 1.0f), 1.0e-6f);
        CHECK_EQ(PxCapsuleClimbingMode::eEASY, physxCCTcontroller->getClimbingMode());

        // modify parameters
        capsuleHeight = 0.536;
        capsuleRadius = 0.154;
        slopeLimit = 0.2345f;
        contactOffset = 0.64f;
        stepOffset = 0.767f;
        capsule.CreateHeightAttr().Set(capsuleHeight);
        capsule.CreateRadiusAttr().Set(capsuleRadius);
        physxCctAPI.CreateSlopeLimitAttr().Set(slopeLimit);
        physxCctAPI.CreateContactOffsetAttr().Set(contactOffset);
        physxCctAPI.CreateStepOffsetAttr().Set(stepOffset);
        physxCctAPI.CreateNonWalkableModeAttr().Set(PhysxSchemaTokens->preventClimbingForceSliding);
        physxCctAPI.CreateUpAxisAttr().Set(PhysxSchemaTokens->Y);
        physxCctAPI.CreateClimbingModeAttr().Set(PhysxSchemaTokens->constrained);

        // step to trigger updates
        physxSim->simulate(frameTime, 0.0f);
        physxSim->fetchResults();

        // check updated CCT
        CHECK(doctest::Approx(capsuleHeight) == physxCCTcontroller->getHeight());
        CHECK(doctest::Approx(capsuleRadius) == physxCCTcontroller->getRadius());
        CHECK(doctest::Approx(slopeLimit) == physxCCTcontroller->getSlopeLimit());
        CHECK(doctest::Approx(contactOffset) == physxCCTcontroller->getContactOffset());
        CHECK(doctest::Approx(stepOffset) == physxCCTcontroller->getStepOffset());
        CHECK_EQ(PxControllerNonWalkableMode::ePREVENT_CLIMBING_AND_FORCE_SLIDING, physxCCTcontroller->getNonWalkableMode());
        compare(physxCCTcontroller->getUpDirection(), GfVec3f(0.0f, 1.0f, 0.0f), 1.0e-6f);
        CHECK_EQ(PxCapsuleClimbingMode::eCONSTRAINED, physxCCTcontroller->getClimbingMode());
    }

    SUBCASE("Multiple Scenes")
    {
        // create an additional physics scene
        const SdfPath physicsScene2Path = defaultPrimPath.AppendChild(TfToken("physicsScene2"));
        UsdPhysicsScene scene2 = UsdPhysicsScene::Define(stage, physicsScene2Path);
        scene2.GetGravityMagnitudeAttr().Set(gravityMagnitude);
        scene2.GetGravityDirectionAttr().Set(gravityDirection);

        // set the new scene as a cct owner
        SdfPathVector targets;
        targets.push_back(physicsScene2Path);
        physxCctAPI.CreateSimulationOwnerRel().SetTargets(targets);

        // parse
        physxSim->attachStage(stageId);

        // check
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScene2Path, ePTScene));
        REQUIRE(basePtr != nullptr);
        PxScene* pxScene2 = (PxScene*)basePtr;

        PxCapsuleController* physxCCTcontroller = reinterpret_cast<PxCapsuleController*>(physx->getPhysXPtr(cctPath, ePTCct));
        REQUIRE(physxCCTcontroller != nullptr);

        CHECK(pxScene2 == physxCCTcontroller->getScene());
    }

    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
