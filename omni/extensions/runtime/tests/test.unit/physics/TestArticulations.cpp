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

#include <cmath>

using namespace pxr;
using namespace omni::physx;
using namespace ::physx;

//-----------------------------------------------------------------------------
// Init test
TEST_CASE("Articulations Tests",
          "[omniphysics]"
          "[component=OmniPhysics][owner=preist][priority=mandatory]")
{
    // constants for test checks
    const float positionToleranceCM = 0.1f;
    const float rotToleranceDeg = 1.0f;
    const float angleToleranceRad = rotToleranceDeg * PxPi / 180.0f;

    // constants for setup
    const GfVec3f gravityDirection(0.0f, 0.0f, -1.0f); // use z-up
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const float kilogramsPerUnit = 1.0f;
    const float lengthScale = 1.0f / metersPerStageUnit;
    const float massScale = 1.0f / kilogramsPerUnit;
    const float forceScale = massScale * lengthScale;
    const float gravityMagnitude = 10.0f / metersPerStageUnit;
    const float density = 1000.f * massScale / (lengthScale * lengthScale * lengthScale);
    const float linkLength = 0.1f * lengthScale;
    const float linkWidth = 0.01f * lengthScale;
    const float linkMass = linkLength * linkWidth * linkWidth * density;
    const GfVec3f linkDims(linkLength, linkWidth, linkWidth);

    // setup common to all subcases
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    pxr::UsdPhysicsSetStageKilogramsPerUnit(stage, static_cast<double>(kilogramsPerUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);
    scene.GetGravityMagnitudeAttr().Set(gravityMagnitude);
    scene.GetGravityDirectionAttr().Set(gravityDirection);

    // setup linear articulation rigid bodies, extending in +X
    auto SetupRBLinksAtPaths = [&](const std::vector<SdfPath>& absolutePaths, const GfVec3f& offset = GfVec3f(0.0f))
    {
        const GfVec3f rootColor(0.7f, 0.1f, 0.1f);
        const GfVec3f linkColor(0.1f, 0.1f, 0.7f);

        GfVec3f linkPos = offset;
        for(size_t i = 0; i < absolutePaths.size(); ++i)
        {
            addRigidBox(stage, absolutePaths[i].GetString(), linkDims, linkPos, GfQuatf(1.0f), i ? linkColor : rootColor, density);
            CHECK(stage->GetPrimAtPath(absolutePaths[i]));
            linkPos[0] += linkLength;
        }
    };

    auto SetupFixedJointToWorld = [&](const SdfPath& jointAbsPath, const SdfPath& rootLinkAbsPath, const bool setRootOnBody0Rel)
    {
        UsdPhysicsFixedJoint joint = UsdPhysicsFixedJoint::Define(stage, jointAbsPath);
        CHECK(joint);

        if(setRootOnBody0Rel)
        {
            joint.CreateBody0Rel().AddTarget(rootLinkAbsPath);
        }
        else
        {
            joint.CreateBody1Rel().AddTarget(rootLinkAbsPath);
        }
    };

    auto SetupRevoluteJoint = [&](const SdfPath& jointAbsPath, const SdfPath& parentLinkAbsPath, const SdfPath& childLinkAbsPath)
    {
        UsdPhysicsRevoluteJoint joint = UsdPhysicsRevoluteJoint::Define(stage, jointAbsPath);
        CHECK(joint);
        joint.CreateBody0Rel().SetTargets(SdfPathVector({ parentLinkAbsPath }));
        joint.CreateBody1Rel().SetTargets(SdfPathVector({ childLinkAbsPath }));
        joint.CreateAxisAttr().Set(TfToken("Y"));
        joint.CreateLocalPos0Attr().Set(GfVec3f(0.5f, 0.f, 0.f));
        joint.CreateLocalPos1Attr().Set(GfVec3f(-0.5f, 0.f, 0.f));
        joint.CreateLocalRot0Attr().Set(GfQuatf(1.0f));
        joint.CreateLocalRot1Attr().Set(GfQuatf(1.0f));
        return joint;
    };

    auto SetupRevoluteJointDrive = [&](const SdfPath& jointAbsPath, const float deflectionAngle)
    {
        UsdPhysicsRevoluteJoint joint = UsdPhysicsRevoluteJoint::Get(stage, jointAbsPath);
        CHECK(joint);
        const float gravityTorque = gravityMagnitude * linkLength * linkMass * 0.5f;
        const float stiffness = gravityTorque / deflectionAngle;
        const float damping = 0.1f * stiffness;

        UsdPhysicsDriveAPI drive = UsdPhysicsDriveAPI::Apply(joint.GetPrim(), TfToken("angular"));
        CHECK(drive);
        drive.CreateTypeAttr().Set(TfToken("force"));
        drive.CreateMaxForceAttr().Set(20e6f * forceScale);
        drive.CreateTargetPositionAttr().Set(0.f);
        drive.CreateDampingAttr().Set(damping);
        drive.CreateStiffnessAttr().Set(stiffness);
    };

    auto SetupSphericalJoint = [&](const SdfPath& jointAbsPath, const SdfPath& parentLinkAbsPath, const SdfPath& childLinkAbsPath)
    {
        UsdPhysicsSphericalJoint joint = UsdPhysicsSphericalJoint::Define(stage, jointAbsPath);
        CHECK(joint);
        joint.CreateBody0Rel().SetTargets(SdfPathVector({ parentLinkAbsPath }));
        joint.CreateBody1Rel().SetTargets(SdfPathVector({ childLinkAbsPath }));
        joint.CreateAxisAttr().Set(TfToken("X"));
        joint.CreateLocalPos0Attr().Set(GfVec3f(0.5f, 0.f, 0.f));
        joint.CreateLocalPos1Attr().Set(GfVec3f(-0.5f, 0.f, 0.f));
        joint.CreateLocalRot0Attr().Set(GfQuatf(1.0f));
        joint.CreateLocalRot1Attr().Set(GfQuatf(1.0f));
    };

    auto SetupD6SphericalJointDriver = [&](const SdfPath& jointAbsPath, const SdfPath& parentLinkAbsPath, const SdfPath& childLinkAbsPath, const float deflectionAngle)
    {
        UsdPhysicsJoint joint = UsdPhysicsJoint::Define(stage, jointAbsPath);
        CHECK(joint);
        joint.CreateBody0Rel().SetTargets(SdfPathVector({ parentLinkAbsPath }));
        joint.CreateBody1Rel().SetTargets(SdfPathVector({ childLinkAbsPath }));
        joint.CreateLocalPos0Attr().Set(GfVec3f(0.5f, 0.f, 0.f));
        joint.CreateLocalPos1Attr().Set(GfVec3f(-0.5f, 0.f, 0.f));
        joint.CreateLocalRot0Attr().Set(GfQuatf(1.0f));
        joint.CreateLocalRot1Attr().Set(GfQuatf(1.0f));
        joint.CreateExcludeFromArticulationAttr().Set(true);

        const float gravityTorque = gravityMagnitude * linkLength * linkMass * 0.5f;
        const float stiffness = gravityTorque / deflectionAngle;
        const float damping = 0.1f * stiffness;

        for(const auto& axis : { TfToken("rotY"), TfToken("rotZ") })
        {
            UsdPhysicsDriveAPI drive = UsdPhysicsDriveAPI::Apply(joint.GetPrim(), axis);
            CHECK(drive);
            drive.CreateTypeAttr().Set(TfToken("force"));
            drive.CreateMaxForceAttr().Set(20e6f * forceScale);
            drive.CreateTargetPositionAttr().Set(0.f);
            drive.CreateDampingAttr().Set(damping);
            drive.CreateStiffnessAttr().Set(stiffness);
        }
    };

    auto SetupD6FloatingBaseTranslationDriver = [&](const SdfPath& jointAbsPath, const SdfPath& xFormAnchorPath, const SdfPath& rootLinkPath, const float linearDeflection, const float totalMass)
    {
        UsdGeomXform anchor = UsdGeomXform::Define(stage, xFormAnchorPath);
        UsdPhysicsRigidBodyAPI rbAPI = UsdPhysicsRigidBodyAPI::Apply(anchor.GetPrim());
        rbAPI.CreateKinematicEnabledAttr().Set(true);
        UsdPhysicsJoint joint = UsdPhysicsJoint::Define(stage, jointAbsPath);
        CHECK(joint);
        joint.CreateBody0Rel().SetTargets(SdfPathVector({ xFormAnchorPath }));
        joint.CreateBody1Rel().SetTargets(SdfPathVector({ rootLinkPath }));
        joint.CreateLocalPos0Attr().Set(GfVec3f(0.0f, 0.f, 0.f));
        joint.CreateLocalPos1Attr().Set(GfVec3f(0.0f, 0.f, 0.f));
        joint.CreateLocalRot0Attr().Set(GfQuatf(1.0f));
        joint.CreateLocalRot1Attr().Set(GfQuatf(1.0f));
        joint.CreateExcludeFromArticulationAttr().Set(true);

        const float gravityForce = gravityMagnitude * totalMass;
        const float stiffness = gravityForce / linearDeflection;
        const float damping = 0.1f * stiffness;

        for(const auto& axis : { TfToken("transX"), TfToken("transY"), TfToken("transZ") })
        {
            UsdPhysicsDriveAPI drive = UsdPhysicsDriveAPI::Apply(joint.GetPrim(), axis);
            CHECK(drive);
            drive.CreateTypeAttr().Set(TfToken("force"));
            drive.CreateMaxForceAttr().Set(20e6f * forceScale);
            drive.CreateTargetPositionAttr().Set(0.f);
            drive.CreateDampingAttr().Set(damping);
            drive.CreateStiffnessAttr().Set(stiffness);
        }

        // lock rot DOFs:
        for(const auto& axis : { TfToken("rotX"), TfToken("rotY"), TfToken("rotZ") })
        {
            UsdPhysicsLimitAPI limit = UsdPhysicsLimitAPI::Apply(joint.GetPrim(), axis);
            CHECK(limit);
            limit.CreateHighAttr().Set(-1.0f);
            limit.CreateLowAttr().Set(1.0f);
        }

    };

    SUBCASE("Articulation RBs")
    {
        // setup RB links:
        std::vector<SdfPath> paths = { defaultPrimPath.AppendChild(TfToken("root")) , defaultPrimPath.AppendChild(TfToken("child")) };
        SetupRBLinksAtPaths(paths);

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        // check that links were created as RBs and check their weight
        for(const SdfPath& path : paths)
        {
            PxRigidDynamic* pxActor = getPhysxBaseDerivedFromPathChecked<PxRigidDynamic>(path, PhysXType::ePTActor);
            CHECK(std::abs(pxActor->getMass() - linkMass) < 0.001f);
        }
    }

    SUBCASE("Articulation Mass Updates")
    {
        SdfPath rootLinkPath = { defaultPrimPath.AppendChild(TfToken("root")) };
        SetupRBLinksAtPaths({ rootLinkPath });
        UsdPhysicsArticulationRootAPI::Apply(defaultPrim);
        SetupFixedJointToWorld(defaultPrimPath.AppendChild(TfToken("fixedJoint")), rootLinkPath, true);
        physxSim->attachStage(stageId);

        PxArticulationLink* pxActor =
            getPhysxBaseDerivedFromPathChecked<PxArticulationLink>(rootLinkPath, PhysXType::ePTLink);
        CHECK(std::abs(pxActor->getMass() - linkMass) < 0.001f);

        const float newLinkMass = 2.0f * linkMass;
        UsdPhysicsMassAPI::Get(stage, rootLinkPath).CreateMassAttr().Set(newLinkMass);

        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        CHECK(std::abs(pxActor->getMass() - newLinkMass) < 0.001f);
    }

    SUBCASE("Mark Parent XFORM dirty") // OM-82939
    {
        // In this test we create a parent transform with an ArticulationRootAPI and a child rigid body link with a
        // fixed joint. We check that the USD Notice is marking correctly links as dirty when the articulation root
        // is being moved. In this case the child link should still have its relative transformation unchanged
        const SdfPath articulationRootPath = defaultPrimPath.AppendChild(TfToken("articulationRoot"));
        UsdGeomXform articulationRoot = UsdGeomXform::Define(stage, articulationRootPath);

        SdfPath rootLinkPath = { articulationRootPath.AppendChild(TfToken("root")) };
        SetupRBLinksAtPaths({ rootLinkPath });

        UsdPhysicsArticulationRootAPI::Apply(articulationRoot.GetPrim());

        SetupFixedJointToWorld(articulationRootPath.AppendChild(TfToken("fixedJoint")), rootLinkPath, true);
        physxSim->attachStage(stageId);

        UsdGeomXformable rootLinkXForm = UsdGeomXformable::Get(stage, rootLinkPath);
        static const pxr::TfToken translateToken = pxr::UsdGeomXformOp::GetOpName(pxr::UsdGeomXformOp::TypeTranslate);
        CHECK(rootLinkXForm);
        pxr::GfVec3f startPos;
        const float epsilon = 0.01f;
        UsdPrim rootLinkPrim = stage->GetPrimAtPath(rootLinkPath);
        rootLinkPrim.GetAttribute(translateToken).Get(&startPos);
        CHECK(fabsf(startPos[1]) < epsilon);

        PxArticulationReducedCoordinate* pxArticulation =
            getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(
                articulationRootPath, PhysXType::ePTArticulation);
        bool isFixedBase = pxArticulation->getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
        CHECK(isFixedBase);

        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();
        PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
        CHECK(1u == pxScene->getNbArticulations());

        const float yoffset = 100.0f;
        CHECK(articulationRoot.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(GfVec3f(0.0f, yoffset, 0.0f)));

        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();
        physx::PxTransform rootPose = pxArticulation->getRootGlobalPose();
        const bool isEqualToYOffset =
            (fabsf(rootPose.p.y) < yoffset + epsilon) && (fabsf(rootPose.p.y) > yoffset - epsilon);
        CHECK(isEqualToYOffset);

        rootLinkPrim.GetAttribute(translateToken).Get(&startPos);
        CHECK(fabsf(startPos[1]) < epsilon);
    }

    SUBCASE("Multiple articulations with Fixed Joint") // OM-84752
    {
        // Test creates two articulations, each one made of two links, placed one after each other on X direction.
        // The first one has is root link fixed to the world.
        // The second one is connected to the second link of first one with a regular joint that is marked as excluded from articulation.
        // We want to test that updating joint local position will actually move the articulation (with correct scaling too)

        // Cteate First Articulation, with root link fixed to world
        const SdfPath articulation1RootPath = defaultPrimPath.AppendChild(TfToken("articulationRoot1"));
        UsdGeomXform articulationRoot1 = UsdGeomXform::Define(stage, articulation1RootPath);
        SdfPath rootLinkPath1 = { articulation1RootPath.AppendChild(TfToken("link0")) };
        SdfPath link1Path1 = { articulation1RootPath.AppendChild(TfToken("link1")) };
        SetupRBLinksAtPaths({ rootLinkPath1, link1Path1 });
        SetupRevoluteJoint(articulation1RootPath.AppendChild(TfToken("revoluteJoint")), rootLinkPath1, link1Path1);
        const SdfPath articulation1FixedJointPath = articulation1RootPath.AppendChild(TfToken("worldFixedJoint"));
        SetupFixedJointToWorld(articulation1FixedJointPath, rootLinkPath1, true);
        UsdPhysicsArticulationRootAPI::Apply(articulationRoot1.GetPrim());

        // Create Second Articulation, without any fixed joint
        const SdfPath articulation2RootPath = defaultPrimPath.AppendChild(TfToken("articulationRoot2"));
        UsdGeomXform articulationRoot2 = UsdGeomXform::Define(stage, articulation2RootPath);
        SdfPath rootLinkPath2 = { articulation2RootPath.AppendChild(TfToken("link0")) };
        SdfPath link1Path2 = { articulation2RootPath.AppendChild(TfToken("link1")) };
        SetupRBLinksAtPaths({ rootLinkPath2, link1Path2 }, GfVec3f(2.0f * linkLength, 0.0f, 0.0f));
        SetupRevoluteJoint(articulation2RootPath.AppendChild(TfToken("revoluteJoint")), rootLinkPath2, link1Path2);
        UsdPhysicsArticulationRootAPI::Apply(articulationRoot2.GetPrim());

        // Setup common fixed joint for both, excluded from articulation, because otherwise it can't be updated on GPU (OM-42711)
        const SdfPath fixedJointBetweenArticulationsPath = defaultPrimPath.AppendChild(TfToken("articulationsJoint"));
        UsdPhysicsFixedJoint fixedJointBetweenArticulations = UsdPhysicsFixedJoint::Define(stage, fixedJointBetweenArticulationsPath);
        CHECK(fixedJointBetweenArticulations);
        fixedJointBetweenArticulations.CreateExcludeFromArticulationAttr().Set(true);
        fixedJointBetweenArticulations.CreateBody0Rel().AddTarget(link1Path1);
        fixedJointBetweenArticulations.CreateBody1Rel().AddTarget(rootLinkPath2);

        fixedJointBetweenArticulations.CreateLocalPos0Attr().Set(pxr::GfVec3f(0.5f, 0.0f, 0.0f));
        fixedJointBetweenArticulations.CreateLocalPos1Attr().Set(pxr::GfVec3f(-0.5f, 0.0f, 0.0f));

        fixedJointBetweenArticulations.CreateLocalRot0Attr().Set(GfQuatf(1.0f));
        fixedJointBetweenArticulations.CreateLocalRot1Attr().Set(GfQuatf(1.0f));

        scene.GetGravityMagnitudeAttr().Set(0.0f); // disable gravity to check the change in fixed joint location more easily

        physxSim->attachStage(stageId);

        const float epsilon = 0.1f;

        // Obtain both articulations
        PxArticulationReducedCoordinate* pxArticulation1 = getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(articulation1RootPath, PhysXType::ePTArticulation);
        PxArticulationReducedCoordinate* pxArticulation2 = getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(articulation2RootPath, PhysXType::ePTArticulation);

        CHECK(pxArticulation1);
        CHECK(pxArticulation2);
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        {
            // check that one articulation root pose to be at the origin and the other at 2*linkLength along X
            PxTransform articulation1GlobalPose =  pxArticulation1->getRootGlobalPose();
            const float articulation1MoveError = fabsf(articulation1GlobalPose.p.x - 0.0f);
            CHECK(articulation1MoveError < epsilon);
            PxTransform articulation2GlobalPose =  pxArticulation2->getRootGlobalPose();
            const float articulation2MoveError = fabsf(articulation2GlobalPose.p.x - 2 * linkLength );
            CHECK(articulation2MoveError < epsilon);
        }

        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // Now we offset localpose 0 by 1 unit, that will offset the second articulation by one more linkLength (so total 3 * linkLength)
        fixedJointBetweenArticulations.GetLocalPos0Attr().Set(pxr::GfVec3f(1.5f, 0.0f, 0.0f));
        fixedJointBetweenArticulations.GetLocalPos1Attr().Set(pxr::GfVec3f(-0.5f, 0.0f, 0.0f));
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();
        {
            PxTransform articulation2GlobalPose =  pxArticulation2->getRootGlobalPose();
            const float articulationMoveError = fabsf(articulation2GlobalPose.p.x - 3 * linkLength);
            CHECK(articulationMoveError < epsilon);    
        }
    }
    // according to the ArticulationRootAPI doc, we can apply the API to either:
    // - a direct or indirect parent of the fixed joint from world->root link
    // - on the joint itself
    // in order to obtain a fixed-base articulation
    // (all other cases should result in a floating-base articulation
    // we can check this easily by querying the Px articulation fixed-base flag)
    SUBCASE("Fixed-base articulation parsing")
    {
        SUBCASE("API on direct USD parent")
        {
            // setup just the root link:
            SdfPath rootLinkPath = { defaultPrimPath.AppendChild(TfToken("root")) };
            SetupRBLinksAtPaths({ rootLinkPath });

            // apply articulation API to root prim
            UsdPhysicsArticulationRootAPI::Apply(defaultPrim);

            SUBCASE("Fixed joint rel variations")
            {
                SUBCASE("Rel on 0")
                {
                    SetupFixedJointToWorld(defaultPrimPath.AppendChild(TfToken("fixedJoint")), rootLinkPath, true);
                }
                SUBCASE("Rel on 1")
                {
                    SetupFixedJointToWorld(defaultPrimPath.AppendChild(TfToken("fixedJoint")), rootLinkPath, false);
                }
                physxSim->attachStage(stageId);

                PxArticulationReducedCoordinate* pxArticulation = getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(defaultPrimPath, PhysXType::ePTArticulation);
                bool isFixedBase = pxArticulation->getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
                CHECK(isFixedBase);

                // put them into the scene to check the articulation count:
                physxSim->simulate(0.01f, 0.0f);
                physxSim->fetchResults();
                // should result in two fixed-base articulations, one for each root:
                PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
                CHECK(1u == pxScene->getNbArticulations());
            }

            SUBCASE("Multiple candidate roots")
            {
                // from here always setup fixed joint:
                SetupFixedJointToWorld(defaultPrimPath.AppendChild(TfToken("fixedJoint")), rootLinkPath, true);

                // setup additional root link:
                SdfPath anotherRootPath = { defaultPrimPath.AppendChild(TfToken("anotherRoot")) };
                SetupRBLinksAtPaths({ anotherRootPath }, GfVec3f(0.0f, 3.0f * linkWidth, 0.0f));
                SetupFixedJointToWorld(defaultPrimPath.AppendChild(TfToken("anotherFixedJoint")), anotherRootPath, true);

                physxSim->attachStage(stageId);

                // put them into the scene to check the articulation count:
                physxSim->simulate(0.01f, 0.0f);
                physxSim->fetchResults();
                // should result in two fixed-base articulations, one for each root:
                PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
                REQUIRE(pxScene);
                CHECK(2u == pxScene->getNbArticulations());
            }

            SUBCASE("Switch Fixed To Floating")
            {
                // from here always setup fixed joint:
                SetupFixedJointToWorld(defaultPrimPath.AppendChild(TfToken("fixedJoint")), rootLinkPath, true);

                physxSim->attachStage(stageId);

                // put them into the scene to check the articulation count:
                physxSim->simulate(0.01f, 0.0f);
                physxSim->fetchResults();
                // should result in two fixed-base articulations, one for each root:
                PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
                REQUIRE(pxScene);
                CHECK(1u == pxScene->getNbArticulations());

                {
                    PxArticulationReducedCoordinate* pxArticulation = getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(defaultPrimPath, PhysXType::ePTArticulation);
                    bool isFixedBase = pxArticulation->getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
                    CHECK(isFixedBase);
                }

                // Disable the fixed joint, will make the articulation floating
                UsdPhysicsJoint fixedJoint(stage->GetPrimAtPath(defaultPrimPath.AppendChild(TfToken("fixedJoint"))));
                REQUIRE(fixedJoint);
                fixedJoint.GetJointEnabledAttr().Set(false);
                physxSim->simulate(0.01f, 0.0f);
                physxSim->fetchResults();

                {
                    PxArticulationReducedCoordinate* pxArticulation = getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(defaultPrimPath, PhysXType::ePTArticulation);
                    bool isFixedBase = pxArticulation->getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
                    CHECK(!isFixedBase);
                }
            }

        }
        // OM-26562
        SUBCASE("API on fixed joint")
        {
            // setup just the root link:
            SdfPath rootLinkPath = { defaultPrimPath.AppendChild(TfToken("root")) };
            SetupRBLinksAtPaths({ rootLinkPath });

            const SdfPath fixedJointPath = defaultPrimPath.AppendChild(TfToken("fixedJoint"));
            SUBCASE("Rel on 0")
            {
                SetupFixedJointToWorld(fixedJointPath, rootLinkPath, true);
            }
            SUBCASE("Rel on 1")
            {
                SetupFixedJointToWorld(fixedJointPath, rootLinkPath, false);
            }

            UsdPhysicsArticulationRootAPI::Apply(stage->GetPrimAtPath(fixedJointPath));

            // parse
            physxSim->attachStage(stageId);

            // stage->Export("fixedOnJoint.usda");

            PxArticulationReducedCoordinate* pxArticulation = getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(fixedJointPath, PhysXType::ePTArticulation);
            bool isFixedBase = pxArticulation->getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
            CHECK(isFixedBase);

            // put them into the scene to check the articulation count:
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();
            PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
            CHECK(1u == pxScene->getNbArticulations());
        }

        // OM-31392
        SUBCASE("Fixed joint location")
        {
            // setup RB links:
            SdfPath rootpath = defaultPrimPath;
            SdfPath fixedJointPath;
            SUBCASE("As parent")
            {
                fixedJointPath = defaultPrimPath.AppendChild(TfToken("fixedJoint"));
                rootpath = fixedJointPath;
                SetupFixedJointToWorld(rootpath, rootpath.AppendChild(TfToken("root")), true);
            }
            const SdfPath rootLinkPath = rootpath.AppendChild(TfToken("root"));
            const SdfPath childLinkPath = rootpath.AppendChild(TfToken("child"));
            std::vector<SdfPath> paths = { rootLinkPath, childLinkPath };
            SetupRBLinksAtPaths(paths);
            SetupRevoluteJoint(childLinkPath.AppendChild(TfToken("revoluteJoint")), rootLinkPath, childLinkPath);

            SUBCASE("As Sibling")
            {
                fixedJointPath = defaultPrimPath.AppendChild(TfToken("fixedJoint"));
                SetupFixedJointToWorld(fixedJointPath, rootLinkPath, true);
            }
            SUBCASE("As Child")
            {
                fixedJointPath = rootLinkPath.AppendChild(TfToken("fixedJoint"));
                SetupFixedJointToWorld(fixedJointPath, rootLinkPath, true);
            }

            UsdPhysicsArticulationRootAPI::Apply(stage->GetPrimAtPath(fixedJointPath));

             // parse
            physxSim->attachStage(stageId);

            PxArticulationReducedCoordinate* pxArticulation = getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(fixedJointPath, PhysXType::ePTArticulation);
            bool isFixedBase = pxArticulation->getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
            CHECK(isFixedBase);

            // check link count:
            CHECK(2u == pxArticulation->getNbLinks());

            // put them into the scene to check the articulation count:
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();
            PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
            CHECK(1u == pxScene->getNbArticulations());
        }

        SUBCASE("Link Cfm scale")
        {
            // setup RB links:
            SdfPath rootpath = defaultPrimPath;
            SdfPath fixedJointPath;
            fixedJointPath = defaultPrimPath.AppendChild(TfToken("fixedJoint"));
            rootpath = fixedJointPath;
            SetupFixedJointToWorld(rootpath, rootpath.AppendChild(TfToken("root")), true);
            const SdfPath rootLinkPath = rootpath.AppendChild(TfToken("root"));
            const SdfPath childLinkPath = rootpath.AppendChild(TfToken("child"));
            std::vector<SdfPath> paths = { rootLinkPath, childLinkPath };
            SetupRBLinksAtPaths(paths);
            SetupRevoluteJoint(childLinkPath.AppendChild(TfToken("revoluteJoint")), rootLinkPath, childLinkPath);

            UsdPhysicsArticulationRootAPI::Apply(stage->GetPrimAtPath(fixedJointPath));

            const float epsilon = 1e-05f;

            // parse
            {
                physxSim->attachStage(stageId);

                PxArticulationReducedCoordinate* pxArticulation = getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(fixedJointPath, PhysXType::ePTArticulation);

                // check link count:
                CHECK(2u == pxArticulation->getNbLinks());

                // put them into the scene to check the articulation count:
                physxSim->simulate(0.01f, 0.0f);
                physxSim->fetchResults();
                PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
                CHECK(1u == pxScene->getNbArticulations());

                PxArticulationLink* link = nullptr;
                for (int i = 0; i < 2; i++)
                {
                    pxArticulation->getLinks(&link, 1, i);
                    CHECK(fabsf(link->getCfmScale() - 0.025f) < epsilon);
                }                
            }

            {
                for (const SdfPath& rboPath : paths)
                {
                    UsdPrim rboPrim = stage->GetPrimAtPath(rboPath);
                    PhysxSchemaPhysxRigidBodyAPI rboAPI = PhysxSchemaPhysxRigidBodyAPI::Apply(rboPrim);
                    rboAPI.GetCfmScaleAttr().Set(0.5f);
                }

                physxSim->simulate(0.01f, 0.0f);
                physxSim->fetchResults();

                PxArticulationReducedCoordinate* pxArticulation = getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(fixedJointPath, PhysXType::ePTArticulation);
                // check link count:
                CHECK(2u == pxArticulation->getNbLinks());

                PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
                CHECK(1u == pxScene->getNbArticulations());

                PxArticulationLink* link = nullptr;
                for (int i = 0; i < 2; i++)
                {
                    pxArticulation->getLinks(&link, 1, i);
                    CHECK(fabsf(link->getCfmScale() - 0.5f) < epsilon);
                }

            }
        }

    }

    SUBCASE("Floating articulation parsing")
    {
        SUBCASE("Articulation Root Center of Graph")
        {
            // setup RB links:
            const SdfPath topXformPath = defaultPrimPath.AppendChild(TfToken("xform"));
            UsdGeomXform topXform = UsdGeomXform::Define(stage, topXformPath);
            const SdfPath body0Path = topXformPath.AppendChild(TfToken("body0"));
            const SdfPath body1Path = topXformPath.AppendChild(TfToken("body1"));
            const SdfPath body2Path = topXformPath.AppendChild(TfToken("body2"));
            const SdfPath driveBodyPath = topXformPath.AppendChild(TfToken("driveBody"));
            std::vector<SdfPath> paths = { body0Path, body1Path, body2Path, driveBodyPath };
            SetupRBLinksAtPaths(paths);

            SetupRevoluteJoint(topXformPath.AppendChild(TfToken("revoluteJoint0")), body0Path, body1Path);
            SetupRevoluteJoint(topXformPath.AppendChild(TfToken("revoluteJoint1")), body1Path, body2Path);
            UsdPhysicsJoint driveJoint = SetupRevoluteJoint(topXformPath.AppendChild(TfToken("revoluteJointDrive")), body2Path, driveBodyPath);
            driveJoint.GetExcludeFromArticulationAttr().Set(true);

            UsdPhysicsArticulationRootAPI::Apply(topXform.GetPrim());

            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
            physxSim->fetchResults();

            PxArticulationReducedCoordinate* art = getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(topXformPath, PhysXType::ePTArticulation);
            REQUIRE(art);
            CHECK(art->getNbLinks() == 3);

            PxArticulationLink* pxLink = getPhysxBaseDerivedFromPathChecked<PxArticulationLink>(body1Path, PhysXType::ePTLink);
            REQUIRE(pxLink);            
            PxArticulationLink* rootLink = nullptr;
            art->getLinks(&rootLink, 1);
            CHECK(pxLink == rootLink);
        }

        // OM-26672
        // Repro of issue with skeleton hand demo where hand is moved by D6 between kinematic Xform and floating base articulation
        SUBCASE("Floating-base articulation driven by D6")
        {
            // Vertical offset
            const float zOffset = 50.0f; // <-- This is the issue. Set to 0 and it will pass.

            // setup root link RB:
            const SdfPath rootLinkPath = defaultPrimPath.AppendChild(TfToken("root"));
            SetupRBLinksAtPaths({ rootLinkPath });
            UsdPhysicsArticulationRootAPI::Apply(stage->GetPrimAtPath(rootLinkPath));

            const float linearDeflection = linkWidth; // how much the link should sag relative to the anchor
            const SdfPath xformAnchorPath = defaultPrimPath.AppendChild(TfToken("XformAnchor"));
            const SdfPath d6DriverPath = defaultPrimPath.AppendChild(TfToken("D6driver"));
            SetupD6FloatingBaseTranslationDriver(d6DriverPath, xformAnchorPath, rootLinkPath, linearDeflection, linkMass);

            // move anchor and base to offset:
            setPhysicsPrimPos(stage, rootLinkPath, GfVec3f(0.0f, 0.0f, zOffset));
            UsdGeomXform anchor = UsdGeomXform::Get(stage, xformAnchorPath);
            anchor.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(GfVec3f(0.0f, 0.0f, zOffset));

            // Debug USDA:
            //stage->Export("D6LinearJointToCube.usda");

            physxSim->attachStage(stageId);

            for(int u = 0; u < 100; ++u)
            {
                physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
                physxSim->fetchResults();
            }

            PxArticulationLink* pxLink = getPhysxBaseDerivedFromPathChecked<PxArticulationLink>(rootLinkPath, PhysXType::ePTLink);
            PxTransform linkTransform = pxLink->getGlobalPose();
            const float expectedPosition = zOffset - linearDeflection;
            CHECK(std::abs(linkTransform.p.z - expectedPosition) < positionToleranceCM);
        }

        // OM-91198 - excluded joint included into articulation topology
        SUBCASE("Floating-base articulation joint exclude")
        {
            // setup root link RB:
            const SdfPath rootLinkPath = defaultPrimPath.AppendChild(TfToken("root"));
            SetupRBLinksAtPaths({ rootLinkPath });
            SetupFixedJointToWorld(defaultPrimPath.AppendChild(TfToken("fixedJoint")), rootLinkPath, true);
            UsdPhysicsJoint joint = UsdPhysicsJoint::Get(stage, defaultPrimPath.AppendChild(TfToken("fixedJoint")));
            CHECK(joint);
            joint.CreateExcludeFromArticulationAttr().Set(true);
            UsdPhysicsArticulationRootAPI::Apply(stage->GetPrimAtPath(defaultPrimPath));

            physxSim->attachStage(stageId);

            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
            physxSim->fetchResults();

            PxArticulationLink* pxLink = getPhysxBaseDerivedFromPathChecked<PxArticulationLink>(rootLinkPath, PhysXType::ePTLink);
            REQUIRE(pxLink);
            CHECK((pxLink->getArticulation().getArticulationFlags() & PxArticulationFlag::eFIX_BASE) == false);
        }
    }

    SUBCASE("Articulation Drives")
    {
        // setup RB links:
        const SdfPath rootLinkPath = defaultPrimPath.AppendChild(TfToken("root"));
        const SdfPath childLinkPath = defaultPrimPath.AppendChild(TfToken("child"));
        std::vector<SdfPath> paths = { rootLinkPath, childLinkPath };
        SetupRBLinksAtPaths(paths);
        const SdfPath fixedJointPath = defaultPrimPath.AppendChild(TfToken("fixedJoint"));
        SetupFixedJointToWorld(fixedJointPath, rootLinkPath, true);
        UsdPhysicsArticulationRootAPI::Apply(defaultPrim);

        const float deflectionAngleDeg = 5.0f;

        // Repro of maximal-coordinate D6 driver on spherical joint that test that is flaky on Linux
        SUBCASE("Articulation Spherical Joint Driven By D6 External Joint")
        {
            SetupSphericalJoint(paths[1].AppendChild(TfToken("SphericalJoint")), rootLinkPath, childLinkPath);
            const SdfPath d6DriverPath = defaultPrimPath.AppendChild(TfToken("D6DriverJoint"));
            SetupD6SphericalJointDriver(d6DriverPath, rootLinkPath, childLinkPath, deflectionAngleDeg);

#if USE_PHYSX_GPU
            SUBCASE("GPU")
            {
                PhysxSchemaPhysxSceneAPI physxSceneAPI = PhysxSchemaPhysxSceneAPI::Apply(scene.GetPrim());
                physxSceneAPI.CreateEnableGPUDynamicsAttr().Set(true);
                physxSim->attachStage(stageId);
                PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
                const bool isGPUsim = pxScene->getFlags() & PxSceneFlag::eENABLE_GPU_DYNAMICS;
                CHECK(isGPUsim);
            }
#endif // USE_PHYSX_GPU
            SUBCASE("CPU")
            {
                PhysxSchemaPhysxSceneAPI physxSceneAPI = PhysxSchemaPhysxSceneAPI::Apply(scene.GetPrim());
                physxSceneAPI.CreateEnableGPUDynamicsAttr().Set(false);
                physxSim->attachStage(stageId);
                PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
                const bool isGPUsim = pxScene->getFlags() & PxSceneFlag::eENABLE_GPU_DYNAMICS;
                CHECK(!isGPUsim);
            }

            // get D6 joint and drives:
            const UsdPhysicsJoint joint = UsdPhysicsJoint::Get(stage, d6DriverPath);
            const UsdPhysicsDriveAPI driveY = UsdPhysicsDriveAPI::Get(joint.GetPrim(), TfToken("rotY"));
            // z-drive has same config

            // get USD parameters:
            const float rad2Deg = 180.0f / PxPi;
            float stiffnessDegUSD = 0.0f;
            CHECK(driveY.GetStiffnessAttr().Get(&stiffnessDegUSD));
            float dampingDegUSD = 0.0f;
            CHECK(driveY.GetDampingAttr().Get(&dampingDegUSD));
            const float stiffnessRadUSD = stiffnessDegUSD * rad2Deg; // stiffness is Nm/rad
            const float dampingRadUSD = dampingDegUSD * rad2Deg;

            // check parsed result:
            PxD6Joint* d6driver = getPhysxBaseDerivedFromPathChecked<PxD6Joint>(d6DriverPath, PhysXType::ePTJoint);
            for(const PxD6Axis::Enum axis : {PxD6Axis::eSWING1, PxD6Axis::eSWING2, PxD6Axis::eTWIST, PxD6Axis::eX, PxD6Axis::eY, PxD6Axis::eZ})
            {
                CHECK(d6driver->getMotion(axis) == PxD6Motion::eFREE);
            }

            PxD6Drive::Enum swingList[] = { PxD6Drive::eSWING1, PxD6Drive::eSWING2 };
            for (PxU32 k = 0; k < (sizeof(swingList) / sizeof(swingList[0])); k++)
            {
                PxD6JointDrive drive = d6driver->getDrive(swingList[k]);
                const float driveParamTol = 1.0e-3f;
                CHECK(std::abs(drive.damping - dampingRadUSD) < driveParamTol);
                CHECK(std::abs(drive.stiffness - stiffnessRadUSD) < driveParamTol);
                CHECK(PxTransform(PxIdentity) == d6driver->getDrivePosition());
            }

            PxArticulationReducedCoordinate* art = getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(defaultPrim.GetPath(), PhysXType::ePTArticulation);

            CHECK(paths.size() == 2u);
            PxArticulationLink* links[2u] = { nullptr };
            art->getLinks(links, 2u, 0u);

            PxArticulationJointReducedCoordinate* sphericalJoint = static_cast<PxArticulationJointReducedCoordinate*>(links[1]->getInboundJoint());
            CHECK(sphericalJoint->getJointType() == PxArticulationJointType::eSPHERICAL);
            CHECK(sphericalJoint->getMotion(PxArticulationAxis::eSWING1) == PxArticulationMotion::eFREE);
            CHECK(sphericalJoint->getMotion(PxArticulationAxis::eSWING2) == PxArticulationMotion::eFREE);

        }

        SUBCASE("Revolute drive")
        {
            const SdfPath revoluteJointPath = paths[1].AppendChild(TfToken("SphericalJoint"));
            SetupRevoluteJoint(revoluteJointPath, rootLinkPath, childLinkPath);
            SetupRevoluteJointDrive(revoluteJointPath, deflectionAngleDeg);
            physxSim->attachStage(stageId);
        }

        for(int u = 0; u < 50; ++u)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        PxArticulationLink* pxLink = getPhysxBaseDerivedFromPathChecked<PxArticulationLink>(childLinkPath, PhysXType::ePTLink);
        PxTransform linkTransform = pxLink->getGlobalPose();

        const float deflectionAngleRad = deflectionAngleDeg * PxPi / 180.0f;
        CHECK(std::abs(linkTransform.q.getAngle() - deflectionAngleRad) < angleToleranceRad);
    }

    // regression test for PX-4075
    SUBCASE("Initial penetration")
    {
        std::string usdFileName = physicsTests.getUnitTestsDataDirectory() + "articulation_initial_penetration.usd";

        UsdStageRefPtr stage = UsdStage::Open(usdFileName);
        REQUIRE(stage);

        pxr::UsdUtilsStageCache::Get().Insert(stage);
        long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

        physxSim->attachStage(stageId);

        std::vector<PxArticulationLink*> links;
        pxr::UsdPrimRange range = stage->Traverse();
        for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
        {
            const pxr::UsdPrim& prim = *iter;
            if (!prim)
                continue;
            if (prim.HasAPI<pxr::UsdPhysicsArticulationRootAPI>())
            {
                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prim.GetPrimPath(), ePTArticulation));
                REQUIRE(basePtr != nullptr);
                REQUIRE(basePtr->is<PxArticulationReducedCoordinate>());

                const PxArticulationReducedCoordinate* articulation = basePtr->is<PxArticulationReducedCoordinate>();
                const uint32_t numLinks = articulation->getNbLinks();

                size_t existingSize = links.size();
                links.resize(existingSize + size_t(numLinks));

                articulation->getLinks(&links[existingSize], numLinks);
            
                iter.PruneChildren();
            }
        }

        // run the sim
        for (size_t i = 0; i < 20; ++i)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
            physxSim->fetchResults();

            // get the link transforms
            for (size_t j = 0; j < links.size(); ++j)
            {
                // check not NAN
                REQUIRE(links[j]->getGlobalPose().isFinite());
            }

        }
    }

    SUBCASE("Max Number Of Shapes")
    {
        // setup root link RB:
        const SdfPath rootLinkPath = defaultPrimPath.AppendChild(TfToken("root"));
        UsdGeomXform xform = UsdGeomXform::Define(stage, rootLinkPath);
        UsdPhysicsRigidBodyAPI::Apply(stage->GetPrimAtPath(rootLinkPath));
        UsdPhysicsArticulationRootAPI::Apply(stage->GetPrimAtPath(defaultPrimPath));

        for (uint32_t i = 0; i < 2048; i++)
        {
            const SdfPath cubePath = rootLinkPath.AppendChild(TfToken("cube" + std::to_string(i)));
            UsdGeomCube cube = UsdGeomCube::Define(stage, cubePath);
            UsdPhysicsCollisionAPI::Apply(cube.GetPrim());
            cube.AddTranslateOp().Set(GfVec3d(0.0, i * 100.0, 0.0));
        }

        const SdfPath dynLinkPath = defaultPrimPath.AppendChild(TfToken("dynLink"));
        UsdGeomCube cube = UsdGeomCube::Define(stage, dynLinkPath);
        UsdPhysicsCollisionAPI::Apply(cube.GetPrim());
        UsdPhysicsRigidBodyAPI::Apply(cube.GetPrim());

        const SdfPath revJointPath = defaultPrimPath.AppendChild(TfToken("revJoint"));
        UsdPhysicsRevoluteJoint revJoint = UsdPhysicsRevoluteJoint::Define(stage, revJointPath);
        revJoint.CreateBody0Rel().AddTarget(rootLinkPath);
        revJoint.CreateBody1Rel().AddTarget(dynLinkPath);

        // The application needs to increase PxGpuDynamicsMemoryConfig::foundLostAggregatePairsCapacity to 2048, otherwise, the simulation will miss interactions

        physxSim->attachStage(stageId);

        physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();

        PxArticulationLink* pxLink = getPhysxBaseDerivedFromPathChecked<PxArticulationLink>(rootLinkPath, PhysXType::ePTLink);
        REQUIRE(pxLink);
        CHECK((pxLink->getArticulation().getArticulationFlags() & PxArticulationFlag::eFIX_BASE) == false);
        CHECK(pxLink->getScene() != nullptr);
    }

    SUBCASE("PrismaticJoint")
    {
#if USE_PHYSX_GPU
        SUBCASE("GPU")
        {
            PhysxSchemaPhysxSceneAPI physxSceneAPI = PhysxSchemaPhysxSceneAPI::Apply(scene.GetPrim());
            physxSceneAPI.CreateEnableGPUDynamicsAttr().Set(true);
            physxSim->attachStage(stageId);
            PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
            const bool isGPUsim = pxScene->getFlags() & PxSceneFlag::eENABLE_GPU_DYNAMICS;
            CHECK(isGPUsim);
        }
#endif // USE_PHYSX_GPU
        SUBCASE("CPU")
        {
            PhysxSchemaPhysxSceneAPI physxSceneAPI = PhysxSchemaPhysxSceneAPI::Apply(scene.GetPrim());
            physxSceneAPI.CreateEnableGPUDynamicsAttr().Set(false);
            physxSim->attachStage(stageId);
            PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
            const bool isGPUsim = pxScene->getFlags() & PxSceneFlag::eENABLE_GPU_DYNAMICS;
            CHECK(!isGPUsim);
        }

        std::string usdFileName = physicsTests.getUnitTestsDataDirectory() + "ArticulationWithPrismaticJoint.usda";

        UsdStageRefPtr stage = UsdStage::Open(usdFileName);
        REQUIRE(stage);

        pxr::UsdUtilsStageCache::Get().Insert(stage);
        long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

        physxSim->attachStage(stageId);

        const SdfPath bodyPath = defaultPrimPath.AppendChild(TfToken("Fix"));
        PxArticulationReducedCoordinate* actor = getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(bodyPath, PhysXType::ePTArticulation);
        PxReal epsilon = 1e-4f;

        // run the sim
        for (int i = 0; i < 20; ++i)
        {            
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
            physxSim->fetchResults();

            PxTransform pose = actor->getRootGlobalPose();
            
            CHECK(pose.q.w > 1.0f - epsilon);
            CHECK(pose.q.x < epsilon);
            CHECK(pose.q.y < epsilon);
            CHECK(pose.q.z < epsilon);

            //printf("iteration %i   %f\n", i, pose.q.w);
        }
    }


    SUBCASE("ArticulationLoopClosure")
    {
        std::string usdFileName = physicsTests.getUnitTestsDataDirectory() + "ArticulationClosedLoop.usda";

        UsdStageRefPtr stage = UsdStage::Open(usdFileName);
        REQUIRE(stage);

        pxr::UsdUtilsStageCache::Get().Insert(stage);
        long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

        physxSim->attachStage(stageId);


        const SdfPath linkBPath = defaultPrimPath.AppendChild(TfToken("B"));
        const SdfPath linkCPath = defaultPrimPath.AppendChild(TfToken("C"));
        const SdfPath jointPath = defaultPrimPath.AppendChild(TfToken("C")).AppendChild(TfToken("RevoluteJoint0"));

        std::string debug1 = linkBPath.GetAsString();
        std::string debug2 = linkCPath.GetAsString();

        // run the sim
        for (int i = 0; i < 30; ++i)
        {
            physxSim->simulate(1.0f / 60.0f, 0.0f);
            physxSim->fetchResults();

            PxArticulationLink* bLink = getPhysxBaseDerivedFromPathChecked<PxArticulationLink>(linkBPath, PhysXType::ePTLink);
            PxTransform bTransform = bLink->getGlobalPose();

            PxArticulationLink* cLink = getPhysxBaseDerivedFromPathChecked<PxArticulationLink>(linkCPath, PhysXType::ePTLink);
            PxTransform cTransform = cLink->getGlobalPose();

            PxD6Joint* joint = getPhysxBaseDerivedFromPathChecked<PxD6Joint>(jointPath, PhysXType::ePTJoint);

            PxTransform local0 = joint->getLocalPose(PxJointActorIndex::eACTOR0);
            PxTransform local1 = joint->getLocalPose(PxJointActorIndex::eACTOR1);

            PxVec3 bJointPos = bTransform.transform(local0).p;
            PxVec3 cJointPos = cTransform.transform(local1).p;

            PxReal distance = (cJointPos - bJointPos).magnitude();

            CHECK(distance < 0.25f);

            //printf("It: %i   Joint error: %f\n", i, distance);
        }
    }


    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}


void testLinkConsistency(const std::string& usdFileName, IPhysx* physx, IPhysxSimulation* physxSim)
{
    UsdStageRefPtr stage = UsdStage::Open(usdFileName);
    REQUIRE(stage);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);

    std::vector<PxArticulationLink*> linksBase;
    std::vector<PxArticulationLink*> links;
    pxr::UsdPrimRange range = stage->Traverse();
    for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
    {
        const pxr::UsdPrim& prim = *iter;
        if (!prim)
            continue;
        if (prim.HasAPI<pxr::UsdPhysicsArticulationRootAPI>())
        {
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prim.GetPrimPath(), ePTArticulation));
            REQUIRE(basePtr != nullptr);
            REQUIRE(basePtr->is<PxArticulationReducedCoordinate>());

            const PxArticulationReducedCoordinate* articulation = basePtr->is<PxArticulationReducedCoordinate>();
            const uint32_t numLinks = articulation->getNbLinks();

            if (links.empty())
            {
                links.resize(size_t(numLinks));
                linksBase.resize(size_t(numLinks));

                articulation->getLinks(linksBase.data(), numLinks);
            }
            else
            {
                CHECK(size_t(numLinks) == linksBase.size());
                articulation->getLinks(links.data(), numLinks);
                for (size_t i = 0; i < links.size(); i++)
                {
                    const char* linkName = links[i]->getName();
                    const char* linkBaseName = linksBase[i]->getName();

                    // cut the first two levels
                    for (int i = 0; i < 4; i++)
                    {
                        linkName = strstr(linkName, "/");
                        linkBaseName = strstr(linkBaseName, "/");
                        linkName++;
                        linkBaseName++;
                    }
                    CHECK(strcmp(linkName, linkBaseName) == 0);
                }
            }
            iter.PruneChildren();
        }
    }

    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;

}

TEST_CASE("Articulations Tests Consistency",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    std::string usdFileName = physicsTests.getUnitTestsDataDirectory() + "scene_assertion_error.usd";

    SUBCASE("Articulation links consistency Franka")
    {
        std::string usdFileName = physicsTests.getUnitTestsDataDirectory() + "franka_ordering_repro.usd";
        testLinkConsistency(usdFileName, physx, physxSim); 
    }

    SUBCASE("Articulation links consistency Floating")
    {
        std::string usdFileName = physicsTests.getUnitTestsDataDirectory() + "floating_articulation_ordering_repro.usd";
        testLinkConsistency(usdFileName, physx, physxSim);
    }

    SUBCASE("Articulation links consistency Floating - Balance bot")
    {
        std::string usdFileName = physicsTests.getUnitTestsDataDirectory() + "articulation_test.usda";
        testLinkConsistency(usdFileName, physx, physxSim);
    }
}
