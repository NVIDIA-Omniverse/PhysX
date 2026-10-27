// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"
#include "../../common/PhysicsChangeTemplate.h"

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>
#include <physicsSchemaTools/physicsSchemaTokens.h>

#include "PhysicsTools.h"

using namespace PXR_NS;
using namespace omni::physx;
using namespace ::physx;

#if defined(NEWTON_SCHEMA_PLUGIN_PATH)

// NewtonMimicAPI is emulated on top of the PhysX articulation mimic joint. The Newton constraint is
//     q_follower = coef0 + coef1 * q_leader
// while PhysX enforces
//     q_A + gearing * q_B + offset = 0     (A = follower/target, B = leader/reference)
// so the parser maps gearing = -coef1, offset = -coef0.
//
// omni.physx flips a joint's coordinate (negates limits / drive target / joint position) when the
// joint's body0/body1 rel does not follow the articulation hierarchy (InternalJoint::mBody0IsParentLink
// == false). The Newton coefficients are authored in the joint's natural USD convention, so when the
// follower and leader joints have different flip parity the mapping must additionally flip the sign.
// These tests author a follower (or leader) joint with reversed body order and verify that the follower
// still physically tracks the leader, which is invariant under a body0/body1 swap.

namespace
{
static const TfToken kMimicCoef0("newton:mimicCoef0");
static const TfToken kMimicCoef1("newton:mimicCoef1");
static const TfToken kMimicEnabled("newton:mimicEnabled");
static const TfToken kMimicJoint("newton:mimicJoint");
static const TfToken kNewtonMimicAPI("NewtonMimicAPI");

// PhysX-convention joint position along the single motion axis. omni.physx arranges joint frames so
// the motion axis is eX for a prismatic joint and the twist axis for a revolute joint.
float jointPos(IPhysx* physx, const SdfPath& jointPath, bool revolute)
{
    PxBase* base = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTLinkJoint));
    REQUIRE(base != nullptr);
    PxArticulationJointReducedCoordinate* j = base->is<PxArticulationJointReducedCoordinate>();
    REQUIRE(j != nullptr);
    return j->getJointPosition(revolute ? PxArticulationAxis::eTWIST : PxArticulationAxis::eX);
}

void makeRigidLink(UsdStageRefPtr stage, const SdfPath& path, const GfVec3f& pos)
{
    addRigidBox(stage, path.GetString(), GfVec3f(0.2f), pos, GfQuatf(1.0f), GfVec3f(0.7f), 1.0f);
    UsdPrim prim = stage->GetPrimAtPath(path);
    // explicit unit mass/inertia (mirrors testNewtonMimicAPI.py) so the stiff drive stays stable
    UsdPhysicsMassAPI massAPI = UsdPhysicsMassAPI::Apply(prim);
    massAPI.CreateMassAttr().Set(1.0f);
    massAPI.CreateDiagonalInertiaAttr().Set(GfVec3f(1.0f));
    PhysxSchemaPhysxRigidBodyAPI rbAPI = PhysxSchemaPhysxRigidBodyAPI::Apply(prim);
    rbAPI.CreateLinearDampingAttr().Set(0.0f);
    rbAPI.CreateAngularDampingAttr().Set(0.0f);
    rbAPI.CreateSleepThresholdAttr().Set(0.0f);
    rbAPI.CreateStabilizationThresholdAttr().Set(0.0f);
}

// Author a single-DOF articulation joint between root (parent) and link (child): a prismatic joint
// along X, or a revolute joint about Z with a finite limit (required by NewtonMimicAPI). When
// reverseBodyOrder is true the body0/body1 rels (and matching local poses) are swapped so the joint
// no longer follows the articulation hierarchy (mBody0IsParentLink == false) while the physical joint
// stays identical.
UsdPrim defineJointLink(UsdStageRefPtr stage, const SdfPath& rootPath, const SdfPath& linkPath,
    const SdfPath& jointPath, const GfVec3f& linkPos, bool reverseBodyOrder, bool revolute)
{
    makeRigidLink(stage, linkPath, linkPos);

    UsdPrim jointPrim;
    if (revolute)
    {
        UsdPhysicsRevoluteJoint joint = UsdPhysicsRevoluteJoint::Define(stage, jointPath);
        joint.CreateAxisAttr().Set(TfToken("Z"));
        joint.CreateLowerLimitAttr().Set(-180.0f);
        joint.CreateUpperLimitAttr().Set(180.0f);
        jointPrim = joint.GetPrim();
    }
    else
    {
        UsdPhysicsPrismaticJoint joint = UsdPhysicsPrismaticJoint::Define(stage, jointPath);
        joint.CreateAxisAttr().Set(TfToken("X"));
        joint.CreateLowerLimitAttr().Set(-10.0f);
        joint.CreateUpperLimitAttr().Set(10.0f);
        jointPrim = joint.GetPrim();
    }

    UsdPhysicsJoint joint(jointPrim);
    if (!reverseBodyOrder)
    {
        joint.CreateBody0Rel().AddTarget(rootPath);
        joint.CreateBody1Rel().AddTarget(linkPath);
        joint.CreateLocalPos0Attr().Set(linkPos); // root-side frame at the link location
        joint.CreateLocalPos1Attr().Set(GfVec3f(0.0f));
    }
    else
    {
        joint.CreateBody0Rel().AddTarget(linkPath);
        joint.CreateBody1Rel().AddTarget(rootPath);
        joint.CreateLocalPos0Attr().Set(GfVec3f(0.0f));
        joint.CreateLocalPos1Attr().Set(linkPos); // after internal swap, identical frames
    }
    return jointPrim;
}

// Build a fixed-base articulation with a driven leader link and a follower link constrained to the
// leader. The follower mimics the leader via NewtonMimicAPI, or, when usePhysXMimic is true, via
// PhysxSchemaPhysxMimicJointAPI authored with the equivalent gearing = -coef1 / offset = -coef0 (so
// PhysX's  jointPos + gearing*refPos + offset = 0  yields the same  follower = coef0 + coef1*leader).
// flipLeader / flipFollower reverse the corresponding joint's body order. Returns paths via out params.
void buildMimicArticulation(UsdStageRefPtr stage, float coef0, float coef1, bool flipLeader,
    bool flipFollower, bool revolute, bool usePhysXMimic, SdfPath& leaderLinkPath,
    SdfPath& followerLinkPath, SdfPath& leaderJointPath)
{
    const SdfPath rootPath("/World/root");
    makeRigidLink(stage, rootPath, GfVec3f(0.0f));

    // fixed-base articulation: root API + tuned solver lives on the world fixed joint (mirrors
    // testNewtonMimicAPI.py). High position-iteration count + disabled self-collision so the hard
    // mimic constraint is well converged in a single step.
    UsdPhysicsFixedJoint worldJoint = UsdPhysicsFixedJoint::Define(stage, SdfPath("/World/worldJoint"));
    worldJoint.CreateBody1Rel().AddTarget(rootPath);
    UsdPrim artPrim = worldJoint.GetPrim();
    UsdPhysicsArticulationRootAPI::Apply(artPrim);
    PhysxSchemaPhysxArticulationAPI artAPI = PhysxSchemaPhysxArticulationAPI::Apply(artPrim);
    artAPI.CreateSolverPositionIterationCountAttr().Set(16);
    artAPI.CreateSolverVelocityIterationCountAttr().Set(1);
    artAPI.CreateEnabledSelfCollisionsAttr().Set(false);

    leaderLinkPath = SdfPath("/World/leader");
    leaderJointPath = SdfPath("/World/leaderJoint");
    UsdPrim leaderJoint =
        defineJointLink(stage, rootPath, leaderLinkPath, leaderJointPath, GfVec3f(-1.f, 0.f, 0.f), flipLeader, revolute);

    // stiff position drive on the leader (implicit RC drive, stable in a single step)
    const TfToken driveToken = revolute ? TfToken("angular") : TfToken("linear");
    UsdPhysicsDriveAPI drive = UsdPhysicsDriveAPI::Apply(leaderJoint, driveToken);
    drive.CreateTargetPositionAttr().Set(revolute ? 15.0f : 0.1f); // degrees / meters
    drive.CreateStiffnessAttr().Set(1.0e10f);
    drive.CreateDampingAttr().Set(0.0f);

    followerLinkPath = SdfPath("/World/follower");
    const SdfPath followerJointPath("/World/followerJoint");
    UsdPrim fPrim =
        defineJointLink(stage, rootPath, followerLinkPath, followerJointPath, GfVec3f(1.f, 0.f, 0.f), flipFollower, revolute);

    if (usePhysXMimic)
    {
        PhysxSchemaPhysxMimicJointAPI api = PhysxSchemaPhysxMimicJointAPI::Apply(fPrim, TfToken("rotX"));
        api.CreateGearingAttr().Set(-coef1);
        api.CreateOffsetAttr().Set(-coef0);
        api.CreateReferenceJointAxisAttr().Set(TfToken("rotX"));
        api.CreateReferenceJointRel().AddTarget(leaderJointPath);
    }
    else
    {
        fPrim.ApplyAPI(kNewtonMimicAPI);
        fPrim.CreateAttribute(kMimicCoef0, SdfValueTypeNames->Float).Set(coef0);
        fPrim.CreateAttribute(kMimicCoef1, SdfValueTypeNames->Float).Set(coef1);
        fPrim.CreateAttribute(kMimicEnabled, SdfValueTypeNames->Bool).Set(true);
        fPrim.CreateRelationship(kMimicJoint).SetTargets({ leaderJointPath });
    }
}

// Run the scenario and check that the follower tracks the leader per the Newton relation, measured in
// the joints' authored USD convention:  q_follower = coef0 + coef1 * q_leader.
//
// omni.physx negates the PhysX articulation joint coordinate of any joint whose body0/body1 rel does
// not follow the articulation hierarchy (see InternalScene::getArticulationJointPosition). So the
// USD-convention position of a joint is its PhysX joint position negated when that joint was authored
// with reversed body order. The follower must obey the Newton relation in this convention regardless
// of either joint's body order, since a body0/body1 swap leaves the physical articulation unchanged.
void checkMimicTracks(float coef0, float coef1, bool flipLeader, bool flipFollower, bool revolute = false,
    bool usePhysXMimic = false)
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    UsdGeomSetStageUpAxis(stage, TfToken("Y"));
    UsdGeomSetStageMetersPerUnit(stage, 1.0);
    UsdPrim defaultPrim = stage->DefinePrim(SdfPath("/World"));
    stage->SetDefaultPrim(defaultPrim);
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, SdfPath("/World/physicsScene"));
    scene.CreateGravityMagnitudeAttr().Set(0.0f);

    SdfPath leaderLinkPath, followerLinkPath, leaderJointPath;
    buildMimicArticulation(stage, coef0, coef1, flipLeader, flipFollower, revolute, usePhysXMimic,
        leaderLinkPath, followerLinkPath, leaderJointPath);

    UsdUtilsStageCache::Get().Insert(stage);
    long stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    physxSim->attachStage(stageId);

    // a stiff implicit RC drive snaps the leader to its target in a single step; the hard mimic
    // constraint pulls the follower to the constrained position within the same solve
    physxSim->simulate(1.0f / 60.0f, 0.0f);
    physxSim->fetchResults();

    const float leaderSign = flipLeader ? -1.0f : 1.0f;
    const float followerSign = flipFollower ? -1.0f : 1.0f;
    const float qLeaderUsd = leaderSign * jointPos(physx, leaderJointPath, revolute);
    const float qFollowerUsd = followerSign * jointPos(physx, SdfPath("/World/followerJoint"), revolute);

    MESSAGE("qLeaderUsd=" << qLeaderUsd << " qFollowerUsd=" << qFollowerUsd);

    // sanity: the drive actually moved the leader
    CHECK(fabsf(qLeaderUsd) > 0.01f);

    const float expectedFollower = coef0 + coef1 * qLeaderUsd;
    CHECK(fabsf(qFollowerUsd - expectedFollower) < 0.02f);

    physxSim->detachStage();
    UsdUtilsStageCache::Get().Erase(stage);
}

// Change newton:mimicCoef1 at runtime on a follower with reversed body order and verify the follower
// re-tracks the leader per the new coefficient. This exercises InternalMimicJoint::setGearing, which
// must reapply the joints' flip parity just like construction does.
void checkMimicRuntimeCoef1Change(float initialCoef1, float newCoef1, bool flipFollower)
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    UsdGeomSetStageUpAxis(stage, TfToken("Y"));
    UsdGeomSetStageMetersPerUnit(stage, 1.0);
    UsdPrim defaultPrim = stage->DefinePrim(SdfPath("/World"));
    stage->SetDefaultPrim(defaultPrim);
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, SdfPath("/World/physicsScene"));
    scene.CreateGravityMagnitudeAttr().Set(0.0f);

    SdfPath leaderLinkPath, followerLinkPath, leaderJointPath;
    buildMimicArticulation(stage, 0.0f, initialCoef1, /*flipLeader*/ false, flipFollower, /*revolute*/ false,
        /*usePhysXMimic*/ false, leaderLinkPath, followerLinkPath, leaderJointPath);
    const SdfPath followerJointPath("/World/followerJoint");

    UsdUtilsStageCache::Get().Insert(stage);
    long stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    USDChange change;
    change.init(stageId, physicsTests.getApp()->getFramework());

    physxSim->attachStage(stageId);
    physxSim->simulate(1.0f / 60.0f, 0.0f);
    physxSim->fetchResults();

    // change the coefficient at runtime
    change.setAttributeValue(followerJointPath, kMimicCoef1, newCoef1);
    physxSim->simulate(1.0f / 60.0f, 0.0f);
    physxSim->fetchResults();

    const float followerSign = flipFollower ? -1.0f : 1.0f;
    const float qLeaderUsd = jointPos(physx, leaderJointPath, false);
    const float qFollowerUsd = followerSign * jointPos(physx, followerJointPath, false);

    MESSAGE("after runtime change: qLeaderUsd=" << qLeaderUsd << " qFollowerUsd=" << qFollowerUsd);

    CHECK(fabsf(qLeaderUsd) > 0.01f);
    CHECK(fabsf(qFollowerUsd - newCoef1 * qLeaderUsd) < 0.01f);

    change.destroy();
    physxSim->detachStage();
    UsdUtilsStageCache::Get().Erase(stage);
}
} // namespace

TEST_CASE("Mimic Joint follows leader regardless of joint body order", "[newton]")
{
    // Baseline: neither joint flipped. coef1 = +1 -> follower mirrors leader displacement.
    SUBCASE("prismatic, no flip, coef1=+1")
    {
        checkMimicTracks(/*coef0*/ 0.0f, /*coef1*/ 1.0f, /*flipLeader*/ false, /*flipFollower*/ false);
    }

    SUBCASE("prismatic, no flip, coef1=-1")
    {
        checkMimicTracks(0.0f, -1.0f, false, false);
    }

    SUBCASE("prismatic, no flip, coef0 offset")
    {
        checkMimicTracks(0.03f, 1.0f, false, false);
    }

    // The follower joint has reversed body order (mBody0IsParentLink == false). The physical articulation
    // is unchanged, so the follower must still track the leader. Without the per-joint sign correction the
    // follower moves the opposite way and this fails.
    SUBCASE("prismatic, follower flipped, coef1=+1")
    {
        checkMimicTracks(0.0f, 1.0f, false, true);
    }

    // The leader joint has reversed body order.
    SUBCASE("prismatic, leader flipped, coef1=+1")
    {
        checkMimicTracks(0.0f, 1.0f, true, false);
    }

    // Both flipped -> parity cancels, should track with the same sign as the no-flip case.
    SUBCASE("prismatic, both flipped, coef1=+1")
    {
        checkMimicTracks(0.0f, 1.0f, true, true);
    }

    // Revolute joints (the inspire-hand case): same sign-flip behaviour, exercising the
    // degrees<->radians gearing translation as well.
    SUBCASE("revolute, no flip, coef1=+1")
    {
        checkMimicTracks(0.0f, 1.0f, false, false, /*revolute*/ true);
    }

    SUBCASE("revolute, follower flipped, coef1=+1")
    {
        checkMimicTracks(0.0f, 1.0f, false, true, /*revolute*/ true);
    }

    SUBCASE("revolute, leader flipped, coef1=+1")
    {
        checkMimicTracks(0.0f, 1.0f, true, false, /*revolute*/ true);
    }

    // Runtime coef1 change must reapply the flip correction (exercises setGearing).
    SUBCASE("prismatic, follower flipped, runtime coef1 +1 -> -1")
    {
        checkMimicRuntimeCoef1Change(/*initial*/ 1.0f, /*new*/ -1.0f, /*flipFollower*/ true);
    }

    SUBCASE("prismatic, no flip, runtime coef1 +1 -> -1")
    {
        checkMimicRuntimeCoef1Change(1.0f, -1.0f, false);
    }

    // PhysxSchemaPhysxMimicJointAPI: same body-order sign-flip behaviour. coef0/coef1 here are the
    // equivalent Newton coefficients; the builder authors gearing = -coef1, offset = -coef0.
    SUBCASE("PhysX mimic prismatic, no flip, coef1=+1")
    {
        checkMimicTracks(0.0f, 1.0f, false, false, /*revolute*/ false, /*usePhysXMimic*/ true);
    }

    SUBCASE("PhysX mimic prismatic, follower flipped, coef1=+1")
    {
        checkMimicTracks(0.0f, 1.0f, false, true, false, /*usePhysXMimic*/ true);
    }

    SUBCASE("PhysX mimic prismatic, leader flipped, coef1=+1")
    {
        checkMimicTracks(0.0f, 1.0f, true, false, false, /*usePhysXMimic*/ true);
    }

    SUBCASE("PhysX mimic prismatic, both flipped, coef1=+1")
    {
        checkMimicTracks(0.0f, 1.0f, true, true, false, /*usePhysXMimic*/ true);
    }

    SUBCASE("PhysX mimic revolute, follower flipped, coef1=+1")
    {
        checkMimicTracks(0.0f, 1.0f, false, true, /*revolute*/ true, /*usePhysXMimic*/ true);
    }
}

#endif // NEWTON_SCHEMA_PLUGIN_PATH
