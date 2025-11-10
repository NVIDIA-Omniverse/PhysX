// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include "PhysicsTools.h"
#include "../../common/PhysicsChangeTemplate.h"

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxTests.h>
#include <omni/physx/IPhysxSimulation.h>

using namespace pxr;
using namespace omni::physx;
using namespace ::physx;

template <typename ChangeT, typename JointT>
struct TestTypePair
{
    typedef ChangeT ChangeType;
    typedef JointT  JointType;
};

//-----------------------------------------------------------------------------
// Revolute and prismatic joint tests
TEST_CASE_TEMPLATE("Revolute and Prismatic Joint Tests", T
, TestTypePair<USDChange, UsdPhysicsRevoluteJoint>
, TestTypePair<FabricChange, UsdPhysicsRevoluteJoint>
, TestTypePair<USDChange, UsdPhysicsPrismaticJoint>
, TestTypePair<FabricChange, UsdPhysicsPrismaticJoint>
)
{
    typename T::ChangeType changeTemplate;
    typedef typename T::JointType JointType;

    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

    const float boxSize = 100.f; 
    // create rigid body with a box collider
    addRigidBox(stage, "/World/box", GfVec3f(boxSize), GfVec3f(0.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath boxPath = SdfPath("/World/box");
    UsdPrim boxPrim = stage->GetPrimAtPath(boxPath);

    const SdfPath jointPath = SdfPath("/World/testJoint");
    JointType joint = JointType::Define(stage, jointPath);
    joint.GetBody1Rel().AddTarget(boxPath);
    joint.GetLowerLimitAttr().Set(-90.0f);
    joint.GetUpperLimitAttr().Set(90.0f);
    TfToken axisToken;
    if(std::is_same<JointType, UsdPhysicsRevoluteJoint>())
    {
        axisToken = TfToken("angular");
    }
    else if (std::is_same<JointType, UsdPhysicsPrismaticJoint>())
    {
        axisToken = TfToken("linear");
    }
    PhysxSchemaPhysxLimitAPI limitAPI = PhysxSchemaPhysxLimitAPI::Apply(joint.GetPrim(), axisToken);
    UsdPhysicsDriveAPI driveAPI = UsdPhysicsDriveAPI::Apply(joint.GetPrim(), axisToken);

    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);

    SUBCASE("Joint Enabled")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsJointEnabled;

        bool enabled = changeTemplate.template getAttributeValue<bool>(jointPath, changeToken);
        // check default value
        CHECK(enabled == true);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());        

        // disable joint, joint should not exist
        changeTemplate.setAttributeValue(jointPath, changeToken, false);
        changeTemplate.broadcastChanges();
        joint.CreateJointEnabledAttr().Set(false); // still through USD as we parse from USD
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // if not enabled static body should be created        
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(newBasePtr == nullptr);

        // enable joint
        changeTemplate.setAttributeValue(jointPath, changeToken, true);
        changeTemplate.broadcastChanges();
        joint.CreateJointEnabledAttr().Set(true); // still through USD as we parse from USD
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr->is<PxD6Joint>());
    }

    SUBCASE("Collision Enabled")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsCollisionEnabled;

        bool enabled = changeTemplate.template getAttributeValue<bool>(jointPath, changeToken);
        // check default value
        CHECK(enabled == false);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        PxD6Joint* physxJoint = basePtr->is<PxD6Joint>();
        CHECK((physxJoint->getConstraintFlags() & PxConstraintFlag::eCOLLISION_ENABLED) == false);

        // change value
        changeTemplate.setAttributeValue(jointPath, changeToken, true);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // if not enabled static body should be created        
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);

        CHECK((physxJoint->getConstraintFlags() & PxConstraintFlag::eCOLLISION_ENABLED) == true);
    }

    SUBCASE("Break Force")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsBreakForce;

        float value = changeTemplate.template getAttributeValue<float>(jointPath, changeToken);
        // check default value
        CHECK(!isfinite(value));

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        PxD6Joint* physxJoint = basePtr->is<PxD6Joint>();
        float breakForce;
        float breakTorque;
        physxJoint->getBreakForce(breakForce, breakTorque);
        CHECK(breakForce >= FLT_MAX);

        // change value
        value = 100.0f;
        changeTemplate.setAttributeValue(jointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // if not enabled static body should be created        
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);

        physxJoint->getBreakForce(breakForce, breakTorque);
        CHECK(fabsf(breakForce - value) < epsilon);
    }

    SUBCASE("Break Torque")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsBreakTorque;

        float value = changeTemplate.template getAttributeValue<float>(jointPath, changeToken);
        // check default value
        CHECK(!isfinite(value));

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        PxD6Joint* physxJoint = basePtr->is<PxD6Joint>();
        float breakForce;
        float breakTorque;
        physxJoint->getBreakForce(breakForce, breakTorque);
        CHECK(breakTorque >= FLT_MAX);

        // change value
        value = 100.0f;
        changeTemplate.setAttributeValue(jointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // if not enabled static body should be created        
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);

        physxJoint->getBreakForce(breakForce, breakTorque);
        CHECK(fabsf(breakTorque - value) < epsilon);
    }

    SUBCASE("Local Pos0")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsLocalPos0;

        GfVec3f value = changeTemplate.template getAttributeValue<GfVec3f>(jointPath, changeToken);
        // check default value
        compare(value, GfVec3f(0.0f), epsilon);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        PxD6Joint* physxJoint = basePtr->is<PxD6Joint>();
        PxTransform localPose = physxJoint->getLocalPose(PxJointActorIndex::eACTOR0);        
        compare(localPose.p, value, epsilon);

        // change value
        value = GfVec3f(2.0f);
        changeTemplate.setAttributeValue(jointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();
        
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);

        localPose = physxJoint->getLocalPose(PxJointActorIndex::eACTOR0);
        compare(localPose.p, value, epsilon);
    }

    SUBCASE("Local Rot0")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsLocalRot0;

        GfQuatf value = changeTemplate.template getAttributeValue<GfQuatf>(jointPath, changeToken);
        // check default value
        compare(value, GfQuatf(1.0f), epsilon);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        PxD6Joint* physxJoint = basePtr->is<PxD6Joint>();
        PxTransform localPose = physxJoint->getLocalPose(PxJointActorIndex::eACTOR0);
        compare(localPose.q, value, epsilon);

        // change value        
        value = GfQuatf(GfRotation(GfVec3d(1.0, 0.0, 0.0), 0.5).GetQuat());
        changeTemplate.setAttributeValue(jointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);

        localPose = physxJoint->getLocalPose(PxJointActorIndex::eACTOR0);
        compare(localPose.q, value, epsilon);
    }

    SUBCASE("Local Pos1")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsLocalPos1;

        GfVec3f value = changeTemplate.template getAttributeValue<GfVec3f>(jointPath, changeToken);
        // check default value
        compare(value, GfVec3f(0.0f), epsilon);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        PxD6Joint* physxJoint = basePtr->is<PxD6Joint>();
        PxTransform localPose = physxJoint->getLocalPose(PxJointActorIndex::eACTOR1);
        compare(localPose.p, value, epsilon);

        // change value
        value = GfVec3f(2.0f / boxSize);
        changeTemplate.setAttributeValue(jointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);

        localPose = physxJoint->getLocalPose(PxJointActorIndex::eACTOR1);
        value = value * boxSize;
        compare(localPose.p, value, epsilon);
    }

    SUBCASE("Local Rot1")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsLocalRot1;

        GfQuatf value = changeTemplate.template getAttributeValue<GfQuatf>(jointPath, changeToken);
        // check default value
        compare(value, GfQuatf(1.0f), epsilon);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        PxD6Joint* physxJoint = basePtr->is<PxD6Joint>();
        PxTransform localPose = physxJoint->getLocalPose(PxJointActorIndex::eACTOR1);
        compare(localPose.q, value, epsilon);

        // change value        
        value = GfQuatf(GfRotation(GfVec3d(1.0, 0.0, 0.0), 0.5).GetQuat());
        changeTemplate.setAttributeValue(jointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);

        localPose = physxJoint->getLocalPose(PxJointActorIndex::eACTOR1);
        compare(localPose.q, value, epsilon);
    }


    SUBCASE("Axis")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsAxis;

        TfToken value = changeTemplate.template getAttributeValue<TfToken>(jointPath, changeToken);
        // check default value
        CHECK(value == UsdPhysicsTokens->x);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        PxD6Joint* physxJoint = basePtr->is<PxD6Joint>();
        if(std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            CHECK(physxJoint->getMotion(PxD6Axis::eX) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eY) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eZ) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eTWIST) == PxD6Motion::eLIMITED);
            CHECK(physxJoint->getMotion(PxD6Axis::eSWING1) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eSWING2) == PxD6Motion::eLOCKED);
        }
        else if(std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            CHECK(physxJoint->getMotion(PxD6Axis::eX) == PxD6Motion::eLIMITED);
            CHECK(physxJoint->getMotion(PxD6Axis::eY) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eZ) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eTWIST) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eSWING1) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eSWING2) == PxD6Motion::eLOCKED);
        }

        // change value not supported
    }

    SUBCASE("Lower Limit")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsLowerLimit;

        float value = changeTemplate.template getAttributeValue<float>(jointPath, changeToken);
        // check default value
        CHECK(fabsf(value + 90.0f) < epsilon);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        PxD6Joint* physxJoint = basePtr->is<PxD6Joint>();
        if(std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            CHECK(physxJoint->getMotion(PxD6Axis::eX) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eY) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eZ) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eTWIST) == PxD6Motion::eLIMITED);
            CHECK(physxJoint->getMotion(PxD6Axis::eSWING1) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eSWING2) == PxD6Motion::eLOCKED);
            PxJointAngularLimitPair limitPair = physxJoint->getTwistLimit();
            CHECK(fabsf(radToDeg(limitPair.lower) - value) < epsilon);
        }
        else if(std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            CHECK(physxJoint->getMotion(PxD6Axis::eX) == PxD6Motion::eLIMITED);
            CHECK(physxJoint->getMotion(PxD6Axis::eY) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eZ) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eTWIST) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eSWING1) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eSWING2) == PxD6Motion::eLOCKED);
            PxJointLinearLimitPair limitPair = physxJoint->getLinearLimit(PxD6Axis::eX);
            CHECK(fabsf(limitPair.lower - value) < epsilon);
        }

        // change value
        value = -10.0f; // degree
        changeTemplate.setAttributeValue(jointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // if not enabled static body should be created        
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        if(std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            CHECK(physxJoint->getMotion(PxD6Axis::eX) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eY) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eZ) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eTWIST) == PxD6Motion::eLIMITED);
            CHECK(physxJoint->getMotion(PxD6Axis::eSWING1) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eSWING2) == PxD6Motion::eLOCKED);
            PxJointAngularLimitPair limitPair = physxJoint->getTwistLimit();
            CHECK(fabsf(radToDeg(limitPair.lower) - value) < epsilon);
        }
        else if(std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            CHECK(physxJoint->getMotion(PxD6Axis::eX) == PxD6Motion::eLIMITED);
            CHECK(physxJoint->getMotion(PxD6Axis::eY) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eZ) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eTWIST) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eSWING1) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eSWING2) == PxD6Motion::eLOCKED);
            PxJointLinearLimitPair limitPair = physxJoint->getLinearLimit(PxD6Axis::eX);
            CHECK(fabsf(limitPair.lower - value) < epsilon);
        }
    }

    SUBCASE("Upper Limit")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsUpperLimit;

        float value = changeTemplate.template getAttributeValue<float>(jointPath, changeToken);
        // check default value
        CHECK(fabsf(value - 90.0f) < epsilon);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        PxD6Joint* physxJoint = basePtr->is<PxD6Joint>();
        if(std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            CHECK(physxJoint->getMotion(PxD6Axis::eX) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eY) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eZ) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eTWIST) == PxD6Motion::eLIMITED);
            CHECK(physxJoint->getMotion(PxD6Axis::eSWING1) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eSWING2) == PxD6Motion::eLOCKED);
            PxJointAngularLimitPair limitPair = physxJoint->getTwistLimit();
            CHECK(fabsf(radToDeg(limitPair.upper) - value) < epsilon);
        }
        else if(std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            CHECK(physxJoint->getMotion(PxD6Axis::eX) == PxD6Motion::eLIMITED);
            CHECK(physxJoint->getMotion(PxD6Axis::eY) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eZ) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eTWIST) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eSWING1) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eSWING2) == PxD6Motion::eLOCKED);
            PxJointLinearLimitPair limitPair = physxJoint->getLinearLimit(PxD6Axis::eX);
            CHECK(fabsf(limitPair.upper - value) < epsilon);
        }

        // change value
        value = 10.0f; // degree
        changeTemplate.setAttributeValue(jointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // if not enabled static body should be created        
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        if(std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            CHECK(physxJoint->getMotion(PxD6Axis::eX) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eY) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eZ) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eTWIST) == PxD6Motion::eLIMITED);
            CHECK(physxJoint->getMotion(PxD6Axis::eSWING1) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eSWING2) == PxD6Motion::eLOCKED);
            PxJointAngularLimitPair limitPair = physxJoint->getTwistLimit();
            CHECK(fabsf(radToDeg(limitPair.upper) - value) < epsilon);
        }
        else if(std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            CHECK(physxJoint->getMotion(PxD6Axis::eX) == PxD6Motion::eLIMITED);
            CHECK(physxJoint->getMotion(PxD6Axis::eY) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eZ) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eTWIST) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eSWING1) == PxD6Motion::eLOCKED);
            CHECK(physxJoint->getMotion(PxD6Axis::eSWING2) == PxD6Motion::eLOCKED);
            PxJointLinearLimitPair limitPair = physxJoint->getLinearLimit(PxD6Axis::eX);
            CHECK(fabsf(limitPair.upper - value) < epsilon);
        }
    }

    SUBCASE("Drive Type")
    {
        const TfToken changeToken(std::string("drive:") + axisToken.GetString() + ":physics:type");

        TfToken value = changeTemplate.template getAttributeValue<TfToken>(jointPath, changeToken);
        // check default value
        CHECK(value == UsdPhysicsTokens->force);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        PxD6Joint* physxJoint = basePtr->is<PxD6Joint>();
        if(std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            PxD6JointDrive twistDrive = physxJoint->getDrive(PxD6Drive::eTWIST);
            CHECK((twistDrive.flags & PxD6JointDriveFlag::eACCELERATION) == false);
        }
        else if(std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            PxD6JointDrive xDrive = physxJoint->getDrive(PxD6Drive::eX);
            CHECK((xDrive.flags & PxD6JointDriveFlag::eACCELERATION) == false);
        }
        // change value
        value = UsdPhysicsTokens->acceleration;
        changeTemplate.setAttributeValue(jointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // if not enabled static body should be created        
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        if(std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            PxD6JointDrive twistDrive = physxJoint->getDrive(PxD6Drive::eTWIST);
            CHECK((twistDrive.flags & PxD6JointDriveFlag::eACCELERATION) == true);
        }
        else if(std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            PxD6JointDrive xDrive = physxJoint->getDrive(PxD6Drive::eX);
            CHECK((xDrive.flags & PxD6JointDriveFlag::eACCELERATION) == true);
        }
    }

    SUBCASE("Drive Max Force")
    {
        const TfToken changeToken(std::string("drive:") + axisToken.GetString() + ":physics:maxForce");

        float value = changeTemplate.template getAttributeValue<float>(jointPath, changeToken);
        // check default value
        CHECK(!isfinite(value));

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        PxD6Joint* physxJoint = basePtr->is<PxD6Joint>();
        if(std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            PxD6JointDrive twistDrive = physxJoint->getDrive(PxD6Drive::eTWIST);
            CHECK(twistDrive.forceLimit >= FLT_MAX);
        }
        else if(std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            PxD6JointDrive xDrive = physxJoint->getDrive(PxD6Drive::eX);
            CHECK(xDrive.forceLimit >= FLT_MAX);
        }

        // change value
        value = 1000.0f;
        changeTemplate.setAttributeValue(jointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // if not enabled static body should be created        
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        if(std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            PxD6JointDrive twistDrive = physxJoint->getDrive(PxD6Drive::eTWIST);
            CHECK(fabsf(twistDrive.forceLimit - value) < epsilon);
        }
        else if(std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            PxD6JointDrive xDrive = physxJoint->getDrive(PxD6Drive::eX);
            CHECK(fabsf(xDrive.forceLimit - value) < epsilon);
        }
    }

    SUBCASE("Drive Target Position")
    {
        const TfToken changeToken(std::string("drive:") + axisToken.GetString() + ":physics:targetPosition");

        float value = changeTemplate.template getAttributeValue<float>(jointPath, changeToken);
        // check default value
        CHECK(fabsf(value) < epsilon);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        PxD6Joint* physxJoint = basePtr->is<PxD6Joint>();
        PxTransform driveTr = physxJoint->getDrivePosition();
        compare(driveTr.p, PxVec3(PxZero), epsilon);
        CHECK(driveTr.q.isIdentity());

        // change value
        value = 90.0f;
        changeTemplate.setAttributeValue(jointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // if not enabled static body should be created        
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        driveTr = physxJoint->getDrivePosition();
        if(std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            compare(driveTr.p, PxVec3(PxZero), epsilon);
            float rad;
            PxVec3 axis;
            driveTr.q.toRadiansAndUnitAxis(rad, axis);
            CHECK(fabsf(radToDeg(rad) - value) < epsilon);
            compare(axis, PxVec3(1.0f, 0.0f, 0.0f), epsilon);
        }
        else if(std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            compare(driveTr.p, PxVec3(90.0f, 0.0f, 0.0f), epsilon);
            float rad;
            PxVec3 axis;
            driveTr.q.toRadiansAndUnitAxis(rad, axis);
            CHECK(fabsf(radToDeg(rad) - 0.0f) < epsilon);
        }
    }

    SUBCASE("Drive Target Velocity")
    {
        const TfToken changeToken(std::string("drive:") + axisToken.GetString() + ":physics:targetVelocity");

        float value = changeTemplate.template getAttributeValue<float>(jointPath, changeToken);
        // check default value
        CHECK(fabsf(value) < epsilon);

        // check nonzero default value for inversion due to SDK bug OM-42441
        physxSim->detachStage();

        value = 10.0f;
        driveAPI.CreateTargetVelocityAttr().Set(value);

        // reattach
        physxSim->attachStage(stageId);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        PxD6Joint* physxJoint = basePtr->is<PxD6Joint>();
        PxVec3 linearVelocity;
        PxVec3 angularVelocity;
        physxJoint->getDriveVelocity(linearVelocity, angularVelocity);
        if(std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            compare(linearVelocity, PxVec3(PxZero), epsilon);
            // inversion due to SDK bug https ://nvidia-omniverse.atlassian.net/browse/OM-42441
            compare(angularVelocity, PxVec3(-degToRad(value), 0.0f, 0.0f), epsilon);
        }
        else if(std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            compare(linearVelocity, PxVec3(value, 0.0f, 0.0f), epsilon);
            compare(angularVelocity, PxVec3(PxZero), epsilon);
        }
        
        // change value
        value = 90.0f;
        changeTemplate.setAttributeValue(jointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // if not enabled static body should be created        
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        physxJoint->getDriveVelocity(linearVelocity, angularVelocity);
        if(std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            compare(linearVelocity, PxVec3(PxZero), epsilon);
            // inversion due to SDK bug https ://nvidia-omniverse.atlassian.net/browse/OM-42441
            compare(angularVelocity, PxVec3(degToRad(-value), 0.0f, 0.0f), epsilon);
        }
        else if(std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            compare(angularVelocity, PxVec3(PxZero), epsilon);
            compare(linearVelocity, PxVec3(value, 0.0f, 0.0f), epsilon);        
        }
    }

    SUBCASE("Drive Damping")
    {
        const TfToken changeToken(std::string("drive:") + axisToken.GetString() + ":physics:damping");

        float value = changeTemplate.template getAttributeValue<float>(jointPath, changeToken);
        // check default value
        CHECK(fabsf(value) < epsilon);

        // check nonzero default value for scaling:
        physxSim->detachStage();

        value = 10.0f;
        driveAPI.CreateDampingAttr().Set(value);

        // reattach
        physxSim->attachStage(stageId);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        PxD6Joint* physxJoint = basePtr->is<PxD6Joint>();
        if(std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            PxD6JointDrive twistDrive = physxJoint->getDrive(PxD6Drive::eTWIST);
            CHECK(fabsf(twistDrive.damping - radToDeg(value)) < epsilon);
        }        
        else if(std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            PxD6JointDrive xDrive = physxJoint->getDrive(PxD6Drive::eX);
            CHECK(fabsf(xDrive.damping - value) < epsilon);
        }
        // change value
        value = 90.0f;
        changeTemplate.setAttributeValue(jointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // if not enabled static body should be created        
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        if(std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            PxD6JointDrive twistDrive = physxJoint->getDrive(PxD6Drive::eTWIST);
            CHECK(fabsf(twistDrive.damping - radToDeg(value)) < epsilon);
        }
        else if(std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            PxD6JointDrive xDrive = physxJoint->getDrive(PxD6Drive::eX);
            CHECK(fabsf(xDrive.damping - value) < epsilon);
        }
    }

    SUBCASE("Drive Damping - Velocity")
    {
        const TfToken changeToken(std::string("drive:") + axisToken.GetString() + ":physics:damping");

        float value = changeTemplate.template getAttributeValue<float>(jointPath, changeToken);
        // check default value
        CHECK(fabsf(value) < epsilon);

        // check nonzero default value for scaling:
        physxSim->detachStage();

        value = 10.0f;
        driveAPI.CreateDampingAttr().Set(value);

        const float velocity_value = 20.0f;
        PxVec3 linearVelocity;
        PxVec3 angularVelocity;
        driveAPI.CreateTargetVelocityAttr().Set(velocity_value);

        // reattach
        physxSim->attachStage(stageId);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        PxD6Joint* physxJoint = basePtr->is<PxD6Joint>();
        if (std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            PxD6JointDrive twistDrive = physxJoint->getDrive(PxD6Drive::eTWIST);
            CHECK(fabsf(twistDrive.damping - radToDeg(value)) < epsilon);
        }
        else if (std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            PxD6JointDrive xDrive = physxJoint->getDrive(PxD6Drive::eX);
            CHECK(fabsf(xDrive.damping - value) < epsilon);
        }

        // check drive velocity
        physxJoint->getDriveVelocity(linearVelocity, angularVelocity);
        if (std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            compare(linearVelocity, PxVec3(PxZero), epsilon);
            // inversion due to SDK bug https ://nvidia-omniverse.atlassian.net/browse/OM-42441
            compare(angularVelocity, PxVec3(-degToRad(velocity_value), 0.0f, 0.0f), epsilon);
        }
        else if (std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            compare(linearVelocity, PxVec3(velocity_value, 0.0f, 0.0f), epsilon);
            compare(angularVelocity, PxVec3(PxZero), epsilon);
        }

        // change value
        value = 90.0f;
        changeTemplate.setAttributeValue(jointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // if not enabled static body should be created
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        if (std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            PxD6JointDrive twistDrive = physxJoint->getDrive(PxD6Drive::eTWIST);
            CHECK(fabsf(twistDrive.damping - radToDeg(value)) < epsilon);
        }
        else if (std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            PxD6JointDrive xDrive = physxJoint->getDrive(PxD6Drive::eX);
            CHECK(fabsf(xDrive.damping - value) < epsilon);
        }

        // check drive velocity
        physxJoint->getDriveVelocity(linearVelocity, angularVelocity);
        if (std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            compare(linearVelocity, PxVec3(PxZero), epsilon);
            // inversion due to SDK bug https ://nvidia-omniverse.atlassian.net/browse/OM-42441
            compare(angularVelocity, PxVec3(-degToRad(velocity_value), 0.0f, 0.0f), epsilon);
        }
        else if (std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            compare(linearVelocity, PxVec3(velocity_value, 0.0f, 0.0f), epsilon);
            compare(angularVelocity, PxVec3(PxZero), epsilon);
        }

    }

    SUBCASE("Drive Stiffness")
    {
        const TfToken changeToken(std::string("drive:") + axisToken.GetString() + ":physics:stiffness");

        float value = changeTemplate.template getAttributeValue<float>(jointPath, changeToken);
        // check default value
        CHECK(fabsf(value) < epsilon);

        // check nonzero default value for scaling:
        physxSim->detachStage();

        value = 10.0f;
        driveAPI.CreateStiffnessAttr().Set(value);

        // reattach
        physxSim->attachStage(stageId);

        // D6 joint should be there, revolute joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        PxD6Joint* physxJoint = basePtr->is<PxD6Joint>();

        if(std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            PxD6JointDrive twistDrive = physxJoint->getDrive(PxD6Drive::eTWIST);
            CHECK(fabsf(twistDrive.stiffness - radToDeg(value)) < epsilon);
        }
        else if(std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            PxD6JointDrive xDrive = physxJoint->getDrive(PxD6Drive::eX);
            CHECK(fabsf(xDrive.stiffness - value) < epsilon);
        }

        // change value
        value = 90.0f;
        changeTemplate.setAttributeValue(jointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // if not enabled static body should be created        
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(newBasePtr != nullptr);
        CHECK(newBasePtr == basePtr);
        if(std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            PxD6JointDrive twistDrive = physxJoint->getDrive(PxD6Drive::eTWIST);
            CHECK(fabsf(twistDrive.stiffness - radToDeg(value)) < epsilon);
        }
        else if(std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            PxD6JointDrive xDrive = physxJoint->getDrive(PxD6Drive::eX);
            CHECK(fabsf(xDrive.stiffness - value) < epsilon);
        }
    }

    auto testLimitParametric = [&](
        const TfToken changeToken,
        pxr::UsdAttribute (pxr::PhysxSchemaPhysxLimitAPI::*CreateFunc)(pxr::VtValue const &defaultValue, bool) const,
        PxReal PxJointAngularLimitPair::*angularField,
        PxReal PxJointLinearLimitPair::*linearField)
    {
        float value = changeTemplate.template getAttributeValue<float>(jointPath, changeToken);
        // check default value
        CHECK(fabsf(value) < epsilon);

        // check nonzero default value for scaling:
        physxSim->detachStage();

        value = 0.3f;
        (limitAPI.*CreateFunc)(pxr::VtValue(), false).Set(value);

        // reattach
        physxSim->attachStage(stageId);

        // D6 joint should be there, prismatic joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxD6Joint>());

        PxD6Joint* physxJoint = basePtr->is<PxD6Joint>();
        if(std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            auto twistLimit = physxJoint->getTwistLimit();
            CHECK(fabsf(twistLimit.*angularField - value) < epsilon);
        }
        else if(std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            auto linearLimit = physxJoint->getLinearLimit(PxD6Axis::eX);
            CHECK(fabsf(linearLimit.*linearField - value) < epsilon);
        }

        // change value
        value = 0.5f;
        changeTemplate.setAttributeValue(jointPath, changeToken, value);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        if(std::is_same<JointType, UsdPhysicsRevoluteJoint>())
        {
            auto twistLimit = physxJoint->getTwistLimit();
            CHECK(fabsf(twistLimit.*angularField - value) < epsilon);
        }
        else if(std::is_same<JointType, UsdPhysicsPrismaticJoint>())
        {
            auto linearLimit = physxJoint->getLinearLimit(PxD6Axis::eX);
            CHECK(fabsf(linearLimit.*linearField - value) < epsilon);
        }
    };

    SUBCASE("Limit Stiffness")
    {
        testLimitParametric(TfToken(std::string("physxLimit:") + axisToken.GetString() + ":stiffness"),
                            &pxr::PhysxSchemaPhysxLimitAPI::CreateStiffnessAttr,
                            &PxJointAngularLimitPair::stiffness,
                            &PxJointLinearLimitPair::stiffness);
    }
    SUBCASE("Limit Damping")
    {
        testLimitParametric(TfToken(std::string("physxLimit:") + axisToken.GetString() + ":damping"),
                            &pxr::PhysxSchemaPhysxLimitAPI::CreateDampingAttr,
                            &PxJointAngularLimitPair::damping,
                            &PxJointLinearLimitPair::damping);
    }
    SUBCASE("Limit Restitution")
    {
        testLimitParametric( TfToken(std::string("physxLimit:") + axisToken.GetString() + ":restitution"),
                            &pxr::PhysxSchemaPhysxLimitAPI::CreateRestitutionAttr,
                            &PxJointAngularLimitPair::restitution,
                            &PxJointLinearLimitPair::restitution);
    }
    SUBCASE("Limit Bounce Threshold")
    {
        testLimitParametric(TfToken(std::string("physxLimit:") + axisToken.GetString() + ":bounceThreshold"),
                            &pxr::PhysxSchemaPhysxLimitAPI::CreateBounceThresholdAttr,
                            &PxJointAngularLimitPair::bounceThreshold,
                            &PxJointLinearLimitPair::bounceThreshold);
    }
    // Common post-test actions
    physxSim->detachStage();

    changeTemplate.destroy();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

