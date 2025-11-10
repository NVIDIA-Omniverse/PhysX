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

//-----------------------------------------------------------------------------
// D6 joint tests
TEST_CASE_TEMPLATE("D6 Joint Tests",T, USDChange, FabricChange)
{
    T changeTemplate;
    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    carb::events::IEventStreamPtr errorStream = physx->getErrorEventStream();

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

    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

    // create rigid body with a box collider
    addRigidBox(stage, "/World/box", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);

    const SdfPath boxPath = SdfPath("/World/box");
    UsdPrim boxPrim = stage->GetPrimAtPath(boxPath);

    const SdfPath jointPath = SdfPath("/World/d6Joint");
    UsdPhysicsJoint joint = UsdPhysicsJoint::Define(stage, jointPath);
    joint.GetBody1Rel().AddTarget(boxPath);

    SUBCASE("Distance limit")
    {
        UsdPhysicsLimitAPI limitAPI = UsdPhysicsLimitAPI::Apply(joint.GetPrim(), TfToken("distance"));        
        limitAPI.GetHighAttr().Set(100.0f);        

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        // D6 joint should be there
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        REQUIRE(basePtr != nullptr);
        REQUIRE(basePtr->is<PxD6Joint>());

        PxD6Joint* d6Joint = basePtr->is<PxD6Joint>();
        CHECK(d6Joint->getMotion(PxD6Axis::eX) == PxD6Motion::eLIMITED);
        CHECK(d6Joint->getMotion(PxD6Axis::eY) == PxD6Motion::eLIMITED);
        CHECK(d6Joint->getMotion(PxD6Axis::eZ) == PxD6Motion::eLIMITED);
        CHECK(d6Joint->getMotion(PxD6Axis::eTWIST) == PxD6Motion::eFREE);
        CHECK(d6Joint->getMotion(PxD6Axis::eSWING1) == PxD6Motion::eFREE);
        CHECK(d6Joint->getMotion(PxD6Axis::eSWING2) == PxD6Motion::eFREE);

        CHECK(d6Joint->getDistanceLimit().value == 100.0f);
    }

    SUBCASE("Drives")
    {
        // preist: getting spurious asserts with fc changes
        // OM-44072
        if (changeTemplate.isFabric())
            return;

        TfTokenVector axisTokens = { UsdPhysicsTokens->transX , UsdPhysicsTokens->transY, UsdPhysicsTokens->transZ, UsdPhysicsTokens->rotX, UsdPhysicsTokens->rotY, UsdPhysicsTokens->rotZ };

        std::vector<UsdPhysicsDriveAPI> driveAPIs;
        TfTokenVector driveAttributeTokens;
        for (const TfToken axis : axisTokens)
        {
            driveAPIs.push_back(UsdPhysicsDriveAPI::Apply(joint.GetPrim(), axis));
        }

        std::vector<PxD6Drive::Enum> usdAxisToPxD6Drive = { PxD6Drive::eX, PxD6Drive::eY, PxD6Drive::eZ, PxD6Drive::eTWIST, PxD6Drive::eSWING1, PxD6Drive::eSWING2 };

        SUBCASE("Damping")
        {
            const float startValue = 10.0f;
            const float delta = 5.0f;
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                driveAPIs[i].CreateDampingAttr().Set(startValue + i * delta);
            }

            // attach
            physxSim->attachStage(stageId);

            PxD6Joint* d6Joint = getPhysxBaseDerivedFromPathChecked<PxD6Joint>(jointPath, ePTJoint);

            // check:
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                float expectedValue = startValue + i * delta;
                if (i > 2) // for rot we need to take deg to rad conversion into account
                {
                    expectedValue = radToDeg(expectedValue);
                }
                PxD6JointDrive drive = d6Joint->getDrive(usdAxisToPxD6Drive[i]);
                CHECK(fabsf(expectedValue - drive.damping) < epsilon);
            }

            // change value:
            const float changedStartValue = 11.0f;
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                TfToken driveAttributeToken = TfToken("drive:" + axisTokens[i].GetString() + ":physics:damping");
                changeTemplate.setAttributeValue(jointPath, driveAttributeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxD6Joint* newD6Joint = getPhysxBaseDerivedFromPathChecked<PxD6Joint>(jointPath, ePTJoint);

            CHECK_EQ(d6Joint, newD6Joint);

           // check:
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                float expectedValue = changedStartValue + i * delta;
                if (i > 2) // for rot we need to take deg to rad conversion into account
                {
                    expectedValue = radToDeg(expectedValue);
                }
                PxD6JointDrive drive = d6Joint->getDrive(usdAxisToPxD6Drive[i]);
                CHECK(fabsf(expectedValue - drive.damping) < epsilon);
            }
        }

        SUBCASE("Stiffness")
        {
            const float startValue = 10.0f;
            const float delta = 5.0f;
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                driveAPIs[i].CreateStiffnessAttr().Set(startValue + i * delta);
            }

            // attach
            physxSim->attachStage(stageId);

            PxD6Joint* d6Joint = getPhysxBaseDerivedFromPathChecked<PxD6Joint>(jointPath, ePTJoint);

            // check:
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                float expectedValue = startValue + i * delta;
                if (i > 2) // for rot we need to take deg to rad conversion into account
                {
                    expectedValue = radToDeg(expectedValue);
                }
                PxD6JointDrive drive = d6Joint->getDrive(usdAxisToPxD6Drive[i]);
                CHECK(fabsf(expectedValue - drive.stiffness) < epsilon);
            }

            // change value:
            const float changedStartValue = 11.0f;
            for (size_t i = 0; i < axisTokens.size(); ++i)
            {
                TfToken driveAttributeToken = TfToken("drive:" + axisTokens[i].GetString() + ":physics:stiffness");
                changeTemplate.setAttributeValue(jointPath, driveAttributeToken, changedStartValue + i * delta);
                changeTemplate.broadcastChanges();
            }
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            PxD6Joint* newD6Joint = getPhysxBaseDerivedFromPathChecked<PxD6Joint>(jointPath, ePTJoint);

            CHECK_EQ(d6Joint, newD6Joint);

           // check:
            for (size_t i = 0; i < driveAPIs.size(); ++i)
            {
                float expectedValue = changedStartValue + i * delta;
                if (i > 2) // for rot we need to take deg to rad conversion into account
                {
                    expectedValue = radToDeg(expectedValue);
                }
                PxD6JointDrive drive = d6Joint->getDrive(usdAxisToPxD6Drive[i]);
                CHECK(fabsf(expectedValue - drive.stiffness) < epsilon);
            }
        }
    }

// GW: There is an issue with limits on CPU ("D6JointSolverPrep: invalid joint setup. Double pyramid mode not supported.")
// Disabling this test for now when this is the case.  In the future it should not make a difference as all platforms will have GPU support by default.
#if USE_PHYSX_GPU
    SUBCASE("Limit Api")
    {
        TfTokenVector axisTokens = { UsdPhysicsTokens->transX , UsdPhysicsTokens->transY, UsdPhysicsTokens->transZ, UsdPhysicsTokens->rotX, UsdPhysicsTokens->rotY, UsdPhysicsTokens->rotZ };
        auto testLimitParametric = [&]( const TfToken changeToken,
                                        pxr::PhysxSchemaPhysxLimitAPI& limitAPI,
                                        pxr::UsdAttribute (pxr::PhysxSchemaPhysxLimitAPI::*CreateFunc)(pxr::VtValue const &defaultValue, bool) const,
                                        PxReal PxJointAngularLimitPair::*angularField,
                                        PxReal PxJointLinearLimitPair::*linearField,
                                        PxReal PxJointLimitPyramid::*pyramidField
                                        )
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
            const auto& tokenString = changeToken.GetString();
            if(tokenString.rfind("physxLimit:transX:") == 0)
            {
                auto linearLimit = physxJoint->getLinearLimit(PxD6Axis::eX);
                CHECK(fabsf(linearLimit.*linearField - value) < epsilon);
            }
            else if(tokenString.rfind("physxLimit:transY:") == 0)
            {
                auto linearLimit = physxJoint->getLinearLimit(PxD6Axis::eY);
                CHECK(fabsf(linearLimit.*linearField - value) < epsilon);
            }
            else if(tokenString.rfind("physxLimit:transZ:") == 0)
            {
                auto linearLimit = physxJoint->getLinearLimit(PxD6Axis::eZ);
                CHECK(fabsf(linearLimit.*linearField - value) < epsilon);
            }
            else if(tokenString.rfind("physxLimit:rotX:") == 0)
            {
                auto twistLimit = physxJoint->getTwistLimit();
                CHECK(fabsf(twistLimit.*angularField - value) < epsilon);
            }
            else if(tokenString.rfind("physxLimit:rotY:") == 0)
            {
                auto pyramidSwingLimit = physxJoint->getPyramidSwingLimit();
                CHECK(fabsf(pyramidSwingLimit.*pyramidField - value) < epsilon);
            }
            else if(tokenString.rfind("physxLimit:rotZ:") == 0)
            {
                auto pyramidSwingLimit = physxJoint->getPyramidSwingLimit();
                CHECK(fabsf(pyramidSwingLimit.*pyramidField - value) < epsilon);
            }

            // change value
            value = 0.5f;
            changeTemplate.setAttributeValue(jointPath, changeToken, value);
            changeTemplate.broadcastChanges();
            physxSim->simulate(0.01f, 0.0f);
            physxSim->fetchResults();

            if(tokenString.rfind("physxLimit:transX:") == 0)
            {
                auto linearLimit = physxJoint->getLinearLimit(PxD6Axis::eX);
                CHECK(fabsf(linearLimit.*linearField - value) < epsilon);
            }
            else if(tokenString.rfind("physxLimit:transY:") == 0)
            {
                auto linearLimit = physxJoint->getLinearLimit(PxD6Axis::eY);
                CHECK(fabsf(linearLimit.*linearField - value) < epsilon);
            }
            else if(tokenString.rfind("physxLimit:transZ:") == 0)
            {
                auto linearLimit = physxJoint->getLinearLimit(PxD6Axis::eZ);
                CHECK(fabsf(linearLimit.*linearField - value) < epsilon);
            }
            else if(tokenString.rfind("physxLimit:rotX:") == 0)
            {
                auto twistLimit = physxJoint->getTwistLimit();
                CHECK(fabsf(twistLimit.*angularField - value) < epsilon);
            }
            else if(tokenString.rfind("physxLimit:rotY:") == 0)
            {
                auto pyramidSwingLimit = physxJoint->getPyramidSwingLimit();
                CHECK(fabsf(pyramidSwingLimit.*pyramidField - value) < epsilon);
            }
            else if(tokenString.rfind("physxLimit:rotZ:") == 0)
            {
                auto pyramidSwingLimit = physxJoint->getPyramidSwingLimit();
                CHECK(fabsf(pyramidSwingLimit.*pyramidField - value) < epsilon);
            }

        };
        SUBCASE("Limit Stiffness")
        {
            for (const TfToken axis : axisTokens)
            {
                auto limitAPI =  PhysxSchemaPhysxLimitAPI::Apply(joint.GetPrim(), axis);
                auto usdLimitAPI = UsdPhysicsLimitAPI::Apply(joint.GetPrim(), axis);
                usdLimitAPI.CreateLowAttr().Set(-90.0f);
                usdLimitAPI.CreateHighAttr().Set(+90.0f);
                testLimitParametric( TfToken(std::string("physxLimit:") + axis.GetString() + ":stiffness"),
                                    limitAPI,
                                    &pxr::PhysxSchemaPhysxLimitAPI::CreateStiffnessAttr,
                                    &PxJointAngularLimitPair::stiffness,
                                    &PxJointLinearLimitPair::stiffness,
                                    &PxJointLimitPyramid::stiffness);
            }
        }
        SUBCASE("Limit Damping")
        {
            for (const TfToken axis : axisTokens)
            {
                auto limitAPI =  PhysxSchemaPhysxLimitAPI::Apply(joint.GetPrim(), axis);
                auto usdLimitAPI = UsdPhysicsLimitAPI::Apply(joint.GetPrim(), axis);
                usdLimitAPI.CreateLowAttr().Set(-90.0f);
                usdLimitAPI.CreateHighAttr().Set(+90.0f);
                testLimitParametric( TfToken(std::string("physxLimit:") + axis.GetString() + ":damping"),
                                    limitAPI,
                                    &pxr::PhysxSchemaPhysxLimitAPI::CreateDampingAttr,
                                    &PxJointAngularLimitPair::damping,
                                    &PxJointLinearLimitPair::damping,
                                    &PxJointLimitPyramid::damping);
            }
        }
        SUBCASE("Limit Restitution")
        {
            for (const TfToken axis : axisTokens)
            {
                auto limitAPI =  PhysxSchemaPhysxLimitAPI::Apply(joint.GetPrim(), axis);
                auto usdLimitAPI = UsdPhysicsLimitAPI::Apply(joint.GetPrim(), axis);
                usdLimitAPI.CreateLowAttr().Set(-90.0f);
                usdLimitAPI.CreateHighAttr().Set(+90.0f);
                testLimitParametric( TfToken(std::string("physxLimit:") + axis.GetString() + ":restitution"),
                                    limitAPI,
                                    &pxr::PhysxSchemaPhysxLimitAPI::CreateRestitutionAttr,
                                    &PxJointAngularLimitPair::restitution,
                                    &PxJointLinearLimitPair::restitution,
                                    &PxJointLimitPyramid::restitution);
            }
        }
        SUBCASE("Limit Bounce Threshold")
        {
            for (const TfToken axis : axisTokens)
            {
                auto limitAPI =  PhysxSchemaPhysxLimitAPI::Apply(joint.GetPrim(), axis);
                auto usdLimitAPI = UsdPhysicsLimitAPI::Apply(joint.GetPrim(), axis);
                usdLimitAPI.CreateLowAttr().Set(-90.0f);
                usdLimitAPI.CreateHighAttr().Set(+90.0f);
                testLimitParametric( TfToken(std::string("physxLimit:") + axis.GetString() + ":bounceThreshold"),
                                    limitAPI,
                                    &pxr::PhysxSchemaPhysxLimitAPI::CreateBounceThresholdAttr,
                                    &PxJointAngularLimitPair::bounceThreshold,
                                    &PxJointLinearLimitPair::bounceThreshold,
                                    &PxJointLimitPyramid::bounceThreshold);
            }
        }
    }
#endif // USE_PHYSX_GPU

    SUBCASE("Limit Pyramidal Validity")
    {
        UsdPhysicsLimitAPI usdLimitAPIY = UsdPhysicsLimitAPI::Apply(joint.GetPrim(), UsdPhysicsTokens->rotY);
        usdLimitAPIY.CreateLowAttr().Set(-INFINITY);
        usdLimitAPIY.CreateHighAttr().Set(INFINITY);
        UsdPhysicsLimitAPI usdLimitAPIZ = UsdPhysicsLimitAPI::Apply(joint.GetPrim(), UsdPhysicsTokens->rotZ);
        usdLimitAPIZ.CreateLowAttr().Set(-INFINITY);
        usdLimitAPIZ.CreateHighAttr().Set(INFINITY);

        // check nonzero default value for scaling:
        physxSim->detachStage();

        // reattach
        physxSim->attachStage(stageId);

        // D6 joint should be there, prismatic joint is represented as D6 joint in physx
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(basePtr != nullptr);
        REQUIRE(basePtr->is<PxD6Joint>());

        PxD6Joint* physxJoint = basePtr->is<PxD6Joint>();

        CHECK(physxJoint->getMotion(PxD6Axis::eSWING1) == PxD6Motion::eFREE);
        CHECK(physxJoint->getMotion(PxD6Axis::eSWING2) == PxD6Motion::eFREE);
    }

    SUBCASE("Body Switch")
    {
        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        // D6 joint should be there
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        REQUIRE(basePtr != nullptr);
        REQUIRE(basePtr->is<PxD6Joint>());

        PxD6Joint* d6Joint = basePtr->is<PxD6Joint>();

        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTActor));
        REQUIRE(basePtr != nullptr);
        REQUIRE(basePtr->is<PxRigidActor>());

        PxRigidActor* body1 = basePtr->is<PxRigidActor>();

        {
            PxRigidActor* actor0 = nullptr;
            PxRigidActor* actor1 = nullptr;
            d6Joint->getActors(actor0, actor1);
            CHECK(actor0 == nullptr);
            CHECK(actor1 == body1);
        }

        // add new rigid
        addRigidBox(stage, "/World/box0", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);
        const SdfPath box0Path = SdfPath("/World/box0");

        joint.GetBody0Rel().AddTarget(box0Path);

        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();


        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(box0Path, ePTActor));
        REQUIRE(basePtr != nullptr);
        REQUIRE(basePtr->is<PxRigidActor>());

        PxRigidActor* body0 = basePtr->is<PxRigidActor>();

        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        REQUIRE(basePtr != nullptr);
        REQUIRE(basePtr->is<PxD6Joint>());

        d6Joint = basePtr->is<PxD6Joint>();

        {
            PxRigidActor* actor0 = nullptr;
            PxRigidActor* actor1 = nullptr;
            d6Joint->getActors(actor0, actor1);
            CHECK(actor0 == body0);
            CHECK(actor1 == body1);
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    changeTemplate.destroy();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

TEST_CASE("Create D6 Joint At Path",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
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
    REQUIRE(pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit)));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

    // create rigid body - start as static
    addRigidBox(stage, "/World/box0", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath box0Path = SdfPath("/World/box0");
    const SdfPath jointPath = SdfPath("/World/joint");

    SUBCASE("Create valid joint")
    {
        PxD6Joint* joint;
        PxBase* base0Ptr;

        SUBCASE("Rigid dynamic")
        {
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            joint = (PxD6Joint*)(physx->createD6JointAtPath(jointPath, box0Path, SdfPath()));

            base0Ptr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(box0Path, ePTActor));
            CHECK(base0Ptr != nullptr);
            CHECK(base0Ptr->is<PxRigidDynamic>());
        }

        SUBCASE("Articulation link")
        {
            UsdPrim boxPrim = stage->GetPrimAtPath(box0Path);
            CHECK(boxPrim.IsValid());
            UsdPhysicsArticulationRootAPI::Apply(boxPrim);

            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            joint = (PxD6Joint*)(physx->createD6JointAtPath(jointPath, box0Path, SdfPath()));

            base0Ptr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(box0Path, ePTLink));
            CHECK(base0Ptr != nullptr);
            CHECK(base0Ptr->is<PxArticulationLink>());
        }

        REQUIRE(joint);

        PxBase* jointPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtrFast((usdparser::ObjectId)joint->userData));
        CHECK(jointPtr != nullptr);
        CHECK(jointPtr->is<PxD6Joint>());

        PxD6Joint* fixedJoint = jointPtr->is<PxD6Joint>();
        PxRigidActor* actor0 = nullptr;
        PxRigidActor* actor1 = nullptr;

        fixedJoint->getActors(actor0, actor1);
        CHECK(actor0 == base0Ptr);
        CHECK(actor1 == nullptr);
    }

    SUBCASE("Create invalid joint")
    {
        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        physicsTests.getErrorEventListener().setEnabled(false);

        PxD6Joint* joint = (PxD6Joint*)(physx->createD6JointAtPath(jointPath, SdfPath(), SdfPath()));
        CHECK(joint == nullptr);

        PxBase* base0Ptr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(box0Path, ePTActor));
        CHECK(base0Ptr != nullptr);
        CHECK(base0Ptr->is<PxRigidDynamic>());

        PxBase* jointPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
        CHECK(jointPtr == nullptr);

        physxSim->simulate(1.0f/60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();

        physicsTests.getErrorEventListener().setEnabled(true);
    }

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
