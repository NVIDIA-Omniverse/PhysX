// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"
#include "../../common/PhysicsChangeTemplate.h"

#include "PhysicsTools.h"

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/PhysxTokens.h>

#include <omni/fabric/SimStageWithHistory.h>
#include <usdrt/hierarchy/IFabricHierarchy.h>


using namespace pxr;
using namespace omni::physx;
using namespace ::physx;
using namespace carb;
using namespace omni::fabric;
//-----------------------------------------------------------------------------
// fabric changes tests
TEST_CASE("Fabric Changes Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    FabricChange fabricTemplate;

    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    ScopedPopulationActivation scopedPopulation;

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

    // create rigid bodies with colliders
    const uint32_t numBodies = 100;
    std::vector<SdfPath> bodiesPaths;
    for (uint32_t i = 0; i < numBodies; i++)
    {
        std::string boxPath = "/World/box" + std::to_string(i);
        addRigidBox(stage, boxPath, GfVec3f(100.f), GfVec3f(200.0f * i), GfQuatf(1.0f),
            GfVec3f(0.7f), 0.001f);

        const SdfPath boxPathSdf = SdfPath(boxPath.c_str());

        bodiesPaths.push_back(boxPathSdf);

        UsdPrim boxPrim = stage->GetPrimAtPath(boxPathSdf);

        // get rigid body API's thats going to be tested
        PhysxSchemaPhysxRigidBodyAPI api = PhysxSchemaPhysxRigidBodyAPI::Apply(boxPrim);
        api.GetDisableGravityAttr().Set(true);
    }
    fabricTemplate.init(stageId, physicsTests.getApp()->getFramework());
    for (const SdfPath& path : bodiesPaths)
    {
        fabricTemplate.initPrim(path);
    }

    const Type typeAppliedSchema(BaseDataType::eTag, 1, 0, AttributeRole::eAppliedSchema);
    const Token tokenRigidBody("PhysicsRigidBodyAPI");
    omni::fabric::Type float3Type(omni::fabric::BaseDataType::eFloat, 3, 0);
    omni::fabric::Type double3Type(omni::fabric::BaseDataType::eDouble, 3, 0);
    omni::fabric::Type float4Type(omni::fabric::BaseDataType::eFloat, 4, 0);
    omni::fabric::Type matrix4dType(omni::fabric::BaseDataType::eDouble, 16, 0, omni::fabric::AttributeRole::eMatrix);

    // setup velocity attr
    const omni::fabric::set<AttrNameAndType_v2> requiredAll = { AttrNameAndType_v2(typeAppliedSchema, tokenRigidBody) };
    omni::fabric::Token velToken(asInt(UsdPhysicsTokens->physicsVelocity));
    omni::fabric::Token localMatrixToken(omni::physx::gLocalMatrixTokenString);


    StageReaderWriter stageRW = fabricTemplate.iStageReaderWriter->get(fabricTemplate.mStageId);
    {
        for (size_t i = 0; i < bodiesPaths.size(); i++)
        {
            const SdfPath& path = bodiesPaths[i];
            fabricTemplate.iStageReaderWriter->createAttribute(stageRW.getId(), omni::fabric::asInt(path), localMatrixToken, omni::fabric::TypeC(matrix4dType));
            fabricTemplate.iStageReaderWriter->createAttribute(stageRW.getId(), omni::fabric::asInt(path), velToken, omni::fabric::TypeC(float3Type));
        }

        const omni::fabric::set<AttrNameAndType_v2> requiredAny = { AttrNameAndType_v2(matrix4dType, localMatrixToken),
                                                        AttrNameAndType_v2(float3Type, velToken) };
        PrimBucketList primBuckets = stageRW.findPrims(requiredAll, requiredAny);
        size_t bucketCount = primBuckets.bucketCount();
        for (size_t i = 0; i != bucketCount; i++)
        {
            auto vels = stageRW.getAttributeArrayWr<Float3>(primBuckets, i, velToken);
            auto mtx = stageRW.getAttributeArrayWr<pxr::GfMatrix4d>(primBuckets, i, localMatrixToken);
            REQUIRE(vels.size() == numBodies);
            REQUIRE(mtx.size() == numBodies);
        }
    }


    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);

    SUBCASE("Velocity Change")
    {
        PrimBucketList primBuckets = stageRW.findPrims(requiredAll);
        size_t bucketCount = primBuckets.bucketCount();
        for (size_t i = 0; i != bucketCount; i++)
        {
            gsl::span<const omni::fabric::Path> paths = stageRW.getPathArray(primBuckets, i);

            gsl::span<carb::Float3> linVelocities = stageRW.getAttributeArray<carb::Float3>(primBuckets, i, velToken);

            size_t j = 0;
            for (const omni::fabric::Path& path : paths)
            {
                linVelocities[j].x = float(j);
                linVelocities[j].y = float(j);
                linVelocities[j].z = float(j);

                j++;
            }
        }

        for (size_t i = 0; i < bodiesPaths.size(); i++)
        {
            // get the dynamic actor
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(bodiesPaths[i], ePTActor));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidDynamic>());

            PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
            compare(actor->getLinearVelocity(), PxVec3(0.0f, 0.0f, 0.0f), epsilon);
        }

        physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();


        for (size_t i = 0; i != bucketCount; i++)
        {
            gsl::span<const omni::fabric::Path> paths = stageRW.getPathArray(primBuckets, i);

            size_t j = 0;
            for (const omni::fabric::Path& path : paths)
            {
                const pxr::SdfPath primPath = omni::fabric::toSdfPath(path);
                // get the dynamic actor
                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(primPath, ePTActor));
                CHECK(basePtr != nullptr);
                CHECK(basePtr->is<PxRigidDynamic>());

                PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
                compare(actor->getLinearVelocity(), PxVec3(float(j), float(j), float(j)), epsilon);
                j++;
            }
        }
    }

    SUBCASE("Velocity Change Listener Paused")
    {
        physxSim->pauseChangeTracking(true);

        {
            PrimBucketList primBuckets = stageRW.findPrims(requiredAll);
            size_t bucketCount = primBuckets.bucketCount();
            for (size_t i = 0; i != bucketCount; i++)
            {
                gsl::span<const omni::fabric::Path> paths = stageRW.getPathArray(primBuckets, i);

                gsl::span<carb::Float3> linVelocities = stageRW.getAttributeArray<carb::Float3>(primBuckets, i, velToken);

                size_t j = 0;
                for (const omni::fabric::Path& path : paths)
                {
                    linVelocities[j].x = float(j);
                    linVelocities[j].y = float(j);
                    linVelocities[j].z = float(j);

                    j++;
                }
            }

            for (size_t i = 0; i < bodiesPaths.size(); i++)
            {
                // get the dynamic actor
                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(bodiesPaths[i], ePTActor));
                CHECK(basePtr != nullptr);
                CHECK(basePtr->is<PxRigidDynamic>());

                PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
                compare(actor->getLinearVelocity(), PxVec3(0.0f, 0.0f, 0.0f), epsilon);
            }

            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
            physxSim->fetchResults();

            // we have a paused tracking, we should not get any updates
            for (size_t i = 0; i != bucketCount; i++)
            {
                gsl::span<const omni::fabric::Path> paths = stageRW.getPathArray(primBuckets, i);

                size_t j = 0;
                for (const omni::fabric::Path& path : paths)
                {
                    const pxr::SdfPath primPath = omni::fabric::toSdfPath(path);
                    // get the dynamic actor
                    PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(primPath, ePTActor));
                    CHECK(basePtr != nullptr);
                    CHECK(basePtr->is<PxRigidDynamic>());

                    PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
                    compare(actor->getLinearVelocity(), PxVec3(0.0f, 0.0f, 0.0f), epsilon);
                    j++;
                }
            }
        }

        physxSim->pauseChangeTracking(false);

        // resumed tracking we should get updates
        {
            PrimBucketList primBuckets = stageRW.findPrims(requiredAll);
            size_t bucketCount = primBuckets.bucketCount();
            for (size_t i = 0; i != bucketCount; i++)
            {
                gsl::span<const omni::fabric::Path> paths = stageRW.getPathArray(primBuckets, i);

                gsl::span<carb::Float3> linVelocities = stageRW.getAttributeArray<carb::Float3>(primBuckets, i, velToken);

                size_t j = 0;
                for (const omni::fabric::Path& path : paths)
                {
                    linVelocities[j].x = float(j + 1);
                    linVelocities[j].y = float(j + 1);
                    linVelocities[j].z = float(j + 1);

                    j++;
                }
            }

            for (size_t i = 0; i < bodiesPaths.size(); i++)
            {
                // get the dynamic actor
                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(bodiesPaths[i], ePTActor));
                CHECK(basePtr != nullptr);
                CHECK(basePtr->is<PxRigidDynamic>());

                PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
                compare(actor->getLinearVelocity(), PxVec3(0.0f, 0.0f, 0.0f), epsilon);
            }

            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
            physxSim->fetchResults();


            for (size_t i = 0; i != bucketCount; i++)
            {
                gsl::span<const omni::fabric::Path> paths = stageRW.getPathArray(primBuckets, i);

                size_t j = 0;
                for (const omni::fabric::Path& path : paths)
                {
                    const pxr::SdfPath primPath = omni::fabric::toSdfPath(path);
                    // get the dynamic actor
                    PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(primPath, ePTActor));
                    CHECK(basePtr != nullptr);
                    CHECK(basePtr->is<PxRigidDynamic>());

                    PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
                    compare(actor->getLinearVelocity(), PxVec3(float(j + 1), float(j + 1), float(j + 1)), epsilon);
                    j++;
                }
            }
        }
    }

    SUBCASE("Velocity Change Indices")
    {
        PrimBucketList primBuckets = stageRW.findPrims(requiredAll);
        size_t bucketCount = primBuckets.bucketCount();
        for (size_t i = 0; i != bucketCount; i++)
        {
            gsl::span<const omni::fabric::Path> paths = stageRW.getPathArray(primBuckets, i);

            size_t j = 0;
            for (const omni::fabric::Path& path : paths)
            {
                if (j % 10 == 0)
                {
                    Float3& linVelocity = *(Float3*)(stageRW.getAttributeWr<Float3>(path, velToken));
                    linVelocity.x = float(j);
                    linVelocity.y = float(j);
                    linVelocity.z = float(j);
                }

                j++;
            }
        }

        for (size_t i = 0; i < bodiesPaths.size(); i++)
        {
            // get the dynamic actor
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(bodiesPaths[i], ePTActor));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidDynamic>());

            PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
            compare(actor->getLinearVelocity(), PxVec3(0.0f, 0.0f, 0.0f), epsilon);
        }

        physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();


        for (size_t i = 0; i != bucketCount; i++)
        {
            gsl::span<const omni::fabric::Path> paths = stageRW.getPathArray(primBuckets, i);

            size_t j = 0;
            for (const omni::fabric::Path& path : paths)
            {
                const pxr::SdfPath primPath = omni::fabric::toSdfPath(path);
                // get the dynamic actor
                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(primPath, ePTActor));
                CHECK(basePtr != nullptr);
                CHECK(basePtr->is<PxRigidDynamic>());

                PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
                if (j % 10 == 0)
                    compare(actor->getLinearVelocity(), PxVec3(float(j), float(j), float(j)), epsilon);
                else
                    compare(actor->getLinearVelocity(), PxVec3(0.0f, 0.0f, 0.0f), epsilon);
                j++;
            }
        }
    }

    SUBCASE("Transformation Change")
    {
        const float sqrt_two = sqrtf(2.0f) / 2.0f;

        PrimBucketList primBuckets = stageRW.findPrims(requiredAll);
        size_t bucketCount = primBuckets.bucketCount();
        for (size_t i = 0; i != bucketCount; i++)
        {
            gsl::span<const omni::fabric::Path> paths = stageRW.getPathArray(primBuckets, i);

            gsl::span<pxr::GfMatrix4d> matrices = stageRW.getAttributeArray<pxr::GfMatrix4d>(primBuckets, i, localMatrixToken);

            size_t j = 0;
            for (const omni::fabric::Path& path : paths)
            {
                matrices[j].SetTranslate({double(j) * 300., double(j) * 300., double(j) * 300.});
                matrices[j].SetRotateOnly({sqrt_two, 0., 0., sqrt_two});

                j++;
            }
        }

        fabricTemplate.flushXforms();

        for (size_t i = 0; i < bodiesPaths.size(); i++)
        {
            // get the dynamic actor
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(bodiesPaths[i], ePTActor));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidDynamic>());

            PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
            PxTransform globalPose = actor->getGlobalPose();
            compare(globalPose.p, PxVec3(float(i) * 200.0f, float(i) * 200.0f, float(i) * 200.0f), epsilon);
            compare(globalPose.q, GfQuatf(1.0f), epsilon);
        }

        physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();


        for (size_t i = 0; i != bucketCount; i++)
        {
            gsl::span<const omni::fabric::Path> paths = stageRW.getPathArray(primBuckets, i);

            size_t j = 0;
            for (const omni::fabric::Path& path : paths)
            {
                const pxr::SdfPath primPath = omni::fabric::toSdfPath(path);
                // get the dynamic actor
                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(primPath, ePTActor));
                CHECK(basePtr != nullptr);
                CHECK(basePtr->is<PxRigidDynamic>());

                PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
                PxTransform globalPose = actor->getGlobalPose();
                compare(globalPose.p, PxVec3(float(j) * 300.0f, float(j) * 300.0f, float(j) * 300.0f), epsilon);
                compare(globalPose.q, PxQuat(0.0f, 0.0f, sqrt_two, sqrt_two), epsilon);
                j++;
            }
        }
    }

    SUBCASE("Transformation Change Indices")
    {
        const float sqrt_two = sqrtf(2.0f) / 2.0f;

        for (size_t j = 0; j < bodiesPaths.size(); j++)
        {
            if (j % 10 == 0)
            {
                pxr::GfMatrix4d& matrices = *(pxr::GfMatrix4d*)(stageRW.getAttributeWr<pxr::GfMatrix4d>(omni::fabric::asInt(bodiesPaths[j]), localMatrixToken));

                matrices.SetTranslate({0., double(j) * 300.0, 0.});
                matrices.SetRotateOnly({sqrt_two, 0., 0., sqrt_two});
            }
        }

        fabricTemplate.flushXforms();

        for (size_t i = 0; i < bodiesPaths.size(); i++)
        {
            // get the dynamic actor
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(bodiesPaths[i], ePTActor));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidDynamic>());

            PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
            PxTransform globalPose = actor->getGlobalPose();
            compare(globalPose.p, PxVec3(float(i) * 200.0f, float(i) * 200.0f, float(i) * 200.0f), epsilon);
            compare(globalPose.q, GfQuatf(1.0f), epsilon);
        }

        physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();


        for (size_t j = 0; j < bodiesPaths.size(); j++)
        {
            const pxr::SdfPath primPath = bodiesPaths[j];
            // get the dynamic actor
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(primPath, ePTActor));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidDynamic>());

            PxRigidDynamic* actor = basePtr->is<PxRigidDynamic>();
            PxTransform globalPose = actor->getGlobalPose();
            if (j % 10 == 0)
            {
                compare(globalPose.p, PxVec3(0.0f, float(j) * 300.0f, 0.0f), epsilon);
                compare(globalPose.q, PxQuat(0.0f, 0.0f, sqrt_two, sqrt_two), epsilon);
            }
            else
            {
                compare(globalPose.p, PxVec3(float(j) * 200.0f, float(j) * 200.0f, float(j) * 200.0f), epsilon);
                compare(globalPose.q, GfQuatf(1.0f), epsilon);
            }
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

