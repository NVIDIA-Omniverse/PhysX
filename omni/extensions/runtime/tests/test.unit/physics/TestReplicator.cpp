// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/Defines.h>

#include "../common/TestHelpers.h"

#include "PhysicsTools.h"
#include "../../common/PhysicsReplicatorTemplate.h"

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxTests.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxReplicator.h>
#include <private/omni/physx/PhysXCompoundShape.h>
#include <omni/physx/PhysxTokens.h>
#include <physicsSchemaTools/physicsSchemaTokens.h>
#include <common/foundation/TypeCast.h>

#include <regex>

using namespace pxr;
using namespace omni::physx;
using namespace ::physx;

//-----------------------------------------------------------------------------
// Replicator rigid body tests
// TEST_CASE_TEMPLATE("Replicator RigidBody Tests", T, USDReplicator, FabricReplicator)
// {
//     ScopedPopulationActivation populationUtilsActivation;

//     const float epsilon = 0.0001f;

//     T replicatorTemplate;

//     PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
//     IPhysx* physx = physicsTests.acquirePhysxInterface();
//     REQUIRE(physx);
//     IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
//     REQUIRE(physxSim);
//     IPhysxReplicator* physxReplicator = physicsTests.acquirePhysxReplicatorInterface();
//     REQUIRE(physxReplicator);

//     // setup basic stage
//     UsdStageRefPtr stage = UsdStage::CreateInMemory();
//     pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
//     const float metersPerStageUnit = 0.01f; // work in default centimeters
//     const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
//     const SdfPath defaultPrimPath = SdfPath("/World");
//     UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
//     stage->SetDefaultPrim(defaultPrim);

//     pxr::UsdUtilsStageCache::Get().Insert(stage);
//     long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

//     const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
//     UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

//     UsdGeomScope::Define(stage, SdfPath("/World/envs"));

//     // create rigid bodies with materials
//     {
//         const std::string envPath = "/World/envs/env0";
//         UsdGeomXform xform = UsdGeomXform::Define(stage, SdfPath(envPath));
//         xform.AddTranslateOp().Set(GfVec3d(0.0));

//         addRigidBox(stage, envPath + "/box", GfVec3f(0.1f), GfVec3f(0.0f), GfQuatf(1.0f),
//             GfVec3f(0.7f), 0.001f);

//         // material
//         const SdfPath materialPath = SdfPath(envPath + "/material");
//         UsdShadeMaterial basePhysicsMaterial = UsdShadeMaterial::Define(stage, materialPath);
//         UsdPhysicsMaterialAPI::Apply(basePhysicsMaterial.GetPrim());

//         UsdPrim cubePrim = stage->GetPrimAtPath(SdfPath(envPath + "/box"));

//         UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(cubePrim.GetPrim());
//         bindingAPI.Bind(basePhysicsMaterial, UsdShadeTokens->weakerThanDescendants, TfToken("physics"));
//     }

//     const SdfPath cloneEnv = SdfPath("/World/envs/env0");

//     replicatorTemplate.init(stageId, physicsTests.getApp()->getFramework());
//     replicatorTemplate.clone(SdfPath("/World/envs"), cloneEnv, 6, 1, 1.0f);    


//     SUBCASE("Non active filtering")
//     {
//         IReplicatorCallback cb = { nullptr, nullptr, nullptr };
//         physxReplicator->registerReplicator(stageId, cb);

//         if (!replicatorTemplate.isFabric())
//         {
//             SUBCASE("Base parsing")
//             {
//                 // attach sim to stage which parses and creates the pointers that we can check directly
//                 physxSim->attachStage(stageId);

//                 PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
//                 REQUIRE(basePtr != nullptr);

//                 PxScene* scene = (PxScene*)basePtr;
//                 REQUIRE(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 6);

//                 for (int i = 0; i < 6; i++)
//                 {
//                     const std::string envPath = "/World/envs/env" + std::to_string(i);

//                     basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath(envPath + "/box"), ePTActor));
//                     REQUIRE(basePtr != nullptr);
//                     CHECK(basePtr->is<PxRigidDynamic>());

//                     basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath(envPath + "/box"), ePTShape));
//                     REQUIRE(basePtr != nullptr);
//                     const PxShape* shape = basePtr->is<PxShape>();
//                     REQUIRE(shape != nullptr);
//                     REQUIRE(shape->getGeometry().getType() == PxGeometryType::eBOX);

//                     basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath(envPath + "/material"), ePTMaterial));
//                     REQUIRE(basePtr != nullptr);
//                     REQUIRE(basePtr->is<PxMaterial>());
//                     REQUIRE(shape->getNbMaterials() == 1);
//                     PxMaterial* mat;
//                     shape->getMaterials(&mat, 1);
//                     CHECK(mat == basePtr);
//                 }
//             }
//         }
//     }

//     SUBCASE("Active filtering")
//     {
//         std::vector<carb::Float3> positions = {
//             carb::Float3{ 0.0f, 0.0f, 0.0f }, carb::Float3{ 1.0f, 0.0f, 0.0f },
//             carb::Float3{ 2.0f, 0.0f, 0.0f }, carb::Float3{ 3.0f, 0.0f, 0.0f },
//             carb::Float3{ 4.0f, 0.0f, 0.0f }, carb::Float3{ 5.0f, 0.0f, 0.0f } };

//         std::vector<carb::Float4> orientations = {
//             carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f }, carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f },
//             carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f }, carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f },
//             carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f }, carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f },
//         };

//         IReplicatorCallback cb = { nullptr, nullptr, nullptr };

//         cb.replicationAttachFn = [](uint64_t stageId, uint32_t& numExludePaths, uint64_t*& excludePaths, void* userData)
//         {
//             numExludePaths = 1;
//             const SdfPath boxPath = SdfPath("/World/envs");
//             static uint64_t excludePath = sdfPathToInt(boxPath);
//             excludePaths = &excludePath;
//         };

//         cb.hierarchyRenameFn = [](uint64_t replicatePath, uint32_t index, void* userData)
//         {
//             std::string stringPath = "/World/envs/env" + std::to_string(index + 1);
//             const SdfPath outPath(stringPath);
//             return sdfPathToInt(outPath);
//         };

//         physxReplicator->registerReplicator(stageId, cb);

//         if (!replicatorTemplate.isFabric())
//         {
//             SUBCASE("Base parsing")
//             {
//                 // attach sim to stage which parses and creates the pointers that we can check directly
//                 physxSim->attachStage(stageId);

//                 PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
//                 REQUIRE(basePtr != nullptr);
//                 PxScene* scene = (PxScene*)basePtr;
//                 CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
//             }

//             SUBCASE("Base parsing replicator re-attach")
//             {
//                 physxReplicator->registerReplicator(stageId, cb);

//                 // attach sim to stage which parses and creates the pointers that we can check directly
//                 physxSim->attachStage(stageId);

//                 PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
//                 REQUIRE(basePtr != nullptr);
//                 PxScene* scene = (PxScene*)basePtr;
//                 CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
//             }
//         }

//         SUBCASE("Replication filtering check")
//         {
//             // create a box outside, check if replication does replicate only the hierarchy
//             addRigidBox(stage, "/World/boxOut", GfVec3f(0.1f), GfVec3f(0.0f), GfQuatf(1.0f),
//                 GfVec3f(0.7f), 0.001f);
//             const SdfPath boxOutPath("/World/boxOut");

//             // attach sim to stage which parses and creates the pointers that we can check directly
//             physxSim->attachStage(stageId);

//             PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
//             REQUIRE(basePtr != nullptr);
//             PxScene* scene = (PxScene*)basePtr;
//             CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 1);

//             basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxOutPath, ePTActor));
//             CHECK(basePtr != nullptr);

//             basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxOutPath, ePTShape));
//             CHECK(basePtr != nullptr);

//             replicatorTemplate.replicate(sdfPathToInt(cloneEnv), 5);


//             CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 7);

//             for (int i = 0; i < 6; i++)
//             {
//                 const std::string envPath = "/World/envs/env" + std::to_string(i);

//                 basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath(envPath + "/box"), ePTActor));
//                 CHECK(basePtr != nullptr);

//                 basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath(envPath + "/box"), ePTShape));
//                 REQUIRE(basePtr != nullptr);
//             }
//         }

//         SUBCASE("Base RigidBody Replication")
//         {
//             // attach sim to stage which parses and creates the pointers that we can check directly
//             physxSim->attachStage(stageId);

//             PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
//             REQUIRE(basePtr != nullptr);
//             PxScene* scene = (PxScene*)basePtr;
//             CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 0);

//             replicatorTemplate.replicate(sdfPathToInt(cloneEnv), 5);

//             for (int i = 0; i < 6; i++)
//             {
//                 const std::string envPath = "/World/envs/env" + std::to_string(i);

//                 basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath(envPath + "/box"), ePTActor));
//                 CHECK(basePtr != nullptr);

//                 basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath(envPath + "/box"), ePTShape));
//                 const PxShape* shape = basePtr->is<PxShape>();
//                 REQUIRE(shape != nullptr);
//                 REQUIRE(shape->getGeometry().getType() == PxGeometryType::eBOX);

//                 basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath(envPath + "/material"), ePTMaterial));
//                 REQUIRE(basePtr != nullptr);
//                 REQUIRE(basePtr->is<PxMaterial>());
//                 REQUIRE(shape->getNbMaterials() == 1);
//                 PxMaterial* mat;
//                 shape->getMaterials(&mat, 1);
//                 CHECK(mat == basePtr);
//             }
//         }

//         SUBCASE("Initial Position RigidBody Replication")
//         {
//             // attach sim to stage which parses and creates the pointers that we can check directly
//             physxSim->attachStage(stageId);

//             PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
//             REQUIRE(basePtr != nullptr);
//             PxScene* scene = (PxScene*)basePtr;

//             replicatorTemplate.replicate(sdfPathToInt(cloneEnv), 5);

//             REQUIRE(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 6);

//             PxActor* actor = nullptr;
//             for (int i = 0; i < 6; i++)
//             {
//                 scene->getActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC, &actor, 1, i);
//                 PxRigidActor* rbo = actor->is<PxRigidActor>();
//                 const PxTransform tr = rbo->getGlobalPose();
//                 CHECK(GfIsClose(omni::physx::toVec3f(tr.p), omni::physx::toVec3f(positions[i]), epsilon));
//             }
//         }

//         if (!replicatorTemplate.isFabric())
//         {
//             SUBCASE("RigidBody Replication API Cache")
//             {
//                 // attach sim to stage which parses and creates the pointers that we can check directly
//                 physxSim->attachStage(stageId);

//                 PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
//                 REQUIRE(basePtr != nullptr);
//                 PxScene* scene = (PxScene*)basePtr;

//                 replicatorTemplate.replicate(sdfPathToInt(cloneEnv), 5);

//                 REQUIRE(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 6);

//                 PxActor* actors[6];
//                 scene->getActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC, &actors[0], 6, 0);

//                 for (int i = 1; i < 6; i++)
//                 {
//                     const std::string envPath = "/World/envs/env" + std::to_string(i);

//                     UsdPrim prim = stage->GetPrimAtPath(SdfPath(envPath + "/box"));

//                     REQUIRE(prim);

//                     // apply the filtering pairs this should not trigger repasring if the flags are right
//                     UsdPhysicsFilteredPairsAPI::Apply(prim);
//                 }

//                 REQUIRE(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 6);
//             }

//             SUBCASE("Update Positions RigidBody Replication")
//             {
//                 // attach sim to stage which parses and creates the pointers that we can check directly
//                 physxSim->attachStage(stageId);

//                 PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
//                 REQUIRE(basePtr != nullptr);
//                 PxScene* scene = (PxScene*)basePtr;

//                 replicatorTemplate.replicate(sdfPathToInt(cloneEnv), 5);

//                 REQUIRE(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 6);

//                 for (int k = 0; k < 20; k++)
//                 {
//                     physxSim->simulate(1.0f / 60.0f, k * 1.0f / 60.0f);
//                     physxSim->fetchResults();
//                 }

//                 for (int i = 0; i < 6; i++)
//                 {
//                     const std::string envPath = "/World/envs/env" + std::to_string(i);
//                     const GfVec3f position = getPhysicsPrimPos(stage, SdfPath(envPath + "/box"));
//                     CHECK(position[2] < 0.0f);
//                 }
//             }
//         }

//     }

//     // Common post-test actions
//     physxSim->detachStage();

//     replicatorTemplate.destroy();

//     physxReplicator->unregisterReplicator(stageId);

//     pxr::UsdUtilsStageCache::Get().Erase(stage);
//     stage = nullptr;
// }

//-----------------------------------------------------------------------------
// Replicator rigid body tests
TEST_CASE("Replicator RigidBody Compound Shape Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")

{
    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);
    IPhysxReplicator* physxReplicator = physicsTests.acquirePhysxReplicatorInterface();
    REQUIRE(physxReplicator);

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

    UsdGeomScope::Define(stage, SdfPath("/World/envs"));

    const SdfPath cloneEnv("/World/envs/env0");
    // create rigid body in envs
    for (int i = 0; i < 5; i++)
    {
        // create rigid body with convex decompostion
        std::string meshPath = "/World/envs/env" + std::to_string(i) + "/concaveMesh";
        const SdfPath concaveMeshPath(meshPath);
        pxr::UsdGeomMesh concaveMesh = createConcaveMesh(stage, concaveMeshPath, 100.0f);
        UsdPhysicsRigidBodyAPI::Apply(concaveMesh.GetPrim());
        UsdPhysicsCollisionAPI::Apply(concaveMesh.GetPrim());
        UsdPhysicsMeshCollisionAPI meshCollisionAPI = UsdPhysicsMeshCollisionAPI::Apply(concaveMesh.GetPrim());
        meshCollisionAPI.CreateApproximationAttr().Set(UsdPhysicsTokens->convexDecomposition);
    }    

    const SdfPath meshPath = SdfPath("/World/envs/env0/concaveMesh");

    SUBCASE("Non active filtering")
    {
        IReplicatorCallback cb = { nullptr, nullptr, nullptr };
        physxReplicator->registerReplicator(stageId, cb);

        SUBCASE("Base parsing")
        {
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(basePtr != nullptr);

            PxScene* scene = (PxScene*)basePtr;
            REQUIRE(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 5);

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(meshPath, ePTActor));
            REQUIRE(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidDynamic>());

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(meshPath, ePTShape));
            REQUIRE(basePtr == nullptr);

            void* voidPtr = physx->getPhysXPtr(meshPath, ePTCompoundShape);
            REQUIRE(voidPtr != nullptr);

            PhysXCompoundShape* cShape = (PhysXCompoundShape*)voidPtr;
            CHECK(cShape->getShapes().size() > 1);
        }
    }

    SUBCASE("Active filtering")
    {
        std::vector<carb::Float3> positions = {
            carb::Float3{ 0.0f, 0.0f, 0.0f }, carb::Float3{ 1.0f, 1.0f, 1.0f },
            carb::Float3{ 2.0f, 2.0f, 2.0f }, carb::Float3{ 3.0f, 3.0f, 3.0f },
            carb::Float3{ 4.0f, 4.0f, 4.0f }, carb::Float3{ 5.0f, 5.0f, 5.0f } };

        std::vector<carb::Float4> orientations = {
            carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f }, carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f },
            carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f }, carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f },
            carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f }, carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f },
        };

        IReplicatorCallback cb = { nullptr, nullptr, nullptr };

        cb.replicationAttachFn = [](uint64_t stageId, uint32_t& numExludePaths, uint64_t*& excludePaths, void* userData)
        {
            numExludePaths = 1;
            const SdfPath boxPath = SdfPath("/World/envs");
            static uint64_t excludePath = sdfPathToInt(boxPath);
            excludePaths = &excludePath;
        };

        cb.hierarchyRenameFn = [](uint64_t replicatePath, uint32_t index, void* userData)
        {
            std::string stringPath = "/World/envs/env" + std::to_string(index + 1);
            const SdfPath outPath(stringPath);
            return sdfPathToInt(outPath);
        };

        physxReplicator->registerReplicator(stageId, cb);

        SUBCASE("Base parsing")
        {
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene = (PxScene*)basePtr;
            CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 0);

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(meshPath, ePTActor));
            CHECK(basePtr == nullptr);

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(meshPath, ePTShape));
            CHECK(basePtr == nullptr);

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(meshPath, ePTCompoundShape));
            CHECK(basePtr == nullptr);
        }

        SUBCASE("Base RigidBody Replication")
        {
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene = (PxScene*)basePtr;
            CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 0);

            physxReplicator->replicate(stageId, sdfPathToInt(cloneEnv), 4, false, false);

            for (int i = 0; i < 5; i++)
            {
                std::string meshStringPath = "/World/envs/env" + std::to_string(i) + "/concaveMesh";
                SdfPath meshSdfStringPath(meshStringPath);
                basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(meshSdfStringPath, ePTActor));
                CHECK(basePtr != nullptr);

                basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(meshSdfStringPath, ePTShape));
                CHECK(basePtr == nullptr);

                void* voidPtr = physx->getPhysXPtr(meshSdfStringPath, ePTCompoundShape);
                REQUIRE(voidPtr != nullptr);

                PhysXCompoundShape* cShape = (PhysXCompoundShape*)voidPtr;
                CHECK(cShape->getShapes().size() >= 2);
            }
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    physxReplicator->unregisterReplicator(stageId);

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

void checkState(IPhysx* physx, float epsilon, const SdfPath& env0Box0Path,
    const SdfPath& env0Box1Path, const SdfPath& env1Box0Path, const SdfPath& env1Box1Path, bool shouldCollide)
{
    PxBase* basePtr = nullptr;
    // box0 should be on the ground
    {
        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(env0Box0Path, ePTActor));
        REQUIRE(basePtr != nullptr);

        PxRigidActor* rbo = basePtr->is<PxRigidActor>();
        const PxTransform tr = rbo->getGlobalPose();
        CHECK(GfIsClose(omni::physx::toVec3f(tr.p), GfVec3f(0.0, 0.0, 0.5f), epsilon));
    }
    {
        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(env1Box0Path, ePTActor));
        REQUIRE(basePtr != nullptr);

        PxRigidActor* rbo = basePtr->is<PxRigidActor>();
        const PxTransform tr = rbo->getGlobalPose();
        CHECK(GfIsClose(omni::physx::toVec3f(tr.p), GfVec3f(0.0, 5.0, 0.5f), epsilon));
    }

    // box1 should be on the box0
    if (shouldCollide)
    {
        {
            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(env0Box1Path, ePTActor));
            REQUIRE(basePtr != nullptr);

            PxRigidActor* rbo = basePtr->is<PxRigidActor>();
            const PxTransform tr = rbo->getGlobalPose();
            CHECK(GfIsClose(omni::physx::toVec3f(tr.p), GfVec3f(0.0, 0.0, 1.5f), epsilon));
        }
        {
            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(env1Box1Path, ePTActor));
            REQUIRE(basePtr != nullptr);

            PxRigidActor* rbo = basePtr->is<PxRigidActor>();
            const PxTransform tr = rbo->getGlobalPose();
            CHECK(GfIsClose(omni::physx::toVec3f(tr.p), GfVec3f(0.0, 5.0, 1.5f), epsilon));
        }
    }
    else
    {
        {
            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(env0Box1Path, ePTActor));
            REQUIRE(basePtr != nullptr);

            PxRigidActor* rbo = basePtr->is<PxRigidActor>();
            const PxTransform tr = rbo->getGlobalPose();
            CHECK(GfIsClose(omni::physx::toVec3f(tr.p), GfVec3f(0.0, 0.0, 0.5f), epsilon));
        }
        {
            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(env1Box1Path, ePTActor));
            REQUIRE(basePtr != nullptr);

            PxRigidActor* rbo = basePtr->is<PxRigidActor>();
            const PxTransform tr = rbo->getGlobalPose();
            CHECK(GfIsClose(omni::physx::toVec3f(tr.p), GfVec3f(0.0, 5.0, 0.5f), epsilon));
        }
    }
}

//-----------------------------------------------------------------------------
// Replicator rigid body collision filtering tests 
TEST_CASE("Replicator RigidBody Collision Filtering Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")

{
    const float epsilon = 0.1f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);
    IPhysxReplicator* physxReplicator = physicsTests.acquirePhysxReplicatorInterface();
    REQUIRE(physxReplicator);

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

    UsdGeomScope::Define(stage, SdfPath("/World/envs"));

    // create rigid body in env0
    UsdGeomXform::Define(stage, SdfPath("/World/envs/env0"));
    addRigidBox(stage, "/World/envs/env0/box0", GfVec3f(1.0f), GfVec3f(0.0f, 0.0f, 1.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);    
    addRigidBox(stage, "/World/envs/env0/box1", GfVec3f(1.0f), GfVec3f(0.0f, 0.0f, 3.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    UsdGeomXform::Define(stage, SdfPath("/World/envs/env1"));
    addRigidBox(stage, "/World/envs/env1/box0", GfVec3f(1.0f), GfVec3f(0.0f, 5.0f, 1.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);
    addRigidBox(stage, "/World/envs/env1/box1", GfVec3f(1.0f), GfVec3f(0.0f, 5.0f, 3.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    // create ground plane
    addGroundPlane(stage, "/World/plane");

    const SdfPath env0Box0Path("/World/envs/env0/box0");
    const SdfPath env0Box1Path("/World/envs/env0/box1");
    const SdfPath env1Box0Path("/World/envs/env1/box0");
    const SdfPath env1Box1Path("/World/envs/env1/box1");
    UsdPrim env0Box0Prim = stage->GetPrimAtPath(env0Box0Path);
    UsdPrim env0Box1Prim = stage->GetPrimAtPath(env0Box1Path);
    UsdPrim env1Box0Prim = stage->GetPrimAtPath(env1Box0Path);
    UsdPrim env1Box1Prim = stage->GetPrimAtPath(env1Box1Path);    

    std::vector<carb::Float3> positions = { carb::Float3{ 0.0f, 5.0f, 0.0f } };

    std::vector<carb::Float4> orientations = { carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f } };

    IReplicatorCallback cb = { nullptr, nullptr, nullptr };

    cb.replicationAttachFn = [](uint64_t stageId, uint32_t& numExludePaths, uint64_t*& excludePaths, void* userData)
    {
        numExludePaths = 1;
        const SdfPath boxPath = SdfPath("/World/envs");
        static uint64_t excludePath = sdfPathToInt(boxPath);
        excludePaths = &excludePath;
    };

    cb.hierarchyRenameFn = [](uint64_t replicatePath, uint32_t index, void* userData)
    {
        std::string stringPath = "/World/envs/env" + std::to_string(index + 1);
        const SdfPath outPath(stringPath);
        return sdfPathToInt(outPath);
    };


    UsdPhysicsFilteredPairsAPI filteringPairsAPI0 = UsdPhysicsFilteredPairsAPI::Apply(env0Box0Prim);
    UsdPhysicsFilteredPairsAPI filteringPairsAPI1 = UsdPhysicsFilteredPairsAPI::Apply(env1Box0Prim);

    SUBCASE("No Replication")
    {
        SUBCASE("Pair Filtering Disabled")
        {
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene = (PxScene*)basePtr;
            CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 5);

            for (int k = 0; k < 20; k++)
            {
                physxSim->simulate(1.0f / 60.0f, k * 1.0f / 60.0f);
                physxSim->fetchResults();
            }

            checkState(physx, epsilon, env0Box0Path, env0Box1Path, env1Box0Path, env1Box1Path, true);
        }

        SUBCASE("Pair Filtering Enabled")
        {
            filteringPairsAPI0.CreateFilteredPairsRel().AddTarget(env0Box1Path);
            filteringPairsAPI1.CreateFilteredPairsRel().AddTarget(env1Box1Path);

            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene = (PxScene*)basePtr;
            CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 5);

            for (int k = 0; k < 20; k++)
            {
                physxSim->simulate(1.0f / 60.0f, k * 1.0f / 60.0f);
                physxSim->fetchResults();
            }

            checkState(physx, epsilon, env0Box0Path, env0Box1Path, env1Box0Path, env1Box1Path, false);
        }

    }

    SUBCASE("Replication")
    {
        // enable collision filtering
        filteringPairsAPI0.CreateFilteredPairsRel().AddTarget(env0Box1Path);
        filteringPairsAPI1.CreateFilteredPairsRel().AddTarget(env1Box1Path);

        physxReplicator->registerReplicator(stageId, cb);


        SUBCASE("Pair Filtering Enabled")
        {
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene = (PxScene*)basePtr;
            CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 1);

            physxReplicator->replicate(stageId, sdfPathToInt(SdfPath("/World/envs/env0")), 1, false, false);

            CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 5);

            for (int k = 0; k < 20; k++)
            {
                physxSim->simulate(1.0f / 60.0f, k * 1.0f / 60.0f);
                physxSim->fetchResults();
            }

            checkState(physx, epsilon, env0Box0Path, env0Box1Path, env1Box0Path, env1Box1Path, false);
        }

    }    

    // Common post-test actions
    physxSim->detachStage();
    physxReplicator->unregisterReplicator(stageId);


    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

//-----------------------------------------------------------------------------
// Replicator rigid body collision groups tests 
TEST_CASE("Replicator RigidBody Collision Groups Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")

{
    const float epsilon = 0.1f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);
    IPhysxReplicator* physxReplicator = physicsTests.acquirePhysxReplicatorInterface();
    REQUIRE(physxReplicator);

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

    UsdGeomScope::Define(stage, SdfPath("/World/envs"));

    // create rigid body in env0
    UsdGeomXform::Define(stage, SdfPath("/World/envs/env0"));
    addRigidBox(stage, "/World/envs/env0/box0", GfVec3f(1.0f), GfVec3f(0.0f, 0.0f, 1.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);
    addRigidBox(stage, "/World/envs/env0/box1", GfVec3f(1.0f), GfVec3f(0.0f, 5.0f, 1.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    UsdGeomXform::Define(stage, SdfPath("/World/envs/env1"));
    addRigidBox(stage, "/World/envs/env1/box0", GfVec3f(1.0f), GfVec3f(0.0f, 0.0f, 3.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);
    addRigidBox(stage, "/World/envs/env1/box1", GfVec3f(1.0f), GfVec3f(0.0f, 5.0f, 3.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    // create ground plane
    addGroundPlane(stage, "/World/plane");

    const SdfPath env0Box0Path("/World/envs/env0/box0");
    const SdfPath env0Box1Path("/World/envs/env0/box1");
    const SdfPath env1Box0Path("/World/envs/env1/box0");
    const SdfPath env1Box1Path("/World/envs/env1/box1");
    UsdPrim env0Box0Prim = stage->GetPrimAtPath(env0Box0Path);
    UsdPrim env0Box1Prim = stage->GetPrimAtPath(env0Box1Path);
    UsdPrim env1Box0Prim = stage->GetPrimAtPath(env1Box0Path);
    UsdPrim env1Box1Prim = stage->GetPrimAtPath(env1Box1Path);

    std::vector<carb::Float3> positions = { carb::Float3{ 0.0f, 0.0f, 3.0f } };

    std::vector<carb::Float4> orientations = { carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f } };

    IReplicatorCallback cb = { nullptr, nullptr, nullptr };

    cb.replicationAttachFn = [](uint64_t stageId, uint32_t& numExludePaths, uint64_t*& excludePaths, void* userData)
    {
        numExludePaths = 1;
        const SdfPath boxPath = SdfPath("/World/envs");
        static uint64_t excludePath = sdfPathToInt(boxPath);
        excludePaths = &excludePath;
    };

    cb.hierarchyRenameFn = [](uint64_t replicatePath, uint32_t index, void* userData)
    {
        std::string stringPath = "/World/envs/env" + std::to_string(index + 1);
        const SdfPath outPath(stringPath);
        return sdfPathToInt(outPath);
    };

    UsdPhysicsCollisionGroup group0 = UsdPhysicsCollisionGroup::Define(stage, SdfPath("/World/CollisionGroup0"));
    UsdPhysicsCollisionGroup group1 = UsdPhysicsCollisionGroup::Define(stage, SdfPath("/World/CollisionGroup1"));

    group0.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(env0Box0Path);
    group0.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(env0Box1Path);
    group1.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(env1Box0Path);
    group1.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(env1Box1Path);

    SUBCASE("No Replication")
    {
        SUBCASE("CollisionGroups Disabled")
        {
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene = (PxScene*)basePtr;
            CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 5);

            for (int k = 0; k < 20; k++)
            {
                physxSim->simulate(1.0f / 60.0f, k * 1.0f / 60.0f);
                physxSim->fetchResults();
            }

            checkState(physx, epsilon, env0Box0Path, env1Box0Path, env0Box1Path, env1Box1Path, true);
        }

        SUBCASE("Pair Filtering Enabled")
        {
            group0.CreateFilteredGroupsRel().AddTarget(group1.GetPrim().GetPrimPath());            

            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene = (PxScene*)basePtr;
            CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 5);

            for (int k = 0; k < 20; k++)
            {
                physxSim->simulate(1.0f / 60.0f, k * 1.0f / 60.0f);
                physxSim->fetchResults();
            }

            checkState(physx, epsilon, env0Box0Path, env1Box0Path, env0Box1Path, env1Box1Path, false);
        }

    }

    SUBCASE("Replication")
    {
        // enable collision filtering
        group0.CreateFilteredGroupsRel().AddTarget(group1.GetPrim().GetPrimPath());

        physxReplicator->registerReplicator(stageId, cb);


        SUBCASE("Pair Filtering Enabled")
        {
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene = (PxScene*)basePtr;
            CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 1);

            physxReplicator->replicate(stageId, sdfPathToInt(SdfPath("/World/envs/env0")), 1, false, false);

            CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 5);

            for (int k = 0; k < 20; k++)
            {
                physxSim->simulate(1.0f / 60.0f, k * 1.0f / 60.0f);
                physxSim->fetchResults();
            }

            checkState(physx, epsilon, env0Box0Path, env1Box0Path, env0Box1Path, env1Box1Path, false);
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    physxReplicator->unregisterReplicator(stageId);

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}


//-----------------------------------------------------------------------------
// Replicator rigid body contact report tests 
TEST_CASE("Replicator RigidBody Contact Report Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    const float epsilon = 0.1f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);
    IPhysxReplicator* physxReplicator = physicsTests.acquirePhysxReplicatorInterface();
    REQUIRE(physxReplicator);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const float metersPerStageUnit = 1.f; // work in default centimeters
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

    UsdGeomScope::Define(stage, SdfPath("/World/envs"));

    // create rigid body in env0
    UsdGeomXform::Define(stage, SdfPath("/World/envs/env0"));
    addRigidBox(stage, "/World/envs/env0/box0", GfVec3f(1.0f), GfVec3f(0.0f, 0.0f, 1.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    UsdGeomXform::Define(stage, SdfPath("/World/envs/env1"));
    addRigidBox(stage, "/World/envs/env1/box0", GfVec3f(1.0f), GfVec3f(0.0f, 5.0f, 1.0f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    // create ground plane
    addGroundPlane(stage, "/World/plane");

    const SdfPath env0Box0Path("/World/envs/env0/box0");
    const SdfPath env1Box0Path("/World/envs/env1/box0");
    UsdPrim env0Box0Prim = stage->GetPrimAtPath(env0Box0Path);
    UsdPrim env1Box0Prim = stage->GetPrimAtPath(env1Box0Path);

    std::vector<carb::Float3> positions = { carb::Float3{ 0.0f, 5.0f, 0.0f } };
    std::vector<carb::Float4> orientations = { carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f } };

    IReplicatorCallback cb = { nullptr, nullptr, nullptr };

    cb.replicationAttachFn = [](uint64_t stageId, uint32_t& numExludePaths, uint64_t*& excludePaths, void* userData)
    {
        numExludePaths = 1;
        const SdfPath boxPath = SdfPath("/World/envs");
        static uint64_t excludePath = sdfPathToInt(boxPath);
        excludePaths = &excludePath;
    };

    cb.hierarchyRenameFn = [](uint64_t replicatePath, uint32_t index, void* userData)
    {
        std::string stringPath = "/World/envs/env" + std::to_string(index + 1);
        const SdfPath outPath(stringPath);
        return sdfPathToInt(outPath);
    };

    PhysxSchemaPhysxContactReportAPI contactReportAPI0 = PhysxSchemaPhysxContactReportAPI::Apply(env0Box0Prim);    
    PhysxSchemaPhysxContactReportAPI contactReportAPI1 = PhysxSchemaPhysxContactReportAPI::Apply(env1Box0Prim);
    contactReportAPI0.CreateThresholdAttr().Set(0.1f);
    contactReportAPI1.CreateThresholdAttr().Set(0.1f);

    const ContactEventHeader* contactEventHeadersBuffer = nullptr;
    const ContactData* contactDataBuffer = nullptr;
    uint32_t numContactHeaders = 0;
    uint32_t numContactData = 0;

    SUBCASE("No Replication")
    {
        SUBCASE("Contact Report")
        {
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene = (PxScene*)basePtr;
            CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 2);

            for (int u = 0; u < 40; u++)
            {
                physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
                physxSim->fetchResults();

                numContactHeaders = physxSim->getContactReport(&contactEventHeadersBuffer, &contactDataBuffer, numContactData);
                if (numContactHeaders)
                    break;
            }

            CHECK(numContactHeaders == 2);

            CHECK(contactEventHeadersBuffer[0].actor0 != contactEventHeadersBuffer[1].actor0);
        }
    }

    SUBCASE("Replication")
    {
        physxReplicator->registerReplicator(stageId, cb);


        SUBCASE("Contact Report")
        {
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            physxReplicator->replicate(stageId, sdfPathToInt(SdfPath("/World/envs/env0")), 1, false, false);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene = (PxScene*)basePtr;

            CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 2);

            for (int u = 0; u < 40; u++)
            {
                physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
                physxSim->fetchResults();

                numContactHeaders = physxSim->getContactReport(&contactEventHeadersBuffer, &contactDataBuffer, numContactData);
                if (numContactHeaders)
                    break;
            }

            CHECK(numContactHeaders == 2);

            CHECK(contactEventHeadersBuffer[0].actor0 != contactEventHeadersBuffer[1].actor0);
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    physxReplicator->unregisterReplicator(stageId);

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}



void createLinkHierarchy(
    const std::string& envPath, int index, UsdStageWeakPtr stage, bool createArticulation, bool disableGravity = false, float offset = 1.0f)
{
    // create rigid body - rootLink
    addRigidBox(stage, envPath + "/rootLink", GfVec3f(0.1f), GfVec3f(float(index) * offset), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath rootLinkPath = SdfPath(envPath + "/rootLink");
    UsdPrim rootLinkPrim = stage->GetPrimAtPath(rootLinkPath);

    const SdfPath fixedJointPath = SdfPath(envPath + "/baseFixedJoint");
    UsdPhysicsFixedJoint fixedJoint = UsdPhysicsFixedJoint::Define(stage, fixedJointPath);
    fixedJoint.GetBody1Rel().AddTarget(rootLinkPath);
    if (!createArticulation)
    {
        addRigidBox(stage, envPath + "/staticAnchor", GfVec3f(0.1f), GfVec3f(float(index)), GfQuatf(1.0f),
            GfVec3f(0.7f), 0.001f);

        const SdfPath staticAnchorPath(envPath + "/staticAnchor");
        UsdPhysicsRigidBodyAPI rboAPI = UsdPhysicsRigidBodyAPI::Get(stage, staticAnchorPath);
        rboAPI.CreateRigidBodyEnabledAttr().Set(false);
        UsdPhysicsCollisionAPI colAPI = UsdPhysicsCollisionAPI::Get(stage, staticAnchorPath);
        colAPI.CreateCollisionEnabledAttr().Set(false);
        fixedJoint.GetBody0Rel().AddTarget(staticAnchorPath);
    }

    // create rigid body - dynamicLink
    addRigidBox(stage, envPath + "/dynamicLink", GfVec3f(0.1f), GfVec3f(float(index) * offset), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath dynamicLinkPath = SdfPath(envPath + "/dynamicLink");
    UsdPrim dynamicLinkPrim = stage->GetPrimAtPath(dynamicLinkPath);

    if (disableGravity)
    {
        PhysxSchemaPhysxRigidBodyAPI rbAPI = PhysxSchemaPhysxRigidBodyAPI::Apply(dynamicLinkPrim);
        rbAPI.CreateDisableGravityAttr().Set(true);
    }

    const SdfPath prismaticJointPath = SdfPath(envPath + "/prismaticJoint");
    UsdPhysicsPrismaticJoint prismaticJoint = UsdPhysicsPrismaticJoint::Define(stage, prismaticJointPath);
    prismaticJoint.GetBody0Rel().AddTarget(rootLinkPath);
    prismaticJoint.GetBody1Rel().AddTarget(dynamicLinkPath);
    prismaticJoint.CreateAxisAttr().Set(UsdPhysicsTokens->z);
    prismaticJoint.GetLowerLimitAttr().Set(-90.0f);
    prismaticJoint.GetUpperLimitAttr().Set(90.0f);
    prismaticJoint.CreateCollisionEnabledAttr().Set(false);

    if (createArticulation)
    {
        UsdPrim artRootPrim = stage->GetPrimAtPath(SdfPath(envPath));
        UsdPhysicsArticulationRootAPI::Apply(artRootPrim);
    }
}

void applyPhysxJointAxisAPI(
    const std::string& envPath, UsdStageWeakPtr stage)
{
    const SdfPath prismaticJointPath = SdfPath(envPath + "/prismaticJoint");
    UsdPhysicsPrismaticJoint prismaticJoint = UsdPhysicsPrismaticJoint::Define(stage, prismaticJointPath);
    
    prismaticJoint.GetPrim().ApplyAPI(PhysxAdditionAPITokens->PhysxJointAxisAPI, TfToken("linear"));
    
    prismaticJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->armatureLinear, SdfValueTypeNames->Float).Set(10.0f);
    prismaticJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->maxJointVelocityLinear, SdfValueTypeNames->Float).Set(10.0f);
    prismaticJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->staticFrictionEffortLinear, SdfValueTypeNames->Float).Set(10.0f);
    prismaticJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->dynamicFrictionEffortLinear, SdfValueTypeNames->Float).Set(10.0f);
    prismaticJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->viscousFrictionCoefficientLinear, SdfValueTypeNames->Float).Set(10.0f);
}

void applyPhysxPerformanceEnvelopeAPI(
    const std::string& envPath, UsdStageWeakPtr stage)
{
    const SdfPath prismaticJointPath = SdfPath(envPath + "/prismaticJoint");
    UsdPhysicsPrismaticJoint prismaticJoint = UsdPhysicsPrismaticJoint::Define(stage, prismaticJointPath);
    UsdPhysicsDriveAPI driveAPI = UsdPhysicsDriveAPI::Apply(prismaticJoint.GetPrim(), TfToken("linear"));

    driveAPI.CreateMaxForceAttr().Set(10.0f);
    driveAPI.CreateStiffnessAttr().Set(10.0f);
    prismaticJoint.GetPrim().ApplyAPI(PhysxAdditionAPITokens->DrivePerformanceEnvelopeAPI, TfToken("linear"));
    
    prismaticJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->maxActuatorVelocityLinear, SdfValueTypeNames->Float).Set(10.0f);
    prismaticJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->velocityDependentResistanceLinear, SdfValueTypeNames->Float).Set(10.0f);
    prismaticJoint.GetPrim().CreateAttribute(PhysxAdditionAttrTokens->speedEffortGradientLinear, SdfValueTypeNames->Float).Set(10.0f);
}


void createFixedTendons(const std::string& envPath, int index, UsdStageWeakPtr stage, const TfToken tendonName)
{
    // create rigid body - dynamicLink
    addRigidBox(stage, envPath + "/dynamicLink0", GfVec3f(0.1f), GfVec3f(float(index)), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath dynamicLinkPath = SdfPath(envPath + "/dynamicLink");
    const SdfPath dynamicLinkPath0 = SdfPath(envPath + "/dynamicLink0");
    UsdPrim dynamicLinkPrim = stage->GetPrimAtPath(dynamicLinkPath);

    const SdfPath prismaticJointPath = SdfPath(envPath + "/prismaticJoint0");
    UsdPhysicsPrismaticJoint prismaticJoint = UsdPhysicsPrismaticJoint::Define(stage, prismaticJointPath);
    prismaticJoint.GetBody0Rel().AddTarget(dynamicLinkPath);
    prismaticJoint.GetBody1Rel().AddTarget(dynamicLinkPath0);
    prismaticJoint.CreateAxisAttr().Set(UsdPhysicsTokens->z);
    prismaticJoint.GetLowerLimitAttr().Set(-90.0f);
    prismaticJoint.GetUpperLimitAttr().Set(90.0f);
    prismaticJoint.CreateCollisionEnabledAttr().Set(false);

    const SdfPath fixedJointPath = SdfPath(envPath + "/prismaticJoint");
    UsdPrim fixedJointPrim = stage->GetPrimAtPath(fixedJointPath);

    pxr::PhysxSchemaPhysxTendonAxisRootAPI rootApi = pxr::PhysxSchemaPhysxTendonAxisRootAPI::Apply(fixedJointPrim, tendonName);
    
    UsdPrim prismaticJointPrim = stage->GetPrimAtPath(prismaticJointPath);

    pxr::PhysxSchemaPhysxTendonAxisAPI axisApi = pxr::PhysxSchemaPhysxTendonAxisAPI::Apply(prismaticJointPrim, tendonName);
}

void createSpatialTendons(const std::string& envPath, int index, UsdStageWeakPtr stage)
{
    // create rigid body - dynamicLink
    addRigidBox(stage, envPath + "/dynamicLink0", GfVec3f(0.1f), GfVec3f(float(index)), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath dynamicLinkPath = SdfPath(envPath + "/dynamicLink");
    const SdfPath dynamicLinkPath0 = SdfPath(envPath + "/dynamicLink0");
    UsdPrim dynamicLinkPrim = stage->GetPrimAtPath(dynamicLinkPath);
    UsdPrim dynamicLinkPrim0 = stage->GetPrimAtPath(dynamicLinkPath0);

    const SdfPath prismaticJointPath = SdfPath(envPath + "/prismaticJoint0");
    UsdPhysicsPrismaticJoint prismaticJoint = UsdPhysicsPrismaticJoint::Define(stage, prismaticJointPath);
    prismaticJoint.GetBody0Rel().AddTarget(dynamicLinkPath);
    prismaticJoint.GetBody1Rel().AddTarget(dynamicLinkPath0);
    prismaticJoint.CreateAxisAttr().Set(UsdPhysicsTokens->z);
    prismaticJoint.GetLowerLimitAttr().Set(-90.0f);
    prismaticJoint.GetUpperLimitAttr().Set(90.0f);
    prismaticJoint.CreateCollisionEnabledAttr().Set(false);

    pxr::PhysxSchemaPhysxTendonAttachmentRootAPI rootApi =
        pxr::PhysxSchemaPhysxTendonAttachmentRootAPI::Apply(dynamicLinkPrim, TfToken("root"));
    
    pxr::PhysxSchemaPhysxTendonAttachmentLeafAPI leafApi =
        pxr::PhysxSchemaPhysxTendonAttachmentLeafAPI::Apply(dynamicLinkPrim0, TfToken("leaf"));
    
    pxr::PhysxSchemaPhysxTendonAttachmentAPI(leafApi, TfToken("leaf")).CreateParentAttachmentAttr().Set(TfToken("root"));
    pxr::PhysxSchemaPhysxTendonAttachmentAPI(leafApi, TfToken("leaf")).CreateParentLinkRel().AddTarget(dynamicLinkPath);
}

//-----------------------------------------------------------------------------
// Replicator Articulation tests
TEST_CASE("Replicator Articulation Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")

{
    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);
    IPhysxReplicator* physxReplicator = physicsTests.acquirePhysxReplicatorInterface();
    REQUIRE(physxReplicator);

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

    UsdGeomScope::Define(stage, SdfPath("/World/envs"));

    for (int i = 0; i < 5; i++)
    {
        createLinkHierarchy("/World/envs/env" + std::to_string(i), i, stage, true);
    }

    std::vector<carb::Float3> positions = {
        carb::Float3{ 0.0f, 0.0f, 0.0f }, carb::Float3{ 1.0f, 1.0f, 1.0f },
        carb::Float3{ 2.0f, 2.0f, 2.0f }, carb::Float3{ 3.0f, 3.0f, 3.0f },
        carb::Float3{ 4.0f, 4.0f, 4.0f }
    };

    std::vector<carb::Float4> orientations = {
        carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f }, carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f },
        carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f }, carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f },
        carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f }
    };

    SUBCASE("Non active filtering")
    {
        IReplicatorCallback cb = { nullptr, nullptr, nullptr };
        physxReplicator->registerReplicator(stageId, cb);

        SUBCASE("Base parsing")
        {          
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);
            physx->forceLoadPhysicsFromUSD();

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene = (PxScene*)basePtr;            

            CHECK(scene->getNbArticulations() == 5);
            REQUIRE(scene->getNbAggregates() == 5);
            CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 0);

            for (int i = 0; i < 5; i++)
            {
                PxArticulationReducedCoordinate* articulation = nullptr;
                scene->getArticulations(&articulation, 1, i);
                CHECK(articulation->getNbLinks() == 2);

                const SdfPath linkPath("/World/envs/env" + std::to_string(i) + "/dynamicLink");

                basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(linkPath, ePTLink));
                REQUIRE(basePtr != nullptr);
                CHECK(basePtr->is<PxArticulationLink>());

                basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(linkPath, ePTShape));
                REQUIRE(basePtr != nullptr);
                CHECK(basePtr->is<PxShape>());
            }
        }
    }

    SUBCASE("Active filtering")
    {
        IReplicatorCallback cb = { nullptr, nullptr, nullptr };

        cb.replicationAttachFn = [](uint64_t stageId, uint32_t& numExludePaths, uint64_t*& excludePaths, void* userData)
        {
            numExludePaths = 1;
            const SdfPath boxPath = SdfPath("/World/envs");
            static uint64_t excludePath = sdfPathToInt(boxPath);
            excludePaths = &excludePath;
        };

        cb.hierarchyRenameFn = [](uint64_t replicatePath, uint32_t index, void* userData)
        {
            std::string stringPath = "/World/envs/env" + std::to_string(index + 1);
            const SdfPath outPath(stringPath);
            return sdfPathToInt(outPath);
        };

        physxReplicator->registerReplicator(stageId, cb);

        SUBCASE("Base parsing")
        {
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);
            physx->forceLoadPhysicsFromUSD();

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene = (PxScene*)basePtr;

            CHECK(scene->getNbArticulations() == 0);
            CHECK(scene->getNbAggregates() == 0);
            CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
        }

        SUBCASE("Base Articulation Replication")
        {
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene = (PxScene*)basePtr;
            CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            CHECK(scene->getNbArticulations() == 0);
            CHECK(scene->getNbAggregates() == 0);

            physxReplicator->replicate(stageId, sdfPathToInt(SdfPath("/World/envs/env0")), 4, false, false);

            CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            REQUIRE(scene->getNbArticulations() == 5);
            CHECK(scene->getNbAggregates() == 5);

            for (int i = 0; i < 5; i++)
            {
                PxArticulationReducedCoordinate* articulation = nullptr;
                scene->getArticulations(&articulation, 1, i);
                CHECK(articulation->getNbLinks() == 2);
            }
        }

        SUBCASE("Physx Joint Axis API parsing")
        {          
            for (int i = 0; i < 1; i++)
            {
                applyPhysxJointAxisAPI("/World/envs/env" + std::to_string(i), stage);
            }
            physxSim->attachStage(stageId);
            physxReplicator->replicate(stageId, sdfPathToInt(SdfPath("/World/envs/env0")), 4, false, false);

            for (int i = 0; i < 5; i++)
            {
                const SdfPath prismaticJointPath("/World/envs/env" + std::to_string(i) + "/prismaticJoint");

                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
                CHECK(basePtr != nullptr);
                REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

                PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

                float arm = joint->getArmature(PxArticulationAxis::eX);
                float vel = joint->getMaxJointVelocity(PxArticulationAxis::eX);
                auto friction = joint->getFrictionParams(PxArticulationAxis::eX);

                CHECK(fabsf(arm - 10.0f) < epsilon);
                CHECK(fabsf(vel - 10.0f) < epsilon);
                CHECK(fabsf(friction.staticFrictionEffort - 10.0f) < epsilon);
                CHECK(fabsf(friction.dynamicFrictionEffort - 10.0f) < epsilon);
                CHECK(fabsf(friction.viscousFrictionCoefficient - 10.0f) < epsilon);
            }
        }

        SUBCASE("Physx Performance Envelope parsing")
        {          
            for (int i = 0; i < 1; i++)
            {
                applyPhysxPerformanceEnvelopeAPI("/World/envs/env" + std::to_string(i), stage);
            }
            physxSim->attachStage(stageId);
            physxReplicator->replicate(stageId, sdfPathToInt(SdfPath("/World/envs/env0")), 4, false, false);

            for (int i = 0; i < 5; i++)
            {
                const SdfPath prismaticJointPath("/World/envs/env" + std::to_string(i) + "/prismaticJoint");

                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(prismaticJointPath, ePTLinkJoint));
                CHECK(basePtr != nullptr);
                REQUIRE(basePtr->is<PxArticulationJointReducedCoordinate>());

                PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();

                auto drive = joint->getDriveParams(PxArticulationAxis::eX);

                printf("result %d, %f", i, drive.envelope.speedEffortGradient);
                CHECK(fabsf(drive.stiffness - 10.0f) < epsilon);
                CHECK(fabsf(drive.envelope.maxActuatorVelocity - 10.0f) < epsilon);
                CHECK(fabsf(drive.envelope.maxEffort - 10.0f) < epsilon);
                CHECK(fabsf(drive.envelope.velocityDependentResistance - 10.0f) < epsilon);
                CHECK(fabsf(drive.envelope.speedEffortGradient - 10.0f) < epsilon);
            }
        }

        SUBCASE("Initial Position Replication")
        {
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene = (PxScene*)basePtr;

            physxReplicator->replicate(stageId, sdfPathToInt(SdfPath("/World/envs/env0")), 4, false, false);

            CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            REQUIRE(scene->getNbArticulations() == 5);
            CHECK(scene->getNbAggregates() == 5);

            for (int i = 0; i < 5; i++)
            {
                PxArticulationReducedCoordinate* articulation = nullptr;
                scene->getArticulations(&articulation, 1, i);
                CHECK(articulation->getNbLinks() == 2);

                const PxTransform tr = articulation->getRootGlobalPose();
                CHECK(GfIsClose(omni::physx::toVec3f(tr.p), omni::physx::toVec3f(positions[i]), epsilon));
            }
        }

        SUBCASE("Path Mapping")
        {
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene = (PxScene*)basePtr;

            physxReplicator->replicate(stageId, sdfPathToInt(SdfPath("/World/envs/env0")), 4, false, false);

            CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            REQUIRE(scene->getNbArticulations() == 5);
            CHECK(scene->getNbAggregates() == 5);

            for (int k = 0; k < 5; k++)
            {
                const SdfPath linkPath("/World/envs/env" + std::to_string(k) + "/dynamicLink");
                const SdfPath jointPath("/World/envs/env" + std::to_string(k) + "/prismaticJoint");
                const SdfPath articulationPath("/World/envs/env" + std::to_string(k));

                basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(linkPath, ePTLink));
                REQUIRE(basePtr != nullptr);
                CHECK(basePtr->is<PxArticulationLink>());

                basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(linkPath, ePTShape));
                REQUIRE(basePtr != nullptr);
                CHECK(basePtr->is<PxShape>());

                basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTLinkJoint));
                REQUIRE(basePtr != nullptr);
                CHECK(basePtr->is<PxArticulationJointReducedCoordinate>());

                basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(articulationPath, ePTArticulation));
                REQUIRE(basePtr != nullptr);
                CHECK(basePtr->is<PxArticulationReducedCoordinate>());
            }
        }

        SUBCASE("Fixed Tendons")
        {
            for (int i = 0; i < 5; i++)
            {
                createFixedTendons("/World/envs/env" + std::to_string(i), i, stage, TfToken("t0"));
            }

            SUBCASE("Path Mapping")
            {
                // attach sim to stage which parses and creates the pointers that we can check directly
                physxSim->attachStage(stageId);

                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
                REQUIRE(basePtr != nullptr);
                PxScene* scene = (PxScene*)basePtr;

                physxReplicator->replicate(stageId, sdfPathToInt(SdfPath("/World/envs/env0")), 4, false, false);

                CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
                REQUIRE(scene->getNbArticulations() == 5);
                CHECK(scene->getNbAggregates() == 5);

                for (int k = 0; k < 5; k++)
                {
                    const SdfPath linkPath("/World/envs/env" + std::to_string(k) + "/dynamicLink");
                    const SdfPath jointPath("/World/envs/env" + std::to_string(k) + "/prismaticJoint");
                    const SdfPath articulationPath("/World/envs/env" + std::to_string(k));

                    basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(linkPath, ePTLink));
                    REQUIRE(basePtr != nullptr);
                    CHECK(basePtr->is<PxArticulationLink>());

                    basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(linkPath, ePTShape));
                    REQUIRE(basePtr != nullptr);
                    CHECK(basePtr->is<PxShape>());

                    basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTLinkJoint));
                    REQUIRE(basePtr != nullptr);
                    CHECK(basePtr->is<PxArticulationJointReducedCoordinate>());

                    basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(articulationPath, ePTArticulation));
                    REQUIRE(basePtr != nullptr);
                    CHECK(basePtr->is<PxArticulationReducedCoordinate>());

                    void* baseVoidPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTFixedTendonAxis));
                    REQUIRE(baseVoidPtr != nullptr);
                }
            }
        }

        SUBCASE("Spatial Tendons")
        {
            for (int i = 0; i < 5; i++)
            {
                createSpatialTendons("/World/envs/env" + std::to_string(i), i, stage);
            }

            SUBCASE("Path Mapping")
            {
                // attach sim to stage which parses and creates the pointers that we can check directly
                physxSim->attachStage(stageId);

                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
                REQUIRE(basePtr != nullptr);
                PxScene* scene = (PxScene*)basePtr;

                physxReplicator->replicate(stageId, sdfPathToInt(SdfPath("/World/envs/env0")), 4, false, false);

                CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
                REQUIRE(scene->getNbArticulations() == 5);
                CHECK(scene->getNbAggregates() == 5);

                for (int k = 0; k < 5; k++)
                {
                    const SdfPath linkPath("/World/envs/env" + std::to_string(k) + "/dynamicLink");
                    const SdfPath linkPath0("/World/envs/env" + std::to_string(k) + "/dynamicLink0");
                    const SdfPath jointPath("/World/envs/env" + std::to_string(k) + "/prismaticJoint");
                    const SdfPath articulationPath("/World/envs/env" + std::to_string(k));

                    basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(linkPath, ePTLink));
                    REQUIRE(basePtr != nullptr);
                    CHECK(basePtr->is<PxArticulationLink>());

                    basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(linkPath, ePTShape));
                    REQUIRE(basePtr != nullptr);
                    CHECK(basePtr->is<PxShape>());

                    basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTLinkJoint));
                    REQUIRE(basePtr != nullptr);
                    CHECK(basePtr->is<PxArticulationJointReducedCoordinate>());

                    basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(articulationPath, ePTArticulation));
                    REQUIRE(basePtr != nullptr);
                    CHECK(basePtr->is<PxArticulationReducedCoordinate>());

                    void* baseVoidPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(linkPath, ePTTendonAttachment));
                    REQUIRE(baseVoidPtr != nullptr);

                    baseVoidPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(linkPath0, ePTTendonAttachment));
                    REQUIRE(baseVoidPtr != nullptr);
                }
            }
        }

        SUBCASE("Update Positions Replication")
        {
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
            REQUIRE(basePtr != nullptr);
            PxScene* scene = (PxScene*)basePtr;

            physxReplicator->replicate(stageId, sdfPathToInt(SdfPath("/World/envs/env0")), 4, false, false);

            CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            REQUIRE(scene->getNbArticulations() == 5);
            CHECK(scene->getNbAggregates() == 5);

            for (int k = 0; k < 20; k++)
            {
                physxSim->simulate(1.0f / 60.0f, k * 1.0f / 60.0f);
                physxSim->fetchResults();
            }

            for (int k = 0; k < 5; k++)
            {
                const SdfPath linkPath("/World/envs/env" + std::to_string(k) + "/dynamicLink");
                const GfVec3f position = getPhysicsPrimPos(stage, linkPath);
                printf("Position: %f, %f, %f\n", position[0], position[1], position[2]);
                CHECK(position[2] < 0.0f);
            }
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    physxReplicator->unregisterReplicator(stageId);

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}


void createGearJoint(const std::string& envPath, int index, UsdStageWeakPtr stage)
{
    // create rigid body 
    addRigidBox(stage, envPath + "/box0", GfVec3f(0.1f), GfVec3f(float(index)), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath boxPath0 = SdfPath(envPath + "/box0");

    const SdfPath revoluteJointPath0 = SdfPath(envPath + "/revoluteJoint0");
    UsdPhysicsRevoluteJoint revoluteJoint0 = UsdPhysicsRevoluteJoint::Define(stage, revoluteJointPath0);
    revoluteJoint0.GetBody1Rel().AddTarget(boxPath0);
    revoluteJoint0.GetAxisAttr().Set(UsdPhysicsTokens->y);

    // create rigid body 0 with a box collider --- rack
    addRigidBox(stage, envPath + "/box1", GfVec3f(0.1f), GfVec3f(float(index)) + GfVec3f(0.0f, 0.0f, 0.3f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath boxPath1 = SdfPath(envPath + "/box1");
    UsdPrim boxPrim1 = stage->GetPrimAtPath(boxPath1);

    const SdfPath revoluteJointPath1 = SdfPath(envPath + "/revoluteJoint1");
    UsdPhysicsRevoluteJoint revoluteJoint1 = UsdPhysicsRevoluteJoint::Define(stage, revoluteJointPath1);
    revoluteJoint1.GetBody1Rel().AddTarget(boxPath1);
    revoluteJoint1.GetAxisAttr().Set(UsdPhysicsTokens->y);
    revoluteJoint1.CreateLocalPos0Attr().Set(GfVec3f(0.0f, 0.0f, 0.3f));

    // rack joint
    const SdfPath gearJointPath = SdfPath(envPath + "/gearJoint");
    PhysxSchemaPhysxPhysicsGearJoint gearJoint = PhysxSchemaPhysxPhysicsGearJoint::Define(stage, gearJointPath);
    gearJoint.CreateBody0Rel().AddTarget(boxPath0);
    gearJoint.CreateBody1Rel().AddTarget(boxPath1);
    gearJoint.CreateHinge0Rel().AddTarget(revoluteJointPath0);
    gearJoint.CreateHinge1Rel().AddTarget(revoluteJointPath1);
}

void createRackAndPinionJoint(const std::string& envPath, int index, UsdStageWeakPtr stage)
{
    // create rigid body 
    addRigidBox(stage, envPath + "/box0", GfVec3f(0.1f), GfVec3f(float(index)), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath boxPath0 = SdfPath(envPath + "/box0");

    const SdfPath revoluteJointPath = SdfPath(envPath + "/revoluteJoint");
    UsdPhysicsRevoluteJoint revoluteJoint = UsdPhysicsRevoluteJoint::Define(stage, revoluteJointPath);
    revoluteJoint.GetBody1Rel().AddTarget(boxPath0);
    revoluteJoint.GetAxisAttr().Set(UsdPhysicsTokens->y);

    // create rigid body 0 with a box collider --- rack
    addRigidBox(stage, envPath + "/box1", GfVec3f(0.1f), GfVec3f(float(index)) + GfVec3f(0.0f, 0.0f, 0.3f), GfQuatf(1.0f),
        GfVec3f(0.7f), 0.001f);

    const SdfPath boxPath1 = SdfPath(envPath + "/box1");
    UsdPrim boxPrim1 = stage->GetPrimAtPath(boxPath1);

    const SdfPath prismaticJointPath = SdfPath(envPath + "/prismaticJoint");
    UsdPhysicsRevoluteJoint prismaticJoint = UsdPhysicsRevoluteJoint::Define(stage, prismaticJointPath);
    prismaticJoint.GetBody1Rel().AddTarget(boxPath1);
    prismaticJoint.GetAxisAttr().Set(UsdPhysicsTokens->y);
    prismaticJoint.CreateLocalPos0Attr().Set(GfVec3f(0.0f, 0.0f, 0.3f));

    // rack joint
    const SdfPath rackJointPath = SdfPath(envPath + "/rackJoint");
    PhysxSchemaPhysxPhysicsRackAndPinionJoint rackJoint = PhysxSchemaPhysxPhysicsRackAndPinionJoint::Define(stage, rackJointPath);
    rackJoint.CreateBody0Rel().AddTarget(boxPath0);
    rackJoint.CreateBody1Rel().AddTarget(boxPath1);
    rackJoint.CreateHingeRel().AddTarget(revoluteJointPath);
    rackJoint.CreatePrismaticRel().AddTarget(prismaticJointPath);
}

// replicator maximal joint tests
TEST_CASE("Replicator Joints Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")

{
    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);
    IPhysxReplicator* physxReplicator = physicsTests.acquirePhysxReplicatorInterface();
    REQUIRE(physxReplicator);

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

    UsdGeomScope::Define(stage, SdfPath("/World/envs"));

    std::vector<carb::Float3> positions = {
    carb::Float3{ 0.0f, 0.0f, 0.0f }, carb::Float3{ 1.0f, 1.0f, 1.0f },
    carb::Float3{ 2.0f, 2.0f, 2.0f }, carb::Float3{ 3.0f, 3.0f, 3.0f },
    carb::Float3{ 4.0f, 4.0f, 4.0f }
    };

    std::vector<carb::Float4> orientations = {
        carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f }, carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f },
        carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f }, carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f },
        carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f }
    };

    SUBCASE("Prismatic Joint")
    {
        for (int i = 0; i < 5; i++)
        {
            createLinkHierarchy("/World/envs/env" + std::to_string(i), i, stage, false);
        }

        SUBCASE("Non active filtering")
        {
            IReplicatorCallback cb = { nullptr, nullptr, nullptr };
            physxReplicator->registerReplicator(stageId, cb);

            SUBCASE("Base parsing")
            {
                // attach sim to stage which parses and creates the pointers that we can check directly
                physxSim->attachStage(stageId);
                physx->forceLoadPhysicsFromUSD();

                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
                REQUIRE(basePtr != nullptr);
                PxScene* scene = (PxScene*)basePtr;

                CHECK(scene->getNbConstraints() == 10);
                CHECK(scene->getNbArticulations() == 0);
                CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 15);
            }
        }

        SUBCASE("Active filtering")
        {
            IReplicatorCallback cb = { nullptr, nullptr, nullptr };

            cb.replicationAttachFn = [](uint64_t stageId, uint32_t& numExludePaths, uint64_t*& excludePaths, void* userData)
            {
                numExludePaths = 1;
                const SdfPath boxPath = SdfPath("/World/envs");
                static uint64_t excludePath = sdfPathToInt(boxPath);
                excludePaths = &excludePath;
            };

            cb.hierarchyRenameFn = [](uint64_t replicatePath, uint32_t index, void* userData)
            {
                std::string stringPath = "/World/envs/env" + std::to_string(index + 1);
                const SdfPath outPath(stringPath);
                return sdfPathToInt(outPath);
            };

            physxReplicator->registerReplicator(stageId, cb);

            SUBCASE("Base parsing")
            {
                // attach sim to stage which parses and creates the pointers that we can check directly
                physxSim->attachStage(stageId);
                physx->forceLoadPhysicsFromUSD();

                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
                REQUIRE(basePtr != nullptr);
                PxScene* scene = (PxScene*)basePtr;

                CHECK(scene->getNbArticulations() == 0);
                CHECK(scene->getNbConstraints() == 0);
                CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            }

            SUBCASE("Base Joint Replication")
            {
                // attach sim to stage which parses and creates the pointers that we can check directly
                physxSim->attachStage(stageId);

                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
                REQUIRE(basePtr != nullptr);
                PxScene* scene = (PxScene*)basePtr;
                CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
                CHECK(scene->getNbArticulations() == 0);
                CHECK(scene->getNbConstraints() == 0);

                physxReplicator->replicate(stageId, sdfPathToInt(SdfPath("/World/envs/env0")), 4, false, false);

                CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 15);
                CHECK(scene->getNbArticulations() == 0);
                CHECK(scene->getNbConstraints() == 10);
            }

            SUBCASE("Initial Position Replication")
            {
                // attach sim to stage which parses and creates the pointers that we can check directly
                physxSim->attachStage(stageId);

                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
                REQUIRE(basePtr != nullptr);
                PxScene* scene = (PxScene*)basePtr;

                physxReplicator->replicate(stageId, sdfPathToInt(SdfPath("/World/envs/env0")), 4, false, false);

                CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC) == 10);
                CHECK(scene->getNbConstraints() == 10);

                for (int i = 0; i < 10; i++)
                {
                    PxActor* actor = nullptr;
                    scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &actor, 1, i);

                    REQUIRE(actor->is<PxRigidActor>());
                    const PxTransform tr = actor->is<PxRigidActor>()->getGlobalPose();
                    CHECK(GfIsClose(omni::physx::toVec3f(tr.p), omni::physx::toVec3f(positions[i / 2]), epsilon));
                }
            }

            SUBCASE("Path Mapping")
            {
                // attach sim to stage which parses and creates the pointers that we can check directly
                physxSim->attachStage(stageId);

                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
                REQUIRE(basePtr != nullptr);
                PxScene* scene = (PxScene*)basePtr;

                physxReplicator->replicate(stageId, sdfPathToInt(SdfPath("/World/envs/env0")), 4, false, false);

                CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 15);
                CHECK(scene->getNbConstraints() == 10);

                for (int k = 0; k < 5; k++)
                {
                    const SdfPath linkPath("/World/envs/env" + std::to_string(k) + "/dynamicLink");
                    const SdfPath rootPath("/World/envs/env" + std::to_string(k) + "/rootLink");
                    const SdfPath jointPath("/World/envs/env" + std::to_string(k) + "/prismaticJoint");
                    const SdfPath fixedJointPath("/World/envs/env" + std::to_string(k) + "/baseFixedJoint");
                    const SdfPath articulationPath("/World/envs/env" + std::to_string(k));

                    basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(linkPath, ePTActor));
                    REQUIRE(basePtr != nullptr);
                    CHECK(basePtr->is<PxRigidBody>());

                    basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(linkPath, ePTShape));
                    REQUIRE(basePtr != nullptr);
                    CHECK(basePtr->is<PxShape>());

                    basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(rootPath, ePTActor));
                    REQUIRE(basePtr != nullptr);
                    CHECK(basePtr->is<PxRigidBody>());

                    basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(rootPath, ePTShape));
                    REQUIRE(basePtr != nullptr);
                    CHECK(basePtr->is<PxShape>());

                    basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
                    REQUIRE(basePtr != nullptr);
                    CHECK(basePtr->is<PxJoint>());
                    CHECK(basePtr->is<PxJoint>()->getConcreteType() == PxJointConcreteType::eD6);

                    basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(fixedJointPath, ePTJoint));
                    REQUIRE(basePtr != nullptr);
                    CHECK(basePtr->is<PxJoint>());
                    CHECK(basePtr->is<PxJoint>()->getConcreteType() == PxJointConcreteType::eD6);
                }
            }

            SUBCASE("Update Positions Replication")
            {
                // attach sim to stage which parses and creates the pointers that we can check directly
                physxSim->attachStage(stageId);

                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
                REQUIRE(basePtr != nullptr);
                PxScene* scene = (PxScene*)basePtr;

                physxReplicator->replicate(stageId, sdfPathToInt(SdfPath("/World/envs/env0")), 4, false, false);

                CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 15);
                CHECK(scene->getNbConstraints() == 10);

                for (int k = 0; k < 20; k++)
                {
                    physxSim->simulate(1.0f / 60.0f, k * 1.0f / 60.0f);
                    physxSim->fetchResults();
                }

                for (int k = 0; k < 5; k++)
                {
                    const SdfPath linkPath("/World/envs/env" + std::to_string(k) + "/dynamicLink");
                    const GfVec3f position = getPhysicsPrimPos(stage, linkPath);
                    CHECK(position[2] < 0.0f);
                }

                for (int k = 0; k < 5; k++)
                {
                    const SdfPath linkPath("/World/envs/env" + std::to_string(k) + "/rootLink");
                    const GfVec3f position = getPhysicsPrimPos(stage, linkPath);
                    CHECK(fabsf(position[2] - (float)k) < epsilon);
                }
            }
        }
    }

    SUBCASE("Gear Joint")
    {
        for (int i = 0; i < 5; i++)
        {
            createGearJoint("/World/envs/env" + std::to_string(i), i, stage);
        }

        SUBCASE("Non active filtering")
        {
            IReplicatorCallback cb = { nullptr, nullptr, nullptr };
            physxReplicator->registerReplicator(stageId, cb);

            SUBCASE("Base parsing")
            {
                // attach sim to stage which parses and creates the pointers that we can check directly
                physxSim->attachStage(stageId);
                physx->forceLoadPhysicsFromUSD();

                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
                REQUIRE(basePtr != nullptr);
                PxScene* scene = (PxScene*)basePtr;

                CHECK(scene->getNbConstraints() == 15);
                CHECK(scene->getNbArticulations() == 0);
                CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 10);
            }
        }

        SUBCASE("Active filtering")
        {
            IReplicatorCallback cb = { nullptr, nullptr, nullptr };

            cb.replicationAttachFn = [](uint64_t stageId, uint32_t& numExludePaths, uint64_t*& excludePaths, void* userData)
            {
                numExludePaths = 1;
                const SdfPath boxPath = SdfPath("/World/envs");
                static uint64_t excludePath = sdfPathToInt(boxPath);
                excludePaths = &excludePath;
            };

            cb.hierarchyRenameFn = [](uint64_t replicatePath, uint32_t index, void* userData)
            {
                std::string stringPath = "/World/envs/env" + std::to_string(index + 1);
                const SdfPath outPath(stringPath);
                return sdfPathToInt(outPath);
            };

            physxReplicator->registerReplicator(stageId, cb);

            SUBCASE("Base parsing")
            {
                // attach sim to stage which parses and creates the pointers that we can check directly
                physxSim->attachStage(stageId);
                physx->forceLoadPhysicsFromUSD();

                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
                REQUIRE(basePtr != nullptr);
                PxScene* scene = (PxScene*)basePtr;

                CHECK(scene->getNbArticulations() == 0);
                CHECK(scene->getNbConstraints() == 0);
                CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            }

            SUBCASE("Base Joint Replication")
            {
                // attach sim to stage which parses and creates the pointers that we can check directly
                physxSim->attachStage(stageId);

                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
                REQUIRE(basePtr != nullptr);
                PxScene* scene = (PxScene*)basePtr;
                CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
                CHECK(scene->getNbArticulations() == 0);
                CHECK(scene->getNbConstraints() == 0);

                physxReplicator->replicate(stageId, sdfPathToInt(SdfPath("/World/envs/env0")), 4, false, false);

                CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 10);
                CHECK(scene->getNbArticulations() == 0);
                CHECK(scene->getNbConstraints() == 15);

                for (size_t i = 0; i < 5; i++)
                {
                    const SdfPath jointPath("/World/envs/env" + std::to_string(i) + "/gearJoint");
                    PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
                    REQUIRE(basePtr != nullptr);
                    CHECK(basePtr->is<PxJoint>());        
                    CHECK(basePtr->is<PxGearJoint>());
                }
            }
        }
    }

    SUBCASE("Rack and Pinion Joint")
    {
        for (int i = 0; i < 5; i++)
        {
            createRackAndPinionJoint("/World/envs/env" + std::to_string(i), i, stage);
        }

        SUBCASE("Non active filtering")
        {
            IReplicatorCallback cb = { nullptr, nullptr, nullptr };
            physxReplicator->registerReplicator(stageId, cb);

            SUBCASE("Base parsing")
            {
                // attach sim to stage which parses and creates the pointers that we can check directly
                physxSim->attachStage(stageId);
                physx->forceLoadPhysicsFromUSD();

                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
                REQUIRE(basePtr != nullptr);
                PxScene* scene = (PxScene*)basePtr;

                CHECK(scene->getNbConstraints() == 15);
                CHECK(scene->getNbArticulations() == 0);
                CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 10);
            }
        }

        SUBCASE("Active filtering")
        {
            IReplicatorCallback cb = { nullptr, nullptr, nullptr };

            cb.replicationAttachFn = [](uint64_t stageId, uint32_t& numExludePaths, uint64_t*& excludePaths, void* userData)
            {
                numExludePaths = 1;
                const SdfPath boxPath = SdfPath("/World/envs");
                static uint64_t excludePath = sdfPathToInt(boxPath);
                excludePaths = &excludePath;
            };

            cb.hierarchyRenameFn = [](uint64_t replicatePath, uint32_t index, void* userData)
            {
                std::string stringPath = "/World/envs/env" + std::to_string(index + 1);
                const SdfPath outPath(stringPath);
                return sdfPathToInt(outPath);
            };

            physxReplicator->registerReplicator(stageId, cb);

            SUBCASE("Base parsing")
            {
                // attach sim to stage which parses and creates the pointers that we can check directly
                physxSim->attachStage(stageId);
                physx->forceLoadPhysicsFromUSD();

                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
                REQUIRE(basePtr != nullptr);
                PxScene* scene = (PxScene*)basePtr;

                CHECK(scene->getNbArticulations() == 0);
                CHECK(scene->getNbConstraints() == 0);
                CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
            }

            SUBCASE("Base Joint Replication")
            {
                // attach sim to stage which parses and creates the pointers that we can check directly
                physxSim->attachStage(stageId);

                PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
                REQUIRE(basePtr != nullptr);
                PxScene* scene = (PxScene*)basePtr;
                CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 0);
                CHECK(scene->getNbArticulations() == 0);
                CHECK(scene->getNbConstraints() == 0);

                physxReplicator->replicate(stageId, sdfPathToInt(SdfPath("/World/envs/env0")), 4, false, false);

                CHECK(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 10);
                CHECK(scene->getNbArticulations() == 0);
                CHECK(scene->getNbConstraints() == 15);

                for (size_t i = 0; i < 5; i++)
                {
                    const SdfPath jointPath("/World/envs/env" + std::to_string(i) + "/rackJoint");
                    PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTJoint));
                    REQUIRE(basePtr != nullptr);
                    CHECK(basePtr->is<PxJoint>());        
                    CHECK(basePtr->is<PxRackAndPinionJoint>());
                }
            }
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    physxReplicator->unregisterReplicator(stageId);

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

TEST_CASE_TEMPLATE("Replicator EnvId RigidBody Tests", T, USDReplicator)
{
    const float epsilon = 0.0001f;

    T replicatorTemplate;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);
    IPhysxReplicator* physxReplicator = physicsTests.acquirePhysxReplicatorInterface();
    REQUIRE(physxReplicator);

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

    UsdGeomScope::Define(stage, SdfPath("/World/envs"));

    // create rigid bodies
    {
        const SdfPath envPath("/World/envs/env0");
        UsdGeomXform xform = UsdGeomXform::Define(stage, envPath);
        xform.AddTranslateOp().Set(GfVec3d(0.0));

        UsdGeomSphere sphere = UsdGeomSphere::Define(stage, envPath.AppendChild(TfToken("sphere")));
        UsdPhysicsCollisionAPI::Apply(sphere.GetPrim());
        UsdPhysicsRigidBodyAPI::Apply(sphere.GetPrim());
        PhysxSchemaPhysxRigidBodyAPI rbAPI = PhysxSchemaPhysxRigidBodyAPI::Apply(sphere.GetPrim());
        rbAPI.GetDisableGravityAttr().Set(true);
    }

    const SdfPath cloneEnv = SdfPath("/World/envs/env0");

    replicatorTemplate.init(stageId, physicsTests.getApp()->getFramework());
    replicatorTemplate.clone(SdfPath("/World/envs"), cloneEnv, 6, 1, 0.0f);

    carb::settings::ISettings* settings =
        physicsTests.getApp()->getFramework()->acquireInterface<carb::settings::ISettings>();

    IReplicatorCallback cb = { nullptr, nullptr, nullptr };

    cb.replicationAttachFn = [](uint64_t stageId, uint32_t& numExludePaths, uint64_t*& excludePaths, void* userData) {
        numExludePaths = 1;
        const SdfPath boxPath = SdfPath("/World/envs");
        static uint64_t excludePath = sdfPathToInt(boxPath);
        excludePaths = &excludePath;
    };

    cb.hierarchyRenameFn = [](uint64_t replicatePath, uint32_t index, void* userData) {
        std::string stringPath = "/World/envs/env" + std::to_string(index + 1);
        const SdfPath outPath(stringPath);
        return sdfPathToInt(outPath);
    };

    physxReplicator->registerReplicator(stageId, cb);

    SUBCASE("EnvIds Disabled")
    {
        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        physxReplicator->replicate(stageId, sdfPathToInt(cloneEnv), 5, false, false);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        REQUIRE(basePtr != nullptr);

        PxScene* scene = (PxScene*)basePtr;
        REQUIRE(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 6);

        for (int i = 0; i < 6; i++)
        {
            const std::string envPath = "/World/envs/env" + std::to_string(i);

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath(envPath + "/sphere"), ePTActor));
            REQUIRE(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidDynamic>());
            CHECK(basePtr->is<PxRigidDynamic>()->getEnvironmentID() == PX_INVALID_U32);

            compare(basePtr->is<PxRigidDynamic>()->getGlobalPose().p, GfVec3f(0.0f), epsilon);
        }

        for (int i = 0; i < 10; i++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
            physxSim->fetchResults();
        }

        for (int i = 0; i < 6; i++)
        {
            const std::string envPath = "/World/envs/env" + std::to_string(i);

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath(envPath + "/sphere"), ePTActor));
            CHECK(basePtr->is<PxRigidDynamic>()->getGlobalPose().p.magnitudeSquared() > epsilon);
        }
    }

    SUBCASE("EnvIds Enabled")
    {
        UsdAttribute attr = scene.GetPrim().CreateAttribute(TfToken("physxScene:envIdInBoundsBitCount"), SdfValueTypeNames->Int);
        attr.Set(4);

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        physxReplicator->replicate(stageId, sdfPathToInt(cloneEnv), 5, true, false);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        REQUIRE(basePtr != nullptr);

        PxScene* scene = (PxScene*)basePtr;
        REQUIRE(scene->getNbActors(PxActorTypeFlag::eRIGID_STATIC | PxActorTypeFlag::eRIGID_DYNAMIC) == 6);

        for (int i = 0; i < 6; i++)
        {
            const std::string envPath = "/World/envs/env" + std::to_string(i);

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath(envPath + "/sphere"), ePTActor));
            REQUIRE(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidDynamic>());
            CHECK(basePtr->is<PxRigidDynamic>()->getEnvironmentID() == i);

            compare(basePtr->is<PxRigidDynamic>()->getGlobalPose().p, GfVec3f(0.0f), epsilon);
        }

        for (int i = 0; i < 10; i++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
            physxSim->fetchResults();
        }

        for (int i = 0; i < 6; i++)
        {
            const std::string envPath = "/World/envs/env" + std::to_string(i);

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath(envPath + "/sphere"), ePTActor));
            //CHECK(basePtr->is<PxRigidDynamic>()->getGlobalPose().p.magnitudeSquared() > epsilon);
            compare(basePtr->is<PxRigidDynamic>()->getGlobalPose().p, GfVec3f(0.0f), epsilon);
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    replicatorTemplate.destroy();

    physxReplicator->unregisterReplicator(stageId);

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

TEST_CASE_TEMPLATE("Replicator EnvId Articulation Tests", T, USDReplicator)
{
    const float epsilon = 0.0001f;

    T replicatorTemplate;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);
    IPhysxReplicator* physxReplicator = physicsTests.acquirePhysxReplicatorInterface();
    REQUIRE(physxReplicator);

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

    UsdGeomScope::Define(stage, SdfPath("/World/envs"));

    for (int i = 0; i < 6; i++)
    {
        createLinkHierarchy("/World/envs/env" + std::to_string(i), i, stage, true, true, 0.0f);
    }

    carb::settings::ISettings* settings =
        physicsTests.getApp()->getFramework()->acquireInterface<carb::settings::ISettings>();

    IReplicatorCallback cb = { nullptr, nullptr, nullptr };

    cb.replicationAttachFn = [](uint64_t stageId, uint32_t& numExludePaths, uint64_t*& excludePaths, void* userData) {
        numExludePaths = 1;
        const SdfPath boxPath = SdfPath("/World/envs");
        static uint64_t excludePath = sdfPathToInt(boxPath);
        excludePaths = &excludePath;
    };

    cb.hierarchyRenameFn = [](uint64_t replicatePath, uint32_t index, void* userData) {
        std::string stringPath = "/World/envs/env" + std::to_string(index + 1);
        const SdfPath outPath(stringPath);
        return sdfPathToInt(outPath);
    };

    physxReplicator->registerReplicator(stageId, cb);

    SUBCASE("EnvIds Disabled")
    {
        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        physxReplicator->replicate(stageId, sdfPathToInt(SdfPath("/World/envs/env0")), 5, false, false);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        REQUIRE(basePtr != nullptr);

        PxScene* scene = (PxScene*)basePtr;
        REQUIRE(scene->getNbArticulations() == 6);
        CHECK(scene->getNbAggregates() == 6);

        for (int i = 0; i < 6; i++)
        {
            PxArticulationReducedCoordinate* articulation = nullptr;
            scene->getArticulations(&articulation, 1, i);
            CHECK(articulation->getNbLinks() == 2);
        }

        for (int i = 0; i < 6; i++)
        {
            const std::string envPath = "/World/envs/env" + std::to_string(i);

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath(envPath + "/dynamicLink"), ePTLink));
            REQUIRE(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidActor>());
            CHECK(basePtr->is<PxRigidActor>()->getEnvironmentID() == PX_INVALID_U32);

            compare(basePtr->is<PxRigidActor>()->getGlobalPose().p, GfVec3f(0.0f), epsilon);
        }

        for (int i = 0; i < 10; i++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
            physxSim->fetchResults();
        }

        for (int i = 0; i < 6; i++)
        {
            const std::string envPath = "/World/envs/env" + std::to_string(i);

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath(envPath + "/dynamicLink"), ePTLink));
            CHECK(basePtr->is<PxRigidActor>()->getGlobalPose().p.magnitudeSquared() > epsilon);
        }
    }

    SUBCASE("EnvIds Enabled")
    {
        UsdAttribute attr = scene.GetPrim().CreateAttribute(TfToken("physxScene:envIdInBoundsBitCount"), SdfValueTypeNames->Int);
        attr.Set(4);

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        physxReplicator->replicate(stageId, sdfPathToInt(SdfPath("/World/envs/env0")), 5, true, false);

        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsScenePath, ePTScene));
        REQUIRE(basePtr != nullptr);

        PxScene* scene = (PxScene*)basePtr;
        REQUIRE(scene->getNbArticulations() == 6);
        CHECK(scene->getNbAggregates() == 6);

        for (int i = 0; i < 6; i++)
        {
            const std::string envPath = "/World/envs/env" + std::to_string(i);

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath(envPath + "/dynamicLink"), ePTLink));
            REQUIRE(basePtr != nullptr);
            CHECK(basePtr->is<PxRigidActor>());
            CHECK(basePtr->is<PxRigidActor>()->getEnvironmentID() == PX_INVALID_U32);

            compare(basePtr->is<PxRigidActor>()->getGlobalPose().p, GfVec3f(0.0f), epsilon);

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath(envPath), ePTArticulation));
            REQUIRE(basePtr != nullptr);
            CHECK(basePtr->is<PxArticulationReducedCoordinate>());
            CHECK(basePtr->is<PxArticulationReducedCoordinate>()->getAggregate()->getEnvironmentID() == i);            
        }

        for (int i = 0; i < 10; i++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
            physxSim->fetchResults();
        }

        for (int i = 0; i < 6; i++)
        {
            const std::string envPath = "/World/envs/env" + std::to_string(i);

            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath(envPath + "/dynamicLink"), ePTLink));            
            compare(basePtr->is<PxRigidActor>()->getGlobalPose().p, GfVec3f(0.0f), epsilon);
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    physxReplicator->unregisterReplicator(stageId);

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

TEST_CASE_TEMPLATE("Replicator Multithreading Tests", T, USDReplicator)
{
    const float epsilon = 0.0001f;

    T replicatorTemplate;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);
    IPhysxReplicator* physxReplicator = physicsTests.acquirePhysxReplicatorInterface();
    REQUIRE(physxReplicator);

    // setup basic stage
    std::string usdFileName = physicsTests.getUnitTestsDataDirectory() + "multithreadedReplicator.usd";
    UsdStageRefPtr stage = UsdStage::Open(usdFileName);
    REQUIRE(stage);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    carb::settings::ISettings* settings =
        physicsTests.getApp()->getFramework()->acquireInterface<carb::settings::ISettings>();
    settings->setBool(kSettingUpdateToUsd, false);
    settings->setBool(kSettingUpdateVelocitiesToUsd, false);    

    IReplicatorCallback cb = { nullptr, nullptr, nullptr };

    cb.replicationAttachFn = [](uint64_t stageId, uint32_t& numExludePaths, uint64_t*& excludePaths, void* userData) {
        numExludePaths = 1;
        const SdfPath boxPath = SdfPath("/World/envs");
        static uint64_t excludePath = sdfPathToInt(boxPath);
        excludePaths = &excludePath;
    };

    cb.hierarchyRenameFn = [](uint64_t replicatePath, uint32_t index, void* userData) {
        std::string stringPath = "/World/envs/env_" + std::to_string(index + 1);
        const SdfPath outPath(stringPath);
        return sdfPathToInt(outPath);
    };

    physxReplicator->registerReplicator(stageId, cb);

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);

    physxReplicator->replicate(stageId, sdfPathToInt(SdfPath("/World/envs/env_0")), 1023, true, false);

    // Common post-test actions
    physxSim->detachStage();

    settings->setBool(kSettingUpdateToUsd, true);
    settings->setBool(kSettingUpdateVelocitiesToUsd, true);    

    physxReplicator->unregisterReplicator(stageId);

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
