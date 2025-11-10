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
#include <private/omni/physx/PhysXCompoundShape.h>

#if CARB_PLATFORM_LINUX
#define sprintf_s sprintf
#endif

using namespace pxr;
using namespace omni::physx;
using namespace ::physx;

//-----------------------------------------------------------------------------
// Collisions tests
TEST_CASE_TEMPLATE("Collision Tests", T, USDChange, FabricChange)
{
    T changeTemplate;

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

    // create rigid body with a box collider
    addRigidBox(stage, "/World/box", GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);
    const SdfPath boxPath = SdfPath("/World/box");
    UsdPrim boxPrim = stage->GetPrimAtPath(boxPath);
    PhysxSchemaPhysxCollisionAPI::Apply(boxPrim);

    // Create a convex decomposition UsdGeomMesh (needed for CompoundShapes tests)
    std::string meshPath = "/World/concaveMesh";
    const SdfPath concaveMeshPath(meshPath);
    pxr::UsdGeomMesh concaveMesh = createConcaveMesh(stage, concaveMeshPath, 100.0f);
    UsdPhysicsRigidBodyAPI::Apply(concaveMesh.GetPrim());
    UsdPhysicsCollisionAPI::Apply(concaveMesh.GetPrim());
    PhysxSchemaPhysxCollisionAPI::Apply(concaveMesh.GetPrim());
    UsdPhysicsMeshCollisionAPI meshCollisionAPI = UsdPhysicsMeshCollisionAPI::Apply(concaveMesh.GetPrim());
    meshCollisionAPI.CreateApproximationAttr().Set(UsdPhysicsTokens->convexDecomposition);

    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);

    SUBCASE("Collision Enabled")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsCollisionEnabled;

        bool enabled = changeTemplate.template getAttributeValue<bool>(boxPath, changeToken);
        // check default value
        CHECK(enabled == true);

        // shape should be there
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTShape));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxShape>());

        PxShape* shape = basePtr->is<PxShape>();
        CHECK((shape->getFlags() & PxShapeFlag::eSIMULATION_SHAPE) == true);
        CHECK((shape->getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE) == true);

        // disable collision API -> shape should be disabled
        changeTemplate.setAttributeValue(boxPath, changeToken, false);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // if not enabled static body should be created
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTShape));
        CHECK(basePtr == newBasePtr);
        CHECK((shape->getFlags() & PxShapeFlag::eSIMULATION_SHAPE) == false);
        CHECK((shape->getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE) == false);
    }

    SUBCASE("Collision Enabled CompoundShape")
    {
        const TfToken& changeToken = UsdPhysicsTokens->physicsCollisionEnabled;

        bool enabled = changeTemplate.template getAttributeValue<bool>(concaveMeshPath, changeToken);
        // check default value
        CHECK(enabled == true);

        // shape should be there
        void* voidPtr = physx->getPhysXPtr(concaveMeshPath, ePTCompoundShape);
        REQUIRE(voidPtr != nullptr);
        PhysXCompoundShape* cShape = (PhysXCompoundShape*)voidPtr;
        CHECK(cShape->getShapes().size() > 0);

        // check defaults
        for(auto shape : cShape->getShapes()) {
            REQUIRE(shape != nullptr);
            CHECK((shape->getFlags() & PxShapeFlag::eSIMULATION_SHAPE) == true);
            CHECK((shape->getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE) == true);
        }

        // disable collision API -> shapes should now be disabled
        changeTemplate.setAttributeValue(concaveMeshPath, changeToken, false);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // get another ptr and check collisions now
        voidPtr = physx->getPhysXPtr(concaveMeshPath, ePTCompoundShape);
        REQUIRE(voidPtr != nullptr);
        cShape = (PhysXCompoundShape*)voidPtr;
        CHECK(cShape->getShapes().size() > 0);
        for(auto shape : cShape->getShapes()) {
            REQUIRE(shape != nullptr);
            CHECK((shape->getFlags() & PxShapeFlag::eSIMULATION_SHAPE) == false);
            CHECK((shape->getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE) == false);
        }
    }

    SUBCASE("Collision ContactOffset")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxCollisionContactOffset;

        float val = changeTemplate.template getAttributeValue<float>(boxPath, changeToken);
        // check default value
        CHECK(isinf(val));

        // shape should be there
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTShape));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxShape>());

        PxShape* shape = basePtr->is<PxShape>();
        CHECK(fabsf(shape->getContactOffset() - 2.0f) < epsilon);

        // change val
        val = 10.0f;
        changeTemplate.setAttributeValue(boxPath, changeToken, val);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // check updated value
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTShape));
        CHECK(basePtr == newBasePtr);
        CHECK(fabsf(shape->getContactOffset() - val) < epsilon);
    }

    SUBCASE("Collision ContactOffset CompoundShape")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxCollisionContactOffset;

        float val = changeTemplate.template getAttributeValue<float>(concaveMeshPath, changeToken);
        // check default value
        CHECK(isinf(val));

        // shape should be there
        void* voidPtr = physx->getPhysXPtr(concaveMeshPath, ePTCompoundShape);
        REQUIRE(voidPtr != nullptr);
        PhysXCompoundShape* cShape = (PhysXCompoundShape*)voidPtr;
        CHECK(cShape->getShapes().size() > 0);

        // we don't care about the defaults here - reasoning: the contact offset is empirically determined
        // based off the shapes part of a cooked mesh, since we would have to hardcode values here for the
        // convexMesh prim and change them each time any part of that mesh changed, we'll just skip this
        // meaningless check here.

        // change val
        val = 10.0f;
        changeTemplate.setAttributeValue(concaveMeshPath, changeToken, val);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // check updated value
        for(auto shape : cShape->getShapes()) {
            REQUIRE(shape != nullptr);
            CHECK(fabsf(shape->getContactOffset() - val) < epsilon);
        }
    }

    SUBCASE("Collision RestOffset")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxCollisionRestOffset;

        float val = changeTemplate.template getAttributeValue<float>(boxPath, changeToken);
        // check default value
        CHECK(!isfinite(val));

        // shape should be there
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTShape));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxShape>());

        PxShape* shape = basePtr->is<PxShape>();
        CHECK(fabsf(shape->getRestOffset() - 0.0f) < epsilon);

        // change val
        val = 0.1f;
        changeTemplate.setAttributeValue(boxPath, changeToken, val);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // check updated value
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTShape));
        CHECK(basePtr == newBasePtr);
        CHECK(fabsf(shape->getRestOffset() - val) < epsilon);
    }

    SUBCASE("Collision RestOffset CompoundShape")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxCollisionRestOffset;

        float val = changeTemplate.template getAttributeValue<float>(concaveMeshPath, changeToken);
        // check default value
        CHECK(!isfinite(val));

        // shape should be there
        void* voidPtr = physx->getPhysXPtr(concaveMeshPath, ePTCompoundShape);
        REQUIRE(voidPtr != nullptr);
        PhysXCompoundShape* cShape = (PhysXCompoundShape*)voidPtr;
        CHECK(cShape->getShapes().size() > 0);

        // change val, just make sure this is LESS than contactOffset
        val = 0.1f;
        changeTemplate.setAttributeValue(concaveMeshPath, changeToken, val);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // check updated value
        for(auto shape : cShape->getShapes()) {
            REQUIRE(shape != nullptr);
            CHECK(fabsf(shape->getRestOffset() - val) < epsilon);
        }
    }

    SUBCASE("Collision TorsionalPatchRadius")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxCollisionTorsionalPatchRadius;

        float val = changeTemplate.template getAttributeValue<float>(boxPath, changeToken);
        // check default value
        CHECK(fabsf(val - 0.0f) < epsilon);

        // shape should be there
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTShape));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxShape>());

        PxShape* shape = basePtr->is<PxShape>();
        CHECK(fabsf(shape->getTorsionalPatchRadius() - val) < epsilon);

        // change val
        val = 1.0f;
        changeTemplate.setAttributeValue(boxPath, changeToken, val);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // check updated value
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTShape));
        CHECK(basePtr == newBasePtr);
        CHECK(fabsf(shape->getTorsionalPatchRadius() - val) < epsilon);
    }

    SUBCASE("Collision TorsionalPatchRadius CompoundShape")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxCollisionTorsionalPatchRadius;

        float val = changeTemplate.template getAttributeValue<float>(concaveMeshPath, changeToken);
        // check default value
        CHECK(fabsf(val - 0.0f) < epsilon);

        // shape should be there
        void* voidPtr = physx->getPhysXPtr(concaveMeshPath, ePTCompoundShape);
        REQUIRE(voidPtr != nullptr);
        PhysXCompoundShape* cShape = (PhysXCompoundShape*)voidPtr;
        CHECK(cShape->getShapes().size() > 0);

        // check defaults
        for(auto shape : cShape->getShapes()) {
            REQUIRE(shape != nullptr);
            CHECK(fabsf(shape->getTorsionalPatchRadius() - val) < epsilon);
        }

        // change val
        val = 1.0f;
        changeTemplate.setAttributeValue(concaveMeshPath, changeToken, val);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // check updated value
        for(auto shape : cShape->getShapes()) {
            REQUIRE(shape != nullptr);
            CHECK(fabsf(shape->getTorsionalPatchRadius() - val) < epsilon);
        }
    }

    SUBCASE("Collision MinTorsionalPatchRadius")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxCollisionMinTorsionalPatchRadius;

        float val = changeTemplate.template getAttributeValue<float>(boxPath, changeToken);
        // check default value
        CHECK(fabsf(val - 0.0f) < epsilon);

        // shape should be there
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTShape));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxShape>());

        PxShape* shape = basePtr->is<PxShape>();
        CHECK(fabsf(shape->getMinTorsionalPatchRadius() - val) < epsilon);

        // change val
        val = 1.0f;
        changeTemplate.setAttributeValue(boxPath, changeToken, val);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // check updated value
        PxBase* newBasePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxPath, ePTShape));
        CHECK(basePtr == newBasePtr);
        CHECK(fabsf(shape->getMinTorsionalPatchRadius() - val) < epsilon);
    }

    SUBCASE("Collision MinTorsionalPatchRadius CompoundShape")
    {
        const TfToken& changeToken = PhysxSchemaTokens->physxCollisionMinTorsionalPatchRadius;

        float val = changeTemplate.template getAttributeValue<float>(concaveMeshPath, changeToken);
        // check default value
        CHECK(fabsf(val - 0.0f) < epsilon);

        // shape should be there
        void* voidPtr = physx->getPhysXPtr(concaveMeshPath, ePTCompoundShape);
        REQUIRE(voidPtr != nullptr);
        PhysXCompoundShape* cShape = (PhysXCompoundShape*)voidPtr;
        CHECK(cShape->getShapes().size() > 0);

        // check defaults
        for(auto shape : cShape->getShapes()) {
            REQUIRE(shape != nullptr);
            CHECK(fabsf(shape->getMinTorsionalPatchRadius() - val) < epsilon);
        }

        // change val
        val = 1.0f;
        changeTemplate.setAttributeValue(concaveMeshPath, changeToken, val);
        changeTemplate.broadcastChanges();
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // check updated value
        for(auto shape : cShape->getShapes()) {
            REQUIRE(shape != nullptr);
            CHECK(fabsf(shape->getMinTorsionalPatchRadius() - val) < epsilon);
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    changeTemplate.destroy();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

TEST_CASE("Collision PointInstancer",
          "[omniphysics]"
          "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

    const float toleranceEpsilon = 1e-4f;

    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxUnitTests* physxUT = physicsTests.acquirePhysxUnitTestInterface();
    REQUIRE(physxUT);
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

    const SdfPath geomPointInstancerPath = defaultPrimPath.AppendChild(TfToken("pointInstancer"));
    const SdfPath boxActorPath = geomPointInstancerPath.AppendChild(TfToken("boxActor"));

    UsdGeomCube cubeGeom = UsdGeomCube::Define(stage, boxActorPath);

    UsdPhysicsCollisionAPI::Apply(cubeGeom.GetPrim());

    const uint32_t numIndices = 2;
    VtArray<int> meshIndices = { 0, 0 };
    VtArray<GfVec3f> positions = { GfVec3f(-125.0, 0.0, 500.0), GfVec3f(-125.0, 0.0, 500.0) };
    VtArray<GfQuath> orientations = { GfQuath(1.0), GfQuath(1.0) };
    VtArray<GfVec3f> linearVelocities = { GfVec3f(0.0), GfVec3f(0.0, 0.0, 0.0) };
    VtArray<GfVec3f> angularVelocities = { GfVec3f(0.0, 10.0, 0.0), GfVec3f(0.0) };

    UsdGeomPointInstancer shapeList = UsdGeomPointInstancer::Define(stage, geomPointInstancerPath);
    shapeList.AddTranslateOp().Set(GfVec3f(100.0f));
    shapeList.GetPrototypesRel().AddTarget(boxActorPath);

    shapeList.GetProtoIndicesAttr().Set(meshIndices);
    shapeList.GetPositionsAttr().Set(positions);
    shapeList.GetOrientationsAttr().Set(orientations);
    shapeList.GetVelocitiesAttr().Set(linearVelocities);
    shapeList.GetAngularVelocitiesAttr().Set(angularVelocities);

    UsdPhysicsRigidBodyAPI::Apply(shapeList.GetPrim());

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);

    SUBCASE("Positions")
    {
        positions.clear();
        shapeList.GetPositionsAttr().Get(&positions);
        // check default value
        CHECK(positions.size() == size_t(numIndices));

        // dynamic rigid body should be created
        std::vector<void*> ptrs;
        ptrs.resize(size_t(numIndices));
        uint32_t numPtrs = physxUT->getPhysXPtrInstanced(boxActorPath, ptrs.data(), numIndices, ePTShape);
        CHECK(numPtrs == numIndices);
        for (uint32_t i = 0; i < numIndices; i++)
        {
            PxBase* basePtr = reinterpret_cast<PxBase*>(ptrs[i]);
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());
            PxShape* shape = basePtr->is<PxShape>();
            PxTransform tr = shape->getLocalPose();
            compare(tr.p, (const PxVec3&)(positions[i]), toleranceEpsilon);
        }

        // update positions
        for (uint32_t i = 0; i < numIndices; i++)
        {
            positions[i][0] = i * 500.0f;
        }
        shapeList.GetPositionsAttr().Set(positions);

        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        numPtrs = physxUT->getPhysXPtrInstanced(boxActorPath, ptrs.data(), numIndices, ePTShape);
        CHECK(numPtrs == numIndices);
        for (uint32_t i = 0; i < numIndices; i++)
        {
            PxBase* basePtr = reinterpret_cast<PxBase*>(ptrs[i]);
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());
            PxShape* shape = basePtr->is<PxShape>();
            PxTransform tr = shape->getLocalPose();
            compare(tr.p, (const PxVec3&)(positions[i]), toleranceEpsilon);
        }
    }

    SUBCASE("Orientations")
    {
        const float quatEpsilon = 0.01f;

        orientations.clear();
        shapeList.GetOrientationsAttr().Get(&orientations);
        // check default value
        CHECK(orientations.size() == size_t(numIndices));

        // dynamic rigid body should be created
        std::vector<void*> ptrs;
        ptrs.resize(size_t(numIndices));
        uint32_t numPtrs = physxUT->getPhysXPtrInstanced(boxActorPath, ptrs.data(), numIndices, ePTShape);
        CHECK(numPtrs == numIndices);
        for (uint32_t i = 0; i < numIndices; i++)
        {
            PxBase* basePtr = reinterpret_cast<PxBase*>(ptrs[i]);
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());
            PxShape* shape = basePtr->is<PxShape>();
            PxTransform tr = shape->getLocalPose();
            compare(tr.q, toPhysX(orientations[i]), quatEpsilon);
        }

        // update orientations
        for (uint32_t i = 0; i < numIndices; i++)
        {
            orientations[i] = GfQuath(0.8660254f, 0.0f, 0.0f, 0.5f).GetNormalized();
        }
        shapeList.GetOrientationsAttr().Set(orientations);

        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        numPtrs = physxUT->getPhysXPtrInstanced(boxActorPath, ptrs.data(), numIndices, ePTShape);
        CHECK(numPtrs == numIndices);
        for (uint32_t i = 0; i < numIndices; i++)
        {
            PxBase* basePtr = reinterpret_cast<PxBase*>(ptrs[i]);
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());
            PxShape* shape = basePtr->is<PxShape>();
            PxTransform tr = shape->getLocalPose();
            compare(tr.q, toPhysX(orientations[i]), quatEpsilon);
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

TEST_CASE("Collision Instanced",
    "[omniphysics]"
    "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

    const float toleranceEpsilon = 1e-4f;

    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxUnitTests* physxUT = physicsTests.acquirePhysxUnitTestInterface();
    REQUIRE(physxUT);
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

    const SdfPath xformPath = defaultPrimPath.AppendChild(TfToken("xform"));
    const SdfPath spherePath = xformPath.AppendChild(TfToken("sphere"));

    UsdGeomSphere sphereGeom = UsdGeomSphere::Define(stage, spherePath);
    sphereGeom.AddTranslateOp().Set(GfVec3d(3.0));
    UsdPhysicsCollisionAPI::Apply(sphereGeom.GetPrim());
    sphereGeom.CreateRadiusAttr().Set(1.0f);

    // Create base physics material
    const SdfPath basePhysicsMaterialPath = defaultPrimPath.AppendChild(TfToken("basePhysicsMaterial"));
    UsdShadeMaterial basePhysicsMaterial = UsdShadeMaterial::Define(stage, basePhysicsMaterialPath);
    UsdPhysicsMaterialAPI::Apply(basePhysicsMaterial.GetPrim());

    const SdfPath xformInstPath = defaultPrimPath.AppendChild(TfToken("xformInst"));
    UsdGeomXform xform = UsdGeomXform::Define(stage, xformInstPath);
    xform.GetPrim().GetReferences().AddInternalReference(xformPath);
    xform.GetPrim().SetInstanceable(true);

    SUBCASE("Scale")
    {
        xform.AddScaleOp().Set(GfVec3f(2.0f));
        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        // shape should be there
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath("/World/xformInst/sphere"), ePTShape));
        CHECK(basePtr != nullptr);
        PxShape* shapePtr = basePtr->is<PxShape>();
        REQUIRE(shapePtr);
        const PxGeometry& geom = shapePtr->getGeometry();
        REQUIRE(geom.getType() == PxGeometryType::eSPHERE);
        const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom);
        CHECK(fabsf(sphereGeom.radius - 2.f) < toleranceEpsilon);

        // static body should be there
        basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath("/World/xformInst/sphere"), ePTActor));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxRigidStatic>());
    }

    SUBCASE("Material Instance")
    {
        UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(xform.GetPrim());
        bindingAPI.Bind(basePhysicsMaterial, UsdShadeTokens->weakerThanDescendants, TfToken("physics"));

        physxSim->attachStage(stageId);

        // material should be there
        PxBase* baseMaterialPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(basePhysicsMaterialPath, ePTMaterial));
        CHECK(baseMaterialPtr != nullptr);
        CHECK(baseMaterialPtr->is<PxMaterial>());


        // shape should be there
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath("/World/xformInst/sphere"), ePTShape));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxShape>());

        PxShape* shape = basePtr->is<PxShape>();
        REQUIRE(shape->getNbMaterials() == 1);
        PxMaterial* material = nullptr;
        shape->getMaterials(&material, 1);
        CHECK(material->userData != nullptr);
        CHECK(material == baseMaterialPtr->is<PxMaterial>());
    }

    SUBCASE("Position")
    {
        xform.AddTranslateOp().Set(GfVec3d(2.0));

        SUBCASE("Dynamic Body")
        {
            UsdPhysicsRigidBodyAPI::Apply(xform.GetPrim());

            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            // shape should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath("/World/xformInst/sphere"), ePTShape));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxShape>());
            PxTransform pose = basePtr->is<PxShape>()->getLocalPose();
            compare(pose.p, GfVec3f(3.0f), toleranceEpsilon);

            // dynamic body should be there
            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath("/World/xformInst"), ePTActor));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxRigidDynamic>());

            PxTransform tr = basePtr->is<PxRigidDynamic>()->getGlobalPose();
            compare(tr.p, GfVec3f(2.0f), toleranceEpsilon);
        }

        SUBCASE("Static Body")
        {
            // attach sim to stage which parses and creates the pointers that we can check directly
            physxSim->attachStage(stageId);

            // shape should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath("/World/xformInst/sphere"), ePTShape));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxShape>());
            PxTransform pose = basePtr->is<PxShape>()->getLocalPose();
            compare(pose.p, GfVec3f(0.0f), toleranceEpsilon);

            // static body should be there
            basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(SdfPath("/World/xformInst/sphere"), ePTActor));
            CHECK(basePtr != nullptr);
            REQUIRE(basePtr->is<PxRigidStatic>());

            PxTransform tr = basePtr->is<PxRigidStatic>()->getGlobalPose();
            compare(tr.p, GfVec3f(5.0f), toleranceEpsilon);
        }
    }
    // Common post-test actions
    physxSim->detachStage();

    UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}


TEST_CASE("Cone and Cylinder Collision",
          "[omniphysics]"
          "[component=OmniPhysics][owner=vreutskyy][priority=mandatory]")
{
    // tokens:
    const TfToken translateAttrToken("xformOp:translate");
    // constants for test checks
    const double positionToleranceCM = 1.0f;

    // constants for setup
    const GfVec3f gravityDirection(0.0f, 0.0f, -1.0f); // use z-up
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const float gravityMagnitude = 10.0f / metersPerStageUnit;

    // setup common to all subcases
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const TfToken sceneUpAxisToken("Z");
    UsdGeomSetStageUpAxis(stage, sceneUpAxisToken);
    UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");

    UsdUtilsStageCache::Get().Insert(stage);
    long stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);
    scene.GetGravityMagnitudeAttr().Set(gravityMagnitude);
    scene.GetGravityDirectionAttr().Set(gravityDirection);

    // setup ground plane at zero pointing up:
    UsdGeomPlane groundPlane = UsdGeomPlane::Define(stage, defaultPrimPath.AppendElementString("GroundPlane"));
    groundPlane.CreateAxisAttr().Set(sceneUpAxisToken);
    groundPlane.AddTranslateOp().Set(GfVec3f(0.0f));
    groundPlane.AddOrientOp().Set(GfQuatf(0.0f));
    groundPlane.AddScaleOp().Set(GfVec3f(1.0f));
    UsdPhysicsCollisionAPI::Apply(groundPlane.GetPrim()); // make it collide

    // Verifies the functionality of the physics setting for whether to use CustomGeomtry or ConvexMesh for cylinders and cones.
    // Note: the setting is assumed to be enabled by default.
    SUBCASE("Physics Setting toggle")
    {
        const SdfPath geomPath = defaultPrimPath.AppendElementString("Geometry");
        const double height = 100.0f;
        const double radius = 50.0f;
        const GfVec3d initialTranslation(0.0f, 0.0f, height * 0.5);

        carb::settings::ISettings* settings = physicsTests.acquireSettingsInterface();

        const char* settingToggle;

        SUBCASE("Cylinder")
        {
            settingToggle = omni::physx::kSettingCollisionApproximateCylinders;

            UsdGeomCylinder cylinder = UsdGeomCylinder::Define(stage, geomPath);
            REQUIRE(cylinder);
            cylinder.CreateHeightAttr().Set(height);
            cylinder.CreateRadiusAttr().Set(radius);
            cylinder.AddTranslateOp().Set(initialTranslation);
        }

        SUBCASE("Cone")
        {
            settingToggle = omni::physx::kSettingCollisionApproximateCones;

            UsdGeomCone cone = UsdGeomCone::Define(stage, geomPath);
            REQUIRE(cone);
            cone.CreateHeightAttr().Set(height);
            cone.CreateRadiusAttr().Set(radius);
            cone.AddTranslateOp().Set(initialTranslation);
        }

        // setup RB and collider:
        const UsdPrim geomPrim = stage->GetPrimAtPath(geomPath);
        REQUIRE(geomPrim);
        UsdPhysicsCollisionAPI::Apply(geomPrim);
        UsdPhysicsRigidBodyAPI::Apply(geomPrim);

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        // check that the actor shape geom is a custom geom:
        PxRigidDynamic* actor = getPhysxBaseDerivedFromPathChecked<PxRigidDynamic>(geomPath, ePTActor);
        PxShape* shape = nullptr;
        actor->getShapes(&shape, 1u, 0u);
        REQUIRE(shape != nullptr);
        CHECK(shape->getGeometry().getType() == PxGeometryType::eCONVEXCORE);

        // Disable the setting and reattach.
        physxSim->detachStage();
        settings->setBool(settingToggle, true);
        physxSim->attachStage(stageId);

        actor = getPhysxBaseDerivedFromPathChecked<PxRigidDynamic>(geomPath, ePTActor);
        actor->getShapes(&shape, 1u, 0u);
        REQUIRE(shape != nullptr);
        CHECK(shape->getGeometry().getType() == PxGeometryType::eCONVEXMESH);

        // Re-enable the setting.
        settings->setBool(settingToggle, false);
    }

    SUBCASE("Free-Fall onto ground")
    {
        const SdfPath geomPath = defaultPrimPath.AppendElementString("Geometry");
        const double freeFallDistance = 20.0; // cm
        const float simTime = std::sqrt(2.0f * float(freeFallDistance) / gravityMagnitude);
        const float simDt = 1.0f / 60.0f;
        const int simSteps = int(simTime / simDt) + 1;
        const double height = 2.0 * freeFallDistance;
        const double radius = freeFallDistance;
        const GfVec3d initialTranslation(0.0f, 0.0f, freeFallDistance * 0.8 + height * 0.5);

        SUBCASE("Cylinder")
        {
            UsdGeomCylinder cylinder = UsdGeomCylinder::Define(stage, geomPath);
            REQUIRE(cylinder);
            cylinder.CreateHeightAttr().Set(height);
            cylinder.CreateRadiusAttr().Set(radius);
            cylinder.AddTranslateOp().Set(initialTranslation);
        }

        SUBCASE("Cone")
        {
            UsdGeomCone cone = UsdGeomCone::Define(stage, geomPath);
            REQUIRE(cone);
            cone.CreateHeightAttr().Set(height);
            cone.CreateRadiusAttr().Set(radius);
            cone.AddTranslateOp().Set(initialTranslation);
        }

        // setup RB and collider:
        const UsdPrim geomPrim = stage->GetPrimAtPath(geomPath);
        UsdPhysicsCollisionAPI::Apply(geomPrim);
        UsdPhysicsRigidBodyAPI::Apply(geomPrim);

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        // check that the actor shape geom is a custom geom:
        PxRigidDynamic* actor = getPhysxBaseDerivedFromPathChecked<PxRigidDynamic>(geomPath, ePTActor);
        PxShape* shape = nullptr;
        actor->getShapes(&shape, 1u, 0u);
        REQUIRE(shape != nullptr);
        CHECK(shape->getGeometry().getType() == PxGeometryType::eCONVEXCORE);

        //stage->Export("customGeom.usda"); // debug output to usda (into compiler dir in test.unit folder)

        PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
        CHECK(1u == pxScene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC));

        // sim:
        for (int u = 0; u < simSteps; u++)
        {
            physxSim->simulate(simDt, simDt * u);
            physxSim->fetchResults();
        }

        // check height:
        UsdAttribute translationAttr = geomPrim.GetAttribute(translateAttrToken);
        REQUIRE(translationAttr);
        GfVec3d finalTranslation;
        translationAttr.Get(&finalTranslation);
        CHECK(finalTranslation[2] > height * 0.5 - positionToleranceCM);
    }

    SUBCASE("Stacking cylinders")
    {
        const float simDt = 1.0f / 60.0f;
        const int simSteps = 500;
        const double height = 100.0f; // cm
        const double radius = height * 0.5f;
        const int shapesCount = 8;

        for (int i = 0; i < shapesCount; ++i)
        {
            char primName[64]; sprintf_s(primName, "Geometry%d", i);
            const SdfPath geomPath = defaultPrimPath.AppendElementString(primName);
            const GfVec3d initialTranslation(0.0f, 0.0f, height * i + height * 0.5);

            UsdGeomCylinder cylinder = UsdGeomCylinder::Define(stage, geomPath);
            REQUIRE(cylinder);
            cylinder.CreateHeightAttr().Set(height);
            cylinder.CreateRadiusAttr().Set(radius);
            cylinder.AddTranslateOp().Set(initialTranslation);

            // setup RB and collider:
            const UsdPrim geomPrim = stage->GetPrimAtPath(geomPath);
            REQUIRE(geomPrim);
            UsdPhysicsCollisionAPI::Apply(geomPrim);
            UsdPhysicsRigidBodyAPI::Apply(geomPrim);
        }

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        //stage->Export("customGeom.usda"); // debug output to usda (into compiler dir in test.unit folder)

        PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
        CHECK(shapesCount == pxScene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC));

        // sim:
        for (int u = 0; u < simSteps; u++)
        {
            physxSim->simulate(simDt, simDt * u);
            physxSim->fetchResults();
        }

        for (int i = 0; i < shapesCount; ++i)
        {
            char primName[64]; sprintf_s(primName, "Geometry%d", i);
            const SdfPath geomPath = defaultPrimPath.AppendElementString(primName);
            const UsdPrim geomPrim = stage->GetPrimAtPath(geomPath);

            // check height:
            UsdAttribute translationAttr = geomPrim.GetAttribute(translateAttrToken);
            REQUIRE(translationAttr);
            GfVec3d finalTranslation;
            translationAttr.Get(&finalTranslation);
            CHECK(finalTranslation[2] > height * i + height * 0.5 - positionToleranceCM);

            // check that the actor shape geom is a convex core geom cylinder:
            PxRigidDynamic* actor = getPhysxBaseDerivedFromPathChecked<PxRigidDynamic>(geomPath, ePTActor);
            PxShape* shape = nullptr;
            actor->getShapes(&shape, 1u, 0u);
            REQUIRE(shape != nullptr);
            const PxGeometry& geom = shape->getGeometry();
            CHECK(geom.getType() == PxGeometryType::eCONVEXCORE);
            const PxConvexCoreGeometry& convexCoreGeom = static_cast<const PxConvexCoreGeometry&>(geom);
            CHECK(convexCoreGeom.getCoreType() == PxConvexCore::eCYLINDER);
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

TEST_CASE("Collision Negative Scale",
          "[omniphysics]"
          "[component=OmniPhysics][owner=scristiano][priority=mandatory]")
{
    // OM-119928: Regression in cooking with respect to negative scale
    // This test creates a bunch of collider shapes that have their Center Of Mass != than Geometrical Center
    // and then proceeds to put them in a hierarchy with some negative scale on some axis.
    // This checks for proper handling of sign scale, because if that's not carried over correctly,
    // the location of physx shape will not match anymore the one of the visual shape. 

    // tokens:
    const TfToken translateAttrToken("xformOp:translate");

    // constants for setup
    const GfVec3f gravityDirection(0.0f, 0.0f, -1.0f); // use z-up
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const float gravityMagnitude = 10.0f / metersPerStageUnit;

    // setup common to all subcases
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const TfToken sceneUpAxisToken("Z");
    UsdGeomSetStageUpAxis(stage, sceneUpAxisToken);
    UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");

    UsdUtilsStageCache::Get().Insert(stage);
    long stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);
    scene.GetGravityMagnitudeAttr().Set(gravityMagnitude);
    scene.GetGravityDirectionAttr().Set(gravityDirection);

    // setup ground plane at zero pointing up:
    UsdGeomPlane groundPlane = UsdGeomPlane::Define(stage, defaultPrimPath.AppendElementString("GroundPlane"));
    groundPlane.CreateAxisAttr().Set(sceneUpAxisToken);
    groundPlane.AddTranslateOp().Set(GfVec3f(0.0f));
    groundPlane.AddOrientOp().Set(GfQuatf(0.0f));
    groundPlane.AddScaleOp().Set(GfVec3f(1.0f));
    UsdPhysicsCollisionAPI::Apply(groundPlane.GetPrim()); // make it collide

    {
        const SdfPath xformPath = defaultPrimPath.AppendChild(TfToken("Xform"));
        UsdGeomXform xform = UsdGeomXform::Define(stage, xformPath);

        const SdfPath geomPath = xformPath.AppendElementString("Geometry");
        const double freeFallDistance = 20.0; // cm
        const float simTime = 2 * std::sqrt(2.0f * float(freeFallDistance) / gravityMagnitude);
        const float simDt = 1.0f / 60.0f;
        const int simSteps = int(simTime / simDt) + 1;
        const double height = 2.0 * freeFallDistance;
        const GfVec3d initialTranslation(0.0f, 0.0f, freeFallDistance * 0.8 + height * 0.5);


        // setup RB and collider:
        UsdGeomMesh geomMesh = createConcaveMesh(stage, geomPath, height, height); // last parameter is offset
        REQUIRE(geomMesh);
        const UsdPrim geomPrim = stage->GetPrimAtPath(geomPath);
        UsdPhysicsCollisionAPI::Apply(geomPrim);
        UsdPhysicsMeshCollisionAPI meshApi = UsdPhysicsMeshCollisionAPI::Apply(geomPrim);
        UsdPhysicsRigidBodyAPI::Apply(geomPrim);
        geomMesh.AddTranslateOp().Set(initialTranslation);
        xform.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(GfVec3f(0, 0, 0));
        xform.AddRotateXYZOp(UsdGeomXformOp::PrecisionFloat).Set(GfVec3f(0, 0, 0));
        xform.AddScaleOp(UsdGeomXformOp::PrecisionFloat).Set(GfVec3f(-1.0, 1.0, 1.0f));

        float checkTolerance = 0.0;
        SUBCASE("Convex Hull")
        {
            checkTolerance = 0.01;
            meshApi.CreateApproximationAttr().Set(UsdPhysicsTokens->convexHull);
        }

        SUBCASE("Convex Decomposition")
        {
            checkTolerance = 1.5;
            meshApi.CreateApproximationAttr().Set(UsdPhysicsTokens->convexDecomposition);
        }

        // attach sim to stage which parses and creates the pointers that we can check directly
        physxSim->attachStage(stageId);

        // stage->Export("customGeom.usda"); // debug output to usda (into compiler dir in test.unit folder)

        // sim:
        for (int u = 0; u < simSteps; u++)
        {
            physxSim->simulate(simDt, simDt * u);
            physxSim->fetchResults();
        }

        // check height:
        UsdAttribute translationAttr = geomPrim.GetAttribute(translateAttrToken);
        REQUIRE(translationAttr);
        GfVec3d finalTranslation;
        translationAttr.Get(&finalTranslation);
        bool res = finalTranslation[2] > -checkTolerance && finalTranslation[2] < checkTolerance;
        CHECK(res);
    }

    // Common post-test actions
    physxSim->detachStage();

    UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
