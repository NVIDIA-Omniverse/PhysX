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
// Material parsing tests
TEST_CASE("Material Parsing Tests",
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
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    // Create physics scene
    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);


    // Create base physics material
    const SdfPath basePhysicsMaterialPath = defaultPrimPath.AppendChild(TfToken("basePhysicsMaterial"));
    UsdShadeMaterial basePhysicsMaterial = UsdShadeMaterial::Define(stage, basePhysicsMaterialPath);
    UsdPhysicsMaterialAPI::Apply(basePhysicsMaterial.GetPrim());


    SUBCASE("Regular Colliders")
    {
        // Create top level xform
        const SdfPath xformPath = defaultPrimPath.AppendChild(TfToken("xform"));
        UsdGeomXform xform = UsdGeomXform::Define(stage, xformPath);

        // Create test collider cube
        const SdfPath cubePath = xformPath.AppendChild(TfToken("cube"));
        UsdGeomCube cube = UsdGeomCube::Define(stage, cubePath);
        UsdPhysicsCollisionAPI::Apply(cube.GetPrim());

        // expect just default material
        SUBCASE("Base Setup")
        {
            physxSim->attachStage(stageId);

            // material should be there
            PxBase* baseMaterialPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(basePhysicsMaterialPath, ePTMaterial));
            CHECK(baseMaterialPtr != nullptr);
            CHECK(baseMaterialPtr->is<PxMaterial>());

            // shape should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(cubePath, ePTShape));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());

            PxShape* shape = basePtr->is<PxShape>();
            REQUIRE(shape->getNbMaterials() == 1);
            PxMaterial* material = nullptr;
            shape->getMaterials(&material, 1);
            CHECK(material->userData == nullptr);
        }

        // Apply physics material to Cube
        SUBCASE("Base Setup Material")
        {
            UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(cube.GetPrim());
            bindingAPI.Bind(basePhysicsMaterial, UsdShadeTokens->weakerThanDescendants, TfToken("physics"));

            physxSim->attachStage(stageId);

            // material should be there
            PxBase* baseMaterialPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(basePhysicsMaterialPath, ePTMaterial));
            CHECK(baseMaterialPtr != nullptr);
            CHECK(baseMaterialPtr->is<PxMaterial>());


            // shape should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(cubePath, ePTShape));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());

            PxShape* shape = basePtr->is<PxShape>();
            REQUIRE(shape->getNbMaterials() == 1);
            PxMaterial* material = nullptr;
            shape->getMaterials(&material, 1);
            CHECK(material->userData != nullptr);
            CHECK(material == baseMaterialPtr->is<PxMaterial>());
        }

        // Apply physics material to Scene
        SUBCASE("Base Setup Scene Default Material")
        {
            UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(scene.GetPrim());
            bindingAPI.Bind(basePhysicsMaterial, UsdShadeTokens->weakerThanDescendants, TfToken("physics"));

            physxSim->attachStage(stageId);

            // material should be there
            PxBase* baseMaterialPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(basePhysicsMaterialPath, ePTMaterial));
            CHECK(baseMaterialPtr != nullptr);
            CHECK(baseMaterialPtr->is<PxMaterial>());


            // shape should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(cubePath, ePTShape));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());

            PxShape* shape = basePtr->is<PxShape>();
            REQUIRE(shape->getNbMaterials() == 1);
            PxMaterial* material = nullptr;
            shape->getMaterials(&material, 1);
            CHECK(material->userData != nullptr);
            CHECK(material == baseMaterialPtr->is<PxMaterial>());
        }

        // Apply physics material to hierarchy
        SUBCASE("Hierarchy Setup Xform Physics Material")
        {
            UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(xform.GetPrim());
            bindingAPI.Bind(basePhysicsMaterial, UsdShadeTokens->weakerThanDescendants, TfToken("physics"));

            physxSim->attachStage(stageId);

            // material should be there
            PxBase* baseMaterialPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(basePhysicsMaterialPath, ePTMaterial));
            CHECK(baseMaterialPtr != nullptr);
            CHECK(baseMaterialPtr->is<PxMaterial>());


            // shape should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(cubePath, ePTShape));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());

            PxShape* shape = basePtr->is<PxShape>();
            REQUIRE(shape->getNbMaterials() == 1);
            PxMaterial* material = nullptr;
            shape->getMaterials(&material, 1);
            CHECK(material->userData != nullptr);
            CHECK(material == baseMaterialPtr->is<PxMaterial>());
        }

        // Apply physics material to hierarchy
        SUBCASE("Hierarchy Setup Physics Binding Precedence")
        {
            // Create second physics material
            const SdfPath secondPhysicsMaterialPath = defaultPrimPath.AppendChild(TfToken("secondPhysicsMaterial"));
            UsdShadeMaterial secondPhysicsMaterial = UsdShadeMaterial::Define(stage, secondPhysicsMaterialPath);
            UsdPhysicsMaterialAPI::Apply(secondPhysicsMaterial.GetPrim());

            UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(cube.GetPrim());
            bindingAPI.Bind(secondPhysicsMaterial, UsdShadeTokens->weakerThanDescendants);


            bindingAPI = UsdShadeMaterialBindingAPI::Apply(xform.GetPrim());
            bindingAPI.Bind(basePhysicsMaterial, UsdShadeTokens->weakerThanDescendants, TfToken("physics"));

            physxSim->attachStage(stageId);

            // material should be there
            PxBase* baseMaterialPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(basePhysicsMaterialPath, ePTMaterial));
            CHECK(baseMaterialPtr != nullptr);
            CHECK(baseMaterialPtr->is<PxMaterial>());

            PxBase* secondMaterialPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(secondPhysicsMaterialPath, ePTMaterial));
            CHECK(secondMaterialPtr != nullptr);
            CHECK(secondMaterialPtr->is<PxMaterial>());

            // shape should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(cubePath, ePTShape));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());

            PxShape* shape = basePtr->is<PxShape>();
            REQUIRE(shape->getNbMaterials() == 1);
            PxMaterial* material = nullptr;
            shape->getMaterials(&material, 1);
            CHECK(material->userData != nullptr);
            CHECK(material == baseMaterialPtr->is<PxMaterial>());
        }

        // Apply physics material to hierarchy
        SUBCASE("Hierarchy Setup Physics Weak Descendent")
        {
            // Create second physics material
            const SdfPath secondPhysicsMaterialPath = defaultPrimPath.AppendChild(TfToken("secondPhysicsMaterial"));
            UsdShadeMaterial secondPhysicsMaterial = UsdShadeMaterial::Define(stage, secondPhysicsMaterialPath);
            UsdPhysicsMaterialAPI::Apply(secondPhysicsMaterial.GetPrim());

            UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(cube.GetPrim());
            bindingAPI.Bind(basePhysicsMaterial, UsdShadeTokens->weakerThanDescendants, TfToken("physics"));


            bindingAPI = UsdShadeMaterialBindingAPI::Apply(xform.GetPrim());
            bindingAPI.Bind(secondPhysicsMaterial, UsdShadeTokens->weakerThanDescendants, TfToken("physics"));

            physxSim->attachStage(stageId);

            // material should be there
            PxBase* baseMaterialPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(basePhysicsMaterialPath, ePTMaterial));
            CHECK(baseMaterialPtr != nullptr);
            CHECK(baseMaterialPtr->is<PxMaterial>());

            PxBase* secondMaterialPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(secondPhysicsMaterialPath, ePTMaterial));
            CHECK(secondMaterialPtr != nullptr);
            CHECK(secondMaterialPtr->is<PxMaterial>());

            // shape should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(cubePath, ePTShape));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());

            PxShape* shape = basePtr->is<PxShape>();
            REQUIRE(shape->getNbMaterials() == 1);
            PxMaterial* material = nullptr;
            shape->getMaterials(&material, 1);
            CHECK(material->userData != nullptr);
            CHECK(material == baseMaterialPtr->is<PxMaterial>());
        }

        // Apply physics material to hierarchy
        SUBCASE("Hierarchy Setup Physics Strong Descendent")
        {
            // Create second physics material
            const SdfPath secondPhysicsMaterialPath = defaultPrimPath.AppendChild(TfToken("secondPhysicsMaterial"));
            UsdShadeMaterial secondPhysicsMaterial = UsdShadeMaterial::Define(stage, secondPhysicsMaterialPath);
            UsdPhysicsMaterialAPI::Apply(secondPhysicsMaterial.GetPrim());

            UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(cube.GetPrim());
            bindingAPI.Bind(secondPhysicsMaterial, UsdShadeTokens->weakerThanDescendants, TfToken("physics"));

            bindingAPI = UsdShadeMaterialBindingAPI::Apply(xform.GetPrim());
            bindingAPI.Bind(basePhysicsMaterial, UsdShadeTokens->strongerThanDescendants, TfToken("physics"));

            physxSim->attachStage(stageId);

            // material should be there
            PxBase* baseMaterialPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(basePhysicsMaterialPath, ePTMaterial));
            CHECK(baseMaterialPtr != nullptr);
            CHECK(baseMaterialPtr->is<PxMaterial>());

            PxBase* secondMaterialPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(secondPhysicsMaterialPath, ePTMaterial));
            CHECK(secondMaterialPtr != nullptr);
            CHECK(secondMaterialPtr->is<PxMaterial>());

            // shape should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(cubePath, ePTShape));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());

            PxShape* shape = basePtr->is<PxShape>();
            REQUIRE(shape->getNbMaterials() == 1);
            PxMaterial* material = nullptr;
            shape->getMaterials(&material, 1);
            CHECK(material->userData != nullptr);
            CHECK(material == baseMaterialPtr->is<PxMaterial>());
        }
    }

    SUBCASE("Point Instancer Colliders")
    {
        const SdfPath geomPointInstancerPath = defaultPrimPath.AppendChild(TfToken("pointInstancer"));
        const SdfPath boxActorPath = geomPointInstancerPath.AppendChild(TfToken("boxActor"));
        const SdfPath instancerMaterialPath = geomPointInstancerPath.AppendChild(TfToken("instancerMaterial"));
        UsdShadeMaterial instancerPhysicsMaterial = UsdShadeMaterial::Define(stage, instancerMaterialPath);
        UsdPhysicsMaterialAPI::Apply(instancerPhysicsMaterial.GetPrim());

        UsdGeomCube cubeGeom = UsdGeomCube::Define(stage, boxActorPath);

        const uint32_t numIndices = 2;
        VtArray<int> meshIndices = { 0, 0 };
        VtArray<GfVec3f> positions = { GfVec3f(-125.0, 0.0, 500.0), GfVec3f(125.0, 0.0, 500.0) };
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

        SUBCASE("RigidBody Instances")
        {
            UsdPhysicsCollisionAPI::Apply(cubeGeom.GetPrim());
            UsdPhysicsRigidBodyAPI::Apply(cubeGeom.GetPrim());

            UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(cubeGeom.GetPrim());
            bindingAPI.Bind(instancerPhysicsMaterial, UsdShadeTokens->weakerThanDescendants, TfToken("physics"));

            physxSim->attachStage(stageId);

            // material should be there
            PxBase* baseMaterialPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(instancerMaterialPath, ePTMaterial));
            REQUIRE(baseMaterialPtr != nullptr);
            CHECK(baseMaterialPtr->is<PxMaterial>());


            // shape should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxActorPath, ePTShape));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());

            PxShape* shape = basePtr->is<PxShape>();
            REQUIRE(shape->getNbMaterials() == 1);
            PxMaterial* material = nullptr;
            shape->getMaterials(&material, 1);
            CHECK(material->userData != nullptr);
            CHECK(material == baseMaterialPtr->is<PxMaterial>());
        }

        SUBCASE("Colliders Instances")
        {
            UsdPhysicsCollisionAPI::Apply(cubeGeom.GetPrim());
            UsdPhysicsRigidBodyAPI::Apply(shapeList.GetPrim());

            UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(cubeGeom.GetPrim());
            bindingAPI.Bind(basePhysicsMaterial, UsdShadeTokens->weakerThanDescendants, TfToken("physics"));

            physxSim->attachStage(stageId);

            // material should be there
            PxBase* baseMaterialPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(basePhysicsMaterialPath, ePTMaterial));
            CHECK(baseMaterialPtr != nullptr);
            CHECK(baseMaterialPtr->is<PxMaterial>());


            // shape should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(boxActorPath, ePTShape));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());

            PxShape* shape = basePtr->is<PxShape>();
            REQUIRE(shape->getNbMaterials() == 1);
            PxMaterial* material = nullptr;
            shape->getMaterials(&material, 1);
            CHECK(material->userData != nullptr);
            CHECK(material == baseMaterialPtr->is<PxMaterial>());
        }

    }

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

//-----------------------------------------------------------------------------
// Material attribute parsing and change tests
TEST_CASE_TEMPLATE("RigidBodyMaterialChangeTests", T, USDChange, FabricChange)
{
    T changeTemplate;
    const float epsilon = 1.0e-5f;

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

    // Create physics scene
    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);

    // Create base physics material
    const SdfPath basePhysicsMaterialPath = defaultPrimPath.AppendChild(TfToken("basePhysicsMaterial"));
    UsdShadeMaterial basePhysicsMaterial = UsdShadeMaterial::Define(stage, basePhysicsMaterialPath);
    REQUIRE(bool(basePhysicsMaterial.GetPrim()));
    UsdPhysicsMaterialAPI usdMaterial = UsdPhysicsMaterialAPI::Apply(basePhysicsMaterial.GetPrim());
    REQUIRE(bool(usdMaterial));
    PhysxSchemaPhysxMaterialAPI physxMaterial = PhysxSchemaPhysxMaterialAPI::Apply(basePhysicsMaterial.GetPrim());
    REQUIRE(bool(physxMaterial));

    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    SUBCASE("Dynamic Friction")
    {
        physxSim->attachStage(stageId);
        const TfToken& changeToken = UsdPhysicsTokens.Get()->physicsDynamicFriction;

        float value = changeTemplate.template getAttributeValue<float>(basePhysicsMaterialPath, changeToken);
        // check default value
        CHECK(fabsf(value - 0.0f) < epsilon);

        // material should be created
        PxMaterial* mat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK(fabsf(mat->getDynamicFriction() - value) < epsilon);

        // redo with authored parsing value:
        physxSim->detachStage();
        value = 0.5f;
        usdMaterial.CreateDynamicFrictionAttr().Set(value);
        physxSim->attachStage(stageId);

        // material should be created
        mat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        
        CHECK(fabsf(mat->getDynamicFriction() - value) < epsilon);

        value = 0.9f;
        changeTemplate.setAttributeValue(basePhysicsMaterialPath, changeToken, value);
        changeTemplate.broadcastChanges();

        // same ptr should be returned - no reparsing
        PxMaterial* newMat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK_EQ(mat, newMat);
        CHECK(fabsf(newMat->getDynamicFriction() - value) < epsilon);
    }

    SUBCASE("Static Friction")
    {
        physxSim->attachStage(stageId);
        const TfToken& changeToken = UsdPhysicsTokens.Get()->physicsStaticFriction;

        float value = changeTemplate.template getAttributeValue<float>(basePhysicsMaterialPath, changeToken);
        // check default value
        CHECK(fabsf(value - 0.0f) < epsilon);

        // material should be created
        PxMaterial* mat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK(fabsf(mat->getStaticFriction() - value) < epsilon);

        // redo with authored parsing value:
        physxSim->detachStage();
        value = 0.5f;
        usdMaterial.CreateStaticFrictionAttr().Set(value);
        physxSim->attachStage(stageId);

        // material should be created
        mat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);

        CHECK(fabsf(mat->getStaticFriction() - value) < epsilon);

        value = 0.9f;
        changeTemplate.setAttributeValue(basePhysicsMaterialPath, changeToken, value);
        changeTemplate.broadcastChanges();

        // same ptr should be returned - no reparsing
        PxMaterial* newMat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK_EQ(mat, newMat);
        CHECK(fabsf(newMat->getStaticFriction() - value) < epsilon);
    }

    SUBCASE("Restitution")
    {
        physxSim->attachStage(stageId);
        const TfToken& changeToken = UsdPhysicsTokens.Get()->physicsRestitution;

        float value = changeTemplate.template getAttributeValue<float>(basePhysicsMaterialPath, changeToken);
        // check default value
        CHECK(fabsf(value - 0.0f) < epsilon);

        // material should be created
        PxMaterial* mat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK(fabsf(mat->getRestitution() - value) < epsilon);

        // redo with authored parsing value:
        physxSim->detachStage();
        value = 0.5f;
        usdMaterial.CreateRestitutionAttr().Set(value);
        physxSim->attachStage(stageId);

        // material should be created
        mat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);

        CHECK(fabsf(mat->getRestitution() - value) < epsilon);

        value = 0.9f;
        changeTemplate.setAttributeValue(basePhysicsMaterialPath, changeToken, value);
        changeTemplate.broadcastChanges();

        // same ptr should be returned - no reparsing
        PxMaterial* newMat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK_EQ(mat, newMat);
        CHECK(fabsf(newMat->getRestitution() - value) < epsilon);
    }

    SUBCASE("Density")
    {
        const float massEps = 0.1f;
        // create rigid body
        const SdfPath boxPath = defaultPrimPath.AppendChild(TfToken("box"));
        UsdGeomCube cubeGeom = UsdGeomCube::Define(stage, boxPath);
        pxr::VtArray<pxr::GfVec3f> extent;
        const float size = 100.f;
        extent.push_back(GfVec3f(-0.5f * size));
        extent.push_back(GfVec3f(0.5f * size));
        REQUIRE(cubeGeom.CreateExtentAttr().Set(extent));
        REQUIRE(cubeGeom.CreateSizeAttr().Set(double(size)));

        UsdPhysicsCollisionAPI::Apply(cubeGeom.GetPrim());
        UsdPhysicsRigidBodyAPI::Apply(cubeGeom.GetPrim());
        UsdShadeMaterialBindingAPI bindAPI = UsdShadeMaterialBindingAPI::Apply(cubeGeom.GetPrim());
        bindAPI.Bind(basePhysicsMaterial, UsdShadeTokens->fallbackStrength, TfToken("physics"));

        physxSim->attachStage(stageId);
        const TfToken& changeToken = UsdPhysicsTokens.Get()->physicsDensity;

        float value = changeTemplate.template getAttributeValue<float>(basePhysicsMaterialPath, changeToken);
        // check default value
        CHECK(fabsf(value - 0.0f) < epsilon);

        // box should be 1000kg from default fallback material
        PxRigidDynamic* rbBox = getPhysxBaseDerivedFromPathChecked<PxRigidDynamic>(boxPath, ePTActor);
        float massBox = rbBox->getMass();
        CHECK(fabsf(massBox - 1000.0f) < massEps);

        // redo with authored parsing value:
        physxSim->detachStage();

        value = 0.002f;
        float parseMass = value * size * size * size;
        usdMaterial.CreateDensityAttr().Set(value);
        physxSim->attachStage(stageId);

        // material should be created
        rbBox = getPhysxBaseDerivedFromPathChecked<PxRigidDynamic>(boxPath, ePTActor);
        massBox = rbBox->getMass();
        CHECK(fabsf(massBox - parseMass) < massEps);

        value = 0.003f;
        float changeMass = value * size * size * size;
        changeTemplate.setAttributeValue(basePhysicsMaterialPath, changeToken, value);
        changeTemplate.broadcastChanges();
        usdMaterial.CreateDensityAttr().Set(value);  // make sure density in USD is synced

        // mass update is not in change listener so we need an update
        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // same ptr should be returned - no reparsing
        // material should be created
        rbBox = getPhysxBaseDerivedFromPathChecked<PxRigidDynamic>(boxPath, ePTActor);
        massBox = rbBox->getMass();
        CHECK(fabsf(massBox - changeMass) < massEps);
    }

    SUBCASE("Friction Combine Mode")
    {
        physxSim->attachStage(stageId);
        const TfToken& changeToken = PhysxSchemaTokens->physxMaterialFrictionCombineMode;

        TfToken fallbackToken = changeTemplate.template getAttributeValue<TfToken>(basePhysicsMaterialPath, changeToken);
        // check default value
        CHECK_EQ(fallbackToken, PhysxSchemaTokens->average);

        // material should be created
        PxMaterial* mat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK_EQ(PxCombineMode::eAVERAGE, mat->getFrictionCombineMode());

        // redo with authored parsing value:
        physxSim->detachStage();
        physxMaterial.CreateFrictionCombineModeAttr().Set(PhysxSchemaTokens->min);
        physxSim->attachStage(stageId);

        // material should be created
        mat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK_EQ(PxCombineMode::eMIN, mat->getFrictionCombineMode());

        changeTemplate.setAttributeValue(basePhysicsMaterialPath, changeToken, PhysxSchemaTokens->multiply);
        changeTemplate.broadcastChanges();

        // same ptr should be returned - no reparsing
        PxMaterial* newMat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK_EQ(mat, newMat);
        CHECK_EQ(PxCombineMode::eMULTIPLY, mat->getFrictionCombineMode());
    }

    SUBCASE("Restitution Combine Mode")
    {
        physxSim->attachStage(stageId);
        const TfToken& changeToken = PhysxSchemaTokens->physxMaterialRestitutionCombineMode;

        TfToken fallbackToken = changeTemplate.template getAttributeValue<TfToken>(basePhysicsMaterialPath, changeToken);
        // check default value
        CHECK_EQ(fallbackToken, PhysxSchemaTokens->average);

        // material should be created
        PxMaterial* mat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK_EQ(PxCombineMode::eAVERAGE, mat->getRestitutionCombineMode());

        // redo with authored parsing value:
        physxSim->detachStage();
        physxMaterial.CreateRestitutionCombineModeAttr().Set(PhysxSchemaTokens->min);
        physxSim->attachStage(stageId);

        // material should be created
        mat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK_EQ(PxCombineMode::eMIN, mat->getRestitutionCombineMode());

        changeTemplate.setAttributeValue(basePhysicsMaterialPath, changeToken, PhysxSchemaTokens->multiply);
        changeTemplate.broadcastChanges();

        // same ptr should be returned - no reparsing
        PxMaterial* newMat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK_EQ(mat, newMat);
        CHECK_EQ(PxCombineMode::eMULTIPLY, mat->getRestitutionCombineMode());
    }

    SUBCASE("Damping Combine Mode")
    {
        physxSim->attachStage(stageId);
        const TfToken& changeToken = PhysxSchemaTokens->physxMaterialDampingCombineMode;

        TfToken fallbackToken = changeTemplate.template getAttributeValue<TfToken>(basePhysicsMaterialPath, changeToken);
        // check default value
        CHECK_EQ(fallbackToken, PhysxSchemaTokens->average);

        // material should be created
        PxMaterial* mat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK_EQ(PxCombineMode::eAVERAGE, mat->getDampingCombineMode());

        // redo with authored parsing value:
        physxSim->detachStage();
        physxMaterial.CreateDampingCombineModeAttr().Set(PhysxSchemaTokens->min);
        physxSim->attachStage(stageId);

        // material should be created
        mat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK_EQ(PxCombineMode::eMIN, mat->getDampingCombineMode());

        changeTemplate.setAttributeValue(basePhysicsMaterialPath, changeToken, PhysxSchemaTokens->multiply);
        changeTemplate.broadcastChanges();

        // same ptr should be returned - no reparsing
        PxMaterial* newMat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK_EQ(mat, newMat);
        CHECK_EQ(PxCombineMode::eMULTIPLY, mat->getDampingCombineMode());
    }

    SUBCASE("CompliantContactDamping")
    {
        physxSim->attachStage(stageId);
        const TfToken& changeToken = PhysxSchemaTokens->physxMaterialCompliantContactDamping;

        // standard case if user error with no stiffness set
        const float parseSetValue = 0.1f;
        const float changeSetValue = 0.2f;

        float parseExpectValue = 0.0f;
        float changeExpectValue = 0.0f;
        bool compliantFlagSetExpect = false;

        SUBCASE("StiffnessSet")
        {
            parseExpectValue = parseSetValue;
            changeExpectValue = changeSetValue;
            compliantFlagSetExpect = true;
            physxMaterial.CreateCompliantContactStiffnessAttr().Set(1.0f);
        }

        float fallback = changeTemplate.template getAttributeValue<float>(basePhysicsMaterialPath, changeToken);
        // check default value
        CHECK(fallback == 0.0f);

        // material should be created
        PxMaterial* mat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK_EQ(0.0f, mat->getDamping());
        CHECK_EQ(compliantFlagSetExpect, mat->getRestitution() < 0.0f);

        // redo with authored parsing value:
        physxSim->detachStage();

        physxMaterial.CreateCompliantContactDampingAttr().Set(parseSetValue);
        physxSim->attachStage(stageId);

        // material should be created
        mat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);

        CHECK_EQ(parseExpectValue, mat->getDamping());
        CHECK_EQ(compliantFlagSetExpect, mat->getRestitution() < 0.0f);

        changeTemplate.setAttributeValue(basePhysicsMaterialPath, changeToken, changeSetValue);
        changeTemplate.broadcastChanges();

        // same ptr should be returned - no reparsing
        PxMaterial* newMat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK_EQ(mat, newMat);
        CHECK_EQ(changeExpectValue, mat->getDamping());
        CHECK_EQ(compliantFlagSetExpect, mat->getRestitution() < 0.0f);
    }

    SUBCASE("CompliantContactStiffness")
    {
        physxSim->attachStage(stageId);
        const TfToken& changeToken = PhysxSchemaTokens->physxMaterialCompliantContactStiffness;

        float fallback = changeTemplate.template getAttributeValue<float>(basePhysicsMaterialPath, changeToken);
        // check default value
        CHECK(fallback == 0.0f);

        // material should be created
        PxMaterial* mat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK_EQ(0.0f, mat->getDamping());
        CHECK_GE(mat->getRestitution(), 0.0f);

        // redo with authored parsing value:
        physxSim->detachStage();

        const float stiffness = 0.54321f;
        const float restitution = 0.12345f;
        usdMaterial.CreateRestitutionAttr().Set(restitution);
        physxMaterial.CreateCompliantContactStiffnessAttr().Set(stiffness);
        physxSim->attachStage(stageId);

        // material should be created
        mat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);

        CHECK_EQ(-stiffness, mat->getRestitution()); // set negative

        // disable via change:
        changeTemplate.setAttributeValue(basePhysicsMaterialPath, changeToken, 0.0f);
        changeTemplate.broadcastChanges();

        // same ptr should be returned - no reparsing
        PxMaterial* newMat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK_EQ(mat, newMat);
        CHECK_EQ(restitution, mat->getRestitution());

        // reenable via change:
        changeTemplate.setAttributeValue(basePhysicsMaterialPath, changeToken, stiffness);
        changeTemplate.broadcastChanges();

        CHECK_EQ(-stiffness, mat->getRestitution());
    }

    SUBCASE("CompliantContactAccelerationSpring")
    {
        physxSim->attachStage(stageId);
        const TfToken& changeToken = PhysxSchemaTokens->physxMaterialCompliantContactAccelerationSpring;

        const bool fallback = changeTemplate.template getAttributeValue<bool>(basePhysicsMaterialPath, changeToken);
        CHECK_EQ(fallback, false); // default should be off

        // material should be created
        PxMaterial* mat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK(mat != nullptr);
        CHECK_EQ(mat->getFlags().isSet(PxMaterialFlag::eCOMPLIANT_ACCELERATION_SPRING), false);

        // redo with authored parsing value:
        physxSim->detachStage();

        const float stiffness = 0.54321f;
        const float restitution = 0.12345f;
        usdMaterial.CreateRestitutionAttr().Set(restitution);
        physxMaterial.CreateCompliantContactStiffnessAttr().Set(stiffness);
        physxMaterial.CreateCompliantContactAccelerationSpringAttr().Set(true);
        physxSim->attachStage(stageId);

        // material should be created
        mat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK_EQ(mat->getFlags().isSet(PxMaterialFlag::eCOMPLIANT_ACCELERATION_SPRING), true);

        // disable via change:
        changeTemplate.setAttributeValue(basePhysicsMaterialPath, changeToken, false);
        changeTemplate.broadcastChanges();

        // same ptr should be returned - no reparsing
        PxMaterial* newMat = getPhysxBaseDerivedFromPathChecked<PxMaterial>(basePhysicsMaterialPath, ePTMaterial);
        CHECK_EQ(mat, newMat);
        CHECK_EQ(mat->getFlags().isSet(PxMaterialFlag::eCOMPLIANT_ACCELERATION_SPRING), false);

        // reenable via change:
        changeTemplate.setAttributeValue(basePhysicsMaterialPath, changeToken, true);
        changeTemplate.broadcastChanges();
        CHECK_EQ(mat->getFlags().isSet(PxMaterialFlag::eCOMPLIANT_ACCELERATION_SPRING), true);
    }

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}

TEST_CASE("Multi Material Parsing Tests",
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
    const double metersPerUnit = pxr::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    // Create physics scene
    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);


    // Create physics materials
    const SdfPath physicsMaterialPath0 = defaultPrimPath.AppendChild(TfToken("physicsMaterial0"));
    UsdShadeMaterial physicsMaterial0 = UsdShadeMaterial::Define(stage, physicsMaterialPath0);

    const SdfPath physicsMaterialPath1 = defaultPrimPath.AppendChild(TfToken("physicsMaterial1"));
    UsdShadeMaterial physicsMaterial1 = UsdShadeMaterial::Define(stage, physicsMaterialPath1);

    const SdfPath planeMeshPath = defaultPrimPath.AppendChild(TfToken("planeMesh"));

    const float halfSize = 10.0f;
    const std::vector<GfVec3f> points = {
                GfVec3f(0.0, -halfSize, -halfSize),
            GfVec3f(0.0, halfSize, -halfSize),
            GfVec3f(0.0, halfSize, halfSize),
            GfVec3f(0.0, -halfSize, halfSize)
    };
    const std::vector<GfVec3f> normals =
    {
        GfVec3f(1, 0, 0), GfVec3f(1, 0, 0), GfVec3f(1, 0, 0), GfVec3f(1, 0, 0)
    };
    const std::vector<int> indices = {
        0, 1, 2,
        0, 2, 3
    };
    const std::vector<int> vertexCounts = { 3, 3 };

    UsdGeomMesh planeMesh = createMesh(stage, planeMeshPath, points, normals, indices, vertexCounts);
    UsdPhysicsCollisionAPI::Apply(planeMesh.GetPrim());

    {
        const SdfPath subsetPath = planeMeshPath.AppendChild(TfToken("subset0"));
        UsdGeomSubset subset = UsdGeomSubset::Define(stage, subsetPath);
        subset.CreateElementTypeAttr().Set(TfToken("face"));
        subset.CreateFamilyNameAttr().Set(UsdShadeTokens->materialBind);
        VtArray<int> indices = { 0 };
        subset.CreateIndicesAttr().Set(indices);
        UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(subset.GetPrim());       
        bindingAPI.Bind(physicsMaterial0, UsdShadeTokens->weakerThanDescendants);
    }

    {
        const SdfPath subsetPath = planeMeshPath.AppendChild(TfToken("subset1"));
        UsdGeomSubset subset = UsdGeomSubset::Define(stage, subsetPath);
        subset.CreateElementTypeAttr().Set(TfToken("face"));
        subset.CreateFamilyNameAttr().Set(UsdShadeTokens->materialBind);
        VtArray<int> indices = { 1 };
        subset.CreateIndicesAttr().Set(indices);
        UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(subset.GetPrim());
        bindingAPI.Bind(physicsMaterial1, UsdShadeTokens->weakerThanDescendants);
    }

    SUBCASE("Triangle Mesh")
    {
        UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(planeMesh.GetPrim());
        meshAPI.CreateApproximationAttr().Set(UsdPhysicsTokens->none);

        SUBCASE("Base Setup")
        {
            UsdPhysicsMaterialAPI::Apply(physicsMaterial0.GetPrim());
            UsdPhysicsMaterialAPI::Apply(physicsMaterial1.GetPrim());

            physxSim->attachStage(stageId);

            // material should be there
            PxBase* baseMaterialPtr0 = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsMaterialPath0, ePTMaterial));
            CHECK(baseMaterialPtr0 != nullptr);
            CHECK(baseMaterialPtr0->is<PxMaterial>());

            PxBase* baseMaterialPtr1 = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsMaterialPath1, ePTMaterial));
            CHECK(baseMaterialPtr1 != nullptr);
            CHECK(baseMaterialPtr1->is<PxMaterial>());

            // shape should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(planeMeshPath, ePTShape));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());

            PxShape* shape = basePtr->is<PxShape>();
            // compare to 3 we added the default material as last
            REQUIRE(shape->getNbMaterials() == 3);
            PxMaterial* material = nullptr;
            shape->getMaterials(&material, 1);
            CHECK(material == baseMaterialPtr0);

            shape->getMaterials(&material, 1, 1);
            CHECK(material == baseMaterialPtr1);
        }

        SUBCASE("Default Material Fallback")
        {
            UsdPhysicsMaterialAPI::Apply(physicsMaterial0.GetPrim());

            physxSim->attachStage(stageId);

            // material should be there
            PxBase* baseMaterialPtr0 = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsMaterialPath0, ePTMaterial));
            CHECK(baseMaterialPtr0 != nullptr);
            CHECK(baseMaterialPtr0->is<PxMaterial>());

            PxBase* baseMaterialPtr1 = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsMaterialPath1, ePTMaterial));
            CHECK(baseMaterialPtr1 == nullptr);

            // shape should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(planeMeshPath, ePTShape));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());

            PxShape* shape = basePtr->is<PxShape>();
            
            REQUIRE(shape->getNbMaterials() == 2);
            PxMaterial* material = nullptr;
            shape->getMaterials(&material, 1);
            CHECK(material == baseMaterialPtr0);

            shape->getMaterials(&material, 1, 1);
            CHECK(material->userData == nullptr);
        }

    }

#if USE_PHYSX_GPU
    SUBCASE("SDF Mesh")
    {
        UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(planeMesh.GetPrim());
        meshAPI.CreateApproximationAttr().Set(TfToken("sdf"));
        PhysxSchemaPhysxSDFMeshCollisionAPI::Apply(planeMesh.GetPrim());

        SUBCASE("Base Setup - Default Material")
        {
            physxSim->attachStage(stageId);

            // material should be there
            PxBase* baseMaterialPtr0 = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsMaterialPath0, ePTMaterial));
            CHECK(baseMaterialPtr0 == nullptr);

            PxBase* baseMaterialPtr1 = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsMaterialPath1, ePTMaterial));
            CHECK(baseMaterialPtr1 == nullptr);

            // shape should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(planeMeshPath, ePTShape));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());

            PxShape* shape = basePtr->is<PxShape>();
            REQUIRE(shape->getNbMaterials() == 1);
            PxMaterial* material = nullptr;
            shape->getMaterials(&material, 1);
            CHECK(material->userData == nullptr);
        }

        SUBCASE("Default Material Fallback")
        {
            UsdPhysicsMaterialAPI::Apply(physicsMaterial0.GetPrim());
            UsdPhysicsMaterialAPI::Apply(physicsMaterial1.GetPrim());

            physxSim->attachStage(stageId);

            // material should be there
            PxBase* baseMaterialPtr0 = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsMaterialPath0, ePTMaterial));
            CHECK(baseMaterialPtr0 != nullptr);
            CHECK(baseMaterialPtr0->is<PxMaterial>());

            // shape should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(planeMeshPath, ePTShape));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());

            PxShape* shape = basePtr->is<PxShape>();

            REQUIRE(shape->getNbMaterials() == 1);
            PxMaterial* material = nullptr;
            shape->getMaterials(&material, 1);
            CHECK(material->userData == nullptr);
        }

        SUBCASE("Mesh Material Usage")
        {
            const SdfPath physicsMaterialPath = defaultPrimPath.AppendChild(TfToken("physicsMaterial"));
            UsdShadeMaterial physicsMaterial = UsdShadeMaterial::Define(stage, physicsMaterialPath);
            UsdPhysicsMaterialAPI::Apply(physicsMaterial.GetPrim());

            UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(planeMesh.GetPrim());
            bindingAPI.Bind(physicsMaterial, UsdShadeTokens->strongerThanDescendants);

            physxSim->attachStage(stageId);

            // material should be there
            PxBase* baseMaterialPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsMaterialPath, ePTMaterial));
            CHECK(baseMaterialPtr != nullptr);
            CHECK(baseMaterialPtr->is<PxMaterial>());

            // shape should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(planeMeshPath, ePTShape));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());

            PxShape* shape = basePtr->is<PxShape>();

            REQUIRE(shape->getNbMaterials() == 1);
            PxMaterial* material = nullptr;
            shape->getMaterials(&material, 1);
            CHECK(material == baseMaterialPtr);
        }
    }
#endif

    SUBCASE("Convex Mesh")
    {
        UsdPhysicsMeshCollisionAPI meshAPI = UsdPhysicsMeshCollisionAPI::Apply(planeMesh.GetPrim());
        meshAPI.CreateApproximationAttr().Set(UsdPhysicsTokens->convexHull);

        SUBCASE("Base Setup - Default Material")
        {
            physxSim->attachStage(stageId);

            // material should be there
            PxBase* baseMaterialPtr0 = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsMaterialPath0, ePTMaterial));
            CHECK(baseMaterialPtr0 == nullptr);

            PxBase* baseMaterialPtr1 = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsMaterialPath1, ePTMaterial));
            CHECK(baseMaterialPtr1 == nullptr);

            // shape should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(planeMeshPath, ePTShape));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());

            PxShape* shape = basePtr->is<PxShape>();
            REQUIRE(shape->getNbMaterials() == 1);
            PxMaterial* material = nullptr;
            shape->getMaterials(&material, 1);
            CHECK(material->userData == nullptr);
        }

        SUBCASE("Default Material Fallback")
        {
            UsdPhysicsMaterialAPI::Apply(physicsMaterial0.GetPrim());
            UsdPhysicsMaterialAPI::Apply(physicsMaterial1.GetPrim());

            physxSim->attachStage(stageId);

            // material should be there
            PxBase* baseMaterialPtr0 = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsMaterialPath0, ePTMaterial));
            CHECK(baseMaterialPtr0 != nullptr);
            CHECK(baseMaterialPtr0->is<PxMaterial>());

            // shape should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(planeMeshPath, ePTShape));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());

            PxShape* shape = basePtr->is<PxShape>();

            REQUIRE(shape->getNbMaterials() == 1);
            PxMaterial* material = nullptr;
            shape->getMaterials(&material, 1);
            CHECK(material->userData == nullptr);
        }

        SUBCASE("Mesh Material Usage")
        {
            const SdfPath physicsMaterialPath = defaultPrimPath.AppendChild(TfToken("physicsMaterial"));
            UsdShadeMaterial physicsMaterial = UsdShadeMaterial::Define(stage, physicsMaterialPath);
            UsdPhysicsMaterialAPI::Apply(physicsMaterial.GetPrim());

            UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(planeMesh.GetPrim());
            bindingAPI.Bind(physicsMaterial, UsdShadeTokens->strongerThanDescendants);

            physxSim->attachStage(stageId);

            // material should be there
            PxBase* baseMaterialPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(physicsMaterialPath, ePTMaterial));
            CHECK(baseMaterialPtr != nullptr);
            CHECK(baseMaterialPtr->is<PxMaterial>());

            // shape should be there
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(planeMeshPath, ePTShape));
            CHECK(basePtr != nullptr);
            CHECK(basePtr->is<PxShape>());

            PxShape* shape = basePtr->is<PxShape>();

            REQUIRE(shape->getNbMaterials() == 1);
            PxMaterial* material = nullptr;
            shape->getMaterials(&material, 1);
            CHECK(material == baseMaterialPtr);
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
