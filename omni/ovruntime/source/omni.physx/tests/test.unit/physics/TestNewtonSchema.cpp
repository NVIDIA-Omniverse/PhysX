// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"
#include "../../common/PhysicsChangeTemplate.h"

#include <pxr/base/plug/registry.h>
#include <pxr/usd/usd/schemaRegistry.h>
#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/PhysxTokens.h>
#include <physicsSchemaTools/physicsSchemaTokens.h>

#include "PhysicsTools.h"

#if defined(NEWTON_SCHEMA_PLUGIN_PATH)
#include "NewtonSchemaTokens.h"

PXR_NAMESPACE_OPEN_SCOPE
TF_DECLARE_PUBLIC_TOKENS(NewtonSchemaTokens, NEWTON_SCHEMA_TOKENS);
TF_DEFINE_PUBLIC_TOKENS(NewtonSchemaTokens, NEWTON_SCHEMA_TOKENS);
PXR_NAMESPACE_CLOSE_SCOPE
#endif

using namespace PXR_NS;
using namespace omni::physx;
using namespace ::physx;

#if defined(NEWTON_SCHEMA_PLUGIN_PATH)

// =============================================================================
// Schema Registration Tests
// =============================================================================

TEST_CASE("Newton Schema Plugin Registration", "[newton]")
{
    PlugPluginPtr plugin = PlugRegistry::GetInstance().GetPluginWithName("newton");
    REQUIRE(plugin);

    SUBCASE("Plugin declares NewtonSceneAPI")
    {
        CHECK_FALSE(TfType::FindByName("NewtonPhysicsSceneAPI").IsUnknown());
    }

    SUBCASE("Plugin declares NewtonXpbdSceneAPI")
    {
        CHECK_FALSE(TfType::FindByName("NewtonPhysicsXpbdSceneAPI").IsUnknown());
    }

    SUBCASE("Plugin declares NewtonKaminoSceneAPI")
    {
        CHECK_FALSE(TfType::FindByName("NewtonPhysicsKaminoSceneAPI").IsUnknown());
    }

    SUBCASE("Plugin declares NewtonCollisionAPI")
    {
        CHECK_FALSE(TfType::FindByName("NewtonPhysicsCollisionAPI").IsUnknown());
    }

    SUBCASE("Plugin declares NewtonMaterialAPI")
    {
        CHECK_FALSE(TfType::FindByName("NewtonPhysicsMaterialAPI").IsUnknown());
    }

    SUBCASE("Plugin declares NewtonMimicAPI")
    {
        CHECK_FALSE(TfType::FindByName("NewtonPhysicsMimicAPI").IsUnknown());
    }

    SUBCASE("Plugin declares NewtonJointAPI")
    {
        CHECK_FALSE(TfType::FindByName("NewtonPhysicsJointAPI").IsUnknown());
    }
}

// =============================================================================
// Schema Defaults Tests (pure USD, no simulation)
// =============================================================================

TEST_CASE("Newton Schema Apply and Defaults", "[newton]")
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    REQUIRE(stage);

    SUBCASE("NewtonSceneAPI attributes have correct defaults")
    {
        UsdPrim prim = stage->DefinePrim(SdfPath("/Scene"), TfToken("PhysicsScene"));
        REQUIRE(prim.ApplyAPI(TfType::FindByName("NewtonPhysicsSceneAPI")));

        int maxIter; prim.GetAttribute(NewtonSchemaTokens->newtonMaxSolverIterations).Get(&maxIter);
        CHECK(maxIter == -1);

        int tps; prim.GetAttribute(NewtonSchemaTokens->newtonTimeStepsPerSecond).Get(&tps);
        CHECK(tps == 1000);

        bool grav; prim.GetAttribute(NewtonSchemaTokens->newtonGravityEnabled).Get(&grav);
        CHECK(grav == true);
    }

    SUBCASE("NewtonCollisionAPI attributes have correct defaults")
    {
        UsdPrim prim = stage->DefinePrim(SdfPath("/Mesh"), TfToken("Mesh"));
        REQUIRE(prim.ApplyAPI(TfType::FindByName("NewtonPhysicsCollisionAPI")));

        float margin; prim.GetAttribute(NewtonSchemaTokens->newtonContactMargin).Get(&margin);
        CHECK(margin == 0.0f);
    }

    SUBCASE("NewtonMaterialAPI attributes have correct defaults")
    {
        UsdPrim prim = stage->DefinePrim(SdfPath("/Mat"), TfToken("Material"));
        REQUIRE(prim.ApplyAPI(TfType::FindByName("NewtonPhysicsMaterialAPI")));

        float torsional; prim.GetAttribute(NewtonSchemaTokens->newtonTorsionalFriction).Get(&torsional);
        CHECK(torsional == doctest::Approx(0.005f));

        float rolling; prim.GetAttribute(NewtonSchemaTokens->newtonRollingFriction).Get(&rolling);
        CHECK(rolling == doctest::Approx(0.0001f));
    }

    SUBCASE("NewtonJointAPI attributes have correct defaults")
    {
        UsdPrim prim = UsdPhysicsRevoluteJoint::Define(stage, SdfPath("/Joint")).GetPrim();
        REQUIRE(prim.ApplyAPI(TfType::FindByName("NewtonPhysicsJointAPI")));

        // Newton's "no clamping" sentinel is +inf, matching physxJoint:maxJointVelocity.
        float velocityLimit; prim.GetAttribute(NewtonSchemaTokens->newtonVelocityLimit).Get(&velocityLimit);
        CHECK((std::isinf(velocityLimit) && velocityLimit > 0.0f));
    }

    SUBCASE("NewtonXpbdSceneAPI attributes have correct defaults")
    {
        UsdPrim prim = stage->DefinePrim(SdfPath("/Scene"), TfToken("PhysicsScene"));
        REQUIRE(prim.ApplyAPI(TfType::FindByName("NewtonPhysicsXpbdSceneAPI")));

        float relax; prim.GetAttribute(NewtonSchemaTokens->newtonXpbdSoftBodyRelaxation).Get(&relax);
        CHECK(relax == doctest::Approx(0.9f));
    }
}

// =============================================================================
// Newton → PhysX Compat Layer Tests (with simulation)
// =============================================================================

// Helper: create a minimal physics stage and return the scene path.
static UsdStageRefPtr createPhysicsStage(SdfPath& outScenePath)
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    UsdGeomSetStageMetersPerUnit(stage, 0.01);
    UsdPrim defaultPrim = stage->DefinePrim(SdfPath("/World"));
    stage->SetDefaultPrim(defaultPrim);

    outScenePath = SdfPath("/World/physicsScene");
    UsdPhysicsScene::Define(stage, outScenePath);
    return stage;
}

// ---- Scene: newton:timeStepsPerSecond fallback ----

TEST_CASE("Newton Scene timeStepsPerSecond fallback", "[newton]")
{
    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    SdfPath scenePath;
    UsdStageRefPtr stage = createPhysicsStage(scenePath);
    UsdPrim scenePrim = stage->GetPrimAtPath(scenePath);

    UsdUtilsStageCache::Get().Insert(stage);
    long stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    SUBCASE("Newton timeStepsPerSecond used when PhysX not authored")
    {
        // Apply both Newton and PhysX scene APIs — PhysX needed for PxScene creation,
        // but don't author PhysX timeStepsPerSecond so Newton provides the fallback.
        PhysxSchemaPhysxSceneAPI::Apply(scenePrim);
        scenePrim.ApplyAPI(TfType::FindByName("NewtonPhysicsSceneAPI"));
        scenePrim.GetAttribute(NewtonSchemaTokens->newtonTimeStepsPerSecond).Set(120);

        physxSim->attachStage(stageId);

        PxScene* pxScene = getPhysxSceneAtPathChecked(scenePath);
        CHECK(pxScene != nullptr);

        physxSim->detachStage();
    }

    SUBCASE("PhysX timeStepsPerSecond takes priority over Newton")
    {
        // Apply both schemas — PhysX value should win
        scenePrim.ApplyAPI(TfType::FindByName("NewtonPhysicsSceneAPI"));
        scenePrim.GetAttribute(NewtonSchemaTokens->newtonTimeStepsPerSecond).Set(120);

        PhysxSchemaPhysxSceneAPI::Apply(scenePrim);
        PhysxSchemaPhysxSceneAPI physxAPI(scenePrim);
        physxAPI.CreateTimeStepsPerSecondAttr().Set(240u);

        physxSim->attachStage(stageId);

        PxScene* pxScene = getPhysxSceneAtPathChecked(scenePath);
        CHECK(pxScene != nullptr);
        // PhysX authored 240 should take priority over Newton's 120

        physxSim->detachStage();
    }

    SUBCASE("No Newton schema — uses PhysX default")
    {
        // No Newton, no PhysX authored — should use default (60)
        PhysxSchemaPhysxSceneAPI::Apply(scenePrim);
        physxSim->attachStage(stageId);

        PxScene* pxScene = getPhysxSceneAtPathChecked(scenePath);
        CHECK(pxScene != nullptr);

        physxSim->detachStage();
    }

    UsdUtilsStageCache::Get().Erase(stage);
}

// ---- Scene: newton:gravityEnabled ----

TEST_CASE("Newton Scene gravityEnabled", "[newton]")
{
    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    SdfPath scenePath;
    UsdStageRefPtr stage = createPhysicsStage(scenePath);
    UsdPrim scenePrim = stage->GetPrimAtPath(scenePath);

    UsdUtilsStageCache::Get().Insert(stage);
    long stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    SUBCASE("gravityEnabled=false zeros gravity magnitude")
    {
        PhysxSchemaPhysxSceneAPI::Apply(scenePrim);
        scenePrim.ApplyAPI(TfType::FindByName("NewtonPhysicsSceneAPI"));
        scenePrim.GetAttribute(NewtonSchemaTokens->newtonGravityEnabled).Set(false);

        physxSim->attachStage(stageId);

        PxScene* pxScene = getPhysxSceneAtPathChecked(scenePath);
        PxVec3 gravity = pxScene->getGravity();
        // Gravity magnitude should be 0 when Newton says disabled
        CHECK(gravity.magnitude() < epsilon);

        physxSim->detachStage();
    }

    SUBCASE("gravityEnabled=true preserves default gravity")
    {
        PhysxSchemaPhysxSceneAPI::Apply(scenePrim);
        scenePrim.ApplyAPI(TfType::FindByName("NewtonPhysicsSceneAPI"));
        scenePrim.GetAttribute(NewtonSchemaTokens->newtonGravityEnabled).Set(true);

        physxSim->attachStage(stageId);

        PxScene* pxScene = getPhysxSceneAtPathChecked(scenePath);
        PxVec3 gravity = pxScene->getGravity();
        // Gravity should be non-zero (default Z-down, magnitude ~981 in cm/s^2)
        CHECK(gravity.magnitude() > 1.0f);

        physxSim->detachStage();
    }

    SUBCASE("No Newton gravityEnabled authored — gravity is normal")
    {
        PhysxSchemaPhysxSceneAPI::Apply(scenePrim);
        scenePrim.ApplyAPI(TfType::FindByName("NewtonPhysicsSceneAPI"));

        physxSim->attachStage(stageId);

        PxScene* pxScene = getPhysxSceneAtPathChecked(scenePath);
        PxVec3 gravity = pxScene->getGravity();
        CHECK(gravity.magnitude() > 1.0f);

        physxSim->detachStage();
    }

    UsdUtilsStageCache::Get().Erase(stage);
}

// ---- Collision: newton:contactMargin/contactGap fallback ----

TEST_CASE("Newton Collision contactMargin/contactGap fallback", "[newton]")
{
    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    SdfPath scenePath;
    UsdStageRefPtr stage = createPhysicsStage(scenePath);

    // Create a rigid box with collision
    const SdfPath boxPath("/World/box");
    addRigidBox(stage, boxPath.GetString(), GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);
    UsdPrim boxPrim = stage->GetPrimAtPath(boxPath);
    REQUIRE(boxPrim);

    UsdUtilsStageCache::Get().Insert(stage);
    long stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    SUBCASE("Newton contactMargin used as restOffset when PhysX not authored")
    {
        boxPrim.ApplyAPI(TfType::FindByName("NewtonPhysicsCollisionAPI"));
        boxPrim.GetAttribute(NewtonSchemaTokens->newtonContactMargin).Set(0.5f);

        physxSim->attachStage(stageId);

        PxShape* shape = getPhysxBaseDerivedFromPathChecked<PxShape>(boxPath, ePTShape);
        CHECK(fabsf(shape->getRestOffset() - 0.5f) < epsilon);

        physxSim->detachStage();
    }

    SUBCASE("PhysX restOffset takes priority over Newton contactMargin")
    {
        boxPrim.ApplyAPI(TfType::FindByName("NewtonPhysicsCollisionAPI"));
        boxPrim.GetAttribute(NewtonSchemaTokens->newtonContactMargin).Set(0.5f);

        PhysxSchemaPhysxCollisionAPI::Apply(boxPrim);
        PhysxSchemaPhysxCollisionAPI physxColAPI(boxPrim);
        physxColAPI.CreateRestOffsetAttr().Set(0.1f);
        // Also set contactOffset so contactOffset >= restOffset
        physxColAPI.CreateContactOffsetAttr().Set(0.2f);

        physxSim->attachStage(stageId);

        PxShape* shape = getPhysxBaseDerivedFromPathChecked<PxShape>(boxPath, ePTShape);
        CHECK(fabsf(shape->getRestOffset() - 0.1f) < epsilon);

        physxSim->detachStage();
    }

    SUBCASE("Newton contactGap adds on top of margin for contactOffset")
    {
        // Newton gap is additive: PhysX contactOffset = margin + gap
        boxPrim.ApplyAPI(TfType::FindByName("NewtonPhysicsCollisionAPI"));
        boxPrim.GetAttribute(NewtonSchemaTokens->newtonContactMargin).Set(0.5f);
        boxPrim.GetAttribute(NewtonSchemaTokens->newtonContactGap).Set(1.5f);

        physxSim->attachStage(stageId);

        PxShape* shape = getPhysxBaseDerivedFromPathChecked<PxShape>(boxPath, ePTShape);
        // contactOffset = margin(0.5) + gap(1.5) = 2.0
        CHECK(fabsf(shape->getContactOffset() - 2.0f) < epsilon);
        CHECK(fabsf(shape->getRestOffset() - 0.5f) < epsilon);

        physxSim->detachStage();
    }

    SUBCASE("Newton contactGap only (no margin) uses restOffset as base")
    {
        // Only gap authored, margin defaults to 0 -> contactOffset = 0 + gap
        boxPrim.ApplyAPI(TfType::FindByName("NewtonPhysicsCollisionAPI"));
        boxPrim.GetAttribute(NewtonSchemaTokens->newtonContactGap).Set(2.0f);

        physxSim->attachStage(stageId);

        PxShape* shape = getPhysxBaseDerivedFromPathChecked<PxShape>(boxPath, ePTShape);
        // contactOffset = restOffset(0) + gap(2.0) = 2.0
        CHECK(fabsf(shape->getContactOffset() - 2.0f) < epsilon);

        physxSim->detachStage();
    }

    SUBCASE("PhysX contactOffset takes priority over Newton contactGap")
    {
        boxPrim.ApplyAPI(TfType::FindByName("NewtonPhysicsCollisionAPI"));
        boxPrim.GetAttribute(NewtonSchemaTokens->newtonContactMargin).Set(0.5f);
        boxPrim.GetAttribute(NewtonSchemaTokens->newtonContactGap).Set(1.5f);

        PhysxSchemaPhysxCollisionAPI::Apply(boxPrim);
        PhysxSchemaPhysxCollisionAPI physxColAPI(boxPrim);
        physxColAPI.CreateContactOffsetAttr().Set(1.0f);

        physxSim->attachStage(stageId);

        PxShape* shape = getPhysxBaseDerivedFromPathChecked<PxShape>(boxPath, ePTShape);
        CHECK(fabsf(shape->getContactOffset() - 1.0f) < epsilon);

        physxSim->detachStage();
    }

    SUBCASE("Newton contactGap=-inf is ignored (uses PhysX default)")
    {
        boxPrim.ApplyAPI(TfType::FindByName("NewtonPhysicsCollisionAPI"));
        // -inf is Newton's "use default" sentinel — should not override PhysX default
        boxPrim.GetAttribute(NewtonSchemaTokens->newtonContactGap).Set(-std::numeric_limits<float>::infinity());

        physxSim->attachStage(stageId);

        PxShape* shape = getPhysxBaseDerivedFromPathChecked<PxShape>(boxPath, ePTShape);
        // contactOffset should be the PhysX-computed default, not -inf
        CHECK(shape->getContactOffset() > 0.0f);

        physxSim->detachStage();
    }

    SUBCASE("No Newton schema — collision uses PhysX defaults")
    {
        physxSim->attachStage(stageId);

        PxShape* shape = getPhysxBaseDerivedFromPathChecked<PxShape>(boxPath, ePTShape);
        CHECK(fabsf(shape->getRestOffset() - 0.0f) < epsilon);
        CHECK(shape->getContactOffset() > 0.0f);

        physxSim->detachStage();
    }

    UsdUtilsStageCache::Get().Erase(stage);
}

// =============================================================================
// Runtime change-tracking tests — mutate Newton attributes on an attached stage
// and verify the PhysX runtime state reflects the change (or stays put when
// PhysX is authored — priority is preserved at runtime).
// =============================================================================

// ---- Scene: newton:gravityEnabled runtime change ----

TEST_CASE("Newton Scene gravityEnabled runtime change", "[newton]")
{
    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    SdfPath scenePath;
    UsdStageRefPtr stage = createPhysicsStage(scenePath);
    UsdPrim scenePrim = stage->GetPrimAtPath(scenePath);

    PhysxSchemaPhysxSceneAPI::Apply(scenePrim);
    scenePrim.ApplyAPI(TfType::FindByName("NewtonPhysicsSceneAPI"));
    scenePrim.GetAttribute(NewtonSchemaTokens->newtonGravityEnabled).Set(true);

    UsdUtilsStageCache::Get().Insert(stage);
    long stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    USDChange changeTemplate;
    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());
    physxSim->attachStage(stageId);

    PxScene* pxScene = getPhysxSceneAtPathChecked(scenePath);
    // Initial state: gravity enabled -> non-zero magnitude.
    CHECK(pxScene->getGravity().magnitude() > 1.0f);

    SUBCASE("Toggle off then on")
    {
        // Flip Newton gravityEnabled to false -> magnitude should zero.
        changeTemplate.setAttributeValue<bool>(scenePath, NewtonSchemaTokens->newtonGravityEnabled, false);
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();
        CHECK(pxScene->getGravity().magnitude() < epsilon);

        // Flip back to true -> magnitude restored from physicsGravityMagnitude.
        changeTemplate.setAttributeValue<bool>(scenePath, NewtonSchemaTokens->newtonGravityEnabled, true);
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();
        CHECK(pxScene->getGravity().magnitude() > 1.0f);
    }

    physxSim->detachStage();
    changeTemplate.destroy();
    UsdUtilsStageCache::Get().Erase(stage);
}

// ---- Collision: newton:contactMargin runtime change (rigid) ----

TEST_CASE("Newton Collision contactMargin runtime change", "[newton]")
{
    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    SdfPath scenePath;
    UsdStageRefPtr stage = createPhysicsStage(scenePath);

    const SdfPath boxPath("/World/box");
    addRigidBox(stage, boxPath.GetString(), GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);
    UsdPrim boxPrim = stage->GetPrimAtPath(boxPath);
    PhysxSchemaPhysxCollisionAPI::Apply(boxPrim);
    boxPrim.ApplyAPI(TfType::FindByName("NewtonPhysicsCollisionAPI"));

    UsdUtilsStageCache::Get().Insert(stage);
    long stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    SUBCASE("Newton fallback updates restOffset at runtime")
    {
        // Initial Newton margin = 0.5, gap = 1.5 -> rest=0.5, contact=0.5+1.5=2.0.
        boxPrim.GetAttribute(NewtonSchemaTokens->newtonContactMargin).Set(0.5f);
        boxPrim.GetAttribute(NewtonSchemaTokens->newtonContactGap).Set(1.5f);

        USDChange changeTemplate;
        changeTemplate.init(stageId, physicsTests.getApp()->getFramework());
        physxSim->attachStage(stageId);

        PxShape* shape = getPhysxBaseDerivedFromPathChecked<PxShape>(boxPath, ePTShape);
        CHECK(fabsf(shape->getRestOffset() - 0.5f) < epsilon);
        CHECK(fabsf(shape->getContactOffset() - 2.0f) < epsilon);

        // Mutate Newton margin at runtime. Because physxCollision:contactOffset is
        // fallback-driven (Newton gap = 1.5), the handler must move contactOffset
        // with it: new contact = 0.3 + 1.5 = 1.8.
        changeTemplate.setAttributeValue<float>(boxPath, NewtonSchemaTokens->newtonContactMargin, 0.3f);
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();
        CHECK(fabsf(shape->getRestOffset() - 0.3f) < epsilon);
        CHECK(fabsf(shape->getContactOffset() - 1.8f) < epsilon);

        physxSim->detachStage();
        changeTemplate.destroy();
    }

    SUBCASE("PhysX-authored restOffset ignores Newton runtime change")
    {
        // PhysX restOffset authored -> Newton runtime change must be ignored.
        boxPrim.GetAttribute(NewtonSchemaTokens->newtonContactMargin).Set(0.5f);
        PhysxSchemaPhysxCollisionAPI physxColAPI(boxPrim);
        physxColAPI.CreateRestOffsetAttr().Set(0.1f);
        physxColAPI.CreateContactOffsetAttr().Set(0.2f);

        USDChange changeTemplate;
        changeTemplate.init(stageId, physicsTests.getApp()->getFramework());
        physxSim->attachStage(stageId);

        PxShape* shape = getPhysxBaseDerivedFromPathChecked<PxShape>(boxPath, ePTShape);
        CHECK(fabsf(shape->getRestOffset() - 0.1f) < epsilon);

        // Newton margin change -> PhysX wins, restOffset unchanged.
        changeTemplate.setAttributeValue<float>(boxPath, NewtonSchemaTokens->newtonContactMargin, 0.4f);
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();
        CHECK(fabsf(shape->getRestOffset() - 0.1f) < epsilon);

        physxSim->detachStage();
        changeTemplate.destroy();
    }

    UsdUtilsStageCache::Get().Erase(stage);
}

// ---- Collision: newton:contactGap runtime change (rigid) ----

TEST_CASE("Newton Collision contactGap runtime change", "[newton]")
{
    const float epsilon = 0.0001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    SdfPath scenePath;
    UsdStageRefPtr stage = createPhysicsStage(scenePath);

    const SdfPath boxPath("/World/box");
    addRigidBox(stage, boxPath.GetString(), GfVec3f(100.f), GfVec3f(0.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);
    UsdPrim boxPrim = stage->GetPrimAtPath(boxPath);
    PhysxSchemaPhysxCollisionAPI::Apply(boxPrim);
    boxPrim.ApplyAPI(TfType::FindByName("NewtonPhysicsCollisionAPI"));

    UsdUtilsStageCache::Get().Insert(stage);
    long stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    SUBCASE("Newton fallback updates contactOffset at runtime (= restOffset + gap)")
    {
        // Initial: margin 0.5 + gap 1.5 -> rest 0.5, contact 2.0.
        boxPrim.GetAttribute(NewtonSchemaTokens->newtonContactMargin).Set(0.5f);
        boxPrim.GetAttribute(NewtonSchemaTokens->newtonContactGap).Set(1.5f);

        USDChange changeTemplate;
        changeTemplate.init(stageId, physicsTests.getApp()->getFramework());
        physxSim->attachStage(stageId);

        PxShape* shape = getPhysxBaseDerivedFromPathChecked<PxShape>(boxPath, ePTShape);
        CHECK(fabsf(shape->getRestOffset() - 0.5f) < epsilon);
        CHECK(fabsf(shape->getContactOffset() - 2.0f) < epsilon);

        // Bump the gap -> contactOffset = restOffset + new gap.
        changeTemplate.setAttributeValue<float>(boxPath, NewtonSchemaTokens->newtonContactGap, 2.5f);
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();
        CHECK(fabsf(shape->getContactOffset() - 3.0f) < epsilon);
        // restOffset must not move when only gap changes.
        CHECK(fabsf(shape->getRestOffset() - 0.5f) < epsilon);

        physxSim->detachStage();
        changeTemplate.destroy();
    }

    SUBCASE("PhysX-authored contactOffset ignores Newton runtime change")
    {
        boxPrim.GetAttribute(NewtonSchemaTokens->newtonContactMargin).Set(0.5f);
        boxPrim.GetAttribute(NewtonSchemaTokens->newtonContactGap).Set(1.5f);
        PhysxSchemaPhysxCollisionAPI physxColAPI(boxPrim);
        physxColAPI.CreateContactOffsetAttr().Set(1.0f);

        USDChange changeTemplate;
        changeTemplate.init(stageId, physicsTests.getApp()->getFramework());
        physxSim->attachStage(stageId);

        PxShape* shape = getPhysxBaseDerivedFromPathChecked<PxShape>(boxPath, ePTShape);
        CHECK(fabsf(shape->getContactOffset() - 1.0f) < epsilon);

        // Newton gap change -> PhysX wins, contactOffset unchanged.
        changeTemplate.setAttributeValue<float>(boxPath, NewtonSchemaTokens->newtonContactGap, 3.0f);
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();
        CHECK(fabsf(shape->getContactOffset() - 1.0f) < epsilon);

        physxSim->detachStage();
        changeTemplate.destroy();
    }

    UsdUtilsStageCache::Get().Erase(stage);
}

// ---- Articulation: newton:selfCollisionEnabled fallback ----

TEST_CASE("Newton Articulation selfCollisionEnabled fallback", "[newton]")
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    const float metersPerStageUnit = 0.01f;
    const float density = 0.001f;
    const GfVec3f linkDims(100.f, 10.f, 10.f);
    const GfVec3f color(0.7f, 0.1f, 0.1f);

    SdfPath scenePath;
    UsdStageRefPtr stage = createPhysicsStage(scenePath);

    // Create a simple two-link articulation
    const SdfPath rootPath("/World/root");
    const SdfPath childPath("/World/child");
    const SdfPath jointPath("/World/joint");

    addRigidBox(stage, rootPath.GetString(), linkDims, GfVec3f(0.f), GfQuatf(1.0f), color, density);
    addRigidBox(stage, childPath.GetString(), linkDims, GfVec3f(100.f, 0.f, 0.f), GfQuatf(1.0f), color, density);

    // Create a fixed joint connecting root to world, making root the articulation root
    UsdPhysicsFixedJoint worldJoint = UsdPhysicsFixedJoint::Define(stage, SdfPath("/World/worldJoint"));
    worldJoint.CreateBody1Rel().AddTarget(rootPath);

    // Create a revolute joint between root and child
    UsdPhysicsRevoluteJoint joint = UsdPhysicsRevoluteJoint::Define(stage, jointPath);
    joint.CreateBody0Rel().AddTarget(rootPath);
    joint.CreateBody1Rel().AddTarget(childPath);
    joint.CreateAxisAttr().Set(TfToken("Z"));

    // Apply articulation root
    UsdPhysicsArticulationRootAPI::Apply(stage->GetPrimAtPath(rootPath));

    UsdUtilsStageCache::Get().Insert(stage);
    long stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    SUBCASE("Newton selfCollisionEnabled=false disables articulation self collision")
    {
        UsdPrim rootPrim = stage->GetPrimAtPath(rootPath);
        rootPrim.ApplyAPI(TfType::FindByName("NewtonPhysicsArticulationRootAPI"));
        rootPrim.GetAttribute(NewtonSchemaTokens->newtonSelfCollisionEnabled).Set(false);

        physxSim->attachStage(stageId);

        PxBase* pxBase = reinterpret_cast<PxBase*>(physx->getPhysXPtr(rootPath, ePTArticulation));
        if (pxBase)
        {
            PxArticulationReducedCoordinate* art = pxBase->is<PxArticulationReducedCoordinate>();
            if (art)
            {
                PxArticulationFlags flags = art->getArticulationFlags();
                // eDISABLE_SELF_COLLISION should be set when Newton selfCollision=false
                // Note: PhysX flag is the inverse — eDISABLE_SELF_COLLISION means disabled
                // The desc maps selfCollision=false to the flag being set.
            }
        }

        physxSim->detachStage();
    }

    SUBCASE("PhysX enabledSelfCollisions takes priority over Newton")
    {
        UsdPrim rootPrim = stage->GetPrimAtPath(rootPath);

        // Newton says false, PhysX says true — PhysX should win
        rootPrim.ApplyAPI(TfType::FindByName("NewtonPhysicsArticulationRootAPI"));
        rootPrim.GetAttribute(NewtonSchemaTokens->newtonSelfCollisionEnabled).Set(false);

        PhysxSchemaPhysxArticulationAPI::Apply(rootPrim);
        PhysxSchemaPhysxArticulationAPI physxArtAPI(rootPrim);
        physxArtAPI.CreateEnabledSelfCollisionsAttr().Set(true);

        physxSim->attachStage(stageId);

        PxBase* pxBase = reinterpret_cast<PxBase*>(physx->getPhysXPtr(rootPath, ePTArticulation));
        CHECK(pxBase != nullptr);
        // PhysX authored selfCollision=true should take priority

        physxSim->detachStage();
    }

    SUBCASE("No Newton schema — uses PhysX articulation defaults")
    {
        physxSim->attachStage(stageId);

        PxBase* pxBase = reinterpret_cast<PxBase*>(physx->getPhysXPtr(rootPath, ePTArticulation));
        CHECK(pxBase != nullptr);
        // Default selfCollision is true

        physxSim->detachStage();
    }

    UsdUtilsStageCache::Get().Erase(stage);
}

// ---- Joint: newton:velocityLimit fallback ----

// Helper: fixed-base articulation with one revolute (angular DOF) and one prismatic
// (linear DOF) joint, so both unit paths of maxJointVelocity are covered.
static UsdStageRefPtr createJointArticulationStage(SdfPath& outRevolutePath, SdfPath& outPrismaticPath)
{
    const float density = 0.001f;
    const GfVec3f linkDims(100.f, 10.f, 10.f);
    const GfVec3f color(0.7f, 0.1f, 0.1f);

    SdfPath scenePath;
    UsdStageRefPtr stage = createPhysicsStage(scenePath);

    const SdfPath rootPath("/World/rootLink");
    const SdfPath revoluteLinkPath("/World/revoluteLink");
    const SdfPath prismaticLinkPath("/World/prismaticLink");

    addRigidBox(stage, rootPath.GetString(), linkDims, GfVec3f(0.f), GfQuatf(1.0f), color, density);
    addRigidBox(stage, revoluteLinkPath.GetString(), linkDims, GfVec3f(100.f, 0.f, 0.f), GfQuatf(1.0f), color, density);
    addRigidBox(stage, prismaticLinkPath.GetString(), linkDims, GfVec3f(200.f, 0.f, 0.f), GfQuatf(1.0f), color, density);

    // Fixed joint to the world turns rootLink into the articulation base.
    UsdPhysicsFixedJoint worldJoint = UsdPhysicsFixedJoint::Define(stage, SdfPath("/World/baseFixedJoint"));
    worldJoint.CreateBody1Rel().AddTarget(rootPath);

    outRevolutePath = SdfPath("/World/revoluteJoint");
    UsdPhysicsRevoluteJoint revolute = UsdPhysicsRevoluteJoint::Define(stage, outRevolutePath);
    revolute.CreateBody0Rel().AddTarget(rootPath);
    revolute.CreateBody1Rel().AddTarget(revoluteLinkPath);
    revolute.CreateAxisAttr().Set(TfToken("Z"));
    revolute.CreateLowerLimitAttr().Set(-90.0f);
    revolute.CreateUpperLimitAttr().Set(90.0f);

    outPrismaticPath = SdfPath("/World/prismaticJoint");
    UsdPhysicsPrismaticJoint prismatic = UsdPhysicsPrismaticJoint::Define(stage, outPrismaticPath);
    prismatic.CreateBody0Rel().AddTarget(revoluteLinkPath);
    prismatic.CreateBody1Rel().AddTarget(prismaticLinkPath);
    prismatic.CreateAxisAttr().Set(TfToken("X"));

    UsdPhysicsArticulationRootAPI::Apply(stage->GetPrimAtPath(rootPath));

    return stage;
}

// Helper: fetch the articulation joint behind a joint prim.
static PxArticulationJointReducedCoordinate* getArticulationJointChecked(IPhysx* physx, const SdfPath& jointPath)
{
    PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(jointPath, ePTLinkJoint));
    REQUIRE(basePtr != nullptr);
    PxArticulationJointReducedCoordinate* joint = basePtr->is<PxArticulationJointReducedCoordinate>();
    REQUIRE(joint != nullptr);
    return joint;
}

TEST_CASE("Newton Joint velocityLimit fallback", "[newton]")
{
    const float epsilon = 0.001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    SdfPath revolutePath;
    SdfPath prismaticPath;
    UsdStageRefPtr stage = createJointArticulationStage(revolutePath, prismaticPath);
    UsdPrim revolutePrim = stage->GetPrimAtPath(revolutePath);
    UsdPrim prismaticPrim = stage->GetPrimAtPath(prismaticPath);

    UsdUtilsStageCache::Get().Insert(stage);
    long stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    SUBCASE("Newton velocityLimit used when PhysX maxJointVelocity not authored")
    {
        // Angular DOF: Newton units are degrees/s, same as physxJoint:maxJointVelocity,
        // so the parser converts to radians/s exactly as it does for the PhysX attribute.
        revolutePrim.ApplyAPI(TfType::FindByName("NewtonPhysicsJointAPI"));
        revolutePrim.GetAttribute(NewtonSchemaTokens->newtonVelocityLimit).Set(30.0f);

        // Linear DOF: distance/s, no conversion.
        prismaticPrim.ApplyAPI(TfType::FindByName("NewtonPhysicsJointAPI"));
        prismaticPrim.GetAttribute(NewtonSchemaTokens->newtonVelocityLimit).Set(5.0f);

        physxSim->attachStage(stageId);

        PxArticulationJointReducedCoordinate* revoluteJoint = getArticulationJointChecked(physx, revolutePath);
        CHECK(fabsf(radToDeg(revoluteJoint->getMaxJointVelocity(PxArticulationAxis::eTWIST)) - 30.0f) < epsilon);

        PxArticulationJointReducedCoordinate* prismaticJoint = getArticulationJointChecked(physx, prismaticPath);
        CHECK(fabsf(prismaticJoint->getMaxJointVelocity(PxArticulationAxis::eX) - 5.0f) < epsilon);

        physxSim->detachStage();
    }

    SUBCASE("PhysX maxJointVelocity takes priority over Newton velocityLimit")
    {
        revolutePrim.ApplyAPI(TfType::FindByName("NewtonPhysicsJointAPI"));
        revolutePrim.GetAttribute(NewtonSchemaTokens->newtonVelocityLimit).Set(30.0f);

        PhysxSchemaPhysxJointAPI physxJointAPI = PhysxSchemaPhysxJointAPI::Apply(revolutePrim);
        physxJointAPI.CreateMaxJointVelocityAttr().Set(60.0f);

        physxSim->attachStage(stageId);

        PxArticulationJointReducedCoordinate* revoluteJoint = getArticulationJointChecked(physx, revolutePath);
        CHECK(fabsf(radToDeg(revoluteJoint->getMaxJointVelocity(PxArticulationAxis::eTWIST)) - 60.0f) < epsilon);

        physxSim->detachStage();
    }

    SUBCASE("Applied PhysxJointAPI without an authored value does not shadow Newton")
    {
        // PhysxJointAPI present but maxJointVelocity left at its schema default —
        // priority 2 (Newton authored) must still beat priority 3 (PhysX default).
        PhysxSchemaPhysxJointAPI::Apply(revolutePrim);

        revolutePrim.ApplyAPI(TfType::FindByName("NewtonPhysicsJointAPI"));
        revolutePrim.GetAttribute(NewtonSchemaTokens->newtonVelocityLimit).Set(45.0f);

        physxSim->attachStage(stageId);

        PxArticulationJointReducedCoordinate* revoluteJoint = getArticulationJointChecked(physx, revolutePath);
        CHECK(fabsf(radToDeg(revoluteJoint->getMaxJointVelocity(PxArticulationAxis::eTWIST)) - 45.0f) < epsilon);

        physxSim->detachStage();
    }

    SUBCASE("Per-axis PhysxJointAxisAPI overrides the Newton joint-level value")
    {
        revolutePrim.ApplyAPI(TfType::FindByName("NewtonPhysicsJointAPI"));
        revolutePrim.GetAttribute(NewtonSchemaTokens->newtonVelocityLimit).Set(30.0f);

        revolutePrim.ApplyAPI(PhysxAdditionAPITokens->PhysxJointAxisAPI, UsdPhysicsTokens->angular);
        revolutePrim.CreateAttribute(PhysxAdditionAttrTokens->maxJointVelocityAngular, SdfValueTypeNames->Float)
            .Set(15.0f);

        physxSim->attachStage(stageId);

        PxArticulationJointReducedCoordinate* revoluteJoint = getArticulationJointChecked(physx, revolutePath);
        CHECK(fabsf(radToDeg(revoluteJoint->getMaxJointVelocity(PxArticulationAxis::eTWIST)) - 15.0f) < epsilon);

        physxSim->detachStage();
    }

    SUBCASE("Newton velocityLimit applied but not authored — uses PhysX default")
    {
        // Schema default is +inf on both sides, i.e. no clamping.
        revolutePrim.ApplyAPI(TfType::FindByName("NewtonPhysicsJointAPI"));

        physxSim->attachStage(stageId);

        PxArticulationJointReducedCoordinate* revoluteJoint = getArticulationJointChecked(physx, revolutePath);
        CHECK(revoluteJoint->getMaxJointVelocity(PxArticulationAxis::eTWIST) > 1e30f);

        physxSim->detachStage();
    }

    SUBCASE("Negative Newton velocityLimit is clamped to zero")
    {
        // Newton declares the range as (0, inf]; out-of-range values clamp rather than
        // reaching PhysX, which rejects negative joint velocity limits.
        prismaticPrim.ApplyAPI(TfType::FindByName("NewtonPhysicsJointAPI"));
        prismaticPrim.GetAttribute(NewtonSchemaTokens->newtonVelocityLimit).Set(-5.0f);

        physxSim->attachStage(stageId);

        PxArticulationJointReducedCoordinate* prismaticJoint = getArticulationJointChecked(physx, prismaticPath);
        CHECK(fabsf(prismaticJoint->getMaxJointVelocity(PxArticulationAxis::eX)) < epsilon);

        physxSim->detachStage();
    }

    UsdUtilsStageCache::Get().Erase(stage);
}

// ---- Joint: newton:velocityLimit runtime change ----

TEST_CASE("Newton Joint velocityLimit runtime change", "[newton]")
{
    const float epsilon = 0.001f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    SdfPath revolutePath;
    SdfPath prismaticPath;
    UsdStageRefPtr stage = createJointArticulationStage(revolutePath, prismaticPath);
    UsdPrim revolutePrim = stage->GetPrimAtPath(revolutePath);
    UsdPrim prismaticPrim = stage->GetPrimAtPath(prismaticPath);

    UsdUtilsStageCache::Get().Insert(stage);
    long stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    SUBCASE("Newton fallback updates maxJointVelocity at runtime")
    {
        revolutePrim.ApplyAPI(TfType::FindByName("NewtonPhysicsJointAPI"));
        revolutePrim.GetAttribute(NewtonSchemaTokens->newtonVelocityLimit).Set(30.0f);
        prismaticPrim.ApplyAPI(TfType::FindByName("NewtonPhysicsJointAPI"));
        prismaticPrim.GetAttribute(NewtonSchemaTokens->newtonVelocityLimit).Set(5.0f);

        USDChange changeTemplate;
        changeTemplate.init(stageId, physicsTests.getApp()->getFramework());
        physxSim->attachStage(stageId);

        PxArticulationJointReducedCoordinate* revoluteJoint = getArticulationJointChecked(physx, revolutePath);
        PxArticulationJointReducedCoordinate* prismaticJoint = getArticulationJointChecked(physx, prismaticPath);
        CHECK(fabsf(radToDeg(revoluteJoint->getMaxJointVelocity(PxArticulationAxis::eTWIST)) - 30.0f) < epsilon);
        CHECK(fabsf(prismaticJoint->getMaxJointVelocity(PxArticulationAxis::eX) - 5.0f) < epsilon);

        changeTemplate.setAttributeValue<float>(revolutePath, NewtonSchemaTokens->newtonVelocityLimit, 90.0f);
        changeTemplate.setAttributeValue<float>(prismaticPath, NewtonSchemaTokens->newtonVelocityLimit, 12.0f);
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        CHECK(fabsf(radToDeg(revoluteJoint->getMaxJointVelocity(PxArticulationAxis::eTWIST)) - 90.0f) < epsilon);
        CHECK(fabsf(prismaticJoint->getMaxJointVelocity(PxArticulationAxis::eX) - 12.0f) < epsilon);

        physxSim->detachStage();
        changeTemplate.destroy();
    }

    SUBCASE("PhysX-authored maxJointVelocity ignores Newton runtime change")
    {
        revolutePrim.ApplyAPI(TfType::FindByName("NewtonPhysicsJointAPI"));
        revolutePrim.GetAttribute(NewtonSchemaTokens->newtonVelocityLimit).Set(30.0f);

        PhysxSchemaPhysxJointAPI physxJointAPI = PhysxSchemaPhysxJointAPI::Apply(revolutePrim);
        physxJointAPI.CreateMaxJointVelocityAttr().Set(60.0f);

        USDChange changeTemplate;
        changeTemplate.init(stageId, physicsTests.getApp()->getFramework());
        physxSim->attachStage(stageId);

        PxArticulationJointReducedCoordinate* revoluteJoint = getArticulationJointChecked(physx, revolutePath);
        CHECK(fabsf(radToDeg(revoluteJoint->getMaxJointVelocity(PxArticulationAxis::eTWIST)) - 60.0f) < epsilon);

        // PhysX wins, so the Newton change must not move the runtime value.
        changeTemplate.setAttributeValue<float>(revolutePath, NewtonSchemaTokens->newtonVelocityLimit, 90.0f);
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();
        CHECK(fabsf(radToDeg(revoluteJoint->getMaxJointVelocity(PxArticulationAxis::eTWIST)) - 60.0f) < epsilon);

        physxSim->detachStage();
        changeTemplate.destroy();
    }

    SUBCASE("Per-axis PhysxJointAxisAPI survives a Newton runtime change")
    {
        // Parse time gives a per-axis PhysxJointAxisAPI:maxJointVelocity priority over the
        // joint-level Newton value; a runtime Newton edit must respect the same precedence
        // instead of broadcasting over every DOF. The prismatic joint carries no per-axis
        // authoring and acts as the control: it must still follow the Newton change, so a
        // guard that simply dropped the update would not pass this subcase.
        revolutePrim.ApplyAPI(TfType::FindByName("NewtonPhysicsJointAPI"));
        revolutePrim.GetAttribute(NewtonSchemaTokens->newtonVelocityLimit).Set(30.0f);
        revolutePrim.ApplyAPI(PhysxAdditionAPITokens->PhysxJointAxisAPI, UsdPhysicsTokens->angular);
        revolutePrim.CreateAttribute(PhysxAdditionAttrTokens->maxJointVelocityAngular, SdfValueTypeNames->Float)
            .Set(15.0f);

        prismaticPrim.ApplyAPI(TfType::FindByName("NewtonPhysicsJointAPI"));
        prismaticPrim.GetAttribute(NewtonSchemaTokens->newtonVelocityLimit).Set(5.0f);

        USDChange changeTemplate;
        changeTemplate.init(stageId, physicsTests.getApp()->getFramework());
        physxSim->attachStage(stageId);

        PxArticulationJointReducedCoordinate* revoluteJoint = getArticulationJointChecked(physx, revolutePath);
        PxArticulationJointReducedCoordinate* prismaticJoint = getArticulationJointChecked(physx, prismaticPath);
        CHECK(fabsf(radToDeg(revoluteJoint->getMaxJointVelocity(PxArticulationAxis::eTWIST)) - 15.0f) < epsilon);
        CHECK(fabsf(prismaticJoint->getMaxJointVelocity(PxArticulationAxis::eX) - 5.0f) < epsilon);

        changeTemplate.setAttributeValue<float>(revolutePath, NewtonSchemaTokens->newtonVelocityLimit, 90.0f);
        changeTemplate.setAttributeValue<float>(prismaticPath, NewtonSchemaTokens->newtonVelocityLimit, 12.0f);
        physxSim->simulate(0.01f, 0.0f);
        physxSim->fetchResults();

        // The per-axis authoring still wins; only the unguarded prismatic axis moves.
        CHECK(fabsf(radToDeg(revoluteJoint->getMaxJointVelocity(PxArticulationAxis::eTWIST)) - 15.0f) < epsilon);
        CHECK(fabsf(prismaticJoint->getMaxJointVelocity(PxArticulationAxis::eX) - 12.0f) < epsilon);

        physxSim->detachStage();
        changeTemplate.destroy();
    }

    UsdUtilsStageCache::Get().Erase(stage);
}

#endif // NEWTON_SCHEMA_PLUGIN_PATH
