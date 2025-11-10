// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"
#include "../../common/Tools.h"

#include <omni/physx/IPhysxSimulation.h>

#include "PhysicsTools.h"

using namespace pxr;
using namespace omni::physx;

struct ContactReportData
{
    std::vector<ContactEventHeader> mContactHeaders;
    std::vector<ContactData>        mContactData;

    void clear()
    {
        mContactData.clear();
        mContactHeaders.clear();
    }
};

void ContactReportEvent(const ContactEventHeader* eventHeaders, uint32_t numEventHeaders, const ContactData* contactData,
    uint32_t numContactData, void* userData)
{
    ContactReportData* contactReportData = (ContactReportData*)userData;
    for (uint32_t i = 0; i < numEventHeaders; i++)
    {
        const ContactEventHeader& header = eventHeaders[i];
        contactReportData->mContactHeaders.push_back(header);
        contactReportData->mContactHeaders.back().contactDataOffset = uint32_t(contactReportData->mContactData.size());
        for (uint32_t j = 0; j < header.numContactData; j++)
        {
            CARB_ASSERT(header.contactDataOffset + j < numContactData);
            contactReportData->mContactData.push_back(contactData[header.contactDataOffset + j]);
        }
    }
}

struct FullContactReportData
{
    std::vector<ContactEventHeader> mContactHeaders;
    std::vector<ContactData>        mContactData;
    std::vector<FrictionAnchor>     mFrictionAnchorsData;

    void clear()
    {
        mFrictionAnchorsData.clear();
        mContactData.clear();
        mContactHeaders.clear();
    }
};

void FullContactReportEvent(const ContactEventHeader* eventHeaders, uint32_t numEventHeaders, const ContactData* contactData,
    uint32_t numContactData, const FrictionAnchor* frictionAnchors, uint32_t numFrictionAnchors, void* userData)
{
    FullContactReportData* contactReportData = (FullContactReportData*)userData;
    for (uint32_t i = 0; i < numEventHeaders; i++)
    {
        const ContactEventHeader& header = eventHeaders[i];
        contactReportData->mContactHeaders.push_back(header);
        contactReportData->mContactHeaders.back().contactDataOffset = uint32_t(contactReportData->mContactData.size());
        for (uint32_t j = 0; j < header.numContactData; j++)
        {
            CARB_ASSERT(header.contactDataOffset + j < numContactData);
            contactReportData->mContactData.push_back(contactData[header.contactDataOffset + j]);
        }
        for (uint32_t j = 0; j < header.numfrictionAnchorsData; ++j)
        {
            CARB_ASSERT(header.frictionAnchorsDataOffset + j < numFrictionAnchors);
            contactReportData->mFrictionAnchorsData.push_back(frictionAnchors[header.frictionAnchorsDataOffset + j]);
        }
    }
}

//-----------------------------------------------------------------------------
// Contact report test for simple rigid bodies - prim with body and a collision
TEST_CASE("Contact Report Simple",
          "[omniphysics]"
          "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);
    IPhysx* iPhysX = physicsTests.acquirePhysxInterface();
    REQUIRE(iPhysX);

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

    // create rigid body
    addRigidBox(stage, "/World/box", GfVec3f(100.f), GfVec3f(0.0f, 0.0f, 200.0f), GfQuatf(1.0f), GfVec3f(0.7f), 0.001f);

    const SdfPath boxPath = SdfPath("/World/box");
    UsdPrim boxPrim = stage->GetPrimAtPath(boxPath);
    PhysxSchemaPhysxContactReportAPI contactReportAPI = PhysxSchemaPhysxContactReportAPI::Apply(boxPrim);
    contactReportAPI.CreateThresholdAttr().Set(3000.0f);

    // create ground plane
    addGroundPlane(stage, "/World/plane");
    const SdfPath planePath = SdfPath("/World/plane/collisionPlane");
    UsdPrim planePrim = stage->GetPrimAtPath(planePath);

    // material
    float restitution = 0.8f;
    UsdShadeMaterial matShade = UsdShadeMaterial::Define(stage, SdfPath("/physicsMaterial"));
    UsdPhysicsMaterialAPI material = UsdPhysicsMaterialAPI::Apply(matShade.GetPrim());
    material.CreateRestitutionAttr().Set(restitution);

    UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(planePrim);
    bindingAPI.Bind(matShade, UsdShadeTokens->weakerThanDescendants, TfToken("physics"));

    ContactReportData contactReportData;

    physxSim->attachStage(stageId);

    SECTION("Contact Report Found/Lost - Callback")
    {
        SubscriptionId id = physxSim->subscribePhysicsContactReportEvents(&ContactReportEvent, &contactReportData);

        contactReportData.clear();

        for (int u = 0; u < 100; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        CHECK(contactReportData.mContactHeaders.size() > 1);
        for (size_t i = 0; i < contactReportData.mContactHeaders.size(); i++)
        {
            const ContactEventHeader& header = contactReportData.mContactHeaders[i];
            CHECK(header.stageId == stageId);
            CHECK(intToPath(header.actor0) == boxPath);
            CHECK(intToPath(header.collider0) == boxPath);
            CHECK(intToPath(header.actor1) == planePath);
            CHECK(intToPath(header.collider1) == planePath);
            CHECK(header.protoIndex0 == 0xFFFFFFFF);
            CHECK(header.protoIndex1 == 0xFFFFFFFF);
            CHECK(header.contactDataOffset <= contactReportData.mContactData.size());
            CHECK(header.contactDataOffset + header.numContactData <= contactReportData.mContactData.size());
        }

        physxSim->unsubscribePhysicsContactReportEvents(id);
    }

    SECTION("Contact Report Found/Lost - Callback, IPhysX Step")
    {
        SubscriptionId id = physxSim->subscribePhysicsContactReportEvents(&ContactReportEvent, &contactReportData);

        contactReportData.clear();

        for (int u = 0; u < 100; u++)
        {
            iPhysX->updateSimulation(1.0f / 60.0f, 1.0f / 60.0f * u);
        }

        CHECK(contactReportData.mContactHeaders.size() > 1);
        for (size_t i = 0; i < contactReportData.mContactHeaders.size(); i++)
        {
            const ContactEventHeader& header = contactReportData.mContactHeaders[i];
            CHECK(header.stageId == stageId);
            CHECK(intToPath(header.actor0) == boxPath);
            CHECK(intToPath(header.collider0) == boxPath);
            CHECK(intToPath(header.actor1) == planePath);
            CHECK(intToPath(header.collider1) == planePath);
            CHECK(header.protoIndex0 == 0xFFFFFFFF);
            CHECK(header.protoIndex1 == 0xFFFFFFFF);
            CHECK(header.contactDataOffset <= contactReportData.mContactData.size());
            CHECK(header.contactDataOffset + header.numContactData <= contactReportData.mContactData.size());
        }

        physxSim->unsubscribePhysicsContactReportEvents(id);
    }

    SECTION("Contact Report Found - DirectAPI")
    {
        const ContactEventHeader* contactEventHeadersBuffer = nullptr;
        const ContactData* contactDataBuffer = nullptr;
        uint32_t numContactHeaders = 0;
        uint32_t numContactData = 0;

        for (int u = 0; u < 100; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();

            numContactHeaders = physxSim->getContactReport(&contactEventHeadersBuffer, &contactDataBuffer, numContactData);
            if (numContactHeaders)
                break;
        }

        CHECK(numContactHeaders == 1);
        REQUIRE(contactEventHeadersBuffer);
        REQUIRE(contactDataBuffer);
        for (uint32_t i = 0; i < numContactHeaders; i++)
        {
            const ContactEventHeader& header = contactEventHeadersBuffer[i];
            CHECK(header.stageId == stageId);
            CHECK(intToPath(header.actor0) == boxPath);
            CHECK(intToPath(header.collider0) == boxPath);
            CHECK(intToPath(header.actor1) == planePath);
            CHECK(intToPath(header.collider1) == planePath);
            CHECK(header.protoIndex0 == 0xFFFFFFFF);
            CHECK(header.protoIndex1 == 0xFFFFFFFF);
            CHECK(header.contactDataOffset <= numContactData);
            CHECK(header.contactDataOffset + header.numContactData <= numContactData);
        }
    }

    SECTION("Contact Report Lost - Plane Delete")
    {
        SubscriptionId id = physxSim->subscribePhysicsContactReportEvents(&ContactReportEvent, &contactReportData);

        contactReportData.clear();

        material.CreateRestitutionAttr().Set(0.0f);

        for (int u = 0; u < 50; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        CHECK(contactReportData.mContactHeaders.size() > 1);
        for (size_t i = 0; i < contactReportData.mContactHeaders.size(); i++)
        {
            const ContactEventHeader& header = contactReportData.mContactHeaders[i];
            CHECK(header.stageId == stageId);
            CHECK(intToPath(header.actor0) == boxPath);
            CHECK(intToPath(header.collider0) == boxPath);
            CHECK(intToPath(header.actor1) == planePath);
            CHECK(intToPath(header.collider1) == planePath);
            CHECK(header.protoIndex0 == 0xFFFFFFFF);
            CHECK(header.protoIndex1 == 0xFFFFFFFF);
            CHECK(header.contactDataOffset <= contactReportData.mContactData.size());
            CHECK(header.contactDataOffset + header.numContactData <= contactReportData.mContactData.size());
        }

        contactReportData.clear();

        stage->RemovePrim(planePath);

        physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();

        REQUIRE(contactReportData.mContactHeaders.size() == 1);
        const ContactEventHeader& header = contactReportData.mContactHeaders[0];
        CHECK(header.stageId == stageId);
        CHECK(intToPath(header.actor0) == boxPath);
        CHECK(intToPath(header.collider0) == boxPath);
        CHECK(intToPath(header.actor1) == planePath);
        CHECK(intToPath(header.collider1) == planePath);
        CHECK(header.type == ContactEventType::eCONTACT_LOST);
        CHECK(header.protoIndex0 == 0xFFFFFFFF);
        CHECK(header.protoIndex1 == 0xFFFFFFFF);
        CHECK(header.contactDataOffset == 0);
        CHECK(header.contactDataOffset == 0);

        physxSim->unsubscribePhysicsContactReportEvents(id);
    }

    SECTION("Contact Report Lost - Box Delete")
    {
        SubscriptionId id = physxSim->subscribePhysicsContactReportEvents(&ContactReportEvent, &contactReportData);

        contactReportData.clear();

        material.CreateRestitutionAttr().Set(0.0f);

        for (int u = 0; u < 50; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        CHECK(contactReportData.mContactHeaders.size() > 1);
        for (size_t i = 0; i < contactReportData.mContactHeaders.size(); i++)
        {
            const ContactEventHeader& header = contactReportData.mContactHeaders[i];
            CHECK(header.stageId == stageId);
            CHECK(intToPath(header.actor0) == boxPath);
            CHECK(intToPath(header.collider0) == boxPath);
            CHECK(intToPath(header.actor1) == planePath);
            CHECK(intToPath(header.collider1) == planePath);
            CHECK(header.protoIndex0 == 0xFFFFFFFF);
            CHECK(header.protoIndex1 == 0xFFFFFFFF);
            CHECK(header.contactDataOffset <= contactReportData.mContactData.size());
            CHECK(header.contactDataOffset + header.numContactData <= contactReportData.mContactData.size());
        }

        contactReportData.clear();

        stage->RemovePrim(boxPath);

        physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();

        REQUIRE(contactReportData.mContactHeaders.size() == 1);
        const ContactEventHeader& header = contactReportData.mContactHeaders[0];
        CHECK(header.stageId == stageId);
        CHECK(intToPath(header.actor0) == boxPath);
        CHECK(intToPath(header.collider0) == boxPath);
        CHECK(intToPath(header.actor1) == planePath);
        CHECK(intToPath(header.collider1) == planePath);
        CHECK(header.type == ContactEventType::eCONTACT_LOST);
        CHECK(header.protoIndex0 == 0xFFFFFFFF);
        CHECK(header.protoIndex1 == 0xFFFFFFFF);
        CHECK(header.contactDataOffset == 0);
        CHECK(header.contactDataOffset == 0);

        physxSim->unsubscribePhysicsContactReportEvents(id);
    }

    FullContactReportData fullContactReportData;

    SECTION("Contact Report - Friction Data - Callback")
    {
        SubscriptionId id = physxSim->subscribePhysicsFullContactReportEvents(&FullContactReportEvent, &fullContactReportData);

        fullContactReportData.clear();

        for (int u = 0; u < 100; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        CHECK(fullContactReportData.mContactHeaders.size() > 1);
        for (size_t i = 0; i < fullContactReportData.mContactHeaders.size(); i++)
        {
            const ContactEventHeader& header = fullContactReportData.mContactHeaders[i];
            CHECK(header.stageId == stageId);
            CHECK(intToPath(header.actor0) == boxPath);
            CHECK(intToPath(header.collider0) == boxPath);
            CHECK(intToPath(header.actor1) == planePath);
            CHECK(intToPath(header.collider1) == planePath);
            CHECK(header.protoIndex0 == 0xFFFFFFFF);
            CHECK(header.protoIndex1 == 0xFFFFFFFF);
            CHECK(header.contactDataOffset <= fullContactReportData.mContactData.size());
            CHECK(header.contactDataOffset + header.numContactData <= fullContactReportData.mContactData.size());
            CHECK(header.frictionAnchorsDataOffset <= fullContactReportData.mFrictionAnchorsData.size());
            CHECK(header.frictionAnchorsDataOffset + header.numfrictionAnchorsData <= fullContactReportData.mFrictionAnchorsData.size());
        }

        physxSim->unsubscribePhysicsFullContactReportEvents(id);
    }

    SECTION("Contact Report - Friction Data - DirectAPI")
    {
        const ContactEventHeader* contactEventHeadersBuffer = nullptr;
        const ContactData* contactDataBuffer = nullptr;
        uint32_t numContactHeaders = 0;
        uint32_t numContactData = 0;
        const FrictionAnchor* frictionAnchorDataBuffer = nullptr;
        uint32_t numFrictionAnchorData = 0;

        for (int u = 0; u < 100; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();

            numContactHeaders = physxSim->getFullContactReport(&contactEventHeadersBuffer, &contactDataBuffer, numContactData, &frictionAnchorDataBuffer, numFrictionAnchorData);
            if (numContactHeaders)
                break;
        }

        CHECK(numContactHeaders == 1);
        REQUIRE(contactEventHeadersBuffer);
        REQUIRE(contactDataBuffer);
        REQUIRE(frictionAnchorDataBuffer);
        for (uint32_t i = 0; i < numContactHeaders; i++)
        {
            const ContactEventHeader& header = contactEventHeadersBuffer[i];
            CHECK(header.stageId == stageId);
            CHECK(intToPath(header.actor0) == boxPath);
            CHECK(intToPath(header.collider0) == boxPath);
            CHECK(intToPath(header.actor1) == planePath);
            CHECK(intToPath(header.collider1) == planePath);
            CHECK(header.protoIndex0 == 0xFFFFFFFF);
            CHECK(header.protoIndex1 == 0xFFFFFFFF);
            CHECK(header.contactDataOffset <= numContactData);
            CHECK(header.contactDataOffset + header.numContactData <= numContactData);
            CHECK(header.frictionAnchorsDataOffset <= numFrictionAnchorData);
            CHECK(header.frictionAnchorsDataOffset + header.numfrictionAnchorsData <= numFrictionAnchorData);
        }
    }

    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}


//-----------------------------------------------------------------------------
// Contact report test for convex decomposition rigid bodies
TEST_CASE("Contact Report Convex Decomposition",
          "[omniphysics]"
          "[component=OmniPhysics][owner=aborovicka][priority=mandatory]")
{
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();

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

    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, SdfPath("/World/scene"));

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    // create rigid body with convex decompostion
    const SdfPath concaveMeshPath = SdfPath("/World/concaveMesh");
    pxr::UsdGeomMesh concaveMesh = createConcaveMesh(stage, concaveMeshPath, 100.0f);
    UsdPhysicsRigidBodyAPI::Apply(concaveMesh.GetPrim());
    UsdPhysicsCollisionAPI::Apply(concaveMesh.GetPrim());
    UsdPhysicsMeshCollisionAPI meshCollisionAPI = UsdPhysicsMeshCollisionAPI::Apply(concaveMesh.GetPrim());
    meshCollisionAPI.CreateApproximationAttr().Set(UsdPhysicsTokens->convexDecomposition);
    concaveMesh.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(GfVec3f(0.0f, 0.0f, 200.0f));

    PhysxSchemaPhysxContactReportAPI contactReportAPI = PhysxSchemaPhysxContactReportAPI::Apply(concaveMesh.GetPrim());
    contactReportAPI.CreateThresholdAttr().Set(3000.0f);

    // create ground plane
    addGroundPlane(stage, "/World/plane");
    const SdfPath planePath = SdfPath("/World/plane/collisionPlane");
    UsdPrim planePrim = stage->GetPrimAtPath(planePath);

    // material
    float restitution = 0.8f;
    UsdShadeMaterial matShade = UsdShadeMaterial::Define(stage, SdfPath("/physicsMaterial"));
    UsdPhysicsMaterialAPI material = UsdPhysicsMaterialAPI::Apply(matShade.GetPrim());
    material.CreateRestitutionAttr().Set(restitution);

    UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(scene.GetPrim());
    bindingAPI.Bind(matShade, UsdShadeTokens->weakerThanDescendants, TfToken("physics"));

    ContactReportData contactReportData;
    SubscriptionId id = physxSim->subscribePhysicsContactReportEvents(&ContactReportEvent, &contactReportData);

    physxSim->attachStage(stageId);

    SECTION("Contact Report Found/Lost")
    {
        contactReportData.clear();

        for (int u = 0; u < 100; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        CHECK(contactReportData.mContactHeaders.size() > 1);
        for (size_t i = 0; i < contactReportData.mContactHeaders.size(); i++)
        {
            const ContactEventHeader& header = contactReportData.mContactHeaders[i];
            CHECK(header.stageId == stageId);
            CHECK(intToPath(header.actor0) == concaveMeshPath);
            CHECK(intToPath(header.collider0) == concaveMeshPath);
            CHECK(intToPath(header.actor1) == planePath);
            CHECK(intToPath(header.collider1) == planePath);
            CHECK(header.protoIndex0 == 0xFFFFFFFF);
            CHECK(header.protoIndex1 == 0xFFFFFFFF);
            CHECK(header.contactDataOffset <= contactReportData.mContactData.size());
            CHECK(header.contactDataOffset + header.numContactData <= contactReportData.mContactData.size());
        }
    }

    SECTION("Contact Report First Found")
    {
        contactReportData.clear();

        for (int u = 0; u < 100; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();

            if (!contactReportData.mContactHeaders.empty())
                break;
        }

        CHECK(contactReportData.mContactHeaders.size() == 1);
        for (size_t i = 0; i < contactReportData.mContactHeaders.size(); i++)
        {
            const ContactEventHeader& header = contactReportData.mContactHeaders[i];
            CHECK(header.type == ContactEventType::eCONTACT_FOUND);
            CHECK(header.stageId == stageId);
            CHECK(intToPath(header.actor0) == concaveMeshPath);
            CHECK(intToPath(header.collider0) == concaveMeshPath);
            CHECK(intToPath(header.actor1) == planePath);
            CHECK(intToPath(header.collider1) == planePath);
            CHECK(header.protoIndex0 == 0xFFFFFFFF);
            CHECK(header.protoIndex1 == 0xFFFFFFFF);
            CHECK(header.contactDataOffset <= contactReportData.mContactData.size());
            CHECK(header.contactDataOffset + header.numContactData <= contactReportData.mContactData.size());
        }
    }

    SECTION("Contact Report Lost - Plane Delete")
    {
        contactReportData.clear();

        material.CreateRestitutionAttr().Set(0.0f);

        for (int u = 0; u < 50; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        CHECK(contactReportData.mContactHeaders.size() > 1);
        for (size_t i = 0; i < contactReportData.mContactHeaders.size(); i++)
        {
            const ContactEventHeader& header = contactReportData.mContactHeaders[i];
            CHECK(header.stageId == stageId);
            CHECK(intToPath(header.actor0) == concaveMeshPath);
            CHECK(intToPath(header.collider0) == concaveMeshPath);
            CHECK(intToPath(header.actor1) == planePath);
            CHECK(intToPath(header.collider1) == planePath);
            CHECK(header.protoIndex0 == 0xFFFFFFFF);
            CHECK(header.protoIndex1 == 0xFFFFFFFF);
            CHECK(header.contactDataOffset <= contactReportData.mContactData.size());
            CHECK(header.contactDataOffset + header.numContactData <= contactReportData.mContactData.size());
        }

        contactReportData.clear();

        stage->RemovePrim(planePath);

        physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();

        REQUIRE(contactReportData.mContactHeaders.size() == 1);
        const ContactEventHeader& header = contactReportData.mContactHeaders[0];
        CHECK(header.stageId == stageId);
        CHECK(intToPath(header.actor0) == concaveMeshPath);
        CHECK(intToPath(header.collider0) == concaveMeshPath);
        CHECK(intToPath(header.actor1) == planePath);
        CHECK(intToPath(header.collider1) == planePath);
        CHECK(header.type == ContactEventType::eCONTACT_LOST);
        CHECK(header.protoIndex0 == 0xFFFFFFFF);
        CHECK(header.protoIndex1 == 0xFFFFFFFF);
        CHECK(header.contactDataOffset == 0);
        CHECK(header.contactDataOffset == 0);
    }

    SECTION("Contact Report Lost - Box Delete")
    {
        contactReportData.clear();

        material.CreateRestitutionAttr().Set(0.0f);

        for (int u = 0; u < 50; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        CHECK(contactReportData.mContactHeaders.size() > 1);
        for (size_t i = 0; i < contactReportData.mContactHeaders.size(); i++)
        {
            const ContactEventHeader& header = contactReportData.mContactHeaders[i];
            CHECK(header.stageId == stageId);
            CHECK(intToPath(header.actor0) == concaveMeshPath);
            CHECK(intToPath(header.collider0) == concaveMeshPath);
            CHECK(intToPath(header.actor1) == planePath);
            CHECK(intToPath(header.collider1) == planePath);
            CHECK(header.protoIndex0 == 0xFFFFFFFF);
            CHECK(header.protoIndex1 == 0xFFFFFFFF);
            CHECK(header.contactDataOffset <= contactReportData.mContactData.size());
            CHECK(header.contactDataOffset + header.numContactData <= contactReportData.mContactData.size());
        }

        contactReportData.clear();

        stage->RemovePrim(concaveMeshPath);

        physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f);
        physxSim->fetchResults();

        REQUIRE(contactReportData.mContactHeaders.size() == 1);
        const ContactEventHeader& header = contactReportData.mContactHeaders[0];
        CHECK(header.stageId == stageId);
        CHECK(intToPath(header.actor0) == concaveMeshPath);
        CHECK(intToPath(header.collider0) == concaveMeshPath);
        CHECK(intToPath(header.actor1) == planePath);
        CHECK(intToPath(header.collider1) == planePath);
        CHECK(header.type == ContactEventType::eCONTACT_LOST);
        CHECK(header.protoIndex0 == 0xFFFFFFFF);
        CHECK(header.protoIndex1 == 0xFFFFFFFF);
        CHECK(header.contactDataOffset == 0);
        CHECK(header.contactDataOffset == 0);
    }

    physxSim->unsubscribePhysicsContactReportEvents(id);

    FullContactReportData fullContactReportData;
    id = physxSim->subscribePhysicsFullContactReportEvents(&FullContactReportEvent, &fullContactReportData);

    SECTION("Contact Report - Friction Data")
    {
        fullContactReportData.clear();

        for (int u = 0; u < 100; u++)
        {
            physxSim->simulate(1.0f / 60.0f, 1.0f / 60.0f * u);
            physxSim->fetchResults();
        }

        CHECK(fullContactReportData.mContactHeaders.size() > 1);
        for (size_t i = 0; i < fullContactReportData.mContactHeaders.size(); i++)
        {
            const ContactEventHeader& header = fullContactReportData.mContactHeaders[i];
            CHECK(header.stageId == stageId);
            CHECK(intToPath(header.actor0) == concaveMeshPath);
            CHECK(intToPath(header.collider0) == concaveMeshPath);
            CHECK(intToPath(header.actor1) == planePath);
            CHECK(intToPath(header.collider1) == planePath);
            CHECK(header.protoIndex0 == 0xFFFFFFFF);
            CHECK(header.protoIndex1 == 0xFFFFFFFF);
            CHECK(header.contactDataOffset <= fullContactReportData.mContactData.size());
            CHECK(header.contactDataOffset + header.numContactData <= fullContactReportData.mContactData.size());
            CHECK(header.frictionAnchorsDataOffset <= fullContactReportData.mFrictionAnchorsData.size());
            CHECK(header.frictionAnchorsDataOffset + header.numfrictionAnchorsData <= fullContactReportData.mFrictionAnchorsData.size());
        }
    }

    physxSim->unsubscribePhysicsFullContactReportEvents(id);

    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
