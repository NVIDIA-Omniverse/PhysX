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

static constexpr GfVec3f spatialTendonLocalPos(0.f, 0.f, 2.f);
static constexpr float spatialTendonStiffness = -5.f;
static constexpr float tendonRootGearing = 3.f;
static constexpr float tendonRootForceCoef = 3.5f;
static constexpr float tendonGearing = 2.f;
static constexpr float tendonForceCoef = 2.5f;
static constexpr float spatialTendonDamping = -2.f;
static constexpr float spatialTendonLimitStiffness = 1.f;
static constexpr float tendonRestLength = -1.f;

// Helper methods
static pxr::PhysxSchemaPhysxTendonAttachmentRootAPI setupSpatialRootAtPath(const UsdStageWeakPtr stage,
                                                                           const SdfPath xformPath,
                                                                           const TfToken instanceName)
{
    pxr::UsdPrim rootPrim = stage->GetPrimAtPath(xformPath);
    pxr::PhysxSchemaPhysxTendonAttachmentRootAPI rootApi =
        pxr::PhysxSchemaPhysxTendonAttachmentRootAPI::Apply(rootPrim, instanceName);

    PXR_NS::PhysxSchemaPhysxTendonAttachmentAPI(rootApi, rootApi.GetName()).CreateGearingAttr().Set(tendonRootGearing);
    rootApi.CreateStiffnessAttr().Set(spatialTendonStiffness);
    rootApi.CreateDampingAttr().Set(spatialTendonDamping);
    rootApi.CreateLimitStiffnessAttr().Set(spatialTendonLimitStiffness);
    PXR_NS::PhysxSchemaPhysxTendonAttachmentAPI(rootApi, rootApi.GetName()).CreateLocalPosAttr().Set(spatialTendonLocalPos);

    return rootApi;
}

static pxr::PhysxSchemaPhysxTendonAttachmentAPI setupSpatialAttachmentAtPath(const UsdStageWeakPtr stage,
                                                                             const SdfPath xformPath,
                                                                             const TfToken instanceName,
                                                                             const SdfPath parentPath,
                                                                             const TfToken parentInstance)
{
    pxr::UsdPrim attachPrim = stage->GetPrimAtPath(xformPath);
    pxr::PhysxSchemaPhysxTendonAttachmentAPI attachApi =
        pxr::PhysxSchemaPhysxTendonAttachmentAPI::Apply(attachPrim, instanceName);

    attachApi.CreateGearingAttr().Set(tendonGearing);
    attachApi.CreateParentAttachmentAttr().Set(parentInstance);
    attachApi.CreateParentLinkRel().AddTarget(parentPath);
    attachApi.CreateLocalPosAttr().Set(spatialTendonLocalPos);

    return attachApi;
}

static pxr::PhysxSchemaPhysxTendonAttachmentLeafAPI setupSpatialLeafAtPath(const UsdStageWeakPtr stage,
                                                                           const SdfPath xformPath,
                                                                           const TfToken instanceName,
                                                                           const SdfPath parentPath,
                                                                           const TfToken parentInstance)
{
    pxr::UsdPrim leafPrim = stage->GetPrimAtPath(xformPath);
    pxr::PhysxSchemaPhysxTendonAttachmentLeafAPI leafApi =
        pxr::PhysxSchemaPhysxTendonAttachmentLeafAPI::Apply(leafPrim, instanceName);

    leafApi.CreateRestLengthAttr().Set(tendonRestLength);
    PXR_NS::PhysxSchemaPhysxTendonAttachmentAPI(leafApi, leafApi.GetName()).CreateParentAttachmentAttr().Set(parentInstance);
    PXR_NS::PhysxSchemaPhysxTendonAttachmentAPI(leafApi, leafApi.GetName()).CreateParentLinkRel().AddTarget(parentPath);
    PXR_NS::PhysxSchemaPhysxTendonAttachmentAPI(leafApi, leafApi.GetName()).CreateLocalPosAttr().Set(spatialTendonLocalPos);

    return leafApi;
}

static pxr::PhysxSchemaPhysxTendonAxisRootAPI setupFixedTendonRoot(const UsdStageWeakPtr stage,
                                                                   const SdfPath jointPath,
                                                                   const TfToken tendonName)
{
    pxr::UsdPrim rootPrim = stage->GetPrimAtPath(jointPath);
    pxr::PhysxSchemaPhysxTendonAxisRootAPI rootApi = pxr::PhysxSchemaPhysxTendonAxisRootAPI::Apply(rootPrim, tendonName);

    VtFloatArray temp(1);
    temp[0] = tendonRootGearing;
    PXR_NS::PhysxSchemaPhysxTendonAttachmentAPI(rootApi, rootApi.GetName()).CreateGearingAttr().Set(temp);
    temp[0] = tendonRootForceCoef;
    PhysxSchemaPhysxTendonAxisAPI(rootApi, rootApi.GetName()).CreateForceCoefficientAttr().Set(temp);
    rootApi.CreateStiffnessAttr().Set(HUGE_VALF); // set to infinity for testing
    rootApi.CreateRestLengthAttr().Set(tendonRestLength);

    return rootApi;
}

static pxr::PhysxSchemaPhysxTendonAxisAPI setupFixedTendonAxis(const UsdStageWeakPtr stage,
                                                               const SdfPath jointPath,
                                                               const TfToken tendonName)
{
    pxr::UsdPrim attachPrim = stage->GetPrimAtPath(jointPath);
    pxr::PhysxSchemaPhysxTendonAxisAPI axisApi = pxr::PhysxSchemaPhysxTendonAxisAPI::Apply(attachPrim, tendonName);

    VtFloatArray temp(1);
    temp[0] = tendonGearing;
    axisApi.CreateGearingAttr().Set(temp);
    temp[0] = tendonForceCoef;
    axisApi.CreateForceCoefficientAttr().Set(temp);

    return axisApi;
}

static void setupFixedJointToWorld(const UsdStageWeakPtr stage,
                                   const SdfPath jointAbsPath,
                                   const SdfPath rootLinkAbsPath,
                                   const bool setRootOnBody0Rel)
{
    UsdPhysicsFixedJoint joint = UsdPhysicsFixedJoint::Define(stage, jointAbsPath);
    CHECK(joint);

    if (setRootOnBody0Rel)
    {
        joint.CreateBody0Rel().AddTarget(rootLinkAbsPath);
    }
    else
    {
        joint.CreateBody1Rel().AddTarget(rootLinkAbsPath);
    }
}

static void setupRevoluteJoint(const UsdStageWeakPtr stage,
                               const SdfPath jointAbsPath,
                               const SdfPath parentLinkAbsPath,
                               const SdfPath childLinkAbsPath)
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
}

static void setupRevoluteBranchJoint(const UsdStageWeakPtr stage,
                                     const SdfPath jointAbsPath,
                                     const SdfPath parentLinkAbsPath,
                                     const SdfPath childLinkAbsPath)
{
    UsdPhysicsRevoluteJoint joint = UsdPhysicsRevoluteJoint::Define(stage, jointAbsPath);
    CHECK(joint);
    joint.CreateBody0Rel().SetTargets(SdfPathVector({ parentLinkAbsPath }));
    joint.CreateBody1Rel().SetTargets(SdfPathVector({ childLinkAbsPath }));
    joint.CreateAxisAttr().Set(TfToken("X"));
    joint.CreateLocalPos0Attr().Set(GfVec3f(0.f, 0.5f, 0.f));
    joint.CreateLocalPos1Attr().Set(GfVec3f(0.f, -0.5f, 0.f));
    joint.CreateLocalRot0Attr().Set(GfQuatf(1.0f));
    joint.CreateLocalRot1Attr().Set(GfQuatf(1.0f));
}

//-----------------------------------------------------------------------------
// Init test
TEST_CASE("Tendons Tests",
          "[omniphysics]"
          "[component=OmniPhysics][owner=preist][priority=mandatory]")
{
    // constants for test checks
    const float radToDeg = 180.0f / PxPi;

    // constants for setup
    const GfVec3f gravityDirection(0.0f, 0.0f, -1.0f); // use z-up
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const float kilogramsPerStageUnit = 1.0f;
    const float gravityMagnitude = 10.0f / metersPerStageUnit;
    const float density = 1000.f * metersPerStageUnit * metersPerStageUnit * metersPerStageUnit / kilogramsPerStageUnit;
    const float linkLength = 10.0f; // cm
    const float linkWidth = 2.0f; // cm
    const GfVec3f linkDims(linkLength, 2.f * linkWidth, linkWidth);

    // setup common to all subcases
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    pxr::UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);
    scene.GetGravityMagnitudeAttr().Set(gravityMagnitude);
    scene.GetGravityDirectionAttr().Set(gravityDirection);

    const GfVec3f rootColor(0.7f, 0.1f, 0.1f);
    const GfVec3f linkColor(0.1f, 0.1f, 0.7f);
    const GfVec3f branchColor(0.1f, 0.7f, 0.1f);

    // setup linear articulation rigid bodies, extending in +X
    auto SetupRBLinksAtPaths = [&](const std::vector<SdfPath>& absolutePaths, const GfVec3f& offset = GfVec3f(0.0f)) {
        GfVec3f linkPos = offset;
        for (size_t i = 0; i < absolutePaths.size(); ++i)
        {
            addRigidBox(stage, absolutePaths[i].GetString(), linkDims, linkPos, GfQuatf(1.0f),
                        i ? linkColor : rootColor, density);
            CHECK(stage->GetPrimAtPath(absolutePaths[i]));
            linkPos[0] += linkLength;
        }
    };

    // setup linear articulation rigid bodies from a root body, extending in +Y
    auto SetupRBBranchLinksAtPaths = [&](const std::vector<SdfPath>& branchPaths, const SdfPath& branchRootPath) {
        GfVec3f rootPos = getPhysicsPrimPos(stage, branchRootPath);
        for (size_t i = 1; i < branchPaths.size(); ++i)
        {
            rootPos[1] += linkDims[1];
            addRigidBox(stage, branchPaths[i].GetString(), linkDims, rootPos, GfQuatf(1.0), branchColor, density);
            CHECK(stage->GetPrimAtPath(branchPaths[i]));
        }
    };

    const GfVec3f scale(1.f, 2.f, 3.f);

    // setup root Xform
    const SdfPath xformPrimPath = defaultPrimPath.AppendChild(TfToken("Xform"));
    pxr::UsdGeomXform xform = pxr::UsdGeomXform::Define(stage, xformPrimPath);
    xform.AddRotateYOp().Set(90.f);
    xform.AddScaleOp().Set(scale);

    // setup just the main chain of links:
    const SdfPath rootLinkPath = xformPrimPath.AppendChild(TfToken("root"));
    std::vector<SdfPath> mainLinks;
    mainLinks.push_back(rootLinkPath);
    for (size_t i = 1; i < 5; ++i)
    {
        std::string linkPath = "body" + std::to_string(i);
        mainLinks.push_back(xformPrimPath.AppendChild(TfToken(linkPath)));
    }
    SetupRBLinksAtPaths(mainLinks);

    // apply articulation root
    const SdfPath fixedJointPath = xformPrimPath.AppendChild(TfToken("fixedJoint"));
    setupFixedJointToWorld(stage, fixedJointPath, rootLinkPath, true);
    UsdPhysicsArticulationRootAPI::Apply(stage->GetPrimAtPath(fixedJointPath));

    std::vector<SdfPath> mainJoints;
    // setup joints on main chain
    for (size_t i = 1; i < 5; ++i)
    {
        std::string jointString = "revJoint" + std::to_string(i - 1) + std::to_string(i);
        pxr::SdfPath jointPath = xformPrimPath.AppendChild(TfToken(jointString));
        setupRevoluteJoint(stage, jointPath, mainLinks[i - 1], mainLinks[i]);
        mainJoints.push_back(jointPath);
    }

    SUBCASE("Spatial Tendon LocalPos Scaling")
    {
        // setup one simple spatial tendon
        pxr::PhysxSchemaPhysxTendonAttachmentRootAPI rootApi =
            setupSpatialRootAtPath(stage, mainLinks.at(0), TfToken("root"));
        pxr::PhysxSchemaPhysxTendonAttachmentLeafAPI leafApi =
            setupSpatialLeafAtPath(stage, mainLinks.at(1), TfToken("leaf"), mainLinks.at(0), TfToken("root"));

        // setup localpos on root such that PhysX position is at 1, 1, 1
        GfVec3f linkDimsScaled = GfCompMult(linkDims, scale);
        const GfVec3f localPos = GfVec3f(1.0f / linkDimsScaled[0], 1.0f / linkDimsScaled[1], 1.0f / linkDimsScaled[2]);
        PXR_NS::PhysxSchemaPhysxTendonAttachmentAPI(rootApi, rootApi.GetName()).CreateLocalPosAttr().Set(localPos);

        // parse
        physxSim->attachStage(stageId);

        //stage->Export("generatedArt.usda");

        // check parsing
        PxArticulationReducedCoordinate* pxArticulation =
            getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(
                fixedJointPath, PhysXType::ePTArticulation);
        PxArticulationSpatialTendon* tendon = nullptr;
        pxArticulation->getSpatialTendons(&tendon, 1u, 0u);
        REQUIRE(tendon);

        PxArticulationAttachment* rootAttachment = nullptr;
        tendon->getAttachments(&rootAttachment, 1u, 0u);

        const PxVec3 localPosParsed = rootAttachment->getRelativeOffset();
        CHECK(localPosParsed.x == doctest::Approx(1.0f).epsilon(0.001));
        CHECK(localPosParsed.y == doctest::Approx(1.0f).epsilon(0.001));
        CHECK(localPosParsed.z == doctest::Approx(1.0f).epsilon(0.001));

        // simulate to put in scene:
        physxSim->simulate(0.001f, 0.0f);
        physxSim->fetchResults();

        // check localPos attr update
        PXR_NS::PhysxSchemaPhysxTendonAttachmentAPI(rootApi, rootApi.GetName()).CreateLocalPosAttr().Set(2.0f * localPos);
        physxSim->simulate(0.001f, 0.0f);
        physxSim->fetchResults();

        const PxVec3 localPosUpdated = rootAttachment->getRelativeOffset();
        CHECK(localPosUpdated.x == doctest::Approx(2.0f).epsilon(0.001));
        CHECK(localPosUpdated.y == doctest::Approx(2.0f).epsilon(0.001));
        CHECK(localPosUpdated.z == doctest::Approx(2.0f).epsilon(0.001));
    }

    SUBCASE("Spatial Tendon Topology")
    {
        // setup one spatial tendon
        pxr::PhysxSchemaPhysxTendonAttachmentRootAPI rootApi =
            setupSpatialRootAtPath(stage, mainLinks.at(4), TfToken("root"));
        setupSpatialAttachmentAtPath(stage, mainLinks.at(2), TfToken("a1"), mainLinks.at(4), TfToken("root"));
        setupSpatialLeafAtPath(stage, mainLinks.at(0), TfToken("leaf"), mainLinks.at(2), TfToken("a1"));

        // customize values a little
        rootApi.CreateOffsetAttr().Set(1.f);
        PXR_NS::PhysxSchemaPhysxTendonAttachmentAPI(rootApi, rootApi.GetName()).GetGearingAttr().Set(42.f); // should be ignored in parsing and set to 1.f

        // parse
        physxSim->attachStage(stageId);

        // stage->Export("generatedArt.usda");

        // sanity check
        PxArticulationReducedCoordinate* pxArticulation =
            getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(
                fixedJointPath, PhysXType::ePTArticulation);
        const bool isFixedBase = pxArticulation->getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
        CHECK(isFixedBase);

        // start simulating to get up-to-date PhysX values
        physxSim->simulate(0.001f, 0.0f);
        physxSim->fetchResults();

        PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
        CHECK_EQ(1u, pxScene->getNbArticulations());
        CHECK_EQ(1u, pxArticulation->getNbSpatialTendons());

        // check if parsing worked correctly
        PxArticulationSpatialTendon* tendon = nullptr;
        pxArticulation->getSpatialTendons(&tendon, 1u, 0u);
        REQUIRE(tendon);
        CHECK_EQ(0.f, tendon->getStiffness()); // negative value should be corrected to 0
        CHECK_EQ(3u, tendon->getNbAttachments());
        CHECK_EQ(0.f, tendon->getDamping()); // negative value should be corrected to 0
        CHECK_EQ(spatialTendonLimitStiffness, tendon->getLimitStiffness());

        PxArticulationAttachment* attachment = nullptr;
        tendon->getAttachments(&attachment, 1u, 0u);
        REQUIRE(attachment);
        CHECK_EQ(1.f, attachment->getCoefficient()); // test if parsing ignored 42.f correctly

        // check whether automatic rest length computation yields expected results
        tendon->getAttachments(&attachment, 1u, 2u);
        REQUIRE(attachment);
        CHECK_UNARY(attachment->isLeaf());
        // 6.f because of gearing at intermediate attachment
        CHECK(attachment->getRestLength() == doctest::Approx(scale[0] * linkDims[0] * 6.f));
        const PxVec3 localPos = attachment->getRelativeOffset();
        CHECK(localPos[2] == doctest::Approx(spatialTendonLocalPos[2] * scale[2] * linkDims[2]));
    }

    SUBCASE("Spatial Tendon USD read")
    {
        // setup one spatial tendon
        pxr::PhysxSchemaPhysxTendonAttachmentRootAPI rootApi =
            setupSpatialRootAtPath(stage, mainLinks.at(4), TfToken("root"));
        pxr::PhysxSchemaPhysxTendonAttachmentAPI attachApi =
            setupSpatialAttachmentAtPath(stage, mainLinks.at(2), TfToken("a1"), mainLinks.at(4), TfToken("root"));
        pxr::PhysxSchemaPhysxTendonAttachmentLeafAPI leafApi =
            setupSpatialLeafAtPath(stage, mainLinks.at(0), TfToken("leaf"), mainLinks.at(2), TfToken("a1"));

        // customize values
        PXR_NS::PhysxSchemaPhysxTendonAttachmentAPI(rootApi, rootApi.GetName()).CreateGearingAttr().Set(42.f); // should be ignored in parsing and set to 1.f
        rootApi.CreateDampingAttr().Set(2.f);
        rootApi.CreateLimitStiffnessAttr().Set(3.f);
        const GfVec3f rootVec(4.f, 5.f, 6.f);
        PXR_NS::PhysxSchemaPhysxTendonAttachmentAPI(rootApi, rootApi.GetName()).CreateLocalPosAttr().Set(rootVec);
        rootApi.CreateOffsetAttr().Set(7.f);
        rootApi.CreateStiffnessAttr().Set(8.f);

        attachApi.CreateGearingAttr().Set(9.f);
        const GfVec3f attachVec(10.f, 11.f, 12.f);
        attachApi.CreateLocalPosAttr().Set(attachVec);

        PXR_NS::PhysxSchemaPhysxTendonAttachmentAPI(leafApi, leafApi.GetName()).CreateGearingAttr().Set(13.f);
        const GfVec3f leafVec(14.f, 15.f, 16.f);
        PXR_NS::PhysxSchemaPhysxTendonAttachmentAPI(leafApi, leafApi.GetName()).CreateLocalPosAttr().Set(leafVec);
        leafApi.CreateLowerLimitAttr().Set(17.f);
        leafApi.CreateUpperLimitAttr().Set(18.f);
        leafApi.CreateRestLengthAttr().Set(19.f);

        // parse
        physxSim->attachStage(stageId);

        // stage->Export("generatedArt.usda");

        // sanity check
        PxArticulationReducedCoordinate* pxArticulation =
            getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(
                fixedJointPath, PhysXType::ePTArticulation);
        const bool isFixedBase = pxArticulation->getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
        CHECK(isFixedBase);

        // start simulating to get up-to-date PhysX values
        physxSim->simulate(0.001f, 0.0f);
        physxSim->fetchResults();

        PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
        CHECK_EQ(1u, pxScene->getNbArticulations());
        CHECK_EQ(1u, pxArticulation->getNbSpatialTendons());

        // get Physx pointers
        PxArticulationSpatialTendon* tendon = nullptr;
        pxArticulation->getSpatialTendons(&tendon, 1u, 0u);
        REQUIRE(tendon);

        PxArticulationAttachment* attachments[3];
        tendon->getAttachments(attachments, 3u, 0u);
        REQUIRE(attachments[0]);
        REQUIRE(attachments[1]);
        REQUIRE(attachments[2]);
        REQUIRE(attachments[2]->isLeaf());

        // test parameters
        CHECK_EQ(1.f, attachments[0]->getCoefficient());
        CHECK_EQ(2.f, tendon->getDamping());
        CHECK_EQ(3.f, tendon->getLimitStiffness());
        CHECK_EQ(4.f * scale[0] * linkDims[0], attachments[0]->getRelativeOffset().x);
        CHECK_EQ(5.f * scale[1] * linkDims[1], attachments[0]->getRelativeOffset().y);
        CHECK_EQ(6.f * scale[2] * linkDims[2], attachments[0]->getRelativeOffset().z);
        CHECK_EQ(7.f, tendon->getOffset());
        CHECK_EQ(8.f, tendon->getStiffness());

        CHECK_EQ(9.f, attachments[1]->getCoefficient());
        CHECK_EQ(10.f * scale[0] * linkDims[0], attachments[1]->getRelativeOffset().x);
        CHECK_EQ(11.f * scale[1] * linkDims[1], attachments[1]->getRelativeOffset().y);
        CHECK_EQ(12.f * scale[2] * linkDims[2], attachments[1]->getRelativeOffset().z);

        CHECK_EQ(13.f, attachments[2]->getCoefficient());
        CHECK_EQ(14.f * scale[0] * linkDims[0], attachments[2]->getRelativeOffset().x);
        CHECK_EQ(15.f * scale[1] * linkDims[1], attachments[2]->getRelativeOffset().y);
        CHECK_EQ(16.f * scale[2] * linkDims[2], attachments[2]->getRelativeOffset().z);
        CHECK_EQ(17.f, attachments[2]->getLimitParameters().lowLimit);
        CHECK_EQ(18.f, attachments[2]->getLimitParameters().highLimit);
        CHECK_EQ(19.f, attachments[2]->getRestLength());
    }

    SUBCASE("Spatial Tendon USD read specials")
    {
        // setup one spatial tendon
        pxr::PhysxSchemaPhysxTendonAttachmentRootAPI rootApi =
            setupSpatialRootAtPath(stage, mainLinks.at(4), TfToken("root"));
        pxr::PhysxSchemaPhysxTendonAttachmentLeafAPI leafApi =
            setupSpatialLeafAtPath(stage, mainLinks.at(0), TfToken("leaf"), mainLinks.at(4), TfToken("root"));

        // customize values
        PXR_NS::PhysxSchemaPhysxTendonAttachmentAPI(rootApi, rootApi.GetName()).CreateGearingAttr().Set(42.f); // should be ignored in parsing and set to 1.f
        rootApi.CreateDampingAttr().Set(-2.f); // should be corrected to 0
        rootApi.CreateLimitStiffnessAttr().Set(-3.f); // should be corrected to 0
        rootApi.CreateStiffnessAttr().Set(-8.f); // should be corrected to 0

        leafApi.CreateLowerLimitAttr().Set(20.f);
        leafApi.CreateUpperLimitAttr().Set(0.f); // should be corrected to 20

        // parse
        physxSim->attachStage(stageId);

        // stage->Export("generatedArt.usda");

        // sanity check
        PxArticulationReducedCoordinate* pxArticulation =
            getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(
                fixedJointPath, PhysXType::ePTArticulation);
        const bool isFixedBase = pxArticulation->getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
        CHECK(isFixedBase);

        // start simulating to get up-to-date PhysX values
        physxSim->simulate(0.001f, 0.0f);
        physxSim->fetchResults();

        PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
        CHECK_EQ(1u, pxScene->getNbArticulations());
        CHECK_EQ(1u, pxArticulation->getNbSpatialTendons());

        // get Physx pointers
        PxArticulationSpatialTendon* tendon = nullptr;
        pxArticulation->getSpatialTendons(&tendon, 1u, 0u);
        REQUIRE(tendon);

        PxArticulationAttachment* attachments[2];
        tendon->getAttachments(attachments, 2u, 0u);
        REQUIRE(attachments[0]);
        REQUIRE(attachments[1]);
        REQUIRE(attachments[1]->isLeaf());

        // test parameters
        CHECK_EQ(1.f, attachments[0]->getCoefficient());
        CHECK_EQ(0.f, tendon->getDamping());
        CHECK_EQ(0.f, tendon->getLimitStiffness());
        CHECK_EQ(0.f, tendon->getStiffness());

        CHECK_EQ(20.f, attachments[1]->getLimitParameters().lowLimit);
        CHECK_EQ(20.f, attachments[1]->getLimitParameters().highLimit);
    }

    SUBCASE("Fixed Tendon Invalid Topology")
    {
        // setup invalid fixed-tendon topology
        // root must be at articulation-topology-wise common joint ancestor
        // so this usd setup must fail at creating the tendon axis at mainJoints.at(1)
        // and, therefore, the total joint will be only 3 instead of 4
        setupFixedTendonAxis(stage, mainJoints.at(1), TfToken("t1"));
        setupFixedTendonRoot(stage, mainJoints.at(2), TfToken("t1"));
        setupFixedTendonAxis(stage, mainJoints.at(3), TfToken("t1"));

        // parse and put in scene:
        physxSim->attachStage(stageId);
        physx->forceLoadPhysicsFromUSD();

        //stage->Export("fixedTendonTopology.usda");

        // check if parsing worked correctly
        PxArticulationReducedCoordinate* pxArticulation =
            getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(
                fixedJointPath, PhysXType::ePTArticulation);
        const bool isFixedBase = pxArticulation->getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
        CHECK(isFixedBase);

        PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
        CHECK_EQ(1u, pxScene->getNbArticulations());
        CHECK_EQ(1u, pxArticulation->getNbFixedTendons());

        PxArticulationFixedTendon* tendon = nullptr;
        pxArticulation->getFixedTendons(&tendon, 1u, 0u);
        REQUIRE(tendon);
        // this check also ensures we found the correct topological root while parsing
        CHECK_EQ(3u, tendon->getNbTendonJoints()); // should be four because of dummy joint creation
    }

    SUBCASE("Fixed Tendon Valid Topology")
    {
        // setup fixed tendon
        setupFixedTendonRoot(stage, mainJoints.at(1), TfToken("t1")); // must be at articulation-topological root
        setupFixedTendonAxis(stage, mainJoints.at(2), TfToken("t1"));
        setupFixedTendonAxis(stage, mainJoints.at(3), TfToken("t1"));

        // parse and put in scene:
        physxSim->attachStage(stageId);
        physx->forceLoadPhysicsFromUSD();

        //stage->Export("fixedTendonTopology.usda");

        // check if parsing worked correctly
        PxArticulationReducedCoordinate* pxArticulation =
            getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(
                fixedJointPath, PhysXType::ePTArticulation);
        const bool isFixedBase = pxArticulation->getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
        CHECK(isFixedBase);
        CHECK_EQ(1u, pxArticulation->getNbFixedTendons());

        PxArticulationFixedTendon* tendon = nullptr;
        pxArticulation->getFixedTendons(&tendon, 1u, 0u);
        REQUIRE(tendon);
        CHECK_EQ(FLT_MAX, tendon->getStiffness());
        // this check also ensures we found the correct topological root while parsing
        CHECK_EQ(4u, tendon->getNbTendonJoints()); // should be four because of dummy joint creation
        CHECK_EQ(0.f, tendon->getDamping());
        CHECK_EQ(-1.f, tendon->getRestLength());

        PxArticulationTendonJoint* tJoints[4];
        float coeff = 0.f;
        float recipCoeff = 0.0f;
        PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
        tendon->getTendonJoints(tJoints, 4u, 0u);

        REQUIRE(tJoints[0]);
        tJoints[0]->getCoefficient(axis, coeff, recipCoeff);
        CHECK_EQ(0.f, coeff); // dummy joint should have 0 coefficient
        CHECK_EQ(0.f, recipCoeff); // dummy joint should have 0 recipCoeff

        REQUIRE(tJoints[1]);
        tJoints[1]->getCoefficient(axis, coeff, recipCoeff);
        CHECK_EQ(doctest::Approx(tendonRootGearing * radToDeg), coeff);
        CHECK_EQ(doctest::Approx(tendonRootForceCoef), recipCoeff);

        REQUIRE(tJoints[2]);
        tJoints[2]->getCoefficient(axis, coeff, recipCoeff);
        CHECK_EQ(doctest::Approx(tendonGearing * radToDeg), coeff);
        CHECK_EQ(doctest::Approx(tendonForceCoef), recipCoeff);

        REQUIRE(tJoints[3]);
        tJoints[3]->getCoefficient(axis, coeff, recipCoeff);
        CHECK_EQ(doctest::Approx(tendonGearing * radToDeg), coeff);
        CHECK_EQ(doctest::Approx(tendonForceCoef), recipCoeff);
    }

    SUBCASE("Fixed Tendon USD read")
    {
        // setup fixed tendon
        pxr::PhysxSchemaPhysxTendonAxisRootAPI rootApi = setupFixedTendonRoot(stage, mainJoints.at(2), TfToken("t1"));

        rootApi.CreateDampingAttr().Set(1.f);
        VtFloatArray temp(1);
        temp[0] = 2.f;
        PXR_NS::PhysxSchemaPhysxTendonAttachmentAPI(rootApi, rootApi.GetName()).CreateGearingAttr().Set(temp);
        temp[0] = 3.0f;
        PhysxSchemaPhysxTendonAxisAPI(rootApi, rootApi.GetName()).CreateForceCoefficientAttr().Set(temp);
        rootApi.CreateLimitStiffnessAttr().Set(3.f);
        rootApi.CreateLowerLimitAttr().Set(4.f);
        rootApi.CreateUpperLimitAttr().Set(5.f);
        rootApi.CreateOffsetAttr().Set(6.f);
        rootApi.CreateRestLengthAttr().Set(7.f);
        rootApi.CreateStiffnessAttr().Set(8.f);

        // parse
        physxSim->attachStage(stageId);

        // stage->Export("generatedArt.usda");

        // sanity check
        PxArticulationReducedCoordinate* pxArticulation =
            getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(
                fixedJointPath, PhysXType::ePTArticulation);
        const bool isFixedBase = pxArticulation->getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
        CHECK(isFixedBase);

        // start simulating to get up-to-date PhysX values
        physxSim->simulate(0.001f, 0.0f);
        physxSim->fetchResults();

        // check if parsing worked correctly
        PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
        CHECK_EQ(1u, pxScene->getNbArticulations());
        CHECK_EQ(1u, pxArticulation->getNbFixedTendons());

        PxArticulationFixedTendon* tendon = nullptr;
        pxArticulation->getFixedTendons(&tendon, 1u, 0u);
        REQUIRE(tendon);
        REQUIRE(2u == tendon->getNbTendonJoints());

        PxArticulationTendonJoint* tJoint;
        PxReal coeff = 0.f;
        PxReal recipCoeff = 0.f;
        PxArticulationAxis::Enum axis = PxArticulationAxis::eTWIST;
        tendon->getTendonJoints(&tJoint, 1u, 1u);
        REQUIRE(tJoint);

        CHECK_EQ(1.f, tendon->getDamping());
        tJoint->getCoefficient(axis, coeff, recipCoeff);
        CHECK_EQ(doctest::Approx(2.f * radToDeg), coeff);
        CHECK_EQ(doctest::Approx(3.0f), recipCoeff);
        CHECK_EQ(3.f, tendon->getLimitStiffness());
        CHECK_EQ(4.f, tendon->getLimitParameters().lowLimit);
        CHECK_EQ(5.f, tendon->getLimitParameters().highLimit);
        CHECK_EQ(6.f, tendon->getOffset());
        CHECK_EQ(7.f, tendon->getRestLength());
        CHECK_EQ(8.f, tendon->getStiffness());
    }

    SUBCASE("Fixed Tendon USD read specials")
    {
        // setup fixed tendon
        pxr::PhysxSchemaPhysxTendonAxisRootAPI rootApi = setupFixedTendonRoot(stage, mainJoints.at(2), TfToken("t1"));

        rootApi.CreateDampingAttr().Set(-1.f); // should be corrected to 0
        rootApi.CreateLimitStiffnessAttr().Set(-3.f); // should be corected to 0
        rootApi.CreateLowerLimitAttr().Set(HUGE_VALF); // should be corrected to FLT_MAX
        rootApi.CreateUpperLimitAttr().Set(-HUGE_VALF); // should be corrected to FLT_MAX
        rootApi.CreateStiffnessAttr().Set(-8.f); // should be corrected to 0

        // parse
        physxSim->attachStage(stageId);

        // stage->Export("generatedArt.usda");

        // sanity check
        PxArticulationReducedCoordinate* pxArticulation =
            getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(
                fixedJointPath, PhysXType::ePTArticulation);
        const bool isFixedBase = pxArticulation->getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
        CHECK(isFixedBase);

        // start simulating to get up-to-date PhysX values
        physxSim->simulate(0.001f, 0.0f);
        physxSim->fetchResults();

        // check if parsing worked correctly
        PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
        CHECK_EQ(1u, pxScene->getNbArticulations());
        CHECK_EQ(1u, pxArticulation->getNbFixedTendons());

        PxArticulationFixedTendon* tendon = nullptr;
        pxArticulation->getFixedTendons(&tendon, 1u, 0u);
        REQUIRE(tendon);
        REQUIRE(2u == tendon->getNbTendonJoints());

        CHECK_EQ(0.f, tendon->getDamping());
        CHECK_EQ(0.f, tendon->getLimitStiffness());
        CHECK_EQ(FLT_MAX, tendon->getLimitParameters().lowLimit);
        CHECK_EQ(FLT_MAX, tendon->getLimitParameters().highLimit);
        CHECK_EQ(0.f, tendon->getStiffness());
    }

    // use a slightly more complicated setup to test correct branching
    SUBCASE("Branching")
    {
        // setup branch at the middle of main chain
        // articulation now has this shape:
        // | (root)
        // |
        // | - - - - (4 branch links)
        // |
        // |
        std::vector<SdfPath> branchLinks;
        branchLinks.push_back(mainLinks[2]);
        for (size_t i = 1; i < 5; ++i)
        {
            std::string linkPath = "branch" + std::to_string(i);
            branchLinks.push_back(xformPrimPath.AppendChild(TfToken(linkPath)));
        }
        SetupRBBranchLinksAtPaths(branchLinks, mainLinks[2]);

        std::vector<SdfPath> branchJoints;
        // setup joints on branch
        for (size_t i = 1; i < 5; ++i)
        {
            std::string jointString = "branchJoint" + std::to_string(i - 1) + std::to_string(i);
            pxr::SdfPath jointPath = xformPrimPath.AppendChild(TfToken(jointString));
            setupRevoluteBranchJoint(stage, jointPath, branchLinks[i - 1], branchLinks[i]);
            branchJoints.push_back(jointPath);
        }

        SUBCASE("Spatial Tendon")
        {
            // setup first tendon branch
            setupSpatialRootAtPath(stage, mainLinks.at(4), TfToken("root1"));
            setupSpatialAttachmentAtPath(stage, mainLinks.at(2), TfToken("a1"), mainLinks.at(4), TfToken("root1"));
            setupSpatialLeafAtPath(stage, mainLinks.at(0), TfToken("leaf1_1"), mainLinks.at(2), TfToken("a1"));

            // setup 2nd tendon branch
            setupSpatialAttachmentAtPath(stage, branchLinks.at(1), TfToken("b1"), mainLinks.at(2), TfToken("a1"));
            pxr::PhysxSchemaPhysxTendonAttachmentAPI attachApi =
                setupSpatialAttachmentAtPath(stage, mainLinks.at(2), TfToken("b2"), branchLinks.at(1), TfToken("b1"));
            setupSpatialAttachmentAtPath(stage, branchLinks.at(4), TfToken("b3"), mainLinks.at(2), TfToken("b2"));
            const pxr::PhysxSchemaPhysxTendonAttachmentLeafAPI leafApi =
                setupSpatialLeafAtPath(stage, branchLinks.at(4), TfToken("leaf1_2"), branchLinks.at(4), TfToken("b3"));

            // setup 2nd spatial tendon
            setupSpatialRootAtPath(stage, branchLinks.at(4), TfToken("root2"));
            setupSpatialLeafAtPath(stage, mainLinks.at(4), TfToken("leaf2_1"), branchLinks.at(4), TfToken("root2"));

            // parse
            physxSim->attachStage(stageId);

            // stage->Export("generatedArt.usda");

            PxArticulationReducedCoordinate* pxArticulation =
                getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(
                    fixedJointPath, PhysXType::ePTArticulation);

            // start simulating to get up-to-date PhysX values
            physxSim->simulate(0.001f, 0.0f);
            physxSim->fetchResults();

            PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
            CHECK_EQ(1u, pxScene->getNbArticulations());
            CHECK_EQ(2u, pxArticulation->getNbSpatialTendons());

            PxArticulationSpatialTendon* tendons[2];
            PxArticulationAttachment* attachment = nullptr;
            pxArticulation->getSpatialTendons(tendons, 2u, 0u);
            REQUIRE(tendons[0]);
            REQUIRE(tendons[1]);

            // We cannot predict which of the two tendons ends up at which PhysX index, so we infer the indices here
            size_t idx1, idx2;
            if (tendons[0]->getNbAttachments() == 7)
            {
                idx1 = 0;
                idx2 = 1;
            }
            else
            {
                idx1 = 1;
                idx2 = 0;
            }

            CHECK_EQ(7u, tendons[idx1]->getNbAttachments());
            CHECK_EQ(2u, tendons[idx2]->getNbAttachments());

            // 1st tendon with seven attachments:
            PxArticulationAttachment* attachments[7u];
            tendons[idx1]->getAttachments(attachments, 7u);

            // root: (tendonRootGearing must be ignored and set to 1.0)
            CHECK_EQ(1.f, attachments[0]->getCoefficient());

            // first attachment:
            CHECK_EQ(tendonGearing, attachments[1]->getCoefficient());

            size_t branchStart = 2u;
            PxArticulationAttachment* leafOne = nullptr;
            if (attachments[2]->isLeaf())
            {
                leafOne = attachments[2];
                branchStart = 3u;
            }
            else
            {
                leafOne = attachments[6u];
            }

            // not straightforward but the first leaf attachment is the sum of two hops:
            // 1) root1 to a1 that is 2x scaled link length scaled by tendonGearing (=attachment gearing)
            // 2) a1 to leaf that is 2x scaled link length scaled by leafGearing
            float leafGearing = 0.0f;
            PXR_NS::PhysxSchemaPhysxTendonAttachmentAPI(leafApi, leafApi.GetName()).GetGearingAttr().Get(&leafGearing);
            const float restLengthOne = scale[0] * linkDims[0] * (2.0f * leafGearing + tendonGearing * 2.0f);
            CHECK(leafOne->getRestLength() == doctest::Approx(restLengthOne));
            CHECK_EQ(leafOne->getCoefficient(), leafGearing);

            // then can check the second branch rest length:
            // this one is even worse:
            // 1) root1 to a1 that is 2.0 scaled link length in x scaled by tendonGearing (=attachment gearing)
            // --> total 2.0 in x
            // 2) a1 to b1 that is 1 scaled link length in y scaled by tendon gearing
            // 3) b1 to b2 (b2 coincides with a1) that is 1 scaled link length in y scaled by tendon gearing
            // 4) b2 to b3 that is 4 scaled link length in y scaled by tendon gearing
            // --> total 6.0 in y
            // 5) leaf coincides with b3, so zero contribution
            PxArticulationAttachment* leafTwo = attachments[branchStart + 3u];
            REQUIRE(leafTwo->isLeaf());
            const float restLengthTwo =
                scale[0] * linkDims[0] * 2.f * tendonGearing + scale[1] * linkDims[1] * 6.0f * tendonGearing;
            CHECK(leafTwo->getRestLength() == doctest::Approx(restLengthTwo));
            CHECK_EQ(leafTwo->getCoefficient(), leafGearing);

            // 2nd tendon
            tendons[idx2]->getAttachments(&attachment, 1u, 1u);
            REQUIRE(attachment);
            REQUIRE(attachment->isLeaf());
            float restLength = static_cast<float>(std::sqrt(std::pow(scale[0] * linkDims[0] * 2.f, 2u) +
                                                            std::pow(scale[1] * linkDims[1] * 4.f, 2u))); // constant
                                                                                                          // factors
                                                                                                          // arise
                                                                                                          // because of
                                                                                                          // gearing
                                                                                                          // values
            CHECK(attachment->getRestLength() == doctest::Approx(restLength));

            // check for specific change listeners & in-depth coefficient checks
            const float setVal = -135.f;
            attachApi.GetGearingAttr().Set(setVal);
            physxSim->simulate(0.001f, 0.001f); // simulate a tiny step so change is reflected in PhysX values
            physxSim->fetchResults();
            CHECK_EQ(tendonGearing, attachments[1]->getCoefficient()); // leaf attachment, has USD default gearing
            CHECK_EQ(leafOne->getCoefficient(), leafGearing);
            CHECK_EQ(tendonGearing, attachments[branchStart]->getCoefficient()); // standard attachment
            CHECK_EQ(setVal, attachments[branchStart + 1u]->getCoefficient()); // standard attachment, modified using
                                                                               // change listeners
            CHECK_EQ(tendonGearing, attachments[branchStart + 2u]->getCoefficient()); // standard attachment
            CHECK_EQ(leafTwo->getCoefficient(), leafGearing);
        }

        SUBCASE("Fixed Tendon")
        {
            // setup fixed tendon
            setupFixedTendonRoot(stage, mainJoints.at(1), TfToken("t1")); // topological root
            setupFixedTendonAxis(stage, mainJoints.at(2), TfToken("t1"));

            // setup branch
            setupFixedTendonAxis(stage, branchJoints.at(0), TfToken("t1"));

            // setup second tendon
            setupFixedTendonRoot(stage, branchJoints.at(0), TfToken("t2"));
            setupFixedTendonAxis(stage, branchJoints.at(1), TfToken("t2"));

            // parse
            physxSim->attachStage(stageId);
            physx->forceLoadPhysicsFromUSD();

            // stage->Export("generatedArt.usda");

            PxArticulationReducedCoordinate* pxArticulation =
                getPhysxBaseDerivedFromPathChecked<PxArticulationReducedCoordinate>(
                    fixedJointPath, PhysXType::ePTArticulation);

            PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
            CHECK_EQ(1u, pxScene->getNbArticulations());
            CHECK_EQ(2u, pxArticulation->getNbFixedTendons());

            PxArticulationFixedTendon* tendons[2];
            pxArticulation->getFixedTendons(tendons, 2u, 0u);

            REQUIRE(tendons[0]);
            REQUIRE(tendons[1]);

            // again we need to check at which PhysX index our tendons ended up
            size_t idx1, idx2;
            if (tendons[0]->getNbTendonJoints() == 4)
            {
                idx1 = 0;
                idx2 = 1;
            }
            else
            {
                idx1 = 1;
                idx2 = 0;
            }

            CHECK_EQ(4u, tendons[idx1]->getNbTendonJoints()); // this check also ensures that we found the correct
                                                              // topological root
            CHECK_EQ(3u, tendons[idx2]->getNbTendonJoints());
        }
    }

    // Common post-test actions
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

    std::string usdFileName = physicsTests.getUnitTestsDataDirectory() + "franka_ordering_repro.usd";

    UsdStageRefPtr stage = UsdStage::Open(usdFileName);
    REQUIRE(stage);

    pxr::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);

    SUBCASE("Articulation links consistency")
    {
        std::vector<PxArticulationLink*> linksBase;
        std::vector<PxArticulationLink*> links;
        pxr::UsdPrimRange range = stage->Traverse();
        for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
        {
            const pxr::UsdPrim& prim = *iter;
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
                        for (int i = 0; i < 3; i++)
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
    }

    physxSim->detachStage();

    pxr::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
