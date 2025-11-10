// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"
#include "../../common/Tools.h"
#include "../../common/PhysicsChangeTemplate.h"

#include "PhysicsTools.h"

#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxFabric.h>

#include <omni/fabric/SimStageWithHistory.h>

#include <omni/physx/PhysxTokens.h>
#include <common/foundation/TypeCast.h>

using namespace omni::physx;
using namespace pxr;

GfMatrix4d computeMatrix(const pxr::GfVec3d& translate, const pxr::GfMatrix3d& rotate, const pxr::GfVec3d& scale)
{
    // Order is scale*rotate*translate
    return GfMatrix4d(rotate[0][0] * scale[0], rotate[0][1] * scale[0], rotate[0][2] * scale[0], 0,
                      rotate[1][0] * scale[1], rotate[1][1] * scale[1], rotate[1][2] * scale[1], 0,
                      rotate[2][0] * scale[2], rotate[2][1] * scale[2], rotate[2][2] * scale[2], 0, translate[0],
                      translate[1], translate[2], 1);
}


TEST_CASE("Fabric Direct-GPU API Changes Tests",
          "[omniphysics]"
          "[component=OmniPhysics][owner=adenzler][priority=mandatory]")
{
    ScopedFabricActivation fabricEnable;
    FabricChange fabricTemplate;

    const GfVec3f gravityDirection(0.0f, 0.0f, -1.0f); // use z-up
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const float kilogramsPerUnit = 1.0f;
    const float gravityMagnitude = 10.0f / metersPerStageUnit;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    // set suppress readback
    carb::settings::ISettings* settings =
        physicsTests.getApp()->getFramework()->acquireInterface<carb::settings::ISettings>();
    const bool suppressReadback = settings->getAsBool(kSettingSuppressReadback);
    settings->setBool(kSettingSuppressReadback, true);

    // setup basic stage
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(metersPerStageUnit));
    UsdPhysicsSetStageKilogramsPerUnit(stage, static_cast<double>(kilogramsPerUnit));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);

    UsdUtilsStageCache::Get().Insert(stage);
    long stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);
    scene.GetGravityMagnitudeAttr().Set(gravityMagnitude);
    scene.GetGravityDirectionAttr().Set(gravityDirection);

    // setup a stage with articulations and rigid bodies
    const GfVec3f linkDims(10.f, 10.f, 10.f);
    GfVec3f linkPos(0.f, 0.f, 50.f);

    const GfVec3f linkColor(0.1f, 0.1f, 0.7f);

    const size_t nbArticulations = 5u;
    std::vector<SdfPath> paths;
    std::vector<carb::Double3> positions;
    std::vector<carb::Float4> orientations;
    std::vector<pxr::GfMatrix4d> worldMatrices;

    for (size_t i = 0; i < nbArticulations; ++i)
    {

        TfToken token("env");
        std::string tokenString = token.GetString() + std::to_string(i);
        token = TfToken(tokenString);
        SdfPath envPath = defaultPrimPath.AppendChild(token);

        SdfPath rootPath = envPath.AppendChild(TfToken("root"));
        addRigidBox(stage, rootPath.GetString(), linkDims, linkPos, GfQuatf(1.0f), linkColor, 1.f);
        UsdPrim rootPrim = stage->GetPrimAtPath(rootPath);
        CHECK(rootPrim);
        UsdGeomXformCache cache;
        const GfMatrix4d worldMat = cache.GetLocalToWorldTransform(rootPrim);
        const GfTransform tr(worldMat);
        UsdPhysicsArticulationRootAPI::Apply(rootPrim);
        paths.push_back(rootPath);
        positions.push_back(toDouble3(linkPos));
        orientations.push_back(toFloat4(GfQuatf(1.0f)));
        pxr::GfMatrix4d mat(1.0);
        //mat.SetTranslate(linkPos);
        //mat.SetRotateOnly(GfQuatf(1.0f));
        mat = computeMatrix(GfVec3d(linkPos), GfMatrix3d(1.0), tr.GetScale());
        worldMatrices.push_back(mat);

        GfVec3f childPos = linkPos;
        childPos[1] += 10.f;
        SdfPath childPath = envPath.AppendChild(TfToken("child"));
        addRigidBox(stage, childPath.GetString(), linkDims, childPos, GfQuatf(1.0f), linkColor, 1.f);
        UsdPrim childPrim = stage->GetPrimAtPath(childPath);
        CHECK(childPrim);
        paths.push_back(childPath);
        positions.push_back(toDouble3(childPos));
        orientations.push_back(toFloat4(GfQuatf(1.0f)));
        pxr::GfMatrix4d mat2(1.0);
        //mat2.SetTranslate(childPos);
        //mat2.SetRotateOnly(GfQuatf(1.0f));
        const GfMatrix4d worldMatChild = cache.GetLocalToWorldTransform(childPrim);
        const GfTransform trChild(worldMatChild);
        mat2 = computeMatrix(GfVec3d(childPos), GfMatrix3d(1.0), trChild.GetScale());
        worldMatrices.push_back(mat2);

        SdfPath jointPath = envPath.AppendChild(TfToken("joint"));
        UsdPhysicsRevoluteJoint joint = UsdPhysicsRevoluteJoint::Define(stage, jointPath);
        CHECK(joint);
        joint.CreateBody0Rel().SetTargets(SdfPathVector({ rootPath }));
        joint.CreateBody1Rel().SetTargets(SdfPathVector({ childPath }));
        joint.CreateAxisAttr().Set(TfToken("Y"));
        joint.CreateLocalPos0Attr().Set(GfVec3f(0.0f, 0.5f, 0.f));
        joint.CreateLocalPos1Attr().Set(GfVec3f(0.0f, -0.5f, 0.f));
        joint.CreateLocalRot0Attr().Set(GfQuatf(1.0f));
        joint.CreateLocalRot1Attr().Set(GfQuatf(1.0f));


        linkPos[0] += 50.f;
    }

    // also do a bunch of normal rigid bodies
    const size_t nbRigidBodies = 10;
    for (size_t i = 0; i < nbRigidBodies; ++i)
    {
        TfToken token("env");
        std::string tokenString = token.GetString() + std::to_string(i);
        token = TfToken(tokenString);
        SdfPath envPath = defaultPrimPath.AppendChild(token);

        SdfPath rigidBodyPath = envPath.AppendChild(TfToken("rb"));
        addRigidBox(stage, rigidBodyPath.GetString(), linkDims, linkPos, GfQuatf(1.0f), linkColor, 1.f);
        UsdPrim rootPrim = stage->GetPrimAtPath(rigidBodyPath);
        CHECK(rootPrim);
        UsdGeomXformCache cache;
        const GfMatrix4d worldMat = cache.GetLocalToWorldTransform(rootPrim);
        const GfTransform tr(worldMat);
        paths.push_back(rigidBodyPath);
        positions.push_back(toDouble3(linkPos));
        orientations.push_back(toFloat4(GfQuatf(1.0f)));
        pxr::GfMatrix4d mat(1.0);
        //mat.SetTranslate(linkPos);
        //mat.SetRotateOnly(GfQuatf(1.0f));
        mat = computeMatrix(GfVec3d(linkPos), GfMatrix3d(1.0), tr.GetScale());
        worldMatrices.push_back(mat);

        linkPos[0] += 50.f;
    }

    // This creates the fabric stage
    fabricTemplate.init(stageId, physicsTests.getApp()->getFramework());

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);
    fabricEnable.mIPhysxFabric->attachStage(stageId);

    const float dt = 1.f / 60.f;

    for (int i = 0; i < 1; ++i)
    {
        physxSim->simulate(dt, 0.0f);
        physxSim->fetchResults();
        fabricEnable.mIPhysxFabric->update(dt, 0.0f);
    }

    omni::fabric::Token worldMatrixToken(omni::physx::gWorldMatrixTokenString);
    omni::fabric::Token linVelToken(UsdPhysicsTokens->physicsVelocity.GetText());
    omni::fabric::Token angVelToken(UsdPhysicsTokens->physicsAngularVelocity.GetText());

    // test that transformations and velocities work.
    omni::fabric::IStageReaderWriter* iSip = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    omni::fabric::StageReaderWriter fabricStage = iSip->get(stageId);

    for (int i = 0; i < paths.size(); ++i)
    {
        omni::fabric::Path fabricPath = omni::fabric::Path(omni::fabric::asInt(paths[i]));

        // transform
        {
            REQUIRE(fabricStage.attributeExists(fabricPath, worldMatrixToken));
            const pxr::GfMatrix4d mat = *fabricStage.getAttributeRd<pxr::GfMatrix4d>(fabricPath, worldMatrixToken);
            
            pxr::GfVec3d pos = mat.ExtractTranslation();
            pxr::GfVec3d refpos = worldMatrices[i].ExtractTranslation();

            //check falling because position integration is not correct after only 1 step.
            CHECK(pos[2] < refpos[2]);
            CHECK((pos[0] - refpos[0]) < 1e-3);
            CHECK((pos[1] - refpos[1]) < 1e-3);

            compare(carb::Double3{ mat[0][0], mat[0][1], mat[0][2] },
                    carb::Double3{ worldMatrices[i][0][0], worldMatrices[i][0][1], worldMatrices[i][0][2] }, 1e-3);
            compare(carb::Double3{ mat[1][0], mat[1][1], mat[1][2] },
                    carb::Double3{ worldMatrices[i][1][0], worldMatrices[i][1][1], worldMatrices[i][1][2] }, 1e-3);
            compare(carb::Double3{ mat[2][0], mat[2][1], mat[2][2] },
                    carb::Double3{ worldMatrices[i][2][0], worldMatrices[i][2][1], worldMatrices[i][2][2] }, 1e-3);

        }

        // linear velocity
        {
            REQUIRE(fabricStage.attributeExists(fabricPath, linVelToken));
            const carb::Float3 linVel = *fabricStage.getAttributeRd<carb::Float3>(fabricPath, linVelToken);

            carb::Float3 referenceVel{ 0.f, 0.f, -gravityMagnitude * dt };
            compare(linVel, referenceVel, 1e-3);
        }

        // angular velocity
        {
            REQUIRE(fabricStage.attributeExists(fabricPath, angVelToken));
            const carb::Float3 angVel = *fabricStage.getAttributeRd<carb::Float3>(fabricPath, angVelToken);

            const carb::Float3 zero{ 0.f, 0.f, 0.f };
            compare(angVel, zero, 1e-3);
        }
    }

    fabricEnable.mIPhysxFabric->detachStage();
    physxSim->detachStage();
    fabricTemplate.destroy();

    UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;

    settings->setBool(kSettingSuppressReadback, suppressReadback);
}
