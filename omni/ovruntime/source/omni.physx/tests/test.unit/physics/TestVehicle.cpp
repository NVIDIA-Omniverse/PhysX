// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/settings/ISettings.h>

#include "../../common/TestVehicleFactory.h"
#include "../../common/PhysicsChangeTemplate.h"

#include "../common/TestHelpers.h"

#include "PhysicsTools.h"

#include <omni/physx/IPhysxSettings.h>
#include <omni/physx/IPhysxSimulation.h>


static const bool gUsePVD = false;


static PXR_NS::PhysxSchemaPhysxVehicleWheelAPI getWheel(const PXR_NS::UsdPrim& wheelAttPrim, PXR_NS::UsdStageRefPtr stage)
{
    PXR_NS::SdfPathVector paths;
    PXR_NS::PhysxSchemaPhysxVehicleWheelAttachmentAPI wheelAttAPI(wheelAttPrim);
    wheelAttAPI.GetWheelRel().GetTargets(&paths);
    PXR_NS::UsdPrim wheelPrim = stage->GetPrimAtPath(paths[0]);
    return PXR_NS::PhysxSchemaPhysxVehicleWheelAPI(wheelPrim);
}

static void getWheelRadiusAndWidth(const PXR_NS::UsdPrim& wheelAttPrim, PXR_NS::UsdStageRefPtr stage,
    float& radius, float& width)
{
    PXR_NS::PhysxSchemaPhysxVehicleWheelAPI wheelAPI = getWheel(wheelAttPrim, stage);
    wheelAPI.GetRadiusAttr().Get(&radius);
    wheelAPI.GetWidthAttr().Get(&width);
}

static PXR_NS::GfVec3f localDirToWorld(const PXR_NS::GfVec3f& dir, const PXR_NS::UsdGeomXformable& xformable)
{
    PXR_NS::GfMatrix4d transform = xformable.ComputeLocalToWorldTransform(0);
    PXR_NS::GfRotation orient = transform.ExtractRotation();
    return PXR_NS::GfVec3f(orient.TransformDir(dir));
}

static float getDeltaAngleRadians(const PXR_NS::GfVec3f& dirA, const PXR_NS::GfVec3f& dirB)
{
    const float angle = acosf(PXR_NS::GfDot(dirA, dirB));
    return angle;
}


TEST_CASE_TEMPLATE("Vehicle Property Write Tests", T, omni::physx::USDChange, omni::physx::FabricChange)
{
    carb::Framework* framework = carb::getFramework();
    carb::settings::ISettings* settings = framework->acquireInterface<carb::settings::ISettings>();

    bool pvdEnabled;
    if (gUsePVD)
    {
        pvdEnabled = settings->getAsBool(omni::physx::kSettingPVDEnabled);
        settings->setBool(omni::physx::kSettingPVDEnabled, true);
    }

    T changeTemplate;

    const UnitScale unitScale = { 1.0f, 1.0f };
    const float timeStep = 1.0f / 60.0f;

    // setup basic stage
    PXR_NS::UsdStageRefPtr stage = PXR_NS::UsdStage::CreateInMemory();
    const PXR_NS::SdfPath defaultPrimPath("/World");
    PXR_NS::UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    PXR_NS::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = PXR_NS::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    omni::physx::IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    omni::physx::IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    const uint32_t vehicleCount = 1;
    PXR_NS::SdfPath vehiclePaths[vehicleCount];
    PXR_NS::SdfPath vehicleWheelPaths[vehicleCount][VehicleFactory::WheelAttId::eCOUNT];

    VehicleFactory::Car4WheelsScenarioParams params;
    params.driveMode = VehicleFactory::DriveMode::eNONE;
    params.vehiclePathsOut = vehiclePaths;
    params.wheelAttachmentPathsOut = vehicleWheelPaths;

    VehicleFactory::create4WheeledCarsScenario(stage, unitScale, vehicleCount, params);

    changeTemplate.init(stageId, physicsTests.getApp()->getFramework());

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);

    float currentTime = 0.0f;

    PXR_NS::UsdPrim vehiclePrim = stage->GetPrimAtPath(vehiclePaths[0]);
    static const PXR_NS::TfToken translateToken = PXR_NS::UsdGeomXformOp::GetOpName(PXR_NS::UsdGeomXformOp::TypeTranslate);
    PXR_NS::GfVec3f startPos;
    vehiclePrim.GetAttribute(translateToken).Get(&startPos);

    const float epsilon = 0.01f;

    SUBCASE("WheelController: drive torque")
    {
        const float driveTorque = 500.0f;
        const PXR_NS::TfToken& driveTorqueToken = PXR_NS::PhysxSchemaTokens->physxVehicleWheelControllerDriveTorque;

        changeTemplate.setAttributeValue(vehicleWheelPaths[0][VehicleFactory::WheelAttId::eFL], driveTorqueToken, driveTorque);
        changeTemplate.setAttributeValue(vehicleWheelPaths[0][VehicleFactory::WheelAttId::eFR], driveTorqueToken, driveTorque);
        
        for (uint32_t i = 0; i < 60; i++)
        {
            physxSim->simulate(timeStep, currentTime);
            physxSim->fetchResults();
            currentTime += timeStep;
        }

        PXR_NS::GfVec3f endPos;
        vehiclePrim.GetAttribute(translateToken).Get(&endPos);
        PXR_NS::GfVec3f delta = endPos - startPos;

        REQUIRE_GT(delta[2], 0.5f);
        REQUIRE_LT(fabsf(delta[0]), epsilon);
        REQUIRE_LT(fabsf(delta[1]), epsilon);
    }

    SUBCASE("WheelController: brake torque")
    {
        const PXR_NS::TfToken& velocityToken = PXR_NS::UsdPhysicsTokens->physicsVelocity;
        const float brakeTorque = 3600.0f;
        const PXR_NS::TfToken& brakeTorqueToken = PXR_NS::PhysxSchemaTokens->physxVehicleWheelControllerBrakeTorque;

        const float startSpeed = 5.0f;
        PXR_NS::GfVec3f startVel(0, 0, startSpeed);
        changeTemplate.setAttributeValue(vehiclePaths[0], velocityToken, startVel);

        PXR_NS::UsdPrim wheelAttPrim = stage->GetPrimAtPath(vehicleWheelPaths[0][0]);
        float wheelRadius, wheelWidth;
        getWheelRadiusAndWidth(wheelAttPrim, stage, wheelRadius, wheelWidth);
        float rotationSpeed = startSpeed / wheelRadius;  // assuming free rolling

        for (uint32_t i = 0; i < VehicleFactory::WheelAttId::eCOUNT; i++)
        {
            omni::physx::usdparser::ObjectId objectId = physx->getObjectId(vehicleWheelPaths[0][i], omni::physx::ePTVehicleWheelAttachment);
            physx->setWheelRotationSpeed(&objectId, 1, &rotationSpeed);
        }

        changeTemplate.setAttributeValue(vehicleWheelPaths[0][VehicleFactory::WheelAttId::eFL], brakeTorqueToken, brakeTorque);
        changeTemplate.setAttributeValue(vehicleWheelPaths[0][VehicleFactory::WheelAttId::eFR], brakeTorqueToken, brakeTorque);
        changeTemplate.setAttributeValue(vehicleWheelPaths[0][VehicleFactory::WheelAttId::eRL], brakeTorqueToken, brakeTorque);
        changeTemplate.setAttributeValue(vehicleWheelPaths[0][VehicleFactory::WheelAttId::eRR], brakeTorqueToken, brakeTorque);

        const uint32_t iterCount = 60;
        const float elapsedTime = iterCount * timeStep;
        const float maxDist = elapsedTime * startSpeed;
        for (uint32_t i = 0; i < iterCount; i++)
        {
            physxSim->simulate(timeStep, currentTime);
            physxSim->fetchResults();
            currentTime += timeStep;
        }

        PXR_NS::GfVec3f endPos;
        vehiclePrim.GetAttribute(translateToken).Get(&endPos);
        PXR_NS::GfVec3f delta = endPos - startPos;

        const float travelDistUpperBound = 2.0f;
        const float travelDistLowerBound = 1.0f;
        REQUIRE_LT(travelDistUpperBound, maxDist * 0.5f);
        REQUIRE_GT(delta[2], travelDistLowerBound);  // make sure the vehicle did move initially
        REQUIRE_LT(delta[2], travelDistUpperBound);
        REQUIRE_LT(fabsf(delta[0]), epsilon);
        REQUIRE_LT(fabsf(delta[1]), epsilon);
    }

    SUBCASE("WheelController: steer angle")
    {
        PXR_NS::GfVec3f refDir(0.0f, 0.0f, 1.0f);
        PXR_NS::UsdPrim wheelAttPrim = stage->GetPrimAtPath(vehicleWheelPaths[0][VehicleFactory::WheelAttId::eFL]);
        PXR_NS::GfVec3f startDir = localDirToWorld(refDir, PXR_NS::UsdGeomXformable(wheelAttPrim));
        startDir.Normalize();

        const float steerAngle = physx::PxHalfPi / 3.0f;
        const PXR_NS::TfToken& steerAngleToken = PXR_NS::PhysxSchemaTokens->physxVehicleWheelControllerSteerAngle;

        changeTemplate.setAttributeValue(vehicleWheelPaths[0][VehicleFactory::WheelAttId::eFL], steerAngleToken, steerAngle);

        for (uint32_t i = 0; i < 1; i++)
        {
            physxSim->simulate(timeStep, currentTime);
            physxSim->fetchResults();
            currentTime += timeStep;
        }

        PXR_NS::GfVec3f endDir = localDirToWorld(refDir, PXR_NS::UsdGeomXformable(wheelAttPrim));
        endDir.Normalize();
        float deltaAngle = fabsf(getDeltaAngleRadians(startDir, endDir));

        REQUIRE_LT(fabsf(deltaAngle - steerAngle), steerAngle * 0.01f);
    }

    // Common post-test actions
    physxSim->detachStage();

    changeTemplate.destroy();

    PXR_NS::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;

    if (gUsePVD)
        settings->setBool(omni::physx::kSettingPVDEnabled, pvdEnabled);
}


TEST_CASE("Vehicle Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=msauter][priority=mandatory]")
{
    carb::Framework* framework = carb::getFramework();
    carb::settings::ISettings* settings = framework->acquireInterface<carb::settings::ISettings>();

    bool pvdEnabled;
    if (gUsePVD)
    {
        pvdEnabled = settings->getAsBool(omni::physx::kSettingPVDEnabled);
        settings->setBool(omni::physx::kSettingPVDEnabled, true);
    }

    const UnitScale unitScale = { 1.0f, 1.0f };
    const float timeStep = 1.0f / 60.0f;

    // setup basic stage
    PXR_NS::UsdStageRefPtr stage = PXR_NS::UsdStage::CreateInMemory();
    const PXR_NS::SdfPath defaultPrimPath("/World");
    PXR_NS::UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);
    stage->SetDefaultPrim(defaultPrim);

    PXR_NS::UsdUtilsStageCache::Get().Insert(stage);
    long stageId = PXR_NS::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    omni::physx::IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    omni::physx::IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    const uint32_t vehicleCount = 1;
    PXR_NS::SdfPath vehiclePaths[vehicleCount];
    PXR_NS::SdfPath vehicleWheelPaths[vehicleCount][VehicleFactory::WheelAttId::eCOUNT];

    VehicleFactory::Car4WheelsScenarioParams params;
    params.driveMode = VehicleFactory::DriveMode::eNONE;
    params.vehiclePathsOut = vehiclePaths;
    params.wheelAttachmentPathsOut = vehicleWheelPaths;

    VehicleFactory::create4WheeledCarsScenario(stage, unitScale, vehicleCount, params);

    // attach sim to stage which parses and creates the pointers that we can check directly
    physxSim->attachStage(stageId);

    float currentTime = 0.0f;

    PXR_NS::UsdPrim vehiclePrim = stage->GetPrimAtPath(vehiclePaths[0]);
    static const PXR_NS::TfToken translateToken = PXR_NS::UsdGeomXformOp::GetOpName(PXR_NS::UsdGeomXformOp::TypeTranslate);
    static const PXR_NS::TfToken orientToken = PXR_NS::UsdGeomXformOp::GetOpName(PXR_NS::UsdGeomXformOp::TypeOrient);
    PXR_NS::GfVec3f startPos;
    vehiclePrim.GetAttribute(translateToken).Get(&startPos);

    SUBCASE("API: getWheelTransformations")
    {
        PXR_NS::GfVec3f pos;
        vehiclePrim.GetAttribute(translateToken).Get(&pos);
        pos[1] += 0.35f * unitScale.lengthScale;
        vehiclePrim.GetAttribute(translateToken).Set(pos);

        physxSim->simulate(timeStep, currentTime);
        physxSim->fetchResults();
        currentTime += timeStep;

        vehiclePrim.GetAttribute(translateToken).Get(&pos);
        PXR_NS::GfQuatf orient;
        vehiclePrim.GetAttribute(orientToken).Get(&orient);

        PXR_NS::SdfPath wheelAttPaths[2];
        wheelAttPaths[0] = vehicleWheelPaths[0][VehicleFactory::WheelAttId::eFL];
        wheelAttPaths[1] = vehicleWheelPaths[0][VehicleFactory::WheelAttId::eRR];
        PXR_NS::UsdPrim wheelAttPrims[2];
        PXR_NS::GfVec3f wheelLocalPos[2];
        PXR_NS::GfVec3f wheelWorldPos[2];
        int wheelIndices[2];
        for (uint32_t i = 0; i < 2; i++)
        {
            wheelAttPrims[i] = stage->GetPrimAtPath(wheelAttPaths[i]);
            wheelAttPrims[i].GetAttribute(translateToken).Get(&wheelLocalPos[i]);
            wheelWorldPos[i] = pos + PXR_NS::GfVec3f(PXR_NS::GfRotation(orient).TransformDir(wheelLocalPos[i]));
            wheelIndices[i] = physx->getWheelIndex(wheelAttPaths[i]);
        }

        omni::physx::usdparser::ObjectId vehicleId = physx->getObjectId(vehiclePrim.GetPath(), omni::physx::ePTVehicle);
        carb::Float3 wPos[2];
        carb::Float4 wOrient[2];

        const float epsilon = 0.0001f;

        bool success = physx->getWheelTransformations(vehicleId, wheelIndices, 2, false, wPos, wOrient);
        REQUIRE(success);
        for (uint32_t i = 0; i < 2; i++)
        {
            compare(wheelLocalPos[i], PXR_NS::GfVec3f(wPos[i].x, wPos[i].y, wPos[i].z), epsilon);
        }

        success = physx->getWheelTransformations(vehicleId, wheelIndices, 2, true, wPos, wOrient);
        REQUIRE(success);
        for (uint32_t i = 0; i < 2; i++)
        {
            compare(wheelWorldPos[i], PXR_NS::GfVec3f(wPos[i].x, wPos[i].y, wPos[i].z), epsilon);
        }
    }

    // Common post-test actions
    physxSim->detachStage();

    PXR_NS::UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;

    if (gUsePVD)
        settings->setBool(omni::physx::kSettingPVDEnabled, pvdEnabled);
}
