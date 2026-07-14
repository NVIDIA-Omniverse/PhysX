// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <private/omni/physx/PhysxUsd.h>

#include "TestVehicleFactory.h"


//
// ring around x-axis
//
static const PXR_NS::GfVec3f gRingMeshPoints[] = {
    PXR_NS::GfVec3f(1.000000f, 0.000000f, -1.000000f), PXR_NS::GfVec3f(-1.000000f, 0.000000f, -1.000000f),
    PXR_NS::GfVec3f(1.000000f, 0.195090f, -0.980785f), PXR_NS::GfVec3f(-1.000000f, 0.195090f, -0.980785f),
    PXR_NS::GfVec3f(1.000000f, 0.382683f, -0.923880f), PXR_NS::GfVec3f(-1.000000f, 0.382683f, -0.923880f),
    PXR_NS::GfVec3f(1.000000f, 0.555570f, -0.831470f), PXR_NS::GfVec3f(-1.000000f, 0.555570f, -0.831470f),
    PXR_NS::GfVec3f(1.000000f, 0.707107f, -0.707107f), PXR_NS::GfVec3f(-1.000000f, 0.707107f, -0.707107f),
    PXR_NS::GfVec3f(1.000000f, 0.831470f, -0.555570f), PXR_NS::GfVec3f(-1.000000f, 0.831470f, -0.555570f),
    PXR_NS::GfVec3f(1.000000f, 0.923880f, -0.382683f), PXR_NS::GfVec3f(-1.000000f, 0.923880f, -0.382683f),
    PXR_NS::GfVec3f(1.000000f, 0.980785f, -0.195090f), PXR_NS::GfVec3f(-1.000000f, 0.980785f, -0.195090f),
    PXR_NS::GfVec3f(1.000000f, 1.000000f, 0.000000f), PXR_NS::GfVec3f(-1.000000f, 1.000000f, 0.000000f),
    PXR_NS::GfVec3f(1.000000f, 0.980785f, 0.195090f), PXR_NS::GfVec3f(-1.000000f, 0.980785f, 0.195090f),
    PXR_NS::GfVec3f(1.000000f, 0.923880f, 0.382684f), PXR_NS::GfVec3f(-1.000000f, 0.923880f, 0.382684f),
    PXR_NS::GfVec3f(1.000000f, 0.831470f, 0.555570f), PXR_NS::GfVec3f(-1.000000f, 0.831470f, 0.555570f),
    PXR_NS::GfVec3f(1.000000f, 0.707107f, 0.707107f), PXR_NS::GfVec3f(-1.000000f, 0.707107f, 0.707107f),
    PXR_NS::GfVec3f(1.000000f, 0.555570f, 0.831470f), PXR_NS::GfVec3f(-1.000000f, 0.555570f, 0.831470f),
    PXR_NS::GfVec3f(1.000000f, 0.382683f, 0.923880f), PXR_NS::GfVec3f(-1.000000f, 0.382683f, 0.923880f),
    PXR_NS::GfVec3f(1.000000f, 0.195090f, 0.980785f), PXR_NS::GfVec3f(-1.000000f, 0.195090f, 0.980785f),
    PXR_NS::GfVec3f(1.000000f, -0.000000f, 1.000000f), PXR_NS::GfVec3f(-1.000000f, -0.000000f, 1.000000f),
    PXR_NS::GfVec3f(1.000000f, -0.195090f, 0.980785f), PXR_NS::GfVec3f(-1.000000f, -0.195090f, 0.980785f),
    PXR_NS::GfVec3f(1.000000f, -0.382683f, 0.923880f), PXR_NS::GfVec3f(-1.000000f, -0.382683f, 0.923880f),
    PXR_NS::GfVec3f(1.000000f, -0.555570f, 0.831470f), PXR_NS::GfVec3f(-1.000000f, -0.555570f, 0.831470f),
    PXR_NS::GfVec3f(1.000000f, -0.707107f, 0.707107f), PXR_NS::GfVec3f(-1.000000f, -0.707107f, 0.707107f),
    PXR_NS::GfVec3f(1.000000f, -0.831470f, 0.555570f), PXR_NS::GfVec3f(-1.000000f, -0.831470f, 0.555570f),
    PXR_NS::GfVec3f(1.000000f, -0.923880f, 0.382683f), PXR_NS::GfVec3f(-1.000000f, -0.923880f, 0.382683f),
    PXR_NS::GfVec3f(1.000000f, -0.980785f, 0.195090f), PXR_NS::GfVec3f(-1.000000f, -0.980785f, 0.195090f),
    PXR_NS::GfVec3f(1.000000f, -1.000000f, -0.000000f), PXR_NS::GfVec3f(-1.000000f, -1.000000f, -0.000000f),
    PXR_NS::GfVec3f(1.000000f, -0.980785f, -0.195090f), PXR_NS::GfVec3f(-1.000000f, -0.980785f, -0.195090f),
    PXR_NS::GfVec3f(1.000000f, -0.923879f, -0.382684f), PXR_NS::GfVec3f(-1.000000f, -0.923879f, -0.382684f),
    PXR_NS::GfVec3f(1.000000f, -0.831469f, -0.555570f), PXR_NS::GfVec3f(-1.000000f, -0.831469f, -0.555570f),
    PXR_NS::GfVec3f(1.000000f, -0.707107f, -0.707107f), PXR_NS::GfVec3f(-1.000000f, -0.707107f, -0.707107f),
    PXR_NS::GfVec3f(1.000000f, -0.555570f, -0.831470f), PXR_NS::GfVec3f(-1.000000f, -0.555570f, -0.831470f),
    PXR_NS::GfVec3f(1.000000f, -0.382683f, -0.923880f), PXR_NS::GfVec3f(-1.000000f, -0.382683f, -0.923880f),
    PXR_NS::GfVec3f(1.000000f, -0.195090f, -0.980785f), PXR_NS::GfVec3f(-1.000000f, -0.195090f, -0.980785f)
};
static const size_t gRingMeshPointCount = sizeof(gRingMeshPoints) / sizeof(gRingMeshPoints[0]);

static const int gRingMeshFaceVertexIndices[] = {
    1, 0, 2,
    1, 2, 3,
    3, 2, 4,
    3, 4, 5,
    5, 4, 6,
    5, 6, 7,
    7, 6, 8,
    7, 8, 9,
    9, 8, 10,
    9, 10, 11,
    11, 10, 12,
    11, 12, 13,
    13, 12, 14,
    13, 14, 15,
    15, 14, 16,
    15, 16, 17,
    17, 16, 18,
    17, 18, 19,
    19, 18, 20,
    19, 20, 21,
    21, 20, 22,
    21, 22, 23,
    23, 22, 24,
    23, 24, 25,
    25, 24, 26,
    25, 26, 27,
    27, 26, 28,
    27, 28, 29,
    29, 28, 30,
    29, 30, 31,
    31, 30, 32,
    31, 32, 33,
    33, 32, 34,
    33, 34, 35,
    35, 34, 36,
    35, 36, 37,
    37, 36, 38,
    37, 38, 39,
    39, 38, 40,
    39, 40, 41,
    41, 40, 42,
    41, 42, 43,
    43, 42, 44,
    43, 44, 45,
    45, 44, 46,
    45, 46, 47,
    47, 46, 48,
    47, 48, 49,
    49, 48, 50,
    49, 50, 51,
    51, 50, 52,
    51, 52, 53,
    53, 52, 54,
    53, 54, 55,
    55, 54, 56,
    55, 56, 57,
    57, 56, 58,
    57, 58, 59,
    59, 58, 60,
    59, 60, 61,
    61, 60, 62,
    61, 62, 63,
    63, 62, 0,
    63, 0, 1
};
const size_t gRingMeshFaceVertexIndexCount = sizeof(gRingMeshFaceVertexIndices) / sizeof(gRingMeshFaceVertexIndices[0]);

static const int gRingMeshFaceVertexCounts[] = {
    3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3
};
const size_t gRingMeshFaceVertexCountEntryCount = sizeof(gRingMeshFaceVertexCounts) / sizeof(gRingMeshFaceVertexCounts[0]);

constexpr float gMaxCompressionInMeters = 0.1f;
constexpr float gMaxDroopInMeters = 0.1f;
constexpr float gTravelDistanceInMeters = gMaxCompressionInMeters + gMaxDroopInMeters;

static const VehicleFactory::AxesIndices gYZXAxes(1, 2, 0);


static PXR_NS::TfToken getAxisToken(const uint32_t axisIndex)
{
    if (axisIndex == 0)
        return PXR_NS::PhysxSchemaTokens.Get()->posX;
    else if (axisIndex == 1)
        return PXR_NS::PhysxSchemaTokens.Get()->posY;
    else
        return PXR_NS::PhysxSchemaTokens.Get()->posZ;
}


template<typename T>
static void convertGfVec3List(const PXR_NS::GfVec3f* pointListSrc, T pointListDst, const uint32_t pointCount,
    const VehicleFactory::AxesIndices& axesIndicesSrc,
    const VehicleFactory::AxesIndices& axesIndicesDst)
{
    for (uint32_t i = 0; i < pointCount; i++)
    {
        const PXR_NS::GfVec3f& pointSrc = pointListSrc[i];
        PXR_NS::GfVec3f& pointDst = pointListDst[i];

        pointDst[axesIndicesDst.up] = pointSrc[axesIndicesSrc.up];
        pointDst[axesIndicesDst.forward] = pointSrc[axesIndicesSrc.forward];
        pointDst[axesIndicesDst.side] = pointSrc[axesIndicesSrc.side];
    }
}


static PXR_NS::SdfPath getDefaultPrimPath(const PXR_NS::UsdStageRefPtr& stage)
{
    PXR_NS::UsdPrim defaultPrim = stage->GetDefaultPrim();
    if (defaultPrim)
        return stage->GetDefaultPrim().GetPath();
    else
        return PXR_NS::SdfPath("/");
}


static void addPhysicsMaterialToPrim(
    const PXR_NS::UsdStageRefPtr& stage,
    PXR_NS::UsdPrim& prim,
    const PXR_NS::SdfPath& materialPath)
{
    PXR_NS::UsdShadeMaterialBindingAPI bindingAPI = PXR_NS::UsdShadeMaterialBindingAPI::Apply(prim);
    PXR_NS::UsdShadeMaterial materialPrim(stage->GetPrimAtPath(materialPath));

    static const PXR_NS::TfToken physicsToken("physics");
    bindingAPI.Bind(materialPrim, PXR_NS::UsdShadeTokens->weakerThanDescendants, physicsToken);
}


static void addCollisionToCollisionGroup(
    const PXR_NS::UsdStageRefPtr& stage,
    const PXR_NS::SdfPath& collisionPrimPath,
    const PXR_NS::SdfPath& collisionGroupPath)
{
    PXR_NS::UsdPrim collisionGroupPrim = stage->GetPrimAtPath(collisionGroupPath);
    if (collisionGroupPrim)
    {
        PXR_NS::UsdCollectionAPI collectionAPI = PXR_NS::UsdCollectionAPI::Get(collisionGroupPrim, PXR_NS::UsdPhysicsTokens->colliders);
        if (collectionAPI)
            collectionAPI.GetIncludesRel().AddTarget(collisionPrimPath);
    }
}


void VehicleFactory::createCollisionGroups(
    const PXR_NS::UsdStageRefPtr& stage,
    PXR_NS::SdfPath(*collisionGroupPathsOut)[CollGroupId::eCOUNT])
{
    PXR_NS::SdfPath rootPath = getDefaultPrimPath(stage);

    static const PXR_NS::SdfPath chassisCollGroupPathPostfix("VehicleChassisCollisionGroup");
    PXR_NS::SdfPath collisionGroupVehicleChassisPath = rootPath.AppendPath(chassisCollGroupPathPostfix);

    static const PXR_NS::SdfPath wheelCollGroupPathPostfix("VehicleWheelCollisionGroup");
    PXR_NS::SdfPath collisionGroupVehicleWheelPath = rootPath.AppendPath(wheelCollGroupPathPostfix);

    static const PXR_NS::SdfPath groundQueryGroupPathPostfix("VehicleGroundQueryGroup");
    PXR_NS::SdfPath collisionGroupVehicleGroundQueryPath = rootPath.AppendPath(groundQueryGroupPathPostfix);

    static const PXR_NS::SdfPath groundSurfaceGroupPathPostfix("GroundSurfaceCollisionGroup");
    PXR_NS::SdfPath collisionGroupGroundSurfacePath = rootPath.AppendPath(groundSurfaceGroupPathPostfix);

    if (collisionGroupPathsOut)
    {
        (*collisionGroupPathsOut)[CollGroupId::eVEHICLE_CHASSIS] = collisionGroupVehicleChassisPath;
        (*collisionGroupPathsOut)[CollGroupId::eVEHICLE_WHEEL] = collisionGroupVehicleWheelPath;
        (*collisionGroupPathsOut)[CollGroupId::eVEHICLE_GROUND_QUERY] = collisionGroupVehicleGroundQueryPath;
        (*collisionGroupPathsOut)[CollGroupId::eGROUND_SURFACE] = collisionGroupGroundSurfacePath;
    }

    PXR_NS::UsdPhysicsCollisionGroup collisionGroupVehicleChassis = PXR_NS::UsdPhysicsCollisionGroup::Define(stage, collisionGroupVehicleChassisPath);
    PXR_NS::UsdRelationship collisionGroupVehicleChassisRel = collisionGroupVehicleChassis.CreateFilteredGroupsRel();
    collisionGroupVehicleChassisRel.AddTarget(collisionGroupVehicleChassisPath);

    PXR_NS::UsdPhysicsCollisionGroup collisionGroupVehicleWheel = PXR_NS::UsdPhysicsCollisionGroup::Define(stage, collisionGroupVehicleWheelPath);
    PXR_NS::UsdRelationship collisionGroupVehicleWheelRel = collisionGroupVehicleWheel.CreateFilteredGroupsRel();
    collisionGroupVehicleWheelRel.AddTarget(collisionGroupVehicleGroundQueryPath);
    collisionGroupVehicleWheelRel.AddTarget(collisionGroupGroundSurfacePath);

    PXR_NS::UsdPhysicsCollisionGroup collisionGroupVehicleGroundQuery = PXR_NS::UsdPhysicsCollisionGroup::Define(stage, collisionGroupVehicleGroundQueryPath);
    PXR_NS::UsdRelationship collisionGroupVehicleGroundQueryRel = collisionGroupVehicleGroundQuery.CreateFilteredGroupsRel();
    collisionGroupVehicleGroundQueryRel.AddTarget(collisionGroupVehicleChassisPath);
    collisionGroupVehicleGroundQueryRel.AddTarget(collisionGroupVehicleWheelPath);

    PXR_NS::UsdPhysicsCollisionGroup collisionGroupGroundSurface = PXR_NS::UsdPhysicsCollisionGroup::Define(stage, collisionGroupGroundSurfacePath);
    PXR_NS::UsdRelationship collisionGroupGroundSurfaceRel = collisionGroupGroundSurface.CreateFilteredGroupsRel();
    collisionGroupGroundSurfaceRel.AddTarget(collisionGroupGroundSurfacePath);
    collisionGroupGroundSurfaceRel.AddTarget(collisionGroupVehicleWheelPath);
}


void  VehicleFactory::createMaterialsAndTireFrictionTables(
    const PXR_NS::UsdStageRefPtr& stage,
    PXR_NS::SdfPath(*materialPathsOut)[MaterialId::eCOUNT],
    PXR_NS::SdfPath(*tireFrictionTablePathsOut)[TireFrictionTableId::eCOUNT])
{
    PXR_NS::SdfPath rootPath = getDefaultPrimPath(stage);

    //
    // materials
    //

    static const PXR_NS::SdfPath tarmacMaterialPathPostfix("TarmacMaterial");
    PXR_NS::SdfPath tarmacMaterialPath = rootPath.AppendPath(tarmacMaterialPathPostfix);
    PXR_NS::UsdShadeMaterial tarmacShadeMaterial = PXR_NS::UsdShadeMaterial::Define(stage, tarmacMaterialPath);
    PXR_NS::UsdPhysicsMaterialAPI tarmacMaterial = PXR_NS::UsdPhysicsMaterialAPI::Apply(tarmacShadeMaterial.GetPrim());
    tarmacMaterial.CreateStaticFrictionAttr().Set(0.9f);
    tarmacMaterial.CreateDynamicFrictionAttr().Set(0.7f);
    tarmacMaterial.CreateRestitutionAttr().Set(0.0f);
    PXR_NS::PhysxSchemaPhysxMaterialAPI::Apply(tarmacMaterial.GetPrim());

    static const PXR_NS::SdfPath gravelMaterialPathPostfix("GravelMaterial");
    PXR_NS::SdfPath gravelMaterialPath = rootPath.AppendPath(gravelMaterialPathPostfix);
    PXR_NS::UsdShadeMaterial gravelShadeMaterial = PXR_NS::UsdShadeMaterial::Define(stage, gravelMaterialPath);
    PXR_NS::UsdPhysicsMaterialAPI gravelMaterial = PXR_NS::UsdPhysicsMaterialAPI::Apply(gravelShadeMaterial.GetPrim());
    gravelMaterial.CreateStaticFrictionAttr().Set(0.6f);
    gravelMaterial.CreateDynamicFrictionAttr().Set(0.6f);
    gravelMaterial.CreateRestitutionAttr().Set(0.0f);
    PXR_NS::PhysxSchemaPhysxMaterialAPI::Apply(gravelMaterial.GetPrim());

    if (materialPathsOut)
    {
        (*materialPathsOut)[MaterialId::eTARMAC] = tarmacMaterialPath;
        (*materialPathsOut)[MaterialId::eGRAVEL] = gravelMaterialPath;
    }

    //
    // tire friction tables
    //

    static const PXR_NS::SdfPath winterTireFrictionTablePathPostfix("WinterTireFrictionTable");
    PXR_NS::SdfPath winterTireFrictionTablePath = rootPath.AppendPath(winterTireFrictionTablePathPostfix);
    PXR_NS::PhysxSchemaPhysxVehicleTireFrictionTable winterTireFrictionTable = PXR_NS::PhysxSchemaPhysxVehicleTireFrictionTable::Define(
        stage, winterTireFrictionTablePath);
    PXR_NS::UsdRelationship winterTireFrictionTableGroundMaterialsRel = winterTireFrictionTable.CreateGroundMaterialsRel();
    winterTireFrictionTableGroundMaterialsRel.AddTarget(tarmacMaterialPath);
    winterTireFrictionTableGroundMaterialsRel.AddTarget(gravelMaterialPath);
    winterTireFrictionTable.CreateFrictionValuesAttr().Set(PXR_NS::VtFloatArray({ 0.75f, 0.6f }));

    static const PXR_NS::SdfPath summerTireFrictionTablePathPostfix("SummerTireFrictionTable");
    PXR_NS::SdfPath summerTireFrictionTablePath = rootPath.AppendPath(summerTireFrictionTablePathPostfix);
    PXR_NS::PhysxSchemaPhysxVehicleTireFrictionTable summerTireFrictionTable = PXR_NS::PhysxSchemaPhysxVehicleTireFrictionTable::Define(
        stage, summerTireFrictionTablePath);
    PXR_NS::UsdRelationship summerTireFrictionTableGroundMaterialsRel = summerTireFrictionTable.CreateGroundMaterialsRel();
    summerTireFrictionTableGroundMaterialsRel.AddTarget(tarmacMaterialPath);
    summerTireFrictionTableGroundMaterialsRel.AddTarget(gravelMaterialPath);
    summerTireFrictionTable.CreateFrictionValuesAttr().Set(PXR_NS::VtFloatArray({ 0.7f, 0.6f }));

    if (tireFrictionTablePathsOut)
    {
        (*tireFrictionTablePathsOut)[TireFrictionTableId::eWINTER_TIRE] = winterTireFrictionTablePath;
        (*tireFrictionTablePathsOut)[TireFrictionTableId::eSUMMER_TIRE] = summerTireFrictionTablePath;
    }
}


void VehicleFactory::createSceneBasics(
    const PXR_NS::UsdStageRefPtr& stage,
    const UnitScale unitScale,
    const AxesIndices& axes,
    const uint32_t timeStepsPerSecond)
{
    PXR_NS::SdfPath rootPath = getDefaultPrimPath(stage);

    if (axes.up == 0)
    {
        // x-axis as up is not supported
        PXR_NS::UsdGeomSetStageUpAxis(stage, PXR_NS::UsdGeomTokens->z);
    }
    else if (axes.up == 1)
        PXR_NS::UsdGeomSetStageUpAxis(stage, PXR_NS::UsdGeomTokens->y);
    else
        PXR_NS::UsdGeomSetStageUpAxis(stage, PXR_NS::UsdGeomTokens->z);

    PXR_NS::GfVec3f lightPos = PXR_NS::GfVec3f(0.0f);
    lightPos[axes.up] = 11.5f;
    lightPos[axes.forward] = 0.0f;
    lightPos[axes.side] = 6.5f;

    PXR_NS::UsdGeomSetStageMetersPerUnit(stage, 1.0f / unitScale.lengthScale);
    PXR_NS::UsdPhysicsSetStageKilogramsPerUnit(stage, 1.0f / unitScale.massScale);

    //
    // light
    //
    static const PXR_NS::SdfPath sphereLightPathPostfix("SphereLight");
    PXR_NS::SdfPath sphereLightPath = rootPath.AppendPath(sphereLightPathPostfix);
    PXR_NS::UsdLuxSphereLight sphereLight = PXR_NS::UsdLuxSphereLight::Define(stage, sphereLightPath);
    sphereLight.CreateRadiusAttr().Set(unitScale.lengthScale * 1.5f);
    sphereLight.CreateIntensityAttr().Set(30000.0f);
    sphereLight.AddTranslateOp(PXR_NS::UsdGeomXformOp::PrecisionFloat).Set(
        lightPos * unitScale.lengthScale);

    //
    // physics scene
    //
    static const PXR_NS::SdfPath scenePathPostfix("PhysicsScene");
    PXR_NS::SdfPath scenePath = rootPath.AppendPath(scenePathPostfix);
    PXR_NS::UsdPhysicsScene scene = PXR_NS::UsdPhysicsScene::Define(stage, scenePath);
    PXR_NS::UsdPrim scenePrim = scene.GetPrim();
    PXR_NS::GfVec3f gravity = PXR_NS::GfVec3f(0.0f);
    gravity[axes.up] = -1.0f;
    scene.CreateGravityDirectionAttr().Set(gravity);
    scene.CreateGravityMagnitudeAttr().Set(10.0f * unitScale.lengthScale);
    PXR_NS::PhysxSchemaPhysxSceneAPI sceneAPI = PXR_NS::PhysxSchemaPhysxSceneAPI::Apply(scenePrim);
    sceneAPI.CreateTimeStepsPerSecondAttr().Set(timeStepsPerSecond);

    //
    // vehicle context
    //
    const PXR_NS::TfToken verticalAxis = getAxisToken(axes.up);
    const PXR_NS::TfToken longitudinalAxis = getAxisToken(axes.forward);
    PXR_NS::PhysxSchemaPhysxVehicleContextAPI vehicleContextAPI = PXR_NS::PhysxSchemaPhysxVehicleContextAPI::Apply(scenePrim);
    vehicleContextAPI.CreateUpdateModeAttr().Set(PXR_NS::PhysxSchemaTokens->velocityChange);
    vehicleContextAPI.CreateVerticalAxisAttr().Set(verticalAxis);
    vehicleContextAPI.CreateLongitudinalAxisAttr().Set(longitudinalAxis);
}


void VehicleFactory::createGroundPlane(
    const PXR_NS::UsdStageRefPtr& stage,
    const UnitScale unitScale,
    const PXR_NS::SdfPath& planeCollisionGroupPath,
    const PXR_NS::SdfPath& planeMaterialPath,
    const AxesIndices& axes)
{
    PXR_NS::SdfPath rootPath = getDefaultPrimPath(stage);

    static const PXR_NS::SdfPath groundPlanePathPostfix("GroundPlane");
    PXR_NS::SdfPath groundPlanePath = rootPath.AppendPath(groundPlanePathPostfix);
    PXR_NS::UsdGeomMesh groundPlane = PXR_NS::UsdGeomMesh::Define(stage, groundPlanePath);
    groundPlane.AddTranslateOp(PXR_NS::UsdGeomXformOp::PrecisionFloat).Set(PXR_NS::GfVec3f(0.0f) * unitScale.lengthScale);
    groundPlane.AddOrientOp(PXR_NS::UsdGeomXformOp::PrecisionFloat).Set(PXR_NS::GfQuatf(1.0f, 0.0f, 0.0f, 0.0f));
    groundPlane.CreateDisplayColorAttr().Set(PXR_NS::VtArray<PXR_NS::GfVec3f>({PXR_NS::GfVec3f(0.5f, 0.5f, 0.5f)}));

    PXR_NS::VtIntArray faceVertexCounts({ 4 });

    PXR_NS::VtIntArray faceVertexIndices({ 0, 1, 2, 3 });

    PXR_NS::GfVec3f normalsBase[] = {
        PXR_NS::GfVec3f(0.0f, 1.0f, 0.0f),
        PXR_NS::GfVec3f(0.0f, 1.0f, 0.0f),
        PXR_NS::GfVec3f(0.0f, 1.0f, 0.0f),
        PXR_NS::GfVec3f(0.0f, 1.0f, 0.0f)
    };
    const size_t normalCount = sizeof(normalsBase) / sizeof(normalsBase[0]);
    PXR_NS::VtVec3fArray normals;
    normals.resize(normalCount);
    convertGfVec3List<PXR_NS::VtVec3fArray&>(normalsBase, normals, normalCount,
        gYZXAxes, axes);

    PXR_NS::GfVec3f pointsBase[] = {
        PXR_NS::GfVec3f(-15.0f, 0.0f, -15.0f) * unitScale.lengthScale,
        PXR_NS::GfVec3f(15.0f, 0.0f, -15.0f) * unitScale.lengthScale,
        PXR_NS::GfVec3f(15.0f, 0.0f, 15.0f) * unitScale.lengthScale,
        PXR_NS::GfVec3f(-15.0f, 0.0f, 15.0f) * unitScale.lengthScale,
    };
    const size_t pointCount = sizeof(pointsBase) / sizeof(pointsBase[0]);
    PXR_NS::VtVec3fArray points;
    points.resize(pointCount);
    convertGfVec3List<PXR_NS::VtVec3fArray&>(pointsBase, points, pointCount,
        gYZXAxes, axes);

    groundPlane.CreateFaceVertexCountsAttr().Set(faceVertexCounts);
    groundPlane.CreateFaceVertexIndicesAttr().Set(faceVertexIndices);
    groundPlane.CreateNormalsAttr().Set(normals);
    groundPlane.CreatePointsAttr().Set(points);

    static const PXR_NS::SdfPath collisionPlanePathPostfix("CollisionPlane");
    PXR_NS::SdfPath collisionPlanePath = rootPath.AppendPath(collisionPlanePathPostfix);
    PXR_NS::UsdGeomPlane collisionPlane = PXR_NS::UsdGeomPlane::Define(stage, collisionPlanePath);
    collisionPlane.CreatePurposeAttr().Set(PXR_NS::UsdGeomTokens->guide);
    if (axes.up == 0)
        collisionPlane.CreateAxisAttr().Set(PXR_NS::UsdGeomTokens->x);
    else if (axes.up == 1)
        collisionPlane.CreateAxisAttr().Set(PXR_NS::UsdGeomTokens->y);
    else
        collisionPlane.CreateAxisAttr().Set(PXR_NS::UsdGeomTokens->z);

    PXR_NS::UsdPrim collisionPlanePrim = collisionPlane.GetPrim();

    PXR_NS::UsdPhysicsCollisionAPI::Apply(collisionPlanePrim);
    addCollisionToCollisionGroup(stage, collisionPlanePath, planeCollisionGroupPath);
    addPhysicsMaterialToPrim(stage, collisionPlanePrim, planeMaterialPath);
}


static void setUpWheel(
    PXR_NS::UsdPrim& prim,
    const UnitScale unitScale)
{
    const float lengthScaleSqr = unitScale.lengthScale * unitScale.lengthScale;
    const float kgmsScale = unitScale.massScale * lengthScaleSqr;
    
    PXR_NS::PhysxSchemaPhysxVehicleWheelAPI wheelAPI = PXR_NS::PhysxSchemaPhysxVehicleWheelAPI::Apply(prim);
    wheelAPI.CreateRadiusAttr().Set(0.35f * unitScale.lengthScale);
    wheelAPI.CreateWidthAttr().Set(0.15f * unitScale.lengthScale);
    wheelAPI.CreateMassAttr().Set(20.0f * unitScale.massScale);
    wheelAPI.CreateMoiAttr().Set(1.225f * kgmsScale);
    wheelAPI.CreateDampingRateAttr().Set(0.25f * kgmsScale);
}


static void setUpTire(
    PXR_NS::UsdPrim& prim,
    const PXR_NS::SdfPath& tireFrictionTablePath,
    const UnitScale unitScale)
{
    const float forceScale = unitScale.massScale * unitScale.lengthScale;
    const float tireRestLoad = 450 * 10 * forceScale;

    PXR_NS::PhysxSchemaPhysxVehicleTireAPI tireAPI = PXR_NS::PhysxSchemaPhysxVehicleTireAPI::Apply(prim);
    tireAPI.CreateLateralStiffnessGraphAttr().Set(PXR_NS::GfVec2f(2.0f, 17.0f * tireRestLoad));
    tireAPI.CreateLongitudinalStiffnessAttr().Set(5000.0f * forceScale);
    tireAPI.CreateCamberStiffnessAttr().Set(0.0f * forceScale);
    tireAPI.CreateFrictionVsSlipGraphAttr().Set(PXR_NS::VtVec2fArray({ PXR_NS::GfVec2f(0.0f, 1.0f), PXR_NS::GfVec2f(0.1f, 1.0f), PXR_NS::GfVec2f(1.0f, 1.0f) }));
    PXR_NS::UsdRelationship tireFrictionTableRel = tireAPI.CreateFrictionTableRel();
    tireFrictionTableRel.AddTarget(tireFrictionTablePath);
}


static void setUpSuspension(
    PXR_NS::UsdPrim& prim,
    const UnitScale unitScale)
{
    PXR_NS::PhysxSchemaPhysxVehicleSuspensionAPI suspensionAPI = PXR_NS::PhysxSchemaPhysxVehicleSuspensionAPI::Apply(prim);
    suspensionAPI.CreateSpringStrengthAttr().Set(45000.0f * unitScale.massScale);
    suspensionAPI.CreateSpringDamperRateAttr().Set(4500.0f * unitScale.massScale);
    suspensionAPI.CreateTravelDistanceAttr().Set(gTravelDistanceInMeters * unitScale.lengthScale);
}


static void createWheelComponents(
    const PXR_NS::UsdStageRefPtr& stage,
    const UnitScale unitScale,
    const PXR_NS::SdfPath& tireFrictionTablePath,
    PXR_NS::SdfPath& wheelPathOut,
    PXR_NS::SdfPath(&tirePathsOut)[VehicleFactory::TireId::eCOUNT],
    PXR_NS::SdfPath(&suspensionPathsOut)[VehicleFactory::SuspensionId::eCOUNT])
{
    const float lengthScaleSqr = unitScale.lengthScale * unitScale.lengthScale;

    PXR_NS::SdfPath rootPath = getDefaultPrimPath(stage);

    //
    // wheels
    //
    static const PXR_NS::SdfPath wheelPathPostfix("Wheel");
    PXR_NS::SdfPath wheelPath = rootPath.AppendPath(wheelPathPostfix);
    wheelPathOut = wheelPath;
    PXR_NS::UsdPrim wheelPrim = PXR_NS::UsdGeomScope::Define(stage, wheelPath).GetPrim();
    setUpWheel(wheelPrim, unitScale);

    //
    // tires
    //
    static const PXR_NS::SdfPath frontTirePathPostfix("FrontTire");
    PXR_NS::SdfPath frontTirePath = rootPath.AppendPath(frontTirePathPostfix);
    tirePathsOut[VehicleFactory::TireId::eFRONT] = frontTirePath;
    PXR_NS::UsdPrim frontTirePrim = PXR_NS::UsdGeomScope::Define(stage, frontTirePath).GetPrim();
    setUpTire(frontTirePrim, tireFrictionTablePath, unitScale);

    static const PXR_NS::SdfPath rearTirePathPostfix("RearTire");
    PXR_NS::SdfPath rearTirePath = rootPath.AppendPath(rearTirePathPostfix);
    tirePathsOut[VehicleFactory::TireId::eREAR] = rearTirePath;
    PXR_NS::UsdPrim rearTirePrim = PXR_NS::UsdGeomScope::Define(stage, rearTirePath).GetPrim();
    setUpTire(rearTirePrim, tireFrictionTablePath, unitScale);

    //
    // suspensions
    //
    static const PXR_NS::SdfPath frontSuspensionPathPostfix("FrontSuspension");
    PXR_NS::SdfPath frontSuspensionPath = rootPath.AppendPath(frontSuspensionPathPostfix);
    suspensionPathsOut[VehicleFactory::SuspensionId::eFRONT] = frontSuspensionPath;
    PXR_NS::UsdPrim frontSuspensionPrim = PXR_NS::UsdGeomScope::Define(stage, frontSuspensionPath).GetPrim();
    setUpSuspension(frontSuspensionPrim, unitScale);

    static const PXR_NS::SdfPath rearSuspensionPathPostfix("RearSuspension");
    PXR_NS::SdfPath rearSuspensionPath = rootPath.AppendPath(rearSuspensionPathPostfix);
    suspensionPathsOut[VehicleFactory::SuspensionId::eREAR] = rearSuspensionPath;
    PXR_NS::UsdPrim rearSuspensionPrim = PXR_NS::UsdGeomScope::Define(stage, rearSuspensionPath).GetPrim();
    setUpSuspension(rearSuspensionPrim, unitScale);
}


static void setUpEngine(
    PXR_NS::UsdPrim& prim,
    const UnitScale unitScale)
{
    const float lengthScaleSqr = unitScale.lengthScale * unitScale.lengthScale;
    const float kgmsScale = unitScale.massScale * lengthScaleSqr;
    
    PXR_NS::PhysxSchemaPhysxVehicleEngineAPI engineAPI = PXR_NS::PhysxSchemaPhysxVehicleEngineAPI::Apply(prim);
    engineAPI.CreateMoiAttr().Set(1.0f * kgmsScale);
    engineAPI.CreatePeakTorqueAttr().Set(1000.0f * kgmsScale);
    engineAPI.CreateMaxRotationSpeedAttr().Set(600.0f);
    engineAPI.CreateTorqueCurveAttr().Set(PXR_NS::VtVec2fArray({ PXR_NS::GfVec2f(0.0f, 0.8f), PXR_NS::GfVec2f(0.33f, 1.0f), PXR_NS::GfVec2f(1.0f, 0.8f) }));
    engineAPI.CreateDampingRateFullThrottleAttr().Set(0.15f * kgmsScale);
    engineAPI.CreateDampingRateZeroThrottleClutchEngagedAttr().Set(2.0f * kgmsScale);
    engineAPI.CreateDampingRateZeroThrottleClutchDisengagedAttr().Set(0.35f * kgmsScale);
}


static void setUpGears(
    PXR_NS::UsdPrim& prim)
{
    PXR_NS::PhysxSchemaPhysxVehicleGearsAPI gearsAPI = PXR_NS::PhysxSchemaPhysxVehicleGearsAPI::Apply(prim);
    gearsAPI.CreateRatiosAttr().Set(PXR_NS::VtFloatArray({ -4.0f, 4.0f, 2.0f, 1.5f, 1.1f, 1.0f }));
    gearsAPI.CreateRatioScaleAttr().Set(4.0f);
    gearsAPI.CreateSwitchTimeAttr().Set(0.5f);
}


static void setUpAutoGearBox(
    PXR_NS::UsdPrim& prim)
{
    PXR_NS::PhysxSchemaPhysxVehicleAutoGearBoxAPI autoGearBoxAPI = PXR_NS::PhysxSchemaPhysxVehicleAutoGearBoxAPI::Apply(prim);
    autoGearBoxAPI.CreateUpRatiosAttr().Set(PXR_NS::VtFloatArray({ 0.65f, 0.65f, 0.65f, 0.65f }));
    autoGearBoxAPI.CreateDownRatiosAttr().Set(PXR_NS::VtFloatArray({ 0.5f, 0.5f, 0.5f, 0.5f }));
    autoGearBoxAPI.CreateLatencyAttr().Set(2.0f);
}


static void setUpClutch(
    PXR_NS::UsdPrim& prim,
    const UnitScale unitScale)
{
    const float lengthScaleSqr = unitScale.lengthScale * unitScale.lengthScale;
    const float kgmsScale = unitScale.massScale * lengthScaleSqr;
    PXR_NS::PhysxSchemaPhysxVehicleClutchAPI clutchAPI = PXR_NS::PhysxSchemaPhysxVehicleClutchAPI::Apply(prim);
    clutchAPI.CreateStrengthAttr().Set(10.0f * kgmsScale);
}


static void setUpDriveBasic(
    PXR_NS::UsdPrim& prim,
    const UnitScale unitScale)
{
    const float lengthScaleSqr = unitScale.lengthScale * unitScale.lengthScale;
    const float kgmsScale = unitScale.massScale * lengthScaleSqr;
    PXR_NS::PhysxSchemaPhysxVehicleDriveBasicAPI driveAPI = PXR_NS::PhysxSchemaPhysxVehicleDriveBasicAPI::Apply(prim);

    driveAPI.CreatePeakTorqueAttr().Set(500.0f * kgmsScale);
}


static void createDriveComponents(
    const PXR_NS::UsdStageRefPtr& stage,
    const UnitScale unitScale,
    PXR_NS::SdfPath& drivePathOut,
    const VehicleFactory::DriveMode::Enum driveMode = VehicleFactory::DriveMode::eSTANDARD,
    const bool createAutoGearBox = true)
{
    const float lengthScaleSqr = unitScale.lengthScale * unitScale.lengthScale;

    PXR_NS::SdfPath rootPath = getDefaultPrimPath(stage);

    if (driveMode == VehicleFactory::DriveMode::eSTANDARD)
    {
        //
        // engine
        //
        static const PXR_NS::SdfPath enginePathPostfix("Engine");
        PXR_NS::SdfPath enginePath = rootPath.AppendPath(enginePathPostfix);
        PXR_NS::UsdPrim enginePrim = PXR_NS::UsdGeomScope::Define(stage, enginePath).GetPrim();
        setUpEngine(enginePrim, unitScale);

        //
        // gears
        //
        static const PXR_NS::SdfPath gearsPathPostfix("Gears");
        PXR_NS::SdfPath gearsPath = rootPath.AppendPath(gearsPathPostfix);
        PXR_NS::UsdPrim gearsPrim = PXR_NS::UsdGeomScope::Define(stage, gearsPath).GetPrim();
        setUpGears(gearsPrim);

        //
        // auto gear box
        //
        static const PXR_NS::SdfPath autoGearBoxPathPostfix("AutoGearBox");
        PXR_NS::SdfPath autoGearBoxPath = rootPath.AppendPath(autoGearBoxPathPostfix);
        if (createAutoGearBox)
        {
            PXR_NS::UsdPrim autoGearBoxPrim = PXR_NS::UsdGeomScope::Define(stage, autoGearBoxPath).GetPrim();
            setUpAutoGearBox(autoGearBoxPrim);
        }

        //
        // clutch
        //
        static const PXR_NS::SdfPath clutchPathPostfix("Clutch");
        PXR_NS::SdfPath clutchPath = rootPath.AppendPath(clutchPathPostfix);
        PXR_NS::UsdPrim clutchPrim = PXR_NS::UsdGeomScope::Define(stage, clutchPath).GetPrim();
        setUpClutch(clutchPrim, unitScale);

        //
        // drive
        //
        static const PXR_NS::SdfPath drivePathPostfix("DriveStandard");
        PXR_NS::SdfPath drivePath = rootPath.AppendPath(drivePathPostfix);
        drivePathOut = drivePath;
        PXR_NS::UsdPrim drivePrim = PXR_NS::UsdGeomScope::Define(stage, drivePath).GetPrim();
        PXR_NS::PhysxSchemaPhysxVehicleDriveStandardAPI driveAPI = PXR_NS::PhysxSchemaPhysxVehicleDriveStandardAPI::Apply(drivePrim);
        PXR_NS::UsdRelationship engineRel = driveAPI.CreateEngineRel();
        engineRel.AddTarget(enginePath);
        PXR_NS::UsdRelationship gearsRel = driveAPI.CreateGearsRel();
        gearsRel.AddTarget(gearsPath);
        if (createAutoGearBox)
        {
            PXR_NS::UsdRelationship autoGearBoxRel = driveAPI.CreateAutoGearBoxRel();
            autoGearBoxRel.AddTarget(autoGearBoxPath);
        }
        PXR_NS::UsdRelationship clutchRel = driveAPI.CreateClutchRel();
        clutchRel.AddTarget(clutchPath);
    }
    else
    {
        //
        // drive
        //
        static const PXR_NS::SdfPath drivePathPostfix("DriveBasic");
        PXR_NS::SdfPath drivePath = rootPath.AppendPath(drivePathPostfix);
        drivePathOut = drivePath;
        PXR_NS::UsdPrim drivePrim = PXR_NS::UsdGeomScope::Define(stage, drivePath).GetPrim();
        setUpDriveBasic(drivePrim, unitScale);
    }
}


static PXR_NS::UsdGeomMesh createCylinderMeshWithConvexHull(
    const PXR_NS::UsdStageRefPtr& stage,
    const PXR_NS::SdfPath& path,
    const VehicleFactory::AxesIndices& axes = VehicleFactory::AxesIndices())
{
    PXR_NS::UsdGeomMesh mesh = PXR_NS::UsdGeomMesh::Define(stage, path);

    mesh.CreateDoubleSidedAttr().Set(false);

    PXR_NS::VtVec3fArray ringMeshPoints;
    ringMeshPoints.resize(gRingMeshPointCount);
    convertGfVec3List<PXR_NS::VtVec3fArray&>(gRingMeshPoints, ringMeshPoints, gRingMeshPointCount,
        gYZXAxes, axes);
    mesh.CreatePointsAttr().Set(ringMeshPoints);

    PXR_NS::VtIntArray ringMeshFaceVertexIndices;
    ringMeshFaceVertexIndices.assign(gRingMeshFaceVertexIndices, gRingMeshFaceVertexIndices + gRingMeshFaceVertexIndexCount);
    mesh.CreateFaceVertexIndicesAttr().Set(ringMeshFaceVertexIndices);

    PXR_NS::VtIntArray ringMeshFaceVertexCounts;
    ringMeshFaceVertexCounts.assign(gRingMeshFaceVertexCounts, gRingMeshFaceVertexCounts + gRingMeshFaceVertexCountEntryCount);
    mesh.CreateFaceVertexCountsAttr().Set(ringMeshFaceVertexCounts);
    
    PXR_NS::UsdPrim meshPrim = mesh.GetPrim();
    PXR_NS::UsdPhysicsMeshCollisionAPI meshCollisionAPI = PXR_NS::UsdPhysicsMeshCollisionAPI::Apply(meshPrim);
    meshCollisionAPI.CreateApproximationAttr().Set(PXR_NS::UsdPhysicsTokens->convexHull);
    
    return mesh;
}


void VehicleFactory::create4WheeledCar(
    const PXR_NS::UsdStageRefPtr& stage,
    const UnitScale unitScale,
    const DriveMode::Enum driveMode,
    const PXR_NS::SdfPath& vehiclePath,
    const PXR_NS::GfVec3f& vehiclePosition,
    const bool(&wheelIsDriven)[4],
    const PXR_NS::SdfPath& groundQueryCollisionGroupPath,
    const PXR_NS::SdfPath* tireFrictionTablePath,
    const PXR_NS::SdfPath* chassisCollisionGroupPath,
    const PXR_NS::SdfPath* wheelCollisionGroupPath,
    const Car4WheelsParams& params)
{
    const float lengthScaleSqr = unitScale.lengthScale * unitScale.lengthScale;
    const float kgmsScale = unitScale.massScale * lengthScaleSqr;
    const AxesIndices& axes = params.axes;

    PXR_NS::UsdGeomXform vehicleXform = PXR_NS::UsdGeomXform::Define(stage, vehiclePath);
    vehicleXform.AddTranslateOp(PXR_NS::UsdGeomXformOp::PrecisionFloat).Set(vehiclePosition);
    vehicleXform.AddOrientOp(PXR_NS::UsdGeomXformOp::PrecisionFloat).Set(PXR_NS::GfQuatf(1.0f, 0.0f, 0.0f, 0.0f));

    PXR_NS::UsdPrim vehiclePrim = vehicleXform.GetPrim();

    PXR_NS::UsdPhysicsRigidBodyAPI::Apply(vehiclePrim);

    PXR_NS::GfVec3f vehicleMassBoxDim(0.0f);
    vehicleMassBoxDim[axes.up] = 1.0f * unitScale.lengthScale;
    vehicleMassBoxDim[axes.forward] = 4.8f * unitScale.lengthScale;
    vehicleMassBoxDim[axes.side] = 1.8f * unitScale.lengthScale;
    const float centerOfMassToGround = 0.75f * unitScale.lengthScale;
    const float chassisHalfHeight = 0.7f * unitScale.lengthScale;
    const float chassisDistToGround = 0.3f * unitScale.lengthScale;
    const float chassisCenterToGround = chassisHalfHeight + chassisDistToGround;
    const float wheelRadius = 0.35f * unitScale.lengthScale;
    const float wheelWidth = 0.15f * unitScale.lengthScale;
    const float mass = 1800.0f * unitScale.massScale;
    PXR_NS::UsdPhysicsMassAPI massAPI = PXR_NS::UsdPhysicsMassAPI::Apply(vehiclePrim);
    massAPI.CreateMassAttr().Set(mass);
    PXR_NS::GfVec3f centerOfMassOffset(0.0f);
    centerOfMassOffset[axes.up] = centerOfMassToGround - chassisCenterToGround;
    massAPI.CreateCenterOfMassAttr().Set(centerOfMassOffset);
    massAPI.CreateDiagonalInertiaAttr().Set(
        PXR_NS::GfVec3f(
            (vehicleMassBoxDim[1] * vehicleMassBoxDim[1]) + (vehicleMassBoxDim[2] * vehicleMassBoxDim[2]),
            (vehicleMassBoxDim[0] * vehicleMassBoxDim[0]) + (vehicleMassBoxDim[2] * vehicleMassBoxDim[2]),
            (vehicleMassBoxDim[0] * vehicleMassBoxDim[0]) + (vehicleMassBoxDim[1] * vehicleMassBoxDim[1])
        )
        * (1.0f / 12.0f)
        * mass
    );
    massAPI.CreatePrincipalAxesAttr().Set(PXR_NS::GfQuatf(1.0f, 0.0f, 0.0f, 0.0f));

    PXR_NS::PhysxSchemaPhysxRigidBodyAPI rigidBodyAPI = PXR_NS::PhysxSchemaPhysxRigidBodyAPI::Apply(vehiclePrim);
    rigidBodyAPI.CreateSleepThresholdAttr().Set(0.0f);
    rigidBodyAPI.CreateStabilizationThresholdAttr().Set(0.0f);
    rigidBodyAPI.CreateDisableGravityAttr().Set(true);

    PXR_NS::PhysxSchemaPhysxVehicleAPI vehicleAPI = PXR_NS::PhysxSchemaPhysxVehicleAPI::Apply(vehiclePrim);
    vehicleAPI.CreateVehicleEnabledAttr().Set(params.enabled);

    if (params.drivePath)
    {
        PXR_NS::UsdRelationship driveRel = vehicleAPI.CreateDriveRel();
        driveRel.AddTarget(*params.drivePath);
    }
    else if (driveMode == DriveMode::eBASIC)
    {
        setUpDriveBasic(vehiclePrim, unitScale);
    }
    else if (driveMode == DriveMode::eSTANDARD)
    {
        PXR_NS::PhysxSchemaPhysxVehicleDriveStandardAPI::Apply(vehiclePrim);
        setUpEngine(vehiclePrim, unitScale);
        setUpGears(vehiclePrim);
        if (params.createAutoGearBox)
            setUpAutoGearBox(vehiclePrim);
        setUpClutch(vehiclePrim, unitScale);
    }

    vehicleAPI.CreateSubStepThresholdLongitudinalSpeedAttr().Set(5.0f * unitScale.lengthScale),
    vehicleAPI.CreateLowForwardSpeedSubStepCountAttr().Set(3);
    vehicleAPI.CreateHighForwardSpeedSubStepCountAttr().Set(1);
    vehicleAPI.CreateMinPassiveLongitudinalSlipDenominatorAttr().Set(4.0f * unitScale.lengthScale);
    vehicleAPI.CreateMinActiveLongitudinalSlipDenominatorAttr().Set(0.1f * unitScale.lengthScale);
    vehicleAPI.CreateMinLateralSlipDenominatorAttr().Set(1.0f * unitScale.lengthScale);

    if (params.useRaycasts)
        vehicleAPI.CreateSuspensionLineQueryTypeAttr().Set(PXR_NS::PhysxSchemaTokens->raycast);
    else
        vehicleAPI.CreateSuspensionLineQueryTypeAttr().Set(PXR_NS::PhysxSchemaTokens->sweep);

    vehiclePrim.SetMetadataByDictKey(PXR_NS::SdfFieldKeys->CustomData, PXR_NS::PhysxSchemaTokens->referenceFrameIsCenterOfMass, false);

    if ((params.drivePath || (driveMode != DriveMode::eNONE)) and (!params.omitControllers))
    {
        PXR_NS::PhysxSchemaPhysxVehicleControllerAPI vehicleControllerAPI = PXR_NS::PhysxSchemaPhysxVehicleControllerAPI::Apply(vehiclePrim);
        vehicleControllerAPI.CreateAcceleratorAttr().Set(0.0f);
        vehicleControllerAPI.CreateBrake0Attr().Set(0.0f);
        vehicleControllerAPI.CreateBrake1Attr().Set(0.0f);
        vehicleControllerAPI.CreateSteerAttr().Set(0.0f);
        if ((driveMode == DriveMode::eSTANDARD) && params.createAutoGearBox)
        {
            const int gearValue = omni::physx::usdparser::VehicleControllerDesc::automaticGearValue;
            vehicleControllerAPI.CreateTargetGearAttr().Set(gearValue);
        }
        else
            vehicleControllerAPI.CreateTargetGearAttr().Set(1);
    }

    //
    // front left wheel, front right wheel, rear left wheel, rear right wheel
    //
    const uint32_t wheelCount = WheelAttId::eCOUNT;
    static const PXR_NS::SdfPath wheelFLPathPostfix("FrontLeftWheel");
    static const PXR_NS::SdfPath wheelFRPathPostfix("FrontRightWheel");
    static const PXR_NS::SdfPath wheelRLPathPostfix("RearLeftWheel");
    static const PXR_NS::SdfPath wheelRRPathPostfix("RearRightWheel");
    PXR_NS::SdfPath wheelAttachmentPaths[wheelCount] = {
        vehiclePath.AppendPath(wheelFLPathPostfix),
        vehiclePath.AppendPath(wheelFRPathPostfix),
        vehiclePath.AppendPath(wheelRLPathPostfix),
        vehiclePath.AppendPath(wheelRRPathPostfix)
    };

    if (params.wheelAttachmentPathsOut)
    {
        (*params.wheelAttachmentPathsOut)[WheelAttId::eFL] = wheelAttachmentPaths[0];
        (*params.wheelAttachmentPathsOut)[WheelAttId::eFR] = wheelAttachmentPaths[1];
        (*params.wheelAttachmentPathsOut)[WheelAttId::eRL] = wheelAttachmentPaths[2];
        (*params.wheelAttachmentPathsOut)[WheelAttId::eRR] = wheelAttachmentPaths[3];
    }

    const float wheelRestPositionY = wheelRadius - chassisCenterToGround;
    const float longitudinalWheelOffset = 1.6f * unitScale.lengthScale;
    const float lateralWheelOffset = 0.8f * unitScale.lengthScale;
    PXR_NS::GfVec3f wheelPositionsBase[wheelCount] = {
        PXR_NS::GfVec3f(lateralWheelOffset, wheelRestPositionY, longitudinalWheelOffset),
        PXR_NS::GfVec3f(-lateralWheelOffset, wheelRestPositionY, longitudinalWheelOffset),
        PXR_NS::GfVec3f(lateralWheelOffset, wheelRestPositionY, -longitudinalWheelOffset),
        PXR_NS::GfVec3f(-lateralWheelOffset, wheelRestPositionY, -longitudinalWheelOffset),
    };
    PXR_NS::VtVec3fArray wheelPositions;
    wheelPositions.resize(wheelCount);
    convertGfVec3List<PXR_NS::VtVec3fArray&>(wheelPositionsBase, wheelPositions, wheelCount,
        gYZXAxes, axes);

    const float suspensionFramePositionY = wheelRestPositionY + (gMaxCompressionInMeters * unitScale.lengthScale);
    PXR_NS::GfVec3f suspensionFramePositionsBase[wheelCount] = {
        PXR_NS::GfVec3f(lateralWheelOffset, suspensionFramePositionY, longitudinalWheelOffset),
        PXR_NS::GfVec3f(-lateralWheelOffset, suspensionFramePositionY, longitudinalWheelOffset),
        PXR_NS::GfVec3f(lateralWheelOffset, suspensionFramePositionY, -longitudinalWheelOffset),
        PXR_NS::GfVec3f(-lateralWheelOffset, suspensionFramePositionY, -longitudinalWheelOffset),
    };
    PXR_NS::VtVec3fArray suspensionFramePositions;
    suspensionFramePositions.resize(wheelCount);
    convertGfVec3List<PXR_NS::VtVec3fArray&>(suspensionFramePositionsBase, suspensionFramePositions, wheelCount,
        gYZXAxes, axes);

    bool wheelIsFront[wheelCount] = {
        true,
        true,
        false,
        false,
    };

    int drivenWheelIndexList[wheelCount];
    uint32_t drivenWheelCount = 0;

    for (uint32_t i = 0; i < wheelCount; i++)
    {
        const PXR_NS::GfVec3f& wheelPos = wheelPositions[i];
        const PXR_NS::GfVec3f& suspFramePos = suspensionFramePositions[i];
        const PXR_NS::SdfPath& vehicleWheelPath = wheelAttachmentPaths[i];
        
        PXR_NS::UsdGeomXform vehicleWheelXform = PXR_NS::UsdGeomXform::Define(stage, vehicleWheelPath);

        vehicleWheelXform.AddTranslateOp(PXR_NS::UsdGeomXformOp::PrecisionFloat).Set(wheelPos);

        PXR_NS::UsdPrim vehicleWheelPrim = vehicleWheelXform.GetPrim();
        PXR_NS::PhysxSchemaPhysxVehicleWheelAttachmentAPI wheelAttachmentAPI = PXR_NS::PhysxSchemaPhysxVehicleWheelAttachmentAPI::Apply(vehicleWheelPrim);

        if (params.wheelPaths)
        {
            PXR_NS::UsdRelationship wheelRel = wheelAttachmentAPI.CreateWheelRel();
            wheelRel.AddTarget((*params.wheelPaths)[i]);
        }
        else
        {
            setUpWheel(vehicleWheelPrim, unitScale);
        }

        if (params.tirePaths)
        {
            PXR_NS::UsdRelationship tireRel = wheelAttachmentAPI.CreateTireRel();
            tireRel.AddTarget((*params.tirePaths)[i]);
        }
        else
        {
            CARB_ASSERT(tireFrictionTablePath);
            setUpTire(vehicleWheelPrim, *tireFrictionTablePath, unitScale);
        }

        if (params.suspensionPaths)
        {
            PXR_NS::UsdRelationship suspensionRel = wheelAttachmentAPI.CreateSuspensionRel();
            suspensionRel.AddTarget((*params.suspensionPaths)[i]);
        }
        else
        {
            setUpSuspension(vehicleWheelPrim, unitScale);
        }

        PXR_NS::UsdRelationship collisionGroupRel = wheelAttachmentAPI.CreateCollisionGroupRel();
        collisionGroupRel.AddTarget(groundQueryCollisionGroupPath);

        PXR_NS::GfVec3f suspTravelDir(0.0f);
        suspTravelDir[axes.up] = -1.0f;
        wheelAttachmentAPI.CreateSuspensionTravelDirectionAttr().Set(suspTravelDir);

        wheelAttachmentAPI.CreateSuspensionFramePositionAttr().Set(suspFramePos);
        wheelAttachmentAPI.CreateSuspensionFrameOrientationAttr().Set(PXR_NS::GfQuatf(1.0f, 0.0f, 0.0f, 0.0f));

        wheelAttachmentAPI.CreateIndexAttr().Set(static_cast<int>(i));
        if (wheelIsDriven[i])
        {
            drivenWheelIndexList[drivenWheelCount] = static_cast<int>(i);
            drivenWheelCount++;
        }

        PXR_NS::PhysxSchemaPhysxVehicleSuspensionComplianceAPI suspensionComplianceAPI = PXR_NS::PhysxSchemaPhysxVehicleSuspensionComplianceAPI::Apply(vehicleWheelPrim);
        // empty values for some attributes just to make sure they exist for tests that might want to make
        // use of them
        suspensionComplianceAPI.CreateWheelToeAngleAttr().Set(PXR_NS::VtVec2fArray());
        suspensionComplianceAPI.CreateWheelCamberAngleAttr().Set(PXR_NS::VtVec2fArray());
        const PXR_NS::GfVec3f forceAppPoint = wheelPos - suspFramePos;
        const PXR_NS::GfVec4f forceAppPointEntry(0.0, forceAppPoint[0], forceAppPoint[1], forceAppPoint[2]);
        PXR_NS::VtVec4fArray forceAppPointArray;
        forceAppPointArray.push_back(forceAppPointEntry);
        suspensionComplianceAPI.CreateSuspensionForceAppPointAttr().Set(forceAppPointArray);
        suspensionComplianceAPI.CreateTireForceAppPointAttr().Set(forceAppPointArray);

        if ((params.drivePath == nullptr) && (driveMode == DriveMode::eNONE) && (!params.omitControllers))
        {
            PXR_NS::PhysxSchemaPhysxVehicleWheelControllerAPI wheelControllerAPI = PXR_NS::PhysxSchemaPhysxVehicleWheelControllerAPI::Apply(vehicleWheelPrim);
            wheelControllerAPI.CreateDriveTorqueAttr().Set(0.0f);
            wheelControllerAPI.CreateBrakeTorqueAttr().Set(0.0f);
            wheelControllerAPI.CreateSteerAngleAttr().Set(0.0f);
        }

        if (params.createCollisionShapesForWheels)
        {
            CARB_ASSERT(wheelCollisionGroupPath);

            static const PXR_NS::SdfPath vehicleWheelCollPathPostfix("Collision");
            PXR_NS::SdfPath vehicleWheelCollPath = vehicleWheelPath.AppendPath(vehicleWheelCollPathPostfix);
            PXR_NS::UsdPrim collisionGeomPrim;
            if (params.useMeshAsWheelCollisionShape)
            {
                PXR_NS::UsdGeomMesh collisionGeom = createCylinderMeshWithConvexHull(stage, vehicleWheelCollPath, axes);

                PXR_NS::GfVec3f scale(1.0f);
                scale[axes.up] = wheelRadius;
                scale[axes.forward] = wheelRadius;
                scale[axes.side] = wheelWidth * 0.5f;
                collisionGeom.AddScaleOp(PXR_NS::UsdGeomXformOp::PrecisionFloat).Set(scale);

                collisionGeomPrim = collisionGeom.GetPrim();
            }
            else
            {
                PXR_NS::UsdGeomCylinder collisionGeom = PXR_NS::UsdGeomCylinder::Define(stage, vehicleWheelCollPath);

                collisionGeom.CreateHeightAttr().Set(static_cast<double>(wheelWidth));
                collisionGeom.CreateRadiusAttr().Set(static_cast<double>(wheelRadius));

                if (axes.side == 0)
                    collisionGeom.CreateAxisAttr().Set(PXR_NS::UsdGeomTokens->x);
                else if (axes.side == 1)
                    collisionGeom.CreateAxisAttr().Set(PXR_NS::UsdGeomTokens->y);
                else
                    collisionGeom.CreateAxisAttr().Set(PXR_NS::UsdGeomTokens->z);

                // if height or radius is authored, USD expects extent to be authored too
                PXR_NS::VtVec3fArray cylExtent;
                if (PXR_NS::UsdGeomCylinder::ComputeExtentFromPlugins(collisionGeom, 0, &cylExtent))
                    collisionGeom.CreateExtentAttr().Set(cylExtent);

                collisionGeomPrim = collisionGeom.GetPrim();
            }

            PXR_NS::UsdPhysicsCollisionAPI collisionAPI = PXR_NS::UsdPhysicsCollisionAPI::Apply(collisionGeomPrim);
            addCollisionToCollisionGroup(stage, vehicleWheelCollPath, *wheelCollisionGroupPath);

            PXR_NS::PhysxSchemaPhysxCollisionAPI physxCollisionAPI = PXR_NS::PhysxSchemaPhysxCollisionAPI::Apply(collisionGeomPrim);
            physxCollisionAPI.CreateRestOffsetAttr().Set(0.0f * unitScale.lengthScale);
            physxCollisionAPI.CreateContactOffsetAttr().Set(0.02f * unitScale.lengthScale);
        }
    };

    if (driveMode != DriveMode::eNONE)
    {
        // set up one brake configuration that applies to all wheels
        PXR_NS::PhysxSchemaPhysxVehicleBrakesAPI brakes0API = PXR_NS::PhysxSchemaPhysxVehicleBrakesAPI::Apply(
            vehiclePrim, PXR_NS::PhysxSchemaTokens->brakes0);
        brakes0API.CreateMaxBrakeTorqueAttr().Set(3600.0f * kgmsScale);

        // set up a handbrake configuration that applies to the rear wheels only
        PXR_NS::PhysxSchemaPhysxVehicleBrakesAPI brakes1API = PXR_NS::PhysxSchemaPhysxVehicleBrakesAPI::Apply(
            vehiclePrim, PXR_NS::PhysxSchemaTokens->brakes1);
        brakes1API.CreateWheelsAttr().Set(PXR_NS::VtIntArray({ 2, 3 }));
        brakes1API.CreateMaxBrakeTorqueAttr().Set(3000.0f * kgmsScale);

        // set up a steering configuration that applies to the front wheels
        PXR_NS::PhysxSchemaPhysxVehicleSteeringAPI steeringAPI = PXR_NS::PhysxSchemaPhysxVehicleSteeringAPI::Apply(vehiclePrim);
        steeringAPI.CreateWheelsAttr().Set(PXR_NS::VtIntArray({ 0, 1 }));
        steeringAPI.CreateMaxSteerAngleAttr().Set(0.554264f);

        if (drivenWheelCount)
        {
            float ratio = 1.0f / drivenWheelCount;
            PXR_NS::VtFloatArray ratios(drivenWheelCount, ratio);

            PXR_NS::VtIntArray drivenWheelIndexListPxr;
            for (uint32_t j = 0; j < drivenWheelCount; j++)
                drivenWheelIndexListPxr.push_back(drivenWheelIndexList[j]);

            PXR_NS::PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI multiWheelDiffAPI = PXR_NS::PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI::Apply(vehiclePrim);
            multiWheelDiffAPI.CreateWheelsAttr().Set(drivenWheelIndexListPxr);
            multiWheelDiffAPI.CreateTorqueRatiosAttr().Set(ratios);

            if (driveMode == DriveMode::eSTANDARD)
            {
                multiWheelDiffAPI.CreateAverageWheelSpeedRatiosAttr().Set(ratios);
            }
        }
    }

    PXR_NS::GfVec3f chassisHalfExtents(0.0f);
    chassisHalfExtents[axes.up] = chassisHalfHeight;
    chassisHalfExtents[axes.forward] = 2.4f * unitScale.lengthScale;
    chassisHalfExtents[axes.side] = 0.9f * unitScale.lengthScale;
    PXR_NS::GfVec3f chassisOffset(0.0f);
    chassisOffset[axes.up] = 0.0f * unitScale.lengthScale;
    chassisOffset[axes.forward] = 0.0f * unitScale.lengthScale;
    chassisOffset[axes.side] = 0.0f * unitScale.lengthScale;

    //
    // chassis (collision)
    //
    if (params.addChassisCollisionBox)
    {
        CARB_ASSERT(chassisCollisionGroupPath);

        static const PXR_NS::SdfPath vehicleChassisPathPostfix("ChassisCollision");
        PXR_NS::SdfPath vehicleChassisPath = vehiclePath.AppendPath(vehicleChassisPathPostfix);
        PXR_NS::UsdGeomCube vehicleChassis = PXR_NS::UsdGeomCube::Define(stage, vehicleChassisPath);
        vehicleChassis.CreatePurposeAttr().Set(PXR_NS::UsdGeomTokens->guide);
        vehicleChassis.AddTranslateOp(PXR_NS::UsdGeomXformOp::PrecisionFloat).Set(chassisOffset);
        vehicleChassis.AddScaleOp(PXR_NS::UsdGeomXformOp::PrecisionFloat).Set(chassisHalfExtents);

        PXR_NS::UsdPrim vehicleChassisPrim = vehicleChassis.GetPrim();

        PXR_NS::UsdPhysicsCollisionAPI collisionAPI = PXR_NS::UsdPhysicsCollisionAPI::Apply(vehicleChassisPrim);
        addCollisionToCollisionGroup(stage, vehicleChassisPath, *chassisCollisionGroupPath);

        PXR_NS::PhysxSchemaPhysxCollisionAPI physxCollisionAPI = PXR_NS::PhysxSchemaPhysxCollisionAPI::Apply(vehicleChassisPrim);
        physxCollisionAPI.CreateRestOffsetAttr().Set(0.0f * unitScale.lengthScale);
        physxCollisionAPI.CreateContactOffsetAttr().Set(0.02f * unitScale.lengthScale);
    }

    //
    // chassis (render)
    //
    if (params.addChassisRenderMesh)
    {
        static const PXR_NS::SdfPath vehicleChassisPathPostfix("ChassisRender");
        PXR_NS::SdfPath vehicleChassisPath = vehiclePath.AppendPath(vehicleChassisPathPostfix);
        PXR_NS::UsdGeomMesh vehicleChassis = PXR_NS::UsdGeomMesh::Define(stage, vehicleChassisPath);
        vehicleChassis.AddTranslateOp(PXR_NS::UsdGeomXformOp::PrecisionFloat).Set(chassisOffset);
        vehicleChassis.CreateDisplayColorAttr().Set(PXR_NS::VtArray<PXR_NS::GfVec3f>({PXR_NS::GfVec3f(0.2784314f, 0.64705884f, 1.0f)}));

        PXR_NS::VtIntArray faceVertexCounts({
            4, 4, 4, 4, 4, 4
        });

        PXR_NS::VtIntArray faceVertexIndices({
            0, 1, 3, 2, 4, 5, 7, 6, 10, 11, 13, 12, 14, 15, 9, 8, 17, 23, 21, 19, 22, 16, 18, 20
        });

        PXR_NS::GfVec3f normalsBase[] = {
            PXR_NS::GfVec3f(0.0f, 1.0f, 0.0f),
            PXR_NS::GfVec3f(0.0f, 1.0f, 0.0f),
            PXR_NS::GfVec3f(0.0f, 1.0f, 0.0f),
            PXR_NS::GfVec3f(0.0f, 1.0f, 0.0f),
            PXR_NS::GfVec3f(0.0f, -1.0f, 0.0f),
            PXR_NS::GfVec3f(0.0f, -1.0f, 0.0f),
            PXR_NS::GfVec3f(0.0f, -1.0f, 0.0f),
            PXR_NS::GfVec3f(0.0f, -1.0f, 0.0f),
            PXR_NS::GfVec3f(0.0f, 0.0f, -1.0f),
            PXR_NS::GfVec3f(0.0f, 0.0f, -1.0f),
            PXR_NS::GfVec3f(0.0f, 0.0f, 1.0f),
            PXR_NS::GfVec3f(0.0f, 0.0f, 1.0f),
            PXR_NS::GfVec3f(0.0f, 0.0f, 1.0f),
            PXR_NS::GfVec3f(0.0f, 0.0f, 1.0f),
            PXR_NS::GfVec3f(0.0f, 0.0f, -1.0f),
            PXR_NS::GfVec3f(0.0f, 0.0f, -1.0f),
            PXR_NS::GfVec3f(-1.0f, 0.0f, 0.0f),
            PXR_NS::GfVec3f(1.0f, 0.0f, 0.0f),
            PXR_NS::GfVec3f(-1.0f, 0.0f, 0.0f),
            PXR_NS::GfVec3f(1.0f, 0.0f, 0.0f),
            PXR_NS::GfVec3f(-1.0f, 0.0f, 0.0f),
            PXR_NS::GfVec3f(1.0f, 0.0f, 0.0f),
            PXR_NS::GfVec3f(-1.0f, 0.0f, 0.0f),
            PXR_NS::GfVec3f(1.0f, 0.0f, 0.0f),
        };
        static const uint32_t normalCount = sizeof(normalsBase) / sizeof(normalsBase[0]);
        PXR_NS::VtVec3fArray normals;
        normals.resize(normalCount);
        convertGfVec3List<PXR_NS::VtVec3fArray&>(normalsBase, normals, normalCount,
            gYZXAxes, axes);

        chassisHalfExtents[axes.side] = 0.7f * unitScale.lengthScale;  // reduced width to make the wheels easily visible
        PXR_NS::VtVec3fArray points({
            PXR_NS::GfVec3f(-chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
            PXR_NS::GfVec3f(chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
            PXR_NS::GfVec3f(-chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
            PXR_NS::GfVec3f(chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
            PXR_NS::GfVec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
            PXR_NS::GfVec3f(chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
            PXR_NS::GfVec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
            PXR_NS::GfVec3f(chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
            PXR_NS::GfVec3f(-chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
            PXR_NS::GfVec3f(chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
            PXR_NS::GfVec3f(-chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
            PXR_NS::GfVec3f(chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
            PXR_NS::GfVec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
            PXR_NS::GfVec3f(chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
            PXR_NS::GfVec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
            PXR_NS::GfVec3f(chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
            PXR_NS::GfVec3f(-chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
            PXR_NS::GfVec3f(chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
            PXR_NS::GfVec3f(-chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
            PXR_NS::GfVec3f(chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
            PXR_NS::GfVec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
            PXR_NS::GfVec3f(chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
            PXR_NS::GfVec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
            PXR_NS::GfVec3f(chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
        });

        vehicleChassis.CreateFaceVertexCountsAttr().Set(faceVertexCounts);
        vehicleChassis.CreateFaceVertexIndicesAttr().Set(faceVertexIndices);
        vehicleChassis.CreateNormalsAttr().Set(normals);
        vehicleChassis.CreatePointsAttr().Set(points);
    }
}


void VehicleFactory::create4WheeledCarsScenario(
    const PXR_NS::UsdStageRefPtr& stage,
    const UnitScale unitScale,
    const uint32_t vehicleCount,
    const Car4WheelsScenarioParams& params)
{
    Car4WheelsParams carParams;
    carParams.axes = params.axes;
    carParams.createCollisionShapesForWheels = params.createCollisionShapesForWheels;
    carParams.useMeshAsWheelCollisionShape = params.useMeshAsWheelCollisionShape;
    carParams.createAutoGearBox = params.createAutoGearBox;
    carParams.omitControllers = params.omitControllers;
    carParams.addChassisCollisionBox = params.addChassisCollisionBox;
    carParams.addChassisRenderMesh = params.addChassisRenderMesh;

    const AxesIndices& axes = params.axes;
    const uint32_t wheelCount = WheelAttId::eCOUNT;

    PXR_NS::SdfPath rootPath = getDefaultPrimPath(stage);

    createSceneBasics(stage, unitScale, params.axes, params.timeStepsPerSecond);

    PXR_NS::SdfPath collisionGroupPaths[CollGroupId::eCOUNT];
    createCollisionGroups(stage, &collisionGroupPaths);
    if (params.collisionGroupPathsOut)
    {
        for (uint32_t i = 0; i < CollGroupId::eCOUNT; i++)
        {
            (*params.collisionGroupPathsOut)[i] = collisionGroupPaths[i];
        }
    }

    PXR_NS::SdfPath materialPaths[MaterialId::eCOUNT];
    PXR_NS::SdfPath tireFrictionTablePaths[TireFrictionTableId::eCOUNT];
    createMaterialsAndTireFrictionTables(stage, &materialPaths, &tireFrictionTablePaths);
    if (params.tireFrictionTablePathsOut)
    {
        for (uint32_t i = 0; i < TireFrictionTableId::eCOUNT; i++)
        {
            (*params.tireFrictionTablePathsOut)[i] = tireFrictionTablePaths[i];
        }
    }
    
    createGroundPlane(
        stage, unitScale, collisionGroupPaths[CollGroupId::eGROUND_SURFACE],
        materialPaths[MaterialId::eTARMAC], axes);

    bool createShareableComponents;
    if (params.useShareableComponentsList)
    {
        createShareableComponents = false;
        for (uint32_t i = 0; i < vehicleCount; i++)
        {
            if (params.useShareableComponentsList[i])
            {
                createShareableComponents = true;
                break;
            }
        }
    }
    else
        createShareableComponents = true;

    const PXR_NS::SdfPath& tireFrictionTablePath = tireFrictionTablePaths[TireFrictionTableId::eWINTER_TIRE];

    PXR_NS::SdfPath carWheelPaths[wheelCount];
    PXR_NS::SdfPath carTirePaths[wheelCount];
    PXR_NS::SdfPath carSuspensionPaths[wheelCount];
    PXR_NS::SdfPath drivePath;
    if (createShareableComponents)
    {
        PXR_NS::SdfPath wheelPath;
        PXR_NS::SdfPath tirePaths[TireId::eCOUNT];
        PXR_NS::SdfPath suspensionPaths[SuspensionId::eCOUNT];

        createWheelComponents(
            stage,
            unitScale,
            tireFrictionTablePath,
            wheelPath,
            tirePaths,
            suspensionPaths);

        if (params.driveMode != DriveMode::eNONE)
        {
            createDriveComponents(
                stage,
                unitScale,
                drivePath,
                params.driveMode,
                params.createAutoGearBox);
        }

        carWheelPaths[0] = wheelPath;
        carWheelPaths[1] = wheelPath;
        carWheelPaths[2] = wheelPath;
        carWheelPaths[3] = wheelPath;

        carTirePaths[0] = tirePaths[TireId::eFRONT];
        carTirePaths[1] = tirePaths[TireId::eFRONT];
        carTirePaths[2] = tirePaths[TireId::eREAR];
        carTirePaths[3] = tirePaths[TireId::eREAR];

        carSuspensionPaths[0] = suspensionPaths[SuspensionId::eFRONT];
        carSuspensionPaths[1] = suspensionPaths[SuspensionId::eFRONT];
        carSuspensionPaths[2] = suspensionPaths[SuspensionId::eREAR];
        carSuspensionPaths[3] = suspensionPaths[SuspensionId::eREAR];
    }

    bool carWheelIsDriven[] = {false, false, false, false};
    if (params.driveMode != DriveMode::eNONE)
    {
        carWheelIsDriven[0] = true;
        carWheelIsDriven[1] = true;
    }

    PXR_NS::GfVec3f vehiclePositionBase(0.0f);
    vehiclePositionBase[axes.up] = 1.0f * unitScale.lengthScale;
    vehiclePositionBase[axes.forward] = params.basePositionForwardDir;
    vehiclePositionBase[axes.side] = params.basePositionSideDir;

    for (uint32_t i = 0; i < vehicleCount; i++)
    {
        const uint32_t postfixBufferSize = 256;
        char postfixBuffer[postfixBufferSize];
#if CARB_PLATFORM_WINDOWS
#pragma warning(push)
#pragma warning(disable:4996)  // snprintf: This function or variable may be unsafe
#endif
        snprintf(postfixBuffer, postfixBufferSize, "Car_%d", i);
#if CARB_PLATFORM_WINDOWS
#pragma warning(pop)
#endif
        PXR_NS::SdfPath vehiclePath = rootPath.AppendPath(PXR_NS::SdfPath(postfixBuffer));

        if (params.vehiclePathsOut)
            params.vehiclePathsOut[i] = vehiclePath;

        PXR_NS::GfVec3f vehiclePosition(
            vehiclePositionBase[0] + (params.vehicleDelta.x * i),
            vehiclePositionBase[1] + (params.vehicleDelta.y * i),
            vehiclePositionBase[2] + (params.vehicleDelta.z * i));
        
        if (params.vehicleEnabledList)
            carParams.enabled = params.vehicleEnabledList[i];
        else
            carParams.enabled = true;

        if (params.useRaycastsList)
            carParams.useRaycasts = params.useRaycastsList[i];
        else
            carParams.useRaycasts = true;

        bool useShareableComponents;
        if (params.useShareableComponentsList)
            useShareableComponents = params.useShareableComponentsList[i];
        else
            useShareableComponents = true;

        if (useShareableComponents)
        {
            carParams.wheelPaths = &carWheelPaths;
            carParams.tirePaths = &carTirePaths;
            carParams.suspensionPaths = &carSuspensionPaths;
            if (params.driveMode != DriveMode::eNONE)
                carParams.drivePath = &drivePath;
            else
                carParams.drivePath = nullptr;
        }
        else
        {
            carParams.wheelPaths = nullptr;
            carParams.tirePaths = nullptr;
            carParams.suspensionPaths = nullptr;
            carParams.drivePath = nullptr;
        }

        if (params.wheelAttachmentPathsOut)
            carParams.wheelAttachmentPathsOut = &params.wheelAttachmentPathsOut[i];
        else
            carParams.wheelAttachmentPathsOut = nullptr;

        create4WheeledCar(
            stage,
            unitScale,
            params.driveMode,
            vehiclePath,
            vehiclePosition,
            carWheelIsDriven,
            collisionGroupPaths[CollGroupId::eVEHICLE_GROUND_QUERY],
            &tireFrictionTablePath,
            &collisionGroupPaths[CollGroupId::eVEHICLE_CHASSIS],
            &collisionGroupPaths[CollGroupId::eVEHICLE_WHEEL],
            carParams);
    }
}
