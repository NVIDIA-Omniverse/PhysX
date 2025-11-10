// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <private/omni/physx/PhysxUsd.h>

#include "TestVehicleFactory.h"


//
// ring around x-axis
//
static const pxr::GfVec3f gRingMeshPoints[] = {
    pxr::GfVec3f(1.000000f, 0.000000f, -1.000000f), pxr::GfVec3f(-1.000000f, 0.000000f, -1.000000f),
    pxr::GfVec3f(1.000000f, 0.195090f, -0.980785f), pxr::GfVec3f(-1.000000f, 0.195090f, -0.980785f),
    pxr::GfVec3f(1.000000f, 0.382683f, -0.923880f), pxr::GfVec3f(-1.000000f, 0.382683f, -0.923880f),
    pxr::GfVec3f(1.000000f, 0.555570f, -0.831470f), pxr::GfVec3f(-1.000000f, 0.555570f, -0.831470f),
    pxr::GfVec3f(1.000000f, 0.707107f, -0.707107f), pxr::GfVec3f(-1.000000f, 0.707107f, -0.707107f),
    pxr::GfVec3f(1.000000f, 0.831470f, -0.555570f), pxr::GfVec3f(-1.000000f, 0.831470f, -0.555570f),
    pxr::GfVec3f(1.000000f, 0.923880f, -0.382683f), pxr::GfVec3f(-1.000000f, 0.923880f, -0.382683f),
    pxr::GfVec3f(1.000000f, 0.980785f, -0.195090f), pxr::GfVec3f(-1.000000f, 0.980785f, -0.195090f),
    pxr::GfVec3f(1.000000f, 1.000000f, 0.000000f), pxr::GfVec3f(-1.000000f, 1.000000f, 0.000000f),
    pxr::GfVec3f(1.000000f, 0.980785f, 0.195090f), pxr::GfVec3f(-1.000000f, 0.980785f, 0.195090f),
    pxr::GfVec3f(1.000000f, 0.923880f, 0.382684f), pxr::GfVec3f(-1.000000f, 0.923880f, 0.382684f),
    pxr::GfVec3f(1.000000f, 0.831470f, 0.555570f), pxr::GfVec3f(-1.000000f, 0.831470f, 0.555570f),
    pxr::GfVec3f(1.000000f, 0.707107f, 0.707107f), pxr::GfVec3f(-1.000000f, 0.707107f, 0.707107f),
    pxr::GfVec3f(1.000000f, 0.555570f, 0.831470f), pxr::GfVec3f(-1.000000f, 0.555570f, 0.831470f),
    pxr::GfVec3f(1.000000f, 0.382683f, 0.923880f), pxr::GfVec3f(-1.000000f, 0.382683f, 0.923880f),
    pxr::GfVec3f(1.000000f, 0.195090f, 0.980785f), pxr::GfVec3f(-1.000000f, 0.195090f, 0.980785f),
    pxr::GfVec3f(1.000000f, -0.000000f, 1.000000f), pxr::GfVec3f(-1.000000f, -0.000000f, 1.000000f),
    pxr::GfVec3f(1.000000f, -0.195090f, 0.980785f), pxr::GfVec3f(-1.000000f, -0.195090f, 0.980785f),
    pxr::GfVec3f(1.000000f, -0.382683f, 0.923880f), pxr::GfVec3f(-1.000000f, -0.382683f, 0.923880f),
    pxr::GfVec3f(1.000000f, -0.555570f, 0.831470f), pxr::GfVec3f(-1.000000f, -0.555570f, 0.831470f),
    pxr::GfVec3f(1.000000f, -0.707107f, 0.707107f), pxr::GfVec3f(-1.000000f, -0.707107f, 0.707107f),
    pxr::GfVec3f(1.000000f, -0.831470f, 0.555570f), pxr::GfVec3f(-1.000000f, -0.831470f, 0.555570f),
    pxr::GfVec3f(1.000000f, -0.923880f, 0.382683f), pxr::GfVec3f(-1.000000f, -0.923880f, 0.382683f),
    pxr::GfVec3f(1.000000f, -0.980785f, 0.195090f), pxr::GfVec3f(-1.000000f, -0.980785f, 0.195090f),
    pxr::GfVec3f(1.000000f, -1.000000f, -0.000000f), pxr::GfVec3f(-1.000000f, -1.000000f, -0.000000f),
    pxr::GfVec3f(1.000000f, -0.980785f, -0.195090f), pxr::GfVec3f(-1.000000f, -0.980785f, -0.195090f),
    pxr::GfVec3f(1.000000f, -0.923879f, -0.382684f), pxr::GfVec3f(-1.000000f, -0.923879f, -0.382684f),
    pxr::GfVec3f(1.000000f, -0.831469f, -0.555570f), pxr::GfVec3f(-1.000000f, -0.831469f, -0.555570f),
    pxr::GfVec3f(1.000000f, -0.707107f, -0.707107f), pxr::GfVec3f(-1.000000f, -0.707107f, -0.707107f),
    pxr::GfVec3f(1.000000f, -0.555570f, -0.831470f), pxr::GfVec3f(-1.000000f, -0.555570f, -0.831470f),
    pxr::GfVec3f(1.000000f, -0.382683f, -0.923880f), pxr::GfVec3f(-1.000000f, -0.382683f, -0.923880f),
    pxr::GfVec3f(1.000000f, -0.195090f, -0.980785f), pxr::GfVec3f(-1.000000f, -0.195090f, -0.980785f)
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


static pxr::TfToken getAxisToken(const uint32_t axisIndex)
{
    if (axisIndex == 0)
        return pxr::PhysxSchemaTokens.Get()->posX;
    else if (axisIndex == 1)
        return pxr::PhysxSchemaTokens.Get()->posY;
    else
        return pxr::PhysxSchemaTokens.Get()->posZ;
}


template<typename T>
static void convertGfVec3List(const pxr::GfVec3f* pointListSrc, T pointListDst, const uint32_t pointCount,
    const VehicleFactory::AxesIndices& axesIndicesSrc,
    const VehicleFactory::AxesIndices& axesIndicesDst)
{
    for (uint32_t i = 0; i < pointCount; i++)
    {
        const pxr::GfVec3f& pointSrc = pointListSrc[i];
        pxr::GfVec3f& pointDst = pointListDst[i];

        pointDst[axesIndicesDst.up] = pointSrc[axesIndicesSrc.up];
        pointDst[axesIndicesDst.forward] = pointSrc[axesIndicesSrc.forward];
        pointDst[axesIndicesDst.side] = pointSrc[axesIndicesSrc.side];
    }
}


static pxr::SdfPath getDefaultPrimPath(const pxr::UsdStageRefPtr& stage)
{
    pxr::UsdPrim defaultPrim = stage->GetDefaultPrim();
    if (defaultPrim)
        return stage->GetDefaultPrim().GetPath();
    else
        return pxr::SdfPath("/");
}


static void addPhysicsMaterialToPrim(
    const pxr::UsdStageRefPtr& stage,
    pxr::UsdPrim& prim,
    const pxr::SdfPath& materialPath)
{
    pxr::UsdShadeMaterialBindingAPI bindingAPI = pxr::UsdShadeMaterialBindingAPI::Apply(prim);
    pxr::UsdShadeMaterial materialPrim(stage->GetPrimAtPath(materialPath));

    static const pxr::TfToken physicsToken("physics");
    bindingAPI.Bind(materialPrim, pxr::UsdShadeTokens->weakerThanDescendants, physicsToken);
}


static void addCollisionToCollisionGroup(
    const pxr::UsdStageRefPtr& stage,
    const pxr::SdfPath& collisionPrimPath,
    const pxr::SdfPath& collisionGroupPath)
{
    pxr::UsdPrim collisionGroupPrim = stage->GetPrimAtPath(collisionGroupPath);
    if (collisionGroupPrim)
    {
        pxr::UsdCollectionAPI collectionAPI = pxr::UsdCollectionAPI::Get(collisionGroupPrim, pxr::UsdPhysicsTokens->colliders);
        if (collectionAPI)
            collectionAPI.GetIncludesRel().AddTarget(collisionPrimPath);
    }
}


void VehicleFactory::createCollisionGroups(
    const pxr::UsdStageRefPtr& stage,
    pxr::SdfPath(*collisionGroupPathsOut)[CollGroupId::eCOUNT])
{
    pxr::SdfPath rootPath = getDefaultPrimPath(stage);

    static const pxr::SdfPath chassisCollGroupPathPostfix("VehicleChassisCollisionGroup");
    pxr::SdfPath collisionGroupVehicleChassisPath = rootPath.AppendPath(chassisCollGroupPathPostfix);

    static const pxr::SdfPath wheelCollGroupPathPostfix("VehicleWheelCollisionGroup");
    pxr::SdfPath collisionGroupVehicleWheelPath = rootPath.AppendPath(wheelCollGroupPathPostfix);

    static const pxr::SdfPath groundQueryGroupPathPostfix("VehicleGroundQueryGroup");
    pxr::SdfPath collisionGroupVehicleGroundQueryPath = rootPath.AppendPath(groundQueryGroupPathPostfix);

    static const pxr::SdfPath groundSurfaceGroupPathPostfix("GroundSurfaceCollisionGroup");
    pxr::SdfPath collisionGroupGroundSurfacePath = rootPath.AppendPath(groundSurfaceGroupPathPostfix);

    if (collisionGroupPathsOut)
    {
        (*collisionGroupPathsOut)[CollGroupId::eVEHICLE_CHASSIS] = collisionGroupVehicleChassisPath;
        (*collisionGroupPathsOut)[CollGroupId::eVEHICLE_WHEEL] = collisionGroupVehicleWheelPath;
        (*collisionGroupPathsOut)[CollGroupId::eVEHICLE_GROUND_QUERY] = collisionGroupVehicleGroundQueryPath;
        (*collisionGroupPathsOut)[CollGroupId::eGROUND_SURFACE] = collisionGroupGroundSurfacePath;
    }

    pxr::UsdPhysicsCollisionGroup collisionGroupVehicleChassis = pxr::UsdPhysicsCollisionGroup::Define(stage, collisionGroupVehicleChassisPath);
    pxr::UsdRelationship collisionGroupVehicleChassisRel = collisionGroupVehicleChassis.CreateFilteredGroupsRel();
    collisionGroupVehicleChassisRel.AddTarget(collisionGroupVehicleChassisPath);

    pxr::UsdPhysicsCollisionGroup collisionGroupVehicleWheel = pxr::UsdPhysicsCollisionGroup::Define(stage, collisionGroupVehicleWheelPath);
    pxr::UsdRelationship collisionGroupVehicleWheelRel = collisionGroupVehicleWheel.CreateFilteredGroupsRel();
    collisionGroupVehicleWheelRel.AddTarget(collisionGroupVehicleGroundQueryPath);
    collisionGroupVehicleWheelRel.AddTarget(collisionGroupGroundSurfacePath);

    pxr::UsdPhysicsCollisionGroup collisionGroupVehicleGroundQuery = pxr::UsdPhysicsCollisionGroup::Define(stage, collisionGroupVehicleGroundQueryPath);
    pxr::UsdRelationship collisionGroupVehicleGroundQueryRel = collisionGroupVehicleGroundQuery.CreateFilteredGroupsRel();
    collisionGroupVehicleGroundQueryRel.AddTarget(collisionGroupVehicleChassisPath);
    collisionGroupVehicleGroundQueryRel.AddTarget(collisionGroupVehicleWheelPath);

    pxr::UsdPhysicsCollisionGroup collisionGroupGroundSurface = pxr::UsdPhysicsCollisionGroup::Define(stage, collisionGroupGroundSurfacePath);
    pxr::UsdRelationship collisionGroupGroundSurfaceRel = collisionGroupGroundSurface.CreateFilteredGroupsRel();
    collisionGroupGroundSurfaceRel.AddTarget(collisionGroupGroundSurfacePath);
    collisionGroupGroundSurfaceRel.AddTarget(collisionGroupVehicleWheelPath);
}


void  VehicleFactory::createMaterialsAndTireFrictionTables(
    const pxr::UsdStageRefPtr& stage,
    pxr::SdfPath(*materialPathsOut)[MaterialId::eCOUNT],
    pxr::SdfPath(*tireFrictionTablePathsOut)[TireFrictionTableId::eCOUNT])
{
    pxr::SdfPath rootPath = getDefaultPrimPath(stage);

    //
    // materials
    //

    static const pxr::SdfPath tarmacMaterialPathPostfix("TarmacMaterial");
    pxr::SdfPath tarmacMaterialPath = rootPath.AppendPath(tarmacMaterialPathPostfix);
    pxr::UsdShadeMaterial tarmacShadeMaterial = pxr::UsdShadeMaterial::Define(stage, tarmacMaterialPath);
    pxr::UsdPhysicsMaterialAPI tarmacMaterial = pxr::UsdPhysicsMaterialAPI::Apply(tarmacShadeMaterial.GetPrim());
    tarmacMaterial.CreateStaticFrictionAttr().Set(0.9f);
    tarmacMaterial.CreateDynamicFrictionAttr().Set(0.7f);
    tarmacMaterial.CreateRestitutionAttr().Set(0.0f);
    pxr::PhysxSchemaPhysxMaterialAPI::Apply(tarmacMaterial.GetPrim());

    static const pxr::SdfPath gravelMaterialPathPostfix("GravelMaterial");
    pxr::SdfPath gravelMaterialPath = rootPath.AppendPath(gravelMaterialPathPostfix);
    pxr::UsdShadeMaterial gravelShadeMaterial = pxr::UsdShadeMaterial::Define(stage, gravelMaterialPath);
    pxr::UsdPhysicsMaterialAPI gravelMaterial = pxr::UsdPhysicsMaterialAPI::Apply(gravelShadeMaterial.GetPrim());
    gravelMaterial.CreateStaticFrictionAttr().Set(0.6f);
    gravelMaterial.CreateDynamicFrictionAttr().Set(0.6f);
    gravelMaterial.CreateRestitutionAttr().Set(0.0f);
    pxr::PhysxSchemaPhysxMaterialAPI::Apply(gravelMaterial.GetPrim());

    if (materialPathsOut)
    {
        (*materialPathsOut)[MaterialId::eTARMAC] = tarmacMaterialPath;
        (*materialPathsOut)[MaterialId::eGRAVEL] = gravelMaterialPath;
    }

    //
    // tire friction tables
    //

    static const pxr::SdfPath winterTireFrictionTablePathPostfix("WinterTireFrictionTable");
    pxr::SdfPath winterTireFrictionTablePath = rootPath.AppendPath(winterTireFrictionTablePathPostfix);
    pxr::PhysxSchemaPhysxVehicleTireFrictionTable winterTireFrictionTable = pxr::PhysxSchemaPhysxVehicleTireFrictionTable::Define(
        stage, winterTireFrictionTablePath);
    pxr::UsdRelationship winterTireFrictionTableGroundMaterialsRel = winterTireFrictionTable.CreateGroundMaterialsRel();
    winterTireFrictionTableGroundMaterialsRel.AddTarget(tarmacMaterialPath);
    winterTireFrictionTableGroundMaterialsRel.AddTarget(gravelMaterialPath);
    winterTireFrictionTable.CreateFrictionValuesAttr().Set(pxr::VtFloatArray({ 0.75f, 0.6f }));

    static const pxr::SdfPath summerTireFrictionTablePathPostfix("SummerTireFrictionTable");
    pxr::SdfPath summerTireFrictionTablePath = rootPath.AppendPath(summerTireFrictionTablePathPostfix);
    pxr::PhysxSchemaPhysxVehicleTireFrictionTable summerTireFrictionTable = pxr::PhysxSchemaPhysxVehicleTireFrictionTable::Define(
        stage, summerTireFrictionTablePath);
    pxr::UsdRelationship summerTireFrictionTableGroundMaterialsRel = summerTireFrictionTable.CreateGroundMaterialsRel();
    summerTireFrictionTableGroundMaterialsRel.AddTarget(tarmacMaterialPath);
    summerTireFrictionTableGroundMaterialsRel.AddTarget(gravelMaterialPath);
    summerTireFrictionTable.CreateFrictionValuesAttr().Set(pxr::VtFloatArray({ 0.7f, 0.6f }));

    if (tireFrictionTablePathsOut)
    {
        (*tireFrictionTablePathsOut)[TireFrictionTableId::eWINTER_TIRE] = winterTireFrictionTablePath;
        (*tireFrictionTablePathsOut)[TireFrictionTableId::eSUMMER_TIRE] = summerTireFrictionTablePath;
    }
}


void VehicleFactory::createSceneBasics(
    const pxr::UsdStageRefPtr& stage,
    const UnitScale unitScale,
    const AxesIndices& axes,
    const uint32_t timeStepsPerSecond)
{
    pxr::SdfPath rootPath = getDefaultPrimPath(stage);

    if (axes.up == 0)
    {
        // x-axis as up is not supported
        pxr::UsdGeomSetStageUpAxis(stage, pxr::UsdGeomTokens->z);
    }
    else if (axes.up == 1)
        pxr::UsdGeomSetStageUpAxis(stage, pxr::UsdGeomTokens->y);
    else
        pxr::UsdGeomSetStageUpAxis(stage, pxr::UsdGeomTokens->z);

    pxr::GfVec3f lightPos = pxr::GfVec3f(0.0f);
    lightPos[axes.up] = 11.5f;
    lightPos[axes.forward] = 0.0f;
    lightPos[axes.side] = 6.5f;

    pxr::UsdGeomSetStageMetersPerUnit(stage, 1.0f / unitScale.lengthScale);
    pxr::UsdPhysicsSetStageKilogramsPerUnit(stage, 1.0f / unitScale.massScale);

    //
    // light
    //
    static const pxr::SdfPath sphereLightPathPostfix("SphereLight");
    pxr::SdfPath sphereLightPath = rootPath.AppendPath(sphereLightPathPostfix);
    pxr::UsdLuxSphereLight sphereLight = pxr::UsdLuxSphereLight::Define(stage, sphereLightPath);
    sphereLight.CreateRadiusAttr().Set(unitScale.lengthScale * 1.5f);
    sphereLight.CreateIntensityAttr().Set(30000.0f);
    sphereLight.AddTranslateOp(pxr::UsdGeomXformOp::PrecisionFloat).Set(
        lightPos * unitScale.lengthScale);

    //
    // physics scene
    //
    static const pxr::SdfPath scenePathPostfix("PhysicsScene");
    pxr::SdfPath scenePath = rootPath.AppendPath(scenePathPostfix);
    pxr::UsdPhysicsScene scene = pxr::UsdPhysicsScene::Define(stage, scenePath);
    pxr::UsdPrim scenePrim = scene.GetPrim();
    pxr::GfVec3f gravity = pxr::GfVec3f(0.0f);
    gravity[axes.up] = -1.0f;
    scene.CreateGravityDirectionAttr().Set(gravity);
    scene.CreateGravityMagnitudeAttr().Set(10.0f * unitScale.lengthScale);
    pxr::PhysxSchemaPhysxSceneAPI sceneAPI = pxr::PhysxSchemaPhysxSceneAPI::Apply(scenePrim);
    sceneAPI.CreateTimeStepsPerSecondAttr().Set(timeStepsPerSecond);

    //
    // vehicle context
    //
    const pxr::TfToken verticalAxis = getAxisToken(axes.up);
    const pxr::TfToken longitudinalAxis = getAxisToken(axes.forward);
    pxr::PhysxSchemaPhysxVehicleContextAPI vehicleContextAPI = pxr::PhysxSchemaPhysxVehicleContextAPI::Apply(scenePrim);
    vehicleContextAPI.CreateUpdateModeAttr().Set(pxr::PhysxSchemaTokens->velocityChange);
    vehicleContextAPI.CreateVerticalAxisAttr().Set(verticalAxis);
    vehicleContextAPI.CreateLongitudinalAxisAttr().Set(longitudinalAxis);
}


void VehicleFactory::createGroundPlane(
    const pxr::UsdStageRefPtr& stage,
    const UnitScale unitScale,
    const pxr::SdfPath& planeCollisionGroupPath,
    const pxr::SdfPath& planeMaterialPath,
    const AxesIndices& axes)
{
    pxr::SdfPath rootPath = getDefaultPrimPath(stage);

    static const pxr::SdfPath groundPlanePathPostfix("GroundPlane");
    pxr::SdfPath groundPlanePath = rootPath.AppendPath(groundPlanePathPostfix);
    pxr::UsdGeomMesh groundPlane = pxr::UsdGeomMesh::Define(stage, groundPlanePath);
    groundPlane.AddTranslateOp(pxr::UsdGeomXformOp::PrecisionFloat).Set(pxr::GfVec3f(0.0f) * unitScale.lengthScale);
    groundPlane.AddOrientOp(pxr::UsdGeomXformOp::PrecisionFloat).Set(pxr::GfQuatf(1.0f, 0.0f, 0.0f, 0.0f));
    groundPlane.CreateDisplayColorAttr().Set(pxr::VtArray<pxr::GfVec3f>({pxr::GfVec3f(0.5f, 0.5f, 0.5f)}));

    pxr::VtIntArray faceVertexCounts({ 4 });

    pxr::VtIntArray faceVertexIndices({ 0, 1, 2, 3 });

    pxr::GfVec3f normalsBase[] = {
        pxr::GfVec3f(0.0f, 1.0f, 0.0f),
        pxr::GfVec3f(0.0f, 1.0f, 0.0f),
        pxr::GfVec3f(0.0f, 1.0f, 0.0f),
        pxr::GfVec3f(0.0f, 1.0f, 0.0f)
    };
    const size_t normalCount = sizeof(normalsBase) / sizeof(normalsBase[0]);
    pxr::VtVec3fArray normals;
    normals.resize(normalCount);
    convertGfVec3List<pxr::VtVec3fArray&>(normalsBase, normals, normalCount,
        gYZXAxes, axes);

    pxr::GfVec3f pointsBase[] = {
        pxr::GfVec3f(-15.0f, 0.0f, -15.0f) * unitScale.lengthScale,
        pxr::GfVec3f(15.0f, 0.0f, -15.0f) * unitScale.lengthScale,
        pxr::GfVec3f(15.0f, 0.0f, 15.0f) * unitScale.lengthScale,
        pxr::GfVec3f(-15.0f, 0.0f, 15.0f) * unitScale.lengthScale,
    };
    const size_t pointCount = sizeof(pointsBase) / sizeof(pointsBase[0]);
    pxr::VtVec3fArray points;
    points.resize(pointCount);
    convertGfVec3List<pxr::VtVec3fArray&>(pointsBase, points, pointCount,
        gYZXAxes, axes);

    groundPlane.CreateFaceVertexCountsAttr().Set(faceVertexCounts);
    groundPlane.CreateFaceVertexIndicesAttr().Set(faceVertexIndices);
    groundPlane.CreateNormalsAttr().Set(normals);
    groundPlane.CreatePointsAttr().Set(points);

    static const pxr::SdfPath collisionPlanePathPostfix("CollisionPlane");
    pxr::SdfPath collisionPlanePath = rootPath.AppendPath(collisionPlanePathPostfix);
    pxr::UsdGeomPlane collisionPlane = pxr::UsdGeomPlane::Define(stage, collisionPlanePath);
    collisionPlane.CreatePurposeAttr().Set(pxr::UsdGeomTokens->guide);
    if (axes.up == 0)
        collisionPlane.CreateAxisAttr().Set(pxr::UsdGeomTokens->x);
    else if (axes.up == 1)
        collisionPlane.CreateAxisAttr().Set(pxr::UsdGeomTokens->y);
    else
        collisionPlane.CreateAxisAttr().Set(pxr::UsdGeomTokens->z);

    pxr::UsdPrim collisionPlanePrim = collisionPlane.GetPrim();

    pxr::UsdPhysicsCollisionAPI::Apply(collisionPlanePrim);
    addCollisionToCollisionGroup(stage, collisionPlanePath, planeCollisionGroupPath);
    addPhysicsMaterialToPrim(stage, collisionPlanePrim, planeMaterialPath);
}


static void setUpWheel(
    pxr::UsdPrim& prim,
    const UnitScale unitScale)
{
    const float lengthScaleSqr = unitScale.lengthScale * unitScale.lengthScale;
    const float kgmsScale = unitScale.massScale * lengthScaleSqr;
    
    pxr::PhysxSchemaPhysxVehicleWheelAPI wheelAPI = pxr::PhysxSchemaPhysxVehicleWheelAPI::Apply(prim);
    wheelAPI.CreateRadiusAttr().Set(0.35f * unitScale.lengthScale);
    wheelAPI.CreateWidthAttr().Set(0.15f * unitScale.lengthScale);
    wheelAPI.CreateMassAttr().Set(20.0f * unitScale.massScale);
    wheelAPI.CreateMoiAttr().Set(1.225f * kgmsScale);
    wheelAPI.CreateDampingRateAttr().Set(0.25f * kgmsScale);
}


static void setUpTire(
    pxr::UsdPrim& prim,
    const pxr::SdfPath& tireFrictionTablePath,
    const UnitScale unitScale)
{
    const float forceScale = unitScale.massScale * unitScale.lengthScale;
    const float tireRestLoad = 450 * 10 * forceScale;

    pxr::PhysxSchemaPhysxVehicleTireAPI tireAPI = pxr::PhysxSchemaPhysxVehicleTireAPI::Apply(prim);
    tireAPI.CreateLateralStiffnessGraphAttr().Set(pxr::GfVec2f(2.0f, 17.0f * tireRestLoad));
    tireAPI.CreateLongitudinalStiffnessAttr().Set(5000.0f * forceScale);
    tireAPI.CreateCamberStiffnessAttr().Set(0.0f * forceScale);
    tireAPI.CreateFrictionVsSlipGraphAttr().Set(pxr::VtVec2fArray({ pxr::GfVec2f(0.0f, 1.0f), pxr::GfVec2f(0.1f, 1.0f), pxr::GfVec2f(1.0f, 1.0f) }));
    pxr::UsdRelationship tireFrictionTableRel = tireAPI.CreateFrictionTableRel();
    tireFrictionTableRel.AddTarget(tireFrictionTablePath);
}


static void setUpSuspension(
    pxr::UsdPrim& prim,
    const UnitScale unitScale)
{
    pxr::PhysxSchemaPhysxVehicleSuspensionAPI suspensionAPI = pxr::PhysxSchemaPhysxVehicleSuspensionAPI::Apply(prim);
    suspensionAPI.CreateSpringStrengthAttr().Set(45000.0f * unitScale.massScale);
    suspensionAPI.CreateSpringDamperRateAttr().Set(4500.0f * unitScale.massScale);
    suspensionAPI.CreateTravelDistanceAttr().Set(gTravelDistanceInMeters * unitScale.lengthScale);
}


static void createWheelComponents(
    const pxr::UsdStageRefPtr& stage,
    const UnitScale unitScale,
    const pxr::SdfPath& tireFrictionTablePath,
    pxr::SdfPath& wheelPathOut,
    pxr::SdfPath(&tirePathsOut)[VehicleFactory::TireId::eCOUNT],
    pxr::SdfPath(&suspensionPathsOut)[VehicleFactory::SuspensionId::eCOUNT])
{
    const float lengthScaleSqr = unitScale.lengthScale * unitScale.lengthScale;

    pxr::SdfPath rootPath = getDefaultPrimPath(stage);

    //
    // wheels
    //
    static const pxr::SdfPath wheelPathPostfix("Wheel");
    pxr::SdfPath wheelPath = rootPath.AppendPath(wheelPathPostfix);
    wheelPathOut = wheelPath;
    pxr::UsdPrim wheelPrim = pxr::UsdGeomScope::Define(stage, wheelPath).GetPrim();
    setUpWheel(wheelPrim, unitScale);

    //
    // tires
    //
    static const pxr::SdfPath frontTirePathPostfix("FrontTire");
    pxr::SdfPath frontTirePath = rootPath.AppendPath(frontTirePathPostfix);
    tirePathsOut[VehicleFactory::TireId::eFRONT] = frontTirePath;
    pxr::UsdPrim frontTirePrim = pxr::UsdGeomScope::Define(stage, frontTirePath).GetPrim();
    setUpTire(frontTirePrim, tireFrictionTablePath, unitScale);

    static const pxr::SdfPath rearTirePathPostfix("RearTire");
    pxr::SdfPath rearTirePath = rootPath.AppendPath(rearTirePathPostfix);
    tirePathsOut[VehicleFactory::TireId::eREAR] = rearTirePath;
    pxr::UsdPrim rearTirePrim = pxr::UsdGeomScope::Define(stage, rearTirePath).GetPrim();
    setUpTire(rearTirePrim, tireFrictionTablePath, unitScale);

    //
    // suspensions
    //
    static const pxr::SdfPath frontSuspensionPathPostfix("FrontSuspension");
    pxr::SdfPath frontSuspensionPath = rootPath.AppendPath(frontSuspensionPathPostfix);
    suspensionPathsOut[VehicleFactory::SuspensionId::eFRONT] = frontSuspensionPath;
    pxr::UsdPrim frontSuspensionPrim = pxr::UsdGeomScope::Define(stage, frontSuspensionPath).GetPrim();
    setUpSuspension(frontSuspensionPrim, unitScale);

    static const pxr::SdfPath rearSuspensionPathPostfix("RearSuspension");
    pxr::SdfPath rearSuspensionPath = rootPath.AppendPath(rearSuspensionPathPostfix);
    suspensionPathsOut[VehicleFactory::SuspensionId::eREAR] = rearSuspensionPath;
    pxr::UsdPrim rearSuspensionPrim = pxr::UsdGeomScope::Define(stage, rearSuspensionPath).GetPrim();
    setUpSuspension(rearSuspensionPrim, unitScale);
}


static void setUpEngine(
    pxr::UsdPrim& prim,
    const UnitScale unitScale)
{
    const float lengthScaleSqr = unitScale.lengthScale * unitScale.lengthScale;
    const float kgmsScale = unitScale.massScale * lengthScaleSqr;
    
    pxr::PhysxSchemaPhysxVehicleEngineAPI engineAPI = pxr::PhysxSchemaPhysxVehicleEngineAPI::Apply(prim);
    engineAPI.CreateMoiAttr().Set(1.0f * kgmsScale);
    engineAPI.CreatePeakTorqueAttr().Set(1000.0f * kgmsScale);
    engineAPI.CreateMaxRotationSpeedAttr().Set(600.0f);
    engineAPI.CreateTorqueCurveAttr().Set(pxr::VtVec2fArray({ pxr::GfVec2f(0.0f, 0.8f), pxr::GfVec2f(0.33f, 1.0f), pxr::GfVec2f(1.0f, 0.8f) }));
    engineAPI.CreateDampingRateFullThrottleAttr().Set(0.15f * kgmsScale);
    engineAPI.CreateDampingRateZeroThrottleClutchEngagedAttr().Set(2.0f * kgmsScale);
    engineAPI.CreateDampingRateZeroThrottleClutchDisengagedAttr().Set(0.35f * kgmsScale);
}


static void setUpGears(
    pxr::UsdPrim& prim)
{
    pxr::PhysxSchemaPhysxVehicleGearsAPI gearsAPI = pxr::PhysxSchemaPhysxVehicleGearsAPI::Apply(prim);
    gearsAPI.CreateRatiosAttr().Set(pxr::VtFloatArray({ -4.0f, 4.0f, 2.0f, 1.5f, 1.1f, 1.0f }));
    gearsAPI.CreateRatioScaleAttr().Set(4.0f);
    gearsAPI.CreateSwitchTimeAttr().Set(0.5f);
}


static void setUpAutoGearBox(
    pxr::UsdPrim& prim)
{
    pxr::PhysxSchemaPhysxVehicleAutoGearBoxAPI autoGearBoxAPI = pxr::PhysxSchemaPhysxVehicleAutoGearBoxAPI::Apply(prim);
    autoGearBoxAPI.CreateUpRatiosAttr().Set(pxr::VtFloatArray({ 0.65f, 0.65f, 0.65f, 0.65f }));
    autoGearBoxAPI.CreateDownRatiosAttr().Set(pxr::VtFloatArray({ 0.5f, 0.5f, 0.5f, 0.5f }));
    autoGearBoxAPI.CreateLatencyAttr().Set(2.0f);
}


static void setUpClutch(
    pxr::UsdPrim& prim,
    const UnitScale unitScale)
{
    const float lengthScaleSqr = unitScale.lengthScale * unitScale.lengthScale;
    const float kgmsScale = unitScale.massScale * lengthScaleSqr;
    pxr::PhysxSchemaPhysxVehicleClutchAPI clutchAPI = pxr::PhysxSchemaPhysxVehicleClutchAPI::Apply(prim);
    clutchAPI.CreateStrengthAttr().Set(10.0f * kgmsScale);
}


static void setUpDriveBasic(
    pxr::UsdPrim& prim,
    const UnitScale unitScale)
{
    const float lengthScaleSqr = unitScale.lengthScale * unitScale.lengthScale;
    const float kgmsScale = unitScale.massScale * lengthScaleSqr;
    pxr::PhysxSchemaPhysxVehicleDriveBasicAPI driveAPI = pxr::PhysxSchemaPhysxVehicleDriveBasicAPI::Apply(prim);

    driveAPI.CreatePeakTorqueAttr().Set(500.0f * kgmsScale);
}


static void createDriveComponents(
    const pxr::UsdStageRefPtr& stage,
    const UnitScale unitScale,
    pxr::SdfPath& drivePathOut,
    const VehicleFactory::DriveMode::Enum driveMode = VehicleFactory::DriveMode::eSTANDARD,
    const bool createAutoGearBox = true)
{
    const float lengthScaleSqr = unitScale.lengthScale * unitScale.lengthScale;

    pxr::SdfPath rootPath = getDefaultPrimPath(stage);

    if (driveMode == VehicleFactory::DriveMode::eSTANDARD)
    {
        //
        // engine
        //
        static const pxr::SdfPath enginePathPostfix("Engine");
        pxr::SdfPath enginePath = rootPath.AppendPath(enginePathPostfix);
        pxr::UsdPrim enginePrim = pxr::UsdGeomScope::Define(stage, enginePath).GetPrim();
        setUpEngine(enginePrim, unitScale);

        //
        // gears
        //
        static const pxr::SdfPath gearsPathPostfix("Gears");
        pxr::SdfPath gearsPath = rootPath.AppendPath(gearsPathPostfix);
        pxr::UsdPrim gearsPrim = pxr::UsdGeomScope::Define(stage, gearsPath).GetPrim();
        setUpGears(gearsPrim);

        //
        // auto gear box
        //
        static const pxr::SdfPath autoGearBoxPathPostfix("AutoGearBox");
        pxr::SdfPath autoGearBoxPath = rootPath.AppendPath(autoGearBoxPathPostfix);
        if (createAutoGearBox)
        {
            pxr::UsdPrim autoGearBoxPrim = pxr::UsdGeomScope::Define(stage, autoGearBoxPath).GetPrim();
            setUpAutoGearBox(autoGearBoxPrim);
        }

        //
        // clutch
        //
        static const pxr::SdfPath clutchPathPostfix("Clutch");
        pxr::SdfPath clutchPath = rootPath.AppendPath(clutchPathPostfix);
        pxr::UsdPrim clutchPrim = pxr::UsdGeomScope::Define(stage, clutchPath).GetPrim();
        setUpClutch(clutchPrim, unitScale);

        //
        // drive
        //
        static const pxr::SdfPath drivePathPostfix("DriveStandard");
        pxr::SdfPath drivePath = rootPath.AppendPath(drivePathPostfix);
        drivePathOut = drivePath;
        pxr::UsdPrim drivePrim = pxr::UsdGeomScope::Define(stage, drivePath).GetPrim();
        pxr::PhysxSchemaPhysxVehicleDriveStandardAPI driveAPI = pxr::PhysxSchemaPhysxVehicleDriveStandardAPI::Apply(drivePrim);
        pxr::UsdRelationship engineRel = driveAPI.CreateEngineRel();
        engineRel.AddTarget(enginePath);
        pxr::UsdRelationship gearsRel = driveAPI.CreateGearsRel();
        gearsRel.AddTarget(gearsPath);
        if (createAutoGearBox)
        {
            pxr::UsdRelationship autoGearBoxRel = driveAPI.CreateAutoGearBoxRel();
            autoGearBoxRel.AddTarget(autoGearBoxPath);
        }
        pxr::UsdRelationship clutchRel = driveAPI.CreateClutchRel();
        clutchRel.AddTarget(clutchPath);
    }
    else
    {
        //
        // drive
        //
        static const pxr::SdfPath drivePathPostfix("DriveBasic");
        pxr::SdfPath drivePath = rootPath.AppendPath(drivePathPostfix);
        drivePathOut = drivePath;
        pxr::UsdPrim drivePrim = pxr::UsdGeomScope::Define(stage, drivePath).GetPrim();
        setUpDriveBasic(drivePrim, unitScale);
    }
}


static pxr::UsdGeomMesh createCylinderMeshWithConvexHull(
    const pxr::UsdStageRefPtr& stage,
    const pxr::SdfPath& path,
    const VehicleFactory::AxesIndices& axes = VehicleFactory::AxesIndices())
{
    pxr::UsdGeomMesh mesh = pxr::UsdGeomMesh::Define(stage, path);

    mesh.CreateDoubleSidedAttr().Set(false);

    pxr::VtVec3fArray ringMeshPoints;
    ringMeshPoints.resize(gRingMeshPointCount);
    convertGfVec3List<pxr::VtVec3fArray&>(gRingMeshPoints, ringMeshPoints, gRingMeshPointCount,
        gYZXAxes, axes);
    mesh.CreatePointsAttr().Set(ringMeshPoints);

    pxr::VtIntArray ringMeshFaceVertexIndices;
    ringMeshFaceVertexIndices.assign(gRingMeshFaceVertexIndices, gRingMeshFaceVertexIndices + gRingMeshFaceVertexIndexCount);
    mesh.CreateFaceVertexIndicesAttr().Set(ringMeshFaceVertexIndices);

    pxr::VtIntArray ringMeshFaceVertexCounts;
    ringMeshFaceVertexCounts.assign(gRingMeshFaceVertexCounts, gRingMeshFaceVertexCounts + gRingMeshFaceVertexCountEntryCount);
    mesh.CreateFaceVertexCountsAttr().Set(ringMeshFaceVertexCounts);
    
    pxr::UsdPrim meshPrim = mesh.GetPrim();
    pxr::UsdPhysicsMeshCollisionAPI meshCollisionAPI = pxr::UsdPhysicsMeshCollisionAPI::Apply(meshPrim);
    meshCollisionAPI.CreateApproximationAttr().Set(pxr::UsdPhysicsTokens->convexHull);
    
    return mesh;
}


void VehicleFactory::create4WheeledCar(
    const pxr::UsdStageRefPtr& stage,
    const UnitScale unitScale,
    const DriveMode::Enum driveMode,
    const pxr::SdfPath& vehiclePath,
    const pxr::GfVec3f& vehiclePosition,
    const bool(&wheelIsDriven)[4],
    const pxr::SdfPath& groundQueryCollisionGroupPath,
    const pxr::SdfPath* tireFrictionTablePath,
    const pxr::SdfPath* chassisCollisionGroupPath,
    const pxr::SdfPath* wheelCollisionGroupPath,
    const Car4WheelsParams& params)
{
    const float lengthScaleSqr = unitScale.lengthScale * unitScale.lengthScale;
    const float kgmsScale = unitScale.massScale * lengthScaleSqr;
    const AxesIndices& axes = params.axes;

    pxr::UsdGeomXform vehicleXform = pxr::UsdGeomXform::Define(stage, vehiclePath);
    vehicleXform.AddTranslateOp(pxr::UsdGeomXformOp::PrecisionFloat).Set(vehiclePosition);
    vehicleXform.AddOrientOp(pxr::UsdGeomXformOp::PrecisionFloat).Set(pxr::GfQuatf(1.0f, 0.0f, 0.0f, 0.0f));

    pxr::UsdPrim vehiclePrim = vehicleXform.GetPrim();

    pxr::UsdPhysicsRigidBodyAPI::Apply(vehiclePrim);

    pxr::GfVec3f vehicleMassBoxDim(0.0f);
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
    pxr::UsdPhysicsMassAPI massAPI = pxr::UsdPhysicsMassAPI::Apply(vehiclePrim);
    massAPI.CreateMassAttr().Set(mass);
    pxr::GfVec3f centerOfMassOffset(0.0f);
    centerOfMassOffset[axes.up] = centerOfMassToGround - chassisCenterToGround;
    massAPI.CreateCenterOfMassAttr().Set(centerOfMassOffset);
    massAPI.CreateDiagonalInertiaAttr().Set(
        pxr::GfVec3f(
            (vehicleMassBoxDim[1] * vehicleMassBoxDim[1]) + (vehicleMassBoxDim[2] * vehicleMassBoxDim[2]),
            (vehicleMassBoxDim[0] * vehicleMassBoxDim[0]) + (vehicleMassBoxDim[2] * vehicleMassBoxDim[2]),
            (vehicleMassBoxDim[0] * vehicleMassBoxDim[0]) + (vehicleMassBoxDim[1] * vehicleMassBoxDim[1])
        )
        * (1.0f / 12.0f)
        * mass
    );
    massAPI.CreatePrincipalAxesAttr().Set(pxr::GfQuatf(1.0f, 0.0f, 0.0f, 0.0f));

    pxr::PhysxSchemaPhysxRigidBodyAPI rigidBodyAPI = pxr::PhysxSchemaPhysxRigidBodyAPI::Apply(vehiclePrim);
    rigidBodyAPI.CreateSleepThresholdAttr().Set(0.0f);
    rigidBodyAPI.CreateStabilizationThresholdAttr().Set(0.0f);
    rigidBodyAPI.CreateDisableGravityAttr().Set(true);

    pxr::PhysxSchemaPhysxVehicleAPI vehicleAPI = pxr::PhysxSchemaPhysxVehicleAPI::Apply(vehiclePrim);
    vehicleAPI.CreateVehicleEnabledAttr().Set(params.enabled);

    if (params.drivePath)
    {
        pxr::UsdRelationship driveRel = vehicleAPI.CreateDriveRel();
        driveRel.AddTarget(*params.drivePath);
    }
    else if (driveMode == DriveMode::eBASIC)
    {
        setUpDriveBasic(vehiclePrim, unitScale);
    }
    else if (driveMode == DriveMode::eSTANDARD)
    {
        pxr::PhysxSchemaPhysxVehicleDriveStandardAPI::Apply(vehiclePrim);
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
        vehicleAPI.CreateSuspensionLineQueryTypeAttr().Set(pxr::PhysxSchemaTokens->raycast);
    else
        vehicleAPI.CreateSuspensionLineQueryTypeAttr().Set(pxr::PhysxSchemaTokens->sweep);

    vehiclePrim.SetMetadataByDictKey(pxr::SdfFieldKeys->CustomData, pxr::PhysxSchemaTokens->referenceFrameIsCenterOfMass, false);

    if ((params.drivePath || (driveMode != DriveMode::eNONE)) and (!params.omitControllers))
    {
        pxr::PhysxSchemaPhysxVehicleControllerAPI vehicleControllerAPI = pxr::PhysxSchemaPhysxVehicleControllerAPI::Apply(vehiclePrim);
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
    static const pxr::SdfPath wheelFLPathPostfix("FrontLeftWheel");
    static const pxr::SdfPath wheelFRPathPostfix("FrontRightWheel");
    static const pxr::SdfPath wheelRLPathPostfix("RearLeftWheel");
    static const pxr::SdfPath wheelRRPathPostfix("RearRightWheel");
    pxr::SdfPath wheelAttachmentPaths[wheelCount] = {
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
    pxr::GfVec3f wheelPositionsBase[wheelCount] = {
        pxr::GfVec3f(lateralWheelOffset, wheelRestPositionY, longitudinalWheelOffset),
        pxr::GfVec3f(-lateralWheelOffset, wheelRestPositionY, longitudinalWheelOffset),
        pxr::GfVec3f(lateralWheelOffset, wheelRestPositionY, -longitudinalWheelOffset),
        pxr::GfVec3f(-lateralWheelOffset, wheelRestPositionY, -longitudinalWheelOffset),
    };
    pxr::VtVec3fArray wheelPositions;
    wheelPositions.resize(wheelCount);
    convertGfVec3List<pxr::VtVec3fArray&>(wheelPositionsBase, wheelPositions, wheelCount,
        gYZXAxes, axes);

    const float suspensionFramePositionY = wheelRestPositionY + (gMaxCompressionInMeters * unitScale.lengthScale);
    pxr::GfVec3f suspensionFramePositionsBase[wheelCount] = {
        pxr::GfVec3f(lateralWheelOffset, suspensionFramePositionY, longitudinalWheelOffset),
        pxr::GfVec3f(-lateralWheelOffset, suspensionFramePositionY, longitudinalWheelOffset),
        pxr::GfVec3f(lateralWheelOffset, suspensionFramePositionY, -longitudinalWheelOffset),
        pxr::GfVec3f(-lateralWheelOffset, suspensionFramePositionY, -longitudinalWheelOffset),
    };
    pxr::VtVec3fArray suspensionFramePositions;
    suspensionFramePositions.resize(wheelCount);
    convertGfVec3List<pxr::VtVec3fArray&>(suspensionFramePositionsBase, suspensionFramePositions, wheelCount,
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
        const pxr::GfVec3f& wheelPos = wheelPositions[i];
        const pxr::GfVec3f& suspFramePos = suspensionFramePositions[i];
        const pxr::SdfPath& vehicleWheelPath = wheelAttachmentPaths[i];
        
        pxr::UsdGeomXform vehicleWheelXform = pxr::UsdGeomXform::Define(stage, vehicleWheelPath);

        vehicleWheelXform.AddTranslateOp(pxr::UsdGeomXformOp::PrecisionFloat).Set(wheelPos);

        pxr::UsdPrim vehicleWheelPrim = vehicleWheelXform.GetPrim();
        pxr::PhysxSchemaPhysxVehicleWheelAttachmentAPI wheelAttachmentAPI = pxr::PhysxSchemaPhysxVehicleWheelAttachmentAPI::Apply(vehicleWheelPrim);

        if (params.wheelPaths)
        {
            pxr::UsdRelationship wheelRel = wheelAttachmentAPI.CreateWheelRel();
            wheelRel.AddTarget((*params.wheelPaths)[i]);
        }
        else
        {
            setUpWheel(vehicleWheelPrim, unitScale);
        }

        if (params.tirePaths)
        {
            pxr::UsdRelationship tireRel = wheelAttachmentAPI.CreateTireRel();
            tireRel.AddTarget((*params.tirePaths)[i]);
        }
        else
        {
            CARB_ASSERT(tireFrictionTablePath);
            setUpTire(vehicleWheelPrim, *tireFrictionTablePath, unitScale);
        }

        if (params.suspensionPaths)
        {
            pxr::UsdRelationship suspensionRel = wheelAttachmentAPI.CreateSuspensionRel();
            suspensionRel.AddTarget((*params.suspensionPaths)[i]);
        }
        else
        {
            setUpSuspension(vehicleWheelPrim, unitScale);
        }

        pxr::UsdRelationship collisionGroupRel = wheelAttachmentAPI.CreateCollisionGroupRel();
        collisionGroupRel.AddTarget(groundQueryCollisionGroupPath);

        pxr::GfVec3f suspTravelDir(0.0f);
        suspTravelDir[axes.up] = -1.0f;
        wheelAttachmentAPI.CreateSuspensionTravelDirectionAttr().Set(suspTravelDir);

        wheelAttachmentAPI.CreateSuspensionFramePositionAttr().Set(suspFramePos);
        wheelAttachmentAPI.CreateSuspensionFrameOrientationAttr().Set(pxr::GfQuatf(1.0f, 0.0f, 0.0f, 0.0f));

        wheelAttachmentAPI.CreateIndexAttr().Set(static_cast<int>(i));
        if (wheelIsDriven[i])
        {
            drivenWheelIndexList[drivenWheelCount] = static_cast<int>(i);
            drivenWheelCount++;
        }

        pxr::PhysxSchemaPhysxVehicleSuspensionComplianceAPI suspensionComplianceAPI = pxr::PhysxSchemaPhysxVehicleSuspensionComplianceAPI::Apply(vehicleWheelPrim);
        // empty values for some attributes just to make sure they exist for tests that might want to make
        // use of them
        suspensionComplianceAPI.CreateWheelToeAngleAttr().Set(pxr::VtVec2fArray());
        suspensionComplianceAPI.CreateWheelCamberAngleAttr().Set(pxr::VtVec2fArray());
        const pxr::GfVec3f forceAppPoint = wheelPos - suspFramePos;
        const pxr::GfVec4f forceAppPointEntry(0.0, forceAppPoint[0], forceAppPoint[1], forceAppPoint[2]);
        pxr::VtVec4fArray forceAppPointArray;
        forceAppPointArray.push_back(forceAppPointEntry);
        suspensionComplianceAPI.CreateSuspensionForceAppPointAttr().Set(forceAppPointArray);
        suspensionComplianceAPI.CreateTireForceAppPointAttr().Set(forceAppPointArray);

        if ((params.drivePath == nullptr) && (driveMode == DriveMode::eNONE) && (!params.omitControllers))
        {
            pxr::PhysxSchemaPhysxVehicleWheelControllerAPI wheelControllerAPI = pxr::PhysxSchemaPhysxVehicleWheelControllerAPI::Apply(vehicleWheelPrim);
            wheelControllerAPI.CreateDriveTorqueAttr().Set(0.0f);
            wheelControllerAPI.CreateBrakeTorqueAttr().Set(0.0f);
            wheelControllerAPI.CreateSteerAngleAttr().Set(0.0f);
        }

        if (params.createCollisionShapesForWheels)
        {
            CARB_ASSERT(wheelCollisionGroupPath);

            static const pxr::SdfPath vehicleWheelCollPathPostfix("Collision");
            pxr::SdfPath vehicleWheelCollPath = vehicleWheelPath.AppendPath(vehicleWheelCollPathPostfix);
            pxr::UsdPrim collisionGeomPrim;
            if (params.useMeshAsWheelCollisionShape)
            {
                pxr::UsdGeomMesh collisionGeom = createCylinderMeshWithConvexHull(stage, vehicleWheelCollPath, axes);

                pxr::GfVec3f scale(1.0f);
                scale[axes.up] = wheelRadius;
                scale[axes.forward] = wheelRadius;
                scale[axes.side] = wheelWidth * 0.5f;
                collisionGeom.AddScaleOp(pxr::UsdGeomXformOp::PrecisionFloat).Set(scale);

                collisionGeomPrim = collisionGeom.GetPrim();
            }
            else
            {
                pxr::UsdGeomCylinder collisionGeom = pxr::UsdGeomCylinder::Define(stage, vehicleWheelCollPath);

                collisionGeom.CreateHeightAttr().Set(static_cast<double>(wheelWidth));
                collisionGeom.CreateRadiusAttr().Set(static_cast<double>(wheelRadius));

                if (axes.side == 0)
                    collisionGeom.CreateAxisAttr().Set(pxr::UsdGeomTokens->x);
                else if (axes.side == 1)
                    collisionGeom.CreateAxisAttr().Set(pxr::UsdGeomTokens->y);
                else
                    collisionGeom.CreateAxisAttr().Set(pxr::UsdGeomTokens->z);

                // if height or radius is authored, USD expects extent to be authored too
                pxr::VtVec3fArray cylExtent;
                if (pxr::UsdGeomCylinder::ComputeExtentFromPlugins(collisionGeom, 0, &cylExtent))
                    collisionGeom.CreateExtentAttr().Set(cylExtent);

                collisionGeomPrim = collisionGeom.GetPrim();
            }

            pxr::UsdPhysicsCollisionAPI collisionAPI = pxr::UsdPhysicsCollisionAPI::Apply(collisionGeomPrim);
            addCollisionToCollisionGroup(stage, vehicleWheelCollPath, *wheelCollisionGroupPath);

            pxr::PhysxSchemaPhysxCollisionAPI physxCollisionAPI = pxr::PhysxSchemaPhysxCollisionAPI::Apply(collisionGeomPrim);
            physxCollisionAPI.CreateRestOffsetAttr().Set(0.0f * unitScale.lengthScale);
            physxCollisionAPI.CreateContactOffsetAttr().Set(0.02f * unitScale.lengthScale);
        }
    };

    if (driveMode != DriveMode::eNONE)
    {
        // set up one brake configuration that applies to all wheels
        pxr::PhysxSchemaPhysxVehicleBrakesAPI brakes0API = pxr::PhysxSchemaPhysxVehicleBrakesAPI::Apply(
            vehiclePrim, pxr::PhysxSchemaTokens->brakes0);
        brakes0API.CreateMaxBrakeTorqueAttr().Set(3600.0f * kgmsScale);

        // set up a handbrake configuration that applies to the rear wheels only
        pxr::PhysxSchemaPhysxVehicleBrakesAPI brakes1API = pxr::PhysxSchemaPhysxVehicleBrakesAPI::Apply(
            vehiclePrim, pxr::PhysxSchemaTokens->brakes1);
        brakes1API.CreateWheelsAttr().Set(pxr::VtIntArray({ 2, 3 }));
        brakes1API.CreateMaxBrakeTorqueAttr().Set(3000.0f * kgmsScale);

        // set up a steering configuration that applies to the front wheels
        pxr::PhysxSchemaPhysxVehicleSteeringAPI steeringAPI = pxr::PhysxSchemaPhysxVehicleSteeringAPI::Apply(vehiclePrim);
        steeringAPI.CreateWheelsAttr().Set(pxr::VtIntArray({ 0, 1 }));
        steeringAPI.CreateMaxSteerAngleAttr().Set(0.554264f);

        if (drivenWheelCount)
        {
            float ratio = 1.0f / drivenWheelCount;
            pxr::VtFloatArray ratios(drivenWheelCount, ratio);

            pxr::VtIntArray drivenWheelIndexListPxr;
            for (uint32_t j = 0; j < drivenWheelCount; j++)
                drivenWheelIndexListPxr.push_back(drivenWheelIndexList[j]);

            pxr::PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI multiWheelDiffAPI = pxr::PhysxSchemaPhysxVehicleMultiWheelDifferentialAPI::Apply(vehiclePrim);
            multiWheelDiffAPI.CreateWheelsAttr().Set(drivenWheelIndexListPxr);
            multiWheelDiffAPI.CreateTorqueRatiosAttr().Set(ratios);

            if (driveMode == DriveMode::eSTANDARD)
            {
                multiWheelDiffAPI.CreateAverageWheelSpeedRatiosAttr().Set(ratios);
            }
        }
    }

    pxr::GfVec3f chassisHalfExtents(0.0f);
    chassisHalfExtents[axes.up] = chassisHalfHeight;
    chassisHalfExtents[axes.forward] = 2.4f * unitScale.lengthScale;
    chassisHalfExtents[axes.side] = 0.9f * unitScale.lengthScale;
    pxr::GfVec3f chassisOffset(0.0f);
    chassisOffset[axes.up] = 0.0f * unitScale.lengthScale;
    chassisOffset[axes.forward] = 0.0f * unitScale.lengthScale;
    chassisOffset[axes.side] = 0.0f * unitScale.lengthScale;

    //
    // chassis (collision)
    //
    if (params.addChassisCollisionBox)
    {
        CARB_ASSERT(chassisCollisionGroupPath);

        static const pxr::SdfPath vehicleChassisPathPostfix("ChassisCollision");
        pxr::SdfPath vehicleChassisPath = vehiclePath.AppendPath(vehicleChassisPathPostfix);
        pxr::UsdGeomCube vehicleChassis = pxr::UsdGeomCube::Define(stage, vehicleChassisPath);
        vehicleChassis.CreatePurposeAttr().Set(pxr::UsdGeomTokens->guide);
        vehicleChassis.AddTranslateOp(pxr::UsdGeomXformOp::PrecisionFloat).Set(chassisOffset);
        vehicleChassis.AddScaleOp(pxr::UsdGeomXformOp::PrecisionFloat).Set(chassisHalfExtents);

        pxr::UsdPrim vehicleChassisPrim = vehicleChassis.GetPrim();

        pxr::UsdPhysicsCollisionAPI collisionAPI = pxr::UsdPhysicsCollisionAPI::Apply(vehicleChassisPrim);
        addCollisionToCollisionGroup(stage, vehicleChassisPath, *chassisCollisionGroupPath);

        pxr::PhysxSchemaPhysxCollisionAPI physxCollisionAPI = pxr::PhysxSchemaPhysxCollisionAPI::Apply(vehicleChassisPrim);
        physxCollisionAPI.CreateRestOffsetAttr().Set(0.0f * unitScale.lengthScale);
        physxCollisionAPI.CreateContactOffsetAttr().Set(0.02f * unitScale.lengthScale);
    }

    //
    // chassis (render)
    //
    if (params.addChassisRenderMesh)
    {
        static const pxr::SdfPath vehicleChassisPathPostfix("ChassisRender");
        pxr::SdfPath vehicleChassisPath = vehiclePath.AppendPath(vehicleChassisPathPostfix);
        pxr::UsdGeomMesh vehicleChassis = pxr::UsdGeomMesh::Define(stage, vehicleChassisPath);
        vehicleChassis.AddTranslateOp(pxr::UsdGeomXformOp::PrecisionFloat).Set(chassisOffset);
        vehicleChassis.CreateDisplayColorAttr().Set(pxr::VtArray<pxr::GfVec3f>({pxr::GfVec3f(0.2784314f, 0.64705884f, 1.0f)}));

        pxr::VtIntArray faceVertexCounts({
            4, 4, 4, 4, 4, 4
        });

        pxr::VtIntArray faceVertexIndices({
            0, 1, 3, 2, 4, 5, 7, 6, 10, 11, 13, 12, 14, 15, 9, 8, 17, 23, 21, 19, 22, 16, 18, 20
        });

        pxr::GfVec3f normalsBase[] = {
            pxr::GfVec3f(0.0f, 1.0f, 0.0f),
            pxr::GfVec3f(0.0f, 1.0f, 0.0f),
            pxr::GfVec3f(0.0f, 1.0f, 0.0f),
            pxr::GfVec3f(0.0f, 1.0f, 0.0f),
            pxr::GfVec3f(0.0f, -1.0f, 0.0f),
            pxr::GfVec3f(0.0f, -1.0f, 0.0f),
            pxr::GfVec3f(0.0f, -1.0f, 0.0f),
            pxr::GfVec3f(0.0f, -1.0f, 0.0f),
            pxr::GfVec3f(0.0f, 0.0f, -1.0f),
            pxr::GfVec3f(0.0f, 0.0f, -1.0f),
            pxr::GfVec3f(0.0f, 0.0f, 1.0f),
            pxr::GfVec3f(0.0f, 0.0f, 1.0f),
            pxr::GfVec3f(0.0f, 0.0f, 1.0f),
            pxr::GfVec3f(0.0f, 0.0f, 1.0f),
            pxr::GfVec3f(0.0f, 0.0f, -1.0f),
            pxr::GfVec3f(0.0f, 0.0f, -1.0f),
            pxr::GfVec3f(-1.0f, 0.0f, 0.0f),
            pxr::GfVec3f(1.0f, 0.0f, 0.0f),
            pxr::GfVec3f(-1.0f, 0.0f, 0.0f),
            pxr::GfVec3f(1.0f, 0.0f, 0.0f),
            pxr::GfVec3f(-1.0f, 0.0f, 0.0f),
            pxr::GfVec3f(1.0f, 0.0f, 0.0f),
            pxr::GfVec3f(-1.0f, 0.0f, 0.0f),
            pxr::GfVec3f(1.0f, 0.0f, 0.0f),
        };
        static const uint32_t normalCount = sizeof(normalsBase) / sizeof(normalsBase[0]);
        pxr::VtVec3fArray normals;
        normals.resize(normalCount);
        convertGfVec3List<pxr::VtVec3fArray&>(normalsBase, normals, normalCount,
            gYZXAxes, axes);

        chassisHalfExtents[axes.side] = 0.7f * unitScale.lengthScale;  // reduced width to make the wheels easily visible
        pxr::VtVec3fArray points({
            pxr::GfVec3f(-chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
            pxr::GfVec3f(chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
            pxr::GfVec3f(-chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
            pxr::GfVec3f(chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
            pxr::GfVec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
            pxr::GfVec3f(chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
            pxr::GfVec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
            pxr::GfVec3f(chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
            pxr::GfVec3f(-chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
            pxr::GfVec3f(chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
            pxr::GfVec3f(-chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
            pxr::GfVec3f(chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
            pxr::GfVec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
            pxr::GfVec3f(chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
            pxr::GfVec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
            pxr::GfVec3f(chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
            pxr::GfVec3f(-chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
            pxr::GfVec3f(chassisHalfExtents[0], chassisHalfExtents[1], -chassisHalfExtents[2]),
            pxr::GfVec3f(-chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
            pxr::GfVec3f(chassisHalfExtents[0], chassisHalfExtents[1], chassisHalfExtents[2]),
            pxr::GfVec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
            pxr::GfVec3f(chassisHalfExtents[0], -chassisHalfExtents[1], chassisHalfExtents[2]),
            pxr::GfVec3f(-chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
            pxr::GfVec3f(chassisHalfExtents[0], -chassisHalfExtents[1], -chassisHalfExtents[2]),
        });

        vehicleChassis.CreateFaceVertexCountsAttr().Set(faceVertexCounts);
        vehicleChassis.CreateFaceVertexIndicesAttr().Set(faceVertexIndices);
        vehicleChassis.CreateNormalsAttr().Set(normals);
        vehicleChassis.CreatePointsAttr().Set(points);
    }
}


void VehicleFactory::create4WheeledCarsScenario(
    const pxr::UsdStageRefPtr& stage,
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

    pxr::SdfPath rootPath = getDefaultPrimPath(stage);

    createSceneBasics(stage, unitScale, params.axes, params.timeStepsPerSecond);

    pxr::SdfPath collisionGroupPaths[CollGroupId::eCOUNT];
    createCollisionGroups(stage, &collisionGroupPaths);
    if (params.collisionGroupPathsOut)
    {
        for (uint32_t i = 0; i < CollGroupId::eCOUNT; i++)
        {
            (*params.collisionGroupPathsOut)[i] = collisionGroupPaths[i];
        }
    }

    pxr::SdfPath materialPaths[MaterialId::eCOUNT];
    pxr::SdfPath tireFrictionTablePaths[TireFrictionTableId::eCOUNT];
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

    const pxr::SdfPath& tireFrictionTablePath = tireFrictionTablePaths[TireFrictionTableId::eWINTER_TIRE];

    pxr::SdfPath carWheelPaths[wheelCount];
    pxr::SdfPath carTirePaths[wheelCount];
    pxr::SdfPath carSuspensionPaths[wheelCount];
    pxr::SdfPath drivePath;
    if (createShareableComponents)
    {
        pxr::SdfPath wheelPath;
        pxr::SdfPath tirePaths[TireId::eCOUNT];
        pxr::SdfPath suspensionPaths[SuspensionId::eCOUNT];

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

    pxr::GfVec3f vehiclePositionBase(0.0f);
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
        pxr::SdfPath vehiclePath = rootPath.AppendPath(pxr::SdfPath(postfixBuffer));

        if (params.vehiclePathsOut)
            params.vehiclePathsOut[i] = vehiclePath;

        pxr::GfVec3f vehiclePosition(
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

