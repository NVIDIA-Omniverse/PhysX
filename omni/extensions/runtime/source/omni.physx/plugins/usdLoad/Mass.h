// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Types.h>
#include "LoadTools.h"
#include <usdInterface/UsdInterface.h>

namespace omni
{
namespace physx
{
namespace usdparser
{
class AttachedStage;

struct MassApiData
{
    float mass = -1.0f;
    float density = -1.0f;
    bool hasInertia = false;
    pxr::GfVec3f diagonalInertia = { 1.0f, 1.0f, 1.0f };
};

MassApiData parseMassApi(const pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& usdPrim);

struct AbstractComputeRigidBodyMass
{
    virtual bool getRigidBodyShapes(usdparser::ObjectId rbId, usdparser::ObjectIdUsdPrimMap& shapes) = 0;
    virtual PhysXUsdPhysicsInterface::MassInformation getShapeMassInfo(const pxr::SdfPath& path,
                                                                       usdparser::ObjectId objectId) = 0;
};

struct RigidBodyMass
{
    float mass = 0;
    carb::Float3 inertia = { 0, 0, 0 };
    carb::Float3 centerOfMass = { 0, 0, 0 };
    carb::Float4 principalAxes = { 0, 0, 0, 1 };
};

RigidBodyMass computeRigidBodyMass(AbstractComputeRigidBodyMass* crbmInterface,
                                   const pxr::UsdStageWeakPtr stage,
                                   const pxr::UsdPrim& usdPrim,
                                   usdparser::ObjectId rbId);

void RequestRigidBodyMassUpdate(AttachedStage& stage, const pxr::UsdPrim& usdPrim);
void RequestDeformableBodyMassUpdateDeprecated(AttachedStage& stage, const pxr::UsdPrim& usdPrim);
void RequestDeformableSurfaceMassUpdateDeprecated(AttachedStage& stage, const pxr::UsdPrim& usdPrim);
void RequestParticleMassUpdate(AttachedStage& stage, const pxr::UsdPrim& usdPrim);

/**
 * @brief Unit conversion helper to convert a value in SI units to a value in stage units.
 * For example, to convert a density of 10.0 kg/(m*m*m) into stage units use
 * convertSiValueToStageUnits(stage, 10.0f, -3, 1)
 * @param[in] stage Pointer to the stage for extracting stage units
 * @param[in] siValue The value in SI units
 * @param[in] distanceExponent The net exponent of distance in the quanity
 * @param[in] massExponent The net exponent of mass in the quanity
 * @return The value in stage units
 */
float convertSiValueToStageUnits(const pxr::UsdStageWeakPtr stage,
                                 float siValue,
                                 int distanceExponent = 0,
                                 int massExponent = 0);

inline float getScaledDensity(const pxr::UsdStageWeakPtr stage, float baseValue)
{
    return baseValue <= 0.0f ? baseValue : convertSiValueToStageUnits(stage, baseValue, -3, 1);
}

inline float getScaledMass(const pxr::UsdStageWeakPtr stage, float baseValue)
{
    return baseValue <= 0.0f ? baseValue : convertSiValueToStageUnits(stage, baseValue, 0, 1);
}


} // namespace usdparser
} // namespace physx
} // namespace omni
