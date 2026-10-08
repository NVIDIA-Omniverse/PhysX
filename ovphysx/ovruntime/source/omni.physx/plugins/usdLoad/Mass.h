// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <carb/Types.h>
#include <foundation/PxVec3.h>
#include "LoadTools.h"
#include <usdInterface/UsdInterface.h>
#include <omni/physics/parse/IPhysicsSource.h> // IPhysicsSource, ObjectKey, SourceUnits (backend-agnostic mass)
#include <omni/physics/parse/KnownTokens.h> // KnownTokens (attach-scoped batch handed to computeRigidBodyMass)

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
    ::physx::PxVec3 diagonalInertia = { 1.0f, 1.0f, 1.0f };
    bool hasCenterOfMass = false;
    carb::Float3 centerOfMass = { 0.0f, 0.0f, 0.0f };
    bool hasPrincipalAxes = false;
    carb::Float4 principalAxes = { 0.0f, 0.0f, 0.0f, 1.0f };
};

struct AbstractComputeRigidBodyMass
{
    // Shapes keyed by ObjectKey (ADR-0002 M2c-D): backend-agnostic, so mass
    // works without live USD prims (e.g. ovstage). getShapeMassInfo is keyed
    // purely by ObjectId (engine-side analytic properties) -- it never needed
    // the shape's identity at all, so no key/path is passed to it.
    virtual bool getRigidBodyShapes(usdparser::ObjectId rbId, usdparser::ObjectIdPathMap& shapes) = 0;
    virtual PhysXUsdPhysicsInterface::MassInformation getShapeMassInfo(usdparser::ObjectId objectId) = 0;
};

struct RigidBodyMass
{
    float mass = 0;
    carb::Float3 inertia = { 0, 0, 0 };
    carb::Float3 centerOfMass = { 0, 0, 0 };
    carb::Float4 principalAxes = { 0, 0, 0, 1 };
};

// Backend-agnostic mass compute (ADR-0002 M2c-D): MassAPI / material / units /
// world-transform are read through `source` (USD or ovstage) keyed by `bodyKey`;
// shape geometry mass comes from the engine via `crbmInterface`. No USD stage or
// UsdPrim required.
// `knownTokens`: batch already interned for `source` (AttachedStage::getKnownTokens()), so the
// per-body ParseContext does not re-intern it (REQ-LOAD-TOKENS-001).
RigidBodyMass computeRigidBodyMass(AbstractComputeRigidBodyMass* crbmInterface,
                                   omni::physics::parse::IPhysicsSource& source,
                                   omni::physics::parse::ObjectKey bodyKey,
                                   usdparser::ObjectId rbId,
                                   const omni::physics::parse::KnownTokens* knownTokens = nullptr);

void RequestRigidBodyMassUpdate(AttachedStage& stage, omni::physics::parse::ObjectKey bodyKey);
void RequestParticleMassUpdate(AttachedStage& stage, omni::physics::parse::ObjectKey particleKey);

/**
 * @brief Unit conversion helper to convert a value in SI units to a value in stage units.
 * Backend-agnostic — units come through the parse SourceUnits abstraction, not a
 * direct USD-stage read. For example, to convert a density of 10.0 kg/(m*m*m):
 * convertSiValueToStageUnits(units, 10.0f, -3, 1)
 * @param[in] units Stage units (metersPerUnit / kilogramsPerUnit)
 * @param[in] siValue The value in SI units
 * @param[in] distanceExponent The net exponent of distance in the quanity
 * @param[in] massExponent The net exponent of mass in the quanity
 * @return The value in stage units
 */
float convertSiValueToStageUnits(const omni::physics::parse::SourceUnits& units,
                                 float siValue,
                                 int distanceExponent = 0,
                                 int massExponent = 0);

inline float getScaledDensity(const omni::physics::parse::SourceUnits& units, float baseValue)
{
    return baseValue <= 0.0f ? baseValue : convertSiValueToStageUnits(units, baseValue, -3, 1);
}


} // namespace usdparser
} // namespace physx
} // namespace omni
