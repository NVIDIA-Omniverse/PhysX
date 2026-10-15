// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */
#pragma once
#include "Descriptors.h"
#include "PreparedAttribute.h"

namespace omni::physics::parse
{
class ParseContext;

struct PreparedMaterial
{
    bool prepared = false;
    ObjectKey key;
    bool physxMaterialAPI = false;
    TokenId average, min, max, multiply;
    PreparedAttribute<float> staticFriction;
    PreparedAttribute<float> dynamicFriction;
    PreparedAttribute<float> restitution;
    PreparedAttribute<float> density;
    PreparedAttribute<TokenId> frictionCombineMode;
    PreparedAttribute<TokenId> restitutionCombineMode;
    PreparedAttribute<TokenId> dampingCombineMode;
    PreparedAttribute<bool> compliantContactAccelerationSpring;
    PreparedAttribute<float> compliantContactStiffness;
    PreparedAttribute<float> compliantContactDamping;
};

PreparedMaterial prepareMaterial(ParseContext& context, ObjectKey key);
bool fillMaterial(const PreparedMaterial& input, PhysxMaterialDesc& output) noexcept;
struct PreparedPBDMaterial
{
    bool prepared = false, enabled = false;
    ObjectKey key;
    PreparedAttribute<float> friction;
    PreparedAttribute<float> particleFrictionScale;
    PreparedAttribute<float> damping;
    PreparedAttribute<float> viscosity;
    PreparedAttribute<float> vorticityConfinement;
    PreparedAttribute<float> surfaceTension;
    PreparedAttribute<float> cohesion;
    PreparedAttribute<float> adhesion;
    PreparedAttribute<float> particleAdhesionScale;
    PreparedAttribute<float> adhesionOffsetScale;
    PreparedAttribute<float> gravityScale;
    PreparedAttribute<float> cflCoefficient;
    PreparedAttribute<float> density;
};
PreparedPBDMaterial preparePBDMaterial(ParseContext& context, ObjectKey key);
bool fillPBDMaterial(const PreparedPBDMaterial& input, PBDMaterialDesc& output) noexcept;

struct PreparedDeformableMaterial
{
    bool prepared = false, surface = false, elasticity = false, bend = false;
    ObjectKey key;
    PreparedAttribute<float> dynamicFriction;
    PreparedAttribute<float> staticFriction;
    PreparedAttribute<float> density;
    PreparedAttribute<float> youngsModulus;
    PreparedAttribute<float> poissonsRatio;
    PreparedAttribute<float> elasticityDamping;
    PreparedAttribute<float> surfaceThickness;
    PreparedAttribute<float> surfaceStretchStiffness;
    PreparedAttribute<float> surfaceShearStiffness;
    PreparedAttribute<float> surfaceBendStiffness;
    PreparedAttribute<float> bendDamping;
};
PreparedDeformableMaterial prepareDeformableMaterial(ParseContext& context, ObjectKey key, bool surface);
bool fillDeformableMaterial(const PreparedDeformableMaterial& input, PhysxDeformableMaterialDesc& output) noexcept;
bool fillSurfaceDeformableMaterial(const PreparedDeformableMaterial& input, PhysxSurfaceDeformableMaterialDesc& output) noexcept;

} // namespace omni::physics::parse
