// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-MAT-001
 * @covers AC-3
 *
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-1 AC-3
 *
 * @implements REQ-LOAD-TOKENS-001
 * @covers AC-2
 */

// This include must come first
// clang-format off
#include "IceDescriptorAllocator.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <private/omni/physx/PhysxUsd.h>
#include <common/foundation/Allocator.h>

#include "LoadUsd.h"
#include "LoadTools.h"
#include "PhysXTools.h"

#include "Particles.h"
#include "AttributeHelpers.h"

#include <omni/physx/IPhysxSettings.h>
#include <OmniPhysX.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>
#include "IceDescriptorAllocator.h"

using namespace carb;

namespace omni
{
namespace physx
{
namespace usdparser
{

void setToDefault(PhysxMaterialDesc& desc)
{
    // Defaults match the parse::PhysxMaterialDesc constructor; this function
    // resets a recycled descriptor to the same state.
    desc.frictionCombineMode = eAverage;
    desc.restitutionCombineMode = eAverage;
    desc.dampingCombineMode = eAverage;

    desc.staticFriction = 0.5f;
    desc.dynamicFriction = 0.5f;
    desc.restitution = 0.0f;
    desc.density = -1.0f;

    desc.materialKey = omni::physics::parse::ObjectKey{};

    desc.compliantAccelerationSpring = false;
    desc.compliantStiffness = 0.0f;
    desc.compliantDamping = 0.0f;
}

void setToDefault(const omni::physics::parse::SourceUnits& units, PhysxDeformableMaterialDesc& desc)
{
    //same defaults as in deformableUtils.py
    desc.staticFriction = 0.5f;
    desc.dynamicFriction = 0.25f;
    desc.density = -1.0f;

    desc.youngsModulus = float(5.0e5 * double(units.metersPerUnit) / double(units.kilogramsPerUnit));
    desc.poissonsRatio = 0.45f;

    desc.elasticityDamping = 0.0f;

    desc.materialKey = omni::physics::parse::ObjectKey{};
}

void setToDefault(const omni::physics::parse::SourceUnits& units, PhysxSurfaceDeformableMaterialDesc& desc)
{
    setToDefault(units, static_cast<PhysxDeformableMaterialDesc&>(desc));
    desc.surfaceThickness = float(0.001 / double(units.metersPerUnit));
    desc.surfaceStretchStiffness = 0.0f;
    desc.surfaceShearStiffness = 0.0f;
    desc.surfaceBendStiffness = 0.0f;

    desc.bendDamping = 0.0f;
}


void setToDefault(PBDMaterialDesc& desc)
{
    desc.friction = 0.2f;
    desc.particleFrictionScale = 1.0f;
    desc.damping = 0.0f;
    desc.viscosity = 0.0f;

    desc.vorticityConfinement = 0.0f;
    desc.surfaceTension = 0.0f;
    desc.cohesion = 0.0f;
    desc.adhesion = 0.0f;
    desc.particleAdhesionScale = 1.0f;
    desc.adhesionOffsetScale = 0.0f;

    desc.gravityScale = 1.0f;

    desc.cflCoefficient = 1.0f;

    desc.density = 1000.0f;

    desc.materialKey = omni::physics::parse::ObjectKey{};
}

void parsePBDMaterialForPrim(AttachedStage& attachedStage,
                             const omni::physics::parse::ObjectKey& materialKey,
                             PBDMaterialDesc& desc)
{
    setToDefault(desc);

    if (!materialKey.valid())
        return;
    omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return;

    omni::physics::parse::ParseContext ctx(*source, omni::physx::usdparser::iceDescriptorAllocator());
    ctx.adoptKnownTokens(attachedStage.getKnownTokens());
    omni::physics::parse::DescPtr<PBDMaterialDesc> parsed =
        omni::physics::parse::parsePBDMaterial(ctx, materialKey);
    if (parsed)
        desc = *parsed;
}

void parseDeformableMaterialForPrim(AttachedStage& attachedStage,
                                    const omni::physics::parse::ObjectKey& materialKey,
                                    PhysxDeformableMaterialDesc& desc)
{
    setToDefault(attachedStage.getSourceUnits(), desc);

    if (!materialKey.valid())
        return;
    omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return;

    omni::physics::parse::ParseContext ctx(*source, omni::physx::usdparser::iceDescriptorAllocator());
    ctx.adoptKnownTokens(attachedStage.getKnownTokens());
    omni::physics::parse::parseDeformableMaterial(ctx, materialKey, desc);
    desc.poissonsRatio = std::min(desc.poissonsRatio, 0.4999f);
}

void parseSurfaceDeformableMaterialForPrim(AttachedStage& attachedStage,
                                           const omni::physics::parse::ObjectKey& materialKey,
                                           PhysxSurfaceDeformableMaterialDesc& desc)
{
    setToDefault(attachedStage.getSourceUnits(), desc);

    if (!materialKey.valid())
        return;
    omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return;

    omni::physics::parse::ParseContext ctx(*source, omni::physx::usdparser::iceDescriptorAllocator());
    ctx.adoptKnownTokens(attachedStage.getKnownTokens());
    omni::physics::parse::parseSurfaceDeformableMaterial(ctx, materialKey, desc);
    desc.poissonsRatio = std::min(desc.poissonsRatio, 0.4999f);
}

void parseMaterialForPrim(AttachedStage& attachedStage,
                          const omni::physics::parse::ObjectKey& materialKey,
                          PhysxMaterialDesc& desc)
{
    setToDefault(desc);

    if (!materialKey.valid())
        return;
    omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return;

    omni::physics::parse::ParseContext ctx(*source, omni::physx::usdparser::iceDescriptorAllocator());
    ctx.adoptKnownTokens(attachedStage.getKnownTokens());
    omni::physics::parse::DescPtr<PhysxMaterialDesc> parsed =
        omni::physics::parse::parseMaterial(ctx, materialKey);
    if (parsed)
        desc = *parsed;
}

ObjectId getMaterial(AttachedStage& attachedStage, omni::physics::parse::ObjectKey materialKey)
{
    if (materialKey.valid())
        return attachedStage.getObjectDatabase()->findEntry(materialKey, eMaterial);
    else
        return kInvalidObjectId;
}

ObjectId getMaterial(AttachedStage& attachedStage, omni::physics::parse::ObjectKey materialKey, const ObjectCategory objCategory)
{
    if (materialKey.valid())
        return attachedStage.getObjectDatabase()->findEntry(materialKey, objCategory);
    else
        return kInvalidObjectId;
}

} // namespace usdparser
} // namespace physx
} // namespace omni
