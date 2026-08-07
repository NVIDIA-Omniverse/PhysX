// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

/**
 * @implements REQ-PARSE-MAT-001
 * @covers AC-3
 *
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-1 AC-3
 */

// This include must come first
// clang-format off
#include "UsdPCH.h"
#include "IceDescriptorAllocator.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <omni/physics/usd/PrimIterator.h>
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
#include "UsdSource.h"
#include "IceDescriptorAllocator.h"

using namespace PXR_NS;
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

// Re-key a parse-lib materialKey ObjectKey (minted by a per-call
// UsdSource) into the AttachedStage's vocabulary so the descriptor
// can travel across the parse-lib / consumer boundary.
AttachedStage* attachedStageFor(PXR_NS::UsdStageWeakPtr stage)
{
    if (!stage)
        return nullptr;
    const long stageId = PXR_NS::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();
    return UsdLoad::getUsdLoad()->getAttachedStage(stageId);
}

omni::physics::parse::ObjectKey reKeyMaterialPath(PXR_NS::UsdStageWeakPtr stage, const SdfPath& materialKey)
{
    if (!stage || materialKey == SdfPath())
        return {};
    AttachedStage* attachedStage = attachedStageFor(stage);
    return attachedStage ? attachedStage->keyFor(materialKey) : omni::physics::parse::ObjectKey{};
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
    omni::physics::parse::DescPtr<PBDMaterialDesc> parsed =
        omni::physics::parse::parsePBDMaterial(ctx, materialKey);
    if (parsed)
        desc = *parsed;
}

void parsePBDMaterialForPrim(PXR_NS::UsdStageWeakPtr stage, const SdfPath& materialKey, PBDMaterialDesc& desc)
{
    setToDefault(desc);

    if (materialKey != SdfPath())
    {
        AttachedStage* attachedStage = attachedStageFor(stage);
        if (attachedStage)
        {
            parsePBDMaterialForPrim(*attachedStage, attachedStage->keyFor(materialKey), desc);
            return;
        }

        // Per-call UsdSource fallback when no stage is attached. The source parser
        // silently returns null for non-PBD materials, matching the scene default
        // material overlay path.
        omni::physics::usd::UsdSource source(stage);
        omni::physics::parse::ParseContext ctx(source, omni::physx::usdparser::iceDescriptorAllocator());
        omni::physics::parse::DescPtr<PBDMaterialDesc> parsed =
            omni::physics::parse::parsePBDMaterial(ctx, source.keyFor(materialKey));
        if (parsed)
        {
            desc = *parsed;
            desc.materialKey = reKeyMaterialPath(stage, materialKey);
        }
    }
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
    omni::physics::parse::parseDeformableMaterial(ctx, materialKey, desc);
    desc.poissonsRatio = std::min(desc.poissonsRatio, 0.4999f);
}

// Per-stage deformable-material parse for callers outside the
// stage-attach pipeline.  Constructs a per-call UsdSource +
// ParseContext so parse::parseDeformableMaterial can read attrs
// natively, then re-keys materialKey into the AttachedStage.
void parseDeformableMaterialForPrim(PXR_NS::UsdStageWeakPtr stage, const SdfPath& materialKey, PhysxDeformableMaterialDesc& desc)
{
    AttachedStage* attachedStage = attachedStageFor(stage);
    if (attachedStage)
    {
        const omni::physics::parse::ObjectKey key =
            materialKey == SdfPath() ? omni::physics::parse::ObjectKey{} : attachedStage->keyFor(materialKey);
        parseDeformableMaterialForPrim(*attachedStage, key, desc);
        return;
    }

    // Units come through the source abstraction (UsdSource::getSourceUnits), not a
    // direct UsdGeom* read; the source is reused for parsing below.
    omni::physics::usd::UsdSource source(stage);
    setToDefault(source.getSourceUnits(), desc);

    if (materialKey == SdfPath())
        return;
    omni::physics::parse::ParseContext ctx(source, omni::physx::usdparser::iceDescriptorAllocator());
    omni::physics::parse::parseDeformableMaterial(ctx, source.keyFor(materialKey), desc);
    desc.poissonsRatio = std::min(desc.poissonsRatio, 0.4999f);
    desc.materialKey = reKeyMaterialPath(stage, materialKey);
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
    omni::physics::parse::parseSurfaceDeformableMaterial(ctx, materialKey, desc);
    desc.poissonsRatio = std::min(desc.poissonsRatio, 0.4999f);
}

void parseSurfaceDeformableMaterialForPrim(PXR_NS::UsdStageWeakPtr stage, const SdfPath& materialKey, PhysxSurfaceDeformableMaterialDesc& desc)
{
    AttachedStage* attachedStage = attachedStageFor(stage);
    if (attachedStage)
    {
        const omni::physics::parse::ObjectKey key =
            materialKey == SdfPath() ? omni::physics::parse::ObjectKey{} : attachedStage->keyFor(materialKey);
        parseSurfaceDeformableMaterialForPrim(*attachedStage, key, desc);
        return;
    }

    omni::physics::usd::UsdSource source(stage);
    setToDefault(source.getSourceUnits(), desc);

    if (materialKey == SdfPath())
        return;
    omni::physics::parse::ParseContext ctx(source, omni::physx::usdparser::iceDescriptorAllocator());
    omni::physics::parse::parseSurfaceDeformableMaterial(ctx, source.keyFor(materialKey), desc);
    desc.poissonsRatio = std::min(desc.poissonsRatio, 0.4999f);
    desc.materialKey = reKeyMaterialPath(stage, materialKey);
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
    omni::physics::parse::DescPtr<PhysxMaterialDesc> parsed =
        omni::physics::parse::parseMaterial(ctx, materialKey);
    if (parsed)
        desc = *parsed;
}

void parseMaterialForPrim(PXR_NS::UsdStageWeakPtr stage, const SdfPath& materialKey, PhysxMaterialDesc& desc)
{
    setToDefault(desc);

    if (materialKey == SdfPath())
        return;
    AttachedStage* attachedStage = attachedStageFor(stage);
    if (attachedStage)
    {
        parseMaterialForPrim(*attachedStage, attachedStage->keyFor(materialKey), desc);
        return;
    }

    omni::physics::usd::UsdSource source(stage);
    omni::physics::parse::ParseContext ctx(source, omni::physx::usdparser::iceDescriptorAllocator());
    omni::physics::parse::DescPtr<PhysxMaterialDesc> parsed =
        omni::physics::parse::parseMaterial(ctx, source.keyFor(materialKey));
    if (parsed)
        desc = *parsed;
    desc.materialKey = reKeyMaterialPath(stage, materialKey);
}

ObjectId getMaterial(AttachedStage& attachedStage, const SdfPath& path)
{
    if (path != SdfPath())
        return attachedStage.getObjectDatabase()->findEntry(path, eMaterial);
    else
        return kInvalidObjectId;
}

ObjectId getMaterial(AttachedStage& attachedStage, const SdfPath& path, const ObjectCategory objCategory)
{
    if (path != SdfPath())
        return attachedStage.getObjectDatabase()->findEntry(path, objCategory);
    else
        return kInvalidObjectId;
}

} // namespace usdparser
} // namespace physx
} // namespace omni
