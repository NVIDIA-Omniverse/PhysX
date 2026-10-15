// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-PARSE-MAT-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-1 AC-2
 */

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedMaterial.h>
#include <omni/physics/parse/KnownTokens.h>

#include <cfloat>

namespace omni::physics::parse
{

namespace
{

CombineMode parseCombineMode(TokenId val, const PreparedMaterial& tok)
{
    if (!val.valid())
        return eAverage;

    if (val == tok.average) return eAverage;
    if (val == tok.min) return eMin;
    if (val == tok.max) return eMax;
    if (val == tok.multiply) return eMultiply;

    return eAverage;
}

} // anonymous namespace

DescPtr<PhysxMaterialDesc> parseMaterial(ParseContext& ctx, ObjectKey key)
{
    DescPtr<PhysxMaterialDesc> desc = allocateDesc<PhysxMaterialDesc>(ctx.descriptorAllocator());
    if (desc)
        fillMaterial(prepareMaterial(ctx, key), *desc);
    return desc;
}

PreparedMaterial prepareMaterial(ParseContext& ctx, ObjectKey key)
{
    const IPhysicsSource& source = ctx.source();
    const KnownTokens& tok = ctx.knownTokens();
    PreparedMaterial input;
    input.key = key;
    input.average = tok.average;
    input.min = tok.min;
    input.max = tok.max;
    input.multiply = tok.multiply;
    input.staticFriction = prepareAttribute<float>(source, key, tok.staticFriction);
    input.dynamicFriction = prepareAttribute<float>(source, key, tok.dynamicFriction);
    input.restitution = prepareAttribute<float>(source, key, tok.restitution);
    input.density = prepareAttribute<float>(source, key, tok.density);
    input.physxMaterialAPI = source.hasSchema(key, tok.physxMaterialAPI);
    if (input.physxMaterialAPI)
    {
        input.frictionCombineMode = prepareAttribute<TokenId>(source, key, tok.frictionCombineMode);
        input.restitutionCombineMode = prepareAttribute<TokenId>(source, key, tok.restitutionCombineMode);
        input.dampingCombineMode = prepareAttribute<TokenId>(source, key, tok.dampingCombineMode);
        input.compliantContactAccelerationSpring = prepareAttribute<bool>(source, key, tok.compliantContactAccelerationSpring);
        input.compliantContactStiffness = prepareAttribute<float>(source, key, tok.compliantContactStiffness);
        input.compliantContactDamping = prepareAttribute<float>(source, key, tok.compliantContactDamping);
    }
    input.prepared = true;
    return input;
}

bool fillMaterial(const PreparedMaterial& input, PhysxMaterialDesc& output) noexcept
{
    if (!input.prepared)
        return false;
    PhysxMaterialDesc* desc = &output;
    desc->materialKey = input.key;

    desc->staticFriction = input.staticFriction.valueOr(0.5f);
    desc->dynamicFriction = input.dynamicFriction.valueOr(0.5f);
    desc->restitution = input.restitution.valueOr(0.0f);
    desc->density = input.density.valueOr(-1.0f);

    if (input.physxMaterialAPI)
    {
        // Read the combine-mode tokens via the typed TokenId overload: some sources
        // (ovstage) store token attrs as an int-encoded token-id column that the raw
        // AttrValue read surfaces as eInt, not eToken; the typed overload resolves
        // both to a source-space TokenId.
        desc->frictionCombineMode = parseCombineMode(input.frictionCombineMode.valueOr({}), input);
        desc->restitutionCombineMode = parseCombineMode(input.restitutionCombineMode.valueOr({}), input);
        desc->dampingCombineMode = parseCombineMode(input.dampingCombineMode.valueOr({}), input);

        desc->compliantAccelerationSpring = input.compliantContactAccelerationSpring.valueOr(false);
        // Legacy clamps stiffness/damping to [0, FLT_MAX] via getAttribute.
        float v;
        v = input.compliantContactStiffness.valueOr(0.0f);
        desc->compliantStiffness = v < 0.0f ? 0.0f : v;
        v = input.compliantContactDamping.valueOr(0.0f);
        desc->compliantDamping = v < 0.0f ? 0.0f : v;
    }

    return true;
}

// ---------------------------------------------------------------------------
// PBD material — `PhysxSchemaPhysxPBDMaterialAPI` reads.
// ---------------------------------------------------------------------------

DescPtr<PBDMaterialDesc> parsePBDMaterial(ParseContext& ctx, ObjectKey key)
{
    const PreparedPBDMaterial input = preparePBDMaterial(ctx, key);
    if (!input.enabled) return {};
    DescPtr<PBDMaterialDesc> desc = allocateDesc<PBDMaterialDesc>(ctx.descriptorAllocator());
    if (desc) fillPBDMaterial(input, *desc);
    return desc;
}

PreparedPBDMaterial preparePBDMaterial(ParseContext& ctx, ObjectKey key)
{
    IPhysicsSource& source = ctx.source();
    PreparedPBDMaterial input;
    input.key = key;
    input.enabled = source.hasSchema(key, source.internToken("PhysxPBDMaterialAPI"));
    input.friction = prepareAttribute<float>(source, key, source.internToken("physxPBDMaterial:friction"));
    input.enabled = input.enabled || input.friction.valid;
    input.particleFrictionScale = prepareAttribute<float>(source, key, source.internToken("physxPBDMaterial:particleFrictionScale"));
    input.enabled = input.enabled || input.particleFrictionScale.valid;
    input.damping = prepareAttribute<float>(source, key, source.internToken("physxPBDMaterial:damping"));
    input.enabled = input.enabled || input.damping.valid;
    input.viscosity = prepareAttribute<float>(source, key, source.internToken("physxPBDMaterial:viscosity"));
    input.enabled = input.enabled || input.viscosity.valid;
    input.vorticityConfinement = prepareAttribute<float>(source, key, source.internToken("physxPBDMaterial:vorticityConfinement"));
    input.enabled = input.enabled || input.vorticityConfinement.valid;
    input.surfaceTension = prepareAttribute<float>(source, key, source.internToken("physxPBDMaterial:surfaceTension"));
    input.enabled = input.enabled || input.surfaceTension.valid;
    input.cohesion = prepareAttribute<float>(source, key, source.internToken("physxPBDMaterial:cohesion"));
    input.enabled = input.enabled || input.cohesion.valid;
    input.adhesion = prepareAttribute<float>(source, key, source.internToken("physxPBDMaterial:adhesion"));
    input.enabled = input.enabled || input.adhesion.valid;
    input.particleAdhesionScale = prepareAttribute<float>(source, key, source.internToken("physxPBDMaterial:particleAdhesionScale"));
    input.enabled = input.enabled || input.particleAdhesionScale.valid;
    input.adhesionOffsetScale = prepareAttribute<float>(source, key, source.internToken("physxPBDMaterial:adhesionOffsetScale"));
    input.enabled = input.enabled || input.adhesionOffsetScale.valid;
    input.gravityScale = prepareAttribute<float>(source, key, source.internToken("physxPBDMaterial:gravityScale"));
    input.enabled = input.enabled || input.gravityScale.valid;
    input.cflCoefficient = prepareAttribute<float>(source, key, source.internToken("physxPBDMaterial:cflCoefficient"));
    input.enabled = input.enabled || input.cflCoefficient.valid;
    input.density = prepareAttribute<float>(source, key, source.internToken("physxPBDMaterial:density"));
    input.enabled = input.enabled || input.density.valid;
    input.prepared = true;
    return input;
}

bool fillPBDMaterial(const PreparedPBDMaterial& input, PBDMaterialDesc& desc) noexcept
{
    if (!input.prepared || !input.enabled) return false;
    desc.materialKey = input.key;
    desc.friction = input.friction.valueOr(0.2f);
    desc.particleFrictionScale = input.particleFrictionScale.valueOr(1.0f);
    desc.damping = input.damping.valueOr(0.0f);
    desc.viscosity = input.viscosity.valueOr(0.0f);
    desc.vorticityConfinement = input.vorticityConfinement.valueOr(0.0f);
    desc.surfaceTension = input.surfaceTension.valueOr(0.0f);
    desc.cohesion = input.cohesion.valueOr(0.0f);
    desc.adhesion = input.adhesion.valueOr(0.0f);
    desc.particleAdhesionScale = input.particleAdhesionScale.valueOr(1.0f);
    desc.adhesionOffsetScale = input.adhesionOffsetScale.valueOr(0.0f);
    desc.gravityScale = input.gravityScale.valueOr(1.0f);
    desc.cflCoefficient = input.cflCoefficient.valueOr(1.0f);
    desc.density = input.density.valueOr(1000.0f);
    return true;
}

// ---------------------------------------------------------------------------
// Deformable material — `OmniPhysicsBaseMaterialAPI` +
// `OmniPhysicsDeformableMaterialAPI` reads (overlay).
// ---------------------------------------------------------------------------

PreparedDeformableMaterial prepareDeformableMaterial(ParseContext& ctx, ObjectKey key, bool surface)
{
    IPhysicsSource& source = ctx.source();
    PreparedDeformableMaterial input;
    input.key = key;
    if (source.hasSchema(key, source.internToken("OmniPhysicsBaseMaterialAPI")))
    {
        input.dynamicFriction = prepareAttribute<float>(source, key, source.internToken("omniphysics:dynamicFriction"));
        input.staticFriction = prepareAttribute<float>(source, key, source.internToken("omniphysics:staticFriction"));
        input.density = prepareAttribute<float>(source, key, source.internToken("omniphysics:density"));
    }
    if (source.hasSchema(key, source.internToken("OmniPhysicsDeformableMaterialAPI")))
    {
        input.youngsModulus = prepareAttribute<float>(source, key, source.internToken("omniphysics:youngsModulus"));
        input.poissonsRatio = prepareAttribute<float>(source, key, source.internToken("omniphysics:poissonsRatio"));
    }
    if (source.hasSchema(key, source.internToken("PhysxDeformableMaterialAPI")))
    {
        input.elasticity = true;
        input.elasticityDamping = prepareAttribute<float>(source, key, source.internToken("physxDeformableMaterial:elasticityDamping"));
    }
    input.surface = surface && source.hasSchema(key, source.internToken("OmniPhysicsSurfaceDeformableMaterialAPI"));
    if (input.surface)
    {
        input.surfaceThickness = prepareAttribute<float>(source, key, source.internToken("omniphysics:surfaceThickness"));
        input.surfaceStretchStiffness = prepareAttribute<float>(source, key, source.internToken("omniphysics:surfaceStretchStiffness"));
        input.surfaceShearStiffness = prepareAttribute<float>(source, key, source.internToken("omniphysics:surfaceShearStiffness"));
        input.surfaceBendStiffness = prepareAttribute<float>(source, key, source.internToken("omniphysics:surfaceBendStiffness"));
        if (source.hasSchema(key, source.internToken("PhysxSurfaceDeformableMaterialAPI")))
        {
        input.bend = true;
        input.bendDamping = prepareAttribute<float>(source, key, source.internToken("physxDeformableMaterial:bendDamping"));
        }
    }
    input.prepared = true;
    return input;
}

bool fillDeformableMaterial(const PreparedDeformableMaterial& input, PhysxDeformableMaterialDesc& desc) noexcept
{
    if (!input.prepared) return false;
    desc.materialKey = input.key;
    input.dynamicFriction.read(desc.dynamicFriction);
    input.staticFriction.read(desc.staticFriction);
    input.density.read(desc.density);
    input.youngsModulus.read(desc.youngsModulus);
    input.poissonsRatio.read(desc.poissonsRatio);
    input.elasticityDamping.read(desc.elasticityDamping);
    if (input.elasticity) desc.elasticityDamping = desc.elasticityDamping < 0.0f ? 0.0f : desc.elasticityDamping;
    return true;
}

bool fillSurfaceDeformableMaterial(const PreparedDeformableMaterial& input, PhysxSurfaceDeformableMaterialDesc& desc) noexcept
{
    if (!fillDeformableMaterial(input, desc)) return false;
    if (!input.surface) return true;
    input.surfaceThickness.read(desc.surfaceThickness);
    input.surfaceStretchStiffness.read(desc.surfaceStretchStiffness);
    input.surfaceShearStiffness.read(desc.surfaceShearStiffness);
    input.surfaceBendStiffness.read(desc.surfaceBendStiffness);
    input.bendDamping.read(desc.bendDamping);
    if (input.bend) desc.bendDamping = desc.bendDamping < 0.0f ? 0.0f : desc.bendDamping;
    return true;
}

void parseDeformableMaterial(ParseContext& ctx, ObjectKey key, PhysxDeformableMaterialDesc& desc)
{
    fillDeformableMaterial(prepareDeformableMaterial(ctx, key, false), desc);
}
void parseSurfaceDeformableMaterial(ParseContext& ctx, ObjectKey key, PhysxSurfaceDeformableMaterialDesc& desc)
{
    fillSurfaceDeformableMaterial(prepareDeformableMaterial(ctx, key, true), desc);
}

} // namespace omni::physics::parse
