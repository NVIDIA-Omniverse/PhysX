// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-MAT-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-1 AC-2
 */

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/KnownTokens.h>

#include <cfloat>

namespace omni::physics::parse
{

namespace
{

CombineMode parseCombineMode(TokenId val, const KnownTokens& tok)
{
    if (!val.valid())
        return eAverage;

    if (val == tok.average) return eAverage;
    if (val == tok.min) return eMin;
    if (val == tok.max) return eMax;
    if (val == tok.multiply) return eMultiply;

    return eAverage;
}

template <typename T>
T readScalar(const IPhysicsSource& source, ObjectKey key, TokenId attr, T defaultVal)
{
    T v = defaultVal;
    source.getAttribute(key, attr, v);
    return v;
}

} // anonymous namespace

DescPtr<PhysxMaterialDesc> parseMaterial(ParseContext& ctx, ObjectKey key)
{
    IPhysicsSource& source = ctx.source();
    KnownTokens tok;
    tok.intern(source);

    DescPtr<PhysxMaterialDesc> desc = allocateDesc<PhysxMaterialDesc>(ctx.descriptorAllocator());

    desc->materialKey = key;

    desc->staticFriction = readScalar<float>(source, key, tok.staticFriction, 0.5f);
    desc->dynamicFriction = readScalar<float>(source, key, tok.dynamicFriction, 0.5f);
    desc->restitution = readScalar<float>(source, key, tok.restitution, 0.0f);
    desc->density = readScalar<float>(source, key, tok.density, -1.0f);

    if (source.hasSchema(key, tok.physxMaterialAPI))
    {
        // Read the combine-mode tokens via the typed TokenId overload: some sources
        // (ovstage) store token attrs as an int-encoded token-id column that the raw
        // AttrValue read surfaces as eInt, not eToken; the typed overload resolves
        // both to a source-space TokenId.
        auto readCombineMode = [&](TokenId attr)
        {
            TokenId cm{};
            source.getAttribute(key, attr, cm);
            return parseCombineMode(cm, tok);
        };
        desc->frictionCombineMode = readCombineMode(tok.frictionCombineMode);
        desc->restitutionCombineMode = readCombineMode(tok.restitutionCombineMode);
        desc->dampingCombineMode = readCombineMode(tok.dampingCombineMode);

        desc->compliantAccelerationSpring = readScalar<bool>(
            source, key, tok.compliantContactAccelerationSpring, false);
        // Legacy clamps stiffness/damping to [0, FLT_MAX] via getAttribute.
        float v;
        v = readScalar<float>(source, key, tok.compliantContactStiffness, 0.0f);
        desc->compliantStiffness = v < 0.0f ? 0.0f : v;
        v = readScalar<float>(source, key, tok.compliantContactDamping, 0.0f);
        desc->compliantDamping = v < 0.0f ? 0.0f : v;
    }

    return desc;
}

// ---------------------------------------------------------------------------
// PBD material — `PhysxSchemaPhysxPBDMaterialAPI` reads.
// ---------------------------------------------------------------------------

DescPtr<PBDMaterialDesc> parsePBDMaterial(ParseContext& ctx, ObjectKey key)
{
    IPhysicsSource& source = ctx.source();

    // Local token interning — these parsers fire per-bound-material
    // (rare), not per-prim, so adding to KnownTokens is overkill.
    const TokenId apiTok                   = source.internToken("PhysxPBDMaterialAPI");
    const TokenId frictionTok              = source.internToken("physxPBDMaterial:friction");
    const TokenId particleFrictionScaleTok = source.internToken("physxPBDMaterial:particleFrictionScale");
    const TokenId dampingTok               = source.internToken("physxPBDMaterial:damping");
    const TokenId viscosityTok             = source.internToken("physxPBDMaterial:viscosity");
    const TokenId vorticityConfinementTok  = source.internToken("physxPBDMaterial:vorticityConfinement");
    const TokenId surfaceTensionTok        = source.internToken("physxPBDMaterial:surfaceTension");
    const TokenId cohesionTok              = source.internToken("physxPBDMaterial:cohesion");
    const TokenId adhesionTok              = source.internToken("physxPBDMaterial:adhesion");
    const TokenId particleAdhesionScaleTok = source.internToken("physxPBDMaterial:particleAdhesionScale");
    const TokenId adhesionOffsetScaleTok   = source.internToken("physxPBDMaterial:adhesionOffsetScale");
    const TokenId gravityScaleTok          = source.internToken("physxPBDMaterial:gravityScale");
    const TokenId cflCoefficientTok        = source.internToken("physxPBDMaterial:cflCoefficient");
    const TokenId densityTok               = source.internToken("physxPBDMaterial:density");

    float probe = 0.0f;
    const bool hasPBDMaterialData =
        source.getAttribute(key, frictionTok, probe) ||
        source.getAttribute(key, particleFrictionScaleTok, probe) ||
        source.getAttribute(key, dampingTok, probe) ||
        source.getAttribute(key, viscosityTok, probe) ||
        source.getAttribute(key, vorticityConfinementTok, probe) ||
        source.getAttribute(key, surfaceTensionTok, probe) ||
        source.getAttribute(key, cohesionTok, probe) ||
        source.getAttribute(key, adhesionTok, probe) ||
        source.getAttribute(key, particleAdhesionScaleTok, probe) ||
        source.getAttribute(key, adhesionOffsetScaleTok, probe) ||
        source.getAttribute(key, gravityScaleTok, probe) ||
        source.getAttribute(key, cflCoefficientTok, probe) ||
        source.getAttribute(key, densityTok, probe);
    if (!source.hasSchema(key, apiTok) && !hasPBDMaterialData)
        return {};

    DescPtr<PBDMaterialDesc> desc = allocateDesc<PBDMaterialDesc>(ctx.descriptorAllocator());
    desc->materialKey = key;
    // The non-zero defaults below are required for PhysX validation —
    // setParticleFrictionScale rejects NaN/0 with "invalid float" and
    // setParticleAdhesionScale requires >= 0, so unauthored prims need
    // the seeded values to make it through PxParticleMaterial creation.
    desc->friction              = readScalar<float>(source, key, frictionTok,              0.2f);
    desc->particleFrictionScale = readScalar<float>(source, key, particleFrictionScaleTok, 1.0f);
    desc->damping               = readScalar<float>(source, key, dampingTok,               0.0f);
    desc->viscosity             = readScalar<float>(source, key, viscosityTok,             0.0f);
    desc->vorticityConfinement  = readScalar<float>(source, key, vorticityConfinementTok,  0.0f);
    desc->surfaceTension        = readScalar<float>(source, key, surfaceTensionTok,        0.0f);
    desc->cohesion              = readScalar<float>(source, key, cohesionTok,              0.0f);
    desc->adhesion              = readScalar<float>(source, key, adhesionTok,              0.0f);
    desc->particleAdhesionScale = readScalar<float>(source, key, particleAdhesionScaleTok, 1.0f);
    desc->adhesionOffsetScale   = readScalar<float>(source, key, adhesionOffsetScaleTok,   0.0f);
    desc->gravityScale          = readScalar<float>(source, key, gravityScaleTok,          1.0f);
    desc->cflCoefficient        = readScalar<float>(source, key, cflCoefficientTok,        1.0f);
    desc->density               = readScalar<float>(source, key, densityTok,               1000.0f);

    return desc;
}

// ---------------------------------------------------------------------------
// Deformable material — `OmniPhysicsBaseMaterialAPI` +
// `OmniPhysicsDeformableMaterialAPI` reads (overlay).
// ---------------------------------------------------------------------------

void parseDeformableMaterial(ParseContext& ctx, ObjectKey key, PhysxDeformableMaterialDesc& desc)
{
    IPhysicsSource& source = ctx.source();

    const TokenId baseAPITok  = source.internToken("OmniPhysicsBaseMaterialAPI");
    const TokenId defAPITok   = source.internToken("OmniPhysicsDeformableMaterialAPI");

    desc.materialKey = key;

    if (source.hasSchema(key, baseAPITok))
    {
        const TokenId dynFricTok    = source.internToken("omniphysics:dynamicFriction");
        const TokenId statFricTok   = source.internToken("omniphysics:staticFriction");
        const TokenId densityTok    = source.internToken("omniphysics:density");
        desc.dynamicFriction = readScalar<float>(source, key, dynFricTok,  desc.dynamicFriction);
        desc.staticFriction  = readScalar<float>(source, key, statFricTok, desc.staticFriction);
        desc.density         = readScalar<float>(source, key, densityTok,  desc.density);
    }

    if (source.hasSchema(key, defAPITok))
    {
        const TokenId youngsTok    = source.internToken("omniphysics:youngsModulus");
        const TokenId poissonsTok  = source.internToken("omniphysics:poissonsRatio");
        desc.youngsModulus = readScalar<float>(source, key, youngsTok,   desc.youngsModulus);
        desc.poissonsRatio = readScalar<float>(source, key, poissonsTok, desc.poissonsRatio);
    }

    // PhysX extension API — adds engine-specific fields on top of the Omni
    // base/deformable APIs. Legacy: usdLoad/Material.cpp::parseDeformableMaterialDescInt.
    const TokenId physxDefAPITok = source.internToken("PhysxDeformableMaterialAPI");
    if (source.hasSchema(key, physxDefAPITok))
    {
        const TokenId elasticityDampingTok = source.internToken("physxDeformableMaterial:elasticityDamping");
        float v = readScalar<float>(source, key, elasticityDampingTok, desc.elasticityDamping);
        desc.elasticityDamping = v < 0.0f ? 0.0f : v;
    }
}

// ---------------------------------------------------------------------------
// Surface deformable material — runs the deformable overlay first, then
// `OmniPhysicsSurfaceDeformableMaterialAPI`-specific fields on top.
// ---------------------------------------------------------------------------

void parseSurfaceDeformableMaterial(ParseContext& ctx, ObjectKey key, PhysxSurfaceDeformableMaterialDesc& desc)
{
    parseDeformableMaterial(ctx, key, desc);

    IPhysicsSource& source = ctx.source();
    const TokenId apiTok = source.internToken("OmniPhysicsSurfaceDeformableMaterialAPI");
    if (!source.hasSchema(key, apiTok))
        return;

    const TokenId thicknessTok       = source.internToken("omniphysics:surfaceThickness");
    const TokenId stretchStiffTok    = source.internToken("omniphysics:surfaceStretchStiffness");
    const TokenId shearStiffTok      = source.internToken("omniphysics:surfaceShearStiffness");
    const TokenId bendStiffTok       = source.internToken("omniphysics:surfaceBendStiffness");

    desc.surfaceThickness        = readScalar<float>(source, key, thicknessTok,    desc.surfaceThickness);
    desc.surfaceStretchStiffness = readScalar<float>(source, key, stretchStiffTok, desc.surfaceStretchStiffness);
    desc.surfaceShearStiffness   = readScalar<float>(source, key, shearStiffTok,   desc.surfaceShearStiffness);
    desc.surfaceBendStiffness    = readScalar<float>(source, key, bendStiffTok,    desc.surfaceBendStiffness);

    // PhysX extension API — bendDamping. Legacy: parseSurfaceDeformableMaterialDescInt.
    const TokenId physxSurfaceAPITok = source.internToken("PhysxSurfaceDeformableMaterialAPI");
    if (source.hasSchema(key, physxSurfaceAPITok))
    {
        const TokenId bendDampingTok = source.internToken("physxDeformableMaterial:bendDamping");
        float v = readScalar<float>(source, key, bendDampingTok, desc.bendDamping);
        desc.bendDamping = v < 0.0f ? 0.0f : v;
    }
}

} // namespace omni::physics::parse
