// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-DEF-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-6
 */

// Deformable body root-prim attribute reader. Reads the per-prim
// attrs + rels only — hierarchy walk, sim-mesh classification,
// material-binding resolution, and multi-apply pose-purpose token
// discovery happen on the caller side (NativeWalker).

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/KnownTokens.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>

#include <cfloat>
#include <cmath>
#include <cstdint>

namespace omni::physics::parse
{

// Units-aware overlay only. All other defaults are in-class on the
// descriptor (Descriptors.h). Stage-aware defaults are still required:
// leaving fields at value-init garbage (e.g. `autoRemeshingResolution`)
// would feed the cooking pipeline a bogus grid resolution and stall the
// remesher indefinitely — the in-class initialisers cover that.
void setToDefault(PhysxDeformableBodyDesc& desc, const SourceUnits& units)
{
    const float metersPerUnit = units.metersPerUnit;
    const float tolerancesSpeed = 10.0f / metersPerUnit;

    desc.sleepThreshold           = 5e-5f * tolerancesSpeed * tolerancesSpeed;
    desc.settlingThreshold        = 0.1f  / metersPerUnit;
    desc.maxDepenetrationVelocity = 3.0f  / metersPerUnit;
    desc.restOffset               = 0.02f / metersPerUnit;
}

void setToDefault(PhysxVolumeDeformableBodyDesc& desc, const SourceUnits& units)
{
    setToDefault(static_cast<PhysxDeformableBodyDesc&>(desc), units);
}

void setToDefault(PhysxSurfaceDeformableBodyDesc& desc, const SourceUnits& units)
{
    setToDefault(static_cast<PhysxDeformableBodyDesc&>(desc), units);
}

namespace
{

bool readBoolAttr(const IPhysicsSource& src, ObjectKey key, TokenId attr, bool defaultVal)
{
    bool v = defaultVal;
    src.getAttribute(key, attr, v);
    return v;
}

float readFloatAttr(const IPhysicsSource& src, ObjectKey key, TokenId attr, float defaultVal)
{
    float v = defaultVal;
    src.getAttribute(key, attr, v);
    return v;
}

} // anonymous

DeformableBodyParse parseDeformableBody(ParseContext& ctx, ObjectKey key)
{
    DeformableBodyParse out;

    KnownTokens tok;
    tok.intern(ctx.source());

    IPhysicsSource& src = ctx.source();

    if (src.hasSchema(key, tok.omniphysicsDeformableBodyAPI))
    {
        out.bodyEnabled = readBoolAttr(src, key, tok.omniphysicsDeformableBodyEnabled, false);
        out.mass        = readFloatAttr(src, key, tok.omniphysicsMass, 0.0f);
    }

    if (src.hasSchema(key, tok.omniphysicsBodyAPI))
    {
        out.kinematicBody = readBoolAttr(src, key, tok.omniphysicsKinematicEnabled, false);
        out.startsAsleep  = readBoolAttr(src, key, tok.omniphysicsStartsAsleep, false);
        src.getRelationshipTargets(key, tok.omniphysicsSimulationOwner, out.simulationOwners);
    }

    out.filteredCollisions = parseFilteredPairs(ctx, key);
    return out;
}

} // namespace omni::physics::parse
