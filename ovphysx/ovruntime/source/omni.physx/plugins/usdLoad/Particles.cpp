// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27 AC-29
 */

#include <carb/Types.h>
#include <carb/Numeric.h>
#include <carb/logging/Log.h>
#include <common/foundation/Allocator.h>

#include "LoadTools.h"
#include "LoadUsd.h"
#include "CollisionGroup.h"
#include "IceDescriptorAllocator.h"
#include "Material.h"
#include "Particles.h"
#include "Mass.h"
#include <propertiesUpdate/PhysXPropertiesUpdate.h>
#include <PhysXDefines.h>

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>

using namespace carb;

namespace omni
{
namespace physx
{
namespace usdparser
{

ParticleSystemDesc* buildParticleSystemDesc(AttachedStage& attachedStage,
    const omni::physics::parse::ScannedStage& scanned, const omni::physics::parse::ParticleSystemDesc& s)
{
    ParticleSystemDesc* d = ICE_PLACEMENT_NEW(ParticleSystemDesc)();

    // systemKey is minted by the SCAN's own source and must be re-keyed into
    // attachedStage's (persistent) namespace via a source-string round trip (ADR-0019
    // increment 7; mirrors the tire-friction-table / articulation `rekey`
    // pattern elsewhere in this file) -- never resolved through the
    // parse-time table directly (ADR-0004 key-space invariant).
    const omni::physics::parse::IPhysicsSource& scanSrc = scanned.source();
    d->systemKey = attachedStage.keyFor(scanSrc.sourceKeyToString(s.systemKey));

    // Scalar / vector data: the parse library already applied the
    // metersPerUnit-aware offset autocompletion + schema lower limits.
    d->enableParticleSystem = s.enableParticleSystem;
    d->enableCCD = s.enableCCD;
    d->restOffset = s.restOffset;
    d->contactOffset = s.contactOffset;
    d->particleContactOffset = s.particleContactOffset;
    d->solidRestOffset = s.solidRestOffset;
    d->fluidRestOffset = s.fluidRestOffset;
    d->maxDepenetrationVelocity = s.maxDepenetrationVelocity;
    d->maxVelocity = s.maxVelocity;
    d->fluidBoundaryDensityScale = s.fluidBoundaryDensityScale;
    d->enableSmoothing = s.enableSmoothing;
    d->enableAnisotropy = s.enableAnisotropy;
    d->enableIsosurface = s.enableIsosurface;
    d->solverPositionIterations = s.solverPositionIterations;
    d->wind = s.wind;
    d->maxNeighborhood = s.maxNeighborhood;
    d->neighborhoodScale = s.neighborhoodScale;
    d->lockedAxis = s.lockedAxis;

    // Cross-refs: simulation owner (scene) + filtered collisions are scanned
    // ObjectKeys; re-key them into attachedStage's namespace the same way.
    // A dropped/mis-sourced/ObjectKey{} re-key here fails silently (Setup.h's
    // getPhysXScene falls back to the default scene) -- see TestParticles.cpp
    // "Particle System Scene Ownership Resolves Second Scene".
    d->sceneKey = s.sceneKey.valid() ? attachedStage.keyFor(scanSrc.sourceKeyToString(s.sceneKey)) : omni::physics::parse::ObjectKey{};
    d->filteredCollisions.clear();
    for (const omni::physics::parse::ObjectKey& k : s.filteredCollisions)
        d->filteredCollisions.push_back(attachedStage.keyFor(scanSrc.sourceKeyToString(k)));

    // Runtime ObjectId resolution + change-tracking registration: these need
    // the engine ObjectDb / time-sampled callback, so they stay consumer-side.
    // Reads go through the source (no direct USD).
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const omni::physics::parse::ObjectKey pkey = d->systemKey;

    d->material = kInvalidObjectId;
    if (src)
    {
        const omni::physics::parse::ObjectKey matKey = src->getMaterialBinding(pkey);
        if (matKey.valid())
            d->material = getMaterial(attachedStage, matKey, ePBDMaterial);
    }

    d->collisionGroup = getCollisionGroup(attachedStage, d->systemKey);

    // Re-register the time-sampled wind callback (engine reader used
    // GetNumTimeSamples() > 1, i.e. isAttributeTimeSampled). ObjectKey/TokenId-native
    // overload (AttachedStage::registerTimeSampledAttribute) -- no SdfPath needed.
    if (src)
    {
        const omni::physics::parse::TokenId windToken = src->internToken("wind");
        if (src->isAttributeTimeSampled(pkey, windToken))
            attachedStage.registerTimeSampledAttribute(pkey, windToken, updateParticleSystemAttribute);
    }

    return d;
}

// Copy the source-neutral data fields (everything except cross-ref paths)
// from a scanned parse descriptor into the engine descriptor. Shared by the
// parse-time (scanned-keyed) and runtime (persistent-keyed) builders.
static void copyParticleSetData(ParticleSetDesc& d, const omni::physics::parse::ParticleSetDesc& s)
{
    d.numParticles = s.numParticles;
    d.enabled = s.enabled;
    d.selfCollision = s.selfCollision;
    d.particleGroup = s.particleGroup;
    d.points = s.points;
    d.velocities = s.velocities;
    d.simulationPoints = s.simulationPoints;
    d.mass = s.mass;
    d.density = s.density;

    d.fluid = s.fluid;
    d.solidRestOffset = s.solidRestOffset;
    d.fluidRestOffset = s.fluidRestOffset;

    d.enableDiffuseParticles = s.enableDiffuseParticles;
    d.maxDiffuseParticleMultiplier = s.maxDiffuseParticleMultiplier;
    d.diffuseParticlesThreshold = s.diffuseParticlesThreshold;
    d.diffuseParticlesLifetime = s.diffuseParticlesLifetime;
    d.diffuseParticlesAirDrag = s.diffuseParticlesAirDrag;
    d.diffuseParticlesBubbleDrag = s.diffuseParticlesBubbleDrag;
    d.diffuseParticlesBuoyancy = s.diffuseParticlesBuoyancy;
    d.diffuseParticlesKineticEnergyWeight = s.diffuseParticlesKineticEnergyWeight;
    d.diffuseParticlesPressureWeight = s.diffuseParticlesPressureWeight;
    d.diffuseParticlesDivergenceWeight = s.diffuseParticlesDivergenceWeight;
    d.diffuseParticlesCollisionDecay = s.diffuseParticlesCollisionDecay;

    d.maxParticles = s.maxParticles;
}

// Replicates the validity gates the engine reader applied before returning
// a set descriptor (the parse library captures raw data unconditionally).
static bool particleSetDataIsValid(const ParticleSetDesc& d)
{
    const size_t pointCount = d.points.size();
    if (!d.velocities.empty() && d.velocities.size() != pointCount)
        return false;
    if (!d.simulationPoints.empty() && d.simulationPoints.size() != pointCount)
        return false;
    if (d.maxParticles < d.numParticles)
        return false;
    return true;
}

ParticleSetDesc* buildParticleSetDesc(AttachedStage& attachedStage,
    const omni::physics::parse::ScannedStage& scanned, const omni::physics::parse::ParticleSetDesc& s)
{
    ParticleSetDesc* d = ICE_PLACEMENT_NEW(ParticleSetDesc)();

    // Keys are minted by the SCAN's own source; re-key into attachedStage's (persistent)
    // namespace via a source-string round trip (ADR-0019 increment 7).
    const omni::physics::parse::IPhysicsSource& scanSrc = scanned.source();
    d->primKey = attachedStage.keyFor(scanSrc.sourceKeyToString(s.primKey));
    d->particleSystemKey = s.particleSystemKey.valid() ? attachedStage.keyFor(scanSrc.sourceKeyToString(s.particleSystemKey)) : omni::physics::parse::ObjectKey{};
    // Same re-key hazard as buildParticleSystemDesc's sceneKey above, though
    // this field's only current consumer (createParticleSet's CUDA-context
    // check + InternalParticleSet::mPhysXScene, both scene-observable but not
    // asserted anywhere) means no test in this tree isolates a regression here.
    d->sceneKey = s.sceneKey.valid() ? attachedStage.keyFor(scanSrc.sourceKeyToString(s.sceneKey)) : omni::physics::parse::ObjectKey{};

    copyParticleSetData(*d, s);

    if (!particleSetDataIsValid(*d))
    {
        CARB_LOG_WARN("PhysxSchemaPhysxParticleSetAPI parsing failed: %s", attachedStage.textFor(d->primKey));
        ICE_FREE(d);
        return nullptr;
    }

    return d;
}

ParticleSetDesc* buildParticleSetDescRuntime(AttachedStage& attachedStage,
    const omni::physics::parse::ParticleSetDesc& s)
{
    // Runtime re-read: keys already come from the persistent source table
    // (attachedStage's own namespace), so no re-keying round-trip is needed
    // here -- a straight copy.
    ParticleSetDesc* d = ICE_PLACEMENT_NEW(ParticleSetDesc)();
    d->primKey = s.primKey;
    d->particleSystemKey = s.particleSystemKey;
    // sceneKey is minted by IPhysicsSource::getRelationshipTargets, which only ever
    // resolves prim-level ObjectKeys, so it needs no path round-trip either -- same
    // straight copy as primKey/particleSystemKey above.
    d->sceneKey = s.sceneKey;
    copyParticleSetData(*d, s);
    return d;
}

void ParseGridFilteringPasses(const std::string& gridFilteringPassesStr, std::vector<ParticleIsosurfaceDesc::GridFilteringPass::Enum>& gridFilteringPasses)
{
    ParticleIsosurfaceDesc::GridFilteringPass::Enum opLast = ParticleIsosurfaceDesc::GridFilteringPass::eNone;
    for (int s = 0; s < gridFilteringPassesStr.length(); ++s)
    {
        ParticleIsosurfaceDesc::GridFilteringPass::Enum op = ParticleIsosurfaceDesc::GridFilteringPass::eNone;
        if (gridFilteringPassesStr[s] == 'S')
        {
            op = ParticleIsosurfaceDesc::GridFilteringPass::eSmooth;
        }
        else if (gridFilteringPassesStr[s] == 'G')
        {
            op = ParticleIsosurfaceDesc::GridFilteringPass::eGrow;
        }
        else if (gridFilteringPassesStr[s] == 'R')
        {
            op = ParticleIsosurfaceDesc::GridFilteringPass::eReduce;
        }
        if (op != opLast)
        {
            gridFilteringPasses.push_back(op);
            opLast = op;
        }
    }
    if (gridFilteringPasses.size() > 32)
    {
        CARB_LOG_WARN("GridFilteringPasses invalid - more than 32 passes, using default \"GSRS\" instead.");
        gridFilteringPasses.clear();
        gridFilteringPasses.push_back({ParticleIsosurfaceDesc::GridFilteringPass::eGrow});
        gridFilteringPasses.push_back({ParticleIsosurfaceDesc::GridFilteringPass::eSmooth});
        gridFilteringPasses.push_back({ParticleIsosurfaceDesc::GridFilteringPass::eReduce});
        gridFilteringPasses.push_back({ParticleIsosurfaceDesc::GridFilteringPass::eSmooth});
    }
}

/*
The following functions complete the particle system *offset USD attributes, and return autocomputed fallback values if the
property sentinels defined in the schema are present. These functions should be used during the lookup of *offset related
USD values whenever it is not possible to read these values directly from the internal physx particle system, for example
before it has been constructed.

The autocomputation right now assumes a hardcoded default fluid particle spacing (that is scaled by the stage meters per
unit). The offsets are inferred based in this value. These defaults have been shown to work reasonably, however
they are not perfect.
*/

const float DEFAULT_PARTICLE_CONTACT_OFFSET = 0.05f;

float completeRestOffset(float metersPerUnit, float restOffset, float particleContactOffset)
{
    if (restOffset < 0.0f)
    {
        particleContactOffset = completeParticleContactOffset(metersPerUnit, particleContactOffset);
        restOffset = particleContactOffset * 0.99f;
    }

    return restOffset;
}

float completeContactOffset(float metersPerUnit, float contactOffset, float particleContactOffset)
{
    if (contactOffset < 0.0f)
    {
        particleContactOffset = completeParticleContactOffset(metersPerUnit, particleContactOffset);
        contactOffset = particleContactOffset;
    }

    return contactOffset;
}

float completeFluidRestOffset(float metersPerUnit, float fluidRestOffset, float particleContactOffset)
{
    if (fluidRestOffset < 0.0f)
    {
        particleContactOffset = completeParticleContactOffset(metersPerUnit, particleContactOffset);
        fluidRestOffset = (particleContactOffset * 0.99f) * 0.6f;
    }

    return fluidRestOffset;
}

float completeSolidRestOffset(float metersPerUnit, float solidRestOffset, float particleContactOffset)
{
    if (solidRestOffset < 0.0f)
    {
        particleContactOffset = completeParticleContactOffset(metersPerUnit, particleContactOffset);
        solidRestOffset = particleContactOffset * 0.99f;
    }

    return solidRestOffset;
}

float completeParticleContactOffset(float metersPerUnit, float particleContactOffset)
{
    if (particleContactOffset <= 0.0f)
    {
        // there needs to be some way to get the default from the schema..
        particleContactOffset = DEFAULT_PARTICLE_CONTACT_OFFSET / metersPerUnit;
    }

    return particleContactOffset;
}
} // namespace usdparser
} // namespace physx
} // namespace omni
