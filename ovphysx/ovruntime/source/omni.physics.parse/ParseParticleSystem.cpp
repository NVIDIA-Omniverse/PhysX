// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-PARSE-PARTICLE-001
 * @covers AC-1
 */

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/PreparedParticle.h>
#include <omni/physics/parse/ParseContext.h>

#include <algorithm>
#include <cfloat>
#include <string>

namespace omni::physics::parse
{

namespace
{
// Mirrors usdparser::DEFAULT_PARTICLE_CONTACT_OFFSET (Particles.cpp).
constexpr float kDefaultParticleContactOffset = 0.05f;

float completeParticleContactOffset(float particleContactOffset, float metersPerUnit)
{
    if (particleContactOffset <= 0.0f) // still set to default if <= 0
        particleContactOffset = kDefaultParticleContactOffset / metersPerUnit;
    return particleContactOffset;
}

} // namespace

PreparedParticleSystem prepareParticleSystem(ParseContext& ctx, ObjectKey key)
{
    IPhysicsSource& src = ctx.source();
    PreparedParticleSystem input;
    input.key = key;
    input.hasPhysxParticleAnisotropyAPI = src.hasSchema(key, src.internToken("PhysxParticleAnisotropyAPI"));
    input.hasPhysxParticleSmoothingAPI = src.hasSchema(key, src.internToken("PhysxParticleSmoothingAPI"));
    input.hasPhysxParticleIsosurfaceAPI = src.hasSchema(key, src.internToken("PhysxParticleIsosurfaceAPI"));
    input.particleContactOffset = prepareAttribute<float>(src, key, src.internToken("particleContactOffset"));
    input.particleContactOffset.authored = src.hasAuthoredAttribute(key, src.internToken("particleContactOffset"));
    input.contactOffset = prepareAttribute<float>(src, key, src.internToken("contactOffset"));
    input.restOffset = prepareAttribute<float>(src, key, src.internToken("restOffset"));
    input.solidRestOffset = prepareAttribute<float>(src, key, src.internToken("solidRestOffset"));
    input.fluidRestOffset = prepareAttribute<float>(src, key, src.internToken("fluidRestOffset"));
    input.maxDepenetrationVelocity = prepareAttribute<float>(src, key, src.internToken("maxDepenetrationVelocity"));
    input.maxVelocity = prepareAttribute<float>(src, key, src.internToken("maxVelocity"));
    input.physxParticle_fluidBoundaryDensityScale = prepareAttribute<float>(src, key, src.internToken("physxParticle:fluidBoundaryDensityScale"));
    input.neighborhoodScale = prepareAttribute<float>(src, key, src.internToken("neighborhoodScale"));
    input.particleSystemEnabled = prepareAttribute<decltype(ParticleSystemDesc{}.enableParticleSystem)>(src, key, src.internToken("particleSystemEnabled"));
    input.enableCCD = prepareAttribute<decltype(ParticleSystemDesc{}.enableCCD)>(src, key, src.internToken("enableCCD"));
    input.solverPositionIterationCount = prepareAttribute<int64_t>(src, key, src.internToken("solverPositionIterationCount"));
    input.wind = prepareAttribute<decltype(ParticleSystemDesc{}.wind)>(src, key, src.internToken("wind"));
    input.maxNeighborhood = prepareAttribute<int64_t>(src, key, src.internToken("maxNeighborhood"));
    input.lockedAxis = prepareAttribute<int64_t>(src, key, src.internToken("lockedAxis"));
    input.physxParticleAnisotropy_particleAnisotropyEnabled = prepareAttribute<decltype(ParticleSystemDesc{}.enableAnisotropy)>(src, key, src.internToken("physxParticleAnisotropy:particleAnisotropyEnabled"));
    input.physxParticleSmoothing_particleSmoothingEnabled = prepareAttribute<decltype(ParticleSystemDesc{}.enableSmoothing)>(src, key, src.internToken("physxParticleSmoothing:particleSmoothingEnabled"));
    input.physxParticleIsosurface_isosurfaceEnabled = prepareAttribute<decltype(ParticleSystemDesc{}.enableIsosurface)>(src, key, src.internToken("physxParticleIsosurface:isosurfaceEnabled"));
    input.metersPerUnit = ctx.units().metersPerUnit;
    std::vector<ObjectKey> owners;
    src.getRelationshipTargets(key, src.internToken("simulationOwner"), owners);
    if (!owners.empty()) input.sceneKey = owners.front();
    src.getRelationshipTargets(key, src.internToken("physics:filteredPairs"), input.filteredCollisions);
    input.prepared = true;
    return input;
}

void prepareParticleSystemStorage(const PreparedParticleSystem& input, ParticleSystemDesc& output)
{
    output.filteredCollisions.resize(input.filteredCollisions.size());
}

bool fillParticleSystem(const PreparedParticleSystem& input, ParticleSystemDesc& output) noexcept
{
    if (!input.prepared) return false;
    if (output.filteredCollisions.size() != input.filteredCollisions.size()) return false;


    // Guard against malformed source units: a non-positive metersPerUnit would
    // propagate inf/NaN through the offset autocompletion divisions below.
    float mpu = input.metersPerUnit;
    if (!(mpu > 0.0f))
        mpu = 1.0f;

    ParticleSystemDesc* d = &output;
    d->systemKey = input.key;



    // particleContactOffset: schema default is divided by mpu when not authored
    // (matches the consumer's HasAuthoredValue branch).
    d->particleContactOffset = input.particleContactOffset.valueOr( 0.0f);
    if (!input.particleContactOffset.authored)
        d->particleContactOffset = d->particleContactOffset / mpu;

    d->enableParticleSystem = true;
    input.particleSystemEnabled.read(d->enableParticleSystem);
    d->enableCCD = false;
    input.enableCCD.read(d->enableCCD);
    d->contactOffset    = input.contactOffset.valueOr( 0.0f);
    d->restOffset       = input.restOffset.valueOr( 0.0f);
    d->solidRestOffset  = input.solidRestOffset.valueOr( 0.0f);
    d->fluidRestOffset  = input.fluidRestOffset.valueOr( 0.0f);

    // Offset autocompletion (order matters — all relative to particleContactOffset).
    d->particleContactOffset = completeParticleContactOffset(d->particleContactOffset, mpu);
    if (d->fluidRestOffset < 0.0f)
        d->fluidRestOffset = (d->particleContactOffset * 0.99f) * 0.6f;
    if (d->restOffset < 0.0f)
        d->restOffset = d->particleContactOffset * 0.99f;
    if (d->contactOffset < 0.0f)
        d->contactOffset = d->particleContactOffset;
    // schema lower limit: contactOffset > restOffset
    if (d->contactOffset <= d->restOffset)
        d->contactOffset = d->restOffset * 1.01f;
    if (d->solidRestOffset < 0.0f)
        d->solidRestOffset = d->particleContactOffset * 0.99f;
    // schema lower limit: particleContactOffset > max(solid, fluid) rest offset
    const float maxRest = std::max(d->solidRestOffset, d->fluidRestOffset);
    if (d->particleContactOffset <= maxRest)
        d->particleContactOffset = maxRest * 1.01f;

    d->maxDepenetrationVelocity = input.maxDepenetrationVelocity.valueOr(
                                            (d->restOffset == 0.0f) ? (0.2f * 30.0f) : d->restOffset * 30.0f);
    d->maxVelocity = input.maxVelocity.valueOr( FLT_MAX);
    d->fluidBoundaryDensityScale =
        input.physxParticle_fluidBoundaryDensityScale.valueOr( 0.0f);

    d->solverPositionIterations = 0;
    {
        int64_t it = 0;
        if (input.solverPositionIterationCount.read(it))
            d->solverPositionIterations = static_cast<int>(it);
    }

    d->wind = { 0.0f, 0.0f, 0.0f };
    input.wind.read(d->wind);

    d->maxNeighborhood = 0;
    d->neighborhoodScale = 0.0f;
    {
        int64_t mn = 0;
        if (input.maxNeighborhood.read(mn))
            d->maxNeighborhood = static_cast<int>(mn);
    }
    d->neighborhoodScale = input.neighborhoodScale.valueOr( 0.0f);

    // lockedAxis: clamp to [0, 7] (matches getAttribute(...,0,7,...) bound).
    d->lockedAxis = 0;
    {
        int64_t la = 0;
        if (input.lockedAxis.read(la))
            d->lockedAxis = static_cast<int>(std::max<int64_t>(0, std::min<int64_t>(7, la)));
    }

    // Sub-API enables (full sub-descriptors land in a later slice).
    d->enableAnisotropy = false;
    d->enableSmoothing = false;
    d->enableIsosurface = false;
    if (input.hasPhysxParticleAnisotropyAPI)
        input.physxParticleAnisotropy_particleAnisotropyEnabled.read(d->enableAnisotropy);
    if (input.hasPhysxParticleSmoothingAPI)
        input.physxParticleSmoothing_particleSmoothingEnabled.read(d->enableSmoothing);
    if (input.hasPhysxParticleIsosurfaceAPI)
        input.physxParticleIsosurface_isosurfaceEnabled.read(d->enableIsosurface);

    // Scene owner (first simulationOwner target), as a source input.key. PhysxParticleSystem
    // authors its OWN "simulationOwner" relationship (PhysxSchemaTokens::simulationOwner,
    // physxParticleSystem.h's CreateSimulationOwnerRel) -- a bare, un-namespaced property
    // distinct from UsdPhysicsRigidBodyAPI/CollisionAPI's "physics:simulationOwner". Reading
    // the latter here always misses (a particle system carries no RigidBodyAPI/CollisionAPI),
    // so an explicit non-default simulationOwner silently fell through to the caller's
    // ObjectKey{} default and landed in the default scene -- see TestParticles.cpp "Particle
    // System Scene Ownership Resolves Second Scene" (issue #30 note 65087537).
    d->sceneKey = input.sceneKey;

    // filteredPairs targets (source keys); runtime material/collisionGroup
    // ObjectId resolution stays consumer-side.
    std::copy(input.filteredCollisions.begin(), input.filteredCollisions.end(), d->filteredCollisions.begin());

    return true;
}

DescPtr<ParticleSystemDesc> parseParticleSystem(ParseContext& ctx, ObjectKey key)
{
    const PreparedParticleSystem input = prepareParticleSystem(ctx, key);
    if (!input.prepared) return {};
    DescPtr<ParticleSystemDesc> output = allocateDesc<ParticleSystemDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareParticleSystemStorage(input, *output);
    if (!fillParticleSystem(input, *output)) return {};
    ctx.particleSystemCache()[key] = { output->sceneKey, output->solidRestOffset, output->fluidRestOffset, output->particleContactOffset };
    return output;
}



namespace
{
// Resolve (and cache) the particle-system fields a set/sampler needs. On a
// cache miss the system is parsed once (which populates the cache); subsequent
// lookups for the same system are free.
const ParticleSystemResolved& resolveParticleSystemFields(ParseContext& ctx, ObjectKey systemKey)
{
    ParticleSystemResolvedCache& cache = ctx.particleSystemCache();
    ParticleSystemResolvedCache::iterator it = cache.find(systemKey);
    if (it == cache.end())
    {
        const PreparedParticleSystem input = prepareParticleSystem(ctx, systemKey);
        ParticleSystemDesc system;
        prepareParticleSystemStorage(input, system);
        if (fillParticleSystem(input, system))
            cache[systemKey] = { system.sceneKey, system.solidRestOffset, system.fluidRestOffset, system.particleContactOffset };
        it = cache.find(systemKey);
        if (it == cache.end()) // system did not resolve; cache an empty record
            it = cache.emplace(systemKey, ParticleSystemResolved{}).first;
    }
    return it->second;
}
} // namespace

PreparedParticleAnisotropy prepareParticleAnisotropy(ParseContext& ctx, ObjectKey key)
{
    IPhysicsSource& src = ctx.source();
    PreparedParticleAnisotropy input;
    input.key = key;
    input.hasPhysxParticleAnisotropyAPI = src.hasSchema(key, src.internToken("PhysxParticleAnisotropyAPI"));
    if (!input.hasPhysxParticleAnisotropyAPI) return input;
    input.physxParticleAnisotropy_scale = prepareAttribute<float>(src, key, src.internToken("physxParticleAnisotropy:scale"));
    input.physxParticleAnisotropy_min = prepareAttribute<float>(src, key, src.internToken("physxParticleAnisotropy:min"));
    input.physxParticleAnisotropy_max = prepareAttribute<float>(src, key, src.internToken("physxParticleAnisotropy:max"));
    input.physxParticleAnisotropy_particleAnisotropyEnabled = prepareAttribute<decltype(ParticleAnisotropyDesc{}.enableAnisotropy)>(src, key, src.internToken("physxParticleAnisotropy:particleAnisotropyEnabled"));
    input.prepared = true;
    return input;
}

void prepareParticleAnisotropyStorage(const PreparedParticleAnisotropy& input, ParticleAnisotropyDesc& output)
{
    (void)input; (void)output;
}

bool fillParticleAnisotropy(const PreparedParticleAnisotropy& input, ParticleAnisotropyDesc& output) noexcept
{
    if (!input.prepared) return false;


    if (!input.hasPhysxParticleAnisotropyAPI)
        return false;
    ParticleAnisotropyDesc* d = &output;
    d->systemKey = input.key;
    input.physxParticleAnisotropy_particleAnisotropyEnabled.read(d->enableAnisotropy);
    d->scale = input.physxParticleAnisotropy_scale.valueOr( 0.0f);
    d->min   = input.physxParticleAnisotropy_min.valueOr( 0.0f);
    d->max   = input.physxParticleAnisotropy_max.valueOr( 0.0f);
    return true;
}

DescPtr<ParticleAnisotropyDesc> parseParticleAnisotropy(ParseContext& ctx, ObjectKey key)
{
    const PreparedParticleAnisotropy input = prepareParticleAnisotropy(ctx, key);
    if (!input.prepared) return {};
    DescPtr<ParticleAnisotropyDesc> output = allocateDesc<ParticleAnisotropyDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareParticleAnisotropyStorage(input, *output);
    if (!fillParticleAnisotropy(input, *output)) return {};
    return output;
}



PreparedParticleSmoothing prepareParticleSmoothing(ParseContext& ctx, ObjectKey key)
{
    IPhysicsSource& src = ctx.source();
    PreparedParticleSmoothing input;
    input.key = key;
    input.hasPhysxParticleSmoothingAPI = src.hasSchema(key, src.internToken("PhysxParticleSmoothingAPI"));
    if (!input.hasPhysxParticleSmoothingAPI) return input;
    input.physxParticleSmoothing_strength = prepareAttribute<float>(src, key, src.internToken("physxParticleSmoothing:strength"));
    input.physxParticleSmoothing_particleSmoothingEnabled = prepareAttribute<decltype(ParticleSmoothingDesc{}.enableSmoothing)>(src, key, src.internToken("physxParticleSmoothing:particleSmoothingEnabled"));
    input.prepared = true;
    return input;
}

void prepareParticleSmoothingStorage(const PreparedParticleSmoothing& input, ParticleSmoothingDesc& output)
{
    (void)input; (void)output;
}

bool fillParticleSmoothing(const PreparedParticleSmoothing& input, ParticleSmoothingDesc& output) noexcept
{
    if (!input.prepared) return false;


    if (!input.hasPhysxParticleSmoothingAPI)
        return false;
    ParticleSmoothingDesc* d = &output;
    d->systemKey = input.key;
    input.physxParticleSmoothing_particleSmoothingEnabled.read(d->enableSmoothing);
    const float strength = input.physxParticleSmoothing_strength.valueOr( 0.0f);
    d->strength = std::min(1.0f, std::max(0.0f, strength)); // clamped to [0, 1]
    return true;
}

DescPtr<ParticleSmoothingDesc> parseParticleSmoothing(ParseContext& ctx, ObjectKey key)
{
    const PreparedParticleSmoothing input = prepareParticleSmoothing(ctx, key);
    if (!input.prepared) return {};
    DescPtr<ParticleSmoothingDesc> output = allocateDesc<ParticleSmoothingDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareParticleSmoothingStorage(input, *output);
    if (!fillParticleSmoothing(input, *output)) return {};
    return output;
}



namespace
{
// Port of usdparser::ParseGridFilteringPasses: each char S/G/R -> a pass,
// collapsing consecutive duplicates; >32 passes resets to the "GSRS" default.
template <typename Emit>
size_t visitGridFilteringPasses(const std::string& text, Emit emit) noexcept
{
    using P = ParticleIsosurfaceDesc::GridFilteringPass;
    P::Enum last = P::eNone;
    size_t count = 0;
    for (char c : text)
    {
        const P::Enum op = c == 'S' ? P::eSmooth : c == 'G' ? P::eGrow : c == 'R' ? P::eReduce : P::eNone;
        if (op != P::eNone && op != last)
        {
            emit(count++, op);
            last = op;
        }
    }
    return count;
}
size_t gridFilteringPassCount(const std::string& text) noexcept
{
    const size_t count = visitGridFilteringPasses(text, [](size_t, ParticleIsosurfaceDesc::GridFilteringPass::Enum) {});
    return count > 32 ? 4 : count;
}
void fillGridFilteringPasses(const std::string& text, std::vector<ParticleIsosurfaceDesc::GridFilteringPass::Enum>& output) noexcept
{
    const size_t count = visitGridFilteringPasses(text, [&](size_t index, ParticleIsosurfaceDesc::GridFilteringPass::Enum op)
    { if (index < output.size()) output[index] = op; });
    if (count > 32)
    {
        using P = ParticleIsosurfaceDesc::GridFilteringPass;
        output[0] = P::eGrow; output[1] = P::eSmooth; output[2] = P::eReduce; output[3] = P::eSmooth;
    }
}


// Read a Vec3f array attribute into a carb::Float3 vector (an eVec3 buffer is 3
// packed floats per element, layout-compatible with carb::Float3). Empty on
// absent/empty attribute.
void readVec3Array(const IPhysicsSource& src, ObjectKey key, TokenId attr, std::vector<carb::Float3>& out)
{
    out.clear();
    const BufferHandle h = src.getArrayAttribute(key, attr, ReadTime::defaultTime());
    size_t byteCount = 0;
    const void* p = src.resolveBuffer(h, byteCount);
    if (p && byteCount >= sizeof(carb::Float3))
    {
        const size_t n = byteCount / sizeof(carb::Float3);
        const carb::Float3* f = static_cast<const carb::Float3*>(p);
        out.assign(f, f + n);
    }
}
} // namespace

PreparedParticleSet prepareParticleSet(ParseContext& ctx, ObjectKey key, bool resolveSystem)
{
    IPhysicsSource& src = ctx.source();
    PreparedParticleSet input;
    input.key = key;
    input.hasPhysxParticleSetAPI = src.hasSchema(key, src.internToken("PhysxParticleSetAPI"));
    input.hasPhysxDiffuseParticlesAPI = src.hasSchema(key, src.internToken("PhysxDiffuseParticlesAPI"));
    if (!input.hasPhysxParticleSetAPI) return input;
    input.physxParticle_particleGroup = prepareAttribute<int>(src, key, src.internToken("physxParticle:particleGroup"));
    input.physxParticle_maxParticles = prepareAttribute<int>(src, key, src.internToken("physxParticle:maxParticles"), true);
    input.physxDiffuseParticles_maxDiffuseParticleMultiplier = prepareAttribute<float>(src, key, src.internToken("physxDiffuseParticles:maxDiffuseParticleMultiplier"));
    input.physxDiffuseParticles_threshold = prepareAttribute<float>(src, key, src.internToken("physxDiffuseParticles:threshold"));
    input.physxDiffuseParticles_lifetime = prepareAttribute<float>(src, key, src.internToken("physxDiffuseParticles:lifetime"));
    input.physxDiffuseParticles_airDrag = prepareAttribute<float>(src, key, src.internToken("physxDiffuseParticles:airDrag"));
    input.physxDiffuseParticles_bubbleDrag = prepareAttribute<float>(src, key, src.internToken("physxDiffuseParticles:bubbleDrag"));
    input.physxDiffuseParticles_buoyancy = prepareAttribute<float>(src, key, src.internToken("physxDiffuseParticles:buoyancy"));
    input.physxDiffuseParticles_kineticEnergyWeight = prepareAttribute<float>(src, key, src.internToken("physxDiffuseParticles:kineticEnergyWeight"));
    input.physxDiffuseParticles_pressureWeight = prepareAttribute<float>(src, key, src.internToken("physxDiffuseParticles:pressureWeight"));
    input.physxDiffuseParticles_divergenceWeight = prepareAttribute<float>(src, key, src.internToken("physxDiffuseParticles:divergenceWeight"));
    input.physxDiffuseParticles_collisionDecay = prepareAttribute<float>(src, key, src.internToken("physxDiffuseParticles:collisionDecay"));
    input.physics_mass = prepareAttribute<float>(src, key, src.internToken("physics:mass"), true);
    input.physics_density = prepareAttribute<float>(src, key, src.internToken("physics:density"), true);
    input.physxParticle_particleEnabled = prepareAttribute<decltype(ParticleSetDesc{}.enabled)>(src, key, src.internToken("physxParticle:particleEnabled"));
    input.physxParticle_selfCollision = prepareAttribute<decltype(ParticleSetDesc{}.selfCollision)>(src, key, src.internToken("physxParticle:selfCollision"));
    input.physxParticle_fluid = prepareAttribute<decltype(ParticleSetDesc{}.fluid)>(src, key, src.internToken("physxParticle:fluid"));
    input.physxDiffuseParticles_diffuseParticlesEnabled = prepareAttribute<decltype(ParticleSetDesc{}.enableDiffuseParticles)>(src, key, src.internToken("physxDiffuseParticles:diffuseParticlesEnabled"));
    const bool instancer = src.isA(key, src.internToken("PointInstancer"));
    readVec3Array(src, key, src.internToken(instancer ? "positions" : "points"), input.points);
    readVec3Array(src, key, src.internToken("velocities"), input.velocities);
    if (src.hasAuthoredAttribute(key, src.internToken("physxParticle:simulationPoints")))
        readVec3Array(src, key, src.internToken("physxParticle:simulationPoints"), input.simulationPoints);
    std::vector<ObjectKey> systems;
    src.getRelationshipTargets(key, src.internToken("physxParticle:particleSystem"), systems);
    if (!systems.empty()) input.systemKey = systems.front();
    if (resolveSystem) resolvePreparedParticleSetSystem(ctx, input);
    input.prepared = true;
    return input;
}

void resolvePreparedParticleSetSystem(ParseContext& ctx, PreparedParticleSet& input)
{
    if (input.systemKey.valid()) input.system = resolveParticleSystemFields(ctx, input.systemKey);
    input.systemResolved = true;
}

void prepareParticleSetStorage(const PreparedParticleSet& input, ParticleSetDesc& output)
{
    output.points.resize(input.points.size());
    output.velocities.resize(input.velocities.empty() ? input.points.size() : input.velocities.size());
    output.simulationPoints.resize(input.simulationPoints.size());
}

bool fillParticleSet(const PreparedParticleSet& input, ParticleSetDesc& output) noexcept
{
    if (!input.prepared || !input.systemResolved || output.points.size() != input.points.size() ||
        output.velocities.size() != (input.velocities.empty() ? input.points.size() : input.velocities.size()) ||
        output.simulationPoints.size() != input.simulationPoints.size()) return false;


    if (!input.hasPhysxParticleSetAPI)
        return false;

    ParticleSetDesc* d = &output;
    d->primKey = input.key;

    std::copy(input.points.begin(), input.points.end(), d->points.begin());
    if (input.velocities.empty())
        std::fill(d->velocities.begin(), d->velocities.end(), carb::Float3{ 0.0f, 0.0f, 0.0f });
    else
        std::copy(input.velocities.begin(), input.velocities.end(), d->velocities.begin());
    std::copy(input.simulationPoints.begin(), input.simulationPoints.end(), d->simulationPoints.begin());

    d->numParticles = static_cast<int>(d->points.size());
    input.physxParticle_particleEnabled.read(d->enabled);
    input.physxParticle_selfCollision.read(d->selfCollision);
    input.physxParticle_fluid.read(d->fluid);
    d->particleGroup = input.physxParticle_particleGroup.valueOr( 0);

    d->particleSystemKey = input.systemKey;

    // maxParticles defaults to the point count; an authored value overrides.
    d->maxParticles = static_cast<int>(d->points.size());
    if (input.physxParticle_maxParticles.authored)
        d->maxParticles = input.physxParticle_maxParticles.valueOr( d->maxParticles);

    // Diffuse particles (inline on the set desc).
    d->enableDiffuseParticles = false;
    d->maxDiffuseParticleMultiplier = 0.0f;
    if (input.hasPhysxDiffuseParticlesAPI)
    {
        input.physxDiffuseParticles_diffuseParticlesEnabled.read(d->enableDiffuseParticles);
        const float mdpm = input.physxDiffuseParticles_maxDiffuseParticleMultiplier.valueOr( 0.0f);
        d->maxDiffuseParticleMultiplier = mdpm < 0.0f ? 1.5f : mdpm;
        d->diffuseParticlesThreshold          = input.physxDiffuseParticles_threshold.valueOr( 0.0f);
        d->diffuseParticlesLifetime           = input.physxDiffuseParticles_lifetime.valueOr( 0.0f);
        d->diffuseParticlesAirDrag            = input.physxDiffuseParticles_airDrag.valueOr( 0.0f);
        d->diffuseParticlesBubbleDrag         = input.physxDiffuseParticles_bubbleDrag.valueOr( 0.0f);
        d->diffuseParticlesBuoyancy           = input.physxDiffuseParticles_buoyancy.valueOr( 0.0f);
        d->diffuseParticlesKineticEnergyWeight = input.physxDiffuseParticles_kineticEnergyWeight.valueOr( 0.0f);
        d->diffuseParticlesPressureWeight     = input.physxDiffuseParticles_pressureWeight.valueOr( 0.0f);
        d->diffuseParticlesDivergenceWeight   = input.physxDiffuseParticles_divergenceWeight.valueOr( 0.0f);
        d->diffuseParticlesCollisionDecay     = input.physxDiffuseParticles_collisionDecay.valueOr( 0.0f);
    }

    // Mass / density overrides (UsdPhysicsMassAPI), authored-only. The -1.0
    // sentinels from the ParticleSetDesc ctor mean "fall back to MassAPI
    // density / material / default density"; an authored value overrides.
    if (input.physics_mass.authored)
        d->mass = input.physics_mass.valueOr( d->mass);
    if (input.physics_density.authored)
        d->density = input.physics_density.valueOr( d->density);

    // Fields copied from the owning particle system (scene + rest offsets),
    // mirroring the engine reader. Resolved through the memoizing cache so the
    // system is parsed at most once per pass.
    if (d->particleSystemKey.valid())
    {
        const ParticleSystemResolved& sys = input.system;
        d->sceneKey = sys.sceneKey;
        d->solidRestOffset = sys.solidRestOffset;
        d->fluidRestOffset = sys.fluidRestOffset;
    }

    return true;
}

DescPtr<ParticleSetDesc> parseParticleSet(ParseContext& ctx, ObjectKey key)
{
    const PreparedParticleSet input = prepareParticleSet(ctx, key);
    if (!input.prepared) return {};
    DescPtr<ParticleSetDesc> output = allocateDesc<ParticleSetDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareParticleSetStorage(input, *output);
    if (!fillParticleSet(input, *output)) return {};
    return output;
}


PreparedParticleIsosurface prepareParticleIsosurface(ParseContext& ctx, ObjectKey key, float fluidRestOffset)
{
    IPhysicsSource& src = ctx.source();
    PreparedParticleIsosurface input;
    input.key = key;
    input.hasPhysxParticleIsosurfaceAPI = src.hasSchema(key, src.internToken("PhysxParticleIsosurfaceAPI"));
    if (!input.hasPhysxParticleIsosurfaceAPI) return input;
    input.physxParticleIsosurface_maxVertices = prepareAttribute<int>(src, key, src.internToken("physxParticleIsosurface:maxVertices"));
    input.physxParticleIsosurface_maxTriangles = prepareAttribute<int>(src, key, src.internToken("physxParticleIsosurface:maxTriangles"));
    input.physxParticleIsosurface_maxSubgrids = prepareAttribute<int>(src, key, src.internToken("physxParticleIsosurface:maxSubgrids"));
    input.physxParticleIsosurface_gridSpacing = prepareAttribute<float>(src, key, src.internToken("physxParticleIsosurface:gridSpacing"));
    input.physxParticleIsosurface_surfaceDistance = prepareAttribute<float>(src, key, src.internToken("physxParticleIsosurface:surfaceDistance"));
    input.physxParticleIsosurface_gridSmoothingRadius = prepareAttribute<float>(src, key, src.internToken("physxParticleIsosurface:gridSmoothingRadius"));
    input.physxParticleIsosurface_numMeshSmoothingPasses = prepareAttribute<int>(src, key, src.internToken("physxParticleIsosurface:numMeshSmoothingPasses"));
    input.physxParticleIsosurface_numMeshNormalSmoothingPasses = prepareAttribute<int>(src, key, src.internToken("physxParticleIsosurface:numMeshNormalSmoothingPasses"));
    input.physxParticleIsosurface_isosurfaceEnabled = prepareAttribute<decltype(ParticleIsosurfaceDesc{}.enableIsosurface)>(src, key, src.internToken("physxParticleIsosurface:isosurfaceEnabled"));
    input.fluidRestOffset = fluidRestOffset;
    src.getAttribute(key, src.internToken("physxParticleIsosurface:gridFilteringPasses"), input.passes);
    input.passCount = gridFilteringPassCount(input.passes);
    input.prepared = true;
    return input;
}

void prepareParticleIsosurfaceStorage(const PreparedParticleIsosurface& input, ParticleIsosurfaceDesc& output)
{
    output.gridFilteringPasses.resize(input.passCount);
}

bool fillParticleIsosurface(const PreparedParticleIsosurface& input, ParticleIsosurfaceDesc& output) noexcept
{
    if (!input.prepared) return false;
    if (output.gridFilteringPasses.size() != input.passCount) return false;


    if (!input.hasPhysxParticleIsosurfaceAPI)
        return false;
    ParticleIsosurfaceDesc* d = &output;
    d->systemKey = input.key;
    input.physxParticleIsosurface_isosurfaceEnabled.read(d->enableIsosurface);
    d->maxIsosurfaceVertices    = input.physxParticleIsosurface_maxVertices.valueOr( 0);
    d->maxIsosurfaceTriangles   = input.physxParticleIsosurface_maxTriangles.valueOr( 0);
    d->maxNumIsosurfaceSubgrids = input.physxParticleIsosurface_maxSubgrids.valueOr( 0);

    d->gridSpacing = input.physxParticleIsosurface_gridSpacing.valueOr( 0.0f);
    if (d->gridSpacing < 0.0f)
        d->gridSpacing = input.fluidRestOffset * 1.5f; // autocompute
    else if (d->gridSpacing <= input.fluidRestOffset * 0.9f)
        d->gridSpacing = input.fluidRestOffset * 0.9f;

    d->surfaceDistance = input.physxParticleIsosurface_surfaceDistance.valueOr( 0.0f);
    if (d->surfaceDistance < 0.0f)
        d->surfaceDistance = input.fluidRestOffset * 1.6f; // autocompute
    else if (d->surfaceDistance > 2.5f * d->gridSpacing)
        d->surfaceDistance = d->gridSpacing * 2.5f;

    fillGridFilteringPasses(input.passes, d->gridFilteringPasses);

    d->gridSmoothingRadius = input.physxParticleIsosurface_gridSmoothingRadius.valueOr( 0.0f);
    if (d->gridSmoothingRadius < 0.0f)
        d->gridSmoothingRadius = input.fluidRestOffset * 2.0f; // autocompute

    d->numMeshSmoothingPasses =
        input.physxParticleIsosurface_numMeshSmoothingPasses.valueOr( 0);
    d->numMeshNormalSmoothingPasses =
        input.physxParticleIsosurface_numMeshNormalSmoothingPasses.valueOr( 0);
    return true;
}

DescPtr<ParticleIsosurfaceDesc> parseParticleIsosurface(ParseContext& ctx, ObjectKey key, float fluidRestOffset)
{
    const PreparedParticleIsosurface input = prepareParticleIsosurface(ctx, key, fluidRestOffset);
    if (!input.prepared) return {};
    DescPtr<ParticleIsosurfaceDesc> output = allocateDesc<ParticleIsosurfaceDesc>(ctx.descriptorAllocator());
    if (!output) return {};
    prepareParticleIsosurfaceStorage(input, *output);
    if (!fillParticleIsosurface(input, *output)) return {};
    return output;
}



PreparedParticleSampling prepareParticleSampling(ParseContext& ctx, ObjectKey key)
{
    IPhysicsSource& src = ctx.source();
    PreparedParticleSampling input;
    input.key = key;
    if (!src.isA(key, src.internToken("Mesh")) || !src.hasSchema(key, src.internToken("PhysxParticleSamplingAPI")))
        return input;
    input.samplingDistance = prepareAttribute<float>(src, key, src.internToken("physxParticleSampling:samplingDistance"));
    input.maxSamples = prepareAttribute<int>(src, key, src.internToken("physxParticleSampling:maxSamples"));
    input.volume = prepareAttribute<bool>(src, key, src.internToken("physxParticleSampling:volume"));
    std::vector<ObjectKey> sets;
    src.getRelationshipTargets(key, src.internToken("physxParticleSampling:particles"), sets);
    if (!sets.empty()) input.setKey = sets.front();
    if (input.setKey.valid() && src.hasSchema(input.setKey, src.internToken("PhysxParticleSetAPI")))
    {
        // The default mirrors ParticleSetDesc's fluid default. No particle data
        // arrays or temporary set descriptors are required for this dependency.
        input.fluid = prepareAttribute<bool>(src, input.setKey, src.internToken("physxParticle:fluid")).valueOr(ParticleSetDesc{}.fluid);
        std::vector<ObjectKey> systems;
        src.getRelationshipTargets(input.setKey, src.internToken("physxParticle:particleSystem"), systems);
        input.validSet = true;
        if (!systems.empty() && systems.front().valid())
        {
            input.system = resolveParticleSystemFields(ctx, systems.front());
            input.hasSystem = true;
        }
        else
        {
            input.system.solidRestOffset = ParticleSetDesc{}.solidRestOffset;
            input.system.fluidRestOffset = ParticleSetDesc{}.fluidRestOffset;
        }
    }
    input.prepared = true;
    return input;
}

bool fillParticleSampling(const PreparedParticleSampling& input, ParticleSamplingDesc& output) noexcept
{
    if (!input.prepared) return false;
    output.samplingDistance = input.samplingDistance.valueOr(-1.0f);
    output.maxSamples = input.maxSamples.valueOr(0);
    output.sampleVolume = input.volume.valueOr(true);
    output.particleSetKey = input.setKey;
    float pointWidth = 0.5f;
    if (input.validSet)
    {
        pointWidth = input.fluid ? 2.0f * input.system.fluidRestOffset : 2.0f * input.system.solidRestOffset;
        if (pointWidth == 0.0f && input.hasSystem) pointWidth = input.system.particleContactOffset;
    }
    output.pointWidth = pointWidth;
    if (output.samplingDistance <= 0.0f) output.samplingDistance = pointWidth;
    if (output.samplingDistance <= 0.75f * pointWidth) output.samplingDistance = 0.75f * pointWidth;
    return true;
}

DescPtr<ParticleSamplingDesc> parseParticleSampling(ParseContext& ctx, ObjectKey key)
{
    const PreparedParticleSampling input = prepareParticleSampling(ctx, key);
    if (!input.prepared) return {};
    DescPtr<ParticleSamplingDesc> output = allocateDesc<ParticleSamplingDesc>(ctx.descriptorAllocator());
    if (output) fillParticleSampling(input, *output);
    return output;
}

} // namespace omni::physics::parse
