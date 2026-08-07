// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
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

float readFloat(const IPhysicsSource& src, ObjectKey key, TokenId attr, float defaultVal)
{
    float v = defaultVal;
    src.getAttribute(key, attr, v);
    return v;
}
} // namespace

DescPtr<ParticleSystemDesc> parseParticleSystem(ParseContext& ctx, ObjectKey key)
{
    IPhysicsSource& src = ctx.source();
    // Guard against malformed source units: a non-positive metersPerUnit would
    // propagate inf/NaN through the offset autocompletion divisions below.
    float mpu = ctx.units().metersPerUnit;
    if (!(mpu > 0.0f))
        mpu = 1.0f;

    DescPtr<ParticleSystemDesc> d = allocateDesc<ParticleSystemDesc>(ctx.descriptorAllocator());
    d->systemKey = key;

    const TokenId tParticleContactOffset = src.internToken("particleContactOffset");

    // particleContactOffset: schema default is divided by mpu when not authored
    // (matches the consumer's HasAuthoredValue branch).
    d->particleContactOffset = readFloat(src, key, tParticleContactOffset, 0.0f);
    if (!src.hasAuthoredAttribute(key, tParticleContactOffset))
        d->particleContactOffset = d->particleContactOffset / mpu;

    d->enableParticleSystem = true;
    src.getAttribute(key, src.internToken("particleSystemEnabled"), d->enableParticleSystem);
    d->enableCCD = false;
    src.getAttribute(key, src.internToken("enableCCD"), d->enableCCD);
    d->contactOffset    = readFloat(src, key, src.internToken("contactOffset"), 0.0f);
    d->restOffset       = readFloat(src, key, src.internToken("restOffset"), 0.0f);
    d->solidRestOffset  = readFloat(src, key, src.internToken("solidRestOffset"), 0.0f);
    d->fluidRestOffset  = readFloat(src, key, src.internToken("fluidRestOffset"), 0.0f);

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

    d->maxDepenetrationVelocity = readFloat(src, key, src.internToken("maxDepenetrationVelocity"),
                                            (d->restOffset == 0.0f) ? (0.2f * 30.0f) : d->restOffset * 30.0f);
    d->maxVelocity = readFloat(src, key, src.internToken("maxVelocity"), FLT_MAX);
    d->fluidBoundaryDensityScale =
        readFloat(src, key, src.internToken("physxParticle:fluidBoundaryDensityScale"), 0.0f);

    d->solverPositionIterations = 0;
    {
        int64_t it = 0;
        if (src.getAttribute(key, src.internToken("solverPositionIterationCount"), it))
            d->solverPositionIterations = static_cast<int>(it);
    }

    d->wind = { 0.0f, 0.0f, 0.0f };
    src.getAttribute(key, src.internToken("wind"), d->wind);

    d->maxNeighborhood = 0;
    d->neighborhoodScale = 0.0f;
    {
        int64_t mn = 0;
        if (src.getAttribute(key, src.internToken("maxNeighborhood"), mn))
            d->maxNeighborhood = static_cast<int>(mn);
    }
    d->neighborhoodScale = readFloat(src, key, src.internToken("neighborhoodScale"), 0.0f);

    // lockedAxis: clamp to [0, 7] (matches getAttribute(...,0,7,...) bound).
    d->lockedAxis = 0;
    {
        int64_t la = 0;
        if (src.getAttribute(key, src.internToken("lockedAxis"), la))
            d->lockedAxis = static_cast<int>(std::max<int64_t>(0, std::min<int64_t>(7, la)));
    }

    // Sub-API enables (full sub-descriptors land in a later slice).
    d->enableAnisotropy = false;
    d->enableSmoothing = false;
    d->enableIsosurface = false;
    if (src.hasSchema(key, src.internToken("PhysxParticleAnisotropyAPI")))
        src.getAttribute(key, src.internToken("physxParticleAnisotropy:particleAnisotropyEnabled"), d->enableAnisotropy);
    if (src.hasSchema(key, src.internToken("PhysxParticleSmoothingAPI")))
        src.getAttribute(key, src.internToken("physxParticleSmoothing:particleSmoothingEnabled"), d->enableSmoothing);
    if (src.hasSchema(key, src.internToken("PhysxParticleIsosurfaceAPI")))
        src.getAttribute(key, src.internToken("physxParticleIsosurface:isosurfaceEnabled"), d->enableIsosurface);

    // Scene owner (first simulationOwner target), as a source key.
    d->sceneKey = ObjectKey{};
    {
        std::vector<ObjectKey> owners;
        src.getRelationshipTargets(key, src.internToken("physics:simulationOwner"), owners);
        if (!owners.empty())
            d->sceneKey = owners.front();
    }

    // filteredPairs targets (source keys); runtime material/collisionGroup
    // ObjectId resolution stays consumer-side.
    d->filteredCollisions.clear();
    src.getRelationshipTargets(key, src.internToken("physics:filteredPairs"), d->filteredCollisions);

    // Memoize the scalar fields downstream sets/samplers copy, so the system is
    // parsed at most once per pass instead of re-read per set/sampler.
    ctx.particleSystemCache()[key] = { d->sceneKey, d->solidRestOffset, d->fluidRestOffset, d->particleContactOffset };

    return d;
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
        parseParticleSystem(ctx, systemKey); // side effect: inserts into cache
        it = cache.find(systemKey);
        if (it == cache.end()) // system did not resolve; cache an empty record
            it = cache.emplace(systemKey, ParticleSystemResolved{}).first;
    }
    return it->second;
}
} // namespace

DescPtr<ParticleAnisotropyDesc> parseParticleAnisotropy(ParseContext& ctx, ObjectKey key)
{
    IPhysicsSource& src = ctx.source();
    if (!src.hasSchema(key, src.internToken("PhysxParticleAnisotropyAPI")))
        return {};
    DescPtr<ParticleAnisotropyDesc> d = allocateDesc<ParticleAnisotropyDesc>(ctx.descriptorAllocator());
    d->systemKey = key;
    src.getAttribute(key, src.internToken("physxParticleAnisotropy:particleAnisotropyEnabled"), d->enableAnisotropy);
    d->scale = readFloat(src, key, src.internToken("physxParticleAnisotropy:scale"), 0.0f);
    d->min   = readFloat(src, key, src.internToken("physxParticleAnisotropy:min"), 0.0f);
    d->max   = readFloat(src, key, src.internToken("physxParticleAnisotropy:max"), 0.0f);
    return d;
}

DescPtr<ParticleSmoothingDesc> parseParticleSmoothing(ParseContext& ctx, ObjectKey key)
{
    IPhysicsSource& src = ctx.source();
    if (!src.hasSchema(key, src.internToken("PhysxParticleSmoothingAPI")))
        return {};
    DescPtr<ParticleSmoothingDesc> d = allocateDesc<ParticleSmoothingDesc>(ctx.descriptorAllocator());
    d->systemKey = key;
    src.getAttribute(key, src.internToken("physxParticleSmoothing:particleSmoothingEnabled"), d->enableSmoothing);
    const float strength = readFloat(src, key, src.internToken("physxParticleSmoothing:strength"), 0.0f);
    d->strength = std::min(1.0f, std::max(0.0f, strength)); // GfClamp(strength, 0, 1)
    return d;
}

namespace
{
// Port of usdparser::ParseGridFilteringPasses: each char S/G/R -> a pass,
// collapsing consecutive duplicates; >32 passes resets to the "GSRS" default.
void parseGridFilteringPasses(const std::string& s, std::vector<ParticleIsosurfaceDesc::GridFilteringPass::Enum>& out)
{
    using P = ParticleIsosurfaceDesc::GridFilteringPass;
    P::Enum last = P::eNone;
    for (char c : s)
    {
        P::Enum op;
        if (c == 'S') op = P::eSmooth;
        else if (c == 'G') op = P::eGrow;
        else if (c == 'R') op = P::eReduce;
        else continue; // ignore unsupported characters (never emit an eNone pass)
        if (op != last)
        {
            out.push_back(op);
            last = op;
        }
    }
    if (out.size() > 32)
    {
        out.clear();
        out.push_back(P::eGrow);
        out.push_back(P::eSmooth);
        out.push_back(P::eReduce);
        out.push_back(P::eSmooth);
    }
}

int readInt(const IPhysicsSource& src, ObjectKey key, TokenId attr, int defaultVal)
{
    int64_t v = defaultVal;
    src.getAttribute(key, attr, v);
    return static_cast<int>(v);
}

// Read a Vec3f array attribute into a carb::Float3 vector (GfVec3f and
// carb::Float3 share the 3-float layout). Empty on absent/empty attribute.
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

DescPtr<ParticleSetDesc> parseParticleSet(ParseContext& ctx, ObjectKey key)
{
    IPhysicsSource& src = ctx.source();
    if (!src.hasSchema(key, src.internToken("PhysxParticleSetAPI")))
        return {};

    DescPtr<ParticleSetDesc> d = allocateDesc<ParticleSetDesc>(ctx.descriptorAllocator());
    d->primKey = key;

    // Point/velocity attribute names differ between PointBased and PointInstancer.
    const bool isInstancer = src.isA(key, src.internToken("PointInstancer"));
    const TokenId pointsTok = src.internToken(isInstancer ? "positions" : "points");
    readVec3Array(src, key, pointsTok, d->points);
    readVec3Array(src, key, src.internToken("velocities"), d->velocities);
    if (d->velocities.empty())
        d->velocities.assign(d->points.size(), carb::Float3{ 0.0f, 0.0f, 0.0f });
    if (src.hasAuthoredAttribute(key, src.internToken("physxParticle:simulationPoints")))
        readVec3Array(src, key, src.internToken("physxParticle:simulationPoints"), d->simulationPoints);

    d->numParticles = static_cast<int>(d->points.size());
    src.getAttribute(key, src.internToken("physxParticle:particleEnabled"), d->enabled);
    src.getAttribute(key, src.internToken("physxParticle:selfCollision"), d->selfCollision);
    src.getAttribute(key, src.internToken("physxParticle:fluid"), d->fluid);
    d->particleGroup = readInt(src, key, src.internToken("physxParticle:particleGroup"), 0);

    {
        std::vector<ObjectKey> sys;
        src.getRelationshipTargets(key, src.internToken("physxParticle:particleSystem"), sys);
        if (!sys.empty())
            d->particleSystemKey = sys.front();
    }

    // maxParticles defaults to the point count; an authored value overrides.
    d->maxParticles = static_cast<int>(d->points.size());
    if (src.hasAuthoredAttribute(key, src.internToken("physxParticle:maxParticles")))
        d->maxParticles = readInt(src, key, src.internToken("physxParticle:maxParticles"), d->maxParticles);

    // Diffuse particles (inline on the set desc).
    d->enableDiffuseParticles = false;
    d->maxDiffuseParticleMultiplier = 0.0f;
    if (src.hasSchema(key, src.internToken("PhysxDiffuseParticlesAPI")))
    {
        src.getAttribute(key, src.internToken("physxDiffuseParticles:diffuseParticlesEnabled"), d->enableDiffuseParticles);
        const float mdpm = readFloat(src, key, src.internToken("physxDiffuseParticles:maxDiffuseParticleMultiplier"), 0.0f);
        d->maxDiffuseParticleMultiplier = mdpm < 0.0f ? 1.5f : mdpm;
        d->diffuseParticlesThreshold          = readFloat(src, key, src.internToken("physxDiffuseParticles:threshold"), 0.0f);
        d->diffuseParticlesLifetime           = readFloat(src, key, src.internToken("physxDiffuseParticles:lifetime"), 0.0f);
        d->diffuseParticlesAirDrag            = readFloat(src, key, src.internToken("physxDiffuseParticles:airDrag"), 0.0f);
        d->diffuseParticlesBubbleDrag         = readFloat(src, key, src.internToken("physxDiffuseParticles:bubbleDrag"), 0.0f);
        d->diffuseParticlesBuoyancy           = readFloat(src, key, src.internToken("physxDiffuseParticles:buoyancy"), 0.0f);
        d->diffuseParticlesKineticEnergyWeight = readFloat(src, key, src.internToken("physxDiffuseParticles:kineticEnergyWeight"), 0.0f);
        d->diffuseParticlesPressureWeight     = readFloat(src, key, src.internToken("physxDiffuseParticles:pressureWeight"), 0.0f);
        d->diffuseParticlesDivergenceWeight   = readFloat(src, key, src.internToken("physxDiffuseParticles:divergenceWeight"), 0.0f);
        d->diffuseParticlesCollisionDecay     = readFloat(src, key, src.internToken("physxDiffuseParticles:collisionDecay"), 0.0f);
    }

    // Mass / density overrides (UsdPhysicsMassAPI), authored-only. The -1.0
    // sentinels from the ParticleSetDesc ctor mean "fall back to MassAPI
    // density / material / default density"; an authored value overrides.
    if (src.hasAuthoredAttribute(key, src.internToken("physics:mass")))
        d->mass = readFloat(src, key, src.internToken("physics:mass"), d->mass);
    if (src.hasAuthoredAttribute(key, src.internToken("physics:density")))
        d->density = readFloat(src, key, src.internToken("physics:density"), d->density);

    // Fields copied from the owning particle system (scene + rest offsets),
    // mirroring the engine reader. Resolved through the memoizing cache so the
    // system is parsed at most once per pass.
    if (d->particleSystemKey.valid())
    {
        const ParticleSystemResolved& sys = resolveParticleSystemFields(ctx, d->particleSystemKey);
        d->sceneKey = sys.sceneKey;
        d->solidRestOffset = sys.solidRestOffset;
        d->fluidRestOffset = sys.fluidRestOffset;
    }

    return d;
}

DescPtr<ParticleIsosurfaceDesc> parseParticleIsosurface(ParseContext& ctx, ObjectKey key, float fluidRestOffset)
{
    IPhysicsSource& src = ctx.source();
    if (!src.hasSchema(key, src.internToken("PhysxParticleIsosurfaceAPI")))
        return {};
    DescPtr<ParticleIsosurfaceDesc> d = allocateDesc<ParticleIsosurfaceDesc>(ctx.descriptorAllocator());
    d->systemKey = key;
    src.getAttribute(key, src.internToken("physxParticleIsosurface:isosurfaceEnabled"), d->enableIsosurface);
    d->maxIsosurfaceVertices    = readInt(src, key, src.internToken("physxParticleIsosurface:maxVertices"), 0);
    d->maxIsosurfaceTriangles   = readInt(src, key, src.internToken("physxParticleIsosurface:maxTriangles"), 0);
    d->maxNumIsosurfaceSubgrids = readInt(src, key, src.internToken("physxParticleIsosurface:maxSubgrids"), 0);

    d->gridSpacing = readFloat(src, key, src.internToken("physxParticleIsosurface:gridSpacing"), 0.0f);
    if (d->gridSpacing < 0.0f)
        d->gridSpacing = fluidRestOffset * 1.5f; // autocompute
    else if (d->gridSpacing <= fluidRestOffset * 0.9f)
        d->gridSpacing = fluidRestOffset * 0.9f;

    d->surfaceDistance = readFloat(src, key, src.internToken("physxParticleIsosurface:surfaceDistance"), 0.0f);
    if (d->surfaceDistance < 0.0f)
        d->surfaceDistance = fluidRestOffset * 1.6f; // autocompute
    else if (d->surfaceDistance > 2.5f * d->gridSpacing)
        d->surfaceDistance = d->gridSpacing * 2.5f;

    std::string passes;
    src.getAttribute(key, src.internToken("physxParticleIsosurface:gridFilteringPasses"), passes);
    parseGridFilteringPasses(passes, d->gridFilteringPasses);

    d->gridSmoothingRadius = readFloat(src, key, src.internToken("physxParticleIsosurface:gridSmoothingRadius"), 0.0f);
    if (d->gridSmoothingRadius < 0.0f)
        d->gridSmoothingRadius = fluidRestOffset * 2.0f; // autocompute

    d->numMeshSmoothingPasses =
        readInt(src, key, src.internToken("physxParticleIsosurface:numMeshSmoothingPasses"), 0);
    d->numMeshNormalSmoothingPasses =
        readInt(src, key, src.internToken("physxParticleIsosurface:numMeshNormalSmoothingPasses"), 0);
    return d;
}

DescPtr<ParticleSamplingDesc> parseParticleSampling(ParseContext& ctx, ObjectKey key)
{
    IPhysicsSource& src = ctx.source();
    if (!src.isA(key, src.internToken("Mesh")) || !src.hasSchema(key, src.internToken("PhysxParticleSamplingAPI")))
        return {};

    DescPtr<ParticleSamplingDesc> d = allocateDesc<ParticleSamplingDesc>(ctx.descriptorAllocator());
    d->samplingDistance = readFloat(src, key, src.internToken("physxParticleSampling:samplingDistance"), -1.0f);
    d->maxSamples = readInt(src, key, src.internToken("physxParticleSampling:maxSamples"), 0);
    d->sampleVolume = true;
    src.getAttribute(key, src.internToken("physxParticleSampling:volume"), d->sampleVolume);
    {
        std::vector<ObjectKey> sets;
        src.getRelationshipTargets(key, src.internToken("physxParticleSampling:particles"), sets);
        if (!sets.empty())
            d->particleSetKey = sets.front();
    }

    // pointWidth from the referenced set's system rest offsets (the set already
    // copies them); fall back to the system's particleContactOffset when zero.
    float pointWidth = 0.5f;
    if (d->particleSetKey.valid())
    {
        if (DescPtr<ParticleSetDesc> setDesc = parseParticleSet(ctx, d->particleSetKey))
        {
            pointWidth = setDesc->fluid ? 2.0f * setDesc->fluidRestOffset : 2.0f * setDesc->solidRestOffset;
            if (pointWidth == 0.0f && setDesc->particleSystemKey.valid())
                pointWidth = resolveParticleSystemFields(ctx, setDesc->particleSystemKey).particleContactOffset;
        }
    }
    d->pointWidth = pointWidth;

    // Autocompletion + minimum limit for the sampling distance.
    if (d->samplingDistance <= 0.0f)
        d->samplingDistance = pointWidth;
    if (d->samplingDistance <= 0.75f * pointWidth)
        d->samplingDistance = 0.75f * pointWidth;

    return d;
}

} // namespace omni::physics::parse
