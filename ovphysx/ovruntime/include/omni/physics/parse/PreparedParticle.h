// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-1 AC-2 AC-3
 */
#pragma once
#include "ParseApi.h"
#include "PreparedAttribute.h"
namespace omni::physics::parse
{
struct PreparedParticleSystem
{
    bool prepared = false;
    ObjectKey key;
    bool hasPhysxParticleAnisotropyAPI = false;
    bool hasPhysxParticleSmoothingAPI = false;
    bool hasPhysxParticleIsosurfaceAPI = false;
    float metersPerUnit = 1.0f;
    ObjectKey sceneKey;
    std::vector<ObjectKey> filteredCollisions;
    PreparedAttribute<float> particleContactOffset;
    PreparedAttribute<float> contactOffset;
    PreparedAttribute<float> restOffset;
    PreparedAttribute<float> solidRestOffset;
    PreparedAttribute<float> fluidRestOffset;
    PreparedAttribute<float> maxDepenetrationVelocity;
    PreparedAttribute<float> maxVelocity;
    PreparedAttribute<float> physxParticle_fluidBoundaryDensityScale;
    PreparedAttribute<float> neighborhoodScale;
    PreparedAttribute<decltype(ParticleSystemDesc{}.enableParticleSystem)> particleSystemEnabled;
    PreparedAttribute<decltype(ParticleSystemDesc{}.enableCCD)> enableCCD;
    PreparedAttribute<int64_t> solverPositionIterationCount;
    PreparedAttribute<decltype(ParticleSystemDesc{}.wind)> wind;
    PreparedAttribute<int64_t> maxNeighborhood;
    PreparedAttribute<int64_t> lockedAxis;
    PreparedAttribute<decltype(ParticleSystemDesc{}.enableAnisotropy)> physxParticleAnisotropy_particleAnisotropyEnabled;
    PreparedAttribute<decltype(ParticleSystemDesc{}.enableSmoothing)> physxParticleSmoothing_particleSmoothingEnabled;
    PreparedAttribute<decltype(ParticleSystemDesc{}.enableIsosurface)> physxParticleIsosurface_isosurfaceEnabled;
};
PreparedParticleSystem prepareParticleSystem(ParseContext& context, ObjectKey key);
void prepareParticleSystemStorage(const PreparedParticleSystem& input, ParticleSystemDesc& output);
bool fillParticleSystem(const PreparedParticleSystem& input, ParticleSystemDesc& output) noexcept;

struct PreparedParticleAnisotropy
{
    bool prepared = false;
    ObjectKey key;
    bool hasPhysxParticleAnisotropyAPI = false;
    PreparedAttribute<float> physxParticleAnisotropy_scale;
    PreparedAttribute<float> physxParticleAnisotropy_min;
    PreparedAttribute<float> physxParticleAnisotropy_max;
    PreparedAttribute<decltype(ParticleAnisotropyDesc{}.enableAnisotropy)> physxParticleAnisotropy_particleAnisotropyEnabled;
};
PreparedParticleAnisotropy prepareParticleAnisotropy(ParseContext& context, ObjectKey key);
void prepareParticleAnisotropyStorage(const PreparedParticleAnisotropy& input, ParticleAnisotropyDesc& output);
bool fillParticleAnisotropy(const PreparedParticleAnisotropy& input, ParticleAnisotropyDesc& output) noexcept;

struct PreparedParticleSmoothing
{
    bool prepared = false;
    ObjectKey key;
    bool hasPhysxParticleSmoothingAPI = false;
    PreparedAttribute<float> physxParticleSmoothing_strength;
    PreparedAttribute<decltype(ParticleSmoothingDesc{}.enableSmoothing)> physxParticleSmoothing_particleSmoothingEnabled;
};
PreparedParticleSmoothing prepareParticleSmoothing(ParseContext& context, ObjectKey key);
void prepareParticleSmoothingStorage(const PreparedParticleSmoothing& input, ParticleSmoothingDesc& output);
bool fillParticleSmoothing(const PreparedParticleSmoothing& input, ParticleSmoothingDesc& output) noexcept;

struct PreparedParticleIsosurface
{
    bool prepared = false;
    ObjectKey key;
    bool hasPhysxParticleIsosurfaceAPI = false;
    float fluidRestOffset = 0.0f;
    std::string passes;
    size_t passCount = 0;
    PreparedAttribute<int> physxParticleIsosurface_maxVertices;
    PreparedAttribute<int> physxParticleIsosurface_maxTriangles;
    PreparedAttribute<int> physxParticleIsosurface_maxSubgrids;
    PreparedAttribute<float> physxParticleIsosurface_gridSpacing;
    PreparedAttribute<float> physxParticleIsosurface_surfaceDistance;
    PreparedAttribute<float> physxParticleIsosurface_gridSmoothingRadius;
    PreparedAttribute<int> physxParticleIsosurface_numMeshSmoothingPasses;
    PreparedAttribute<int> physxParticleIsosurface_numMeshNormalSmoothingPasses;
    PreparedAttribute<decltype(ParticleIsosurfaceDesc{}.enableIsosurface)> physxParticleIsosurface_isosurfaceEnabled;
};
PreparedParticleIsosurface prepareParticleIsosurface(ParseContext& context, ObjectKey key, float fluidRestOffset);
void prepareParticleIsosurfaceStorage(const PreparedParticleIsosurface& input, ParticleIsosurfaceDesc& output);
bool fillParticleIsosurface(const PreparedParticleIsosurface& input, ParticleIsosurfaceDesc& output) noexcept;

struct PreparedParticleSet
{
    bool prepared = false, systemResolved = false;
    ObjectKey key, systemKey;
    ParticleSystemResolved system;
    std::vector<carb::Float3> points, velocities, simulationPoints;
    bool hasPhysxParticleSetAPI = false;
    bool hasPhysxDiffuseParticlesAPI = false;
    PreparedAttribute<int> physxParticle_particleGroup;
    PreparedAttribute<int> physxParticle_maxParticles;
    PreparedAttribute<float> physxDiffuseParticles_maxDiffuseParticleMultiplier;
    PreparedAttribute<float> physxDiffuseParticles_threshold;
    PreparedAttribute<float> physxDiffuseParticles_lifetime;
    PreparedAttribute<float> physxDiffuseParticles_airDrag;
    PreparedAttribute<float> physxDiffuseParticles_bubbleDrag;
    PreparedAttribute<float> physxDiffuseParticles_buoyancy;
    PreparedAttribute<float> physxDiffuseParticles_kineticEnergyWeight;
    PreparedAttribute<float> physxDiffuseParticles_pressureWeight;
    PreparedAttribute<float> physxDiffuseParticles_divergenceWeight;
    PreparedAttribute<float> physxDiffuseParticles_collisionDecay;
    PreparedAttribute<float> physics_mass;
    PreparedAttribute<float> physics_density;
    PreparedAttribute<decltype(ParticleSetDesc{}.enabled)> physxParticle_particleEnabled;
    PreparedAttribute<decltype(ParticleSetDesc{}.selfCollision)> physxParticle_selfCollision;
    PreparedAttribute<decltype(ParticleSetDesc{}.fluid)> physxParticle_fluid;
    PreparedAttribute<decltype(ParticleSetDesc{}.enableDiffuseParticles)> physxDiffuseParticles_diffuseParticlesEnabled;
};
PreparedParticleSet prepareParticleSet(ParseContext& context, ObjectKey key, bool resolveSystem = true);
void resolvePreparedParticleSetSystem(ParseContext& context, PreparedParticleSet& input);
void prepareParticleSetStorage(const PreparedParticleSet& input, ParticleSetDesc& output);
bool fillParticleSet(const PreparedParticleSet& input, ParticleSetDesc& output) noexcept;

struct PreparedParticleSampling
{
    bool prepared = false, validSet = false, hasSystem = false, fluid = false;
    ObjectKey key, setKey;
    ParticleSystemResolved system;
    PreparedAttribute<float> samplingDistance;
    PreparedAttribute<int> maxSamples;
    PreparedAttribute<bool> volume;
};
PreparedParticleSampling prepareParticleSampling(ParseContext& context, ObjectKey key);
bool fillParticleSampling(const PreparedParticleSampling& input, ParticleSamplingDesc& output) noexcept;

} // namespace omni::physics::parse
