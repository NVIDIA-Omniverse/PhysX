// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/Numeric.h>
#include <carb/logging/Log.h>
#include <common/foundation/Allocator.h>
#include <common/utilities/UsdMaterialParsing.h>

#include "LoadTools.h"
#include "LoadUsd.h"
#include "CollisionGroup.h"
#include "IceDescriptorAllocator.h"
#include "Material.h"
#include "Particles.h"
#include "Mass.h"
#include <propertiesUpdate/PhysXPropertiesUpdate.h>
#include <PhysXDefines.h>
#include "AttributeHelpers.h"
#include "PhysXTools.h"

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>
#include "UsdSource.h"

using namespace PXR_NS;
using namespace carb;

static const TfToken physxParticleInflatableVolumeToken{ "physxParticle:inflatableVolume" };
static const TfToken physxParticleWeldedTriangleIndicesToken{ "physxParticle:weldedTriangleIndices" };
static const TfToken physxParticleWeldedVerticesRemapToWeldToken{ "physxParticle:weldedVerticesRemapToWeld" };
static const TfToken physxParticleWeldedVerticesRemapToOrigToken{ "physxParticle:weldedVerticesRemapToOrig" };
static const TfToken physxParticleFluidBoundaryDensityScaleToken{ "physxParticle:fluidBoundaryDensityScale" };
static const TfToken lockedAxisToken{ "lockedAxis" };

namespace
{
    template <typename T>
    bool SafeGetAttributeP(T* out, UsdAttribute const& attribute)
    {
        if (attribute.HasValue())
        {
            attribute.Get(out);

            return true;
        }

        return false;
    }

    template <typename T>
    bool SafeGetAttributeP(T* out, UsdAttribute const& attribute, T defaultValue)
    {
        if (attribute.HasValue())
        {
            attribute.Get(out);
        }
        else
        {
            *out = defaultValue;
        }

        return true;
    }

    template <>
    bool SafeGetAttributeP<carb::Float3>(carb::Float3* out, UsdAttribute const& attribute)
    {
        if (attribute.HasValue())
        {
            GfVec3f v;
            attribute.Get(&v);
            out->x = v[0];
            out->y = v[1];
            out->z = v[2];

            return true;
        }

        return false;
    }

    void convert(std::vector<carb::Uint4>& out, VtArray<GfVec4i> const& in)
    {
        out.resize(in.size());
        for (size_t i = 0; i < out.size(); i++)
        {
            out[i].x = in[i][0];
            out[i].y = in[i][1];
            out[i].z = in[i][2];
            out[i].w = in[i][3];
        }
    }

    void convert(std::vector<carb::Float3>& out, VtArray<GfVec3f> const& in)
    {
        out.resize(in.size());
        for (size_t i = 0; i < out.size(); i++)
        {
            out[i].x = in[i][0];
            out[i].y = in[i][1];
            out[i].z = in[i][2];
        }
    }

    void convert(std::vector<carb::Int2>& out, VtArray<GfVec2i> const& in)
    {
        out.resize(in.size());
        for (size_t i = 0; i < out.size(); i++)
        {
            out[i].x = in[i][0];
            out[i].y = in[i][1];
        }
    }

    void convert(std::vector<float>& out, VtArray<float> const& in)
    {
        out.resize(in.size());
        for (size_t i = 0; i < out.size(); i++)
        {
            out[i] = in[i];
        }
    }

    void convert(std::vector<uint32_t>& out, VtArray<int> const& in)
    {
        out.resize(in.size());
        for (size_t i = 0; i < out.size(); i++)
        {
            out[i] = in[i];
        }
    }

    void convert(std::vector<uint32_t>& out, VtArray<uint32_t> const& in)
    {
        out.resize(in.size());
        for (size_t i = 0; i < out.size(); i++)
        {
            out[i] = in[i];
        }
    }

    template <typename SrcT, typename DstT>
    bool convertIndexBuffer(DstT* out, const SrcT* in, size_t size, uint32_t multiple, SrcT range, SrcT* ignore = nullptr)
    {
        if (size % multiple != 0)
        {
            return false;
        }

        if (ignore)
        {
            for (size_t i = 0; i < size; ++i)
            {
                if (in[i] >= range && in[i] != *ignore)
                {
                    return false;
                }
                out[i] = in[i];
            }
        }
        else
        {
            for (size_t i = 0; i < size; ++i)
            {
                if (in[i] >= range)
                {
                    return false;
                }
                out[i] = in[i];
            }
        }
        return true;
    }

    template <typename SrcT, typename DstT>
    bool convertIndexBuffer(std::vector<DstT>& out, const VtArray<SrcT>& in, uint32_t multiple, SrcT range, SrcT* ignore = nullptr)
    {
        out.resize(in.size());
        return convertIndexBuffer(out.data(), in.data(), out.size(), multiple, range, ignore);
    }

}

namespace omni
{
namespace physx
{
namespace usdparser
{

ParticleSystemDesc* buildParticleSystemDesc(AttachedStage& attachedStage,
    const omni::physics::usd::ScannedStage& scanned, const omni::physics::parse::ParticleSystemDesc& s)
{
    ParticleSystemDesc* d = ICE_PLACEMENT_NEW(ParticleSystemDesc)();

    // systemPath comes from the parse-time intern table (scanned.pathFor),
    // never the persistent attachedStage table (ADR-0004 key-space invariant).
    d->systemPath = scanned.pathFor(s.systemKey);

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
    // ObjectKeys; map them through the parse-time table to prim paths.
    d->scenePath = s.sceneKey.valid() ? scanned.pathFor(s.sceneKey).GetPrimPath() : SdfPath();
    d->filteredCollisions.clear();
    for (const omni::physics::parse::ObjectKey& k : s.filteredCollisions)
        d->filteredCollisions.push_back(scanned.pathFor(k));

    // Runtime ObjectId resolution + change-tracking registration: these need
    // the engine ObjectDb / time-sampled callback, so they stay consumer-side.
    // Reads go through the source (no direct USD).
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const omni::physics::parse::ObjectKey pkey = attachedStage.keyFor(d->systemPath);

    d->material = kInvalidObjectId;
    if (src)
    {
        const omni::physics::parse::ObjectKey matKey = src->getMaterialBinding(pkey);
        if (matKey.valid())
            d->material = getMaterial(attachedStage, attachedStage.pathFor(matKey), ePBDMaterial);
    }

    d->collisionGroup = getCollisionGroup(attachedStage, d->systemPath);

    // Re-register the time-sampled wind callback (engine reader used
    // GetNumTimeSamples() > 1, i.e. isAttributeTimeSampled).
    if (src && src->isAttributeTimeSampled(pkey, src->internToken("wind")))
    {
        static const TfToken windToken("wind");
        attachedStage.registerTimeSampledAttribute(d->systemPath.AppendProperty(windToken), updateParticleSystemAttribute);
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
    const omni::physics::usd::ScannedStage& scanned, const omni::physics::parse::ParticleSetDesc& s)
{
    ParticleSetDesc* d = ICE_PLACEMENT_NEW(ParticleSetDesc)();

    d->primPath = scanned.pathFor(s.primKey);
    d->particleSystemPath = s.particleSystemKey.valid() ? scanned.pathFor(s.particleSystemKey) : SdfPath();
    d->scenePath = s.sceneKey.valid() ? scanned.pathFor(s.sceneKey).GetPrimPath() : SdfPath();

    copyParticleSetData(*d, s);

    if (!particleSetDataIsValid(*d))
    {
        CARB_LOG_WARN("PhysxSchemaPhysxParticleSetAPI parsing failed: %s", d->primPath.GetText());
        ICE_FREE(d);
        return nullptr;
    }

    return d;
}

ParticleSetDesc* buildParticleSetDescRuntime(AttachedStage& attachedStage,
    const omni::physics::parse::ParticleSetDesc& s)
{
    // Runtime re-read: keys come from the persistent source table, so paths
    // resolve through attachedStage (not a parse-time ScannedStage).
    ParticleSetDesc* d = ICE_PLACEMENT_NEW(ParticleSetDesc)();
    d->primPath = s.primKey.valid() ? attachedStage.pathFor(s.primKey) : SdfPath();
    d->particleSystemPath = s.particleSystemKey.valid() ? attachedStage.pathFor(s.particleSystemKey) : SdfPath();
    d->scenePath = s.sceneKey.valid() ? attachedStage.pathFor(s.sceneKey).GetPrimPath() : SdfPath();
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

PBDMaterialDesc* ParsePBDParticleMaterial(AttachedStage& attachedStage,
                                          const omni::physics::parse::ObjectKey& materialKey)
{
    if (!materialKey.valid())
        return nullptr;

    omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return nullptr;

    omni::physics::parse::ParseContext ctx(*source, omni::physx::usdparser::iceDescriptorAllocator());
    omni::physics::parse::DescPtr<PBDMaterialDesc> parsed =
        omni::physics::parse::parsePBDMaterial(ctx, materialKey);
    if (!parsed)
    {
        CARB_LOG_WARN("ParsePBDParticleMaterial: prim doesn't have a PhysxSchemaPhysxPBDMaterialAPI\n");
        return nullptr;
    }

    PBDMaterialDesc* out = ICE_PLACEMENT_NEW(PBDMaterialDesc)();
    *out = *parsed;
    return out;
}

PBDMaterialDesc* ParsePBDParticleMaterial(const UsdStageWeakPtr stage, const SdfPath& materialPath)
{
    if (materialPath == SdfPath())
        return nullptr;

    const long stageId = stage ? PXR_NS::UsdUtilsStageCache::Get().GetId(stage).ToLongInt() : 0;
    AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
    if (attachedStage)
        return ParsePBDParticleMaterial(*attachedStage, attachedStage->keyFor(materialPath));

    omni::physics::usd::UsdSource source(stage);
    omni::physics::parse::ParseContext ctx(source, omni::physx::usdparser::iceDescriptorAllocator());
    omni::physics::parse::DescPtr<PBDMaterialDesc> parsed =
        omni::physics::parse::parsePBDMaterial(ctx, source.keyFor(materialPath));
    if (!parsed)
    {
        CARB_LOG_WARN("ParsePBDParticleMaterial: prim doesn't have a PhysxSchemaPhysxPBDMaterialAPI\n");
        return nullptr;
    }

    PBDMaterialDesc* out = ICE_PLACEMENT_NEW(PBDMaterialDesc)();
    *out = *parsed;
    out->materialKey = omni::physics::parse::ObjectKey{};
    return out;
}

SdfPath GetParticleSystemPath(const PhysxSchemaPhysxParticleAPI& particleAPI)
{
    SdfPath particleSystemPath;
    UsdRelationship particleRel = particleAPI.GetParticleSystemRel();
    if (particleRel)
    {
        SdfPathVector paths;
        if (particleRel.GetTargets(&paths) && paths.size() > 0)
        {
            particleSystemPath = paths[0];
        }
    }
    return particleSystemPath;
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
