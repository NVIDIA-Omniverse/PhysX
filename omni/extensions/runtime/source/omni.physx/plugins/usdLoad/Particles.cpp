// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <common/foundation/Allocator.h>
#include <common/utilities/UsdMaterialParsing.h>

#include "LoadTools.h"
#include "LoadUsd.h"
#include "CollisionGroup.h"
#include "Material.h"
#include "Particles.h"
#include "Mass.h"
#include <propertiesUpdate/PhysXPropertiesUpdate.h>
#include <PhysXDefines.h>
#include "AttributeHelpers.h"

using namespace pxr;
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
ParticleSystemDesc* ParseParticleSystem(AttachedStage& attachedStage, const UsdPrim& usdPrim)
{
    if (!usdPrim)
        return nullptr;

    ParticleSystemDesc* particleSystem = ICE_PLACEMENT_NEW(ParticleSystemDesc)();
    particleSystem->systemPath = usdPrim.GetPath();

    UsdStageWeakPtr stage = usdPrim.GetStage();

    PhysxSchemaPhysxParticleSystem particleSystemPrim(usdPrim);

    SafeGetAttributeP(&particleSystem->particleContactOffset, particleSystemPrim.GetParticleContactOffsetAttr(), 0.0f);
    float mpu = float(UsdGeomGetStageMetersPerUnit(usdPrim.GetStage()));
    if (!particleSystemPrim.GetParticleContactOffsetAttr().HasAuthoredValue())
    {
        // this means we are using the schema default
        particleSystem->particleContactOffset = particleSystem->particleContactOffset / mpu;
    }

    SafeGetAttributeP(&particleSystem->enableParticleSystem, particleSystemPrim.GetParticleSystemEnabledAttr(), false);
    SafeGetAttributeP(&particleSystem->enableCCD, particleSystemPrim.GetEnableCCDAttr(), false);
    SafeGetAttributeP(&particleSystem->contactOffset, particleSystemPrim.GetContactOffsetAttr(), 0.0f);
    SafeGetAttributeP(&particleSystem->restOffset, particleSystemPrim.GetRestOffsetAttr(), 0.0f);
    SafeGetAttributeP(&particleSystem->solidRestOffset, particleSystemPrim.GetSolidRestOffsetAttr(), 0.0f);
    SafeGetAttributeP(&particleSystem->fluidRestOffset, particleSystemPrim.GetFluidRestOffsetAttr(), 0.0f);

    // apply autocomputation - order of these somewhat matters to avoid duplicate autocomputation
    // because everything is computed relative to particleContactOffset
    particleSystem->particleContactOffset = usdparser::completeParticleContactOffset(stage, particleSystem->particleContactOffset);

    particleSystem->fluidRestOffset = usdparser::completeFluidRestOffset(stage, particleSystem->fluidRestOffset, particleSystem->particleContactOffset);
    particleSystem->restOffset = usdparser::completeRestOffset(stage, particleSystem->restOffset, particleSystem->particleContactOffset);

    particleSystem->contactOffset = usdparser::completeContactOffset(stage, particleSystem->contactOffset, particleSystem->particleContactOffset);
    // enforce schema lower limit
    if (particleSystem->contactOffset <= particleSystem->restOffset)
    {
        REPORT_PHYSICS_MESSAGE(usdparser::ErrorCode::eInfo,
            "Particle System parsing: Adjusting Contact Offset to a minimum of 1.01 times the Rest Offset (%f).",
            particleSystem->restOffset);
        particleSystem->contactOffset = particleSystem->restOffset * 1.01f;
    }

    particleSystem->solidRestOffset = usdparser::completeSolidRestOffset(stage, particleSystem->solidRestOffset, particleSystem->particleContactOffset);
    // enforce schema lower limit
    if (particleSystem->particleContactOffset <= GfMax(particleSystem->solidRestOffset, particleSystem->fluidRestOffset))
    {
        REPORT_PHYSICS_MESSAGE(usdparser::ErrorCode::eInfo,
            "Particle System parsing: Adjusting Particle Contact Offset to a minimum of 1.01 times the max(Fluid Rest Offset (%f), Solid Rest Offset (%f)).",
            particleSystem->fluidRestOffset, particleSystem->solidRestOffset);
        particleSystem->particleContactOffset = GfMax(particleSystem->solidRestOffset, particleSystem->fluidRestOffset) * 1.01f;
    }

    // TODO: get defaults from PhysX, or at least make dependent on unit scale.
    SafeGetAttributeP(&particleSystem->maxDepenetrationVelocity, particleSystemPrim.GetMaxDepenetrationVelocityAttr(), ((particleSystem->restOffset == 0.0f) ? (0.2f * 30.0f) : particleSystem->restOffset * 30.0f));
    SafeGetAttributeP(&particleSystem->maxVelocity, particleSystemPrim.GetMaxVelocityAttr(), CARB_FLOAT_MAX);

    SafeGetAttributeP(&particleSystem->fluidBoundaryDensityScale, particleSystemPrim.GetPrim().GetAttribute(physxParticleFluidBoundaryDensityScaleToken), 0.0f);

    particleSystemPrim.GetSolverPositionIterationCountAttr().Get(&particleSystem->solverPositionIterations);

    particleSystem->wind = carb::Float3({ 0.0f, 0.0f, 0.0f });
    SafeGetAttributeP(&particleSystem->wind, particleSystemPrim.GetWindAttr());
    // check for time samples
    if (particleSystemPrim.GetWindAttr().GetNumTimeSamples() > 1)
    {
        UsdLoad::getUsdLoad()->registerTimeSampledAttribute(particleSystemPrim.GetWindAttr(), updateParticleSystemAttribute);
    }

    particleSystemPrim.GetMaxNeighborhoodAttr().Get(&particleSystem->maxNeighborhood);
    particleSystemPrim.GetNeighborhoodScaleAttr().Get(&particleSystem->neighborhoodScale);
    UsdAttribute lockedAxisAttr = particleSystemPrim.GetPrim().GetAttribute(lockedAxisToken);
    if (!getAttribute(particleSystem->lockedAxis, lockedAxisAttr, 0, 7, nullptr))
    {
        particleSystem->lockedAxis = 0;
    }

    /* add default values here */
    particleSystem->enableAnisotropy = false;
    particleSystem->enableSmoothing = false;
    particleSystem->enableIsosurface = false;

    if (usdPrim.HasAPI<PhysxSchemaPhysxParticleAnisotropyAPI>())
    {
        PhysxSchemaPhysxParticleAnisotropyAPI anisotropyAPI(usdPrim);
        anisotropyAPI.GetParticleAnisotropyEnabledAttr().Get(&particleSystem->enableAnisotropy);
    }

    if (usdPrim.HasAPI<PhysxSchemaPhysxParticleSmoothingAPI>())
    {
        PhysxSchemaPhysxParticleSmoothingAPI smoothingAPI(usdPrim);
        smoothingAPI.GetParticleSmoothingEnabledAttr().Get(&particleSystem->enableSmoothing);
    }

    if (usdPrim.HasAPI<PhysxSchemaPhysxParticleIsosurfaceAPI>())
    {
        PhysxSchemaPhysxParticleIsosurfaceAPI isosurfaceAPI(usdPrim);
        isosurfaceAPI.GetIsosurfaceEnabledAttr().Get(&particleSystem->enableIsosurface);
    }

    particleSystem->scenePath = SdfPath();
    if (UsdRelationship rel = particleSystemPrim.GetSimulationOwnerRel())
    {
        SdfPathVector paths;
        if (rel.GetTargets(&paths) && paths.size() > 0)
        {
            particleSystem->scenePath = paths[0].GetPrimPath();
        }
    }

    particleSystem->material = kInvalidObjectId;
    const SdfPath& materialPath = usdmaterialutils::getMaterialBinding(usdPrim);
    if (materialPath != SdfPath())
    {
        particleSystem->material = getMaterial(attachedStage, materialPath, ePBDMaterial);
    }

    particleSystem->collisionGroup = getCollisionGroup(attachedStage, usdPrim.GetPrimPath());

    // filteredPairs
    UsdPhysicsFilteredPairsAPI filteredPairsAPI(usdPrim);
    if (filteredPairsAPI && filteredPairsAPI.GetFilteredPairsRel())
    {
        filteredPairsAPI.GetFilteredPairsRel().GetTargets(&particleSystem->filteredCollisions);
    }

    return particleSystem;
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

PBDMaterialDesc* ParsePBDParticleMaterial(const UsdStageWeakPtr stage, const UsdPrim& usdPrim)
{
    if (!usdPrim.HasAPI<PhysxSchemaPhysxPBDMaterialAPI>())
    {
        CARB_LOG_WARN("ParsePBDParticleMaterial: UsdPrim doesn't have a PhysxSchemaPhysxPBDMaterialAPI\n");
        return nullptr;
    }

    // create and fill in descriptor
    PBDMaterialDesc* out = ICE_PLACEMENT_NEW(PBDMaterialDesc)();

    PhysxSchemaPhysxPBDMaterialAPI materialAPI(usdPrim);

    materialAPI.GetFrictionAttr().Get(&out->friction);
    materialAPI.GetParticleFrictionScaleAttr().Get(&out->particleFrictionScale);
    materialAPI.GetDampingAttr().Get(&out->damping);
    materialAPI.GetViscosityAttr().Get(&out->viscosity);
    materialAPI.GetVorticityConfinementAttr().Get(&out->vorticityConfinement);
    materialAPI.GetSurfaceTensionAttr().Get(&out->surfaceTension);
    materialAPI.GetCohesionAttr().Get(&out->cohesion);
    materialAPI.GetAdhesionAttr().Get(&out->adhesion);
    materialAPI.GetParticleAdhesionScaleAttr().Get(&out->particleAdhesionScale);
    materialAPI.GetAdhesionOffsetScaleAttr().Get(&out->adhesionOffsetScale);
    materialAPI.GetLiftAttr().Get(&out->lift);
    materialAPI.GetDragAttr().Get(&out->drag);
    materialAPI.GetGravityScaleAttr().Get(&out->gravityScale);
    materialAPI.GetCflCoefficientAttr().Get(&out->cflCoefficient);
    materialAPI.GetDensityAttr().Get(&out->density);

    out->materialPath = usdPrim.GetPath();

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

ParticleSetDesc* ParseParticleSet(AttachedStage& attachedStage, const UsdPrim& usdPrim)
{
    if (!(usdPrim.IsA<UsdGeomPointBased>() || usdPrim.IsA<UsdGeomPointInstancer>()))
    {
        CARB_LOG_WARN("PhysxSchemaPhysxParticleSetAPI parsing: UsdPrim is neither of type UsdGeomPointBased nor of type UsdGeomPointInstancer.");
        return nullptr;
    }
    UsdGeomPointBased pointBased = UsdGeomPointBased(usdPrim);
    UsdGeomPointInstancer pointInstancer = UsdGeomPointInstancer(usdPrim);

    if (!usdPrim.HasAPI<PhysxSchemaPhysxParticleSetAPI>())
    {
        CARB_LOG_WARN("PhysxSchemaPhysxParticleSetAPI parsing: UsdPrim doesn't have a PhysxSchemaPhysxParticleSetAPI.");
        return nullptr;
    }
    PhysxSchemaPhysxParticleSetAPI particleSetAPI(usdPrim);
    PhysxSchemaPhysxParticleAPI particleAPI(particleSetAPI);

    SdfPath particleSystemPath = GetParticleSystemPath(particleAPI);
    if (particleSystemPath.IsEmpty())
    {
        CARB_LOG_WARN("PhysxSchemaPhysxParticleSetAPI parsing: No valid ParticleSystem Relationship found.");
        return nullptr;
    }

    PhysxSchemaPhysxParticleSystem particleSystem = PhysxSchemaPhysxParticleSystem::Get(usdPrim.GetStage(), particleSystemPath);
    if (!particleSystem)
    {
        CARB_LOG_WARN("PhysxSchemaPhysxParticleSetAPI parsing: ParticleSystem Relationship doesn't point to valid ParticleSystem.");
        return nullptr;
    }
    UsdPrim particleSystemPrim = particleSystem.GetPrim();

    ParticleSetDesc* particleSet = ICE_PLACEMENT_NEW(ParticleSetDesc)();
    bool isValid = true;

    VtArray<GfVec3f> velocities;
    VtArray<GfVec3f> points;
    VtArray<GfVec3f> simulationPoints;
    const char* pointsAttributeName = nullptr;
    const char* velocitiesAttributeName = nullptr;
    if (pointBased)
    {
        SafeGetAttributeP(&points, pointBased.GetPointsAttr());
        SafeGetAttributeP(&velocities, pointBased.GetVelocitiesAttr());
        velocitiesAttributeName = "UsdGeomPointBased:velocities";
        pointsAttributeName = "UsdGeomPointBased:points";
    }
    else if (pointInstancer)
    {
        SafeGetAttributeP(&points, pointInstancer.GetPositionsAttr());
        SafeGetAttributeP(&velocities, pointInstancer.GetVelocitiesAttr());
        velocitiesAttributeName = "UsdGeomPointInstancer:velocities";
        pointsAttributeName = "UsdGeomPointInstancer:positions";
    }

    if (velocities.size() && velocities.size() != points.size())
    {
        CARB_LOG_WARN("%s and %s have inconsistent sizes.", velocitiesAttributeName, pointsAttributeName);
        isValid = false;
    }

    // initialize velocities, if not set
    if (velocities.empty())
    {
        velocities.assign(points.size(), GfVec3f(0.0f));
    }

    UsdAttribute simulationPointsAttr = particleSetAPI.GetSimulationPointsAttr();
    if (simulationPointsAttr.HasAuthoredValue())
    {
        SafeGetAttributeP(&simulationPoints, simulationPointsAttr);
    }

    if (simulationPoints.size() && simulationPoints.size() != points.size())
    {
        CARB_LOG_WARN("PhysxSchemaPhysxParticleSetAPI:simulationPoints and %s have inconsistent sizes.", pointsAttributeName);
        isValid = false;
    }

    particleSet->numParticles = static_cast<int>(points.size());
    convert(particleSet->points, points);
    convert(particleSet->velocities, velocities);
    convert(particleSet->simulationPoints, simulationPoints);

    SafeGetAttributeP(&particleSet->enabled, particleAPI.GetParticleEnabledAttr());
    SafeGetAttributeP(&particleSet->selfCollision, particleAPI.GetSelfCollisionAttr());
    SafeGetAttributeP(&particleSet->fluid, particleSetAPI.GetFluidAttr());
    SafeGetAttributeP(&particleSet->particleGroup, particleAPI.GetParticleGroupAttr());

    particleSet->particleSystemPath = particleSystemPath;
    particleSet->primPath = usdPrim.GetPath();

    particleSet->maxParticles = static_cast<int>(points.size());
    static const TfToken maxParticlesToken{ "physxParticle:maxParticles" };
    const UsdAttribute maxParticlesAttr = usdPrim.GetAttribute(maxParticlesToken);
    if (maxParticlesAttr.HasAuthoredValue())
    {
        maxParticlesAttr.Get(&particleSet->maxParticles);
    }

    if (particleSet->maxParticles < particleSet->numParticles)
    {
        CARB_LOG_WARN("PhysxSchemaPhysxParticleSetAPI:maxParticles is lower than %s size.", pointsAttributeName);
        isValid = false;
    }

#if ENABLE_FABRIC_FOR_PARTICLE_SETS
    // check whether to use fabric
    particleSet->useFabric = false;
    static const TfToken fabricToken{ "physxParticle:fabric" };
    const UsdAttribute fabricAttr = usdPrim.GetAttribute(fabricToken);
    if (fabricAttr.HasAuthoredValue())
    {
        fabricAttr.Get(&particleSet->useFabric);
    }
#endif

    /* add default values here */
    particleSet->enableDiffuseParticles = false;
    particleSet->maxDiffuseParticleMultiplier = 0.0f;

    if (usdPrim.HasAPI<PhysxSchemaPhysxDiffuseParticlesAPI>())
    {
        const float maxDiffuseParticleMultiplierDefault = 1.5f;
        PhysxSchemaPhysxDiffuseParticlesAPI diffuseParticlesAPI(usdPrim);

        diffuseParticlesAPI.GetDiffuseParticlesEnabledAttr().Get(&particleSet->enableDiffuseParticles);

        float mdpm;
        diffuseParticlesAPI.GetMaxDiffuseParticleMultiplierAttr().Get(&mdpm);
        particleSet->maxDiffuseParticleMultiplier = mdpm < 0.0f ? maxDiffuseParticleMultiplierDefault : mdpm;

        diffuseParticlesAPI.GetThresholdAttr().Get(&particleSet->diffuseParticlesThreshold);
        diffuseParticlesAPI.GetLifetimeAttr().Get(&particleSet->diffuseParticlesLifetime);

        diffuseParticlesAPI.GetAirDragAttr().Get(&particleSet->diffuseParticlesAirDrag);
        diffuseParticlesAPI.GetBubbleDragAttr().Get(&particleSet->diffuseParticlesBubbleDrag);
        diffuseParticlesAPI.GetBuoyancyAttr().Get(&particleSet->diffuseParticlesBuoyancy);

        diffuseParticlesAPI.GetKineticEnergyWeightAttr().Get(&particleSet->diffuseParticlesKineticEnergyWeight);
        diffuseParticlesAPI.GetPressureWeightAttr().Get(&particleSet->diffuseParticlesPressureWeight);
        diffuseParticlesAPI.GetDivergenceWeightAttr().Get(&particleSet->diffuseParticlesDivergenceWeight);
        diffuseParticlesAPI.GetCollisionDecayAttr().Get(&particleSet->diffuseParticlesCollisionDecay);
    }

    ParticleSystemDesc* desc = ParseParticleSystem(attachedStage, particleSystemPrim);
    particleSet->scenePath = desc->scenePath;
    particleSet->solidRestOffset = desc->solidRestOffset;
    particleSet->fluidRestOffset = desc->fluidRestOffset;
    ICE_FREE(desc);

    // only parse mass property if it was provided
    UsdPhysicsMassAPI massApi(usdPrim);
    const UsdAttribute massAttribute = massApi.GetMassAttr();
    if (massAttribute.HasAuthoredValue())
    {
        massAttribute.Get(&particleSet->mass);
    }
    const UsdAttribute densityAttribute = massApi.GetDensityAttr();
    if (densityAttribute.HasAuthoredValue())
    {
        densityAttribute.Get(&particleSet->density);
    }

    if (!isValid)
    {
        CARB_LOG_WARN("PhysxSchemaPhysxParticleSetAPI parsing failed: %s", usdPrim.GetPrimPath().GetText());
        ICE_FREE(particleSet);
        return nullptr;
    }

    return particleSet;
}

ParticleAnisotropyDesc* ParseParticleAnisotropy(const UsdPrim& usdPrim)
{
    if (!usdPrim.IsA<PhysxSchemaPhysxParticleSystem>() || !usdPrim.HasAPI<PhysxSchemaPhysxParticleAnisotropyAPI>())
    {
        CARB_LOG_WARN("ParseParticleAnisotropy: UsdPrim is not a PhysxSchemaPhysxParticleSystem or doesn't have a PhysxSchemaPhysxParticleAnisotropyAPI\n");
        return nullptr;
    }

    PhysxSchemaPhysxParticleAnisotropyAPI anisotropyAPI(usdPrim);

    ParticleAnisotropyDesc* anisotropyDesc = ICE_PLACEMENT_NEW(ParticleAnisotropyDesc)();
    anisotropyDesc->systemPath = usdPrim.GetPath();

    anisotropyAPI.GetParticleAnisotropyEnabledAttr().Get(&anisotropyDesc->enableAnisotropy);
    anisotropyAPI.GetScaleAttr().Get(&anisotropyDesc->scale);
    anisotropyAPI.GetMinAttr().Get(&anisotropyDesc->min);
    anisotropyAPI.GetMaxAttr().Get(&anisotropyDesc->max);

    return anisotropyDesc;
}

ParticleSmoothingDesc* ParseParticleSmoothing(const UsdPrim& usdPrim)
{
    if (!usdPrim.IsA<PhysxSchemaPhysxParticleSystem>() || !usdPrim.HasAPI<PhysxSchemaPhysxParticleSmoothingAPI>())
    {
        CARB_LOG_WARN("ParseParticleSmoothing: UsdPrim is not a PhysxSchemaPhysxParticleSystem or doesn't have a PhysxSchemaPhysxParticleSmoothingAPI\n");
        return nullptr;
    }

    PhysxSchemaPhysxParticleSmoothingAPI smoothingAPI(usdPrim);

    ParticleSmoothingDesc* smoothingDesc = ICE_PLACEMENT_NEW(ParticleSmoothingDesc)();
    smoothingDesc->systemPath = usdPrim.GetPath();

    smoothingAPI.GetParticleSmoothingEnabledAttr().Get(&smoothingDesc->enableSmoothing);

    float strength;
    smoothingAPI.GetStrengthAttr().Get(&strength);
    smoothingDesc->strength = GfClamp(strength, 0.0f, 1.0f);

    return smoothingDesc;
}

ParticleIsosurfaceDesc* ParseParticleIsosurface(const UsdPrim& usdPrim)
{
    if (!usdPrim.IsA<PhysxSchemaPhysxParticleSystem>() || !usdPrim.HasAPI<PhysxSchemaPhysxParticleIsosurfaceAPI>())
    {
        CARB_LOG_WARN("ParseParticleIsosurface: UsdPrim is not a PhysxSchemaPhysxParticleSystem or doesn't have a PhysxSchemaPhysxParticleIsosurfaceAPI\n");
        return nullptr;
    }

    PhysxSchemaPhysxParticleSystem particleSystem(usdPrim);
    PhysxSchemaPhysxParticleIsosurfaceAPI isosurfaceAPI(usdPrim);

    ParticleIsosurfaceDesc* isosurfaceDesc = ICE_PLACEMENT_NEW(ParticleIsosurfaceDesc)();
    isosurfaceDesc->systemPath = usdPrim.GetPath();

    float fluidRestOffset = 0.0f;
    particleSystem.GetFluidRestOffsetAttr().Get(&fluidRestOffset);
    float particleContactOffset = 0.0f;

    particleSystem.GetParticleContactOffsetAttr().Get(&particleContactOffset);
    float mpu = float(UsdGeomGetStageMetersPerUnit(usdPrim.GetStage()));
    if (!particleSystem.GetParticleContactOffsetAttr().HasAuthoredValue())
    {
        // this means we are using the schema default
        particleContactOffset = particleContactOffset / mpu;
    }

    fluidRestOffset = completeFluidRestOffset(usdPrim.GetStage(), fluidRestOffset, particleContactOffset);

    isosurfaceAPI.GetIsosurfaceEnabledAttr().Get(&isosurfaceDesc->enableIsosurface);
    isosurfaceAPI.GetMaxVerticesAttr().Get(&isosurfaceDesc->maxIsosurfaceVertices);
    isosurfaceAPI.GetMaxTrianglesAttr().Get(&isosurfaceDesc->maxIsosurfaceTriangles);
    isosurfaceAPI.GetMaxSubgridsAttr().Get(&isosurfaceDesc->maxNumIsosurfaceSubgrids);
    isosurfaceAPI.GetGridSpacingAttr().Get(&isosurfaceDesc->gridSpacing);

    if (isosurfaceDesc->gridSpacing < 0.0f)
    {
        isosurfaceDesc->gridSpacing = fluidRestOffset * 1.5f; // autocompute
    }
    else if (isosurfaceDesc->gridSpacing <= fluidRestOffset * 0.9f)
    {
        isosurfaceDesc->gridSpacing = fluidRestOffset * 0.9f;
        REPORT_PHYSICS_MESSAGE(usdparser::ErrorCode::eInfo,
            "Adjusting Grid Spacing to a minimum of 0.9 times the Fluid Rest Offset (%f).",
            isosurfaceDesc->gridSpacing);
    }

    isosurfaceAPI.GetSurfaceDistanceAttr().Get(&isosurfaceDesc->surfaceDistance);

    if (isosurfaceDesc->surfaceDistance < 0.0f)
    {
        isosurfaceDesc->surfaceDistance = fluidRestOffset * 1.6f; // autocompute
    }
    else if (isosurfaceDesc->surfaceDistance > 2.5f*isosurfaceDesc->gridSpacing)
    {
        isosurfaceDesc->surfaceDistance = isosurfaceDesc->gridSpacing * 2.5f;
        REPORT_PHYSICS_MESSAGE(usdparser::ErrorCode::eInfo,
            "Adjusting Surface Distance to a maximum of 2.5 times the Grid Spacing (%f).",
            isosurfaceDesc->surfaceDistance);
    }

    std::string gridFilteringPassesStr;
    isosurfaceAPI.GetGridFilteringPassesAttr().Get(&gridFilteringPassesStr);
    ParseGridFilteringPasses(gridFilteringPassesStr, isosurfaceDesc->gridFilteringPasses);

    isosurfaceAPI.GetGridSmoothingRadiusAttr().Get(&isosurfaceDesc->gridSmoothingRadius);
    if (isosurfaceDesc->gridSmoothingRadius < 0.0f)
    {
        isosurfaceDesc->gridSmoothingRadius = fluidRestOffset * 2.0f; // autocompute
    }

    isosurfaceAPI.GetNumMeshSmoothingPassesAttr().Get(&isosurfaceDesc->numMeshSmoothingPasses);
    isosurfaceAPI.GetNumMeshNormalSmoothingPassesAttr().Get(&isosurfaceDesc->numMeshNormalSmoothingPasses);

    return isosurfaceDesc;
}

uint64_t edgeKey(uint32_t edgeA, uint32_t edgeB)
{
    if (edgeA <= edgeB)
        return (uint64_t(edgeA) << 32) | edgeB;
    else
        return (uint64_t(edgeB) << 32) | edgeA;
}

const int localEdges[3][2] = { { 0, 1 }, { 0, 2 }, { 1, 2 } };

bool isMeshWatertight(const std::vector<uint32_t>& triangleIndices, int triangleIndicesCount, uint32_t* indexMap = nullptr)
{
    std::map<uint64_t, int> edges;
    for (int i = 0; i < triangleIndicesCount; i += 3)
    {
        for (int j = 0; j < 3; ++j)
        {
            uint32_t a = triangleIndices[i + localEdges[j][0]];
            uint32_t b = triangleIndices[i + localEdges[j][1]];
            if (indexMap)
            {
                a = indexMap[a];
                b = indexMap[b];
            }
            uint64_t k = edgeKey(a, b);
            auto iterator = edges.find(k);
            if (iterator != edges.end())
                iterator->second++;
            else
                edges[k] = 1;
        }
    }

    for (auto it = edges.begin(); it != edges.end(); it++)
        if (it->second != 2)
            return false;

    return true;
}

ParticleClothDesc* ParseParticleClothDeprecated(AttachedStage& attachedStage, const UsdPrim& usdPrim)
{
    if (!usdPrim.IsA<UsdGeomMesh>())
    {
        CARB_LOG_WARN("PhysxSchemaPhysxParticleClothAPI parsing: UsdPrim isn't of type UsdGeomMesh.");
        return nullptr;
    }
    UsdGeomMesh simMesh(usdPrim);

    if (!usdPrim.HasAPI<PhysxSchemaPhysxParticleClothAPI>())
    {
        CARB_LOG_WARN("PhysxSchemaPhysxParticleClothAPI parsing: UsdPrim doesn't have a PhysxSchemaPhysxParticleClothAPI.");
        return nullptr;
    }
    PhysxSchemaPhysxParticleClothAPI particleClothAPI(usdPrim);
    PhysxSchemaPhysxParticleAPI particleAPI(particleClothAPI);

    SdfPath particleSystemPath = GetParticleSystemPath(particleAPI);
    if (particleSystemPath.IsEmpty())
    {
        CARB_LOG_WARN("PhysxSchemaPhysxParticleClothAPI parsing: No valid ParticleSystem Relationship found.");
        return nullptr;
    }

    PhysxSchemaPhysxParticleSystem particleSystem = PhysxSchemaPhysxParticleSystem::Get(usdPrim.GetStage(), particleSystemPath);
    if (!particleSystem)
    {
        CARB_LOG_WARN("PhysxSchemaPhysxParticleClothAPI: ParticleSystem Relationship doesn't point to valid ParticleSystem.");
        return nullptr;
    }
    UsdPrim particleSystemPrim = particleSystem.GetPrim();

    ParticleClothDesc* clothDesc = ICE_PLACEMENT_NEW(ParticleClothDesc)();
    clothDesc->primPath = usdPrim.GetPath();
    clothDesc->particleSystemPath = particleSystemPath;

    if (clothDesc->particleSystemPath.IsEmpty())
    {
        CARB_LOG_ERROR(
            "PhysxSchemaPhysxParticleClothAPI: Particle system reference from prim %s could not be found.", usdPrim.GetPath().GetString().c_str());
        ICE_FREE(clothDesc);
        return nullptr;
    }

    bool isValid = true;

    VtArray<GfVec3f> points;
    SafeGetAttributeP(&points, simMesh.GetPointsAttr());

    VtArray<GfVec3f> velocities;
    UsdAttribute velocitiesAttr = simMesh.GetVelocitiesAttr();
    if (velocitiesAttr.HasAuthoredValue())
    {
        SafeGetAttributeP(&velocities, velocitiesAttr);
    }
    else
    {
        velocities.assign(points.size(), GfVec3f(0.0f));
        velocitiesAttr.Set(velocities);
    }

    if (velocities.size() != points.size())
    {
        CARB_LOG_WARN("UsdGeomMesh:velocities and points have inconsistent sizes.");
        isValid = false;
    }

    VtArray<GfVec3f> restPoints;
    UsdAttribute restPointsAttr = particleClothAPI.GetRestPointsAttr();
    if (restPointsAttr.HasAuthoredValue())
    {
        SafeGetAttributeP(&restPoints, restPointsAttr);
    }
    else
    {
        restPoints = points;
        restPointsAttr.Set(restPoints);
    }

    if (restPoints.size() != points.size())
    {
        CARB_LOG_WARN("PhysxSchemaPhysxParticleClothAPI:restPoints and UsdGeomMesh:points have inconsistent sizes.");
        isValid = false;
    }

    VtArray<uint32_t> weldedTriangleIndices;
    UsdAttribute weldedTriangleIndicesAttr = particleClothAPI.GetPrim().GetAttribute(physxParticleWeldedTriangleIndicesToken);
    if (weldedTriangleIndicesAttr.HasAuthoredValue())
    {
        SafeGetAttributeP(&weldedTriangleIndices, weldedTriangleIndicesAttr);
    }

    PhysxSchemaPhysxAutoParticleClothAPI autoParticleCloth(usdPrim);
    bool disableWelding = true;
    if (autoParticleCloth)
    {
        autoParticleCloth.GetDisableMeshWeldingAttr().Get(&disableWelding);
    }
    clothDesc->isWelded = !disableWelding && weldedTriangleIndices.size();

    if (clothDesc->isWelded)
    {
        if (!convertIndexBuffer(clothDesc->triangleIndices, weldedTriangleIndices, 3, uint32_t(points.size())))
        {
            CARB_LOG_WARN("PhysxSchemaPhysxParticleClothAPI:weldedTriangleIndices has invalid size or contains out of range indices.");
            isValid = false;
        }

        VtArray<uint32_t> verticesRemapToWeld;
        UsdAttribute verticesRemapToWeldAttr = particleClothAPI.GetPrim().GetAttribute(physxParticleWeldedVerticesRemapToWeldToken);
        if (verticesRemapToWeldAttr.HasAuthoredValue())
        {
            SafeGetAttributeP(&verticesRemapToWeld, verticesRemapToWeldAttr);
        }

        VtArray<uint32_t> verticesRemapToOrig;
        UsdAttribute verticesRemapToOrigAttr = particleClothAPI.GetPrim().GetAttribute(physxParticleWeldedVerticesRemapToOrigToken);
        if (verticesRemapToOrigAttr.HasAuthoredValue())
        {
            SafeGetAttributeP(&verticesRemapToOrig, verticesRemapToOrigAttr);
        }

        bool toWeldValid = verticesRemapToWeld.size() == points.size();
        uint32_t toWeldRange = uint32_t(verticesRemapToOrig.size());
        uint32_t toWeldSentinel = uint32_t(-1);
        toWeldValid &= convertIndexBuffer(clothDesc->verticesRemapToWeld, verticesRemapToWeld, 1, toWeldRange, &toWeldSentinel);
        if (!toWeldValid)
        {
            CARB_LOG_WARN("PhysxSchemaPhysxParticleClothAPI:weldedVerticesRemapToWeld has invalid size or contains out of range indices.");
            isValid = false;
        }
        if (!convertIndexBuffer(clothDesc->verticesRemapToOrig, verticesRemapToOrig, 1, uint32_t(points.size())))
        {
            CARB_LOG_WARN("PhysxSchemaPhysxParticleClothAPI:weldedVerticesRemapToOrig has invalid size or contains out of range indices.");
            isValid = false;
        }

        if (isValid)
        {
            // Get welded points through remap table
            clothDesc->points.resize(verticesRemapToOrig.size());
            clothDesc->restPoints.resize(verticesRemapToOrig.size());
            clothDesc->velocities.resize(verticesRemapToOrig.size());
            for (uint32_t weldedIndex = 0; weldedIndex < verticesRemapToOrig.size(); ++weldedIndex)
            {
                const uint32_t origIndex = verticesRemapToOrig[weldedIndex];
                clothDesc->points[weldedIndex] = (carb::Float3&)points[origIndex];
                clothDesc->restPoints[weldedIndex] = (carb::Float3&)restPoints[origIndex];
                clothDesc->velocities[weldedIndex] = (carb::Float3&)velocities[origIndex];
            }
        }
    }
    else
    {
        if (!ExtractTriangulatedFaces(clothDesc->triangleIndices, simMesh))
        {
            CARB_LOG_WARN("PhysxSchemaPhysxParticleClothAPI extracting triangle faces from UsdGeomMesh failed.");
            isValid = false;
        }

        convert(clothDesc->points, points);
        convert(clothDesc->restPoints, restPoints);
        convert(clothDesc->velocities, velocities);
    }

    clothDesc->numParticles = (int)clothDesc->points.size();

    std::vector<carb::Float3>& scaledRestPoints = clothDesc->scaledRestPoints;
    scaledRestPoints.resize(clothDesc->restPoints.size());
    //scale rest points, to account for correct area when applying particle masses
    {
        GfMatrix4d localToWorld = simMesh.ComputeLocalToWorldTransform(UsdTimeCode::Default());
        GfVec3f scale = GfVec3f(GfTransform(localToWorld).GetScale());
        for (size_t i = 0; i < scaledRestPoints.size(); ++i)
        {
            const GfVec3f& src = (GfVec3f&)clothDesc->restPoints[i];
            GfVec3f& dst = (GfVec3f&)scaledRestPoints[i];
            dst = GfCompMult(src, scale);
        }
    }

    ParticleSystemDesc* desc = ParseParticleSystem(attachedStage, particleSystemPrim);
    clothDesc->scenePath = desc->scenePath;
    clothDesc->restOffset = desc->restOffset;
    ICE_FREE(desc);

    // only parse mass property if it was provided
    UsdPhysicsMassAPI massApi(usdPrim);
    const UsdAttribute massAttribute = massApi.GetMassAttr();
    if (massAttribute.HasAuthoredValue())
    {
        massAttribute.Get(&clothDesc->mass);
    }
    const UsdAttribute densityAttribute = massApi.GetDensityAttr();
    if (densityAttribute.HasAuthoredValue())
    {
        densityAttribute.Get(&clothDesc->density);
    }

    SafeGetAttributeP(&clothDesc->enabled, particleAPI.GetParticleEnabledAttr());
    SafeGetAttributeP(&clothDesc->selfCollision, particleAPI.GetSelfCollisionAttr(), true);
    SafeGetAttributeP(&clothDesc->selfCollisionFilter, particleClothAPI.GetSelfCollisionFilterAttr(), true);
    SafeGetAttributeP(&clothDesc->particleGroup, particleAPI.GetParticleGroupAttr(), 0);

    VtArray<GfVec2i> springIndices;
    SafeGetAttributeP(&springIndices, particleClothAPI.GetSpringIndicesAttr());

    if (springIndices.size() == 0)
    {
        CARB_LOG_WARN("PhysxSchemaPhysxParticleClothAPI:springIndices has no elements.");
        isValid = false;
    }

    for (const GfVec2i& indices : springIndices)
    {
        if (indices[0] == indices[1])
        {
            CARB_LOG_WARN("PhysxSchemaPhysxParticleClothAPI:springIndices has duplicate indices in one spring.");
            isValid = false;
            break;
        }
    }
    clothDesc->springIndices.resize(springIndices.size());
    int32_t* springIndicesDst = (int32_t*)clothDesc->springIndices.data();
    const int32_t* springIndicesSrc = (int32_t*)springIndices.data();
    const uint32_t springIndicesSize = uint32_t(clothDesc->springIndices.size() * 2);
    if (!convertIndexBuffer(springIndicesDst, springIndicesSrc, springIndicesSize, 2, clothDesc->numParticles))
    {
        CARB_LOG_WARN("PhysxSchemaPhysxParticleClothAPI:springIndices has invalid size or contains out of range indices.");
        isValid = false;
    }

    VtArray<float> springStiffnesses;
    SafeGetAttributeP(&springStiffnesses, particleClothAPI.GetSpringStiffnessesAttr());
    if (springStiffnesses.size() != springIndices.size())
    {
        CARB_LOG_WARN("PhysxSchemaPhysxParticleClothAPI:springStiffnesses and springIndices have inconsistent sizes.");
        isValid = false;
    }
    convert(clothDesc->springStiffnesses, springStiffnesses);

    VtArray<float> springDampings;
    SafeGetAttributeP(&springDampings, particleClothAPI.GetSpringDampingsAttr());
    if (springDampings.size() != springIndices.size())
    {
        CARB_LOG_WARN("PhysxSchemaPhysxParticleClothAPI:springDampings and springIndices have inconsistent sizes.");
        isValid = false;
    }
    convert(clothDesc->springDampings, springDampings);

    VtArray<float> springRestLengths;
    SafeGetAttributeP(&springRestLengths, particleClothAPI.GetSpringRestLengthsAttr());
    if (springRestLengths.size() != springIndices.size())
    {
        CARB_LOG_WARN("PhysxSchemaPhysxParticleClothAPI:springRestLengths and springIndices have inconsistent sizes.");
        isValid = false;
    }
    convert(clothDesc->springRestLengths, springRestLengths);

    SafeGetAttributeP(&clothDesc->pressure, particleClothAPI.GetPressureAttr(), 0.0f);
    SafeGetAttributeP(&clothDesc->inflatableVolume, particleClothAPI.GetPrim().GetAttribute(physxParticleInflatableVolumeToken), 0.0f);

    if (isValid)
    {
        bool isWatertight = isMeshWatertight(clothDesc->triangleIndices, (int)clothDesc->triangleIndices.size());
        if (!isWatertight && clothDesc->pressure > 0.0f)
        {
            CARB_LOG_WARN("Non watertight mesh on inflatable is not supported.");
            isValid = false;
        }
    }

    if (!isValid)
    {
        CARB_LOG_WARN("PhysxSchemaPhysxParticleClothAPI parsing failed: %s", usdPrim.GetPrimPath().GetText());
        ICE_FREE(clothDesc);
        return nullptr;
    }

    return clothDesc;
}

ParticleSamplingDesc* ParseParticleSampling(AttachedStage& attachedStage, const UsdPrim& usdPrim)
{
    if (!usdPrim.IsA<UsdGeomMesh>())
    {
        CARB_LOG_WARN("ParseParticleSampling: UsdPrim isn't of type UsdGeomMesh\n");
        return nullptr;
    }
    UsdGeomMesh mesh(usdPrim);

    if (!usdPrim.HasAPI<PhysxSchemaPhysxParticleSamplingAPI>())
    {
        CARB_LOG_WARN("ParseParticleSampling: UsdPrim doesn't have a PhysxSchemaPhysxParticleSamplingAPI\n");
        return nullptr;
    }
    PhysxSchemaPhysxParticleSamplingAPI samplingAPI(usdPrim);

    ParticleSamplingDesc* samplingDesc = ICE_PLACEMENT_NEW(ParticleSamplingDesc)();

    // sampling api
    SafeGetAttributeP(&samplingDesc->samplingDistance, samplingAPI.GetSamplingDistanceAttr(), -1.0f);
    SafeGetAttributeP(&samplingDesc->maxSamples, samplingAPI.GetMaxSamplesAttr(), 0);
    SafeGetAttributeP(&samplingDesc->sampleVolume, samplingAPI.GetVolumeAttr(), true);

    if (UsdRelationship rel = samplingAPI.GetParticlesRel())
    {
        SdfPathVector paths;
        if (rel.GetTargets(&paths) && paths.size() > 0)
        {
            samplingDesc->particleSetPath = paths[0];
        }
    }

    // particle set
    SdfPath particleSystemPath;
    bool fluid;
    PhysxSchemaPhysxParticleSetAPI particleSetAPI(attachedStage.getStage()->GetPrimAtPath(samplingDesc->particleSetPath));
    PhysxSchemaPhysxParticleAPI particleAPI(particleSetAPI);

    if (particleSetAPI)
    {
        SafeGetAttributeP(&fluid, particleSetAPI.GetFluidAttr(), true);
        particleSystemPath = GetParticleSystemPath(particleAPI);
    }

    // default point size
    float particleSystemPointWidth = 0.5f;
    if (!particleSystemPath.IsEmpty())
    {
        ParticleSystemDesc* systemDesc = ParseParticleSystem(attachedStage, attachedStage.getStage()->GetPrimAtPath(particleSystemPath));
        if (systemDesc)
        {
            particleSystemPointWidth = fluid ? 2.0f * systemDesc->fluidRestOffset : 2.0f * systemDesc->solidRestOffset;

            // switch to particle contact offset - can never be 0 according to schema.
            if (particleSystemPointWidth == 0.0f)
            {
                REPORT_PHYSICS_MESSAGE(usdparser::ErrorCode::eInfo,
                "Switching to Particle Contact Offset to determine a suitable sampling distance because Particle Rest Offset is 0.");
                particleSystemPointWidth = systemDesc->particleContactOffset;
            }

            ICE_FREE(systemDesc);
        }
    }

    samplingDesc->pointWidth = particleSystemPointWidth;

    // handle autocompute and limits for sampling distance
    if (samplingDesc->samplingDistance <= 0.0f) // autocompute
    {
        samplingDesc->samplingDistance = particleSystemPointWidth;
    }

    // reasonable limit to avoid particles exploding
    if (samplingDesc->samplingDistance <= 0.75f * particleSystemPointWidth)
    {
        samplingDesc->samplingDistance = 0.75f * particleSystemPointWidth;
    }

    return samplingDesc;
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

float completeRestOffset(UsdStageWeakPtr stage, float restOffset, float particleContactOffset)
{
    if (restOffset < 0.0f)
    {
        particleContactOffset = completeParticleContactOffset(stage, particleContactOffset);
        restOffset = particleContactOffset * 0.99f;
    }

    return restOffset;
}

float completeContactOffset(UsdStageWeakPtr stage, float contactOffset, float particleContactOffset)
{
    if (contactOffset < 0.0f)
    {
        particleContactOffset = completeParticleContactOffset(stage, particleContactOffset);
        contactOffset = particleContactOffset;
    }

    return contactOffset;
}

float completeFluidRestOffset(UsdStageWeakPtr stage, float fluidRestOffset, float particleContactOffset)
{
    if (fluidRestOffset < 0.0f)
    {
        particleContactOffset = completeParticleContactOffset(stage, particleContactOffset);
        fluidRestOffset = (particleContactOffset * 0.99f) * 0.6f;
    }

    return fluidRestOffset;
}

float completeSolidRestOffset(UsdStageWeakPtr stage, float solidRestOffset, float particleContactOffset)
{
    if (solidRestOffset < 0.0f)
    {
        particleContactOffset = completeParticleContactOffset(stage, particleContactOffset);
        solidRestOffset = particleContactOffset * 0.99f;
    }

    return solidRestOffset;
}

float completeParticleContactOffset(UsdStageWeakPtr stage, float particleContactOffset)
{
    if (particleContactOffset <= 0.0f) // still set to default if less than 0.
    {
        float mpu = float(UsdGeomGetStageMetersPerUnit(stage));
        // there needs to be some way to get the default from the schema..
        particleContactOffset = DEFAULT_PARTICLE_CONTACT_OFFSET / mpu;
    }

    return particleContactOffset;
}
} // namespace usdparser
} // namespace physx
} // namespace omni
