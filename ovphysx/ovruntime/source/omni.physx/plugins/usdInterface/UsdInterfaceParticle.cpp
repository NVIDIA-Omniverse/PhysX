// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include "UsdPCH.h"

#include "UsdInterface.h"

#include <usdLoad/LoadUsd.h>
#include <usdLoad/Material.h>
#include <usdLoad/Mass.h>

#include <particles/PhysXParticlePost.h>
#include <common/utilities/UsdMaterialParsing.h>

#include <PhysXTools.h>

#if USE_PHYSX_GPU
#include <extensions/PxParticleExt.h>
#endif

using namespace omni::physx::usdparser;
using namespace PXR_NS;
using namespace ::physx;
using namespace omni::physx::internal;
using namespace omni::physx;

namespace
{

InternalPbdParticleSystem* getParticleSystem(usdparser::AttachedStage& attachedStage, SdfPath path)
{
    const ObjectId systemId = attachedStage.getObjectDatabase()->findEntry(path, eParticleSystem);
    if (systemId != kInvalidObjectId)
    {
        void* objectRecord = OmniPhysX::getInstance().getInternalPhysXDatabase().getInternalTypedRecord(ePTParticleSystem, systemId);
        if (objectRecord)
        {
            return reinterpret_cast<InternalPbdParticleSystem*>(objectRecord);
        }
    }

    InternalPbdParticleSystem* internalPS = nullptr;
    const PhysXScenesMap& physxScenes = OmniPhysX::getInstance().getPhysXSetup().getPhysXScenes();

    for (PhysXScenesMap::const_reference ref : physxScenes)
    {
        const PhysXScene* sc = ref.second;

        for (size_t particleSystemIndex = 0; particleSystemIndex < sc->getInternalScene()->mParticleSystems.size(); particleSystemIndex++)
        {
            if (sc->getInternalScene()->mParticleSystems[particleSystemIndex]->getPath() == path)
            {
                internalPS = sc->getInternalScene()->mParticleSystems[particleSystemIndex];
                break;
            }
        }

        if (internalPS)
            break;
    }
    return internalPS;
}

ObjectId getPBDParticleSystemMaterialId(const PhysXScene& scene, const ParticleSystemDesc& particleSystemDesc)
{
    ObjectId materialId = kInvalidObjectId;
    if (particleSystemDesc.material != kInvalidObjectId)
    {
        materialId = particleSystemDesc.material;
    }
    else
    {
        // Scene might have a default material with InternalPBDParticleMaterial wrapper and valid ObjectId
        PxPBDMaterial* material = scene.getDefaultPBDMaterial();
        if (material && material->userData)
        {
            materialId = (size_t)material->userData;
        }
    }
    return materialId;
}

PxPBDMaterial* getPBDMaterial(InternalPBDParticleMaterial** internalMaterial, const PhysXScene& scene, ObjectId materialId)
{
    if (internalMaterial)
    {
        *internalMaterial = getInternalPtr<InternalPBDParticleMaterial>(ePTPBDMaterial, materialId);
    }
    PxPBDMaterial* material = getPtr<PxPBDMaterial>(ePTPBDMaterial, materialId);
    if (!material)
    {
        // Default material migth not have InternalPBDParticleMaterial wrapper and valid ObjectId
        material = scene.getDefaultPBDMaterial();
    }
    return material;
}

uint32_t allocatePhase(InternalPbdParticleSystem& particleSystem, physx::PxPBDMaterial& material,
    int group, bool selfCollision, bool selfCollisionFilter, bool fluid)
{
    PxParticlePhaseFlags phaseFlags = PxParticlePhaseFlags(0);
    if (selfCollision)
        phaseFlags |= PxParticlePhaseFlags(PxParticlePhaseFlag::eParticlePhaseSelfCollide);

    if (selfCollisionFilter)
        phaseFlags |= PxParticlePhaseFlags(PxParticlePhaseFlag::eParticlePhaseSelfCollideFilter);

    if (fluid)
        phaseFlags |= PxParticlePhaseFlags(PxParticlePhaseFlag::eParticlePhaseFluid);

    auto result = particleSystem.mPhaseMap.find(group);

    PxU32 phase = 0u;

    if (result == particleSystem.mPhaseMap.end())
    {
        // Do not pass phase flags in
        phase = particleSystem.mPS->createPhase(&material, PxParticlePhaseFlags(0)); 
        particleSystem.mPhaseMap[group] = phase;
    }
    else
    {
        phase = result->second;
    }

    phase |= PxU32(phaseFlags);
    return phase;
}

void getParticleMassDensity(const ParticleDesc* desc, const float materialDensity, float& mass, float& density)
{
    PBDMaterialDesc materialDesc;
    setToDefault(materialDesc);
    float defaultDensity = materialDesc.density;

    density = materialDensity;
    mass = desc->mass;
    if (desc->density > 0.0f)
    {
        density = desc->density;
    }

    if (mass <= 0.0f && density <= 0.0f)
    {
        // set default if no mass specified and both massAPI and material density are invalid/not provided:
        density = getScaledDensity(usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage()->getSourceUnits(), defaultDensity);
    }
}

carb::Float3 sub(const carb::Float3& a, const carb::Float3& b)
{
    return carb::Float3{ a.x - b.x, a.y - b.y, a.z - b.z };
}

float length(const carb::Float3& d)
{
    return std::sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
}

float area(const carb::Float3& a, const carb::Float3& b, const carb::Float3& c)
{
    carb::Float3 ab = sub(b, a);
    carb::Float3 ac = sub(c, a);
    carb::Float3 cross{ ab.y * ac.z - ab.z * ac.y, ab.z * ac.x - ab.x * ac.z, ab.x * ac.y - ab.y * ac.x };
    return 0.5f * length(cross);
}

void inverseMass(std::vector<float>& massPerParticle)
{
    for (int i = 0; i < massPerParticle.size(); i++)
        massPerParticle[i] = 1.0f / massPerParticle[i];
}

#define USE_AREA_DISTRIBUTION 0
void distributeMassOnTriangleMesh(std::vector<carb::Float3>& positions, std::vector<uint32_t>& triangleIndices, float density, float clothThickness, std::vector<float>& massPerParticle)
{
    massPerParticle.clear();
    massPerParticle.resize(positions.size(), 0.0f);

    float totalArea = 0.0f;
    for (int i = 0; i < triangleIndices.size(); i += 3)
    {
        const uint32_t triIndex0 = triangleIndices[i];
        const uint32_t triIndex1 = triangleIndices[i + 1];
        const uint32_t triIndex2 = triangleIndices[i + 2];

#if USE_AREA_DISTRIBUTION
        float triangleAreaDiv3 = (1.0f / 3.0f) * area(positions[triIndex0], positions[triIndex1], positions[triIndex2]);
        massPerParticle[triIndex0] += triangleAreaDiv3 * clothThickness * density;
        massPerParticle[triIndex1] += triangleAreaDiv3 * clothThickness * density;
        massPerParticle[triIndex2] += triangleAreaDiv3 * clothThickness * density;
#else
        totalArea += area(positions[triIndex0], positions[triIndex1], positions[triIndex2]);
#endif
    }

#if !USE_AREA_DISTRIBUTION
    float totalMass = totalArea * clothThickness * density;
    massPerParticle.assign(positions.size(), totalMass / positions.size());
#endif
}

void distributeMassOnTriangleMesh(std::vector<carb::Float3>& positions, std::vector<uint32_t>& triangleIndices, float targetMass, std::vector<float>& massPerParticle)
{
    distributeMassOnTriangleMesh(positions, triangleIndices, 1.0f, 1.0f, massPerParticle);
    float m = 0.0f;
    for (int i = 0; i < massPerParticle.size(); ++i)
        m += massPerParticle[i];

    const float massScale = targetMass / m;
    for (int i = 0; i < massPerParticle.size(); ++i)
        massPerParticle[i] *= massScale;
}

// schemaTypeToken lives in PhysXTools.h (single boundary translation).
using omni::physx::internal::schemaTypeToken;

} // namespace

namespace omni
{
namespace physx
{

extern bool checkScenes();

float computeParticleInvMass(const ParticleSetDesc& particleSetDesc, const float materialDensity)
{
    float computedMass = 0.0f;
    float density;
    float mass;
    getParticleMassDensity(&particleSetDesc, materialDensity, mass, density);
    if (mass > 0.0f)
    {
        computedMass = mass / particleSetDesc.points.size();
    }
    else if (density > 0.0f)
    {
        float restOffsetVol;

        if (particleSetDesc.fluid)
            restOffsetVol = particleSetDesc.fluidRestOffset * particleSetDesc.fluidRestOffset * particleSetDesc.fluidRestOffset;
        else
            restOffsetVol = particleSetDesc.solidRestOffset * particleSetDesc.solidRestOffset * particleSetDesc.solidRestOffset;
        // Use FLT_EPSILON to ensure that mass will never be 0
        restOffsetVol = std::max(restOffsetVol, FLT_EPSILON);
        computedMass = 8 * restOffsetVol * density;
    }
    else
    {
        CARB_ASSERT(0);
    }

    return 1.0f / computedMass;
}

bool PhysXUsdPhysicsInterface::updateParticleMass(const SdfPath& path, ObjectId objectId, const ParticleDesc& particleDesc)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    PhysXType internalType;
    const InternalDatabase::Record* objectFullRecord = db.getFullRecord(internalType, objectId);
    if (!objectFullRecord || !objectFullRecord->mPtr)
        return false;

    InternalParticle* internalParticle = (InternalParticle*)objectFullRecord->mInternalPtr;
    if (!internalParticle || !internalParticle->mParentParticleSystem)
        return false;

    const ObjectId materialId = internalParticle->mParentParticleSystem->mMaterialId;
    InternalPBDParticleMaterial* internalMaterial = getInternalPtr<InternalPBDParticleMaterial>(ePTPBDMaterial, materialId);
    const float materialDensity = internalMaterial ? internalMaterial->mDensity : 0.0f;

    if (internalType == ePTParticleSet)
    {
        InternalParticleSet* internalParticleSet = (InternalParticleSet*)objectFullRecord->mInternalPtr;
        if (internalParticleSet)
        {
            const ParticleSetDesc& particleSetDesc = (const ParticleSetDesc&)particleDesc;

            internalParticleSet->mParticleInvMass = computeParticleInvMass(particleSetDesc, materialDensity);

            for (int index = 0; index < particleSetDesc.maxParticles; ++index)
            {
                internalParticleSet->mPositions[index] = PxVec4(internalParticleSet->mPositions[index].getXYZ(), internalParticleSet->mParticleInvMass);
            }

            internalParticleSet->mUploadDirtyFlags = ParticleBufferFlags::eALL;
        }
    }

    return true;
}

usdparser::ObjectId PhysXUsdPhysicsInterface::createPbdParticleSystem(usdparser::AttachedStage& attachedStage, const SdfPath& path, const usdparser::ParticleSystemDesc& desc)
{
    if (!checkScenes())
    {
        PhysXUsdPhysicsInterface::reportLoadError(
            usdparser::ErrorCode::eError, "No physics scene created, please add physics scene into stage!");
        return kInvalidObjectId;
    }

    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    PxPhysics* physics = physxSetup.getPhysics();

    const ObjectId sceneId = attachedStage.getObjectDatabase()->findEntry(desc.scenePath, eScene);
    PhysXScene* physxScene = physxSetup.getPhysXScene(sceneId);

    PxCudaContextManager* cudaContextManager = physxScene->getScene()->getCudaContextManager();
    if (!cudaContextManager)
    {
        CARB_LOG_ERROR("PhysX Particle System creation failed due to missing PxCudaContextManager.");
        return kInvalidObjectId;
    }

    InternalPbdParticleSystem* internalPS = ICE_NEW(InternalPbdParticleSystem)(physxScene);

    PxPBDParticleSystem* particleSystem = physics->createPBDParticleSystem(*cudaContextManager,
        desc.maxNeighborhood, desc.neighborhoodScale);

    particleSystem->setParticleFlag(PxParticleFlag::eENABLE_SPECULATIVE_CCD, desc.enableCCD);
    particleSystem->setContactOffset(desc.contactOffset);
    particleSystem->setRestOffset(desc.restOffset);
    particleSystem->setParticleContactOffset(desc.particleContactOffset);
    particleSystem->setSolidRestOffset(desc.solidRestOffset);
    particleSystem->setFluidRestOffset(desc.fluidRestOffset);

    particleSystem->setMaxDepenetrationVelocity(desc.maxDepenetrationVelocity);
    particleSystem->setSolverIterationCounts(
        physxScene->getInternalScene()->clampPosIterationCount(desc.solverPositionIterations), 1);
    particleSystem->setWind(PxVec3(desc.wind.x, desc.wind.y, desc.wind.z));
    particleSystem->setMaxLinearVelocity(desc.maxVelocity);
    particleSystem->setFluidBoundaryDensityScale(desc.fluidBoundaryDensityScale);

    if (desc.lockedAxis)
    {
        if (desc.lockedAxis & 1 << 0)
            particleSystem->setParticleLockFlag(PxParticleLockFlag::eLOCK_X, true);
        if (desc.lockedAxis & 1 << 1)
            particleSystem->setParticleLockFlag(PxParticleLockFlag::eLOCK_Y, true);
        if (desc.lockedAxis & 1 << 2)
            particleSystem->setParticleLockFlag(PxParticleLockFlag::eLOCK_Z, true);
    }

    physxScene->getScene()->addActor(*particleSystem);

    internalPS->mPS = particleSystem;
    internalPS->mKey = attachedStage.keyFor(path);
    internalPS->mEnabled = desc.enableParticleSystem;
    internalPS->mMaterialId = getPBDParticleSystemMaterialId(*physxScene, desc);

    physxScene->getInternalScene()->mParticleSystems.push_back(internalPS);

#if USE_PHYSX_GPU
    internalPS->mCallback = ICE_NEW(particles::PostProcessCallback)(internalPS);
    particleSystem->setParticleSystemCallback(internalPS->mCallback);
#endif

    uint32_t postFlags = ParticlePostFlag::eNone;
    postFlags |= desc.enableAnisotropy ? ParticlePostFlag::eAnisotropy : 0;
    postFlags |= desc.enableSmoothing ? ParticlePostFlag::eSmoothing : 0;
    postFlags |= desc.enableIsosurface ? ParticlePostFlag::eIsosurface : 0;
    internalPS->setPost(postFlags);

    if (desc.enableAnisotropy)
        attachedStage.getObjectDatabase()->addSchemaAPI(desc.systemPath, SchemaAPIFlag::eParticleAnisotropyAPI);
    if (desc.enableSmoothing)
        attachedStage.getObjectDatabase()->addSchemaAPI(desc.systemPath, SchemaAPIFlag::eParticleSmoothingAPI);
    if (desc.enableIsosurface)
        attachedStage.getObjectDatabase()->addSchemaAPI(desc.systemPath, SchemaAPIFlag::eParticleIsosurfaceAPI);

    const uint32_t collisionGroup = convertToCollisionGroup(desc.collisionGroup);
    PxFilterData fd;
    convertCollisionGroupToPxFilterData(collisionGroup, fd);
    particleSystem->setSimulationFilterData(fd);

    if (!internalPS->mEnabled)
        physxScene->getScene()->removeActor(*particleSystem);

    const ObjectId objectId = OmniPhysX::getInstance().getInternalPhysXDatabase().addRecord(ePTParticleSystem, particleSystem, internalPS, attachedStage.keyFor(path));
    particleSystem->userData = (void*)(objectId);
    if (mExposePrimNames)
        particleSystem->setName(path.GetText());

    mParticleSystems.push_back(internalPS);
    mDirty = true;

    return objectId;
}

ObjectId PhysXUsdPhysicsInterface::createParticleSet(usdparser::AttachedStage& attachedStage, const SdfPath& path, const ParticleSetDesc& particleDesc)
{
#if USE_PHYSX_GPU
    if (!checkScenes())
    {
        PhysXUsdPhysicsInterface::reportLoadError(
            usdparser::ErrorCode::eError, "No physics scene created, please add physics scene into stage!");
        return kInvalidObjectId;
    }

    InternalPhysXDatabase& internalPhysxDB = OmniPhysX::getInstance().getInternalPhysXDatabase();
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    PxPhysics* physics = physxSetup.getPhysics();

    const ObjectId sceneId = attachedStage.getObjectDatabase()->findEntry(particleDesc.scenePath, eScene);
    PhysXScene* physxScene = physxSetup.getPhysXScene(sceneId);

    PxCudaContextManager* cudaContextManager = physxScene->getScene()->getCudaContextManager();
    if (!cudaContextManager)
    {
        CARB_LOG_ERROR("PhysX Particles creation failed due to missing PxCudaContextManager.");
        return kInvalidObjectId;
    }

    InternalPbdParticleSystem* internalPS = getParticleSystem(attachedStage, particleDesc.particleSystemPath);
    if (!internalPS)
    {
        CARB_LOG_ERROR("PhysX Particles at %s are missing a valid simulation-owner particle system and will not be simulated.", path.GetText());
        return kInvalidObjectId;
    }

    InternalPBDParticleMaterial* internalMaterial;
    PxPBDMaterial* material = getPBDMaterial(&internalMaterial, *physxScene, internalPS->mMaterialId);
    if (!material)
    {
        return kInvalidObjectId;
    }
    const float materialDensity = internalMaterial ? internalMaterial->mDensity : 0.0f;

    UsdStageWeakPtr stage = attachedStage.getStage();
    UsdPrim usdPrim = stage ? stage->GetPrimAtPath(path) : UsdPrim{};
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    const omni::physics::parse::ObjectKey primKey = attachedStage.keyFor(path);

    uint32_t postFlags = particles::getPostprocessStages(internalPS->getPath());
    bool hasSmoothing = postFlags & ParticlePostFlag::eSmoothing;
    bool hasAnisotropy = postFlags & ParticlePostFlag::eAnisotropy;
    bool hasIsosurface = postFlags & ParticlePostFlag::eIsosurface;

    bool renderAnisotropicParticles = hasAnisotropy && !hasIsosurface;
    if (renderAnisotropicParticles && src && src->isA(primKey, schemaTypeToken<UsdGeomPoints>(*src)))
    {
        CARB_LOG_WARN("Physx Particles: cannot render anisotropy using UsdGeomPoints. Consider using point instancers or isosurface rendering.\n");
    }

    InternalParticleSet* internalParticleSet;
    internalParticleSet = ICE_NEW(InternalParticleSet)(*internalPS);

    internalParticleSet->mKey = primKey;
    internalParticleSet->mEnabled = particleDesc.enabled;
    internalParticleSet->mFluid = particleDesc.fluid;
    internalParticleSet->mPhysXScene = physxScene;
    internalParticleSet->mPhase = allocatePhase(*internalPS, *material, particleDesc.particleGroup, particleDesc.selfCollision, false, particleDesc.fluid);
    internalParticleSet->mParticleInvMass = computeParticleInvMass(particleDesc, materialDensity);

    GfMatrix4d localToWorld = getWorldTransform(attachedStage, attachedStage.keyFor(path), UsdTimeCode::Default());

    internalParticleSet->mWorldToLocal = localToWorld.GetInverse();

    internalPS->mParticleSets.push_back(internalParticleSet);

    // allocate some buffers, TODO somehow make InternalParticleSet::resize do all this.
    internalParticleSet->mMaxParticles = particleDesc.maxParticles;
    internalParticleSet->mPhases = PX_PINNED_HOST_ALLOC_T(PxU32, cudaContextManager, particleDesc.maxParticles);
    internalParticleSet->mPositions = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, particleDesc.maxParticles);
    internalParticleSet->mVelocities = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, particleDesc.maxParticles);

    const std::vector<carb::Float3>& positionSrc = particleDesc.simulationPoints.empty() ? particleDesc.points : particleDesc.simulationPoints;

    for (int index = 0; index < particleDesc.numParticles; ++index)
    {
        internalParticleSet->mPhases[index] = internalParticleSet->mPhase;

        GfVec3f localPos = { positionSrc[index].x, positionSrc[index].y, positionSrc[index].z };
        GfVec3f pos = PXR_NS::GfVec3f(localToWorld.Transform(localPos));

        internalParticleSet->mPositions[index] = PxVec4(pos[0], pos[1], pos[2], internalParticleSet->mParticleInvMass);

        internalParticleSet->mVelocities[index] = PxVec4(particleDesc.velocities[index].x, particleDesc.velocities[index].y,
            particleDesc.velocities[index].z, 0.0f);
    }

    internalParticleSet->mNumParticles = (uint32_t)particleDesc.numParticles;

    // diffuse particles
    internalParticleSet->mDiffuseParticlesEnabled = particleDesc.enableDiffuseParticles;
    internalParticleSet->mMaxDiffuseParticleMultiplier = particleDesc.maxDiffuseParticleMultiplier;
    internalParticleSet->mDiffuseParticleParams.threshold = particleDesc.diffuseParticlesThreshold;
    internalParticleSet->mDiffuseParticleParams.lifetime = particleDesc.diffuseParticlesLifetime;
    internalParticleSet->mDiffuseParticleParams.airDrag = particleDesc.diffuseParticlesAirDrag;
    internalParticleSet->mDiffuseParticleParams.bubbleDrag = particleDesc.diffuseParticlesBubbleDrag;
    internalParticleSet->mDiffuseParticleParams.buoyancy = particleDesc.diffuseParticlesBuoyancy;
    internalParticleSet->mDiffuseParticleParams.kineticEnergyWeight = particleDesc.diffuseParticlesKineticEnergyWeight;
    internalParticleSet->mDiffuseParticleParams.pressureWeight = particleDesc.diffuseParticlesPressureWeight;
    internalParticleSet->mDiffuseParticleParams.divergenceWeight = particleDesc.diffuseParticlesDivergenceWeight;
    internalParticleSet->mDiffuseParticleParams.collisionDecay = particleDesc.diffuseParticlesCollisionDecay;

    uint32_t maxDiffuseParticles = InternalParticleSet::getMaxDiffuseParticles(particleDesc.maxParticles,
        particleDesc.enableDiffuseParticles, particleDesc.maxDiffuseParticleMultiplier);

    if (maxDiffuseParticles > 0)
    {
        internalParticleSet->mDiffuseParticlePositions = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, maxDiffuseParticles);
        internalParticleSet->createSharedDiffuseParticles();
        attachedStage.getObjectDatabase()->addSchemaAPI(path, SchemaAPIFlag::eDiffuseParticlesAPI);
    }

    internalParticleSet->mNumDiffuseParticles = 0;

    // AD we always use diffuse buffers and just set/update params to 0 if we disable.
    PxParticleAndDiffuseBuffer* userBuffer = InternalParticleSet::createUserBuffer(internalParticleSet->mMaxParticles, internalParticleSet->mNumParticles,
        internalParticleSet->mDiffuseParticlesEnabled, internalParticleSet->mMaxDiffuseParticleMultiplier,
        internalParticleSet->mDiffuseParticleParams, physxSetup);

    if (internalParticleSet->mEnabled && internalPS->mEnabled)
        internalPS->mPS->addParticleBuffer(userBuffer);

    // this triggers the DMA in the onBegin() callback
    internalParticleSet->mParticleBuffer = userBuffer;
    internalParticleSet->mUploadDirtyFlags = ParticleBufferFlags::eALL;
    if (mExposePrimNames)
        userBuffer->setName(path.GetText());

    // Set the phase on the non active particles
    for (int index = particleDesc.numParticles; index < particleDesc.maxParticles; ++index)
    {
        internalParticleSet->mPhases[index] = internalParticleSet->mPhase;
    }

    // Set the invMass on the non active particles
    for (int index = particleDesc.numParticles; index < particleDesc.maxParticles; ++index)
    {
        internalParticleSet->mPositions[index] = PxVec4(0.0f, 0.0f, 0.0f, internalParticleSet->mParticleInvMass);
    }

    // create primVars for Q1/Q2/Q3
    if (hasAnisotropy && usdPrim && src && src->isA(primKey, schemaTypeToken<UsdGeomPoints>(*src)))
    {
        UsdGeomPrimvarsAPI primVarsAPI = UsdGeomPrimvarsAPI(usdPrim);
        primVarsAPI.CreatePrimvar(TfToken("anisotropyQ1"), SdfValueTypeNames->Float4Array, UsdGeomTokens->vertex);
        primVarsAPI.CreatePrimvar(TfToken("anisotropyQ2"), SdfValueTypeNames->Float4Array, UsdGeomTokens->vertex);
        primVarsAPI.CreatePrimvar(TfToken("anisotropyQ3"), SdfValueTypeNames->Float4Array, UsdGeomTokens->vertex);
    }

    // store start transformations
    internalParticleSet->mPositionSaveRestoreBuf = std::vector<carb::Float3>(particleDesc.numParticles);
    internalParticleSet->mVelocitySaveRestoreBuf = std::vector<carb::Float3>(particleDesc.numParticles);

    copyBuffer(internalParticleSet->mPositionSaveRestoreBuf, internalParticleSet->mPositions, internalParticleSet->mNumParticles);
    copyBuffer(internalParticleSet->mVelocitySaveRestoreBuf, internalParticleSet->mVelocities, internalParticleSet->mNumParticles);

    attachedStage.getObjectDatabase()->addSchemaAPI(path, SchemaAPIFlag::eParticleSetAPI);

    mParticleSystems.push_back(internalPS);
    mDirty = true;

    ObjectId objectId = internalPhysxDB.addRecord(ePTParticleSet, userBuffer, internalParticleSet, attachedStage.keyFor(path));
    if (internalMaterial)
    {
        internalMaterial->addParticleId(objectId);
    }
    return objectId;
#else
    return kInvalidObjectId;
#endif
}

void PhysXUsdPhysicsInterface::changeParticlePostProcess(usdparser::AttachedStage& attachedStage, const SdfPath& path, bool removed, SchemaAPIFlag::Enum flag)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const ObjectId id = attachedStage.getObjectDatabase()->findEntry(path, eParticleSystem);
    if (kInvalidObjectId != id)
    {
        void* objectRecord = db.getInternalTypedRecord(ePTParticleSystem, id);
        if (objectRecord)
        {
            InternalPbdParticleSystem* ps = reinterpret_cast<InternalPbdParticleSystem*>(objectRecord);

            ParticlePostFlag::Enum postFlags = ParticlePostFlag::eNone;
            switch (flag)
            {
                case SchemaAPIFlag::eParticleIsosurfaceAPI:
                    postFlags = ParticlePostFlag::eIsosurface;
                    break;

                case SchemaAPIFlag::eParticleAnisotropyAPI:
                    postFlags = ParticlePostFlag::eAnisotropy;
                    break;

                case SchemaAPIFlag::eParticleSmoothingAPI:
                    postFlags = ParticlePostFlag::eSmoothing;
                    break;
            }

            if (removed)
            {
                ps->enablePost(postFlags, false);
                attachedStage.getObjectDatabase()->removeSchemaAPI(path, flag);
            }
            else
            {
                ps->enablePost(postFlags, true);
                attachedStage.getObjectDatabase()->addSchemaAPI(path, flag);
            }
        }
    }
}

void PhysXUsdPhysicsInterface::changeParticleDiffuseParticles(usdparser::AttachedStage& attachedStage, const SdfPath& path, bool removed)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    const ObjectId id = attachedStage.getObjectDatabase()->findEntry(path, eParticleSet);
    if (kInvalidObjectId != id)
    {
        void* objectRecord = db.getInternalTypedRecord(ePTParticleSet, id);
        if (objectRecord)
        {
            InternalParticleSet* particleSet = reinterpret_cast<InternalParticleSet*>(objectRecord);
            particleSet->changeDiffuseParticles(removed);
            if (removed)
            {
                attachedStage.getObjectDatabase()->removeSchemaAPI(path, SchemaAPIFlag::eDiffuseParticlesAPI);
            }
            else
            {
                attachedStage.getObjectDatabase()->addSchemaAPI(path, SchemaAPIFlag::eDiffuseParticlesAPI);
            }
        }
    }
}

} // namespace physx
} // namespace omni
