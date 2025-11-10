// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXPropertiesUpdate.h"

#include <internal/InternalParticle.h>
#include <particles/PhysXParticlePost.h>
#include <PhysXTools.h>
#include <Setup.h>
#include <OmniPhysX.h>

#include <usdLoad/Particles.h>
#include <usdLoad/Material.h>
#include <usdLoad/LoadUsd.h>

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>

#include <common/foundation/TypeCast.h>

using namespace ::physx;
using namespace carb;
using namespace pxr;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

////////////////////////////////////////////////////////////////////////////////////////////////////////
// PARTICLE SYSTEM
bool omni::physx::updateParticleSystemAttribute(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const TfToken& property, const UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTParticleSystem)
    {
        PxPBDParticleSystem* physxPS = reinterpret_cast<PxPBDParticleSystem*>(objectRecord->mPtr);
        if (physxPS)
        {
            if (property == PhysxSchemaTokens.Get()->particleSystemEnabled)
            {
                bool enabled;
                if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, enabled))
                    return true;

                InternalPbdParticleSystem* internalPS = reinterpret_cast<InternalPbdParticleSystem*>(objectRecord->mInternalPtr);
                internalPS->enableParticleSystem(enabled);
            }
            else if (property == PhysxSchemaTokens.Get()->enableCCD)
            {
                bool enabled;
                if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, enabled))
                    return true;

                physxPS->setParticleFlag(PxParticleFlag::eENABLE_SPECULATIVE_CCD, enabled);
            }
            else if (property == PhysxSchemaTokens.Get()->contactOffset)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                data = completeContactOffset(attachedStage.getStage(), data, physxPS->getParticleContactOffset());

                physxPS->setContactOffset(data);
            }
            else if (property == PhysxSchemaTokens.Get()->fluidRestOffset)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                data = completeFluidRestOffset(attachedStage.getStage(), data, physxPS->getParticleContactOffset());

                physxPS->setFluidRestOffset(data);
            }
            else if (property == PhysxSchemaTokens.Get()->maxDepenetrationVelocity)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                physxPS->setMaxDepenetrationVelocity(data);
            }
            else if (property == PhysxSchemaTokens.Get()->maxVelocity)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                physxPS->setMaxVelocity(data);
            }
            else if (property == PhysxSchemaTokens.Get()->particleContactOffset)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                data = completeParticleContactOffset(attachedStage.getStage(), data);

                physxPS->setParticleContactOffset(data);
            }            
            else if (property == PhysxSchemaTokens.Get()->restOffset)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                data = completeRestOffset(attachedStage.getStage(), data, physxPS->getParticleContactOffset());

                physxPS->setRestOffset(data);
            }
            else if (property == PhysxSchemaTokens.Get()->solidRestOffset)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                data = completeSolidRestOffset(attachedStage.getStage(), data, physxPS->getParticleContactOffset());

                physxPS->setSolidRestOffset(data);
            }
            else if (property == PhysxSchemaTokens.Get()->solverPositionIterationCount)
            {
                PxU32 pos, vel;
                physxPS->getSolverIterationCounts(pos, vel);

                int data;
                if (!getValue(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                physxPS->setSolverIterationCounts(PxU32(data), vel);
            }
            else if (property == PhysxSchemaTokens.Get()->wind)
            {
                GfVec3f data;
                if (!getValue(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                physxPS->setWind(toPhysX(data));
            }
            else if (property == PhysxSchemaTokens.Get()->maxNeighborhood)
            {
                const InternalPbdParticleSystem* internalPS = reinterpret_cast<InternalPbdParticleSystem*>(objectRecord->mInternalPtr);
                CARB_LOG_WARN("Cannot update maxNeighborhood of %s after simulation start.", internalPS->mPath.GetText());
            }
            else if (property == PhysxSchemaTokens.Get()->neighborhoodScale)
            {
                const InternalPbdParticleSystem* internalPS = reinterpret_cast<InternalPbdParticleSystem*>(objectRecord->mInternalPtr);
                CARB_LOG_WARN("Cannot update neighborhoodScale of %s after simulation start.", internalPS->mPath.GetText());
            }
        }
    }

    return true;
}

bool omni::physx::updateParticleSmoothingEnabledAttribute(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const TfToken& property, const UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTParticleSystem)
    {
        InternalPbdParticleSystem* internalPS = reinterpret_cast<InternalPbdParticleSystem*>(objectRecord->mInternalPtr);

        if (internalPS)
        {
            if (property == PhysxSchemaTokens.Get()->physxParticleSmoothingParticleSmoothingEnabled)
            {
                bool data;
                if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                internalPS->enablePost(ParticlePostFlag::eSmoothing, data);
            }
        }
    }

    return true;
}

bool omni::physx::updateParticleAnisotropyEnabledAttribute(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const TfToken& property, const UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTParticleSystem)
    {
        InternalPbdParticleSystem* internalPS = reinterpret_cast<InternalPbdParticleSystem*>(objectRecord->mInternalPtr);

        if (internalPS)
        {
            if (property == PhysxSchemaTokens.Get()->physxParticleAnisotropyParticleAnisotropyEnabled)
            {
                bool data;
                if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                internalPS->enablePost(ParticlePostFlag::eAnisotropy, data);
            }
        }
    }

    return true;
}

bool omni::physx::updateParticleIsosurfaceEnabledAttribute(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const TfToken& property, const UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTParticleSystem)
    {
        InternalPbdParticleSystem* internalPS = reinterpret_cast<InternalPbdParticleSystem*>(objectRecord->mInternalPtr);

        if (internalPS)
        {
            if (property == PhysxSchemaTokens.Get()->physxParticleIsosurfaceIsosurfaceEnabled)
            {
                bool data;
                if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                internalPS->enablePost(ParticlePostFlag::eIsosurface, data);
            }
        }
    }

    return true;
}

bool omni::physx::updateParticleIsosurfaceAttribute(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const TfToken& property, const UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTParticleSystem)
    {
        InternalPbdParticleSystem* internalPS = reinterpret_cast<InternalPbdParticleSystem*>(objectRecord->mInternalPtr);

        uint32_t postFlags = particles::getPostprocessStages(internalPS->mPath);

        if (internalPS && (postFlags & ParticlePostFlag::eIsosurface))
        {
            if (property == PhysxSchemaTokens.Get()->physxParticleIsosurfaceSurfaceDistance)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

               /* ExtGpu::PxIsosurfaceParams p = internalPS->mIsosurface->mIsosurfaceBuffer->getParams();
                p.particleCenterToIsosurfaceDistance = data;
                internalPS->mIsosurface->mIsosurfaceBuffer->setParams(p);*/
            }
            else if (property == PhysxSchemaTokens.Get()->physxParticleIsosurfaceGridFilteringPasses)
            {
                std::string data;
                if (!getValue<std::string>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                /*ExtGpu::PxIsosurfaceParams p = internalPS->mIsosurface->mIsosurfaceBuffer->getParams();
                std::vector<ParticleSystemDesc::GridFilteringPass> passes;
                ParseGridFilteringPasses(data, passes); 
                omni::physx::particle::setIsosurfaceGridFilteringPasses(p, passes);
                internalPS->mIsosurface->mIsosurfaceBuffer->setParams(p);*/
            }
            else if (property == PhysxSchemaTokens.Get()->physxParticleIsosurfaceGridSmoothingRadius)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

               /* ExtGpu::PxIsosurfaceParams p = internalPS->mIsosurface->mIsosurfaceBuffer->getParams();
                p.gridSmoothingRadiusRelativeToCellSize = data;
                internalPS->mIsosurface->mIsosurfaceBuffer->setParams(p);*/
            }
            else if (property == PhysxSchemaTokens.Get()->physxParticleIsosurfaceNumMeshSmoothingPasses)
            {
                int data;
                if (!getValue<int>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

               /* PxIsosurfaceParams p = internalPS->mIsosurface->mIsosurfaceBuffer->getParams();
                p.numMeshSmoothingPasses = data;
                internalPS->mIsosurface->mIsosurfaceBuffer->setParams(p);*/
            }
            else if (property == PhysxSchemaTokens.Get()->physxParticleIsosurfaceNumMeshNormalSmoothingPasses)
            {
                int data;
                if (!getValue<int>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                /* PxIsosurfaceParams p = internalPS->mIsosurface->mIsosurfaceBuffer->getParams();
                p.numMeshNormalSmoothingPasses = data;
                internalPS->mIsosurface->mIsosurfaceBuffer->setParams(p);*/
            }
        }
    }

    return true;
}

bool omni::physx::updateDiffuseParticlesEnabledAttribute(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const TfToken& property, const UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTParticleSet)
    {
        InternalParticleSet* particleSet = reinterpret_cast<InternalParticleSet*>(objectRecord->mInternalPtr);

        if (particleSet)
        {
            if (property == PhysxSchemaTokens.Get()->physxDiffuseParticlesDiffuseParticlesEnabled)
            {
                bool enabled;
                if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, enabled))
                    return true;

                particleSet->enableDiffuseParticles(enabled);
            }
        }
    }
    return true;
}

bool omni::physx::updateDiffuseParticlesAttribute(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const TfToken& property, const UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTParticleSet)
    {
        InternalParticleSet* particleSet = reinterpret_cast<InternalParticleSet*>(objectRecord->mInternalPtr);
        if (particleSet)
        {
            if (property == PhysxSchemaTokens.Get()->physxDiffuseParticlesMaxDiffuseParticleMultiplier)
            {
                particleSet->changeDiffuseParticles(false);
                return true;
            }
            else if (property == PhysxSchemaTokens.Get()->physxDiffuseParticlesThreshold)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                particleSet->mDiffuseParticleParams.threshold = data;
            }
            else if (property == PhysxSchemaTokens.Get()->physxDiffuseParticlesLifetime)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                particleSet->mDiffuseParticleParams.lifetime = data;
            }
            else if (property == PhysxSchemaTokens.Get()->physxDiffuseParticlesAirDrag)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                particleSet->mDiffuseParticleParams.airDrag = data;
            }
            else if (property == PhysxSchemaTokens.Get()->physxDiffuseParticlesBubbleDrag)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                particleSet->mDiffuseParticleParams.bubbleDrag = data;
            }
            else if (property == PhysxSchemaTokens.Get()->physxDiffuseParticlesBuoyancy)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                particleSet->mDiffuseParticleParams.buoyancy = data;
            }
            else if (property == PhysxSchemaTokens.Get()->physxDiffuseParticlesKineticEnergyWeight)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                particleSet->mDiffuseParticleParams.kineticEnergyWeight = data;
            }
            else if (property == PhysxSchemaTokens.Get()->physxDiffuseParticlesPressureWeight)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                particleSet->mDiffuseParticleParams.pressureWeight = data;
            }
            else if (property == PhysxSchemaTokens.Get()->physxDiffuseParticlesDivergenceWeight)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                particleSet->mDiffuseParticleParams.divergenceWeight = data;
            }
            else if (property == PhysxSchemaTokens.Get()->physxDiffuseParticlesCollisionDecay)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                particleSet->mDiffuseParticleParams.collisionDecay = data;
            }
            
            particleSet->setDiffuseParticleParams();
        }
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// PARTICLE SET
bool omni::physx::updateParticleSetEnabled(AttachedStage& attachedStage, ObjectId objectId, const TfToken& property, const UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTParticleSet;
    InternalParticleSet* internalParticleSet = reinterpret_cast<InternalParticleSet*>(db.getInternalTypedRecord(internalType, objectId));

    if (internalParticleSet)
    {
        bool enabled;
        if (!getValue<bool>(attachedStage, internalParticleSet->mPrim.GetPrimPath(), property, timeCode, enabled))
            return true;

        internalParticleSet->enableParticleSet(enabled);
    }

    return true;
}

bool omni::physx::updateParticleSetSelfCollision(AttachedStage& attachedStage, ObjectId objectId, const TfToken& property, const UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTParticleSet;
    InternalParticleSet* internalParticleSet = reinterpret_cast<InternalParticleSet*>(db.getInternalTypedRecord(internalType, objectId));

    if (internalParticleSet)
    {
        bool selfCollision;
        if (!getValue<bool>(attachedStage, internalParticleSet->mPrim.GetPrimPath(), property, timeCode, selfCollision))
            return true;

        PxU32* phases = internalParticleSet->mPhases;
        if (selfCollision)
        {
            for (uint32_t i = 0; i < internalParticleSet->mNumParticles; i++)
            {
                phases[i] |= PxParticlePhaseFlag::eParticlePhaseSelfCollide;
            }
            internalParticleSet->mPhase |= PxParticlePhaseFlag::eParticlePhaseSelfCollide;
        }
        else
        {
            for (uint32_t i = 0; i < internalParticleSet->mNumParticles; i++)
            {
                phases[i] &= ~PxParticlePhaseFlag::eParticlePhaseSelfCollide;
            }
            internalParticleSet->mPhase &= ~PxParticlePhaseFlag::eParticlePhaseSelfCollide;
        }
        internalParticleSet->mUploadDirtyFlags |= ParticleBufferFlags::ePHASES;
    }

    return true;
}

bool omni::physx::updateParticleSetFluid(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const TfToken& property, const UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTParticleSet)
    {
        InternalParticleSet* internalParticleSet = (InternalParticleSet*)objectRecord->mInternalPtr;

        bool fluid;
        if (!getValue<bool>(attachedStage, objectRecord->mPath, property, timeCode, fluid))
            return true;

        PxU32* phases = internalParticleSet->mPhases;
        if (fluid)
        {
            for (uint32_t i = 0; i < internalParticleSet->mNumParticles; i++)
            {
                phases[i] |= PxParticlePhaseFlag::eParticlePhaseFluid;
            }
            internalParticleSet->mPhase |= PxParticlePhaseFlag::eParticlePhaseFluid;
            internalParticleSet->mFluid = true;
        }
        else
        {
            for (uint32_t i = 0; i < internalParticleSet->mNumParticles; i++)
            {
                phases[i] &= ~PxParticlePhaseFlag::eParticlePhaseFluid;
            }
            internalParticleSet->mPhase &= ~PxParticlePhaseFlag::eParticlePhaseFluid;
            internalParticleSet->mFluid = false;
        }

        internalParticleSet->mUploadDirtyFlags |= ParticleBufferFlags::ePHASES;
    }

    return true;
}

bool omni::physx::updateParticleSetParticleGroup(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const TfToken& property, const UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTParticleSet;
    InternalParticleSet* internalParticleSet = reinterpret_cast<InternalParticleSet*>(db.getInternalTypedRecord(internalType, objectId));

    if (internalParticleSet)
    {
        int particleGroup;
        if (!getValue<int>(attachedStage, internalParticleSet->mPrim.GetPrimPath(), property, timeCode, particleGroup))
            return true;

        PxU32* phases = internalParticleSet->mPhases;
        for (uint32_t i = 0; i < internalParticleSet->mNumParticles; i++)
        {
            PxU32 phase = phases[i];
            // clear the current group
            phase &= ~PxParticlePhaseFlag::eParticlePhaseGroupMask;
            // Set the new group
            phase |= (particleGroup & PxParticlePhaseFlag::eParticlePhaseGroupMask);
            phases[i] = phase;
        }
        internalParticleSet->mPhase &= ~PxParticlePhaseFlag::eParticlePhaseGroupMask;
        internalParticleSet->mPhase |= (particleGroup & PxParticlePhaseFlag::eParticlePhaseGroupMask);
        internalParticleSet->mUploadDirtyFlags |= ParticleBufferFlags::ePHASES;
    }

    return true;
}

static void updateParticlePositions(InternalParticleSet* internalParticleSet, const VtArray<GfVec3f>& positions)
{
    const UsdPrim& usdPrim = internalParticleSet->mPrim;
    UsdGeomXform xform(usdPrim);
    GfMatrix4d localToWorld = xform.ComputeLocalToWorldTransform(UsdTimeCode::Default());

    uint32_t newNumParticles = (uint32_t)positions.size();
    if (newNumParticles != internalParticleSet->mNumParticles)
    {
        internalParticleSet->resize(newNumParticles);
    }

    // better make sure the resizing worked
    if (internalParticleSet->mNumParticles != newNumParticles)
    {
        CARB_LOG_ERROR("Changing number of particles in %s failed - skipping update.", internalParticleSet->mPrim.GetPath().GetText());
        return;
    }
    PxVec4* positionsInvMass = internalParticleSet->mPositions;
    for (uint32_t i = 0; i < internalParticleSet->mNumParticles; i++)
    {
        GfVec3f localPos = positions[i];
        GfVec3f pos = localToWorld.Transform(localPos);

        positionsInvMass[i] = PxVec4(pos[0], pos[1], pos[2], internalParticleSet->mParticleInvMass);
    }

    // we don't change velocities/widths here because we assume that the user updates all data before sim runs.
    internalParticleSet->mUploadDirtyFlags |= ParticleBufferFlags::ePOSITIONS;
}


bool omni::physx::updateParticlePositions(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const TfToken& property, const UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTParticleSet;
    InternalParticleSet* internalParticleSet = reinterpret_cast<InternalParticleSet*>(db.getInternalTypedRecord(internalType, objectId));

    if (internalParticleSet)
    {
        const UsdPrim& usdPrim = internalParticleSet->mPrim;
        UsdGeomPointBased pointBased = UsdGeomPointBased(usdPrim);
        UsdGeomPointInstancer pointInstancer = UsdGeomPointInstancer(usdPrim);
        VtArray<GfVec3f> positions;
        if (pointBased)
        {
            pointBased.GetPointsAttr().Get(&positions);
        }
        else if (pointInstancer)
        {
            pointInstancer.GetPositionsAttr().Get(&positions);
        }
        updateParticlePositions(internalParticleSet, positions);
    }

    return true;
}

bool omni::physx::updateParticleSimPositions(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const TfToken& property, const UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTParticleSet;
    InternalParticleSet* internalParticleSet = reinterpret_cast<InternalParticleSet*>(db.getInternalTypedRecord(internalType, objectId));

    if (internalParticleSet)
    {
        const UsdPrim& usdPrim = internalParticleSet->mPrim;
        PhysxSchemaPhysxParticleSetAPI particleSet(usdPrim);
        VtArray<GfVec3f> positions;
        particleSet.GetSimulationPointsAttr().Get(&positions);
        updateParticlePositions(internalParticleSet, positions);
    }

    return true;
}

bool omni::physx::updateParticleVelocities(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const TfToken& property, const UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTParticleSet;
    InternalParticleSet* internalParticleSet = reinterpret_cast<InternalParticleSet*>(db.getInternalTypedRecord(internalType, objectId));

    if (internalParticleSet)
    {
        const UsdPrim& usdPrim = internalParticleSet->mPrim;

        UsdGeomPointBased pointBased = UsdGeomPointBased(usdPrim);
        UsdGeomPointInstancer pointInstancer = UsdGeomPointInstancer(usdPrim);
        UsdAttribute velocitiesAttr;
        if (pointBased)
        {
            velocitiesAttr = pointBased.GetVelocitiesAttr();
        }
        else if (pointInstancer)
        {
            velocitiesAttr = pointInstancer.GetVelocitiesAttr();
        }

        VtArray<GfVec3f> velocities;
        velocitiesAttr.Get(&velocities);
        uint32_t newNumParticles = (uint32_t)velocities.size();

        if (newNumParticles != internalParticleSet->mNumParticles)
        {
            internalParticleSet->resize(newNumParticles);
        }

        // better make sure the resizing worked
        if (internalParticleSet->mNumParticles != newNumParticles)
        {
            CARB_LOG_ERROR("Changing number of particles in %s failed - skipping update.", internalParticleSet->mPrim.GetPath().GetText());
            return true;
        }
        PxVec4* velocitiesPhysX = internalParticleSet->mVelocities;
        for (uint32_t i = 0; i < internalParticleSet->mNumParticles; ++i)
        {
            velocitiesPhysX[i] = PxVec4(toPhysX(velocities[i]), 0.0f);
        }

        // we don't change positions/widths here because we assume that the user updates all data before sim runs.
        internalParticleSet->mUploadDirtyFlags |= ParticleBufferFlags::eVELOCITIES;
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// material
bool omni::physx::updatePBDMaterialAttribute(AttachedStage& attachedStage, ObjectId objectId, const TfToken& property, const UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTRemoved;
    const InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
    if (!objectRecord)
        return true;

    if (internalType == ePTPBDMaterial)
    {
        PxPBDMaterial* material = (PxPBDMaterial*)objectRecord->mPtr;
        if (material)
        {
            if (property == PhysxSchemaTokens.Get()->physxPBDMaterialCohesion)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                material->setCohesion(data);
            }
            else if (property == PhysxSchemaTokens.Get()->physxPBDMaterialAdhesion)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                material->setAdhesion(data);
            }
            else if (property == PhysxSchemaTokens.Get()->physxPBDMaterialParticleAdhesionScale)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                material->setParticleAdhesionScale(data);
            }
            else if (property == PhysxSchemaTokens.Get()->physxPBDMaterialAdhesionOffsetScale)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                material->setAdhesionRadiusScale(data);
            }
            else if (property == PhysxSchemaTokens.Get()->physxPBDMaterialDrag)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                material->setDrag(data);
            }
            else if (property == PhysxSchemaTokens.Get()->physxPBDMaterialLift)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                material->setLift(data);
            }
            else if (property == PhysxSchemaTokens.Get()->physxPBDMaterialFriction)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                material->setFriction(data);
            }
            else if (property == PhysxSchemaTokens.Get()->physxPBDMaterialParticleFrictionScale)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                material->setParticleFrictionScale(data);
            }
            else if (property == PhysxSchemaTokens.Get()->physxPBDMaterialDamping)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                material->setDamping(data);
            }
            else if (property == PhysxSchemaTokens.Get()->physxPBDMaterialSurfaceTension)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                material->setSurfaceTension(data);
            }
            else if (property == PhysxSchemaTokens.Get()->physxPBDMaterialViscosity)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                material->setViscosity(data);
            }
            else if (property == PhysxSchemaTokens.Get()->physxPBDMaterialVorticityConfinement)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                material->setVorticityConfinement(data);
            }
            else if (property == PhysxSchemaTokens.Get()->physxPBDMaterialGravityScale)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                material->setGravityScale(data);
            }
            else if (property == PhysxSchemaTokens.Get()->physxPBDMaterialCflCoefficient)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                material->setCFLCoefficient(data);
            }
            else if (property == PhysxSchemaTokens.Get()->physxPBDMaterialDensity)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mPath, property, timeCode, data))
                    return true;

                InternalPBDParticleMaterial* intMat = (InternalPBDParticleMaterial*)objectRecord->mInternalPtr;
                intMat->mDensity = data;

                for (size_t i = 0; i < intMat->mParticleIds.size(); i++)
                {
                    OmniPhysX::getInstance().getInternalPhysXDatabase().addDirtyMassParticle(intMat->mParticleIds[i]);
                }
            }
        }
    }
    return true;
}

bool omni::physx::updateParticleDensity(AttachedStage& attachedStage, ObjectId objectId, const TfToken& property, const UsdTimeCode& timeCode)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    db.addDirtyMassParticle(size_t(objectId));
    return true;
}
