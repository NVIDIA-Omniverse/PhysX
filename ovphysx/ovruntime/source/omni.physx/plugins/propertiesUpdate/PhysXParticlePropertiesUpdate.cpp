// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-BACKEND-001
 * @covers AC-9
 */

#include "PhysXPropertiesUpdate.h"

#include <omni/physics/parse/KnownTokens.h>

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

using namespace ::physx;
using namespace carb;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

////////////////////////////////////////////////////////////////////////////////////////////////////////
// PARTICLE SYSTEM
bool omni::physx::updateParticleSystemAttribute(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
            const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
            omni::physics::parse::KnownTokens tok;
            if (source)
                tok.intern(*source);

            if (property == tok.particleSystemEnabled)
            {
                bool enabled;
                if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, enabled))
                    return true;

                InternalPbdParticleSystem* internalPS = reinterpret_cast<InternalPbdParticleSystem*>(objectRecord->mInternalPtr);
                internalPS->enableParticleSystem(enabled);
            }
            else if (property == tok.enableCCD)
            {
                bool enabled;
                if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, enabled))
                    return true;

                physxPS->setParticleFlag(PxParticleFlag::eENABLE_SPECULATIVE_CCD, enabled);
            }
            else if (property == tok.contactOffset)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                data = completeContactOffset(attachedStage.getSourceUnits().metersPerUnit, data, physxPS->getParticleContactOffset());

                physxPS->setContactOffset(data);
            }
            else if (property == tok.fluidRestOffset)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                data = completeFluidRestOffset(attachedStage.getSourceUnits().metersPerUnit, data, physxPS->getParticleContactOffset());

                physxPS->setFluidRestOffset(data);
            }
            else if (property == tok.maxDepenetrationVelocity)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                physxPS->setMaxDepenetrationVelocity(data);
            }
            else if (property == tok.maxVelocity)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                physxPS->setMaxLinearVelocity(data);
            }
            else if (property == tok.particleContactOffset)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                data = completeParticleContactOffset(attachedStage.getSourceUnits().metersPerUnit, data);

                physxPS->setParticleContactOffset(data);
            }            
            else if (property == tok.restOffset)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                data = completeRestOffset(attachedStage.getSourceUnits().metersPerUnit, data, physxPS->getParticleContactOffset());

                physxPS->setRestOffset(data);
            }
            else if (property == tok.solidRestOffset)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                data = completeSolidRestOffset(attachedStage.getSourceUnits().metersPerUnit, data, physxPS->getParticleContactOffset());

                physxPS->setSolidRestOffset(data);
            }
            else if (property == tok.solverPositionIterationCount)
            {
                PxU32 pos, vel;
                physxPS->getSolverIterationCounts(pos, vel);

                int data;
                if (!getValue(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                physxPS->setSolverIterationCounts(PxU32(data), vel);
            }
            else if (source && property == source->internToken("wind"))
            {
                carb::Float3 data;
                if (!getValue(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                physxPS->setWind(toPhysX(data));
            }
            else if (property == tok.maxNeighborhood)
            {
                const InternalPbdParticleSystem* internalPS = reinterpret_cast<InternalPbdParticleSystem*>(objectRecord->mInternalPtr);
                CARB_LOG_WARN("Cannot update maxNeighborhood of %s after simulation start.", attachedStage.textFor(internalPS->mKey));
            }
            else if (property == tok.neighborhoodScale)
            {
                const InternalPbdParticleSystem* internalPS = reinterpret_cast<InternalPbdParticleSystem*>(objectRecord->mInternalPtr);
                CARB_LOG_WARN("Cannot update neighborhoodScale of %s after simulation start.", attachedStage.textFor(internalPS->mKey));
            }
        }
    }

    return true;
}

bool omni::physx::updateParticleSmoothingEnabledAttribute(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
            const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
            omni::physics::parse::KnownTokens tok;
            if (source)
                tok.intern(*source);

            if (property == tok.physxParticleSmoothingParticleSmoothingEnabled)
            {
                bool data;
                if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                internalPS->enablePost(ParticlePostFlag::eSmoothing, data);
            }
        }
    }

    return true;
}

bool omni::physx::updateParticleAnisotropyEnabledAttribute(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
            const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
            omni::physics::parse::KnownTokens tok;
            if (source)
                tok.intern(*source);

            if (property == tok.physxParticleAnisotropyParticleAnisotropyEnabled)
            {
                bool data;
                if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                internalPS->enablePost(ParticlePostFlag::eAnisotropy, data);
            }
        }
    }

    return true;
}

bool omni::physx::updateParticleIsosurfaceEnabledAttribute(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
            const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
            omni::physics::parse::KnownTokens tok;
            if (source)
                tok.intern(*source);

            if (property == tok.physxParticleIsosurfaceIsosurfaceEnabled)
            {
                bool data;
                if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                internalPS->enablePost(ParticlePostFlag::eIsosurface, data);
            }
        }
    }

    return true;
}

bool omni::physx::updateParticleIsosurfaceAttribute(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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

        // particles::getPostprocessStages is ObjectKey-keyed and unconditional (see
        // PhysXParticlePost.h's top-of-file comment): the postprocess registry it queries is
        // always empty in OvruntimePhysX, so it returns eNone. Isosurface is therefore never
        // enabled, so the block below (already almost entirely commented-out
        // real work, see the getValue<>() calls it guards) stays unreachable there.
        // Read the key only once internalPS is known good -- the check below used to
        // come after this dereference.
        const uint32_t postFlags =
            internalPS ? particles::getPostprocessStages(internalPS->mKey) : uint32_t(ParticlePostFlag::eNone);

        if (internalPS && (postFlags & ParticlePostFlag::eIsosurface))
        {
            const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
            omni::physics::parse::KnownTokens tok;
            if (source)
                tok.intern(*source);

            if (property == tok.physxParticleIsosurfaceSurfaceDistance)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

               /* ExtGpu::PxIsosurfaceParams p = internalPS->mIsosurface->mIsosurfaceBuffer->getParams();
                p.particleCenterToIsosurfaceDistance = data;
                internalPS->mIsosurface->mIsosurfaceBuffer->setParams(p);*/
            }
            else if (property == tok.physxParticleIsosurfaceGridFilteringPasses)
            {
                std::string data;
                if (!getValue<std::string>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                /*ExtGpu::PxIsosurfaceParams p = internalPS->mIsosurface->mIsosurfaceBuffer->getParams();
                std::vector<ParticleSystemDesc::GridFilteringPass> passes;
                ParseGridFilteringPasses(data, passes); 
                omni::physx::particle::setIsosurfaceGridFilteringPasses(p, passes);
                internalPS->mIsosurface->mIsosurfaceBuffer->setParams(p);*/
            }
            else if (property == tok.physxParticleIsosurfaceGridSmoothingRadius)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

               /* ExtGpu::PxIsosurfaceParams p = internalPS->mIsosurface->mIsosurfaceBuffer->getParams();
                p.gridSmoothingRadiusRelativeToCellSize = data;
                internalPS->mIsosurface->mIsosurfaceBuffer->setParams(p);*/
            }
            else if (property == tok.physxParticleIsosurfaceNumMeshSmoothingPasses)
            {
                int data;
                if (!getValue<int>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

               /* PxIsosurfaceParams p = internalPS->mIsosurface->mIsosurfaceBuffer->getParams();
                p.numMeshSmoothingPasses = data;
                internalPS->mIsosurface->mIsosurfaceBuffer->setParams(p);*/
            }
            else if (property == tok.physxParticleIsosurfaceNumMeshNormalSmoothingPasses)
            {
                int data;
                if (!getValue<int>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                /* PxIsosurfaceParams p = internalPS->mIsosurface->mIsosurfaceBuffer->getParams();
                p.numMeshNormalSmoothingPasses = data;
                internalPS->mIsosurface->mIsosurfaceBuffer->setParams(p);*/
            }
        }
    }

    return true;
}

bool omni::physx::updateDiffuseParticlesEnabledAttribute(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
            const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
            omni::physics::parse::KnownTokens tok;
            if (source)
                tok.intern(*source);

            if (property == tok.physxDiffuseParticlesDiffuseParticlesEnabled)
            {
                bool enabled;
                if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, enabled))
                    return true;

                particleSet->enableDiffuseParticles(enabled);
            }
        }
    }
    return true;
}

bool omni::physx::updateDiffuseParticlesAttribute(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
            const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
            omni::physics::parse::KnownTokens tok;
            if (source)
                tok.intern(*source);

            if (property == tok.physxDiffuseParticlesMaxDiffuseParticleMultiplier)
            {
                particleSet->changeDiffuseParticles(false);
                return true;
            }
            else if (property == tok.physxDiffuseParticlesThreshold)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                particleSet->mDiffuseParticleParams.threshold = data;
            }
            else if (property == tok.physxDiffuseParticlesLifetime)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                particleSet->mDiffuseParticleParams.lifetime = data;
            }
            else if (property == tok.physxDiffuseParticlesAirDrag)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                particleSet->mDiffuseParticleParams.airDrag = data;
            }
            else if (property == tok.physxDiffuseParticlesBubbleDrag)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                particleSet->mDiffuseParticleParams.bubbleDrag = data;
            }
            else if (property == tok.physxDiffuseParticlesBuoyancy)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                particleSet->mDiffuseParticleParams.buoyancy = data;
            }
            else if (property == tok.physxDiffuseParticlesKineticEnergyWeight)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                particleSet->mDiffuseParticleParams.kineticEnergyWeight = data;
            }
            else if (property == tok.physxDiffuseParticlesPressureWeight)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                particleSet->mDiffuseParticleParams.pressureWeight = data;
            }
            else if (property == tok.physxDiffuseParticlesDivergenceWeight)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                particleSet->mDiffuseParticleParams.divergenceWeight = data;
            }
            else if (property == tok.physxDiffuseParticlesCollisionDecay)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
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
bool omni::physx::updateParticleSetEnabled(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTParticleSet;
    InternalParticleSet* internalParticleSet = reinterpret_cast<InternalParticleSet*>(db.getInternalTypedRecord(internalType, objectId));

    if (internalParticleSet)
    {
        bool enabled;
        if (!getValue<bool>(attachedStage, internalParticleSet->mKey, property, timeCode, enabled))
            return true;

        internalParticleSet->enableParticleSet(enabled);
    }

    return true;
}

bool omni::physx::updateParticleSetSelfCollision(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTParticleSet;
    InternalParticleSet* internalParticleSet = reinterpret_cast<InternalParticleSet*>(db.getInternalTypedRecord(internalType, objectId));

    if (internalParticleSet)
    {
        bool selfCollision;
        if (!getValue<bool>(attachedStage, internalParticleSet->mKey, property, timeCode, selfCollision))
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

bool omni::physx::updateParticleSetFluid(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
        if (!getValue<bool>(attachedStage, objectRecord->mKey, property, timeCode, fluid))
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

bool omni::physx::updateParticleSetParticleGroup(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTParticleSet;
    InternalParticleSet* internalParticleSet = reinterpret_cast<InternalParticleSet*>(db.getInternalTypedRecord(internalType, objectId));

    if (internalParticleSet)
    {
        int particleGroup;
        if (!getValue<int>(attachedStage, internalParticleSet->mKey, property, timeCode, particleGroup))
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

// Takes the stage from the caller: `positions` were just read off `attachedStage`,
// so the local-to-world transform must come from the same stage. Re-fetching the
// process-global active stage here was also an unguarded null dereference.
static void updateParticlePositions(AttachedStage& attachedStage, InternalParticleSet* internalParticleSet, const std::vector<carb::Float3>& positions)
{
    const PxMat44d localToWorld = getWorldTransform(attachedStage, internalParticleSet->mKey, omni::physics::parse::ReadTime::defaultTime());

    uint32_t newNumParticles = (uint32_t)positions.size();
    if (newNumParticles != internalParticleSet->mNumParticles)
    {
        internalParticleSet->resize(newNumParticles);
    }

    // better make sure the resizing worked
    if (internalParticleSet->mNumParticles != newNumParticles)
    {
        CARB_LOG_ERROR("Changing number of particles in %s failed - skipping update.", attachedStage.textFor(internalParticleSet->mKey));
        return;
    }
    PxVec4* positionsInvMass = internalParticleSet->mPositions;
    for (uint32_t i = 0; i < internalParticleSet->mNumParticles; i++)
    {
        const PxVec3d pos = localToWorld.transform(toPhysXd(positions[i]));

        positionsInvMass[i] =
            PxVec4(float(pos.x), float(pos.y), float(pos.z), internalParticleSet->mParticleInvMass);
    }

    // we don't change velocities/widths here because we assume that the user updates all data before sim runs.
    internalParticleSet->mUploadDirtyFlags |= ParticleBufferFlags::ePOSITIONS;
}


bool omni::physx::updateParticlePositions(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTParticleSet;
    InternalParticleSet* internalParticleSet = reinterpret_cast<InternalParticleSet*>(db.getInternalTypedRecord(internalType, objectId));

    if (internalParticleSet)
    {
        const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
        omni::physics::parse::KnownTokens tok;
        if (src)
            tok.intern(*src);
        std::vector<carb::Float3> positions;
        if (src && src->isA(internalParticleSet->mKey, tok.pointBasedType))
        {
            getArrayValue(attachedStage, internalParticleSet->mKey, tok.points, omni::physics::parse::ReadTime::defaultTime(), positions);
        }
        else if (src && src->isA(internalParticleSet->mKey, tok.pointInstancerType))
        {
            getArrayValue(attachedStage, internalParticleSet->mKey, tok.positions, omni::physics::parse::ReadTime::defaultTime(), positions);
        }
        else
        {
            // Neither a point-based nor a point-instancer particle set: bail out
            // rather than passing empty positions into updateParticlePositions,
            // which would resize the set to zero and silently clear all particles.
            CARB_LOG_ERROR("Cannot update positions of particle set %s - it is neither a UsdGeomPointBased nor a UsdGeomPointInstancer; skipping update.",
                           attachedStage.textFor(internalParticleSet->mKey));
            return true;
        }
        updateParticlePositions(attachedStage, internalParticleSet, positions);
    }

    return true;
}

bool omni::physx::updateParticleSimPositions(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTParticleSet;
    InternalParticleSet* internalParticleSet = reinterpret_cast<InternalParticleSet*>(db.getInternalTypedRecord(internalType, objectId));

    if (internalParticleSet)
    {
        const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
        omni::physics::parse::KnownTokens tok;
        if (source)
            tok.intern(*source);

        std::vector<carb::Float3> positions;
        getArrayValue(attachedStage, internalParticleSet->mKey, tok.physxParticleSimulationPoints, omni::physics::parse::ReadTime::defaultTime(), positions);
        updateParticlePositions(attachedStage, internalParticleSet, positions);
    }

    return true;
}

bool omni::physx::updateParticleVelocities(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTParticleSet;
    InternalParticleSet* internalParticleSet = reinterpret_cast<InternalParticleSet*>(db.getInternalTypedRecord(internalType, objectId));

    if (internalParticleSet)
    {
        const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
        omni::physics::parse::KnownTokens tok;
        if (src)
            tok.intern(*src);

        // Both UsdGeomPointBased and UsdGeomPointInstancer expose a "velocities"
        // attribute under the same name; gate on the prim type so neither case
        // simply skips the read.
        std::vector<carb::Float3> velocities;
        if (src && (src->isA(internalParticleSet->mKey, tok.pointBasedType) ||
                    src->isA(internalParticleSet->mKey, tok.pointInstancerType)))
        {
            getArrayValue(attachedStage, internalParticleSet->mKey, tok.velocities, omni::physics::parse::ReadTime::defaultTime(), velocities);
        }
        uint32_t newNumParticles = (uint32_t)velocities.size();

        if (newNumParticles != internalParticleSet->mNumParticles)
        {
            internalParticleSet->resize(newNumParticles);
        }

        // better make sure the resizing worked
        if (internalParticleSet->mNumParticles != newNumParticles)
        {
            CARB_LOG_ERROR("Changing number of particles in %s failed - skipping update.", attachedStage.textFor(internalParticleSet->mKey));
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
bool omni::physx::updatePBDMaterialAttribute(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
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
            const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
            omni::physics::parse::KnownTokens tok;
            if (source)
                tok.intern(*source);

            if (property == tok.physxPBDMaterialCohesion)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                material->setCohesion(data);
            }
            else if (property == tok.physxPBDMaterialAdhesion)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                material->setAdhesion(data);
            }
            else if (property == tok.physxPBDMaterialParticleAdhesionScale)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                material->setParticleAdhesionScale(data);
            }
            else if (property == tok.physxPBDMaterialAdhesionOffsetScale)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                material->setAdhesionRadiusScale(data);
            }
            else if (property == tok.physxPBDMaterialFriction)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                material->setFriction(data);
            }
            else if (property == tok.physxPBDMaterialParticleFrictionScale)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                material->setParticleFrictionScale(data);
            }
            else if (property == tok.physxPBDMaterialDamping)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                material->setDamping(data);
            }
            else if (property == tok.physxPBDMaterialSurfaceTension)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                material->setSurfaceTension(data);
            }
            else if (property == tok.physxPBDMaterialViscosity)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                material->setViscosity(data);
            }
            else if (property == tok.physxPBDMaterialVorticityConfinement)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                material->setVorticityConfinement(data);
            }
            else if (property == tok.physxPBDMaterialGravityScale)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                material->setGravityScale(data);
            }
            else if (property == tok.physxPBDMaterialCflCoefficient)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
                    return true;

                material->setCFLCoefficient(data);
            }
            else if (property == tok.physxPBDMaterialDensity)
            {
                float data;
                if (!getValue<float>(attachedStage, objectRecord->mKey, property, timeCode, data))
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

bool omni::physx::updateParticleDensity(AttachedStage& attachedStage, ObjectId objectId, omni::physics::parse::TokenId property, omni::physics::parse::ReadTime timeCode)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    db.addDirtyMassParticle(size_t(objectId));
    return true;
}
