// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXPropertiesUpdate.h"

#include <internal/InternalParticle.h>
#include <PhysXTools.h>
#include <Setup.h>
#include <OmniPhysX.h>

#include <carb/logging/Log.h>

#include <PxPhysicsAPI.h>


using namespace ::physx;
using namespace carb;
using namespace pxr;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

////////////////////////////////////////////////////////////////////////////////////////////////////////
// PARTICLE CLOTH
bool omni::physx::updateParticleClothEnabledDeprecated(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTParticleClothDeprecated;
    InternalParticleClothDeprecated* internalCloth = reinterpret_cast<InternalParticleClothDeprecated*>(db.getInternalTypedRecord(internalType, objectId));

    if (internalCloth)
    {
        bool enabled;
        if (!getValue<bool>(attachedStage, internalCloth->mPrim.GetPrimPath(), property, timeCode, enabled))
            return true;

        internalCloth->enableCloth(enabled);
    }

    return true;
}


bool omni::physx::updateParticleClothSelfCollisionDeprecated(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTParticleClothDeprecated;
    InternalParticleClothDeprecated* internalCloth = reinterpret_cast<InternalParticleClothDeprecated*>(db.getInternalTypedRecord(internalType, objectId));

    if (internalCloth)
    {
        bool selfCollision;
        if (!getValue<bool>(attachedStage, internalCloth->mPrim.GetPrimPath(), property, timeCode, selfCollision))
            return true;

        PxU32* phases = internalCloth->mPhases;
        if (selfCollision)
        {
            for (uint32_t i = 0; i < internalCloth->mNumParticles; i++)
            {
                phases[i] |= PxParticlePhaseFlag::eParticlePhaseSelfCollide;
            }
        }
        else
        {
            for (uint32_t i = 0; i < internalCloth->mNumParticles; i++)
            {
                phases[i] &= ~PxParticlePhaseFlag::eParticlePhaseSelfCollide;
            }
        }
        
        internalCloth->mUploadDirtyFlags |= ParticleBufferFlags::ePHASES;
    }

    return true;
}

bool omni::physx::updateParticleClothSelfCollisionFilterDeprecated(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTParticleClothDeprecated;
    InternalParticleClothDeprecated* internalCloth = reinterpret_cast<InternalParticleClothDeprecated*>(db.getInternalTypedRecord(internalType, objectId));

    if (internalCloth)
    {
        bool selfCollisionFilter;
        if (!getValue<bool>(attachedStage, internalCloth->mPrim.GetPrimPath(), property, timeCode, selfCollisionFilter))
            return true;

        PxU32* phases = internalCloth->mPhases;
        if (selfCollisionFilter)
        {
            for (uint32_t i = 0; i < internalCloth->mNumParticles; i++)
            {
                phases[i] |= PxParticlePhaseFlag::eParticlePhaseSelfCollideFilter;
            }
        }
        else
        {
            for (uint32_t i = 0; i < internalCloth->mNumParticles; i++)
            {
                phases[i] &= ~PxParticlePhaseFlag::eParticlePhaseSelfCollideFilter;
            }
        }
        
        internalCloth->mUploadDirtyFlags |= ParticleBufferFlags::ePHASES;
    }

    return true;
}

bool omni::physx::updateParticleClothParticleGroupDeprecated(AttachedStage& attachedStage, omni::physx::usdparser::ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTParticleClothDeprecated;
    InternalParticleClothDeprecated* internalCloth = reinterpret_cast<InternalParticleClothDeprecated*>(db.getInternalTypedRecord(internalType, objectId));

    if (internalCloth)
    {

        int collisionGroup;
        if (!getValue<int>(attachedStage, internalCloth->mPrim.GetPrimPath(), property, timeCode, collisionGroup))
            return true;

        PxU32* phases = internalCloth->mPhases;
        for (uint32_t i = 0; i < internalCloth->mNumParticles; i++)
        {
            PxU32 phase = phases[i];
            // clear the current group
            phase &= ~PxParticlePhaseFlag::eParticlePhaseGroupMask;
            // Set the new group
            phase |= (collisionGroup & PxParticlePhaseFlag::eParticlePhaseGroupMask);
            phases[i] = phase;
        }
        
        internalCloth->mUploadDirtyFlags |= ParticleBufferFlags::ePHASES;
    }

    return true;
}

bool omni::physx::updateParticleClothPressureDeprecated(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTParticleClothDeprecated;
    InternalParticleClothDeprecated* internalCloth = reinterpret_cast<InternalParticleClothDeprecated*>(db.getInternalTypedRecord(internalType, objectId));

    if (internalCloth)
    {
        float pressure;
        if (!getValue<float>(attachedStage, internalCloth->mPrim.GetPrimPath(), property, timeCode, pressure))
            return true;

        if (pressure > 0.0f)
        {
            CARB_LOG_WARN("Runtime adjustments for pressure on a mesh simulated as cloth won't have an effect. Please stop the simulation and adjust the pressure before rerunning the simulation.\n");
            return false;
        }
    }

    return false;
}

bool omni::physx::updateParticleClothPointsDeprecated(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTParticleClothDeprecated;
    InternalParticleClothDeprecated* internalCloth = reinterpret_cast<InternalParticleClothDeprecated*>(db.getInternalTypedRecord(internalType, objectId));

    if (internalCloth)
    {
        VtArray<GfVec3f> pointsSrc;
        UsdGeomMesh mesh(internalCloth->mPrim);
        UsdAttribute pointsAttr = mesh.GetPointsAttr();
        if (pointsAttr)
        {
            pointsAttr.Get(&pointsSrc);
            uint32_t numSrcExpected = internalCloth->mIsWelded ? uint32_t(internalCloth->mVerticesRemapToWeld.size()) : internalCloth->mNumParticles;

            if (pointsSrc.size() == numSrcExpected)
            {
                if (internalCloth->mIsWelded)
                {
                    for (uint32_t i = 0; i < internalCloth->mNumParticles; ++i)
                    {
                        PxVec4& dst = internalCloth->mPositions[i];
                        uint32_t srcIndex = internalCloth->mVerticesRemapToOrig[i];
                        GfVec3f srcWorld = internalCloth->mLocalToWorld.Transform(pointsSrc[srcIndex]);
                        dst =  PxVec4(toPhysX(srcWorld), dst.w);
                    }
                }
                else
                {
                    for (uint32_t i = 0; i < internalCloth->mNumParticles; ++i)
                    {
                        PxVec4& dst = internalCloth->mPositions[i];
                        GfVec3f srcWorld = internalCloth->mLocalToWorld.Transform(pointsSrc[i]);
                        dst =  PxVec4(toPhysX(srcWorld), dst.w);
                    }
                }
                internalCloth->mUploadDirtyFlags |= ParticleBufferFlags::ePOSITIONS;
                return true;
            }
        }
    }

    return false;
}

bool omni::physx::updateParticleClothVelocitiesDeprecated(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

    PhysXType internalType = ePTParticleClothDeprecated;
    InternalParticleClothDeprecated* internalCloth = reinterpret_cast<InternalParticleClothDeprecated*>(db.getInternalTypedRecord(internalType, objectId));

    if (internalCloth)
    {
        VtArray<GfVec3f> velocitiesSrc;
        UsdGeomMesh mesh(internalCloth->mPrim);
        UsdAttribute velocitiesAttr = mesh.GetVelocitiesAttr();
        if (velocitiesAttr)
        {
            velocitiesAttr.Get(&velocitiesSrc);
            uint32_t numSrcExpected = internalCloth->mIsWelded ? uint32_t(internalCloth->mVerticesRemapToWeld.size()) : internalCloth->mNumParticles;

            if (velocitiesSrc.size() == numSrcExpected)
            {
                if (internalCloth->mIsWelded)
                {
                    for (uint32_t i = 0; i < internalCloth->mNumParticles; ++i)
                    {
                        PxVec4& dst = internalCloth->mVelocities[i];
                        uint32_t srcIndex = internalCloth->mVerticesRemapToOrig[i];
                        dst =  PxVec4(toPhysX(velocitiesSrc[srcIndex]), dst.w);
                    }
                }
                else
                {
                    for (uint32_t i = 0; i < internalCloth->mNumParticles; ++i)
                    {
                        PxVec4& dst = internalCloth->mVelocities[i];
                        dst = PxVec4(toPhysX(velocitiesSrc[i]), dst.w);
                    }
                }
                internalCloth->mUploadDirtyFlags |= ParticleBufferFlags::eVELOCITIES;
                return true;
            }
        }
    }

    return false;
}

bool omni::physx::warnParticleClothNoRuntimeCookingDeprecated(AttachedStage& attachedStage, ObjectId objectId, const pxr::TfToken& property, const pxr::UsdTimeCode& timeCode)
{
    const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();
    PhysXType internalType = ePTParticleClothDeprecated;
    const InternalParticleClothDeprecated* internalCloth = reinterpret_cast<const InternalParticleClothDeprecated*>(db.getInternalTypedRecord(internalType, objectId));
    if (internalCloth && internalCloth->mPrim)
    {
        CARB_LOG_WARN("Changing particle cloth mesh or PhysxSchemaPhysxAutoParticleClothAPI parameter during simulation is not supported. Prim: %s",
            internalCloth->mPrim.GetPath().GetText());
    }
    return true;
}
