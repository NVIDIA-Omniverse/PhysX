// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "Internal.h"
#include "InternalVehicle.h"
#include "InternalVoxelMap.h"
#include "InternalActor.h"
#include "InternalScene.h"
#include "InternalDebugDraw.h"

#include <utils/SplinesCurve.h>
#include <utils/OmniRenderBuffer.h>

#include <omni/physx/IPhysxVisualization.h>

namespace omni { namespace physx { namespace usdparser { class AttachedStage; } } }

namespace omni
{
namespace physx
{
namespace internal
{


// All keyed by source-agnostic ObjectKey, not a stored SdfPath.
using PxJointMap = std::unordered_multimap<omni::physics::parse::ObjectKey, std::pair<::physx::PxJoint*, bool>,
                                           omni::physics::parse::ObjectKey::Hash>;
using SplinesCurveMap =
    std::unordered_map<omni::physics::parse::ObjectKey, SplinesCurve*, omni::physics::parse::ObjectKey::Hash>;

struct InitialInstancerData
{
    PXR_NS::VtArray<PXR_NS::GfVec3f> positions;
    PXR_NS::VtArray<PXR_NS::GfQuath> orientations;
    PXR_NS::VtArray<PXR_NS::GfVec3f> scales;
    PXR_NS::VtArray<PXR_NS::GfVec3f> velocities;
    PXR_NS::VtArray<PXR_NS::GfVec3f> angularVelocities;
};

using TransformsInstanceMap =
    std::unordered_map<omni::physics::parse::ObjectKey, InitialInstancerData, omni::physics::parse::ObjectKey::Hash>;

class InternalPhysXDatabase : public InternalDatabase
{
public:
    InternalPhysXDatabase();
    ~InternalPhysXDatabase();

    void release();

    void resetStartProperties(bool useUsdUpdate, bool useVelocitiesUSDUpdate, bool outputVelocitiesLocalSpace);

    omni::physx::usdparser::ObjectId createTireFrictionTable(
        const omni::physx::usdparser::TireFrictionTableDesc& tireFrictionTableDesc, const PXR_NS::UsdPrim& usdPrim);

    void addDirtyMassActor(size_t actorIndex);
    void addDirtyMassActor(InternalActor* actor);
    void removeDirtyMassActor(InternalActor* actor);

    void addDirtyMassParticle(size_t actorIndex);
    void addDirtyMassParticle(InternalParticle* particle);
    void removeDirtyMassParticle(InternalParticle* particle);

    void updateDirtyMassActors();

    void debugDraw();
    void setVisualizationParameter(PhysXVisualizationParameter param, bool val);
    const ::physx::PxRenderBuffer& getDebugRenderBuffer() const
    {
        return mRenderBuffer;
    }
    ::physx::PxRenderBuffer& getDebugRenderBuffer()
    {
        return mRenderBuffer;
    }
    void clearDebugRenderBuffer()
    {
        mRenderBuffer.clear();
    }
    uint64_t getDebugDrawFlags() const 
    {
        return mDebugDrawFlags;
    }

    const PxJointMap& getPxJointMap() const
    {
        return mPxJointMap;
    }
    void clearPxJointMap()
    {
        mPxJointMap.clear();
    }
    void storePxJoint(const ::physx::PxRigidActor* actor, omni::physics::parse::ObjectKey key);
    void removePxJoint(const ::physx::PxJoint* joint)
    {
        PxJointMap::const_iterator it = mPxJointMap.begin();
        PxJointMap::const_iterator itEnd = mPxJointMap.end();
        while (it != itEnd)
        {
            if (it->second.first == joint)
            {
                it = mPxJointMap.erase(it);
            }
            else
            {
                it++;
            }
        }
    }

    SplinesCurve* addSplinesCurve(const usdparser::AttachedStage& attachedStage,
                                  omni::physics::parse::ObjectKey curveKey,
                                  bool& added);

    bool getNestedBodiesUsed() const
    {
        return mNestedBodiesUsed;
    }
    void setNestedBodiesUsed(bool val)
    {
        mNestedBodiesUsed = val;
    }

public:
    bool mInitialTransformsStored;
    ActorInitialDataMap mInitialActorDataMap;
    TransformsInstanceMap mInitialPointInstancerTransforms;

    PxJointMap mPxJointMap;

private:
    std::vector<InternalActor*> mDirtyMassActorList;
    std::vector<InternalParticle*> mDirtyMassParticleList;

    std::unordered_set<const ::physx::PxJoint*> mResitualPxJoints;

    SplinesCurveMap mSplinesMap;

    OmniRenderBuffer mRenderBuffer; // used for debug vis
    uint64_t mDebugDrawFlags;
    bool mNestedBodiesUsed;
};


} // namespace internal
} // namespace physx
} // namespace omni
