// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "Internal.h"
#include "InternalVehicle.h"
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

// Source-agnostic mirror of a point instancer's initial-transform arrays, written by
// InternalActor's constructor (raw USD capture, still fenced there) and consumed by
// resetStartProperties()'s write-back through IPhysicsDataWrite::writeArray (unconditional,
// same tok.positions/orientations/scales/velocities/angularVelocities tokens
// flushInstancerArrays in InternalScene.cpp already uses). orientations are xyzw
// (real-last, PxQuat/IPhysicsDataWrite convention), not GfQuath's wxyz storage order.
struct InitialInstancerData
{
    std::vector<carb::Float3> positions;
    std::vector<carb::Float4> orientations;
    std::vector<carb::Float3> scales;
    std::vector<carb::Float3> velocities;
    std::vector<carb::Float3> angularVelocities;
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
        const omni::physx::usdparser::TireFrictionTableDesc& tireFrictionTableDesc);

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
    // ActorInitialDataMap (internal/InternalActor.h): Kit-only USD xform-op-restore-on-stop
    // state; see its own comment.
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
