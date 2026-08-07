// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-WRITE-TRANSFORM-001
 * @covers AC-7
 */

#pragma once

#include <PhysXDefines.h>
#include <private/omni/physx/PhysxUsd.h>
#include <common/foundation/Allocator.h>
#include <internal/InternalXformOpResetStorage.h>

#include <omni/physics/parse/Handles.h>

#include <utils/SplinesCurve.h>

#include <extensions/PxCollectionExt.h>

namespace omni { namespace physx { namespace usdparser { class AttachedStage; } } }

namespace omni
{
namespace physx
{
class PhysXScene;

namespace internal
{

struct ActorInitialData
{
    ActorInitialData()
    {
        velocity = PXR_NS::GfVec3f(0.0f);
        angularVelocity = PXR_NS::GfVec3f(0.0f);
    }

    XformOpResetStorage xformOpStorage;
    PXR_NS::GfVec3f velocity;
    PXR_NS::GfVec3f angularVelocity;
    bool velocityWritten;
    bool angularVelocityWritten;
};
// Keyed by source-agnostic ObjectKey (the actor's key), not a stored SdfPath.
using ActorInitialDataMap =
    std::unordered_map<omni::physics::parse::ObjectKey, ActorInitialData, omni::physics::parse::ObjectKey::Hash>;

struct InternalActorFlag
{
    enum Enum
    {
        eIS_KINEMATIC = 1 << 0,
        eHAS_DIRTY_MASS = 1 << 2,
        eSKIP_UPDATE_TRANSFORM = 1 << 3,
        eNOTIFY_TRANSFORM = 1 << 4,
        eSKIP_UPDATE_VELOCITY = 1 << 5,
        eNOTIFY_VELOCITY = 1 << 6,
        eNOTIFY_VELOCITY_RADIANS = 1 << 7,
        eLOCALSPACE_VELOCITIES = 1 << 8,
        eHAS_TIME_SAMPLED_XFORM = 1 << 10,
        // Transform write-back is routed through IPhysicsDataWrite: the xform-op
        // stack was prepared via prepareTransformWrite, and the sink owns the
        // world->local conversion + residual extra-transform (and XformCommonAPI
        // authoring when that mode is enabled).
        eUSE_DATAWRITE_SINK = 1 << 16
    };
};

struct MirrorActor
{
    void release()
    {
        ::physx::PxCollectionExt::releaseObjects(*collection);
        collection->release();
        free(mirrorMemory);
    }

    void* mirrorMemory;
    ::physx::PxCollection* collection;
    ::physx::PxRigidActor* actor;
};

class InternalActor : public Allocateable
{
public:
    InternalActor(PhysXScene* ps,
                  const PXR_NS::SdfPath& primKey,
                  const PXR_NS::UsdPrim& prim,
                  bool dynamicActor,
                  const usdparser::ObjectInstance* instance,
                  bool localSpaceVelocities,
                  omni::physics::parse::ObjectKey key);
    virtual ~InternalActor();

    // Source-agnostic handle for this actor's prim, used to route transform
    // write-back through IPhysicsDataWrite. Threaded in from the creator (which
    // already minted it) rather than resolved here, to avoid minting a key on
    // the parallel replicator clone path.
    omni::physics::parse::ObjectKey mKey;

    // ObjectKey of the point-instancer prim (when this actor is an instance);
    // resolved to a prim/path on demand. Invalid for non-instanced actors.
    omni::physics::parse::ObjectKey mInstanceKey;
    uint32_t mInstanceIndex;
    ::physx::PxRigidActor* mActor;
    carb::Float3 mScale;
    PXR_NS::GfMatrix4d mProtoTransformInverse;
    int mID;
    uint32_t mFlags;

    omni::physics::parse::ObjectKey mSourceGPrimKey;

    void enableSurfaceVelocity(bool enable, ::physx::PxRigidActor& actor);
    void enableSplineSurfaceVelocity(bool enable,
                                     ::physx::PxRigidActor& actor,
                                     const usdparser::AttachedStage& attachedStage,
                                     omni::physics::parse::ObjectKey splinesCurveKey);
    void enableContactSolve(bool enable, ::physx::PxRigidActor* actor);
    void switchFromKinematic();

    // Replicates PhysxSurfaceVelocityAPI state onto a cloned actor. mirrorHierarchy
    // copies the PxShape filter bit but not the InternalActor fields the contact-modify
    // callback reads, so they must be copied explicitly.
    // mSurfaceAngularVelocityPivot uses the caller-supplied clonePivotPose (not
    // cloneActor.getGlobalPose()): in the replicator path per-target setGlobalPose runs
    // after this copy, so reading the pose here would use the stale source pose and pivot
    // around the wrong point under non-identity parentTransforms. Callers without a
    // separate per-target pose (e.g. setupActor at scene-load time) can pass
    // cloneActor.getGlobalPose().
    // Splines surface velocity isn't handled here -- mSplineLocalSpace would need
    // per-clone recomputation and no current caller needs it.
    void copySurfaceVelocityState(const InternalActor& source,
                                  ::physx::PxRigidActor& cloneActor,
                                  const ::physx::PxTransform& clonePivotPose);

    bool mSurfaceVelocityLocalSpace;
    ::physx::PxVec3 mSurfaceVelocity = ::physx::PxVec3(::physx::PxZero);
    ::physx::PxVec3 mSurfaceAngularVelocity = ::physx::PxVec3(::physx::PxZero);
    ::physx::PxTransform mSurfaceAngularVelocityPivot = ::physx::PxTransform(::physx::PxIdentity);

    float mSplinesSurfaceVelocityMagnitude;
    SplinesCurve* mSplinesCurve;
    ::physx::PxTransform mSplineLocalSpace = ::physx::PxTransform(::physx::PxIdentity);

    PhysXScene* mPhysXScene;

    std::vector<MirrorActor> mMirrors;
    ::physx::PxCollection* mMirrorSharedCollection;
    uint32_t mMirrorMemsize;
    void* mMirrorMemory;

private:
    void initializeDynamicActor(bool runtimeInitialization = false);

    bool mSurfaceVelocityEnabled;
    bool mSplinesSurfaceVelocityEnabled;
    bool mSolveContactEnabled;
};

class InternalLink : public InternalActor
{
public:
    InternalLink(PhysXScene* ps,
                 const PXR_NS::SdfPath& primKey,
                 const PXR_NS::UsdPrim& prim,
                 const usdparser::ObjectInstance* instance,
                 omni::physics::parse::ObjectKey key)
        : InternalActor(ps, primKey, prim, true, instance, false, key), hasInboundJointWithStateAPI(false)
    {
    }
    bool hasInboundJointWithStateAPI;
};

class InternalCct : public InternalActor
{
public:
    InternalCct(PhysXScene* ps,
                const PXR_NS::SdfPath& primKey,
                const PXR_NS::UsdPrim& prim,
                const usdparser::ObjectInstance* instance,
                omni::physics::parse::ObjectKey key)
        : InternalActor(ps, primKey, prim, true, instance, false, key), mFixupQ(::physx::PxIdentity)
    {
    }

    ::physx::PxQuat mFixupQ;
};

class InternalForce : public Allocateable
{
public:
    void setForce(const ::physx::PxVec3& force)
    {
        if (force.magnitudeSquared() > kAlmostZero)
        {
            mForceEnabled = true;
        }
        else
        {
            mForceEnabled = false;
        }
        mForce = force;
    }
    const ::physx::PxVec3& getForce() const
    {
        return mForce;
    }

    void setTorque(const ::physx::PxVec3& torque)
    {
        if (torque.magnitudeSquared() > kAlmostZero)
        {
            mTorqueEnabled = true;
        }
        else
        {
            mTorqueEnabled = false;
        }
        mTorque = torque;
    }
    const ::physx::PxVec3& getTorque() const
    {
        return mTorque;
    }

    bool mWorldFrame;
    bool mAccelerationMode;

    bool mEnabled;
    bool mCoMApplied;

    bool mForceEnabled;
    bool mTorqueEnabled;

    bool mBodyPrimDifferent;

    ::physx::PxQuat mLocalRot;
    ::physx::PxRigidActor* mRigidActor;
    PhysXScene* mPhysXScene;
    ::physx::PxVec3 mLocalPos;

private:
    ::physx::PxVec3 mForce;
    ::physx::PxVec3 mTorque;
};

} // namespace internal
} // namespace physx
} // namespace omni
