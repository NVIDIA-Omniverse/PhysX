// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-WRITE-TRANSFORM-001
 * @covers AC-7
 *
 * @implements REQ-PARSE-BODY-001
 * @covers AC-6
 *
 * @implements REQ-SIM-ACTIVEACTOR-001
 * @covers AC-1
 */

/** @implements REQ-CACHE-ATTACH-001
 * @covers AC-14
 */

// @implements REQ-LOAD-JOINTLIMIT-001
// @covers AC-9

#pragma once

// InternalActor's own constructor is ObjectKey-native and pulls no pxr headers in this header;
// sdf/path.h / usd/prim.h are pulled by InternalActor.cpp itself, only inside the fenced
// sub-block that re-derives a UsdPrim from a real backing stage (see the .cpp).

#include <PhysXDefines.h>
#include <private/omni/physx/PhysxUsd.h>
#include <common/foundation/Allocator.h>

#include <omni/physics/parse/Handles.h>
#include <unordered_map>

#include <utils/SplinesCurve.h>

#include <extensions/PxCollectionExt.h>
#include <foundation/PxMat44.h>

#include <cstdint>
#include <limits>
#include <optional>

namespace physx
{
class PxController;
}

namespace omni { namespace physx { namespace usdparser { class AttachedStage; } } }

namespace omni
{
namespace physx
{
class PhysXScene;

namespace internal
{

class InternalScene;

// Restores authored velocity when simulation stops. The xform-op-stack restore this used to
// embed by value (XformOpResetStorage, genuine USD-authoring-only functionality with no
// ovstage equivalent) now lives inside UsdPhysicsDataWrite itself, keyed by ObjectKey
// (IPhysicsDataWrite::storeXformOpReset/restoreXformOpReset) -- so this struct carries only
// backend-neutral data and needs no fencing. velocity/angularVelocity are plain numeric data --
// carb::Float3-compatible like every other write-sink-routed field in this codebase.
struct ActorInitialData
{
    carb::Float3 velocity{ 0.0f, 0.0f, 0.0f };
    carb::Float3 angularVelocity{ 0.0f, 0.0f, 0.0f };
    bool velocityWritten = false;
    bool angularVelocityWritten = false;
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
    void release(bool trackReleasedActor = true);

    void* mirrorMemory;
    ::physx::PxCollection* collection;
    ::physx::PxRigidActor* actor;
    InternalScene* internalScene;
};

// Whether a prim's children are nested under an enabled rigid body, with no resetXformStack in
// between, keyed by that prim. The answer depends only on ancestors, so a caller that creates
// many actors from one unchanging source read can share it between siblings.
using NestedBodyMemo = std::unordered_map<omni::physics::parse::ObjectKey, bool, omni::physics::parse::ObjectKey::Hash>;

class InternalActor : public Allocateable
{
public:
    // No SdfPath/UsdPrim parameter: the constructor reads what it needs through the
    // attachment's IPhysicsSource, for a USD stage, a stageless ovstage attach and a
    // compiled-cache restore alike. An instanced actor captures its prototype transform and
    // its point instancer's initial transforms for restore-on-stop; any other actor scans
    // its ancestors for an enabled rigid body it is nested under, with no resetXformStack
    // in between. `nestedBodies`, when given, memoizes that scan per parent prim.
    InternalActor(PhysXScene* ps,
                  bool dynamicActor,
                  const usdparser::ObjectInstance* instance,
                  bool localSpaceVelocities,
                  omni::physics::parse::ObjectKey key,
                  NestedBodyMemo* nestedBodies = nullptr);
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
    // Prototype-to-instancer transform, inverted. PhysX layout/convention: a
    // product written A * B in the old GfMatrix4d form is B * A here (see
    // common/foundation/MatrixTools.h).
    ::physx::PxMat44d mProtoTransformInverse{ ::physx::PxIdentity };
    int mID;
    uint32_t mFlags;

    omni::physics::parse::ObjectKey mSourceGPrimKey;

    // How an ordinary load placed this dynamic body, which a compiled-cache hit replays
    // (see cache/AttachmentRecords.cpp): it created the body at mInsertionPose and added it to
    // its scene with the center of mass at the actor origin, and its mass update then passed
    // mAppliedCenterOfMass to PhysX. PhysX normalizes both rotations, on creation as in its pose
    // setter. It keeps each shape's world transform from the insertion until the body moves;
    // the first step reads it.
    std::optional<::physx::PxTransform> mInsertionPose;
    ::physx::PxTransform mAppliedCenterOfMass = ::physx::PxTransform(::physx::PxIdentity);
    // The center of mass PhysX stored for mAppliedCenterOfMass, as the body reported it. The
    // runtime cannot normalize that rotation again bit for bit: the SDK builds without
    // floating-point contraction, and the runtime may fuse multiply-adds, as GCC does on aarch64.
    ::physx::PxTransform mNormalizedCenterOfMass = ::physx::PxTransform(::physx::PxIdentity);

    bool surfaceVelocityEnabled() const
    {
        return mSurfaceVelocityEnabled;
    }
    bool contactSolveEnabled() const
    {
        return mSolveContactEnabled;
    }
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
    // around the wrong point under non-identity anchor transforms. Callers without a
    // separate per-target pose (e.g. setupActor at scene-load time) can pass
    // cloneActor.getGlobalPose().
    // Splines surface velocity isn't handled here -- mSplineLocalSpace would need
    // per-clone recomputation and no current caller needs it.
    void copySurfaceVelocityState(const InternalActor& source,
                                  ::physx::PxRigidActor& cloneActor,
                                  const ::physx::PxTransform& clonePivotPose);

    bool mSurfaceVelocityLocalSpace;
    // Effective surface velocity handed to the contact-modify callback: the authored
    // value with mScale folded in when mSurfaceVelocityLocalSpace is set.
    ::physx::PxVec3 mSurfaceVelocity = ::physx::PxVec3(::physx::PxZero);
    // The same quantity as authored, before that fold. Kept in step with
    // mSurfaceVelocity so that a change to mSurfaceVelocityLocalSpace on its own can
    // re-derive mSurfaceVelocity locally. Reading the surfaceVelocity attribute back
    // from the source instead would resolve at the source's latest state, which is not
    // necessarily the state the change being processed belongs to.
    ::physx::PxVec3 mSurfaceVelocityAuthored = ::physx::PxVec3(::physx::PxZero);
    ::physx::PxVec3 mSurfaceAngularVelocity = ::physx::PxVec3(::physx::PxZero);
    ::physx::PxTransform mSurfaceAngularVelocityPivot = ::physx::PxTransform(::physx::PxIdentity);

    float mSplinesSurfaceVelocityMagnitude = 0.0f;
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

// The authored low and high bound of one joint limit pair as last delivered by a live update, NaN
// until one is (InternalJoint::mLiveLimits, REQ-LOAD-JOINTLIMIT-001 AC-9).
struct JointLiveLimitBounds
{
    float low = std::numeric_limits<float>::quiet_NaN();
    float high = std::numeric_limits<float>::quiet_NaN();
};

class InternalLink : public InternalActor
{
public:
    InternalLink(PhysXScene* ps,
                 const usdparser::ObjectInstance* instance,
                 omni::physics::parse::ObjectKey key)
        : InternalActor(ps, true, instance, false, key), hasInboundJointWithStateAPI(false)
    {
    }
    bool hasInboundJointWithStateAPI;
    // The live limit bounds of the inbound joint while its InternalJoint is rebuilt around the same
    // PhysX joint, for example after a mimic joint API is applied to the joint prim. Handed to the
    // new InternalJoint, so the bounds delivered before the rebuild still pair with later changes
    // (REQ-LOAD-JOINTLIMIT-001 AC-9).
    JointLiveLimitBounds mInboundJointLiveLimits[6];
};

class InternalCct : public InternalActor
{
public:
    InternalCct(PhysXScene* ps,
                const usdparser::ObjectInstance* instance,
                omni::physics::parse::ObjectKey key)
        : InternalActor(ps, true, instance, false, key), mFixupQ(::physx::PxIdentity)
    {
    }

    ::physx::PxQuat mFixupQ;
    // The controller this actor was created for. Released with the record; null only
    // before that assignment.
    ::physx::PxController* mController = nullptr;
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
