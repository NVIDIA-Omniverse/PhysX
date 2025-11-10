// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <PhysXDefines.h>
#include <private/omni/physx/PhysxUsd.h>
#include <common/foundation/Allocator.h>
#include <internal/InternalXformOpResetStorage.h>

#include <utils/SplinesCurve.h>

#include <extensions/PxCollectionExt.h>

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
        velocity = pxr::GfVec3f(0.0f);
        angularVelocity = pxr::GfVec3f(0.0f);
    }

    XformOpResetStorage xformOpStorage;
    pxr::GfVec3f velocity;
    pxr::GfVec3f angularVelocity;
    bool velocityWritten;
    bool angularVelocityWritten;
};
using ActorInitialDataMap = std::unordered_map<pxr::SdfPath, ActorInitialData, pxr::SdfPath::Hash>;

struct InternalActorFlag
{
    enum Enum
    {
        eIS_KINEMATIC = 1 << 0,
        eHAS_PARENT_XFORM = 1 << 1,
        eHAS_DIRTY_MASS = 1 << 2,
        eSKIP_UPDATE_TRANSFORM = 1 << 3,
        eNOTIFY_TRANSFORM = 1 << 4,
        eSKIP_UPDATE_VELOCITY = 1 << 5,
        eNOTIFY_VELOCITY = 1 << 6,
        eNOTIFY_VELOCITY_RADIANS = 1 << 7,
        eLOCALSPACE_VELOCITIES = 1 << 8,
        ePARENT_XFORM_DIRTY = 1 << 9,
        eHAS_TIME_SAMPLED_XFORM = 1 << 10,
        eHAS_EXTRA_TRANSFORM = 1 << 11,
        eEXTRA_TRANSFORM_PRE_OP = 1 << 12,
        eFAST_TRANSFORM = 1 << 13
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
                  const pxr::SdfPath& primPath,
                  const pxr::UsdPrim& prim,
                  bool dynamicActor,
                  const usdparser::ObjectInstance* instance,
                  bool localSpaceVelocities);
    virtual ~InternalActor();

    pxr::UsdPrim mPrim;
    pxr::UsdPrim mParentXformPrim;
    pxr::UsdPrim mInstancePrim;
    pxr::SdfPath mXformOpTranslatePath;
    pxr::SdfPath mXformOpOrientPath;
    pxr::SdfPath mVelocityPath;
    pxr::SdfPath mAngularVelocityPath;
    uint32_t mInstanceIndex;
    ::physx::PxRigidActor* mActor;
    carb::Float3 mScale;
    pxr::GfMatrix4d mProtoTransformInverse;
    int mID;
    uint32_t mFlags;

    pxr::GfMatrix4d mParentWorldTransfInv;
    pxr::GfMatrix4d mExtraTransfInv;

    void enableSurfaceVelocity(bool enable, ::physx::PxRigidActor& actor);
    void enableSplineSurfaceVelocity(bool enable,
                                     ::physx::PxRigidActor& actor,
                                     const pxr::UsdGeomBasisCurves& splinesCurvePrim);
    void enableContactSolve(bool enable, ::physx::PxRigidActor* actor);
    void switchFromKinematic();

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
    void initializeDynamicActor();

    bool mSurfaceVelocityEnabled;
    bool mSplinesSurfaceVelocityEnabled;
    bool mSolveContactEnabled;
};

class InternalLink : public InternalActor
{
public:
    InternalLink(PhysXScene* ps,
                 const pxr::SdfPath& primPath,
                 const pxr::UsdPrim& prim,
                 const usdparser::ObjectInstance* instance)
        : InternalActor(ps, primPath, prim, true, instance, false), hasInboundJointWithStateAPI(false)
    {
    }
    bool hasInboundJointWithStateAPI;
};

class InternalCct : public InternalActor
{
public:
    InternalCct(PhysXScene* ps,
                const pxr::SdfPath& primPath,
                const pxr::UsdPrim& prim,
                const usdparser::ObjectInstance* instance)
        : InternalActor(ps, primPath, prim, true, instance, false), mFixupQ(::physx::PxIdentity)
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
