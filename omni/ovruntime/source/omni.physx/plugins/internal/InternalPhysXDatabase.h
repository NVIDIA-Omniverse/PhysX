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

namespace omni
{
namespace physx
{
namespace internal
{


using TransformsMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, ::physx::PxTransform, PXR_NS::SdfPath::Hash>;
using PxJointMap = std::unordered_multimap<PXR_NS::SdfPath, std::pair<::physx::PxJoint*, bool>, PXR_NS::SdfPath::Hash>;
using SplinesCurveMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, SplinesCurve*, PXR_NS::SdfPath::Hash>;

struct InitialInstancerData
{
    PXR_NS::VtArray<PXR_NS::GfVec3f> positions;
    PXR_NS::VtArray<PXR_NS::GfQuath> orientations;
    PXR_NS::VtArray<PXR_NS::GfVec3f> scales;
    PXR_NS::VtArray<PXR_NS::GfVec3f> velocities;
    PXR_NS::VtArray<PXR_NS::GfVec3f> angularVelocities;
};

using TransformsInstanceMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, InitialInstancerData, PXR_NS::SdfPath::Hash>;
using VectorMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, ::physx::PxVec3, PXR_NS::SdfPath::Hash>;

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

    void updateStats();

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
    void storePxJoint(const ::physx::PxRigidActor* actor, const PXR_NS::SdfPath& path);
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

    SplinesCurve* addSplinesCurve(const PXR_NS::UsdGeomBasisCurves& curve, bool& added);

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

    PXR_NS::UsdGeomXformCache mXformCache;

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
