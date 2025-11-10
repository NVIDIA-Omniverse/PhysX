// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "Internal.h"
#include "InternalVehicle.h"
#include "InternalVoxelMap.h"
#include "InternalActor.h"
#include "InternalScene.h"

#include <utils/SplinesCurve.h>

namespace omni
{
namespace physx
{
namespace internal
{

using TransformsMap = pxr::TfHashMap<pxr::SdfPath, ::physx::PxTransform, pxr::SdfPath::Hash>;
using PxJointMap = std::unordered_multimap<pxr::SdfPath, std::pair<::physx::PxJoint*, bool>, pxr::SdfPath::Hash>;
using SplinesCurveMap = pxr::TfHashMap<pxr::SdfPath, SplinesCurve*, pxr::SdfPath::Hash>;

struct InitialInstancerData
{
    pxr::VtArray<pxr::GfVec3f> positions;
    pxr::VtArray<pxr::GfQuath> orientations;
    pxr::VtArray<pxr::GfVec3f> scales;
    pxr::VtArray<pxr::GfVec3f> velocities;
    pxr::VtArray<pxr::GfVec3f> angularVelocities;
};

using TransformsInstanceMap = pxr::TfHashMap<pxr::SdfPath, InitialInstancerData, pxr::SdfPath::Hash>;
using VectorMap = pxr::TfHashMap<pxr::SdfPath, ::physx::PxVec3, pxr::SdfPath::Hash>;

class InternalPhysXDatabase : public InternalDatabase
{
public:
    InternalPhysXDatabase();
    ~InternalPhysXDatabase();

    void release();

    void resetStartProperties(bool useUsdUpdate, bool useVelocitiesUSDUpdate, bool outputVelocitiesLocalSpace);

    omni::physx::usdparser::ObjectId createTireFrictionTable(
        const omni::physx::usdparser::TireFrictionTableDesc& tireFrictionTableDesc, const pxr::UsdPrim& usdPrim);

    void addDirtyMassActor(size_t actorIndex);
    void addDirtyMassActor(InternalActor* actor);
    void removeDirtyMassActor(InternalActor* actor);

    void addDirtyMassDeformableBodyDeprecated(size_t actorIndex);
    void addDirtyMassDeformableBodyDeprecated(InternalDeformableBodyDeprecated* deformable);
    void removeDirtyMassDeformableBodyDeprecated(InternalDeformableBodyDeprecated* deformable);

    void addDirtyMassDeformableSurfaceDeprecated(size_t actorIndex);
    void addDirtyMassDeformableSurfaceDeprecated(InternalDeformableSurfaceDeprecated* deformable);
    void removeDirtyMassDeformableSurfaceDeprecated(InternalDeformableSurfaceDeprecated* deformable);

    void addDirtyMassParticle(size_t actorIndex);
    void addDirtyMassParticle(InternalParticle* particle);
    void removeDirtyMassParticle(InternalParticle* particle);

    void updateDirtyMassActors();

    const PxJointMap& getPxJointMap() const
    {
        return mPxJointMap;
    }
    void clearPxJointMap()
    {
        mPxJointMap.clear();
    }
    void storePxJoint(const ::physx::PxRigidActor* actor, const pxr::SdfPath& path);
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

    // These method names might be confusing because other method with very similar names exist
    void addResidualReportingJoint(const ::physx::PxJoint* joint)
    {
        mResitualPxJoints.insert(joint);
    }
    void removeResidualReportingJoint(const ::physx::PxJoint* joint)
    {
        mResitualPxJoints.erase(joint);
    }
    void clearResidualReportingJoints()
    {
        mResitualPxJoints.clear();
    }

    void updateSimulationOutputs(bool updateResidualsToUsd);

    SplinesCurve* addSplinesCurve(const pxr::UsdGeomBasisCurves& curve, bool& added);

private:
    void updateJointResiduals(bool updateResidualsToUsd);

public:
    bool mInitialTransformsStored;
    ActorInitialDataMap mInitialActorDataMap;
    TransformsInstanceMap mInitialPointInstancerTransforms;

    PxJointMap mPxJointMap;

    pxr::UsdGeomXformCache mXformCache;

private:
    std::vector<InternalActor*> mDirtyMassActorList;
    std::vector<InternalDeformableBodyDeprecated*> mDirtyMassDeformableBodyListDeprecated;
    std::vector<InternalDeformableSurfaceDeprecated*> mDirtyMassDeformableSurfaceListDeprecated;
    std::vector<InternalParticle*> mDirtyMassParticleList;

    std::unordered_set<const ::physx::PxJoint*> mResitualPxJoints;

    SplinesCurveMap mSplinesMap;
};


} // namespace internal
} // namespace physx
} // namespace omni
