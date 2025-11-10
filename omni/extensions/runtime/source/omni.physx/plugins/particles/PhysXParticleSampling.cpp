// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXParticleSampling.h"

#include <OmniPhysX.h>
#include <CookingDataAsync.h>
#include "usdLoad/LoadUsd.h"

#include <common/utilities/MemoryMacros.h>

// only for logging
#include <carb/PluginUtils.h>

using namespace pxr;
using namespace omni::physx::particles;

static const TfToken particleSamplingCrcToken = TfToken("physxParticleSampling:crc");

namespace
{

typedef std::map<SdfPath, PhysxParticleFactory*> PathToFactoryMap;
PathToFactoryMap gParticlePrimsToFactoryMap;

PhysxParticleFactory* getParticleFactory(const SdfPath& particlePrimPath)
{
    auto it = gParticlePrimsToFactoryMap.find(particlePrimPath);
    return (it != gParticlePrimsToFactoryMap.end()) ? it->second : nullptr;
}

template <typename T>
void move(VtArray<T>& vtArray, size_t dst, size_t src, size_t count)
{
    std::memmove(&vtArray[dst], &vtArray[src], count * sizeof(T));
}

template <typename T>
void copy(VtArray<T>& dstArray, const VtArray<T>& srcArray, size_t dst, size_t src, size_t count)
{
    std::memcpy(&dstArray[dst], &srcArray[src], count * sizeof(T));
}

struct ParticleData
{
    ParticleData(UsdPrim particlePrim)
    {
        geomPoints = UsdGeomPoints(particlePrim);
        geomInstancer = UsdGeomPointInstancer(particlePrim);
        PhysxSchemaPhysxParticleSetAPI tmpSetAPI(particlePrim);
        if (tmpSetAPI.GetSimulationPointsAttr().HasAuthoredValue())
        {
            // only initialize particleAPI if simulation points are present
            // since that's the only attribute we are reading through particleAPI
            particleAPI = tmpSetAPI;
        }
    }

    void read()
    {
        if (particleAPI)
        {
            particleAPI.GetSimulationPointsAttr().Get(&simPoints);
        }

        if (geomPoints)
        {
            geomPoints.GetPointsAttr().Get(&points);
            geomPoints.GetVelocitiesAttr().Get(&velocities);
            geomPoints.GetWidthsAttr().Get(&pointsWidths);
        }
        else if (geomInstancer)
        {
            geomInstancer.GetPositionsAttr().Get(&points);
            geomInstancer.GetVelocitiesAttr().Get(&velocities);
            geomInstancer.GetProtoIndicesAttr().Get(&instancerProtoIndices);
            geomInstancer.GetScalesAttr().Get(&instancerScales);
            geomInstancer.GetOrientationsAttr().Get(&instancerOrientations);
        }
    }

    void readPoints()
    {
        if (particleAPI)
        {
            particleAPI.GetSimulationPointsAttr().Get(&simPoints);
        }

        if (geomPoints)
        {
            geomPoints.GetPointsAttr().Get(&points);
        }
        else if (geomInstancer)
        {
            geomInstancer.GetPositionsAttr().Get(&points);
        }
    }

    void write()
    {
        if (particleAPI)
        {
            particleAPI.GetSimulationPointsAttr().Set(simPoints);
        }

        if (geomPoints)
        {
            geomPoints.GetPointsAttr().Set(points);
            geomPoints.GetVelocitiesAttr().Set(velocities);
            geomPoints.GetWidthsAttr().Set(pointsWidths);
        }
        else if (geomInstancer)
        {
            geomInstancer.GetPositionsAttr().Set(points);
            geomInstancer.GetVelocitiesAttr().Set(velocities);
            geomInstancer.GetProtoIndicesAttr().Set(instancerProtoIndices);
            geomInstancer.GetScalesAttr().Set(instancerScales);
            geomInstancer.GetOrientationsAttr().Set(instancerOrientations);
        }
    }

    void writePoints()
    {
        if (particleAPI)
        {
            particleAPI.GetSimulationPointsAttr().Set(simPoints);
        }

        if (geomPoints)
        {
            geomPoints.GetPointsAttr().Set(points);
        }
        else if (geomInstancer)
        {
            geomInstancer.GetPositionsAttr().Set(points);
        }
    }

    void writeInstancerProtoRadius(float radius)
    {
        if (geomInstancer)
        {
            SdfPathVector targets;
            geomInstancer.GetPrototypesRel().GetTargets(&targets);
            if (targets.size() > 0)
            {
                SdfPath protoPath = targets[0];
                UsdGeomSphere sphere = UsdGeomSphere::Get(geomInstancer.GetPrim().GetStage(), protoPath);
                if (sphere)
                {
                    sphere.GetRadiusAttr().Set(double(radius));
                }
            }
        }
    }

    void resize(size_t newSize)
    {
        if (particleAPI)
        {
            simPoints.resize(newSize);
        }

        if (geomPoints)
        {
            points.resize(newSize);
            velocities.resize(newSize);
            pointsWidths.resize(newSize);
        }
        else if (geomInstancer)
        {
            points.resize(newSize);
            velocities.resize(newSize);
            instancerProtoIndices.resize(newSize);
            instancerScales.resize(newSize);
            instancerOrientations.resize(newSize);
        }
    }

    void move(size_t dst, size_t src, size_t count)
    {
        if (count > 0)
        {
            if (particleAPI)
            {
                ::move(simPoints, dst, src, count);
            }

            if (geomPoints)
            {
                ::move(points, dst, src, count);
                ::move(velocities, dst, src, count);
                ::move(pointsWidths, dst, src, count);
            }
            else if (geomInstancer)
            {
                ::move(points, dst, src, count);
                ::move(velocities, dst, src, count);
                ::move(instancerProtoIndices, dst, src, count);
                ::move(instancerScales, dst, src, count);
                ::move(instancerOrientations, dst, src, count);
            }
        }
    }

    void copy(const ParticleData& pdSrc, size_t dst, size_t src, size_t count)
    {
        if (count > 0)
        {
            if (pdSrc.particleAPI && particleAPI)
            {
                ::copy(simPoints, pdSrc.simPoints, dst, src, count);
            }

            if (pdSrc.geomPoints && geomPoints)
            {
                ::copy(points, pdSrc.points, dst, src, count);
                ::copy(velocities, pdSrc.velocities, dst, src, count);
                ::copy(pointsWidths, pdSrc.pointsWidths, dst, src, count);
            }
            else if (pdSrc.geomInstancer && geomInstancer)
            {
                ::copy(points, pdSrc.points, dst, src, count);
                ::copy(velocities, pdSrc.velocities, dst, src, count);
                ::copy(instancerProtoIndices, pdSrc.instancerProtoIndices, dst, src, count);
                ::copy(instancerScales, pdSrc.instancerScales, dst, src, count);
                ::copy(instancerOrientations, pdSrc.instancerOrientations, dst, src, count);
            }
        }
    }

    void set(const GfVec3f* positions, size_t numPoints, const GfMatrix4d& rigidTransform, float pointWidth,
             size_t dstIndex, size_t initializedEndIndex)
    {
        for (size_t p = 0; p < numPoints; ++p)
        {
            GfVec3f position = rigidTransform.Transform(positions[p]);
            points[dstIndex + p] = position;
        }

        if (particleAPI)
            ::copy(simPoints, points, dstIndex, dstIndex, numPoints);

        for (size_t i = dstIndex; i < dstIndex + numPoints; ++i)
        {
            velocities[i] = { 0.0, 0.0, 0.0 };
            if (geomPoints)
            {
                pointsWidths[i] = pointWidth;
            }
            else if (geomInstancer)
            {
                instancerProtoIndices[i] = 0;
                if (i >= initializedEndIndex)
                {
                    instancerScales[i] = GfVec3f(1.0f);
                    instancerOrientations[i] = GfQuath::GetIdentity();
                }
            }
        }
    }

    void transformPoints(const GfMatrix4d& transform, size_t dstIndex, size_t count)
    {
        if (dstIndex + count <= points.size())
        {
            for (size_t i = dstIndex; i < dstIndex + count; ++i)
                points[i] = transform.Transform(points[i]);

            if (particleAPI)
            {
                for (size_t i = dstIndex; i < dstIndex + count; ++i)
                    simPoints[i] = transform.Transform(simPoints[i]);
            }
        }
    }

    bool checkSizes()
    {
        bool sizesMatch = true;
        if (particleAPI)
        {
            sizesMatch &= (simPoints.size() == points.size());
        }

        if (geomPoints)
        {
            sizesMatch &= (velocities.size() == points.size());
            sizesMatch &= (pointsWidths.size() == points.size());
        }
        else if (geomInstancer)
        {
            sizesMatch &= (velocities.size() == points.size());
            sizesMatch &= (instancerProtoIndices.size() == points.size());
            sizesMatch &= (instancerScales.size() == points.size());
            sizesMatch &= (instancerOrientations.size() == points.size());
        }
        return sizesMatch;
    }

    UsdGeomPoints geomPoints;
    UsdGeomPointInstancer geomInstancer;
    PhysxSchemaPhysxParticleSetAPI particleAPI;

    VtArray<GfVec3f> points;
    VtArray<GfVec3f> velocities;
    VtArray<GfVec3f> simPoints;
    VtArray<float> pointsWidths;
    VtArray<int> instancerProtoIndices;
    VtArray<GfVec3f> instancerScales;
    VtArray<GfQuath> instancerOrientations;
};

} // namespace

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PhysxParticleFactory::PhysxParticleFactory(SdfPath particlePath) :
    mPath(particlePath),
    mSamplers(PathToSamplerMap()),
    mInitialized(false),
    mTotalParticleCount(0)
{ 

}

PhysxParticleFactory::~PhysxParticleFactory()
{
    // need to release all the samplers that are left
    for (auto it: mSamplers)
    {
        SAFE_DELETE_SINGLE(it.second);
    }
    mSamplers.clear();
}

void PhysxParticleFactory::addSampler(SdfPath samplerPath)
{
    PhysxParticleSampler* sampler = ICE_NEW(PhysxParticleSampler)(samplerPath, mPath);
    mSamplers[samplerPath] = sampler;
}

bool PhysxParticleFactory::updateSampler(SdfPath samplerPath, bool forceResampling)
{
    PhysxParticleSampler* sampler = getParticleSampler(samplerPath);
    if (sampler)
    {
        return sampler->update(forceResampling);
    }
    return false;
}

void PhysxParticleFactory::removeSampler(SdfPath samplerPath)
{
    // we need to do this on the factory level to make sure the data stays consistent.
    PathToSamplerMap::const_iterator samplerIterator = mSamplers.find(samplerPath);
    if (samplerIterator != mSamplers.end())
    {
        PhysxParticleSampler* samplerPointer = samplerIterator->second;
        // make sure we only do USD stuff with a valid stage
        UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
        if (stage)
        {
            UsdPrim particlePrim = stage->GetPrimAtPath(samplerIterator->second->getTarget());

            if (particlePrim)
            {
                ParticleData particleData(particlePrim);
                particleData.read();

                // sanity check: if points or velocities are empty, early out
                if (particleData.points.empty() || particleData.velocities.empty())
                    return;

                ParticleData newParticleData(particlePrim);
                uint32_t copyDst = 0;

                // loop over other samplers and recreate the particle prim.
                for (PathToSamplerMap::const_iterator otherSamplers = mSamplers.cbegin(); otherSamplers != mSamplers.cend(); otherSamplers++)
                {
                    SdfPath path = otherSamplers->first;

                    if (path == samplerPath)
                        continue;

                    uint32_t copySrc = otherSamplers->second->getStartIndex();
                    uint32_t copyCount = otherSamplers->second->getParticleCount();

                    newParticleData.resize(copyDst + copyCount);
                    newParticleData.copy(particleData, copyDst, copySrc, copyCount);

                    otherSamplers->second->setStartIndex(copyDst);
                    copyDst += copyCount;
                }

                // set the new data.

                // AD hack to make USD updates work and silence warnings about inconsistent primvars - remove from hydra db
                particlePrim.CreateAttribute(TfToken("omni:rtx:skip"), SdfValueTypeNames->Bool).Set(true);

                SdfChangeBlock changeBlock;
                {
                    newParticleData.write();
                } // changeblock.

                // correct the total count
                mTotalParticleCount = newParticleData.points.size();

                // AD hack to make usd updates appear and silence warnings about inconsistent primvars.
                particlePrim.CreateAttribute(TfToken("omni:rtx:skip"), SdfValueTypeNames->Bool).Set(false);
            }

            // remove the custom attributes if the prim still exists
            // and toggle visibility
            UsdPrim prim = stage->GetPrimAtPath(samplerPath);
            if (prim)
            {
                prim.RemoveProperty(particleSamplingCrcToken);
                UsdGeomImageable img(prim);
                img.MakeVisible();
            }
        }

        // cleanup the lists
        SAFE_DELETE_SINGLE(samplerPointer);
        mSamplers.erase(samplerPath);
    }
}

PhysxParticleSampler* PhysxParticleFactory::getParticleSampler(SdfPath samplerPath)
{
    PathToSamplerMap::const_iterator it = mSamplers.find(samplerPath);
    return (it != mSamplers.end()) ? it->second : nullptr;
}

void PhysxParticleFactory::processParticleSamplingResults(SdfPath samplerPath,
                                                          const GfVec3f* positions,
                                                          size_t numPoints,
                                                          float pointWidth,
                                                          const GfMatrix4d& rigidTransform,
                                                          const GfMatrix3d& shearScaleTransform,
                                                          bool registerOriginalCount)
{
    PhysxParticleSampler* sampler = getParticleSampler(samplerPath);

    if (!sampler)
        return;

    // check whether total count of the target prim and array lengths are still consistent
    bool consistent = checkTargetCountsAndSizes(registerOriginalCount);

    if (!consistent)
    {
        CARB_LOG_WARN("%s: Physx particle sampling - target particle prim has a different point count than expected! Trying to recover..", mPath.GetText());

        // set mStartIndex and mParticleCount to 0 for all samplers of this factory - will force recreation, but other samplers need
        // to be resampled first.
        resetStartAndCountForAllSamplers();
    }

    if (registerOriginalCount)
    {
        sampler->processSamplingRegistration(numPoints, mTotalParticleCount,
                                             rigidTransform, shearScaleTransform);
        mTotalParticleCount += numPoints;
    }
    else
    {
        int firstChangedIndex = 0;
        int shiftValue = 0;
        bool topologyChange = false;
        topologyChange = sampler->processSamplingResults(positions, numPoints, pointWidth,
                                                         rigidTransform, shearScaleTransform,
                                                         mInitialized, !consistent,
                                                         firstChangedIndex, shiftValue);

        // update the other samplers sampling to that prim to make sure we can resample them again.
        if (topologyChange)
        {
            moveStartIndices(firstChangedIndex, shiftValue);
            saveTotalCount();
        }
    }
    
    // hack for renaming - this tells us that there is at least 1 sampler that actually sampled particles into this set.
    mInitialized = true;
}

void PhysxParticleFactory::moveStartIndices(int firstChangedIndex, int shiftValue)
{
    for (auto it: mSamplers)
    {
        it.second->moveStartIndex(firstChangedIndex, shiftValue);
    }
}

/*
*  checks:
*
*  1) does particle prim exist
*  2) are the sizes of all the array elements consistent
*  3) do these sizes match what we last wrote into the prim when sampling
*
*/
bool PhysxParticleFactory::checkTargetCountsAndSizes(bool registerOriginalCount)
{
    bool result = false;

    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    const UsdPrim particlePrim = stage->GetPrimAtPath(mPath);

    if (particlePrim)
    {
        // get all the attributes
        ParticleData particleData(particlePrim);
        particleData.read();

        // check if sizes are consistent
        bool sizesMatch = particleData.checkSizes();

        // check whether actual count matches what we wrote the last time.
        bool countsMatch = (particleData.points.size() == mTotalParticleCount);

        result = ((countsMatch || registerOriginalCount) && sizesMatch);
    }

    return result;
}

void PhysxParticleFactory::resetStartAndCountForAllSamplers()
{
    for (auto it: mSamplers)
    {
        it.second->setStartIndex(0);
        it.second->setParticleCount(0);
    }

    mTotalParticleCount = 0;
}

void PhysxParticleFactory::saveTotalCount()
{
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    const UsdPrim particlePrim = stage->GetPrimAtPath(mPath);

    if (particlePrim)
    {
        UsdGeomPoints points = UsdGeomPoints(particlePrim);
        UsdGeomPointInstancer instancer = UsdGeomPointInstancer(particlePrim);
        UsdAttribute positionsAttr = points ? points.GetPointsAttr() : instancer.GetPositionsAttr();

        VtArray<GfVec3f> oldPoints;
        positionsAttr.Get(&oldPoints);

        mTotalParticleCount = oldPoints.size();
        return;
    }

    // sanity exit if prim does not exist.
    mTotalParticleCount = 0;
}

// STATIC:
void PhysxParticleFactory::processSamplingResults(SdfPath samplerPath,
                                                  SdfPath particleSetPath,
                                                  const GfVec3f* positions,
                                                  size_t numPoints,
                                                  float pointWidth,
                                                  const GfMatrix4d& rigidTransform,
                                                  const GfMatrix3d& shearScaleTransform,
                                                  bool registerOriginalCount)
{
    PhysxParticleFactory* factory = getParticleFactory(particleSetPath);
    if (factory)
    {
        factory->processParticleSamplingResults(samplerPath, positions, numPoints, pointWidth,
                                                rigidTransform, shearScaleTransform, registerOriginalCount);
    }
    else
    {
        // factory doesn't exist - means API has been removed etc.
        // we remove the crc + reset the visibility.
        UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
        if (stage)
        {
            UsdPrim prim = stage->GetPrimAtPath(samplerPath);
            if (prim)
            {
                prim.RemoveProperty(particleSamplingCrcToken);
                UsdGeomImageable img(prim);
                if (img)
                    img.MakeVisible();
            }
        }
    }
}

// STATIC:
bool PhysxParticleFactory::getDecomposedTransform(SdfPath samplerPath,
                                                  SdfPath particleSetPath,
                                                  GfMatrix4d& rigidTransform,
                                                  GfMatrix3d& shearScaleTransform)
{
    PhysxParticleFactory* factory = getParticleFactory(particleSetPath);
    if (!factory)
        return false;

    PhysxParticleSampler* sampler = factory->getParticleSampler(samplerPath);
    if (!sampler)
        return false;

    return sampler->getDecomposedTransform(rigidTransform, shearScaleTransform);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PhysxParticleSampler::PhysxParticleSampler(SdfPath path, SdfPath target) :
    mSamplerPath(path),
    mTarget(target),
    mStartIndex(0),
    mParticleCount(0)
{
    if (!getDecomposedTransform(mRigidTransform, mShearScaleTransform))
    {
        mRigidTransform.SetIdentity();
        mShearScaleTransform.SetIdentity();
    }
}

PhysxParticleSampler::~PhysxParticleSampler()
{

}

bool PhysxParticleSampler::update(bool forceResampling)
{
    // check for validity of API
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    UsdPrim prim = stage->GetPrimAtPath(mSamplerPath);
    PhysxSchemaPhysxParticleSamplingAPI samplingApi(prim);
    SdfPath particlesPath = mTarget;

    if (!samplingApi)
    {
        return false;
    }

    // check if targets still match
    SdfPathVector targets;
    samplingApi.GetParticlesRel().GetTargets(&targets);
    if (targets.size() > 0 && targets[0] != mTarget)
    {
        return false;
    }

    // check for transform/translate update.
    bool needToResample;
    GfMatrix4d newRigidTransform;
    bool needToTransform = checkTransforms(needToResample, newRigidTransform);

    bool result = true;
    if (needToTransform && needToResample || forceResampling)
    {
        // then kick off if resampling might be needed.
        UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
        omni::physx::usdparser::ParticleSamplingDesc* samplingDesc = omni::physx::usdparser::parseParticleSampling(OmniPhysX::getInstance().getStageId(), mSamplerPath);
        cookingdataasync::CookingDataAsync* cookingDataAsync = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
        UsdPrim prim = stage->GetPrimAtPath(mSamplerPath);

        if (cookingDataAsync && samplingDesc && prim)
        {
            cookingDataAsync->poissonSampleMesh(prim, *samplingDesc, forceResampling, true);
        }
        else
        {
            result = false;
        }
        ICE_FREE(samplingDesc);
    }
    else if (needToTransform)
    {
        transformPoints(newRigidTransform);
    }

    return result;
}

bool PhysxParticleSampler::processSamplingRegistration(size_t numPoints,
                                                       size_t totalRegisteredPoints,
                                                       const GfMatrix4d& rigidTransform,
                                                       const GfMatrix3d& shearScaleTransform)
{
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    const UsdPrim prim = stage->GetPrimAtPath(mSamplerPath);

    // update the member fields.
    mStartIndex = (uint32_t)totalRegisteredPoints;
    mParticleCount = (uint32_t)numPoints;
    mRigidTransform = rigidTransform;
    mShearScaleTransform = shearScaleTransform;

    // hide the source mesh
    UsdGeomImageable img(prim);
    img.MakeInvisible();

    return true;
}

bool PhysxParticleSampler::processSamplingResults(const GfVec3f* positions,
                                                  size_t numPoints,
                                                  float pointWidth,
                                                  const GfMatrix4d& rigidTransform,
                                                  const GfMatrix3d& shearScaleTransform,
                                                  bool factoryInitialized,
                                                  bool recreate,
                                                  int& firstChangedIndex,
                                                  int& shiftValue)
{
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    const UsdPrim prim = stage->GetPrimAtPath(mSamplerPath);
    const UsdPrim particlePrim = stage->GetPrimAtPath(mTarget);

    if (particlePrim)
    {
        // AD hack to make USD updates work and silence warnings about inconsistent primvars - remove from hydra db
        particlePrim.CreateAttribute(TfToken("omni:rtx:skip"), SdfValueTypeNames->Bool).Set(true);

        ParticleData oldParticleData(particlePrim);
        oldParticleData.read();

        // AD: we checked whether the counts match outside of this function.
        size_t oldSize = oldParticleData.points.size();

        // if we had an inconsistent particle prim, we force recreation.
        if (recreate)
            oldSize = 0;

        // need to know the start index and number of points of the sampled prim here.
        uint32_t start = mStartIndex;
        uint32_t currentPoints = mParticleCount;

        // AD: legacy, I'm not sure if we still need the first part of the if?
        // I added the second part when adding support for complete recreate ops.
        if ((oldSize > 0 && !factoryInitialized) || recreate)
        {
            ParticleData particleData(particlePrim);

            SdfChangeBlock changeBlock;
            particleData.write();
            oldSize = 0;
        }

        // This is a means to figure out if the sampler is new and we should append.
        if (mStartIndex == 0 && mParticleCount == 0)
            start = (uint32_t)oldSize;
        
        size_t newNumPoints = oldSize - currentPoints + numPoints;

        // resize and move elements of samplers after elements of this sampler
        size_t moveDst = start + numPoints;
        size_t moveSrc = start + currentPoints;
        size_t moveCount = moveSrc < oldSize ? oldSize - moveSrc : 0;
        if (newNumPoints < oldSize)
        {
            //if we shrink, we need to move before resizing
            oldParticleData.move(moveDst, moveSrc, moveCount);
            oldParticleData.resize(newNumPoints);
        }
        else if (newNumPoints > oldSize)
        {
            //if we grow, we need to resize first
            oldParticleData.resize(newNumPoints);
            oldParticleData.move(moveDst, moveSrc, moveCount);
        }

        CARB_ASSERT((start + numPoints) <= oldParticleData.points.size());

        // fill the new data, transform points
        oldParticleData.set(positions, numPoints, rigidTransform, pointWidth, start, oldSize);

        {
            // set the new data
            SdfChangeBlock changeBlock;
            oldParticleData.write();
            oldParticleData.writeInstancerProtoRadius(0.5f * pointWidth);
        } // changeblock

        // update the member fields.
        mStartIndex = (uint32_t)start;
        mParticleCount = (uint32_t)numPoints;
        mRigidTransform = rigidTransform;
        mShearScaleTransform = shearScaleTransform;

        // communicate to other samplers.
        int correction = (int)numPoints - (int)currentPoints;
        firstChangedIndex = start;
        shiftValue = correction;

        CARB_LOG_INFO("%zu particles sampled\n", numPoints);

        // hide the source mesh
        UsdGeomImageable img(prim);
        img.MakeInvisible();

        // AD hack to make usd updates appear and silence warnings about inconsistent primvars.
        particlePrim.CreateAttribute(TfToken("omni:rtx:skip"), SdfValueTypeNames->Bool).Set(false);

        // return whether or not topology changed.
        return (newNumPoints != oldSize);
    }
    return false;
}

void PhysxParticleSampler::moveStartIndex(int firstChangedIndex, int correction)
{
    if ((int)mStartIndex > firstChangedIndex)
    {
        int correctedStart = (int)mStartIndex + correction;
        mStartIndex = (uint32_t)correctedStart;
    }
}

bool PhysxParticleSampler::getDecomposedTransform(GfMatrix4d& rigidTransform, GfMatrix3d& shearScaleTransform) const
{
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    if (!stage)
        return false;

    const UsdPrim prim = stage->GetPrimAtPath(mSamplerPath);
    UsdGeomMesh geomMesh(prim);
    if (!geomMesh)
        return false;

    // decompose local to world into rigid and shear/scale transforms
    GfMatrix4d l2w = geomMesh.ComputeLocalToWorldTransform(UsdTimeCode::Default());
    GfMatrix4d l2w_r = l2w.RemoveScaleShear();
    GfMatrix4d l2w_ss = l2w * l2w_r.GetInverse();

    // compactify shear/scale transform and store
    GfMatrix3d l2w_ss_compact(l2w_ss[0][0], l2w_ss[0][1], l2w_ss[0][2],
                              l2w_ss[1][0], l2w_ss[1][1], l2w_ss[1][2],
                              l2w_ss[2][0], l2w_ss[2][1], l2w_ss[2][2]);

    rigidTransform = l2w_r;
    shearScaleTransform = l2w_ss_compact;
    return true;
}

bool PhysxParticleSampler::checkTransforms(bool& resample, GfMatrix4d& newRigidTransform)
{
    GfMatrix3d newShearScaleTransform;
    if (!getDecomposedTransform(newRigidTransform, newShearScaleTransform))
    {
        return false;
    }

    const bool rigidChange = !GfIsClose(mRigidTransform, newRigidTransform, 1e-5);
    const bool shearScaleChange = !GfIsClose(mShearScaleTransform, newShearScaleTransform, 1e-5);

    resample = shearScaleChange;
    return rigidChange || shearScaleChange;
}

void PhysxParticleSampler::transformPoints(GfMatrix4d& newRigidTransform)
{
    UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
    const UsdPrim particlePrim = stage->GetPrimAtPath(mTarget);
    if (particlePrim)
    {
        ParticleData particleData(particlePrim);
        particleData.readPoints();

        GfMatrix4d oldRigidTransformInv = mRigidTransform.GetInverse();
        GfMatrix4d transform = oldRigidTransformInv * newRigidTransform;
        particleData.transformPoints(transform, mStartIndex, mParticleCount);
        particleData.writePoints();

        // save the new transform
        mRigidTransform = newRigidTransform;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// IPhysxParticlesPrivate API
namespace omni
{
namespace physx
{
namespace particles
{

void createParticleSampler(const SdfPath samplerPath, const SdfPath particlePrimPath)
{
    PhysxParticleFactory* factory = getParticleFactory(particlePrimPath);
    if (!factory)
    {
        factory = ICE_NEW(PhysxParticleFactory)(particlePrimPath);
        gParticlePrimsToFactoryMap.insert({particlePrimPath, factory});
    }

    if (!factory->getParticleSampler(samplerPath))
        factory->addSampler(samplerPath);
}

void updateParticleSampler(const SdfPath path, const SdfPath particlePrimPath, bool forceResampling)
{
    // AD: we currently treat the tuple (particlePrim,sampler) as a whole - so if the particle path
    // changes, we destroy the sampler object and recreate it in the new factory. We cannot transfer
    // any state between the factories anyway.

    PhysxParticleFactory* factory = getParticleFactory(particlePrimPath);
    if (factory)
    {
        if (!factory->updateSampler(path, forceResampling))
            removeParticleSampler(path, particlePrimPath);
    }
}

void removeParticleSampler(const SdfPath path, SdfPath particlePrimPath)
{
    PhysxParticleFactory* factory = nullptr;
    if (particlePrimPath != SdfPath())
    {
        factory = getParticleFactory(particlePrimPath);
    } 
    else
    {
        // AD this loop is needed because if we remove the particles relationship from a sampler
        // the second argument of the parent function will be an empty path, but we still need to
        // clean up the factory etc. We just search for this right now.
        for (auto it: gParticlePrimsToFactoryMap)
        {
            PhysxParticleSampler* sampler = it.second->getParticleSampler(path);
            if (sampler)
            {
                factory = it.second;
                particlePrimPath = it.first;
                break;
            }
        }
    }

    if (factory)
    {
        factory->removeSampler(path);
        
        // destroy factory if the samplers set is empty
        if (factory->empty())
        {
            gParticlePrimsToFactoryMap.erase(particlePrimPath);
            SAFE_DELETE_SINGLE(factory);
        }
    }
}

}
}
}
