// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXParticleSampling.h"

#include <OmniPhysX.h>
#include <CookingDataAsync.h>
#include "usdLoad/LoadUsd.h"
#include "usdLoad/IceDescriptorAllocator.h"
#include <PhysXTools.h>

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>
#include <omni/physics/parse/IParseBackend.h>
#include <omni/physics/parse/IPhysicsSource.h>

#include <common/utilities/MemoryMacros.h>

#include <limits>

// only for logging
#include <carb/PluginUtils.h>

using namespace PXR_NS;
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

using AttachedStage = omni::physx::usdparser::AttachedStage;
using IPhysicsSource = omni::physics::parse::IPhysicsSource;
using ObjectKey = omni::physics::parse::ObjectKey;

AttachedStage* getActiveAttachedStage()
{
    return omni::physx::usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
}

bool activeSourceUsesExternalPayload()
{
    const AttachedStage* attachedStage = getActiveAttachedStage();
    if (!attachedStage)
        return false;

    const omni::physics::parse::AttachTarget target = attachedStage->attachTarget();
    return target.nativeStage && target.stageId == 0;
}

size_t getSourceArraySize(const IPhysicsSource& src, ObjectKey key, const char* attrName, size_t elementSize)
{
    const omni::physics::parse::BufferHandle buffer =
        src.getArrayAttribute(key, src.internToken(attrName), omni::physics::parse::ReadTime::defaultTime());
    size_t byteCount = 0;
    const void* ptr = src.resolveBuffer(buffer, byteCount);
    const size_t count = (ptr && elementSize > 0) ? byteCount / elementSize : 0;
    src.releaseBuffer(buffer);
    return count;
}

bool getSourceParticleData(AttachedStage& attachedStage, const SdfPath& particlePath, size_t& pointCount, bool& sizesMatch)
{
    const IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;

    const ObjectKey particleKey = attachedStage.keyFor(particlePath);
    if (!src->exists(particleKey) ||
        !omni::physx::internal::hasAppliedSchema<PhysxSchemaPhysxParticleSetAPI>(*src, particleKey))
    {
        return false;
    }

    const bool isPoints = omni::physx::internal::isAType<UsdGeomPoints>(*src, particleKey);
    const bool isInstancer = omni::physx::internal::isAType<UsdGeomPointInstancer>(*src, particleKey);
    if (!isPoints && !isInstancer)
        return false;

    pointCount = getSourceArraySize(*src, particleKey, isInstancer ? "positions" : "points", sizeof(GfVec3f));
    sizesMatch = getSourceArraySize(*src, particleKey, "velocities", sizeof(GfVec3f)) == pointCount;

    if (src->hasAuthoredAttribute(particleKey, src->internToken("physxParticle:simulationPoints")))
        sizesMatch &= getSourceArraySize(*src, particleKey, "physxParticle:simulationPoints", sizeof(GfVec3f)) == pointCount;

    if (isPoints)
    {
        sizesMatch &= getSourceArraySize(*src, particleKey, "widths", sizeof(float)) == pointCount;
    }
    else
    {
        sizesMatch &= getSourceArraySize(*src, particleKey, "protoIndices", sizeof(int)) == pointCount;
        sizesMatch &= getSourceArraySize(*src, particleKey, "scales", sizeof(GfVec3f)) == pointCount;
        sizesMatch &= getSourceArraySize(*src, particleKey, "orientations", sizeof(GfQuath)) == pointCount;
    }

    return true;
}

bool parseSamplingDesc(AttachedStage& attachedStage,
                       const SdfPath& samplerPath,
                       omni::physx::usdparser::ParticleSamplingDesc& samplingDesc)
{
    const IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;

    const ObjectKey samplerKey = attachedStage.keyFor(samplerPath);
    if (!src->exists(samplerKey) || !omni::physx::internal::isAType<UsdGeomMesh>(*src, samplerKey) ||
        !omni::physx::internal::hasAppliedSchema<PhysxSchemaPhysxParticleSamplingAPI>(*src, samplerKey))
    {
        return false;
    }

    omni::physics::parse::ParseContext ctx(
        const_cast<IPhysicsSource&>(*src), omni::physx::usdparser::iceDescriptorAllocator());
    omni::physics::parse::DescPtr<omni::physics::parse::ParticleSamplingDesc> desc =
        omni::physics::parse::parseParticleSampling(ctx, samplerKey);
    if (!desc)
        return false;

    samplingDesc.samplingDistance = desc->samplingDistance;
    samplingDesc.sampleVolume = desc->sampleVolume;
    samplingDesc.maxSamples = desc->maxSamples;
    samplingDesc.pointWidth = desc->pointWidth;
    samplingDesc.particleSetPath = desc->particleSetKey.valid() ? attachedStage.pathFor(desc->particleSetKey) : SdfPath();
    return true;
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
            // particleAPI is only needed to read/write simulation points
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
            GfVec3f position = PXR_NS::GfVec3f(rigidTransform.Transform(positions[p]));
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
                points[i] = PXR_NS::GfVec3f(transform.Transform(points[i]));

            if (particleAPI)
            {
                for (size_t i = dstIndex; i < dstIndex + count; ++i)
                    simPoints[i] = PXR_NS::GfVec3f(transform.Transform(simPoints[i]));
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
    mHasAuthoredSamplingResults(false),
    mTotalParticleCount(0)
{ 

}

PhysxParticleFactory::~PhysxParticleFactory()
{
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
        UsdStageWeakPtr stage = omni::physx::usdparser::UsdLoad::getUsdLoad()->getActiveStage();
        if (stage)
        {
            UsdPrim particlePrim = stage->GetPrimAtPath(samplerIterator->second->getTarget());

            if (particlePrim)
            {
                ParticleData particleData(particlePrim);
                particleData.read();

                if (particleData.points.empty() || particleData.velocities.empty())
                    return;

                ParticleData newParticleData(particlePrim);
                uint32_t copyDst = 0;

                // recreate the particle prim from the remaining (non-removed) samplers
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

                // hack to make USD updates work and silence warnings about inconsistent primvars - remove from hydra db
                particlePrim.CreateAttribute(TfToken("omni:rtx:skip"), SdfValueTypeNames->Bool).Set(true);

                SdfChangeBlock changeBlock;
                {
                    newParticleData.write();
                } // changeblock.

                mTotalParticleCount = newParticleData.points.size();

                // hack to make usd updates appear and silence warnings about inconsistent primvars.
                particlePrim.CreateAttribute(TfToken("omni:rtx:skip"), SdfValueTypeNames->Bool).Set(false);
            }

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

    const bool registerExistingSamples = registerOriginalCount && canRegisterOriginalCount(numPoints);

    // check whether total count of the target prim and array lengths are still consistent
    bool consistent = checkTargetCountsAndSizes(registerExistingSamples);

    if (!consistent)
    {
        CARB_LOG_WARN("%s: Physx particle sampling - target particle prim has a different point count than expected! Trying to recover..", mPath.GetText());

        // reset start/count for all samplers of this factory to force recreation; other samplers need to be resampled first
        resetStartAndCountForAllSamplers();
    }

    if (registerExistingSamples)
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
        const bool wroteSamplingResults = sampler->processSamplingResults(positions, numPoints, pointWidth,
                                                                          rigidTransform, shearScaleTransform,
                                                                          mInitialized, !consistent,
                                                                          firstChangedIndex, shiftValue,
                                                                          topologyChange);

        if (wroteSamplingResults)
        {
            mHasAuthoredSamplingResults = true;

            // update the other samplers sampling to that prim to make sure we can resample them again.
            if (topologyChange)
            {
                moveStartIndices(firstChangedIndex, shiftValue);
                if (activeSourceUsesExternalPayload())
                    applyTotalCountDelta(shiftValue);
                else
                    saveTotalCount();
            }
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

void PhysxParticleFactory::applyTotalCountDelta(int shiftValue)
{
    if (shiftValue < 0)
    {
        const size_t removed = static_cast<size_t>(-shiftValue);
        mTotalParticleCount = (removed < mTotalParticleCount) ? (mTotalParticleCount - removed) : 0;
    }
    else
    {
        mTotalParticleCount += static_cast<size_t>(shiftValue);
    }
}

bool PhysxParticleFactory::canRegisterOriginalCount(size_t numPoints)
{
    AttachedStage* attachedStage = getActiveAttachedStage();
    if (!attachedStage)
        return false;

    size_t pointCount = 0;
    bool sizesMatch = false;
    if (!getSourceParticleData(*attachedStage, mPath, pointCount, sizesMatch) || !sizesMatch)
        return false;

    if (numPoints > std::numeric_limits<size_t>::max() - mTotalParticleCount)
        return false;

    return pointCount >= mTotalParticleCount + numPoints;
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
    AttachedStage* attachedStage = getActiveAttachedStage();
    if (!attachedStage)
        return false;

    size_t pointCount = 0;
    bool sizesMatch = false;
    if (!getSourceParticleData(*attachedStage, mPath, pointCount, sizesMatch))
        return false;

    if (activeSourceUsesExternalPayload() && mHasAuthoredSamplingResults)
        return sizesMatch;

    const bool countsMatch = (pointCount == mTotalParticleCount);
    return ((countsMatch || registerOriginalCount) && sizesMatch);
}

void PhysxParticleFactory::resetStartAndCountForAllSamplers()
{
    for (auto it: mSamplers)
    {
        it.second->setStartIndex(0);
        it.second->setParticleCount(0);
    }

    mHasAuthoredSamplingResults = false;
    mTotalParticleCount = 0;
}

void PhysxParticleFactory::saveTotalCount()
{
    AttachedStage* attachedStage = getActiveAttachedStage();
    if (attachedStage)
    {
        size_t pointCount = 0;
        bool sizesMatch = false;
        if (getSourceParticleData(*attachedStage, mPath, pointCount, sizesMatch))
        {
            mTotalParticleCount = pointCount;
            return;
        }
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
        UsdStageWeakPtr stage = omni::physx::usdparser::UsdLoad::getUsdLoad()->getActiveStage();
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
    AttachedStage* attachedStage = getActiveAttachedStage();
    if (!attachedStage)
        return false;

    omni::physx::usdparser::ParticleSamplingDesc samplingDesc{};
    if (!parseSamplingDesc(*attachedStage, mSamplerPath, samplingDesc))
        return false;

    if (!samplingDesc.particleSetPath.IsEmpty() && samplingDesc.particleSetPath != mTarget)
        return false;

    bool needToResample;
    GfMatrix4d newRigidTransform;
    bool needToTransform = checkTransforms(needToResample, newRigidTransform);

    bool result = true;
    if ((needToTransform && needToResample) || forceResampling)
    {
        cookingdataasync::CookingDataAsync* cookingDataAsync = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
        if (cookingDataAsync)
        {
            cookingDataAsync->poissonSampleMesh(
                attachedStage->keyFor(mSamplerPath), *attachedStage, samplingDesc, forceResampling, true);
        }
        else
        {
            result = false;
        }
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
    mStartIndex = (uint32_t)totalRegisteredPoints;
    mParticleCount = (uint32_t)numPoints;
    mRigidTransform = rigidTransform;
    mShearScaleTransform = shearScaleTransform;

    // hide the source mesh when there is a USD authoring stage.
    UsdStageWeakPtr stage = omni::physx::usdparser::UsdLoad::getUsdLoad()->getActiveStage();
    const UsdPrim prim = stage ? stage->GetPrimAtPath(mSamplerPath) : UsdPrim();
    UsdGeomImageable img(prim);
    if (img)
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
                                                  int& shiftValue,
                                                  bool& topologyChange)
{
    UsdStageWeakPtr stage = omni::physx::usdparser::UsdLoad::getUsdLoad()->getActiveStage();
    const UsdPrim prim = stage ? stage->GetPrimAtPath(mSamplerPath) : UsdPrim();
    const UsdPrim particlePrim = stage ? stage->GetPrimAtPath(mTarget) : UsdPrim();

    if (particlePrim)
    {
        // hack to make USD updates work and silence warnings about inconsistent primvars - remove from hydra db
        particlePrim.CreateAttribute(TfToken("omni:rtx:skip"), SdfValueTypeNames->Bool).Set(true);

        ParticleData oldParticleData(particlePrim);
        if (!oldParticleData.geomPoints && !oldParticleData.geomInstancer)
        {
            CARB_LOG_WARN("%s: Physx particle sampling - target particle prim is not UsdGeomPoints or UsdGeomPointInstancer (type=%s).",
                          mTarget.GetText(), particlePrim.GetTypeName().GetText());
            return false;
        }
        oldParticleData.read();

        // counts are checked to match outside of this function.
        size_t oldSize = oldParticleData.points.size();

        // an inconsistent particle prim forces recreation.
        if (recreate)
            oldSize = 0;

        uint32_t start = mStartIndex;
        uint32_t currentPoints = mParticleCount;

        // legacy: unclear if the first condition below is still needed; the second handles full recreate ops.
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

        const bool pointRangeValid = start <= oldParticleData.points.size() &&
                                     numPoints <= oldParticleData.points.size() - start;
        const bool velocityRangeValid = start <= oldParticleData.velocities.size() &&
                                        numPoints <= oldParticleData.velocities.size() - start;
        const bool widthRangeValid = !oldParticleData.geomPoints ||
                                     (start <= oldParticleData.pointsWidths.size() &&
                                      numPoints <= oldParticleData.pointsWidths.size() - start);
        if (!pointRangeValid || !velocityRangeValid || !widthRangeValid)
        {
            CARB_LOG_WARN("%s: Physx particle sampling - target arrays are not large enough after resize (start=%u, samples=%zu, points=%zu, velocities=%zu, widths=%zu, new=%zu, old=%zu, current=%u).",
                          mTarget.GetText(), start, numPoints, oldParticleData.points.size(),
                          oldParticleData.velocities.size(), oldParticleData.pointsWidths.size(),
                          newNumPoints, oldSize, currentPoints);
            return false;
        }

        oldParticleData.set(positions, numPoints, rigidTransform, pointWidth, start, oldSize);

        {
            SdfChangeBlock changeBlock;
            oldParticleData.write();
            oldParticleData.writeInstancerProtoRadius(0.5f * pointWidth);
        } // changeblock

        mStartIndex = (uint32_t)start;
        mParticleCount = (uint32_t)numPoints;
        mRigidTransform = rigidTransform;
        mShearScaleTransform = shearScaleTransform;

        // report the changed range so other samplers can shift their start indices
        int correction = (int)numPoints - (int)currentPoints;
        firstChangedIndex = start;
        shiftValue = correction;

        CARB_LOG_INFO("%zu particles sampled\n", numPoints);

        UsdGeomImageable img(prim);
        img.MakeInvisible();

        // hack to make usd updates appear and silence warnings about inconsistent primvars.
        particlePrim.CreateAttribute(TfToken("omni:rtx:skip"), SdfValueTypeNames->Bool).Set(false);

        topologyChange = (newNumPoints != oldSize);
        return true;
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
    const AttachedStage* attachedStage = getActiveAttachedStage();
    const IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr;
    if (!src)
        return false;

    const ObjectKey samplerKey = attachedStage->keyFor(mSamplerPath);
    if (!src->exists(samplerKey) || !omni::physx::internal::isAType<UsdGeomMesh>(*src, samplerKey))
        return false;

    // decompose local to world into rigid and shear/scale transforms
    GfMatrix4d l2w = omni::physx::internal::getWorldTransform(*attachedStage, samplerKey, UsdTimeCode::Default());
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
    UsdStageWeakPtr stage = omni::physx::usdparser::UsdLoad::getUsdLoad()->getActiveStage();
    const UsdPrim particlePrim = stage ? stage->GetPrimAtPath(mTarget) : UsdPrim();
    if (particlePrim)
    {
        ParticleData particleData(particlePrim);
        particleData.readPoints();

        GfMatrix4d oldRigidTransformInv = mRigidTransform.GetInverse();
        GfMatrix4d transform = oldRigidTransformInv * newRigidTransform;
        particleData.transformPoints(transform, mStartIndex, mParticleCount);
        particleData.writePoints();

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
    // The tuple (particlePrim, sampler) is treated as a whole: if the particle path changes, the
    // sampler is destroyed and recreated in the new factory since state can't transfer between factories.

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
        // If the particles relationship was removed from a sampler, particlePrimPath is empty here,
        // so search all factories for the sampler to still clean it up.
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
