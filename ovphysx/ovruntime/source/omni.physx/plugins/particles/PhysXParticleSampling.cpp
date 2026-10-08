// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27
 *
 * @implements REQ-WRITE-AUTHORING-001
 * @covers AC-5
 */

/**
 * @implements REQ-MATH-001
 * @covers AC-9
 */

#include "PhysXParticleSampling.h"

#include <OmniPhysX.h>
#include <CookingDataAsync.h>
#include "usdLoad/LoadUsd.h"
#include "usdLoad/IceDescriptorAllocator.h"
#include <PhysXTools.h>

#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>
#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/IPhysicsDataWrite.h>
#include <omni/physics/parse/KnownTokens.h>


// only for logging
#include <carb/PluginUtils.h>

#include <common/foundation/MatrixTools.h>
#include <common/utilities/MemoryMacros.h>

#include <algorithm>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>

using namespace omni::physx::particles;

namespace
{

typedef std::unordered_map<omni::physics::parse::ObjectKey, PhysxParticleFactory*, omni::physics::parse::ObjectKey::Hash>
    PathToFactoryMap;
PathToFactoryMap gParticlePrimsToFactoryMap;

PhysxParticleFactory* getParticleFactory(const omni::physics::parse::ObjectKey particlePrimKey)
{
    auto it = gParticlePrimsToFactoryMap.find(particlePrimKey);
    return (it != gParticlePrimsToFactoryMap.end()) ? it->second : nullptr;
}

using AttachedStage = omni::physx::usdparser::AttachedStage;
using IPhysicsSource = omni::physics::parse::IPhysicsSource;
using IPhysicsDataWrite = omni::physics::parse::IPhysicsDataWrite;
using ObjectKey = omni::physics::parse::ObjectKey;

// Ad hoc marker names, outside the standard token vocabulary (no getSourceParticleData /
// KnownTokens entry): used only by the USD-only cosmetic islands below.
constexpr const char* kOmniRtxSkipAttrName = "omni:rtx:skip";
constexpr const char* kParticleSamplingCrcAttrName = "physxParticleSampling:crc";

AttachedStage* getActiveAttachedStage()
{
    return omni::physx::usdparser::UsdLoad::getUsdLoad()->getActiveAttachedStage();
}

bool activeSourceUsesExternalPayload()
{
    const AttachedStage* attachedStage = getActiveAttachedStage();
    if (!attachedStage)
        return false;

    return attachedStage->hasExternalSource();
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

bool getSourceParticleData(AttachedStage& attachedStage, ObjectKey particleKey, size_t& pointCount, bool& sizesMatch)
{
    const IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;

    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);
    if (!src->exists(particleKey) || !src->hasSchema(particleKey, tok.physxParticleSetAPI))
    {
        return false;
    }

    const bool isPoints = src->isA(particleKey, tok.pointsType);
    const bool isInstancer = src->isA(particleKey, tok.pointInstancerType);
    if (!isPoints && !isInstancer)
        return false;

    // Element sizes are wire sizes, not Gf types: a point is 3 packed floats
    // (carb::Float3 == GfVec3f) and an instancer orientation is 4 packed halves
    // (GfQuath - HALF precision, deliberately spelled as 4 x uint16_t rather
    // than any carb/PhysX 4-float type, which would be twice as large).
    pointCount = getSourceArraySize(*src, particleKey, isInstancer ? "positions" : "points", sizeof(carb::Float3));
    sizesMatch = getSourceArraySize(*src, particleKey, "velocities", sizeof(carb::Float3)) == pointCount;

    if (src->hasAuthoredAttribute(particleKey, src->internToken("physxParticle:simulationPoints")))
        sizesMatch &= getSourceArraySize(*src, particleKey, "physxParticle:simulationPoints", sizeof(carb::Float3)) == pointCount;

    if (isPoints)
    {
        sizesMatch &= getSourceArraySize(*src, particleKey, "widths", sizeof(float)) == pointCount;
    }
    else
    {
        sizesMatch &= getSourceArraySize(*src, particleKey, "protoIndices", sizeof(int)) == pointCount;
        sizesMatch &= getSourceArraySize(*src, particleKey, "scales", sizeof(carb::Float3)) == pointCount;
        sizesMatch &= getSourceArraySize(*src, particleKey, "orientations", 4 * sizeof(uint16_t)) == pointCount;
    }

    return true;
}

bool parseSamplingDesc(AttachedStage& attachedStage,
                       const ObjectKey samplerKey,
                       omni::physx::usdparser::ParticleSamplingDesc& samplingDesc)
{
    const IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;

    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);
    if (!src->exists(samplerKey) || !src->isA(samplerKey, tok.meshType) ||
        !src->hasSchema(samplerKey, tok.physxParticleSamplingAPI))
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
    samplingDesc.particleSetKey = desc->particleSetKey;
    return true;
}

// Mirrors GfIsClose(GfMatrix{3,4}d, ..., tol), which is elementwise
// (|a-b| <= tol) and therefore invariant under the element-copy transpose.
template <typename MatT>
bool isCloseElementwise(const MatT& a, const MatT& b, double tolerance)
{
    const double* pa = &a.column0.x;
    const double* pb = &b.column0.x;
    constexpr size_t kCount = sizeof(MatT) / sizeof(double);
    for (size_t i = 0; i < kCount; ++i)
    {
        if (!(std::fabs(pb[i] - pa[i]) <= tolerance))
            return false;
    }
    return true;
}

template <typename T>
void moveRange(std::vector<T>& v, size_t dst, size_t src, size_t count)
{
    if (count > 0)
        std::memmove(&v[dst], &v[src], count * sizeof(T));
}

template <typename T>
void copyRange(std::vector<T>& dstV, const std::vector<T>& srcV, size_t dst, size_t src, size_t count)
{
    if (count > 0)
        std::memcpy(&dstV[dst], &srcV[src], count * sizeof(T));
}

// Per-channel particle arrays, read via IPhysicsSource and written via
// IPhysicsDataWrite::writeArray. Orientations are xyzw float quaternions (PxQuat convention,
// matching writeArray's documented layout); the wire format is 4 packed halves, but the
// half<->float conversion on both sides is exact, so carrying floats here costs no precision.
struct ParticleArrays
{
    bool isPoints = false;
    bool isInstancer = false;
    bool hasSimPoints = false;

    std::vector<carb::Float3> points;
    std::vector<carb::Float3> velocities;
    std::vector<carb::Float3> simPoints;
    std::vector<float> widths; // geomPoints only
    std::vector<int32_t> protoIndices; // instancer only
    std::vector<carb::Float3> scales; // instancer only
    std::vector<carb::Float4> orientations; // instancer only, xyzw (PxQuat convention)

    size_t size() const
    {
        return points.size();
    }

    void resize(size_t newSize)
    {
        if (hasSimPoints)
            simPoints.resize(newSize);

        if (isPoints)
        {
            points.resize(newSize);
            velocities.resize(newSize);
            widths.resize(newSize);
        }
        else if (isInstancer)
        {
            points.resize(newSize);
            velocities.resize(newSize);
            protoIndices.resize(newSize);
            scales.resize(newSize);

            // VtArray<GfQuath>::resize() default-constructs new elements to GfQuath's
            // identity, unlike a plain std::vector<carb::Float4> (which zero-inits) --
            // reproduce that explicitly for the newly grown tail.
            const size_t oldSize = orientations.size();
            orientations.resize(newSize);
            for (size_t i = oldSize; i < newSize; ++i)
                orientations[i] = carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f };
        }
    }

    void move(size_t dst, size_t src, size_t count)
    {
        if (count == 0)
            return;

        if (hasSimPoints)
            moveRange(simPoints, dst, src, count);

        if (isPoints)
        {
            moveRange(points, dst, src, count);
            moveRange(velocities, dst, src, count);
            moveRange(widths, dst, src, count);
        }
        else if (isInstancer)
        {
            moveRange(points, dst, src, count);
            moveRange(velocities, dst, src, count);
            moveRange(protoIndices, dst, src, count);
            moveRange(scales, dst, src, count);
            moveRange(orientations, dst, src, count);
        }
    }

    void copyFrom(const ParticleArrays& srcArrays, size_t dst, size_t src, size_t count)
    {
        if (count == 0)
            return;

        if (srcArrays.hasSimPoints && hasSimPoints)
            copyRange(simPoints, srcArrays.simPoints, dst, src, count);

        if (srcArrays.isPoints && isPoints)
        {
            copyRange(points, srcArrays.points, dst, src, count);
            copyRange(velocities, srcArrays.velocities, dst, src, count);
            copyRange(widths, srcArrays.widths, dst, src, count);
        }
        else if (srcArrays.isInstancer && isInstancer)
        {
            copyRange(points, srcArrays.points, dst, src, count);
            copyRange(velocities, srcArrays.velocities, dst, src, count);
            copyRange(protoIndices, srcArrays.protoIndices, dst, src, count);
            copyRange(scales, srcArrays.scales, dst, src, count);
            copyRange(orientations, srcArrays.orientations, dst, src, count);
        }
    }

    void set(const carb::Float3* positions, size_t numPoints, const ::physx::PxMat44d& rigidTransform,
             float pointWidth, size_t dstIndex, size_t initializedEndIndex)
    {
        for (size_t p = 0; p < numPoints; ++p)
        {
            // Same arithmetic as the previous GfMatrix4d::Transform(GfVec3f): widen
            // to double, accumulate column-by-column in the same order, narrow back.
            const carb::Float3& s = positions[p];
            const ::physx::PxVec3d t =
                rigidTransform.transform(::physx::PxVec3d(double(s.x), double(s.y), double(s.z)));
            points[dstIndex + p] = carb::Float3{ float(t.x), float(t.y), float(t.z) };
        }

        if (hasSimPoints)
            copyRange(simPoints, points, dstIndex, dstIndex, numPoints);

        for (size_t i = dstIndex; i < dstIndex + numPoints; ++i)
        {
            velocities[i] = carb::Float3{ 0.0f, 0.0f, 0.0f };
            if (isPoints)
            {
                widths[i] = pointWidth;
            }
            else if (isInstancer)
            {
                protoIndices[i] = 0;
                if (i >= initializedEndIndex)
                {
                    scales[i] = carb::Float3{ 1.0f, 1.0f, 1.0f };
                    orientations[i] = carb::Float4{ 0.0f, 0.0f, 0.0f, 1.0f };
                }
            }
        }
    }

    void transformPoints(const ::physx::PxMat44d& transform, size_t dstIndex, size_t count)
    {
        auto xf = [&transform](const carb::Float3& v)
        {
            const ::physx::PxVec3d t = transform.transform(::physx::PxVec3d(double(v.x), double(v.y), double(v.z)));
            return carb::Float3{ float(t.x), float(t.y), float(t.z) };
        };

        if (dstIndex + count <= points.size())
        {
            for (size_t i = dstIndex; i < dstIndex + count; ++i)
                points[i] = xf(points[i]);

            if (hasSimPoints)
            {
                for (size_t i = dstIndex; i < dstIndex + count; ++i)
                    simPoints[i] = xf(simPoints[i]);
            }
        }
    }
};

// Classify the particle target's concrete geometry type. Pure isA/authored-attribute
// checks, no schema requirement.
void classifyParticleTarget(const IPhysicsSource& src, ObjectKey key, ParticleArrays& out)
{
    omni::physics::parse::KnownTokens tok;
    tok.intern(src);
    out.isPoints = src.isA(key, tok.pointsType);
    out.isInstancer = src.isA(key, tok.pointInstancerType);
    out.hasSimPoints = src.hasAuthoredAttribute(key, src.internToken("physxParticle:simulationPoints"));
}

// Populate every channel `out`'s classification calls for. `out.isPoints`/`isInstancer`/
// `hasSimPoints` must already be set (via classifyParticleTarget).
void readParticleArrayContents(AttachedStage& attachedStage, ObjectKey key, ParticleArrays& out)
{
    const IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return;

    const omni::physics::parse::ReadTime t = omni::physics::parse::ReadTime::defaultTime();
    omni::physx::internal::getArrayValue(attachedStage, key, src->internToken(out.isInstancer ? "positions" : "points"), t, out.points);
    omni::physx::internal::getArrayValue(attachedStage, key, src->internToken("velocities"), t, out.velocities);

    if (out.hasSimPoints)
        omni::physx::internal::getArrayValue(attachedStage, key, src->internToken("physxParticle:simulationPoints"), t, out.simPoints);

    if (out.isPoints)
    {
        omni::physx::internal::getArrayValue(attachedStage, key, src->internToken("widths"), t, out.widths);
    }
    else if (out.isInstancer)
    {
        omni::physx::internal::getArrayValue(attachedStage, key, src->internToken("protoIndices"), t, out.protoIndices);
        omni::physx::internal::getArrayValue(attachedStage, key, src->internToken("scales"), t, out.scales);
        omni::physx::internal::getArrayValue(attachedStage, key, src->internToken("orientations"), t, out.orientations);
    }
}

// Classify + read in one call. Returns false when `key` is neither UsdGeomPoints- nor
// UsdGeomPointInstancer-typed.
bool readParticleArrays(AttachedStage& attachedStage, ObjectKey key, ParticleArrays& out)
{
    const IPhysicsSource* src = attachedStage.getSource();
    if (!src || !src->exists(key))
        return false;

    classifyParticleTarget(*src, key, out);
    if (!out.isPoints && !out.isInstancer)
        return false;

    readParticleArrayContents(attachedStage, key, out);
    return true;
}

// Write every channel present in `arrays` back out through IPhysicsDataWrite::writeArray --
// one call per attribute, full-array replace. Element shape/precision are inferred by the
// sink from the destination attribute (see IPhysicsDataWrite.h).
void writeParticleArrays(IPhysicsDataWrite& dataWrite, const IPhysicsSource& src, ObjectKey key, const ParticleArrays& arrays)
{
    using omni::physics::parse::DataWriteView;

    const size_t n = arrays.size();

    auto writeVec3 = [&](const char* attrName, const std::vector<carb::Float3>& data)
    {
        DataWriteView view;
        view.data = data.empty() ? nullptr : data.data();
        view.count = n;
        dataWrite.writeArray(key, src.internToken(attrName), view);
    };

    writeVec3(arrays.isInstancer ? "positions" : "points", arrays.points);
    writeVec3("velocities", arrays.velocities);

    if (arrays.hasSimPoints)
        writeVec3("physxParticle:simulationPoints", arrays.simPoints);

    if (arrays.isPoints)
    {
        DataWriteView view;
        view.data = arrays.widths.empty() ? nullptr : arrays.widths.data();
        view.count = n;
        dataWrite.writeArray(key, src.internToken("widths"), view);
    }
    else if (arrays.isInstancer)
    {
        DataWriteView protoView;
        protoView.data = arrays.protoIndices.empty() ? nullptr : arrays.protoIndices.data();
        protoView.count = n;
        dataWrite.writeArray(key, src.internToken("protoIndices"), protoView);

        writeVec3("scales", arrays.scales);

        DataWriteView orientView;
        orientView.data = arrays.orientations.empty() ? nullptr : arrays.orientations.data();
        orientView.count = n;
        dataWrite.writeArray(key, src.internToken("orientations"), orientView);
    }
}

// The three genuinely USD-only cosmetic side effects this file needs, each behind one
// pxr-free call. No-op whenever there is no active USD sink or the prim doesn't exist.
void hideHydraDuringRewrite(IPhysicsDataWrite* dataWrite, ObjectKey key, bool hidden)
{
    if (dataWrite)
        dataWrite->writeBoolAttribute(key, kOmniRtxSkipAttrName, hidden);
}

void setPrimVisible(IPhysicsDataWrite* dataWrite, ObjectKey key, bool visible)
{
    if (dataWrite)
        dataWrite->writeVisibility(key, visible);
}

void removeSamplingCrc(IPhysicsDataWrite* dataWrite, ObjectKey key)
{
    if (dataWrite)
        dataWrite->removeAttribute(key, kParticleSamplingCrcAttrName);
}

void setInstancerProtoRadius(IPhysicsDataWrite* dataWrite, ObjectKey key, bool isInstancer, float radius)
{
    if (!isInstancer || !dataWrite)
        return;
    dataWrite->writeInstancerProtoRadius(key, radius);
}

} // namespace

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PhysxParticleFactory::PhysxParticleFactory(ObjectKey particleKey) :
    mParticleKey(particleKey),
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

void PhysxParticleFactory::addSampler(ObjectKey samplerKey)
{
    PhysxParticleSampler* sampler = ICE_NEW(PhysxParticleSampler)(samplerKey, mParticleKey);
    mSamplers[samplerKey] = sampler;
}

bool PhysxParticleFactory::updateSampler(ObjectKey samplerKey, bool forceResampling)
{
    PhysxParticleSampler* sampler = getParticleSampler(samplerKey);
    if (sampler)
    {
        return sampler->update(forceResampling);
    }
    return false;
}

void PhysxParticleFactory::removeSampler(ObjectKey samplerKey)
{
    // we need to do this on the factory level to make sure the data stays consistent.
    PathToSamplerMap::const_iterator samplerIterator = mSamplers.find(samplerKey);
    if (samplerIterator != mSamplers.end())
    {
        PhysxParticleSampler* samplerPointer = samplerIterator->second;
        AttachedStage* attachedStage = getActiveAttachedStage();
        if (attachedStage)
        {
            const IPhysicsSource* src = attachedStage->getSource();
            IPhysicsDataWrite* dataWrite = attachedStage->getAuthoringDataWrite();

            if (src && src->exists(mParticleKey))
            {
                ParticleArrays oldArrays;
                readParticleArrays(*attachedStage, mParticleKey, oldArrays);

                if (oldArrays.points.empty() || oldArrays.velocities.empty())
                    return;

                ParticleArrays newArrays;
                newArrays.isPoints = oldArrays.isPoints;
                newArrays.isInstancer = oldArrays.isInstancer;
                newArrays.hasSimPoints = oldArrays.hasSimPoints;
                uint32_t copyDst = 0;

                // recreate the particle prim from the remaining (non-removed) samplers.
                // Deterministic order (previously a std::map<SdfPath,...>'s path order):
                // sort the remaining samplers by their source path text. (Plain
                // pair<ObjectKey,...>, not PathToSamplerMap::value_type: the map's own
                // pair<const ObjectKey,...> is not assignable, so it can't be std::sort'ed.)
                std::vector<std::pair<ObjectKey, PhysxParticleSampler*>> remaining;
                remaining.reserve(mSamplers.size());
                for (const PathToSamplerMap::value_type& entry : mSamplers)
                {
                    if (entry.first != samplerKey)
                        remaining.push_back(entry);
                }
                std::sort(remaining.begin(), remaining.end(),
                          [attachedStage](const std::pair<ObjectKey, PhysxParticleSampler*>& a,
                                          const std::pair<ObjectKey, PhysxParticleSampler*>& b)
                          { return attachedStage->textViewFor(a.first) < attachedStage->textViewFor(b.first); });

                for (const std::pair<ObjectKey, PhysxParticleSampler*>& entry : remaining)
                {
                    const uint32_t copySrc = entry.second->getStartIndex();
                    const uint32_t copyCount = entry.second->getParticleCount();

                    newArrays.resize(copyDst + copyCount);
                    newArrays.copyFrom(oldArrays, copyDst, copySrc, copyCount);

                    entry.second->setStartIndex(copyDst);
                    copyDst += copyCount;
                }

                if (dataWrite)
                {
                    // hack to make USD updates work and silence warnings about inconsistent
                    // primvars - remove from hydra db
                    hideHydraDuringRewrite(dataWrite, mParticleKey, true);

                    dataWrite->beginWrite();
                    writeParticleArrays(*dataWrite, *src, mParticleKey, newArrays);
                    dataWrite->endWrite();

                    mTotalParticleCount = newArrays.size();

                    // hack to make usd updates appear and silence warnings about inconsistent primvars.
                    hideHydraDuringRewrite(dataWrite, mParticleKey, false);
                }
            }

            if (src && src->exists(samplerKey))
            {
                removeSamplingCrc(dataWrite, samplerKey);
                setPrimVisible(dataWrite, samplerKey, true);
            }
        }

        // cleanup the lists
        SAFE_DELETE_SINGLE(samplerPointer);
        mSamplers.erase(samplerKey);
    }
}

PhysxParticleSampler* PhysxParticleFactory::getParticleSampler(ObjectKey samplerKey)
{
    PathToSamplerMap::const_iterator it = mSamplers.find(samplerKey);
    return (it != mSamplers.end()) ? it->second : nullptr;
}

void PhysxParticleFactory::processParticleSamplingResults(ObjectKey samplerKey,
                                                          const carb::Float3* positions,
                                                          size_t numPoints,
                                                          float pointWidth,
                                                          const ::physx::PxMat44d& rigidTransform,
                                                          const ::physx::PxMat33d& shearScaleTransform,
                                                          bool registerOriginalCount)
{
    PhysxParticleSampler* sampler = getParticleSampler(samplerKey);

    if (!sampler)
        return;

    const bool registerExistingSamples = registerOriginalCount && canRegisterOriginalCount(numPoints);

    // check whether total count of the target prim and array lengths are still consistent
    bool consistent = checkTargetCountsAndSizes(registerExistingSamples);

    if (!consistent)
    {
        AttachedStage* attachedStage = getActiveAttachedStage();
        CARB_LOG_WARN("%s: Physx particle sampling - target particle prim has a different point count than expected! Trying to recover..",
                     attachedStage ? attachedStage->textFor(mParticleKey) : "");

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
    if (!getSourceParticleData(*attachedStage, mParticleKey, pointCount, sizesMatch) || !sizesMatch)
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
    if (!getSourceParticleData(*attachedStage, mParticleKey, pointCount, sizesMatch))
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
        if (getSourceParticleData(*attachedStage, mParticleKey, pointCount, sizesMatch))
        {
            mTotalParticleCount = pointCount;
            return;
        }
    }

    // sanity exit if prim does not exist.
    mTotalParticleCount = 0;
}

// STATIC:
void PhysxParticleFactory::processSamplingResults(ObjectKey samplerKey,
                                                  ObjectKey particleSetKey,
                                                  const carb::Float3* positions,
                                                  size_t numPoints,
                                                  float pointWidth,
                                                  const ::physx::PxMat44d& rigidTransform,
                                                  const ::physx::PxMat33d& shearScaleTransform,
                                                  bool registerOriginalCount)
{
    PhysxParticleFactory* factory = getParticleFactory(particleSetKey);
    if (factory)
    {
        factory->processParticleSamplingResults(samplerKey, positions, numPoints, pointWidth,
                                                rigidTransform, shearScaleTransform, registerOriginalCount);
    }
    else
    {
        // factory doesn't exist - means API has been removed etc.
        // we remove the crc + reset the visibility.
        AttachedStage* attachedStage = getActiveAttachedStage();
        if (attachedStage)
        {
            IPhysicsDataWrite* dataWrite = attachedStage->getAuthoringDataWrite();
            removeSamplingCrc(dataWrite, samplerKey);
            setPrimVisible(dataWrite, samplerKey, true);
        }
    }
}

// STATIC:
bool PhysxParticleFactory::getDecomposedTransform(ObjectKey samplerKey,
                                                  ObjectKey particleSetKey,
                                                  ::physx::PxMat44d& rigidTransform,
                                                  ::physx::PxMat33d& shearScaleTransform)
{
    PhysxParticleFactory* factory = getParticleFactory(particleSetKey);
    if (!factory)
        return false;

    PhysxParticleSampler* sampler = factory->getParticleSampler(samplerKey);
    if (!sampler)
        return false;

    return sampler->getDecomposedTransform(rigidTransform, shearScaleTransform);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PhysxParticleSampler::PhysxParticleSampler(ObjectKey key, ObjectKey target) :
    mSamplerKey(key),
    mTargetKey(target),
    mStartIndex(0),
    mParticleCount(0)
{
    // PxMat44d/PxMat33d default-construct UNINITIALIZED, unlike GfMatrix4d/GfMatrix3d
    // (whose default ctor is the identity) - so seed both before the decompose attempt,
    // not only on its failure path.
    mRigidTransform = ::physx::PxMat44d(::physx::PxIdentity);
    mShearScaleTransform = ::physx::PxMat33d(::physx::PxIdentity);
    if (!getDecomposedTransform(mRigidTransform, mShearScaleTransform))
    {
        mRigidTransform = ::physx::PxMat44d(::physx::PxIdentity);
        mShearScaleTransform = ::physx::PxMat33d(::physx::PxIdentity);
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
    if (!parseSamplingDesc(*attachedStage, mSamplerKey, samplingDesc))
        return false;

    if (samplingDesc.particleSetKey.valid() && samplingDesc.particleSetKey != mTargetKey)
        return false;

    bool needToResample;
    ::physx::PxMat44d newRigidTransform(::physx::PxIdentity);
    bool needToTransform = checkTransforms(needToResample, newRigidTransform);

    bool result = true;
    if ((needToTransform && needToResample) || forceResampling)
    {
        cookingdataasync::CookingDataAsync* cookingDataAsync = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
        if (cookingDataAsync)
        {
            cookingDataAsync->poissonSampleMesh(mSamplerKey, *attachedStage, samplingDesc, forceResampling, true);
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
                                                       const ::physx::PxMat44d& rigidTransform,
                                                       const ::physx::PxMat33d& shearScaleTransform)
{
    mStartIndex = (uint32_t)totalRegisteredPoints;
    mParticleCount = (uint32_t)numPoints;
    mRigidTransform = rigidTransform;
    mShearScaleTransform = shearScaleTransform;

    // hide the source mesh when there is a USD authoring stage.
    AttachedStage* attachedStage = getActiveAttachedStage();
    if (attachedStage)
    {
        const IPhysicsSource* src = attachedStage->getSource();
        if (src && src->exists(mSamplerKey))
            setPrimVisible(attachedStage->getAuthoringDataWrite(), mSamplerKey, false);
    }

    return true;
}

bool PhysxParticleSampler::processSamplingResults(const carb::Float3* positions,
                                                  size_t numPoints,
                                                  float pointWidth,
                                                  const ::physx::PxMat44d& rigidTransform,
                                                  const ::physx::PxMat33d& shearScaleTransform,
                                                  bool factoryInitialized,
                                                  bool recreate,
                                                  int& firstChangedIndex,
                                                  int& shiftValue,
                                                  bool& topologyChange)
{
    AttachedStage* attachedStage = getActiveAttachedStage();
    const IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr;

    if (src && src->exists(mTargetKey))
    {
        IPhysicsDataWrite* dataWrite = attachedStage->getAuthoringDataWrite();

        // hack to make USD updates work and silence warnings about inconsistent primvars - remove from hydra db
        hideHydraDuringRewrite(dataWrite, mTargetKey, true);

        ParticleArrays oldArrays;
        if (!readParticleArrays(*attachedStage, mTargetKey, oldArrays))
        {
            const std::string typeName(src->tokenToString(src->getTypeName(mTargetKey)));
            CARB_LOG_WARN("%s: Physx particle sampling - target particle prim is not UsdGeomPoints or UsdGeomPointInstancer (type=%s).",
                          attachedStage->textFor(mTargetKey), typeName.c_str());
            return false;
        }

        // counts are checked to match outside of this function.
        size_t oldSize = oldArrays.size();

        // an inconsistent particle prim forces recreation.
        if (recreate)
            oldSize = 0;

        uint32_t start = mStartIndex;
        uint32_t currentPoints = mParticleCount;

        // legacy: unclear if the first condition below is still needed; the second handles full recreate ops.
        if ((oldSize > 0 && !factoryInitialized) || recreate)
        {
            ParticleArrays emptyArrays;
            emptyArrays.isPoints = oldArrays.isPoints;
            emptyArrays.isInstancer = oldArrays.isInstancer;
            emptyArrays.hasSimPoints = oldArrays.hasSimPoints;

            if (dataWrite)
            {
                dataWrite->beginWrite();
                writeParticleArrays(*dataWrite, *src, mTargetKey, emptyArrays);
                dataWrite->endWrite();
            }

            oldArrays = emptyArrays;
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
            oldArrays.move(moveDst, moveSrc, moveCount);
            oldArrays.resize(newNumPoints);
        }
        else if (newNumPoints > oldSize)
        {
            //if we grow, we need to resize first
            oldArrays.resize(newNumPoints);
            oldArrays.move(moveDst, moveSrc, moveCount);
        }

        const bool pointRangeValid = start <= oldArrays.points.size() &&
                                     numPoints <= oldArrays.points.size() - start;
        const bool velocityRangeValid = start <= oldArrays.velocities.size() &&
                                        numPoints <= oldArrays.velocities.size() - start;
        const bool widthRangeValid = !oldArrays.isPoints ||
                                     (start <= oldArrays.widths.size() &&
                                      numPoints <= oldArrays.widths.size() - start);
        if (!pointRangeValid || !velocityRangeValid || !widthRangeValid)
        {
            CARB_LOG_WARN("%s: Physx particle sampling - target arrays are not large enough after resize (start=%u, samples=%zu, points=%zu, velocities=%zu, widths=%zu, new=%zu, old=%zu, current=%u).",
                          attachedStage->textFor(mTargetKey), start, numPoints, oldArrays.points.size(),
                          oldArrays.velocities.size(), oldArrays.widths.size(),
                          newNumPoints, oldSize, currentPoints);
            return false;
        }

        oldArrays.set(positions, numPoints, rigidTransform, pointWidth, start, oldSize);

        if (dataWrite)
        {
            dataWrite->beginWrite();
            writeParticleArrays(*dataWrite, *src, mTargetKey, oldArrays);
            dataWrite->endWrite();
        }
        setInstancerProtoRadius(dataWrite, mTargetKey, oldArrays.isInstancer, 0.5f * pointWidth);

        mStartIndex = (uint32_t)start;
        mParticleCount = (uint32_t)numPoints;
        mRigidTransform = rigidTransform;
        mShearScaleTransform = shearScaleTransform;

        // report the changed range so other samplers can shift their start indices
        int correction = (int)numPoints - (int)currentPoints;
        firstChangedIndex = start;
        shiftValue = correction;

        CARB_LOG_INFO("%zu particles sampled\n", numPoints);

        setPrimVisible(dataWrite, mSamplerKey, false);

        // hack to make usd updates appear and silence warnings about inconsistent primvars.
        hideHydraDuringRewrite(dataWrite, mTargetKey, false);

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

bool PhysxParticleSampler::getDecomposedTransform(::physx::PxMat44d& rigidTransform,
                                                  ::physx::PxMat33d& shearScaleTransform) const
{
    const AttachedStage* attachedStage = getActiveAttachedStage();
    const IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr;
    if (!src)
        return false;

    omni::physics::parse::KnownTokens tok;
    tok.intern(*src);
    if (!src->exists(mSamplerKey) || !src->isA(mSamplerKey, tok.meshType))
        return false;

    const ::physx::PxMat44d l2w = omni::physx::internal::getWorldTransform(
        *attachedStage, mSamplerKey, omni::physics::parse::ReadTime::defaultTime());
    return decomposeSamplerTransform(l2w, rigidTransform, shearScaleTransform);
}

bool PhysxParticleSampler::checkTransforms(bool& resample, ::physx::PxMat44d& newRigidTransform)
{
    ::physx::PxMat33d newShearScaleTransform;
    if (!getDecomposedTransform(newRigidTransform, newShearScaleTransform))
    {
        return false;
    }

    const bool rigidChange = !isCloseElementwise(mRigidTransform, newRigidTransform, 1e-5);
    const bool shearScaleChange = !isCloseElementwise(mShearScaleTransform, newShearScaleTransform, 1e-5);

    resample = shearScaleChange;
    return rigidChange || shearScaleChange;
}

void PhysxParticleSampler::transformPoints(::physx::PxMat44d& newRigidTransform)
{
    AttachedStage* attachedStage = getActiveAttachedStage();
    const IPhysicsSource* src = attachedStage ? attachedStage->getSource() : nullptr;
    if (!src || !src->exists(mTargetKey))
        return;

    ParticleArrays arrays;
    if (!readParticleArrays(*attachedStage, mTargetKey, arrays))
        return;

    // Gf `oldRigidTransformInv * newRigidTransform` -- operands swap under the
    // PhysX convention (the element-copy transpose).
    const ::physx::PxMat44d oldRigidTransformInv = omni::physx::affineInverse(mRigidTransform);
    const ::physx::PxMat44d transform = newRigidTransform * oldRigidTransformInv;
    arrays.transformPoints(transform, mStartIndex, mParticleCount);

    IPhysicsDataWrite* dataWrite = attachedStage->getAuthoringDataWrite();
    if (dataWrite)
    {
        using omni::physics::parse::DataWriteView;
        dataWrite->beginWrite();
        DataWriteView pointsView;
        pointsView.data = arrays.points.empty() ? nullptr : arrays.points.data();
        pointsView.count = arrays.points.size();
        dataWrite->writeArray(mTargetKey, src->internToken(arrays.isInstancer ? "positions" : "points"), pointsView);
        if (arrays.hasSimPoints)
        {
            DataWriteView simView;
            simView.data = arrays.simPoints.empty() ? nullptr : arrays.simPoints.data();
            simView.count = arrays.simPoints.size();
            dataWrite->writeArray(mTargetKey, src->internToken("physxParticle:simulationPoints"), simView);
        }
        dataWrite->endWrite();
    }

    mRigidTransform = newRigidTransform;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Particle-sampler entry points (internal, ObjectKey-keyed)
namespace omni
{
namespace physx
{
namespace particles
{

// Formerly GF ISLAND #2. Both matrices are element copies of the Gf originals --
// the same doubles in the same flat order -- so each holds the TRANSPOSE linear
// map of its Gf counterpart, and the arithmetic is spelled in `gfmath`, the
// bit-exact pxr transcriptions, rather than in the PhysX-semantics helpers
// beside them. `gfmath::removeScaleShearGf` is NOT `omni::physx::removeScaleShear`
// (that one is an orthonormalization and disagrees on sheared input) and
// `gfmath::inverse` is NOT `affineInverse`; either substitution changes the
// poisson-sampling cache key. See the gfmath banner in MatrixTools.h.
bool decomposeSamplerTransform(const ::physx::PxMat44d& l2w,
                               ::physx::PxMat44d& rigidTransform,
                               ::physx::PxMat33d& shearScaleTransform)
{
    // Gf's `l2w.RemoveScaleShear()` and `l2w * l2w_r.GetInverse()`.
    rigidTransform = gfmath::removeScaleShearGf(l2w);
    const ::physx::PxMat44d shearScale = gfmath::multiply(l2w, gfmath::inverse(rigidTransform));

    // The nine wire bytes are Gf's ROW-MAJOR l2w_ss[i][j] for i,j in 0..2, which
    // is STRIDE-4 in the flat sixteen. Reading column0/1/2 of a PxMat33d built
    // from the 3x3 linear map instead would transpose three off-diagonal pairs
    // and silently rekey every sheared sampler; omni.physx.cooking reinterprets
    // these same bytes back as a PxMat33d, so neither side may move.
    const double* d = shearScale.front();
    const double ss[9] = { d[0], d[1], d[2], d[4], d[5], d[6], d[8], d[9], d[10] };
    std::memcpy(&shearScaleTransform.column0.x, ss, sizeof(ss));
    return true;
}

void createParticleSampler(const ObjectKey samplerKey, const ObjectKey particlePrimKey)
{
    PhysxParticleFactory* factory = getParticleFactory(particlePrimKey);
    if (!factory)
    {
        factory = ICE_NEW(PhysxParticleFactory)(particlePrimKey);
        gParticlePrimsToFactoryMap.insert({particlePrimKey, factory});
    }

    if (!factory->getParticleSampler(samplerKey))
        factory->addSampler(samplerKey);
}

void updateParticleSampler(const ObjectKey samplerKey, const ObjectKey particlePrimKey, bool forceResampling)
{
    // The tuple (particlePrim, sampler) is treated as a whole: if the particle path changes, the
    // sampler is destroyed and recreated in the new factory since state can't transfer between factories.

    PhysxParticleFactory* factory = getParticleFactory(particlePrimKey);
    if (factory)
    {
        if (!factory->updateSampler(samplerKey, forceResampling))
            removeParticleSampler(samplerKey, particlePrimKey);
    }
}

void removeParticleSampler(const ObjectKey samplerKey, ObjectKey particlePrimKey)
{
    PhysxParticleFactory* factory = nullptr;
    if (particlePrimKey.valid())
    {
        factory = getParticleFactory(particlePrimKey);
    }
    else
    {
        // If the particles relationship was removed from a sampler, particlePrimKey is invalid here,
        // so search all factories for the sampler to still clean it up.
        for (auto it: gParticlePrimsToFactoryMap)
        {
            PhysxParticleSampler* sampler = it.second->getParticleSampler(samplerKey);
            if (sampler)
            {
                factory = it.second;
                particlePrimKey = it.first;
                break;
            }
        }
    }

    if (factory)
    {
        factory->removeSampler(samplerKey);

        if (factory->empty())
        {
            gParticlePrimsToFactoryMap.erase(particlePrimKey);
            SAFE_DELETE_SINGLE(factory);
        }
    }
}

}
}
}
