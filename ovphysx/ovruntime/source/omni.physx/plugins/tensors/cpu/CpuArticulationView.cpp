// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-INPUT-CORE-001
 * @covers AC-3 AC-4
 *
 * @implements REQ-READ-CORE-001
 * @covers AC-6
 *
 * @implements REQ-READ-TENDON-001
 * @covers AC-4
 *
 * @implements REQ-TENSOR-PATH-001
 * @covers AC-3
 *
 * @implements REQ-TENSOR-INDEX-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-READ-ARTICULATION-001
 * @covers AC-3 AC-4
 *
 * @implements REQ-READ-ATTRS-001
 * @covers AC-15, AC-17
 *
 * @implements REQ-READ-INVDYN-001
 * @covers AC-1, AC-7, AC-10
 *
 * @implements REQ-INPUT-COVERAGE-001
 * @covers AC-11
 */

// clang-format off
// clang-format on

#include "tensors/cpu/CpuArticulationView.h"
#include "tensors/cpu/CpuSimulationView.h"

#include "tensors/GlobalsAreBad.h"
#include "tensors/CommonTypes.h"

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>

#include <algorithm> // std::sort / std::unique for the touched-articulation dedup
#include <cstring> // memcpy / memset in the inverse dynamics gathers
#include <omni/physx/IPhysx.h>

#include <omni/physics/tensors/TensorUtils.h>

using omni::physics::tensors::checkTensorDevice;
using omni::physics::tensors::checkTensorFloat32;
using omni::physics::tensors::checkTensorInt32;
using omni::physics::tensors::checkTensorSizeExact;
using omni::physics::tensors::checkTensorSizeMinimum;
using omni::physics::tensors::checkRecordIndices;
using omni::physics::tensors::getTensorTotalSize;

using namespace physx;

namespace omni
{
namespace physx
{
namespace tensors
{

namespace
{
// PhysX computes dense inverse dynamics in its parent-first generalized-coordinate basis. The tensor APIs
// expose the authored DOF basis instead. CPU reads transform while copying here; DirectGPU reads
// fuse the same transforms into their existing gather kernels in CudaKernels.cu. The layouts stay
// explicit because J transforms columns, M transforms rows and columns, and [A | b] leaves b alone.
void copyJacobianToAuthoredDofBasis(float* dst,
                                    const float* src,
                                    ::physx::PxU32 rows,
                                    ::physx::PxU32 cols,
                                    ::physx::PxU32 rootDofs,
                                    const ArticulationMetatype& metatype)
{
    memcpy(dst, src, size_t(rows) * cols * sizeof(float));
    if (!metatype.hasReversedDofBodyOrder())
    {
        return;
    }
    const ::physx::PxU32 dofCount = metatype.getDofCount();
    for (::physx::PxU32 dof = 0; dof < dofCount; ++dof)
    {
        if (!metatype.isDofBody0Parent(dof))
        {
            const ::physx::PxU32 col = rootDofs + dof;
            for (::physx::PxU32 row = 0; row < rows; ++row)
            {
                dst[row * cols + col] = -dst[row * cols + col];
            }
        }
    }
}

void copyMassMatrixToAuthoredDofBasis(float* dst,
                                      const float* src,
                                      ::physx::PxU32 generalizedCoords,
                                      ::physx::PxU32 rootDofs,
                                      const ArticulationMetatype& metatype)
{
    memcpy(dst, src, size_t(generalizedCoords) * generalizedCoords * sizeof(float));
    if (!metatype.hasReversedDofBodyOrder())
    {
        return;
    }
    const ::physx::PxU32 dofCount = metatype.getDofCount();
    for (::physx::PxU32 dof = 0; dof < dofCount; ++dof)
    {
        if (!metatype.isDofBody0Parent(dof))
        {
            const ::physx::PxU32 coord = rootDofs + dof;
            for (::physx::PxU32 col = 0; col < generalizedCoords; ++col)
            {
                dst[coord * generalizedCoords + col] = -dst[coord * generalizedCoords + col];
            }
            for (::physx::PxU32 row = 0; row < generalizedCoords; ++row)
            {
                dst[row * generalizedCoords + coord] = -dst[row * generalizedCoords + coord];
            }
        }
    }
}

void copyCentroidalMomentumToAuthoredDofBasis(
    float* dst, const float* matrix, const float* bias, ::physx::PxU32 numDofs, const ArticulationMetatype& metatype)
{
    const ::physx::PxU32 generalizedCoords = numDofs + 6u;
    const ::physx::PxU32 dstStride = generalizedCoords + 1u;
    for (::physx::PxU32 row = 0; row < 6u; ++row)
    {
        memcpy(dst + row * dstStride, matrix + row * generalizedCoords, size_t(generalizedCoords) * sizeof(float));
        dst[row * dstStride + generalizedCoords] = bias[row];
    }
    if (!metatype.hasReversedDofBodyOrder())
    {
        return;
    }
    const ::physx::PxU32 dofCount = metatype.getDofCount();
    for (::physx::PxU32 dof = 0; dof < dofCount; ++dof)
    {
        if (!metatype.isDofBody0Parent(dof))
        {
            const ::physx::PxU32 col = 6u + dof;
            for (::physx::PxU32 row = 0; row < 6u; ++row)
            {
                dst[row * dstStride + col] = -dst[row * dstStride + col];
            }
        }
    }
}

// Resolve the whole selection before any gather writes: the root workers below discover a null
// articulation or cache only while walking it, and buildArticulationRootGroups publishes nothing on
// failure, which only holds if a failed column leaves no half-filled buffer behind.
bool ovStageRootRowsResolvable(const std::vector<ArticulationEntry>& entries,
                               const std::vector<::physx::PxArticulationCache*>& caches,
                               const ::physx::PxU32* rows,
                               ::physx::PxU32 count,
                               bool needCache,
                               const char* label,
                               const char* funcName)
{
    for (::physx::PxU32 i = 0; i < count; ++i)
    {
        const ::physx::PxU32 row = rows ? rows[i] : i;
        if (!entries[row].arti || (needCache && (!caches[row] || !caches[row]->rootLinkData)))
        {
            CARB_LOG_ERROR("%s: %s row %u has no resolvable articulation state", funcName, label, row);
            return false;
        }
    }
    return true;
}
} // namespace

CpuArticulationView::CpuArticulationView(CpuSimulationView* sim, const std::vector<ArticulationEntry>& entries)
    : BaseArticulationView(sim, entries)
{
    PxU32 numArtis = PxU32(mEntries.size());

    // initialize articulation cache
    mArticulationCaches.resize(numArtis);
    for (PxU32 i = 0; i < numArtis; i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        mArticulationCaches[i] = arti->createCache();
        if (mArticulationCaches[i])
        {
            arti->copyInternalStateToCache(*mArticulationCaches[i], PxArticulationCacheFlag::eALL);
        }
    }
    mCpuSimData = sim->getCpuSimulationData();
}

CpuArticulationView::~CpuArticulationView()
{
    // PxArticulationReducedCoordinate::release() does not free caches created from it, so the view
    // owns them. Skipped once the PhysX plugin unloads: release() frees through the foundation
    // allocator.
    if (g_physx)
    {
        for (PxArticulationCache* cache : mArticulationCaches)
        {
            if (cache)
            {
                cache->release();
            }
        }
    }
    mArticulationCaches.clear();
}

bool CpuArticulationView::getLinkTransforms(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "link transform", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "link transform", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxLinks * 7u, "link transform", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        auto& links = mEntries[i].links;
        Subspace* subspace = mEntries[i].subspace;
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxLinks * 7;
        for (PxU32 j = 0; j < mEntries[i].numLinks; j++)
        {
            PxTransform pose = links[j]->getGlobalPose();
            if (subspace)
            {
                pose.p.x -= subspace->origin.x;
                pose.p.y -= subspace->origin.y;
                pose.p.z -= subspace->origin.z;
            }
            *dst++ = pose.p.x;
            *dst++ = pose.p.y;
            *dst++ = pose.p.z;
            *dst++ = pose.q.x;
            *dst++ = pose.q.y;
            *dst++ = pose.q.z;
            *dst++ = pose.q.w;
        }
    }

    return true;
}

bool CpuArticulationView::getLinkVelocities(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "link velocity", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "link velocity", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxLinks * 6u, "link velocity", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        auto& links = mEntries[i].links;
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxLinks * 6;
        for (PxU32 j = 0; j < mEntries[i].numLinks; j++)
        {
            PxVec3 linvel = links[j]->getLinearVelocity();
            PxVec3 angvel = links[j]->getAngularVelocity();
            *dst++ = linvel.x;
            *dst++ = linvel.y;
            *dst++ = linvel.z;
            *dst++ = angvel.x;
            *dst++ = angvel.y;
            *dst++ = angvel.z;
        }
    }

    return true;
}

bool CpuArticulationView::getLinkAccelerations(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "link acceleration", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "link acceleration", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxLinks * 6u, "link acceleration", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {

        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];
        if (cache && arti)
        {
            arti->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eLINK_ACCELERATION);
            float* dst = static_cast<float*>(dstTensor->data) + i * mMaxLinks * 6;
            for (PxU32 j = 0; j < mEntries[i].numLinks; j++)
            {
                PxVec3 linAcc = cache->linkAcceleration[j].linear;
                PxVec3 angAcc = cache->linkAcceleration[j].angular;
                *dst++ = linAcc.x;
                *dst++ = linAcc.y;
                *dst++ = linAcc.z;
                *dst++ = angAcc.x;
                *dst++ = angAcc.y;
                *dst++ = angAcc.z;
            }
        }
    }

    return true;
}

bool CpuArticulationView::getRootTransforms(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "root transform", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "root transform", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 7u, "root transform", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];
        if (arti && cache)
        {
            arti->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eROOT_TRANSFORM);
            PxTransform pose = cache->rootLinkData->transform;
            Subspace* subspace = mEntries[i].subspace;
            if (subspace)
            {
                pose.p.x -= subspace->origin.x;
                pose.p.y -= subspace->origin.y;
                pose.p.z -= subspace->origin.z;
            }
            float* dst = static_cast<float*>(dstTensor->data) + i * 7;
            *dst++ = pose.p.x;
            *dst++ = pose.p.y;
            *dst++ = pose.p.z;
            *dst++ = pose.q.x;
            *dst++ = pose.q.y;
            *dst++ = pose.q.z;
            *dst++ = pose.q.w;
        }
    }

    return true;
}

bool CpuArticulationView::getRootVelocities(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "root velocity", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "root velocity", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 6u, "root velocity", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];
        if (arti && cache)
        {
            arti->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eROOT_VELOCITIES);
            const PxVec3& linvel = cache->rootLinkData->worldLinVel;
            const PxVec3& angvel = cache->rootLinkData->worldAngVel;
            float* dst = static_cast<float*>(dstTensor->data) + i * 6;
            *dst++ = linvel.x;
            *dst++ = linvel.y;
            *dst++ = linvel.z;
            *dst++ = angvel.x;
            *dst++ = angvel.y;
            *dst++ = angvel.z;
        }
    }

    return true;
}

// Every requested root-state column in ONE pass over the articulations.
//
// The four columns are served by TWO host fetches: eROOT_TRANSFORM fills rootLinkData->transform for
// both pose columns, and eROOT_VELOCITIES fills worldLinVel and worldAngVel for both velocity ones.
// copyInternalStateToCache is what a column costs, so the flags are OR'd into a single call rather
// than issued per source.
bool CpuArticulationView::getRootStateColumnsOvStage(const RootStateColumn* columns,
                                                     PxU32 numColumns,
                                                     const PxU32* rows,
                                                     PxU32 count) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    // Two conditions, two answers: zero columns or zero rows is a well-formed request with nothing
    // to do, while a null `columns` is a caller bug and is refused.
    if (numColumns == 0 || count == 0)
        return true; // nothing to emit -- not a failure
    if (!columns)
    {
        return false;
    }

    // Which fetches this SET needs, decided once. rootStateSource owns the relation: the fill loop
    // reads fields of rootLinkData that only the named flag populates, so a quantity missing from
    // the union would read a slot nobody filled -- a previous step's value, under the right name.
    uint32_t sources = 0;
    for (PxU32 c = 0; c < numColumns; ++c)
    {
        const TensorDesc* const dst = columns[c].dst;
        const char* const label = rootStateLabel(columns[c].quantity);
        if (!dst || !dst->data)
        {
            return false;
        }
        if (!checkTensorDevice(*dst, -1, label, __FUNCTION__) ||
            !checkTensorFloat32(*dst, label, __FUNCTION__) ||
            !checkTensorSizeExact(*dst, count * rootStateComponents(columns[c].quantity), label, __FUNCTION__))
        {
            return false;
        }
        sources |= rootStateSource(columns[c].quantity);
    }

    // Once for the read rather than once per column.
    const char* const setLabel = rootStateSetLabel();
    if (!checkRecordIndices(rows, count, mEntries.size(), setLabel, __FUNCTION__) ||
        !ovStageRootRowsResolvable(mEntries, mArticulationCaches, rows, count, true, setLabel, __FUNCTION__))
    {
        return false;
    }

    PxArticulationCacheFlags flags = PxArticulationCacheFlags(0);
    if (sources & eRootSrcPose)
        flags |= PxArticulationCacheFlag::eROOT_TRANSFORM;
    if (sources & (eRootSrcLinearVel | eRootSrcAngularVel))
        flags |= PxArticulationCacheFlag::eROOT_VELOCITIES;

    for (PxU32 i = 0; i < count; i++)
    {
        const PxU32 row = rows ? rows[i] : i;
        PxArticulationReducedCoordinate* const articulation = mEntries[row].arti;
        PxArticulationCache* const cache = mArticulationCaches[row];
        articulation->copyInternalStateToCache(*cache, flags);

        for (PxU32 c = 0; c < numColumns; ++c)
        {
            float* const dst = static_cast<float*>(columns[c].dst->data);
            switch (columns[c].quantity)
            {
            case RootStateQuantity::ePosition:
            {
                const PxVec3& p = cache->rootLinkData->transform.p;
                dst[size_t(i) * 3 + 0] = p.x;
                dst[size_t(i) * 3 + 1] = p.y;
                dst[size_t(i) * 3 + 2] = p.z;
                break;
            }
            case RootStateQuantity::eOrientation:
            {
                const PxQuat& q = cache->rootLinkData->transform.q;
                dst[size_t(i) * 4 + 0] = q.x;
                dst[size_t(i) * 4 + 1] = q.y;
                dst[size_t(i) * 4 + 2] = q.z;
                dst[size_t(i) * 4 + 3] = q.w;
                break;
            }
            case RootStateQuantity::eLinearVelocity:
            case RootStateQuantity::eAngularVelocity:
            {
                const PxVec3& v = columns[c].quantity == RootStateQuantity::eAngularVelocity ?
                                      cache->rootLinkData->worldAngVel :
                                      cache->rootLinkData->worldLinVel;
                dst[size_t(i) * 3 + 0] = v.x;
                dst[size_t(i) * 3 + 1] = v.y;
                dst[size_t(i) * 3 + 2] = v.z;
                break;
            }
            }
        }
    }
    return true;
}

bool CpuArticulationView::getMassCentersOvStage(const TensorDesc* dstTensor,
                                                const PxU32* rows,
                                                PxU32 count,
                                                bool localFrame) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    const char* label = localFrame ? "articulation local mass center" : "articulation world mass center";
    if (!checkTensorDevice(*dstTensor, -1, label, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, label, __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, count * 3u, label, __FUNCTION__) ||
        !checkRecordIndices(rows, count, mEntries.size(), label, __FUNCTION__) ||
        !ovStageRootRowsResolvable(mEntries, mArticulationCaches, rows, count, false, label, __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < count; i++)
    {
        const PxU32 row = rows ? rows[i] : i;
        PxArticulationReducedCoordinate* const articulation = mEntries[row].arti;
        const PxVec3 center = articulation->computeArticulationCOM(localFrame);
        dst[size_t(i) * 3 + 0] = center.x;
        dst[size_t(i) * 3 + 1] = center.y;
        dst[size_t(i) * 3 + 2] = center.z;
    }
    return true;
}

// The four entry points the attribute table names, each a ONE-column call into the gather above, so
// a single-column read pays one fetch and the fill is written once.
bool CpuArticulationView::getPositionsOvStage(const TensorDesc* dstTensor, const PxU32* rows, PxU32 count, uint64_t) const
{
    const RootStateColumn column{ RootStateQuantity::ePosition, dstTensor };
    return getRootStateColumnsOvStage(&column, 1, rows, count);
}

bool CpuArticulationView::getOrientationsOvStage(const TensorDesc* dstTensor, const PxU32* rows, PxU32 count, uint64_t) const
{
    const RootStateColumn column{ RootStateQuantity::eOrientation, dstTensor };
    return getRootStateColumnsOvStage(&column, 1, rows, count);
}

bool CpuArticulationView::getLinearVelocitiesOvStage(const TensorDesc* dstTensor,
                                                     const PxU32* rows,
                                                     PxU32 count,
                                                     uint64_t) const
{
    const RootStateColumn column{ RootStateQuantity::eLinearVelocity, dstTensor };
    return getRootStateColumnsOvStage(&column, 1, rows, count);
}

bool CpuArticulationView::getAngularVelocitiesOvStage(const TensorDesc* dstTensor,
                                                      const PxU32* rows,
                                                      PxU32 count,
                                                      uint64_t) const
{
    const RootStateColumn column{ RootStateQuantity::eAngularVelocity, dstTensor };
    return getRootStateColumnsOvStage(&column, 1, rows, count);
}

bool CpuArticulationView::getMassCentersWorldOvStage(const TensorDesc* dstTensor,
                                                     const PxU32* rows,
                                                     PxU32 count,
                                                     uint64_t) const
{
    return getMassCentersOvStage(dstTensor, rows, count, false);
}

bool CpuArticulationView::getMassCentersLocalOvStage(const TensorDesc* dstTensor,
                                                     const PxU32* rows,
                                                     PxU32 count,
                                                     uint64_t) const
{
    return getMassCentersOvStage(dstTensor, rows, count, true);
}

// ------------------------------------------------------------------------------------------------
// Inverse dynamics columns (REQ-READ-INVDYN-001).
//
// The width comes from the CALLER, in dims[1], because it is a function of the selected
// articulations' topology rather than of the attribute -- the reader groups rows into cohorts of one
// metatype and passes that cohort's width down. Each getter re-derives the expected width from the
// rows it was given and refuses a mismatch, which would otherwise stride into the next row and
// produce a plausible matrix rather than a failure.
//
// Every quantity here is a cache read, so the shared prologue also demands a resolvable cache per
// row and fails before writing, like the state getters above.
// ------------------------------------------------------------------------------------------------

bool CpuArticulationView::getJacobiansOvStage(const TensorDesc* dstTensor,
                                              const PxU32* rows,
                                              PxU32 count,
                                              uint64_t) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    const char* label = "articulation jacobian";
    PxU32 width = 0;
    if (!checkTensorDevice(*dstTensor, -1, label, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, label, __FUNCTION__) ||
        !checkRecordIndices(rows, count, mEntries.size(), label, __FUNCTION__) ||
        !ovStageRootRowsResolvable(mEntries, mArticulationCaches, rows, count, true, label, __FUNCTION__) ||
        !checkInverseDynamicsColumn(*dstTensor, rows, count, InverseDynamicsColumn::eJacobian, width, label,
                                    __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, count * width, label, __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < count; i++)
    {
        const PxU32 row = rows ? rows[i] : i;
        PxArticulationReducedCoordinate* const arti = mEntries[row].arti;
        PxArticulationCache* const cache = mArticulationCaches[row];

        PxU32 producedRows = 0, producedCols = 0;
        arti->commonInit();
        arti->computeDenseJacobian(*cache, producedRows, producedCols);
        // The width was agreed above from this row's metatype, so PhysX disagreeing means the
        // topology moved mid-read.
        //
        // Fails the whole gather rather than zeroing the row and carrying on, unlike the view-wide
        // getJacobians: this read is atomic, so a zeroed row behind a `true` would surface through
        // ovxFetchReadNext as kOvxReadStatusOk with an all-zero jacobian indistinguishable from a
        // real one.
        const PxU32 rootDofs = mEntries[row].metatype->getFixedBase() ? 0u : 6u;
        const PxU32 expectedCols = rootDofs + mEntries[row].numDofs;
        if (producedRows * producedCols != width || producedCols != expectedCols)
        {
            CARB_LOG_ERROR("%s: jacobian row %u produced shape %ux%u; expected %u columns and %u elements",
                           __FUNCTION__, row, producedRows, producedCols, expectedCols, width);
            return false;
        }
        copyJacobianToAuthoredDofBasis(dst + size_t(i) * width, cache->denseJacobian, producedRows, producedCols,
                                       rootDofs, *mEntries[row].metatype);
    }
    return true;
}

bool CpuArticulationView::getMassMatricesOvStage(const TensorDesc* dstTensor,
                                                 const PxU32* rows,
                                                 PxU32 count,
                                                 uint64_t) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    const char* label = "articulation mass matrix";
    PxU32 width = 0;
    if (!checkTensorDevice(*dstTensor, -1, label, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, label, __FUNCTION__) ||
        !checkRecordIndices(rows, count, mEntries.size(), label, __FUNCTION__) ||
        !ovStageRootRowsResolvable(mEntries, mArticulationCaches, rows, count, true, label, __FUNCTION__) ||
        !checkInverseDynamicsColumn(*dstTensor, rows, count, InverseDynamicsColumn::eMassMatrix, width, label,
                                    __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, count * width, label, __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < count; i++)
    {
        const PxU32 row = rows ? rows[i] : i;
        PxArticulationReducedCoordinate* const arti = mEntries[row].arti;
        PxArticulationCache* const cache = mArticulationCaches[row];
        arti->commonInit();
        arti->computeMassMatrix(*cache);
        const PxU32 rootDofs = mEntries[row].metatype->getFixedBase() ? 0u : 6u;
        const PxU32 generalizedCoords = rootDofs + mEntries[row].numDofs;
        copyMassMatrixToAuthoredDofBasis(
            dst + size_t(i) * width, cache->massMatrix, generalizedCoords, rootDofs, *mEntries[row].metatype);
    }
    return true;
}

// Coriolis and gravity share everything but their compute call and their cache array, including the
// per-dof sign flip -- a dof whose body0 is not the parent reports the opposite sign, the same
// convention the view-wide getters apply.
//
// The sign flip reads mEntries[row].metatype without a local null check: checkInverseDynamicsColumn rejects
// the gather unless every selected row agrees with a NONZERO width, and inverseDynamicsColumnWidthForRow
// answers 0 for a row whose metatype is null, so reaching the loop means every metatype resolved.
bool CpuArticulationView::getGeneralizedForceColumnOvStage(const TensorDesc* dstTensor,
                                                           const PxU32* rows,
                                                           PxU32 count,
                                                           bool gravity) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    const char* label = gravity ? "articulation gravity force" : "articulation coriolis force";
    PxU32 width = 0;
    if (!checkTensorDevice(*dstTensor, -1, label, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, label, __FUNCTION__) ||
        !checkRecordIndices(rows, count, mEntries.size(), label, __FUNCTION__) ||
        !ovStageRootRowsResolvable(mEntries, mArticulationCaches, rows, count, true, label, __FUNCTION__) ||
        !checkInverseDynamicsColumn(*dstTensor, rows, count, InverseDynamicsColumn::eGeneralizedForce, width, label,
                                    __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, count * width, label, __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < count; i++)
    {
        const PxU32 row = rows ? rows[i] : i;
        PxArticulationReducedCoordinate* const arti = mEntries[row].arti;
        PxArticulationCache* const cache = mArticulationCaches[row];
        // computeCoriolisCompensation reads the joint velocities OUT OF THE CACHE, not off the
        // articulation, so without this refresh the Coriolis force comes out of whatever the cache
        // happened to hold: the right shape, the wrong size, and no failure reported.
        //
        // Gravity compensation needs no copy -- it is a function of configuration, which commonInit
        // refreshes -- and this runs per articulation per read, so the distinction is worth keeping.
        arti->commonInit();
        if (gravity)
        {
            arti->computeGravityCompensation(*cache);
        }
        else
        {
            arti->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eVELOCITY);
            arti->computeCoriolisCompensation(*cache);
        }

        const PxReal* const src = gravity ? cache->gravityCompensationForce : cache->coriolisForce;
        const bool isFixedBase = mEntries[row].metatype->getFixedBase();
        const PxU32 numDofs = mEntries[row].numDofs;
        // A floating base contributes six leading root entries, which carry no dof and so take no
        // sign flip; the dof entries follow.
        const PxU32 rootDofs = isFixedBase ? 0u : 6u;
        float* const out = dst + size_t(i) * width;
        for (PxU32 j = 0; j < rootDofs; j++)
            out[j] = src[j];
        for (PxU32 j = 0; j < numDofs; j++)
            out[rootDofs + j] = mEntries[row].metatype->isDofBody0Parent(j) ? src[rootDofs + j] : -src[rootDofs + j];
    }
    return true;
}

bool CpuArticulationView::getCoriolisForcesOvStage(const TensorDesc* dstTensor,
                                                   const PxU32* rows,
                                                   PxU32 count,
                                                   uint64_t) const
{
    return getGeneralizedForceColumnOvStage(dstTensor, rows, count, false);
}

bool CpuArticulationView::getGravityForcesOvStage(const TensorDesc* dstTensor,
                                                  const PxU32* rows,
                                                  PxU32 count,
                                                  uint64_t) const
{
    return getGeneralizedForceColumnOvStage(dstTensor, rows, count, true);
}

bool CpuArticulationView::getCentroidalMomentaOvStage(const TensorDesc* dstTensor,
                                                      const PxU32* rows,
                                                      PxU32 count,
                                                      uint64_t) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    // Floating base only. The reader omits the group for a fixed-base cohort, so reaching here with
    // one is a reader bug rather than a user error -- fail rather than invent rows.
    const char* label = "articulation centroidal momentum";
    PxU32 width = 0;
    if (!checkTensorDevice(*dstTensor, -1, label, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, label, __FUNCTION__) ||
        !checkRecordIndices(rows, count, mEntries.size(), label, __FUNCTION__) ||
        !ovStageRootRowsResolvable(mEntries, mArticulationCaches, rows, count, true, label, __FUNCTION__) ||
        !checkInverseDynamicsColumn(*dstTensor, rows, count, InverseDynamicsColumn::eCentroidalMomentum, width, label,
                                    __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, count * width, label, __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < count; i++)
    {
        const PxU32 row = rows ? rows[i] : i;
        PxArticulationReducedCoordinate* const arti = mEntries[row].arti;
        PxArticulationCache* const cache = mArticulationCaches[row];
        // The centroidal matrix is defined against the mass matrix and the Coriolis term, so both
        // have to be computed into the cache first; PhysX does not do it for us.
        //
        // The velocity copy is defensive, kept for symmetry with getArticulationCentroidalMomentum:
        // computeCentroidalMomentumMatrix appears to take its velocities from the articulation
        // rather than from the cache, but that is not a documented contract.
        arti->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eVELOCITY);
        arti->commonInit();
        arti->computeMassMatrix(*cache);
        arti->computeCoriolisCompensation(*cache);
        arti->computeCentroidalMomentumMatrix(*cache);

        const PxU32 numDofs = mEntries[row].numDofs;
        float* const out = dst + size_t(i) * width;
        copyCentroidalMomentumToAuthoredDofBasis(
            out, cache->centroidalMomentumMatrix, cache->centroidalMomentumBias, numDofs, *mEntries[row].metatype);
    }
    return true;
}

bool CpuArticulationView::setRootTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    
    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "root transform", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "root transform", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * 7u, "root transform", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            PxArticulationReducedCoordinate* arti = mEntries[idx].arti;
            PxArticulationCache* cache = mArticulationCaches[idx];
            if (arti && cache)
            {
                const float* src = static_cast<const float*>(srcTensor->data) + idx * 7;
                PxTransform& pose = cache->rootLinkData->transform;
                pose.p.x = *src++;
                pose.p.y = *src++;
                pose.p.z = *src++;
                pose.q.x = *src++;
                pose.q.y = *src++;
                pose.q.z = *src++;
                pose.q.w = *src++;

                Subspace* subspace = mEntries[idx].subspace;
                if (subspace)
                {
                    pose.p.x += subspace->origin.x;
                    pose.p.y += subspace->origin.y;
                    pose.p.z += subspace->origin.z;
                }
                arti->applyCache(*cache, PxArticulationCacheFlag::eROOT_TRANSFORM);
            }
        }
    }

    return true;
}

bool CpuArticulationView::setRootVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "root velocity", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "root velocity", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * 6u, "root velocity", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            PxArticulationReducedCoordinate* arti = mEntries[idx].arti;
            PxArticulationCache* cache = mArticulationCaches[idx];
            if (arti && cache)
            {
                const float* src = static_cast<const float*>(srcTensor->data) + idx * 6;
                PxVec3& linvel = cache->rootLinkData->worldLinVel;
                PxVec3& angvel = cache->rootLinkData->worldAngVel;
                linvel.x = *src++;
                linvel.y = *src++;
                linvel.z = *src++;
                angvel.x = *src++;
                angvel.y = *src++;
                angvel.z = *src++;

                arti->applyCache(*cache, PxArticulationCacheFlag::eROOT_VELOCITIES);
            }
        }
    }

    return true;
}

bool CpuArticulationView::getDofPositions(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "DOF position", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "DOF position", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "DOF position", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];
        if (arti && cache)
        {
            arti->copyInternalStateToCache(*cache, PxArticulationCacheFlag::ePOSITION);
            float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs;
            for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
            {
                dst[j] = mEntries[i].metatype->isDofBody0Parent(j) ? cache->jointPosition[j] : -cache->jointPosition[j];
            }
        }
    }

    return true;
}

bool CpuArticulationView::getDofVelocities(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "DOF velocity", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "DOF velocity", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "DOF velocity", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];
        if (arti && cache)
        {
            arti->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eVELOCITY);
            float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs;
            for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
            {
                dst[j] = mEntries[i].metatype->isDofBody0Parent(j) ? cache->jointVelocity[j] : -cache->jointVelocity[j];
            }
        }
    }

    return true;
}

bool CpuArticulationView::setDofPositionsOvStage(const TensorDesc* srcTensor,
                                                 const ArticulationDofOvStageRecord* records, PxU32 numOutputs)
{
    return setDofAttributeOvStage(srcTensor, DofStateQuantity::ePosition, records, numOutputs);
}

bool CpuArticulationView::setDofVelocitiesOvStage(const TensorDesc* srcTensor,
                                                  const ArticulationDofOvStageRecord* records, PxU32 numOutputs)
{
    return setDofAttributeOvStage(srcTensor, DofStateQuantity::eVelocity, records, numOutputs);
}

// The three drive INPUTS (ADR-0012). Each names the same QUANTITY its getter does, so the label,
// the cache flag and the scale policy all come from one facts table rather than being restated
// here -- which is what keeps a value read and written back unchanged.
bool CpuArticulationView::setDofPositionTargetsOvStage(const TensorDesc* srcTensor,
                                                       const ArticulationDofOvStageRecord* records,
                                                       PxU32 numOutputs)
{
    return setDofAttributeOvStage(srcTensor, DofStateQuantity::ePositionTarget, records, numOutputs);
}

bool CpuArticulationView::setDofVelocityTargetsOvStage(const TensorDesc* srcTensor,
                                                       const ArticulationDofOvStageRecord* records,
                                                       PxU32 numOutputs)
{
    return setDofAttributeOvStage(srcTensor, DofStateQuantity::eVelocityTarget, records, numOutputs);
}

bool CpuArticulationView::setDofActuationForcesOvStage(const TensorDesc* srcTensor,
                                                       const ArticulationDofOvStageRecord* records,
                                                       PxU32 numOutputs)
{
    return setDofAttributeOvStage(srcTensor, DofStateQuantity::eActuationForce, records, numOutputs);
}

bool CpuArticulationView::setRootAttributeOvStage(const char* const attribName,
                                                  const TensorDesc* const srcTensor,
                                                  const PxU32* const rows,
                                                  const PxU32 numOutputs,
                                                  const uint64_t /*rowsToken*/,
                                                  const bool angular,
                                                  const bool pose)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    if (!srcTensor || !srcTensor->data || !rows)
        return false;
    if (numOutputs == 0)
        return true; // nothing to publish -- not a failure

    // Only the ORIENTATION is a quaternion; an angular VELOCITY is still a vec3. See the header.
    const PxU32 comp = (pose && angular) ? 4u : 3u;
    if (!checkTensorDevice(*srcTensor, -1, attribName, __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, attribName, __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, numOutputs * comp, attribName, __FUNCTION__) ||
        !checkRecordIndices(rows, numOutputs, mEntries.size(), attribName, __FUNCTION__) ||
        !ovStageRootRowsResolvable(mEntries, mArticulationCaches, rows, numOutputs, true, attribName, __FUNCTION__))
    {
        return false;
    }

    const PxArticulationCacheFlag::Enum cacheFlag =
        pose ? PxArticulationCacheFlag::eROOT_TRANSFORM : PxArticulationCacheFlag::eROOT_VELOCITIES;

    const float* const src = static_cast<const float*>(srcTensor->data);
    for (PxU32 i = 0; i < numOutputs; ++i)
    {
        // Bounds and root-state resolvability (arti, cache and rootLinkData) were validated for every
        // row up front by checkRecordIndices and ovStageRootRowsResolvable, so a partial write cannot
        // happen here -- applyCache runs per row below, and a mid-loop failure would otherwise leave the
        // earlier rows already pushed.
        const PxU32 row = rows[i];
        PxArticulationReducedCoordinate* arti = mEntries[row].arti;
        PxArticulationCache* cache = mArticulationCaches[row];

        // The read half of the read-modify-write: everything this call does not overwrite has to be
        // the articulation's CURRENT value, because applyCache pushes the whole flagged block back.
        arti->copyInternalStateToCache(*cache, cacheFlag);

        const float* o = src + static_cast<size_t>(i) * comp;
        if (pose)
        {
            PxTransform& rootPose = cache->rootLinkData->transform;
            if (angular)
            {
                rootPose.q = PxQuat(o[0], o[1], o[2], o[3]);
            }
            else
            {
                // Stage-local in, world out, mirroring what getRootTransforms subtracts.
                rootPose.p = PxVec3(o[0], o[1], o[2]);
                const Subspace* subspace = mEntries[row].subspace;
                if (subspace)
                {
                    rootPose.p.x += subspace->origin.x;
                    rootPose.p.y += subspace->origin.y;
                    rootPose.p.z += subspace->origin.z;
                }
            }
        }
        else
        {
            // No origin on a velocity: it is invariant under the subspace translation, which is why
            // getRootVelocities applies none either.
            PxVec3& v = angular ? cache->rootLinkData->worldAngVel : cache->rootLinkData->worldLinVel;
            v = PxVec3(o[0], o[1], o[2]);
        }
        arti->applyCache(*cache, cacheFlag);
    }
    return true;
}

bool CpuArticulationView::setRootPositionsOvStage(const TensorDesc* srcTensor,
                                                  const PxU32* rows,
                                                  const PxU32 numOutputs,
                                                  const uint64_t rowsToken)
{
    return setRootAttributeOvStage("position (ovstage write)", srcTensor, rows, numOutputs, rowsToken, false, true);
}

bool CpuArticulationView::setRootOrientationsOvStage(const TensorDesc* srcTensor,
                                                     const PxU32* rows,
                                                     const PxU32 numOutputs,
                                                     const uint64_t rowsToken)
{
    return setRootAttributeOvStage("orientation (ovstage write)", srcTensor, rows, numOutputs, rowsToken, true, true);
}

bool CpuArticulationView::setRootLinearVelocitiesOvStage(const TensorDesc* srcTensor,
                                                         const PxU32* rows,
                                                         const PxU32 numOutputs,
                                                         const uint64_t rowsToken)
{
    return setRootAttributeOvStage("linearVelocity (ovstage write)", srcTensor, rows, numOutputs, rowsToken, false,
                                   false);
}

bool CpuArticulationView::setRootAngularVelocitiesOvStage(const TensorDesc* srcTensor,
                                                          const PxU32* rows,
                                                          const PxU32 numOutputs,
                                                          const uint64_t rowsToken)
{
    return setRootAttributeOvStage("angularVelocity (ovstage write)", srcTensor, rows, numOutputs, rowsToken, true,
                                   false);
}

// The inverse of getDofAttributeOvStage: same `source` and `policy`, so the two cannot disagree
// about which folds a column carries. The scale is MULTIPLIED by the record's stored inverse rather
// than divided by its forward one -- see the loop below for why the distinction is not cosmetic.
bool CpuArticulationView::setDofAttributeOvStage(const TensorDesc* srcTensor,
                                                 const DofStateQuantity quantity,
                                                 const ArticulationDofOvStageRecord* records,
                                                 PxU32 numOutputs)
{
    // Label, cache flag and scale policy all come from the quantity's facts table, so the write
    // cannot disagree with the gather about any of them.
    const char* const attribName = dofStateLabel(quantity);
    const DofScalePolicy policy = dofStateScalePolicy(quantity);
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    if (!srcTensor || !srcTensor->data)
        return false;
    if (numOutputs == 0)
        return true;
    if (!records)
    {
        CARB_LOG_ERROR("%s: null ovstage DOF records", attribName);
        return false;
    }
    if (!checkTensorDevice(*srcTensor, -1, attribName, __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, attribName, __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, numOutputs, attribName, __FUNCTION__))
    {
        return false;
    }

    const float* srcF = static_cast<const float*>(srcTensor->data);

    // The two drive targets are per-axis joint properties, not cache columns -- the getter reads
    // them with getDriveTarget/getDriveVelocity for the same reason. No read-modify-write: each
    // setter addresses exactly one axis, so an axis this call skips is not touched.
    if (quantity == DofStateQuantity::ePositionTarget || quantity == DofStateQuantity::eVelocityTarget)
    {
        const bool wantPosition = (quantity == DofStateQuantity::ePositionTarget);
        for (PxU32 i = 0; i < numOutputs; i++)
        {
            const ArticulationDofOvStageRecord& r = records[i];
            const DofImpl* dofImpl = resolveDofImpl(r);
            if (!dofImpl || !dofImpl->joint)
                continue;
            const float v = srcF[i] * dofInverseScaleFor(r, policy);
            if (wantPosition)
                dofImpl->joint->setDriveTarget(dofImpl->axis, v);
            else
                dofImpl->joint->setDriveVelocity(dofImpl->axis, v);
        }
        return true;
    }

    // One place decides which cache backs a quantity, shared with the gather.
    const PxArticulationCacheFlags flag = dofStateCacheFlag(quantity);

    // Which cache column, decided once -- the same guard the gather uses
    // (getDofStateColumnsOvStage). A member pointer rather than a chain defaulting to jointPosition:
    // an unlisted quantity leaves this null and the write REFUSES, where a default would publish
    // values into jointPosition under that quantity's name. -Wno-switch is tree-wide, so a new
    // quantity would not be flagged for us.
    float* PxArticulationCache::*cacheColumn = nullptr;
    switch (quantity)
    {
    case DofStateQuantity::ePosition:
        cacheColumn = &PxArticulationCache::jointPosition;
        break;
    case DofStateQuantity::eVelocity:
        cacheColumn = &PxArticulationCache::jointVelocity;
        break;
    case DofStateQuantity::eActuationForce:
        cacheColumn = &PxArticulationCache::jointForce;
        break;
    case DofStateQuantity::ePositionTarget:
    case DofStateQuantity::eVelocityTarget:
    case DofStateQuantity::eNone:
        break; // targets were applied above; eNone is not a writable cache column
    }
    if (!cacheColumn)
    {
        CARB_LOG_ERROR("%s: no writable cache column for this quantity", dofStateLabel(quantity));
        return false;
    }

    // Read-modify-write, but only over the articulations records actually reference: applyCache pushes
    // a whole articulation's DOF block, so a touched articulation's unaddressed DOFs must hold their
    // current value first. Running it over the whole view would instead round-trip every UNTOUCHED
    // articulation's block at O(view size) -- the cost an RL caller writing a few of thousands of
    // parallel articulations would pay per write. Distinct rows (a write commonly names several DOFs of
    // one articulation), mirroring setRootAttributeOvStage's restriction to its referenced rows.
    std::vector<PxU32> touchedArtis;
    touchedArtis.reserve(numOutputs);
    for (PxU32 i = 0; i < numOutputs; i++)
    {
        const PxU32 idx = records[i].viewArtiIdx;
        if (idx < mArticulationCaches.size() && mArticulationCaches[idx] && mEntries[idx].arti)
            touchedArtis.push_back(idx);
    }
    std::sort(touchedArtis.begin(), touchedArtis.end());
    touchedArtis.erase(std::unique(touchedArtis.begin(), touchedArtis.end()), touchedArtis.end());

    for (const PxU32 idx : touchedArtis)
        mEntries[idx].arti->copyInternalStateToCache(*mArticulationCaches[idx], flag);

    for (PxU32 i = 0; i < numOutputs; i++)
    {
        const ArticulationDofOvStageRecord& r = records[i];
        PxArticulationCache* cache =
            (r.viewArtiIdx < mArticulationCaches.size()) ? mArticulationCaches[r.viewArtiIdx] : nullptr;
        if (!cache)
            continue; // no cache for this row: leave the pre-read value in place
        float& dst = (cache->*cacheColumn)[r.physxDofIdx];
        // MULTIPLY by invAngScale, not divide by angScale. The record stores both because the float
        // reciprocal of the rad->deg constant is not the float deg->rad one -- so a value the read
        // published as `authored * angScale` only returns to `authored` when the write multiplies by
        // the inverse constant the parse library actually used.
        //
        // The sign divides out the same way it multiplied in: it is +-1, so multiplying is exact.
        // WHICH folds apply is the attribute's, via the policy: position and velocity are
        // eAngularSigned, but an actuation force is eSigned -- a joint effort is not a rad->deg
        // quantity, and folding one in would scale every torque by 57.3.
        dst = srcF[i] * dofInverseScaleFor(r, policy);
    }

    for (const PxU32 idx : touchedArtis)
        mEntries[idx].arti->applyCache(*mArticulationCaches[idx], flag);
    return true;
}

bool CpuArticulationView::getDofPositionsOvStage(const ArticulationDofOvStageRecord* records,
                                                 PxU32 numOutputs, const TensorDesc* dstTensor) const
{
    return getDofAttributeOvStage(dstTensor, DofStateQuantity::ePosition, records, numOutputs);
}

bool CpuArticulationView::getDofVelocitiesOvStage(const ArticulationDofOvStageRecord* records,
                                                  PxU32 numOutputs, const TensorDesc* dstTensor) const
{
    return getDofAttributeOvStage(dstTensor, DofStateQuantity::eVelocity, records, numOutputs);
}

bool CpuArticulationView::getDofPositionTargetsOvStage(const ArticulationDofOvStageRecord* records,
                                                       PxU32 numOutputs, const TensorDesc* dstTensor) const
{
    return getDofAttributeOvStage(dstTensor, DofStateQuantity::ePositionTarget, records, numOutputs);
}

bool CpuArticulationView::getDofVelocityTargetsOvStage(const ArticulationDofOvStageRecord* records,
                                                       PxU32 numOutputs, const TensorDesc* dstTensor) const
{
    return getDofAttributeOvStage(dstTensor, DofStateQuantity::eVelocityTarget, records, numOutputs);
}

// Sign but no degree fold, matching the GPU path: a generalized force on the axis is a newton or a
// newton-metre, not an angle.
bool CpuArticulationView::getDofActuationForcesOvStage(const ArticulationDofOvStageRecord* records,
                                                       PxU32 numOutputs, const TensorDesc* dstTensor) const
{
    return getDofAttributeOvStage(dstTensor, DofStateQuantity::eActuationForce, records, numOutputs);
}

// One column, served by the set form so the refresh and the gather have a single definition. Label
// and scale policy are derived from the quantity, so a new quantity cannot arrive with one and not
// the other.
bool CpuArticulationView::getDofAttributeOvStage(const TensorDesc* dstTensor, DofStateQuantity quantity,
                                                 const ArticulationDofOvStageRecord* records,
                                                 PxU32 numOutputs) const
{
    PASS_EMPTY_TENSOR(dstTensor);
    const DofStateColumn column{ quantity, dstTensor };
    return getDofStateColumnsOvStage(&column, 1, records, numOutputs);
}

bool CpuArticulationView::getDofStateColumnsOvStage(const DofStateColumn* columns, PxU32 numColumns,
                                                    const ArticulationDofOvStageRecord* records,
                                                    PxU32 numOutputs) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    // Two conditions, two answers. Zero columns or zero rows is a well-formed request with nothing to
    // do; null pointers are a caller bug and stay refused. Same split as getRootStateColumnsOvStage.
    if (numColumns == 0 || numOutputs == 0)
    {
        return true; // nothing to emit -- not a failure
    }
    if (!columns)
    {
        return false;
    }
    if (!records)
    {
        CARB_LOG_ERROR("%s: null ovstage DOF records", dofStateSetLabel());
        return false;
    }

    // Validate every column and decide the union in the same pass. dofStateCacheFlag owns the
    // relation: the gather reads a cache column only the named flag fills, so a quantity missing
    // from the union would read a slot nobody wrote. Quantities with no cache behind them add none.
    PxArticulationCacheFlags flags = PxArticulationCacheFlags(0);
    for (PxU32 c = 0; c < numColumns; ++c)
    {
        const TensorDesc* const dst = columns[c].dst;
        if (columns[c].quantity == DofStateQuantity::eNone)
        {
            CARB_LOG_ERROR("%s: eNone is not a DOF state column", __FUNCTION__);
            return false;
        }
        const char* const label = dofStateLabel(columns[c].quantity);
        if (!dst || !dst->data)
        {
            return false;
        }
        if (!checkTensorDevice(*dst, -1, label, __FUNCTION__) ||
            !checkTensorFloat32(*dst, label, __FUNCTION__) ||
            !checkTensorSizeExact(*dst, numOutputs, label, __FUNCTION__))
        {
            return false;
        }
        flags |= dofStateCacheFlag(columns[c].quantity);
    }

    // ONE refresh pass, over the articulations these records name rather than over the view: a
    // multi-column read of a large scene otherwise refreshes every articulation once per column.
    if (flags != PxArticulationCacheFlags(0))
    {
        // The marks are all-zero on entry and left that way on exit, so the dedup costs one byte per
        // NAMED articulation; clearing them here instead would put an O(view) pass back on the read.
        if (mDofRefreshMarks.size() < mEntries.size())
            mDofRefreshMarks.resize(mEntries.size(), 0u);
        mDofRefreshList.clear();
        for (PxU32 i = 0; i < numOutputs; i++)
        {
            const PxU32 idx = records[i].viewArtiIdx;
            if (idx < mEntries.size() && !mDofRefreshMarks[idx])
            {
                mDofRefreshMarks[idx] = 1u;
                mDofRefreshList.push_back(idx);
            }
        }
        for (const PxU32 idx : mDofRefreshList)
        {
            PxArticulationReducedCoordinate* const arti = mEntries[idx].arti;
            PxArticulationCache* const cache = mArticulationCaches[idx];
            if (arti && cache)
                arti->copyInternalStateToCache(*cache, flags);
            mDofRefreshMarks[idx] = 0u;
        }
    }

    // Columns outer, rows inner -- the opposite of getRootStateColumnsOvStage. N walks of the record
    // list is the price of keeping the gather in a function of its own; the alternative interleaves
    // gather with refresh, which is what makes the refresh run once per column.
    for (PxU32 c = 0; c < numColumns; ++c)
    {
        if (!gatherDofStateColumn(columns[c], records, numOutputs))
        {
            return false;
        }
    }
    return true;
}

// One column's values, off state the caller has already refreshed.
bool CpuArticulationView::gatherDofStateColumn(const DofStateColumn& column,
                                               const ArticulationDofOvStageRecord* records,
                                               PxU32 numOutputs) const
{
    const DofStateQuantity quantity = column.quantity;
    const DofScalePolicy policy = dofStateScalePolicy(quantity);
    float* dst = static_cast<float*>(column.dst->data);

    // The two drive targets are per-joint PhysX state, so they gather straight off the (joint, axis)
    // the record names -- no cache, and nothing refreshed for them above.
    if (quantity == DofStateQuantity::ePositionTarget || quantity == DofStateQuantity::eVelocityTarget)
    {
        const bool wantPosition = (quantity == DofStateQuantity::ePositionTarget);
        for (PxU32 i = 0; i < numOutputs; i++)
        {
            const ArticulationDofOvStageRecord& r = records[i];
            const DofImpl* dofImpl = resolveDofImpl(r);
            if (!dofImpl || !dofImpl->joint)
            {
                dst[i] = 0.0f;
                continue;
            }
            const float raw = wantPosition ? dofImpl->joint->getDriveTarget(dofImpl->axis) :
                                             dofImpl->joint->getDriveVelocity(dofImpl->axis);
            dst[i] = dofScaleFor(r, policy) * raw;
        }
        return true;
    }

    // Which cache column, named per quantity and decided once for the whole gather. A member pointer
    // rather than a chain defaulting to jointPosition: an unlisted quantity leaves this null and the
    // gather refuses, where a default would publish joint POSITIONS under that quantity's attribute
    // name. -Wno-switch is set tree-wide, so nothing would report the missing case for us.
    float* PxArticulationCache::*cacheColumn = nullptr;
    switch (quantity)
    {
    case DofStateQuantity::ePosition:
        cacheColumn = &PxArticulationCache::jointPosition;
        break;
    case DofStateQuantity::eVelocity:
        cacheColumn = &PxArticulationCache::jointVelocity;
        break;
    case DofStateQuantity::eActuationForce:
        cacheColumn = &PxArticulationCache::jointForce;
        break;
    case DofStateQuantity::ePositionTarget:
    case DofStateQuantity::eVelocityTarget:
    case DofStateQuantity::eNone:
        break; // the targets returned above; eNone is refused at the set form's entry
    }
    if (!cacheColumn)
    {
        CARB_LOG_ERROR("%s: no cache column for this quantity", dofStateLabel(quantity));
        return false;
    }

    // Off the cache the caller refreshed. The raw value is unsigned; the record's axis facts plus the
    // quantity's policy carry the deg-conversion and the body0IsParent sign, matching the GPU path.
    //
    // BOTH of the record's indices are checked, against the same bounds the drive-target branch gets
    // from resolveDofImpl: that branch reaches physxDofIdx through a helper that refuses an
    // out-of-range one, while indexing the cache column directly has to say so itself.
    for (PxU32 i = 0; i < numOutputs; i++)
    {
        const ArticulationDofOvStageRecord& r = records[i];
        const PxArticulationCache* cache =
            (r.viewArtiIdx < mArticulationCaches.size()) ? mArticulationCaches[r.viewArtiIdx] : nullptr;
        if (!cache || r.physxDofIdx >= mEntries[r.viewArtiIdx].numDofs)
        {
            dst[i] = 0.0f;
            continue;
        }
        const float* const values = cache->*cacheColumn;
        dst[i] = dofScaleFor(r, policy) * values[r.physxDofIdx];
    }
    return true;
}

bool CpuArticulationView::getDofProjectedForcesOvStage(const ArticulationDofOvStageRecord* records,
                                                       PxU32 numOutputs, const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }
    if (numOutputs == 0)
    {
        return true; // nothing to emit -- not a failure
    }
    if (!records)
    {
        CARB_LOG_ERROR("DOF projected force (ovstage): null ovstage DOF records");
        return false;
    }
    const char* label = "jointProjectedForce";
    if (!checkTensorDevice(*dstTensor, -1, label, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, label, __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, numOutputs, label, __FUNCTION__))
    {
        return false;
    }

    // Projected onto the axis from the link's incoming joint force by the dense getter, parent-frame
    // swap and per-joint-type component included. Staged and gathered rather than re-derived here:
    // the value is computed per LINK, and a second copy of that switch is a second place to disagree.
    //
    // Zero-initialized because the dense pass writes only the DOFs of links that HAVE an inbound
    // joint -- a fixed joint contributes none, and an unwritten slot must read as 0.
    const size_t denseFloats = size_t(getCount()) * mMaxDofs;
    if (denseFloats == 0)
    {
        return true;
    }
    // Sized once and reused: `assign` keeps the capacity a previous read grew and re-zeroes in the
    // same pass the zero-fill above already requires.
    mDofStagingRow.assign(denseFloats, 0.0f);
    std::vector<float>& dense = mDofStagingRow;
    TensorDesc denseDesc;
    denseDesc.device = -1;
    denseDesc.dtype = omni::physics::tensors::TensorDataType::eFloat32;
    denseDesc.numDims = 1;
    denseDesc.dims[0] = static_cast<int64_t>(denseFloats);
    denseDesc.data = dense.data();
    if (!getDofProjectedJointForces(&denseDesc))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < numOutputs; i++)
    {
        const ArticulationDofOvStageRecord& r = records[i];
        const size_t src = size_t(r.viewArtiIdx) * mMaxDofs + r.physxDofIdx;
        // No fold on top: see the row comment in kJointAttributes. The dense pass resolved the
        // joint frame and the body order while projecting, so scaling again would apply the sign
        // twice.
        dst[i] = (src < denseFloats) ? dense[src] : 0.0f;
    }
    return true;
}

bool CpuArticulationView::getLinkIncomingJointForcesOvStage(const ArticulationLinkOvStageRecord* records,
                                                            PxU32 numOutputs, const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }
    if (numOutputs == 0)
    {
        return true;
    }
    if (!records)
    {
        CARB_LOG_ERROR("link incoming joint force (ovstage): null ovstage link records");
        return false;
    }
    const char* label = "linkIncomingJointForce";
    if (!checkTensorDevice(*dstTensor, -1, label, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, label, __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, numOutputs * 6u, label, __FUNCTION__))
    {
        return false;
    }

    // A dense staging row, then a gather -- the shape the GPU side uses, for the same reason: reusing
    // the tensor API's dense getter keeps the frame and sign convention to one implementation per
    // backend.
    //
    // Zero-initialized because THIS backend's dense pass writes only the links an articulation
    // actually has: the view's rows are padded to the widest articulation, and an unwritten slot must
    // read as 0. The GPU sibling needs no memset -- its kernel writes every slot of the padded row.
    const size_t denseFloats = size_t(getCount()) * mMaxLinks * 6u;
    if (denseFloats == 0)
    {
        return true;
    }
    // Reused across reads, like the DOF row above and like the GPU sibling's persistent scratch.
    mLinkStagingRow.assign(denseFloats, 0.0f);
    std::vector<float>& dense = mLinkStagingRow;
    TensorDesc denseDesc;
    denseDesc.device = -1;
    denseDesc.dtype = omni::physics::tensors::TensorDataType::eFloat32;
    denseDesc.numDims = 1;
    denseDesc.dims[0] = static_cast<int64_t>(denseFloats);
    denseDesc.data = dense.data();
    if (!getLinkIncomingJointForce(&denseDesc))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < numOutputs; i++)
    {
        const ArticulationLinkOvStageRecord& r = records[i];
        const size_t src = (size_t(r.viewArtiIdx) * mMaxLinks + r.physxLinkIdx) * 6u;
        for (PxU32 c = 0; c < 6u; ++c)
            dst[i * 6u + c] = (src + c < denseFloats) ? dense[src + c] : 0.0f;
    }
    return true;
}

bool CpuArticulationView::setDofPositions(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "DOF position", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "DOF position", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxDofs, "DOF position", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            PxArticulationReducedCoordinate* arti = mEntries[idx].arti;
            PxArticulationCache* cache = mArticulationCaches[idx];
            if (arti && cache)
            {
                const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxDofs;
                for (PxU32 j = 0; j < mEntries[idx].numDofs; j++)
                {
                    cache->jointPosition[j] = mEntries[idx].metatype->isDofBody0Parent(j) ? src[j] : -src[j];
                }

                arti->applyCache(*cache, PxArticulationCacheFlag::ePOSITION);
            }
        }
    }

    return true;
}

bool CpuArticulationView::setDofVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "DOF velocity", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "DOF velocity", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxDofs, "DOF velocity", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            PxArticulationReducedCoordinate* arti = mEntries[idx].arti;
            PxArticulationCache* cache = mArticulationCaches[idx];
            if (arti && cache)
            {
                const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxDofs;
                for (PxU32 j = 0; j < mEntries[idx].numDofs; j++)
                {
                    cache->jointVelocity[j] = mEntries[idx].metatype->isDofBody0Parent(j) ? src[j] : -src[j];
                }
                arti->applyCache(*cache, PxArticulationCacheFlag::eVELOCITY);
            }
        }
    }

    return true;
}

bool CpuArticulationView::setDofActuationForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "DOF force", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "DOF force", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxDofs, "DOF force", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            PxArticulationReducedCoordinate* arti = mEntries[idx].arti;
            PxArticulationCache* cache = mArticulationCaches[idx];
            if (arti && cache)
            {
                const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxDofs;
                for (PxU32 j = 0; j < mEntries[idx].numDofs; j++)
                {
                    cache->jointForce[j] = mEntries[idx].metatype->isDofBody0Parent(j) ? src[j] : -src[j];
                }
                arti->applyCache(*cache, PxArticulationCacheFlag::eFORCE);
            }
        }
    }

    return true;
}

bool CpuArticulationView::setDofPositionTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "DOF target", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "DOF target", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxDofs, "DOF target", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            ArticulationEntry& entry = mEntries[idx];
            const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxDofs;
            for (PxU32 j = 0; j < entry.numDofs; j++)
            {
                const DofImpl& dofImpl = entry.dofImpls[j];
                if (dofImpl.joint)
                {
                    dofImpl.joint->setDriveTarget(
                        dofImpl.axis, mEntries[idx].metatype->isDofBody0Parent(j) ? src[j] : -src[j]);
                }
            }
        }
    }

    return true;
}

bool CpuArticulationView::setDofVelocityTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);

    if (!srcTensor || !srcTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*srcTensor, -1, "DOF target", __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, "DOF target", __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, getCount() * mMaxDofs, "DOF target", __FUNCTION__))
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            ArticulationEntry& entry = mEntries[idx];
            const float* src = static_cast<const float*>(srcTensor->data) + idx * mMaxDofs;
            for (PxU32 j = 0; j < entry.numDofs; j++)
            {
                const DofImpl& dofImpl = entry.dofImpls[j];
                if (dofImpl.joint)
                {
                    dofImpl.joint->setDriveVelocity(dofImpl.axis, mEntries[idx].metatype->isDofBody0Parent(j) ? src[j] : -src[j]);
                }
            }
        }
    }

    return true;
}

bool CpuArticulationView::getDofPositionTargets(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "DOF target", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "DOF target", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "DOF target", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs;
        for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
        {
            const DofImpl& dofImpl = mEntries[i].dofImpls[j];
            if (dofImpl.joint)
            {
                dst[j] = mEntries[i].metatype->isDofBody0Parent(j) ? dofImpl.joint->getDriveTarget(dofImpl.axis) :
                                                                      -dofImpl.joint->getDriveTarget(dofImpl.axis);
            }
            else
            {
                dst[j] = 0.0f;
            }
        }
    }

    return true;
}

bool CpuArticulationView::getDofVelocityTargets(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "DOF target", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "DOF target", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "DOF target", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs;
        for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
        {
            const DofImpl& dofImpl = mEntries[i].dofImpls[j];
            if (dofImpl.joint)
            {
                dst[j] = mEntries[i].metatype->isDofBody0Parent(j) ? dofImpl.joint->getDriveVelocity(dofImpl.axis) :
                                                                      -dofImpl.joint->getDriveVelocity(dofImpl.axis);
            }
            else
            {
                dst[j] = 0.0f;
            }
        }
    }

    return true;
}


bool CpuArticulationView::getDofActuationForces(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "dof actuation force", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "dof actuation force", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "dof actuation force", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];
        if (arti && cache)
        {
            arti->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eFORCE);
            float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs;
            for (PxU32 j = 0; j < mEntries[i].numDofs; j++)
            {
                dst[j] = mEntries[i].metatype->isDofBody0Parent(j) ? cache->jointForce[j] : -cache->jointForce[j];
            }
        }
    }

    return true;
}


bool CpuArticulationView::getJacobians(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    uint32_t jacobianRows = 0;
    uint32_t jacobianCols = 0;

    // this will fail if view is not homogeneous
    if (!getJacobianShape(&jacobianRows, &jacobianCols))
    {
        return false;
    }

    uint32_t jacobianSize = jacobianRows * jacobianCols;

    if (!checkTensorDevice(*dstTensor, -1, "Jacobian", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Jacobian", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * jacobianSize, "Jacobian", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];

        PxU32 nRows, nCols;
        arti->commonInit();
        arti->computeDenseJacobian(*cache, nRows, nCols);

        // safety check: this could fail if articulation structure or fixed base changed
        if (nRows == jacobianRows && nCols == jacobianCols)
        {
            const PxU32 rootDofs = mEntries[i].metatype->getFixedBase() ? 0u : 6u;
            copyJacobianToAuthoredDofBasis(dst, cache->denseJacobian, nRows, nCols, rootDofs, *mEntries[i].metatype);
        }
        else
        {
            CARB_LOG_ERROR("Invalid Jacobian shape for destination tensor!");
            memset(dst, 0, jacobianSize * sizeof(float));
        }

        dst += jacobianSize;
    }

    return true;
}

bool CpuArticulationView::getGeneralizedMassMatrices(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    const bool isFixedBase = mEntries[0].metatype->getFixedBase();
    const PxU32 maxDofs = isFixedBase ? mMaxDofs : mMaxDofs + 6;

    uint32_t massMatrixRows = 0;
    uint32_t massMatrixCols = 0;

    // this will fail if view is not homogeneous
    if (!getGeneralizedMassMatrixShape(&massMatrixRows, &massMatrixCols))
    {
        return false;
    }

    uint32_t massMatrixSize = massMatrixRows * massMatrixCols;

    if (!checkTensorDevice(*dstTensor, -1, "Mass Matrix", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Mass Matrix", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * massMatrixSize, "Mass Matrix", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];
        arti->commonInit();
        arti->computeMassMatrix(*cache);
        const PxU32 rootDofs = isFixedBase ? 0u : 6u;
        copyMassMatrixToAuthoredDofBasis(dst, cache->massMatrix, maxDofs, rootDofs, *mEntries[i].metatype);

        dst += massMatrixSize;
    }

    return true;
}

bool CpuArticulationView::getCoriolisAndCentrifugalCompensationForces(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!requireUniformBaseType(__FUNCTION__))
    {
        return false;
    }

    const bool isFixedBase = mEntries[0].metatype->getFixedBase();
    const PxU32 maxDofs = isFixedBase ? mMaxDofs : mMaxDofs + 6;

    if (!checkTensorDevice(*dstTensor, -1, "coriolis and centrifugal forces", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "coriolis and centrifugal forces", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * maxDofs, "coriolis and centrifugal forces", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];

        arti->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eVELOCITY);
        arti->commonInit();
        arti->computeCoriolisCompensation(*cache);
        if (isFixedBase)
        {
            const PxU32 numDofs = mEntries[i].numDofs;
            for (PxU32 j = 0; j < numDofs; j++)
            {
                dst[j] = mEntries[i].metatype->isDofBody0Parent(j) ? cache->coriolisForce[j] : -cache->coriolisForce[j];
            }
            for (PxU32 j = numDofs; j < mMaxDofs; j++)
            {
                dst[j] = 0.0f;
            }
        }
        else
        {
            for (PxU32 j = 0; j < 6; j++)
            {
                dst[j] = cache->coriolisForce[j];
            }
            const PxU32 numDofs = mEntries[i].numDofs;
            for (PxU32 j = 6; j < 6 + numDofs; j++)
            {
                dst[j] = mEntries[i].metatype->isDofBody0Parent(j - 6) ? cache->coriolisForce[j] : -cache->coriolisForce[j];
            }
            for (PxU32 j = 6 + numDofs; j < maxDofs; j++)
            {
                dst[j] = 0.0f;
            }
        }

        dst += maxDofs;
    }

    return true;
}

bool CpuArticulationView::getGravityCompensationForces(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!requireUniformBaseType(__FUNCTION__))
    {
        return false;
    }

    const bool isFixedBase = mEntries[0].metatype->getFixedBase();
    const PxU32 maxDofs = isFixedBase ? mMaxDofs : mMaxDofs + 6;

    if (!checkTensorDevice(*dstTensor, -1, "gravity compensation forces", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "gravity compensation forces", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * maxDofs, "gravity compensation forces", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];

        arti->commonInit();
        arti->computeGravityCompensation(*cache);
        if (isFixedBase)
        {
            const PxU32 numDofs = mEntries[i].numDofs;
            for (PxU32 j = 0; j < numDofs; j++)
            {
                dst[j] = mEntries[i].metatype->isDofBody0Parent(j) ? cache->gravityCompensationForce[j] : -cache->gravityCompensationForce[j];
            }
            for (PxU32 j = numDofs; j < mMaxDofs; j++)
            {
                dst[j] = 0.0f;
            }
        }
        else
        {
            for (PxU32 j = 0; j < 6; j++)
            {
                dst[j] = cache->gravityCompensationForce[j];
            }
            const PxU32 numDofs = mEntries[i].numDofs;
            for (PxU32 j = 6; j < 6 + numDofs; j++)
            {
                dst[j] = mEntries[i].metatype->isDofBody0Parent(j - 6) ? cache->gravityCompensationForce[j] : -cache->gravityCompensationForce[j];
            }
            for (PxU32 j = 6 + numDofs; j < maxDofs; j++)
            {
                dst[j] = 0.0f;
            }
        }

        dst += maxDofs;
    }

    return true;
}

bool CpuArticulationView::getArticulationMassCenter(const TensorDesc* dstTensor, bool localFrame) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Articulation Mass Center", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Articulation Mass Center", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 3, "Articulation Mass Center", __FUNCTION__))
    {
        return false;
    }

    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxVec3 center = mEntries[i].arti->computeArticulationCOM(localFrame);
        if (!localFrame)
        {
            const Subspace* const subspace = mEntries[i].subspace;
            if (subspace)
            {
                center -= PxVec3(subspace->origin.x, subspace->origin.y, subspace->origin.z);
            }
        }
        dst[size_t(i) * 3 + 0] = center.x;
        dst[size_t(i) * 3 + 1] = center.y;
        dst[size_t(i) * 3 + 2] = center.z;
    }

    return true;
}

bool CpuArticulationView::getArticulationCentroidalMomentum(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!requireUniformBaseType(__FUNCTION__))
    {
        return false;
    }

    // Heterogeneous views are not supported here, and unlike the per-DOF readers this one cannot
    // degrade to padding: PhysX sizes centroidalMomentumMatrix 6 x (own dofs + 6), so a view whose
    // articulations differ has no single row stride to read it with.
    if (!isHomogeneous())
    {
        CARB_LOG_ERROR("%s: the articulations in this view are not homogeneous. Centroidal momentum is read with one row stride for the whole view, which cannot describe articulations of differing size. Build one view per articulation type.",
                       __FUNCTION__);
        return false;
    }

    const bool isFixedBase = mEntries[0].metatype->getFixedBase();
    if (isFixedBase)
    {
        CARB_LOG_ERROR("Articulation has fixed base, centroidal momentum is not defined");
        return false;
    }

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Articulation Centroidal Momentum", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Articulation Centroidal Momentum", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * 6 * (mMaxDofs + 7), "Articulation Centroidal Momentum", __FUNCTION__))
    {
        return false;
    }


    PxReal* dst = (PxReal*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];
        arti->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eVELOCITY);
        arti->commonInit();
        arti->computeMassMatrix(*cache);
        arti->computeCoriolisCompensation(*cache);
        arti->computeCentroidalMomentumMatrix(*cache);
        const PxU32 startIdx = i * 6 * (mMaxDofs + 7);
        copyCentroidalMomentumToAuthoredDofBasis(dst + startIdx, cache->centroidalMomentumMatrix,
                                                 cache->centroidalMomentumBias, mMaxDofs, *mEntries[i].metatype);
    }

    return true;
}

static void transformToParentFrame(const PxArticulationJointReducedCoordinate* joint, PxVec3& force, PxVec3& torque)
{
    // joint child/parent frame representation in the global frame
    PxTransform GpLp = joint->getParentArticulationLink().getGlobalPose() * joint->getParentPose();
    PxTransform GcLc = joint->getChildArticulationLink().getGlobalPose() * joint->getChildPose();
    // child to parent rotation and translation
    PxQuat J = GpLp.q.getConjugate() * GcLc.q;
    PxVec3 d = GpLp.p - GcLc.p; // global frame
    d = GcLc.q.rotateInv(d); // local frame
    // torque needs further modification before transforming to the parent frame
    torque = -1.0f * J.rotate(torque - d.cross(force));
    force = -1.0f * J.rotate(force);
}

bool CpuArticulationView::getLinkIncomingJointForce(const TensorDesc* dstTensor) const
{
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "link incoming joint force", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "link incoming joint force", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxLinks * 6u, "link incoming joint force", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];
        if (arti && cache)
        {
            arti->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eLINK_INCOMING_JOINT_FORCE);
            float* dst = static_cast<float*>(dstTensor->data) + i * mMaxLinks * 6;
            for (PxU32 j = 0; j < mEntries[i].numLinks; ++j)
            {
                PxArticulationLink* link = mEntries[i].links[j];
                bool isJointBody0Parent = mEntries[i].isIncomingJointBody0Parent[j];
                PxQuat physxToUsdRotation = mEntries[i].incomingJointPhysxToUsdRotations[j];
                CARB_ASSERT(physxToUsdRotation.isUnit());

                PxSpatialForce sf = cache->linkIncomingJointForce[j];
                if (!isJointBody0Parent)
                {
                    PxArticulationJointReducedCoordinate* joint = link->getInboundJoint();
                    if (joint)
                    {
                        transformToParentFrame(joint, sf.force, sf.torque);
                    }
                }
                PxVec3 usdSpaceForce = physxToUsdRotation.rotate(sf.force);
                PxVec3 usdSpaceTorque = physxToUsdRotation.rotate(sf.torque);

                *dst++ = usdSpaceForce.x;
                *dst++ = usdSpaceForce.y;
                *dst++ = usdSpaceForce.z;
                *dst++ = usdSpaceTorque.x;
                *dst++ = usdSpaceTorque.y;
                *dst++ = usdSpaceTorque.z;
            }
        }
    }

    return true;
}

bool CpuArticulationView::getDofProjectedJointForces(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "dof projected joint force", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "dof projected joint force", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxDofs, "dof projected joint force", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        PxArticulationReducedCoordinate* arti = mEntries[i].arti;
        PxArticulationCache* cache = mArticulationCaches[i];

        if (arti && cache)
        {
            arti->copyInternalStateToCache(*cache, PxArticulationCacheFlag::eLINK_INCOMING_JOINT_FORCE);
            float* dst = static_cast<float*>(dstTensor->data) + i * mMaxDofs;
            for (PxU32 j = 0; j < mEntries[i].numLinks; ++j)
            {
                PxArticulationLink* link = mEntries[i].links[j];
                PxArticulationJointReducedCoordinate* joint = link->getInboundJoint();
                PxU32 offset = mEntries[i].dofStarts[j];
                if (joint)
                {
                    PxSpatialForce sf = cache->linkIncomingJointForce[j];
                    PxArticulationJointType::Enum jointType = joint->getJointType();
                    if (!mEntries[i].isIncomingJointBody0Parent[j])
                    {
                        transformToParentFrame(joint, sf.force, sf.torque);
                    }
                    switch (jointType)
                    {
                    case PxArticulationJointType::eUNDEFINED:
                        CARB_LOG_ERROR("Undefined joint type encountered while calculating forces along DOFs");
                        break;
                    case PxArticulationJointType::eFIX:
                        break;
                    case PxArticulationJointType::eREVOLUTE:
                        dst[offset] = sf.torque.x;
                        break;
                    case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
                        dst[offset] = sf.torque.x;
                        break;
                    case PxArticulationJointType::ePRISMATIC:
                        dst[offset] = sf.force.x;
                        break;
                    case PxArticulationJointType::eSPHERICAL:
                        PxQuat physxToUsdRotation = mEntries[i].incomingJointPhysxToUsdRotations[j];
                        CARB_ASSERT(physxToUsdRotation.isUnit());
                        PxVec3 usdSpaceTorque = physxToUsdRotation.rotate(sf.torque);
                        PxU32 DofIdx = 0;
                        if(mEntries[i].freeD6Axes[j] & FreeD6RotationAxesFlag::eTWIST)
                            dst[offset + DofIdx++] = usdSpaceTorque.x;

                        if(mEntries[i].freeD6Axes[j] & FreeD6RotationAxesFlag::eSWING1)
                            dst[offset + DofIdx++] = usdSpaceTorque.y;

                        if(mEntries[i].freeD6Axes[j] & FreeD6RotationAxesFlag::eSWING2)
                            dst[offset + DofIdx++] = usdSpaceTorque.z;
                        break;
                    }
                }
            }
        }
    }

    return true;
}

void CpuArticulationView::prepareDirtyForceTracker()
{
    if (!mDirtyForceTracker)
    {
        mDirtyForceTracker = std::make_shared<CpuRigidBodyDirtyForceTracker>();

        PxU32 numBodies = getCount() * mMaxLinks;
        mDirtyForceTracker->bodies.resize(numBodies);
        mDirtyForceTracker->dirtyFlags.resize(numBodies);

        for (PxU32 i = 0; i < getCount(); i++)
        {
            for (PxU32 j = 0; j < mEntries[i].numLinks; j++)
            {
                mDirtyForceTracker->bodies[i * mMaxLinks + j] = mEntries[i].links[j];
            }
        }

        if (mCpuSimData)
        {
            mCpuSimData->addRigidBodyDirtyForceTracker(mDirtyForceTracker);
        }
    }
}

bool CpuArticulationView::applyForcesAndTorquesAtPosition(const TensorDesc* srcForceTensor,
                                                          const TensorDesc* srcTorqueTensor,
                                                          const TensorDesc* srcPositionTensor,
                                                          const TensorDesc* indexTensor,
                                                          const bool isGlobal)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);

    bool validForceTensor = false;
    bool validTorqueTensor = false;
    bool validPositionTensor = false;
    bool hasForce = srcForceTensor && srcForceTensor->data;
    bool hasTorque = srcTorqueTensor && srcTorqueTensor->data;
    bool hasPosition = srcPositionTensor && srcPositionTensor->data;
    if (!hasForce && !hasTorque){
        CARB_LOG_WARN("No force or torque tensor is provided\n.");
        return false;
    }

    if (hasForce)
        validForceTensor = checkTensorDevice(*srcForceTensor, -1, "force", __FUNCTION__) &&
                           checkTensorFloat32(*srcForceTensor, "force", __FUNCTION__) &&
                           checkTensorSizeExact(*srcForceTensor, getCount() * mMaxLinks * 3u, "force", __FUNCTION__);

    if (hasTorque)
        validTorqueTensor = checkTensorDevice(*srcTorqueTensor, -1, "torque", __FUNCTION__) &&
                            checkTensorFloat32(*srcTorqueTensor, "torque", __FUNCTION__) &&
                            checkTensorSizeExact(*srcTorqueTensor, getCount() * mMaxLinks * 3u, "torque", __FUNCTION__);

    if (!validForceTensor && !validTorqueTensor)
    {
        CARB_LOG_WARN("No correct force or torque tensor is provided\n.");
        return false;
    }

    if (hasPosition)
    {
        if (!validForceTensor)
        {
            CARB_LOG_ERROR("Received a position tensor wihtout a compatible force tensor.");
            return false;
        }
        validPositionTensor = checkTensorDevice(*srcPositionTensor, -1, "position", __FUNCTION__) &&
                              checkTensorFloat32(*srcPositionTensor, "position", __FUNCTION__) &&
                              checkTensorSizeExact(*srcPositionTensor, getCount() * mMaxLinks * 3u, "position", __FUNCTION__);
        if (!validPositionTensor)
            return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    prepareDirtyForceTracker();

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            ArticulationEntry& entry = mEntries[idx];
            for (PxU32 j = 0; j < mEntries[i].numLinks; j++)
            {
                if (validForceTensor)
                {
                    const float* src = static_cast<const float*>(srcForceTensor->data) + (idx * mMaxLinks + j) * 3;
                    PxVec3 force(src[0], src[1], src[2]);
                    if (!isGlobal)
                    {
                        // translate force vector into global space
                        PxTransform pose = entry.links[j]->getGlobalPose();
                        force = pose.q.rotate(force);
                    }
                    entry.links[j]->addForce(force);
                    mDirtyForceTracker->dirtyFlags[idx * mMaxLinks + j] |= RigidBodyDirtyForceFlags::eForce;
                    if (validPositionTensor)
                    {
                        PxTransform pose = entry.links[j]->getGlobalPose();
                        const PxVec3 com = pose.transform(entry.links[j]->getCMassLocalPose().p);
                        const float* srcP = static_cast<const float*>(srcPositionTensor->data) + (idx * mMaxLinks + j) * 3;
                        PxVec3 position(srcP[0], srcP[1], srcP[2]);
                        if (!isGlobal)
                            position = pose.transform(position);
                        PxVec3 tmp = (position - com).cross(force);
                        entry.links[j]->addTorque((position - com).cross(force));
                        mDirtyForceTracker->dirtyFlags[idx * mMaxLinks + j] |= RigidBodyDirtyForceFlags::eTorque;
                    }
                }
                if (validTorqueTensor)
                {
                    const float* src = static_cast<const float*>(srcTorqueTensor->data) + (idx * mMaxLinks + j) * 3;
                    PxVec3 torque;
                    torque.x = src[0];
                    torque.y = src[1];
                    torque.z = src[2];
                    if (!isGlobal)
                    {
                        // translate force vector into global space
                        PxTransform pose = entry.links[j]->getGlobalPose();
                        torque = pose.q.rotate(torque);
                    }
                    entry.links[j]->addTorque(torque);
                    mDirtyForceTracker->dirtyFlags[idx * mMaxLinks + j] |= RigidBodyDirtyForceFlags::eTorque;
                }
            }
        }
    }

    mDirtyForceTracker->isDirty = true;

    return true;
}

bool CpuArticulationView::getFixedTendonPropertiesOvStage(const ArticulationTendonOvStageRecord* records,
                                                          PxU32 numOutputs, TendonProperty prop,
                                                          const TensorDesc* dstTensor) const
{
    return getTendonPropertiesOvStage("fixed tendon property (ovstage)", dstTensor, /*fixed=*/true, prop,
                                      records, numOutputs);
}

bool CpuArticulationView::getSpatialTendonPropertiesOvStage(const ArticulationTendonOvStageRecord* records,
                                                            PxU32 numOutputs, TendonProperty prop,
                                                            const TensorDesc* dstTensor) const
{
    return getTendonPropertiesOvStage("spatial tendon property (ovstage)", dstTensor, /*fixed=*/false, prop,
                                      records, numOutputs);
}

bool CpuArticulationView::setFixedTendonPropertiesOvStage(const TensorDesc* srcTensor,
                                                          const ArticulationTendonOvStageRecord* records,
                                                          PxU32 numOutputs, TendonProperty prop)
{
    return setTendonPropertiesOvStage("fixed tendon property (ovstage write)", srcTensor, /*fixed=*/true, prop,
                                      records, numOutputs);
}

bool CpuArticulationView::setSpatialTendonPropertiesOvStage(const TensorDesc* srcTensor,
                                                            const ArticulationTendonOvStageRecord* records,
                                                            PxU32 numOutputs, TendonProperty prop)
{
    return setTendonPropertiesOvStage("spatial tendon property (ovstage write)", srcTensor, /*fixed=*/false, prop,
                                      records, numOutputs);
}

// The inverse of getTendonPropertiesOvStage, and deliberately its mirror line for line: same
// validation, same record walk, same switch. NO SCALE in either direction -- see
// ArticulationTendonOvStageRecord.h for why the rad2deg fold lives in the gearing coefficient and
// everything downstream of it is passthrough.
bool CpuArticulationView::setTendonPropertiesOvStage(const char* attribName, const TensorDesc* srcTensor,
                                                     bool fixed, TendonProperty prop,
                                                     const ArticulationTendonOvStageRecord* records,
                                                     PxU32 numOutputs)
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(srcTensor);
    if (!srcTensor || !srcTensor->data)
        return false;
    if (numOutputs == 0)
        return true; // nothing to publish -- not a failure
    if (!records)
    {
        CARB_LOG_ERROR("%s: null ovstage tendon records", attribName);
        return false;
    }
    // Same guard as the getter, and it is a property of the function rather than the call site for
    // the same reason: the switch below static_casts to PxArticulationFixedTendon, which is
    // undefined on a spatial tendon. The schema makes the case impossible -- a spatial tendon's
    // limit and rest length live on its LEAF attachment.
    if (!fixed && (prop == TendonProperty::eLimit || prop == TendonProperty::eRestLength))
    {
        CARB_LOG_ERROR("%s: spatial tendons have no limit or rest length", attribName);
        return false;
    }
    const PxU32 comp = tendonPropertyComponents(prop);
    if (!checkTensorDevice(*srcTensor, -1, attribName, __FUNCTION__) ||
        !checkTensorFloat32(*srcTensor, attribName, __FUNCTION__) ||
        !checkTensorSizeExact(*srcTensor, numOutputs * comp, attribName, __FUNCTION__))
    {
        return false;
    }

    const float* src = static_cast<const float*>(srcTensor->data);
    for (PxU32 i = 0; i < numOutputs; ++i)
    {
        const ArticulationTendonOvStageRecord& r = records[i];
        // Out of range FAILS rather than being skipped, matching the getter: the only way to land
        // here is a record list that does not describe this view, and carrying on would write some
        // of the caller's values and silently drop the rest.
        if (r.viewArtiIdx >= mEntries.size())
        {
            CARB_LOG_ERROR("%s: record %u names articulation row %u of %zu", attribName, i, r.viewArtiIdx,
                           mEntries.size());
            return false;
        }
        const ArticulationEntry& ent = mEntries[r.viewArtiIdx];
        PxArticulationTendon* tendon = nullptr;
        if (fixed)
        {
            if (r.tendonIdx >= ent.fixedTendons.size())
            {
                CARB_LOG_ERROR("%s: record %u names fixed tendon %u of %zu", attribName, i, r.tendonIdx,
                               ent.fixedTendons.size());
                return false;
            }
            tendon = ent.fixedTendons[r.tendonIdx];
        }
        else
        {
            if (r.tendonIdx >= ent.spatialTendons.size())
            {
                CARB_LOG_ERROR("%s: record %u names spatial tendon %u of %zu", attribName, i, r.tendonIdx,
                               ent.spatialTendons.size());
                return false;
            }
            tendon = ent.spatialTendons[r.tendonIdx];
        }
        if (!tendon)
        {
            CARB_LOG_ERROR("%s: record %u names a null tendon", attribName, i);
            return false;
        }

        switch (prop)
        {
        case TendonProperty::eStiffness:
            tendon->setStiffness(src[i]);
            break;
        case TendonProperty::eDamping:
            tendon->setDamping(src[i]);
            break;
        case TendonProperty::eLimitStiffness:
            tendon->setLimitStiffness(src[i]);
            break;
        case TendonProperty::eOffset:
            tendon->setOffset(src[i]);
            break;
        case TendonProperty::eLimit:
        {
            PxArticulationTendonLimit limit;
            limit.lowLimit = src[i * 2 + 0];
            limit.highLimit = src[i * 2 + 1];
            static_cast<PxArticulationFixedTendon*>(tendon)->setLimitParameters(limit);
            break;
        }
        case TendonProperty::eRestLength:
            static_cast<PxArticulationFixedTendon*>(tendon)->setRestLength(src[i]);
            break;
        }
    }
    return true;
}

bool CpuArticulationView::getTendonPropertiesOvStage(const char* attribName, const TensorDesc* dstTensor,
                                                     bool fixed, TendonProperty prop,
                                                     const ArticulationTendonOvStageRecord* records,
                                                     PxU32 numOutputs) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }
    if (numOutputs == 0)
    {
        return true; // nothing to emit -- not a failure
    }
    if (!records)
    {
        CARB_LOG_ERROR("%s: null ovstage tendon records", attribName);
        return false;
    }
    // Guarded in the worker, not at the call sites: the switch below reaches
    // static_cast<const PxArticulationFixedTendon*>(tendon) for these two, which is undefined on a
    // spatial tendon. The schema is what makes the case impossible to serve -- limit and rest length
    // live on the spatial tendon's LEAF attachment, not on the tendon.
    if (!fixed && (prop == TendonProperty::eLimit || prop == TendonProperty::eRestLength))
    {
        CARB_LOG_ERROR("%s: spatial tendons have no limit or rest length", attribName);
        return false;
    }
    const PxU32 comp = tendonPropertyComponents(prop);
    if (!checkTensorDevice(*dstTensor, -1, attribName, __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, attribName, __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, numOutputs * comp, attribName, __FUNCTION__))
    {
        return false;
    }

    // Straight off the PhysX objects the view already holds per entry -- no cache to populate and
    // nothing to fetch, unlike the DOF path. The records name (view row, tendon slot) and the entry
    // owns the tendon pointers, so the whole read is a pointer chase per output.
    //
    // An out-of-range record FAILS here rather than zero-filling the slot as the DOF path does: a
    // tendon read needs no per-articulation cache, so the only way to land out of range is a record
    // list that does not describe this view, and zero-filling would hand back plausible zeros
    // instead of reporting that the records and the view disagree.
    float* dst = static_cast<float*>(dstTensor->data);
    for (PxU32 i = 0; i < numOutputs; ++i)
    {
        const ArticulationTendonOvStageRecord& r = records[i];
        if (r.viewArtiIdx >= mEntries.size())
        {
            CARB_LOG_ERROR("%s: record %u names articulation row %u of %zu", attribName, i, r.viewArtiIdx,
                           mEntries.size());
            return false;
        }
        const ArticulationEntry& ent = mEntries[r.viewArtiIdx];
        const PxArticulationTendon* tendon = nullptr;
        if (fixed)
        {
            if (r.tendonIdx >= ent.fixedTendons.size())
            {
                CARB_LOG_ERROR("%s: record %u names fixed tendon %u of %zu", attribName, i, r.tendonIdx,
                               ent.fixedTendons.size());
                return false;
            }
            tendon = ent.fixedTendons[r.tendonIdx];
        }
        else
        {
            if (r.tendonIdx >= ent.spatialTendons.size())
            {
                CARB_LOG_ERROR("%s: record %u names spatial tendon %u of %zu", attribName, i, r.tendonIdx,
                               ent.spatialTendons.size());
                return false;
            }
            tendon = ent.spatialTendons[r.tendonIdx];
        }
        if (!tendon)
        {
            CARB_LOG_ERROR("%s: record %u names a null tendon", attribName, i);
            return false;
        }

        switch (prop)
        {
        case TendonProperty::eStiffness:
            dst[i] = tendon->getStiffness();
            break;
        case TendonProperty::eDamping:
            dst[i] = tendon->getDamping();
            break;
        case TendonProperty::eLimitStiffness:
            dst[i] = tendon->getLimitStiffness();
            break;
        case TendonProperty::eOffset:
            dst[i] = tendon->getOffset();
            break;
        case TendonProperty::eLimit:
        {
            const PxArticulationTendonLimit limit =
                static_cast<const PxArticulationFixedTendon*>(tendon)->getLimitParameters();
            dst[i * 2 + 0] = limit.lowLimit;
            dst[i * 2 + 1] = limit.highLimit;
            break;
        }
        case TendonProperty::eRestLength:
            dst[i] = static_cast<const PxArticulationFixedTendon*>(tendon)->getRestLength();
            break;
        }
    }

    return true;
}

bool CpuArticulationView::getFixedTendonStiffnesses(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Tendon stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Tendon stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxFixedTendons, "Tendon stiffness", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numFixedTendons; j++)
        {
            *dst++ = mEntries[i].fixedTendons[j]->getStiffness();
        }
    }

    return true;
}

bool CpuArticulationView::getFixedTendonDampings(const TensorDesc* dstTensor) const
{
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Tendon damping", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Tendon damping", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxFixedTendons, "Tendon damping", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numFixedTendons; j++)
        {
            *dst++ = mEntries[i].fixedTendons[j]->getDamping();
        }
    }

    return true;
}

bool CpuArticulationView::getFixedTendonLimitStiffnesses(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Tendon limit stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Tendon limit stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxFixedTendons, "Tendon limit stiffness", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numFixedTendons; j++)
        {
            *dst++ = mEntries[i].fixedTendons[j]->getLimitStiffness();
        }
    }

    return true;
}

bool CpuArticulationView::getFixedTendonLimits(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Tendon limits", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Tendon limits", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxFixedTendons * 2, "Tendon limits", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numFixedTendons; j++)
        {
            *dst++ = mEntries[i].fixedTendons[j]->getLimitParameters().lowLimit;
            *dst++ = mEntries[i].fixedTendons[j]->getLimitParameters().highLimit;
        }
    }

    return true;
}

bool CpuArticulationView::getFixedTendonfixedSpringRestLengths(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Tendon rest length", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Tendon rest length", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxFixedTendons, "Tendon rest length", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numFixedTendons; j++)
        {
            *dst++ = mEntries[i].fixedTendons[j]->getRestLength();
        }
    }

    return true;
}

bool CpuArticulationView::getFixedTendonOffsets(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Tendon offset", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Tendon offset", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxFixedTendons, "Tendon offset", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numFixedTendons; j++)
        {
            *dst++ = mEntries[i].fixedTendons[j]->getOffset();
        }
    }

    return true;
}

bool CpuArticulationView::getSpatialTendonStiffnesses(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Tendon stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Tendon stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxSpatialTendons, "Tendon stiffness", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numSpatialTendons; j++)
        {
            *dst++ = mEntries[i].spatialTendons[j]->getStiffness();
        }
    }

    return true;
}

bool CpuArticulationView::getSpatialTendonDampings(const TensorDesc* dstTensor) const
{
    PASS_EMPTY_TENSOR(dstTensor);
    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Tendon damping", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Tendon damping", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxSpatialTendons, "Tendon damping", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numSpatialTendons; j++)
        {
            *dst++ = mEntries[i].spatialTendons[j]->getDamping();
        }
    }

    return true;
}

bool CpuArticulationView::getSpatialTendonLimitStiffnesses(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Tendon limit stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Tendon limit stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxSpatialTendons, "Tendon limit stiffness", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numSpatialTendons; j++)
        {
            *dst++ = mEntries[i].spatialTendons[j]->getLimitStiffness();
        }
    }

    return true;
}

bool CpuArticulationView::getSpatialTendonOffsets(const TensorDesc* dstTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(dstTensor);

    if (!dstTensor || !dstTensor->data)
    {
        return false;
    }

    if (!checkTensorDevice(*dstTensor, -1, "Tendon offset", __FUNCTION__) ||
        !checkTensorFloat32(*dstTensor, "Tendon offset", __FUNCTION__) ||
        !checkTensorSizeExact(*dstTensor, getCount() * mMaxSpatialTendons, "Tendon offset", __FUNCTION__))
    {
        return false;
    }

    float* dst = (float*)dstTensor->data;
    for (PxU32 i = 0; i < mEntries.size(); i++)
    {
        for (PxU32 j = 0; j < mEntries[i].numSpatialTendons; j++)
        {
            *dst++ = mEntries[i].spatialTendons[j]->getOffset();
        }
    }

    return true;
}

bool CpuArticulationView::setFixedTendonProperties(const TensorDesc* stiffnesses, const TensorDesc* dampings, const TensorDesc* limitStiffnesses, const TensorDesc* limits, const TensorDesc* restLengths, const TensorDesc* offsets, const TensorDesc* indexTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(stiffnesses);
    PASS_EMPTY_TENSOR(dampings);
    PASS_EMPTY_TENSOR(limitStiffnesses);
    PASS_EMPTY_TENSOR(limits);
    PASS_EMPTY_TENSOR(restLengths);
    PASS_EMPTY_TENSOR(offsets);

    if (!stiffnesses || !stiffnesses->data)
    {
        return false;
    }
    if (!dampings || !dampings->data)
    {
        return false;
    }
    if (!limitStiffnesses || !limitStiffnesses->data)
    {
        return false;
    }
    if (!limits || !limits->data)
    {
        return false;
    }
    if (!restLengths || !restLengths->data)
    {
        return false;
    }
    if (!offsets || !offsets->data)
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    if (!checkTensorDevice(*stiffnesses, -1, "tendon stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*stiffnesses, "tendon stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*stiffnesses, getCount() * mMaxFixedTendons, "tendon stiffness", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(stiffnesses->data) + idx * mMaxFixedTendons;
            for (PxU32 j = 0; j < mEntries[idx].numFixedTendons; j++)
            {
                mEntries[idx].fixedTendons[j]->setStiffness(src[j]);
            }
        }
    }

    if (!checkTensorDevice(*dampings, -1, "tendon damping", __FUNCTION__) ||
        !checkTensorFloat32(*dampings, "tendon damping", __FUNCTION__) ||
        !checkTensorSizeExact(*dampings, getCount() * mMaxFixedTendons, "tendon damping", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(dampings->data) + idx * mMaxFixedTendons;
            for (PxU32 j = 0; j < mEntries[idx].numFixedTendons; j++)
            {
                mEntries[idx].fixedTendons[j]->setDamping(src[j]);
            }
        }
    }

    if (!checkTensorDevice(*limitStiffnesses, -1, "tendon limit stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*limitStiffnesses, "tendon limit stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*limitStiffnesses, getCount() * mMaxFixedTendons, "tendon limit stiffness", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(limitStiffnesses->data) + idx * mMaxFixedTendons;
            for (PxU32 j = 0; j < mEntries[idx].numFixedTendons; j++)
            {
                mEntries[idx].fixedTendons[j]->setLimitStiffness(src[j]);
            }
        }
    }

    if (!checkTensorDevice(*limits, -1, "tendon limits", __FUNCTION__) ||
        !checkTensorFloat32(*limits, "tendon limits", __FUNCTION__) ||
        !checkTensorSizeExact(*limits, getCount() * mMaxFixedTendons * 2u, "tendon limits", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(limits->data) + idx * mMaxFixedTendons * 2;
            for (PxU32 j = 0; j < mEntries[idx].numFixedTendons; j++)
            {
                PxArticulationTendonLimit limit;
                limit.lowLimit = src[j*2];
                limit.highLimit = src[j*2+1];
                mEntries[idx].fixedTendons[j]->setLimitParameters(limit);
            }
        }
    }

    if (!checkTensorDevice(*restLengths, -1, "tendon rest length", __FUNCTION__) ||
        !checkTensorFloat32(*restLengths, "tendon rest length", __FUNCTION__) ||
        !checkTensorSizeExact(*restLengths, getCount() * mMaxFixedTendons, "tendon rest length", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(restLengths->data) + idx * mMaxFixedTendons;
            for (PxU32 j = 0; j < mEntries[idx].numFixedTendons; j++)
            {
                mEntries[idx].fixedTendons[j]->setRestLength(src[j]);
            }
        }
    }

    if (!checkTensorDevice(*offsets, -1, "tendon offset", __FUNCTION__) ||
        !checkTensorFloat32(*offsets, "tendon offset", __FUNCTION__) ||
        !checkTensorSizeExact(*offsets, getCount() * mMaxFixedTendons, "tendon offset", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(offsets->data) + idx * mMaxFixedTendons;
            for (PxU32 j = 0; j < mEntries[idx].numFixedTendons; j++)
            {
                mEntries[idx].fixedTendons[j]->setOffset(src[j]);
            }
        }
    }

    return true;
}

bool CpuArticulationView::setSpatialTendonProperties(const TensorDesc* stiffnesses, const TensorDesc* dampings, const TensorDesc* limitStiffnesses, const TensorDesc* offsets, const TensorDesc* indexTensor) const
{
    CHECK_VALID_DATA_SIM_RETURN(mCpuSimData, mSim, false);
    PASS_EMPTY_TENSOR(stiffnesses);
    PASS_EMPTY_TENSOR(dampings);
    PASS_EMPTY_TENSOR(limitStiffnesses);
    PASS_EMPTY_TENSOR(offsets);

    if (!stiffnesses || !stiffnesses->data)
    {
        return false;
    }
    if (!dampings || !dampings->data)
    {
        return false;
    }
    if (!limitStiffnesses || !limitStiffnesses->data)
    {
        return false;
    }
    if (!offsets || !offsets->data)
    {
        return false;
    }

    const PxU32* indices = nullptr;
    PxU32 numIndices = 0;
    if (indexTensor && indexTensor->data)
    {
        if (!checkTensorDevice(*indexTensor, -1, "index", __FUNCTION__) ||
            !checkTensorInt32(*indexTensor, "index", __FUNCTION__) ||
            !checkIndexTensorSize(*indexTensor, getCount(), __FUNCTION__))
        {
            return false;
        }
        indices = static_cast<const PxU32*>(indexTensor->data);
        numIndices = PxU32(getTensorTotalSize(*indexTensor));
    }
    else
    {
        indices = mAllIndices.data();
        numIndices = PxU32(mAllIndices.size());
    }

    if (!checkTensorDevice(*stiffnesses, -1, "tendon stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*stiffnesses, "tendon stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*stiffnesses, getCount() * mMaxSpatialTendons, "tendon stiffness", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(stiffnesses->data) + idx * mMaxSpatialTendons;
            for (PxU32 j = 0; j < mEntries[idx].numSpatialTendons; j++)
            {
                mEntries[idx].spatialTendons[j]->setStiffness(src[j]);
            }
        }
    }

    if (!checkTensorDevice(*dampings, -1, "tendon damping", __FUNCTION__) ||
        !checkTensorFloat32(*dampings, "tendon damping", __FUNCTION__) ||
        !checkTensorSizeExact(*dampings, getCount() * mMaxSpatialTendons, "tendon damping", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(dampings->data) + idx * mMaxSpatialTendons;
            for (PxU32 j = 0; j < mEntries[idx].numSpatialTendons; j++)
            {
                mEntries[idx].spatialTendons[j]->setDamping(src[j]);
            }
        }
    }

    if (!checkTensorDevice(*limitStiffnesses, -1, "tendon limit stiffness", __FUNCTION__) ||
        !checkTensorFloat32(*limitStiffnesses, "tendon limit stiffness", __FUNCTION__) ||
        !checkTensorSizeExact(*limitStiffnesses, getCount() * mMaxSpatialTendons, "tendon limit stiffness", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(limitStiffnesses->data) + idx * mMaxSpatialTendons;
            for (PxU32 j = 0; j < mEntries[idx].numSpatialTendons; j++)
            {
                mEntries[idx].spatialTendons[j]->setLimitStiffness(src[j]);
            }
        }
    }

    if (!checkTensorDevice(*offsets, -1, "tendon offset", __FUNCTION__) ||
        !checkTensorFloat32(*offsets, "tendon offset", __FUNCTION__) ||
        !checkTensorSizeExact(*offsets, getCount() * mMaxSpatialTendons, "tendon offset", __FUNCTION__))
    {
        return false;
    }

    for (PxU32 i = 0; i < numIndices; i++)
    {
        PxU32 idx = indices[i];
        if (idx < mEntries.size())
        {
            const float* src = static_cast<const float*>(offsets->data) + idx * mMaxSpatialTendons;
            for (PxU32 j = 0; j < mEntries[idx].numSpatialTendons; j++)
            {
                mEntries[idx].spatialTendons[j]->setOffset(src[j]);
            }
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// Mask support - uses shared resolveMaskToIndices() / makeIndexTensorDesc()
// from TensorUtils.h to avoid duplicating validation logic across view classes.
// ---------------------------------------------------------------------------

using omni::physics::tensors::MaskResult;
using omni::physics::tensors::resolveMaskToIndices;
using omni::physics::tensors::makeIndexTensorDesc;

// Macro to stamp out the ~24 simple two-arg masked setters: setFooMasked(src, mask) -> setFoo(src, &idx).
// Chosen over a template because target methods have mixed const/non-const qualifiers and live in both
// base and derived classes (would need CRTP or std::function); #undef'd immediately after use.
#define CPU_ARTI_MASKED_SETTER(MethodName, IsConst) \
bool CpuArticulationView::MethodName##Masked(const TensorDesc* src, const TensorDesc* maskTensor) IsConst \
{ \
    std::vector<uint32_t> indices; \
    auto result = resolveMaskToIndices(maskTensor, getCount(), -1, indices, __FUNCTION__); \
    if (result == MaskResult::Error) return false; \
    if (result == MaskResult::Empty) return true; \
    if (result == MaskResult::All)   return MethodName(src, nullptr); \
    TensorDesc idx = makeIndexTensorDesc(indices, -1); \
    return MethodName(src, &idx); \
}

// Non-const masked setters (20)
CPU_ARTI_MASKED_SETTER(setRootTransforms, )
CPU_ARTI_MASKED_SETTER(setRootVelocities, )
CPU_ARTI_MASKED_SETTER(setDofPositions, )
CPU_ARTI_MASKED_SETTER(setDofVelocities, )
CPU_ARTI_MASKED_SETTER(setDofActuationForces, )
CPU_ARTI_MASKED_SETTER(setDofPositionTargets, )
CPU_ARTI_MASKED_SETTER(setDofVelocityTargets, )
CPU_ARTI_MASKED_SETTER(setDofLimits, )
CPU_ARTI_MASKED_SETTER(setDofStiffnesses, )
CPU_ARTI_MASKED_SETTER(setDofDampings, )
CPU_ARTI_MASKED_SETTER(setDofMaxForces, )
CPU_ARTI_MASKED_SETTER(setDofDriveModelProperties, )
CPU_ARTI_MASKED_SETTER(setDofFrictionCoefficients, )
CPU_ARTI_MASKED_SETTER(setDofFrictionProperties, )
CPU_ARTI_MASKED_SETTER(setDofMaxVelocities, )
CPU_ARTI_MASKED_SETTER(setDofArmatures, )
CPU_ARTI_MASKED_SETTER(setMasses, )
CPU_ARTI_MASKED_SETTER(setCOMs, )
CPU_ARTI_MASKED_SETTER(setInertias, )
// setDisableGravities/material/rest/contact/compliant Masked: BaseArticulationView

#undef CPU_ARTI_MASKED_SETTER

// applyForcesAndTorquesAtPositionMasked – has force, torque, position, mask, isGlobal
bool CpuArticulationView::applyForcesAndTorquesAtPositionMasked(const TensorDesc* srcForceTensor,
                                                                  const TensorDesc* srcTorqueTensor,
                                                                  const TensorDesc* srcPositionTensor,
                                                                  const TensorDesc* maskTensor,
                                                                  const bool isGlobal)
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(maskTensor, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return applyForcesAndTorquesAtPosition(srcForceTensor, srcTorqueTensor, srcPositionTensor, nullptr, isGlobal);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return applyForcesAndTorquesAtPosition(srcForceTensor, srcTorqueTensor, srcPositionTensor, &idx, isGlobal);
}

// setFixedTendonPropertiesMasked – multi-param tendon setter
bool CpuArticulationView::setFixedTendonPropertiesMasked(const TensorDesc* stiffnesses,
                                                          const TensorDesc* dampings,
                                                          const TensorDesc* limitStiffnesses,
                                                          const TensorDesc* limits,
                                                          const TensorDesc* restLengths,
                                                          const TensorDesc* offsets,
                                                          const TensorDesc* maskTensor) const
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(maskTensor, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setFixedTendonProperties(stiffnesses, dampings, limitStiffnesses, limits, restLengths, offsets, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setFixedTendonProperties(stiffnesses, dampings, limitStiffnesses, limits, restLengths, offsets, &idx);
}

// setSpatialTendonPropertiesMasked – multi-param tendon setter
bool CpuArticulationView::setSpatialTendonPropertiesMasked(const TensorDesc* stiffnesses,
                                                            const TensorDesc* dampings,
                                                            const TensorDesc* limitStiffnesses,
                                                            const TensorDesc* offsets,
                                                            const TensorDesc* maskTensor) const
{
    std::vector<uint32_t> indices;
    auto result = resolveMaskToIndices(maskTensor, getCount(), -1, indices, __FUNCTION__);
    if (result == MaskResult::Error) return false;
    if (result == MaskResult::Empty) return true;
    if (result == MaskResult::All)   return setSpatialTendonProperties(stiffnesses, dampings, limitStiffnesses, offsets, nullptr);
    TensorDesc idx = makeIndexTensorDesc(indices, -1);
    return setSpatialTendonProperties(stiffnesses, dampings, limitStiffnesses, offsets, &idx);
}

}
}
}
