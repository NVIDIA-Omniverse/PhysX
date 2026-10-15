// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-TENSOR-CONTACT-003
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-40
 *
 * @implements REQ-READ-CONTACT-001
 * @covers AC-4 AC-7
 *
 * @implements REQ-READ-CONTACT-004
 * @covers AC-2
 *
 * @implements REQ-READ-CONTACT-005
 * @covers AC-2
 */

#pragma once

#include "tensors/CommonTypes.h"
#include "tensors/ForceComponent.h"
#include "tensors/base/BaseRigidContactView.h"
#include "tensors/gpu/GpuSimulationData.h"

#include <omni/physics/tensors/IRigidContactView.h>

#include <string>
#include <vector>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class GpuSimulationView;


class GpuRigidContactView : public BaseRigidContactView
{
public:
    GpuRigidContactView(GpuSimulationView* sim,
                        std::vector<RigidContactSensorEntry>&& entries,
                        uint32_t numFilters,
                        uint32_t maxContactDataCount,
                        int device);

    ~GpuRigidContactView() override;

    bool getNetNormalContactForces(const TensorDesc* dstTensor, float dt) const override;

    bool getNetFrictionContactForces(const TensorDesc* dstTensor, float dt) const override;

    // Non-blocking net-force reduction (either component) for the ovstage output-read hot path. Enqueues
    // the reduction on the null stream and returns WITHOUT the trailing cudaStreamSynchronize getNetForces
    // does, so the caller's own completion event covers it -- no per-step CPU-GPU stall. dt is guarded: a
    // non-positive dt yields a zero column (1/dt would be inf -> NaN) rather than dividing blindly.
    bool getNetContactForcesOvStage(const TensorDesc* dstTensor, float dt, ForceComponent component) const;

    // The force matrix [S, F, 3] (either component) on the same terms: no stream synchronization, and a
    // non-positive dt yields a zero matrix.
    bool getContactForceMatrixOvStage(const TensorDesc* dstTensor, float dt, ForceComponent component) const;

    // The filtered detail (getNormalContactData / getFrictionContactData) split at its count, so the caller
    // sizes the outputs exactly between the two passes: getFilteredContactCountsOvStage writes the [S, F]
    // element counts of one component into `countsDev`; the fill then writes each element at startsDev[slot] +
    // its rank in the slot, `cursorDev` (zeroed here) serving as the per-slot rank counter. Stream-ordered on
    // the null stream with no synchronization; a non-positive dt yields zero forces. Every output pointer is
    // required: the kernels write all of them. Sizing is the caller's, as only a null pointer is refused:
    // `countsDev`, `startsDev` and `cursorDev` hold [S, F] PxU32, and each output at least `capacity` elements
    // (1 float per force or separation, 3 per point, normal or friction force); an element at or past
    // `capacity` is dropped.
    bool getFilteredContactCountsOvStage(::physx::PxU32* countsDev, ForceComponent component) const;
    bool getFilteredContactDataOvStage(float* forceDev,
                                       float* pointDev,
                                       float* normalDev,
                                       float* separationDev,
                                       ::physx::PxU32* cursorDev,
                                       const ::physx::PxU32* startsDev,
                                       ::physx::PxU32 capacity,
                                       float dt) const;
    bool getFilteredFrictionDataOvStage(float* forceDev,
                                        float* pointDev,
                                        ::physx::PxU32* cursorDev,
                                        const ::physx::PxU32* startsDev,
                                        ::physx::PxU32 capacity,
                                        float dt) const;

    // The raw contacts (getRawContactData) split the same way, one slot per sensor: getRawContactCountsOvStage
    // writes each sensor's element count into `countsDev`; getRawContactsOvStage writes the elements and, per
    // element, the (sensor, other) body pair's identities into `bodyPairsDev` through the table
    // setActorHandles installed -- 0 for an actor it does not hold. Sized as above, one slot per sensor:
    // `countsDev`, `startsDev` and `cursorDev` hold [S] PxU32, and `bodyPairsDev` 2 uint64 per element.
    bool getRawContactCountsOvStage(::physx::PxU32* countsDev) const;
    bool getRawContactsOvStage(float* forceDev,
                               float* pointDev,
                               float* normalDev,
                               float* separationDev,
                               uint64_t* bodyPairsDev,
                               ::physx::PxU32* cursorDev,
                               const ::physx::PxU32* startsDev,
                               ::physx::PxU32 capacity,
                               float dt) const;

    // The raw identities are the caller's, not the legacy path ids: getLegacyActorIds reads the scene's actor
    // table (the legacy path id of each actor, in table order), and setActorHandles installs the caller's
    // identity for each, in the same order, as the table getRawContactsOvStage writes through. Blocking; meant
    // to run once per view.
    bool getLegacyActorIds(std::vector<uint64_t>& out) const;
    bool setActorHandles(const std::vector<uint64_t>& handles);
    bool hasActorHandles() const
    {
        return mActorHandlesSet;
    }

    bool getNormalContactForceMatrix(const TensorDesc* dstTensor, float dt) const override;

    bool getFrictionContactForceMatrix(const TensorDesc* dstTensor, float dt) const override;

    bool getNormalContactData(const TensorDesc* contactForceTensor,
                              const TensorDesc* contactPointTensor,
                              const TensorDesc* contactNormalTensor,
                              const TensorDesc* contactSeparationTensor,
                              const TensorDesc* contactCountTensor,
                              const TensorDesc* contactStartIndicesTensor,
                              uint32_t* outRequiredContactCount,
                              float dt) const override;

    bool getFrictionContactData(const TensorDesc* FrictionForceTensor,
                                const TensorDesc* contactPointTensor,
                                const TensorDesc* contactCountTensor,
                                const TensorDesc* contactStartIndicesTensor,
                                uint32_t* outRequiredFrictionCount,
                                float dt) const override;

    bool getRawContactData(const TensorDesc* contactForceTensor,
                           const TensorDesc* contactPointTensor,
                           const TensorDesc* contactNormalTensor,
                           const TensorDesc* contactSeparationTensor,
                           const TensorDesc* sensorLayoutTensor,
                           const TensorDesc* actorIdsTensor,
                           uint32_t* outRequiredContactCount,
                           float dt) const override;

private:
    bool getNetForces(const TensorDesc* dstTensor,
                      float dt,
                      ForceComponent component,
                      const char* description,
                      const char* functionName) const;

    bool getForceMatrix(const TensorDesc* dstTensor,
                       float dt,
                       ForceComponent component,
                       const char* description,
                       const char* functionName) const;

    int mDevice = -1;

    GpuSimulationDataPtr mGpuSimData;

    // O(1) map from physx GPU rigid dynamic index to referent index
    ::physx::PxU32* mRdContactIndicesDev = nullptr;

    // O(1) map from physx GPU articulation link index to referent index
    ::physx::PxU32* mLinkContactIndicesDev = nullptr;

    GpuRigidContactFilterIdPair* mFilterLookupDev = nullptr;

    // Scratch for getRawContactData's per-sensor bookkeeping: [0, numSensors) counts,
    // [numSensors, 2*numSensors) start indices. Kept contiguous and separate from the
    // caller's interleaved (numSensors, 2) tensor so exclusiveScan and the count/fill
    // kernels keep taking plain arrays; a final pack kernel interleaves the result.
    ::physx::PxU32* mRawLayoutScratchDev = nullptr;

    ::physx::PxU32* mContactCountMatrix = nullptr;

    // getRawContactsOvStage's identity table (setActorHandles): the legacy actor table's actors, sorted as it is,
    // with the caller's identities.
    GpuActorPathIdPair* mActorHandleLookupDev = nullptr;
    ::physx::PxU32 mNumActorHandlePairs = 0;
    bool mActorHandlesSet = false;
};

} // namespace tensors
} // namespace physx
} // namespace omni
