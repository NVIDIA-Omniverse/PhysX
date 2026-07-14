// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "../CommonTypes.h"
#include "../base/BaseRigidBodyView.h"
#include "GpuSimulationData.h"
#include "PxDirectGPUAPI.h"

#include <omni/physics/tensors/IRigidBodyView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class GpuSimulationView;

class GpuRigidBodyView : public BaseRigidBodyView
{
public:
    GpuRigidBodyView(GpuSimulationView* sim, const std::vector<RigidBodyEntry>& entries, int device);

    ~GpuRigidBodyView() override;

    bool getTransforms(const TensorDesc* dstTensor) const override;
    bool getVelocities(const TensorDesc* dstTensor) const override;
    bool getAccelerations(const TensorDesc* dstTensor) const override;

    bool setKinematicTargets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setTransforms(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    bool applyForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool applyForcesAndTorquesAtPosition(const TensorDesc* srcForceTensor,
                                         const TensorDesc* srcTorqueTensor,
                                         const TensorDesc* srcPositionTensor,
                                         const TensorDesc* indexTensor,
                                         const bool isGlobal) override;

    // Masked overrides
    bool setKinematicTargetsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setTransformsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setVelocitiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool applyForcesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool applyForcesAndTorquesAtPositionMasked(const TensorDesc* srcForceTensor,
                                               const TensorDesc* srcTorqueTensor,
                                               const TensorDesc* srcPositionTensor,
                                               const TensorDesc* maskTensor,
                                               const bool isGlobal) override;
    bool setMassesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setCOMsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setInertiasMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setDisableGravitiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setDisableSimulationsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setMaterialPropertiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) const override;
    bool setCompliantMaterialPropertiesMasked(const TensorDesc* srcTensor,
                                              const TensorDesc* srcCombineTensor,
                                              const TensorDesc* maskTensor) const override;
    bool setRestOffsetsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) const override;
    bool setContactOffsetsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) const override;

private:
    void copyActorAndLinksTransorms();
    bool updateCMassData();
    bool clearDataFlagsAndIndices();
    bool resolveMask(const TensorDesc* maskTensor, ::physx::PxU32& outK) const;

    int mDevice = -1;

    GpuSimulationDataPtr mGpuSimData;

    uint32_t mNumRds = 0;
    uint32_t mNumArtis = 0;
    uint32_t mNumArtiRoots = 0;

    // will keep this until direct GPU API for articulation link mass properties is available
    // made class memebrs to avoid frequent memory allocation
    // TODO: clean up these variables
    std::vector<GpuRigidBodyRecord> rbRecords;
    std::vector<::physx::PxVec3> cMassLocalPosePos;
    ::physx::PxVec3* cMassLocalPosePosDev = nullptr; // coms for links + rd

    // indexing data for all rigid bodies
    GpuRigidBodyRecord* mRbRecordsDev = nullptr;

    // all body indices in this view
    ::physx::PxU32* mRbIndicesDev = nullptr;
    ::physx::PxRigidDynamicGPUIndex* mRdGpuIndicesDev = nullptr;
    ::physx::PxRigidDynamicGPUIndex* mDirtyRdGpuIndices = nullptr;
    // articulation indices for fetching all link data and applying forces
    ::physx::PxArticulationGPUIndex* mArtiIndicesDev = nullptr;
    ::physx::PxArticulationGPUIndex* mDirtyArtiGpuIndices = nullptr;
    // dirty flag buffers with the size of articulation link and rigid link in the view
    ArticulationGpuFlags* mArtiDirtyFlagsDev = nullptr;
    ArticulationGpuFlags* mArtiLinksDirtyFlagsDev = nullptr;
    ActorGpuFlags* mRdDirtyFlagsDev = nullptr;

    uint32_t mMaxLinks = 0;

    SingleAllocPolicy mArtiIndexSingleAllocPolicy;
    SingleAllocPolicy mRdIndexSingleAllocPolicy;

    // Mask -> indices scratch (cached). Declared mutable because resolveMask()
    // is const - some interface setters (e.g. material/shape properties) are
    // const by pre-existing TensorAPI convention ("const" = view object unchanged,
    // simulation state may be mutated via pointer indirection). These buffers are
    // internal caching state, not logical view state.
    mutable ::physx::PxU32* mMaskIndicesDev = nullptr;
    mutable ::physx::PxU32 mMaskIndicesCapacity = 0;
    mutable SingleAllocPolicy mMaskAllocPolicy;
};
} // namespace tensors
} // namespace physx
} // namespace omni
