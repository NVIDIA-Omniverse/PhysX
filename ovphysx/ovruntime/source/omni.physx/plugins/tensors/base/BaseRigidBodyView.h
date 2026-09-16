// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "tensors/CommonTypes.h"
#include "tensors/base/BaseSimulationData.h"
#include "tensors/base/OvStageShapeProperty.h"

#include <omni/physics/tensors/IRigidBodyView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class BaseSimulationView;

class BaseRigidBodyView : public omni::physics::tensors::IRigidBodyView
{
public:
    BaseRigidBodyView(BaseSimulationView* sim, const std::vector<RigidBodyEntry>& entries);

    virtual ~BaseRigidBodyView() override;

    uint32_t getCount() const override;
    uint32_t getMaxShapes() const override;

    const char* getUsdPrimPath(uint32_t rbIdx) const override;

    bool check() const override;

    void release() override;

    // rigid body properties
    bool getMasses(const TensorDesc* dstTensor) const override;
    bool getInvMasses(const TensorDesc* dstTensor) const override;
    bool getCOMs(const TensorDesc* dstTensor) const override;
    bool getInertias(const TensorDesc* dstTensor) const override;
    bool getInvInertias(const TensorDesc* dstTensor) const override;
    bool getDisableGravities(const TensorDesc* dstTensor) const override;
    bool getDisableSimulations(const TensorDesc* dstTensor) const override;

    bool setMasses(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setCOMs(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setInertias(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDisableGravities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDisableSimulations(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    bool wakeUp(const TensorDesc* indexTensor) override;
    bool putToSleep(const TensorDesc* indexTensor) override;

    // materials/shapes
    bool getMaterialProperties(const TensorDesc* dstTensor) const override;
    bool getCompliantMaterialProperties(const TensorDesc* dstTensor, const TensorDesc* dstCombineModeTensor) const override;
    bool getRestOffsets(const TensorDesc* dstTensor) const override;
    bool getContactOffsets(const TensorDesc* dstTensor) const override;

    bool setMaterialProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;
    bool setCompliantMaterialProperties(const TensorDesc* srcTensor,
                                        const TensorDesc* srcCombineTensor,
                                        const TensorDesc* indexTensor) const override;
    bool setRestOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;
    bool setContactOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;

    // CPU-only property masked wrappers (host mask -> indexed Base/Gpu setters).
    // Device-mask paths for pose/vel/mass stay on Cpu/Gpu views.
    bool setDisableGravitiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setDisableSimulationsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setMaterialPropertiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) const override;
    bool setCompliantMaterialPropertiesMasked(const TensorDesc* srcTensor,
                                              const TensorDesc* srcCombineTensor,
                                              const TensorDesc* maskTensor) const override;
    bool setRestOffsetsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) const override;
    bool setContactOffsetsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) const override;

    void setComsCacheStateValid(bool flag)
    {
        validComsCache = flag;
    };
    bool getComsCacheStateValid() const
    {
        return validComsCache;
    };
    //
    // helpers
    //

    void _onParentRelease();

    // ------------------------------------------------------------------------------------------
    // ovstage column reads for the body properties (ADR-0008, REQ-READ-ATTRS-001).
    //
    // Host-only and shared by both backends: these are simulation *inputs* that PhysX never writes
    // back, so DirectGPU readback suppression cannot make them stale and the emitted column is
    // `kDLCPU` even on a DirectGPU scene. `rowsToken` is unused; it exists so the signature matches
    // the pose/velocity family the reader's attribute table dispatches through.

    // ovstage property WRITE (ADR-0012), public alongside the gathers they invert. Host-only: none
    // of these has a device destination, so one implementation serves both backends -- see
    // setBodyPropertyOvStage.
    bool setMassesOvStage(const TensorDesc* srcTensor, const ::physx::PxU32* outRecordIdx, ::physx::PxU32 numOutputs, uint64_t rowsToken);
    bool setInertiasOvStage(const TensorDesc* srcTensor, const ::physx::PxU32* outRecordIdx, ::physx::PxU32 numOutputs, uint64_t rowsToken);
    bool setComPositionsOvStage(const TensorDesc* srcTensor, const ::physx::PxU32* outRecordIdx, ::physx::PxU32 numOutputs, uint64_t rowsToken);
    bool setComOrientationsOvStage(const TensorDesc* srcTensor, const ::physx::PxU32* outRecordIdx, ::physx::PxU32 numOutputs, uint64_t rowsToken);
    bool setDisableGravitiesOvStage(const TensorDesc* srcTensor, const ::physx::PxU32* outRecordIdx, ::physx::PxU32 numOutputs, uint64_t rowsToken);
    bool setDisableSimulationsOvStage(const TensorDesc* srcTensor, const ::physx::PxU32* outRecordIdx, ::physx::PxU32 numOutputs, uint64_t rowsToken);
    bool setStaticFrictionsOvStage(const TensorDesc* srcTensor, const ::physx::PxU32* outRecordIdx, ::physx::PxU32 numOutputs, uint64_t rowsToken);
    bool setDynamicFrictionsOvStage(const TensorDesc* srcTensor, const ::physx::PxU32* outRecordIdx, ::physx::PxU32 numOutputs, uint64_t rowsToken);
    bool setRestitutionsOvStage(const TensorDesc* srcTensor, const ::physx::PxU32* outRecordIdx, ::physx::PxU32 numOutputs, uint64_t rowsToken);
    bool setContactOffsetsOvStage(const TensorDesc* srcTensor, const ::physx::PxU32* outRecordIdx, ::physx::PxU32 numOutputs, uint64_t rowsToken);
    bool setRestOffsetsOvStage(const TensorDesc* srcTensor, const ::physx::PxU32* outRecordIdx, ::physx::PxU32 numOutputs, uint64_t rowsToken);

    bool getMassesOvStage(const TensorDesc* dstTensor,
                          const ::physx::PxU32* outRecordIdx,
                          ::physx::PxU32 numOutputs,
                          uint64_t rowsToken) const;
    bool getInvMassesOvStage(const TensorDesc* dstTensor,
                             const ::physx::PxU32* outRecordIdx,
                             ::physx::PxU32 numOutputs,
                             uint64_t rowsToken) const;
    bool getInertiasOvStage(const TensorDesc* dstTensor,
                            const ::physx::PxU32* outRecordIdx,
                            ::physx::PxU32 numOutputs,
                            uint64_t rowsToken) const;
    bool getInvInertiasOvStage(const TensorDesc* dstTensor,
                               const ::physx::PxU32* outRecordIdx,
                               ::physx::PxU32 numOutputs,
                               uint64_t rowsToken) const;
    bool getComPositionsOvStage(const TensorDesc* dstTensor,
                                const ::physx::PxU32* outRecordIdx,
                                ::physx::PxU32 numOutputs,
                                uint64_t rowsToken) const;
    bool getComOrientationsOvStage(const TensorDesc* dstTensor,
                                   const ::physx::PxU32* outRecordIdx,
                                   ::physx::PxU32 numOutputs,
                                   uint64_t rowsToken) const;
    // uint8 columns, not float: the tensor API reports these as byte flags and the read emits them
    // the same way rather than widening them to 0.0/1.0.
    bool getDisableGravitiesOvStage(const TensorDesc* dstTensor,
                                    const ::physx::PxU32* outRecordIdx,
                                    ::physx::PxU32 numOutputs,
                                    uint64_t rowsToken) const;
    bool getDisableSimulationsOvStage(const TensorDesc* dstTensor,
                                      const ::physx::PxU32* outRecordIdx,
                                      ::physx::PxU32 numOutputs,
                                      uint64_t rowsToken) const;

    // ------------------------------------------------------------------------------------------
    // Per-SHAPE columns (REQ-READ-ATTRS-001 AC-10).
    //
    // A body has a variable number of shapes, so these columns are padded to the widest body in the
    // read and every row is `shapeWidth` wide. The width is a property of the read, not of the
    // attribute: it comes from `dstTensor->dims[1]`, which the emit sizes from maxShapesForRows()
    // over the same row list. Padding is zero-filled; the companion shapeCount column tells a
    // consumer where the real values stop.
    bool getStaticFrictionsOvStage(const TensorDesc* dstTensor,
                                   const ::physx::PxU32* outRecordIdx,
                                   ::physx::PxU32 numOutputs,
                                   uint64_t rowsToken) const;
    bool getDynamicFrictionsOvStage(const TensorDesc* dstTensor,
                                    const ::physx::PxU32* outRecordIdx,
                                    ::physx::PxU32 numOutputs,
                                    uint64_t rowsToken) const;
    bool getRestitutionsOvStage(const TensorDesc* dstTensor,
                                const ::physx::PxU32* outRecordIdx,
                                ::physx::PxU32 numOutputs,
                                uint64_t rowsToken) const;
    bool getContactOffsetsOvStage(const TensorDesc* dstTensor,
                                  const ::physx::PxU32* outRecordIdx,
                                  ::physx::PxU32 numOutputs,
                                  uint64_t rowsToken) const;
    bool getRestOffsetsOvStage(const TensorDesc* dstTensor,
                               const ::physx::PxU32* outRecordIdx,
                               ::physx::PxU32 numOutputs,
                               uint64_t rowsToken) const;
    // int32, one per body: how many of that row's `shapeWidth` entries are real.
    bool getShapeCountsOvStage(const TensorDesc* dstTensor,
                               const ::physx::PxU32* outRecordIdx,
                               ::physx::PxU32 numOutputs,
                               uint64_t rowsToken) const;

    // Widest shape count among the named rows. The QUERIED set, not the whole view: mMaxShapes is
    // computed over the superset (every body in the scene), so using it would let an unrelated
    // many-shaped body widen every read's per-shape column.
    uint32_t maxShapesForRows(const ::physx::PxU32* outRecordIdx, ::physx::PxU32 numOutputs) const;

    // One requested per-shape column: which property, and where its values go. Keyed by
    // OvStageShapeProperty rather than the protected BodyProperty so the reader can name it.
    struct ShapePropertyColumn
    {
        OvStageShapeProperty property;
        const TensorDesc* dst;
    };

    // Every requested per-shape column in one walk of each body's shapes, resolving a shape's
    // material once for the whole set rather than once per column (REQ-READ-ATTRS-001 AC-12).
    bool getShapePropertyColumnsOvStage(const ShapePropertyColumn* columns,
                                        ::physx::PxU32 numColumns,
                                        const ::physx::PxU32* outRecordIdx,
                                        ::physx::PxU32 numOutputs) const;

protected:
    // Which property a column carries, so one gather serves them all.
    enum class BodyProperty
    {
        eMass,
        eInvMass,
        eInertia,
        eInvInertia,
        eComPosition,
        eComOrientation,
        eDisableGravity,
        eDisableSimulation,
        // Per-shape, below. Their width comes from the destination tensor, not from layoutOf().
        eStaticFriction,
        eDynamicFriction,
        eRestitution,
        eContactOffset,
        eRestOffset,
        eShapeCount,
        eCount // layoutOf's table is indexed by this enum and static_asserts against it
    };
    // Column width and element type per property, so the gather and its tensor checks size a column
    // from one place.
    struct BodyPropertyLayout
    {
        // 0 for a per-shape column: its width is a property of the read, not of the attribute, and
        // comes from the destination tensor instead.
        uint32_t comp;
        bool isByte;   // uint8 column rather than float32
        bool perShape; // one value per shape of the body, padded to the read's widest
        const char* label;
    };
    static BodyPropertyLayout layoutOf(BodyProperty prop);

    // The per-shape row's OvStageShapeProperty, or false if the row is not per-shape. Kept beside
    // layoutOf so the one-column and set forms below cannot answer it differently.
    static bool ovStageShapePropertyOf(BodyProperty prop, OvStageShapeProperty& out);

    bool getBodyPropertyOvStage(BodyProperty prop,
                                const TensorDesc* dstTensor,
                                const ::physx::PxU32* outRecordIdx,
                                ::physx::PxU32 numOutputs) const;

    // ovstage property WRITE (ADR-0012): the inverse of the gather above, sharing its
    // layout table and its record indirection.
    //
    // Host-only, and that is the contract rather than a limitation: none of these has a device
    // source, because they are simulation INPUTS PhysX never writes back. So there is no DirectGPU
    // call, no packed pair and no scatter kernel here -- just the setters, which is why one
    // implementation on the base view serves both backends where pose and velocity needed two.
    //
    // Read-only properties (inverse mass, inverse inertia, shape count) have no setter and are
    // rejected by layout rather than silently ignored.
    bool setBodyPropertyOvStage(BodyProperty prop,
                                const TensorDesc* srcTensor,
                                const ::physx::PxU32* outRecordIdx,
                                ::physx::PxU32 numOutputs);


protected:
    BaseSimulationView* mSim = nullptr;

    BaseSimulationDataPtr mSimData;

    std::vector<RigidBodyEntry> mEntries;
    std::vector<uint32_t> mAllIndices;
    mutable bool validComsCache = true;
    uint32_t mMaxShapes = 0;
};

} // namespace tensors
} // namespace physx
} // namespace omni
