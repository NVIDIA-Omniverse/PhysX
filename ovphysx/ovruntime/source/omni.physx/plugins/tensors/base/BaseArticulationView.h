// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-INVDYN-001
 * @covers AC-1, AC-2, AC-6
 */

#pragma once

#include "tensors/ArticulationDofOvStageRecord.h"
#include "tensors/ArticulationMetatype.h"
#include "tensors/CommonTypes.h"
#include "tensors/base/BaseSimulationData.h"
#include "tensors/base/OvStageShapeProperty.h"

#include <omni/physics/tensors/IArticulationView.h>

namespace omni
{
namespace physx
{
namespace tensors
{
using omni::physics::tensors::TensorDesc;

class BaseSimulationView;

class BaseArticulationView : public omni::physics::tensors::IArticulationView
{
public:
    BaseArticulationView(BaseSimulationView* sim, const std::vector<ArticulationEntry>& entries);

    ~BaseArticulationView() override;

    //
    // public API
    //

    uint32_t getCount() const override;
    uint32_t getMaxLinks() const override;
    uint32_t getMaxDofs() const override;
    uint32_t getMaxShapes() const override;
    uint32_t getMaxFixedTendons() const override;
    uint32_t getMaxSpatialTendons() const override;

    const char* getUsdPrimPath(uint32_t artiIdx) const override;
    const char* getUsdDofPath(uint32_t artiIdx, uint32_t dofIdx) const override;
    const char* getUsdLinkPath(uint32_t artiIdx, uint32_t linkIdx) const override;

    // articulation type info
    bool isHomogeneous() const override;
    const ArticulationMetatype* getSharedMetatype() const override;
    const ArticulationMetatype* getMetatype(uint32_t artiIdx) const override;

    bool getJacobianShape(uint32_t* numRows, uint32_t* numCols) const override;
    bool getGeneralizedMassMatrixShape(uint32_t* numRows, uint32_t* numCols) const override;

    // Which inverse dynamics column a width is being asked for. Not a per-attribute enum: coriolis and
    // gravity are both eGeneralizedForce because their width rule is identical.
    enum class InverseDynamicsColumn
    {
        eNone, // not an inverse dynamics column; width comes from the attribute, not the topology
        eJacobian,
        eMassMatrix,
        eGeneralizedForce,
        eCentroidalMomentum,
        // Width is a constant 2, so it needs none of the topology arithmetic the others do. It is here
        // so that it is EMITTED per cohort like them, keeping it on the same partitioning as the
        // jacobian it describes, which a caller then matches by prim list.
        eJacobianShape
    };

    // The flattened lane count one row needs, or 0 if the row cannot serve the column at all --
    // which today means a fixed-base articulation asked for centroidal momentum.
    ::physx::PxU32 inverseDynamicsColumnWidthForRow(::physx::PxU32 row, InverseDynamicsColumn column) const;

    // One requested DOF property column: which quantity, and where its values go. Declared here rather
    // than beside DofProperty because it names a TensorDesc, and that record header is included by
    // CudaKernels.cu and must stay dependency-free.
    struct DofPropertyColumn
    {
        omni::physx::tensors::DofProperty prop;
        const TensorDesc* dst;
    };

    // Every requested property in ONE pass over the records, in the convention the USD author used (see
    // DofProperty and DofScalePolicy): seven of the columns read getDriveParams and three read
    // getFrictionParams, so one pass needs at most four PhysX fetches per DOF instead of one per column.
    // Host-resident on both backends -- a simulation input has no device copy to read.
    bool getDofPropertiesOvStage(const DofPropertyColumn* columns,
                                 ::physx::PxU32 numColumns,
                                 const ArticulationDofOvStageRecord* records,
                                 ::physx::PxU32 numOutputs) const;

    // Which DOF STATE quantity a column carries. In-class because the .cu-facing record header stays
    // dependency-free and nothing in a kernel needs this.
    enum class DofStateQuantity : uint32_t
    {
        ePosition = 0,
        eVelocity,
        eActuationForce,
        ePositionTarget,
        eVelocityTarget,
        // "Not a DOF state column", so the reader's attribute table can carry the quantity as a FIELD
        // rather than a second hand-written list of names. jointProjectedForce is the row that takes
        // it: derived from the link incoming joint forces, not read from a cache.
        eNone
    };

    // The per-quantity facts, kept beside each other so a new quantity cannot get one and miss the
    // others: which cache flag fills it, what to call it, and how its raw value folds. eNone answers
    // every one of them with "not a column" rather than a value that reads as real.
    //
    // Which articulation-cache flag a quantity needs filled before the gather, or an EMPTY set for one
    // that needs none -- the two drive targets are per-joint PhysX state with no cache behind them. The
    // set form ORs these together, so this is the only place deciding which quantities are cache-backed.
    static ::physx::PxArticulationCacheFlags dofStateCacheFlag(DofStateQuantity q)
    {
        switch (q)
        {
        case DofStateQuantity::ePosition:        return ::physx::PxArticulationCacheFlag::ePOSITION;
        case DofStateQuantity::eVelocity:        return ::physx::PxArticulationCacheFlag::eVELOCITY;
        case DofStateQuantity::eActuationForce:  return ::physx::PxArticulationCacheFlag::eFORCE;
        case DofStateQuantity::ePositionTarget:
        case DofStateQuantity::eVelocityTarget:
        case DofStateQuantity::eNone:            break;
        }
        return ::physx::PxArticulationCacheFlags(0);
    }

    // The attribute name for the check helpers' messages -- the columns share one gather, so without
    // this a wrong-size tensor on any of them logs the same line.
    static const char* dofStateLabel(DofStateQuantity q)
    {
        switch (q)
        {
        case DofStateQuantity::ePosition:        return "DOF position (ovstage)";
        case DofStateQuantity::eVelocity:        return "DOF velocity (ovstage)";
        case DofStateQuantity::eActuationForce:  return "DOF actuation force (ovstage)";
        case DofStateQuantity::ePositionTarget:  return "DOF position target (ovstage)";
        case DofStateQuantity::eVelocityTarget:  return "DOF velocity target (ovstage)";
        case DofStateQuantity::eNone:            return "DOF state (ovstage)";
        }
        return "DOF state (ovstage)";
    }

    // For a check that concerns the whole SET rather than one column, as on the root-state side.
    static const char* dofStateSetLabel()
    {
        return "DOF state (ovstage)";
    }

    // A generalized force on the axis is a newton-metre, not an angle: it takes the body-order sign and
    // NOT the degree fold. Every other state quantity is a generalized coordinate or its derivative and
    // takes both. eNone must answer eNone -- no fold, dofScaleFor returns 1.0f.
    static DofScalePolicy dofStateScalePolicy(DofStateQuantity q)
    {
        switch (q)
        {
        case DofStateQuantity::eActuationForce:  return DofScalePolicy::eSigned;
        case DofStateQuantity::ePosition:
        case DofStateQuantity::eVelocity:
        case DofStateQuantity::ePositionTarget:
        case DofStateQuantity::eVelocityTarget:  return DofScalePolicy::eAngularSigned;
        case DofStateQuantity::eNone:            return DofScalePolicy::eNone;
        }
        return DofScalePolicy::eNone;
    }

    // One requested DOF state column: which quantity, and where its values go.
    struct DofStateColumn
    {
        DofStateQuantity quantity;
        const TensorDesc* dst;
    };

    // Which whole-articulation ROOT-STATE quantity a column carries. The four columns are served by
    // fewer sources than there are columns -- both pose columns come out of one fetch, and on the host
    // both velocity columns do too -- so they are gathered as a SET, like the DOF properties above.
    // In-class rather than in a record header: no .cu consumes these and there is no per-row device
    // record for them to sit beside.
    enum class RootStateQuantity : uint32_t
    {
        ePosition = 0,
        eOrientation,
        eLinearVelocity,
        eAngularVelocity,
        // "Not a root-state column", so the reader's attribute table can carry the quantity as a FIELD
        // the way it already carries InverseDynamicsColumn::eNone. Never handed to a backend: the reader
        // filters on it first.
        eNone
    };

    // Which SOURCE a quantity's value comes from, as a bitmask so a set of columns can ask for the
    // union. Not a PhysX cache flag or read type: the backends partition these differently -- the host
    // fills both velocity components from one eROOT_VELOCITIES copy while DirectGPU has a read type for
    // each -- so the shared fact is the quantity GROUP, which each backend maps to its own fetch.
    enum RootStateSource : uint32_t
    {
        eRootSrcNone = 0u,
        eRootSrcPose = 1u << 0,        // position and orientation, always one fetch on both backends
        eRootSrcLinearVel = 1u << 1,   // one fetch with the angular on the host, its own on the device
        eRootSrcAngularVel = 1u << 2,
    };

    static uint32_t rootStateSource(RootStateQuantity q)
    {
        switch (q)
        {
        case RootStateQuantity::ePosition:
        case RootStateQuantity::eOrientation:      return eRootSrcPose;
        case RootStateQuantity::eLinearVelocity:   return eRootSrcLinearVel;
        case RootStateQuantity::eAngularVelocity:  return eRootSrcAngularVel;
        case RootStateQuantity::eNone:             return eRootSrcNone;
        }
        return eRootSrcNone;
    }

    // Lane count: the orientation is a quaternion, the other three are vectors. eNone answers 0, "not a
    // column", rather than a 3 that would read as a real lane count.
    static uint32_t rootStateComponents(RootStateQuantity q)
    {
        switch (q)
        {
        case RootStateQuantity::eOrientation:      return 4u;
        case RootStateQuantity::ePosition:
        case RootStateQuantity::eLinearVelocity:
        case RootStateQuantity::eAngularVelocity:  return 3u;
        case RootStateQuantity::eNone:             return 0u;
        }
        return 0u;
    }

    // The attribute name for the check helpers' messages -- four columns share one gather, so
    // without this a wrong-size tensor on any of them logs the same line.
    static const char* rootStateLabel(RootStateQuantity q)
    {
        switch (q)
        {
        case RootStateQuantity::ePosition:         return "articulation root position";
        case RootStateQuantity::eOrientation:      return "articulation root orientation";
        case RootStateQuantity::eLinearVelocity:   return "articulation root linear velocity";
        case RootStateQuantity::eAngularVelocity:  return "articulation root angular velocity";
        case RootStateQuantity::eNone:             return "articulation root state";
        }
        return "articulation root state";
    }

    // For a check that concerns the whole SET rather than one column; borrowing columns[0]'s label
    // would report a set-level failure as a single-column one.
    static const char* rootStateSetLabel()
    {
        return "articulation root state";
    }

    // One requested root-state column: which quantity, and where its values go.
    struct RootStateColumn
    {
        RootStateQuantity quantity;
        const TensorDesc* dst;
    };

    // DOF properties
    bool getDofTypes(const TensorDesc* dstTensor) const override;
    bool getDofMotions(const TensorDesc* dstTensor) const override;
    bool getDofLimits(const TensorDesc* dstTensor) const override;
    bool getDriveTypes(const TensorDesc* dstTensor) const override;
    bool getDofStiffnesses(const TensorDesc* dstTensor) const override;
    bool getDofDampings(const TensorDesc* dstTensor) const override;
    bool getDofMaxForces(const TensorDesc* dstTensor) const override;
    bool getDofDriveModelProperties(const TensorDesc* dstTensor) const override;
    bool getDofFrictionCoefficients(const TensorDesc* dstTensor) const override; // deprecated
    bool getDofFrictionProperties(const TensorDesc* dstTensor) const override;
    bool getDofMaxVelocities(const TensorDesc* dstTensor) const override;
    bool getDofArmatures(const TensorDesc* dstTensor) const override;

    // ovstage per-DOF PROPERTY write (ADR-0012): the return direction of
    // getDofPropertyOvStage, sharing its record list, its policies and its (joint, axis) resolution.
    //
    // Host-resident on both devices, like the getter and for the same reason: these are simulation
    // INPUTS PhysX never writes back, so there is no device copy to scatter into.
    //
    // READ-MODIFY-WRITE where a property shares a PhysX struct with its neighbours -- stiffness,
    // damping, maxForce, the drive envelope and driveType all live in one PxArticulationDrive, and
    // the three frictions in one PxArticulationJointFrictionParams. A session carries ONE attribute,
    // so each of those reads the struct back before overwriting its own field.
    bool setDofPropertyOvStage(DofProperty prop,
                               const ArticulationDofOvStageRecord* records,
                               ::physx::PxU32 numOutputs,
                               const TensorDesc* srcTensor);

    bool setDofLimits(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDofStiffnesses(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDofDampings(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDofMaxForces(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDofDriveModelProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDofFrictionCoefficients(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override; // deprecated
    bool setDofFrictionProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDofMaxVelocities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDofArmatures(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;

    // rigid body properties
    bool getMasses(const TensorDesc* dstTensor) const override;
    bool getInvMasses(const TensorDesc* dstTensor) const override;
    bool getCOMs(const TensorDesc* dstTensor) const override;
    bool getInertias(const TensorDesc* dstTensor) const override;
    bool getInvInertias(const TensorDesc* dstTensor) const override;
    bool getDisableGravities(const TensorDesc* srcTensor) const override;

    bool setCOMs(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setMasses(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setInertias(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;
    bool setDisableGravities(const TensorDesc* srcTensor, const TensorDesc* indexTensor) override;


    // materials/shapes
    bool getMaterialProperties(const TensorDesc* dstTensor) const override;
    bool getCompliantMaterialProperties(const TensorDesc* dstTensor, const TensorDesc* dstCombineModeTensor) const override;
    bool getRestOffsets(const TensorDesc* dstTensor) const override;
    bool getContactOffsets(const TensorDesc* dstTensor) const override;

    // ovstage whole-articulation shape columns. Rows select articulations from this view; per-shape
    // columns use dstTensor->dims[1] as the selected set's padded width, which OvxPhysicsRead has
    // already validated -- these repeat a defensive bounds check but trust that width.
    bool getStaticFrictionsOvStage(const TensorDesc* dstTensor, const ::physx::PxU32* rows, ::physx::PxU32 count) const;
    bool getDynamicFrictionsOvStage(const TensorDesc* dstTensor, const ::physx::PxU32* rows, ::physx::PxU32 count) const;
    bool getRestitutionsOvStage(const TensorDesc* dstTensor, const ::physx::PxU32* rows, ::physx::PxU32 count) const;
    bool getContactOffsetsOvStage(const TensorDesc* dstTensor, const ::physx::PxU32* rows, ::physx::PxU32 count) const;
    bool getRestOffsetsOvStage(const TensorDesc* dstTensor, const ::physx::PxU32* rows, ::physx::PxU32 count) const;
    bool getShapeCountsOvStage(const TensorDesc* dstTensor, const ::physx::PxU32* rows, ::physx::PxU32 count) const;

    // Host-resident like getShapeCountsOvStage above: derived from the metatype, not from simulation
    // state. It is the shape side channel for the flattened `jacobian` column, whose two dimensions both
    // vary with topology and so cannot be recovered from dtype.lanes (REQ-READ-INVDYN-001 AC-6).
    bool getJacobianShapesOvStage(const TensorDesc* dstTensor, const ::physx::PxU32* rows, ::physx::PxU32 count) const;

    uint32_t maxShapesForRows(const ::physx::PxU32* rows, ::physx::PxU32 count) const;

    bool setMaterialProperties(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;
    bool setCompliantMaterialProperties(const TensorDesc* srcTensor,
                                        const TensorDesc* srcCombineTensor,
                                        const TensorDesc* indexTensor) const override;
    bool setRestOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;
    bool setContactOffsets(const TensorDesc* srcTensor, const TensorDesc* indexTensor) const override;

    // CPU-only property masked wrappers (host mask). Device-mask paths stay on Cpu/Gpu.
    bool setDisableGravitiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) override;
    bool setMaterialPropertiesMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) const override;
    bool setCompliantMaterialPropertiesMasked(const TensorDesc* srcTensor,
                                              const TensorDesc* srcCombineTensor,
                                              const TensorDesc* maskTensor) const override;
    bool setRestOffsetsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) const override;
    bool setContactOffsetsMasked(const TensorDesc* srcTensor, const TensorDesc* maskTensor) const override;

    bool check() const override;

    void release() override;

    //
    // helpers
    //

    void _onParentRelease();


protected:
    // Per-ROW shape, for the inverse dynamics output read. getJacobianShape and getGeneralizedMassMatrixShape
    // describe the whole view and refuse a non-homogeneous one, since one row stride cannot describe
    // articulations of differing size; the read partitions rows into cohorts of one metatype and asks
    // per row, so these answer for any view (REQ-READ-INVDYN-001 AC-2). `M` is the generalized
    // coordinate count -- dofs plus six for a floating base -- the width of the force columns and the
    // side of the mass matrix.
    ::physx::PxU32 generalizedCoordinateCountForRow(::physx::PxU32 row) const;
    bool jacobianShapeForRow(::physx::PxU32 row, ::physx::PxU32& numRows, ::physx::PxU32& numCols) const;

    // Shared width check for both backends, so the CPU and GPU paths cannot drift on the rule. The
    // caller supplies the cohort's width in dstTensor.dims[1] because it depends on topology rather
    // than on the attribute; this confirms every selected row agrees with it. A mismatch means the
    // selection is not one cohort, and accepting it would stride into the next row.
    bool checkInverseDynamicsColumn(const TensorDesc& dstTensor,
                                    const ::physx::PxU32* rows,
                                    ::physx::PxU32 count,
                                    InverseDynamicsColumn column,
                                    ::physx::PxU32& widthOut,
                                    const char* label,
                                    const char* funcName) const;

    // Shared body of the five per-shape float columns. shapeCount is not one of them -- different
    // dtype, size rule and destination type -- so it stays in getShapeCountsOvStage.
    bool getShapePropertyOvStage(OvStageShapeProperty property,
                                 const TensorDesc* dstTensor,
                                 const ::physx::PxU32* rows,
                                 ::physx::PxU32 count) const;

    BaseSimulationView* mSim = nullptr;
    BaseSimulationDataPtr mSimData;

    std::vector<ArticulationEntry> mEntries;
    std::vector<uint32_t> mAllIndices;

    uint32_t mMaxLinks = 0;
    uint32_t mMaxDofs = 0;
    uint32_t mMaxShapes = 0;
    uint32_t mMaxFixedTendons = 0;
    uint32_t mMaxSpatialTendons = 0;

    // whether all articulations have the same metatype
    bool mIsHomogeneous = false;
    // weaker than mIsHomogeneous: what the base-type-dependent accessors need
    bool mIsBaseTypeUniform = true;

    bool requireUniformBaseType(const char* funcName) const;

    // The (joint, axis) pair an ovstage DOF record names, or null when the record does not resolve
    // to a live DOF of this view. Shared by the property gather here and the state gathers on the
    // CPU view, so an out-of-range record means the same thing everywhere.
    const DofImpl* resolveDofImpl(const ArticulationDofOvStageRecord& record) const;

    bool validComsCache = true;

    void setComsCacheStateValid(bool flag)
    {
        validComsCache = flag;
    };
    bool getComsCacheStateValid()
    {
        return validComsCache;
    };
};

} // namespace tensors
} // namespace physx
} // namespace omni
