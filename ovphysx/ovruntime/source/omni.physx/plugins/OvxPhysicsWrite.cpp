// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-INPUT-CORE-001
 * @covers AC-6 AC-10
 *
 * @implements REQ-INPUT-DEVICE-001
 * @covers AC-1 AC-1a AC-1b AC-3
 *
 * @implements REQ-INPUT-COVERAGE-001
 * @covers AC-7 AC-8 AC-9 AC-12 AC-13 AC-14 AC-15 AC-16
 *
 * @implements REQ-SIM-OVSTAGE-WRITEAPPLY-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9 AC-10 AC-12 AC-13
 *
 * ADR-0012: the app -> physics write, the return direction of OvxPhysicsRead.cpp.
 *
 * WHAT IS HERE: the session -- handle table, group registry, the commit/discard lifecycle, the
 * address-stability rule that makes commit-by-pointer decidable, and rigid-body group planning.
 *
 * SCOPE: four object types, each with its own attribute table, because each reaches DIFFERENT state
 * through a different view -- not because the vocabulary differs.
 *
 *   kOvxRigidBody          per-body state and properties, through a rigid view.
 *   kOvxArticulationLink   the same per-body set MINUS what PhysX cannot write on a link. A
 *                          PxArticulationLink IS a PxRigidBody, so this shares kRigidWriteAttributes
 *                          and refuses per ROW (`linkWritable`), not per type.
 *   kOvxArticulationJoint  per-DOF state, indexed through per-DOF records rather than body rows.
 *   kOvxArticulation       the articulation's ROOT state, per articulation, through an articulation
 *                          view. Overlaps kOvxArticulationLink on the root prim, deliberately:
 *                          these are queries, not a partition.
 *
 * A name only has to be unique WITHIN a type, which is why `position` appears in two tables meaning
 * two different things and each table has to enforce its own list.
 *
 * Groups are planned to mirror the read exactly: the same enumeration (OvxPhysicsShared.h), the same
 * per-scene split, the same prim list. That symmetry is the point -- a caller reads a column, edits
 * it, and writes it back, so a write that enumerated differently would hand back a prim list that
 * does not line up with the rows it scatters into.
 *
 * Locked, matching OvxPhysicsRead.cpp, which guards its own tables with a file-local mutex in every
 * entry point. ovphysx.h already requires callers to serialize across instances and within one
 * (REQ-CAPI-WRITE-001 AC-10), so this does not promise concurrency the API does not offer -- it
 * keeps the table itself from being corrupted by a caller who ignores that, which is the same
 * bargain the read makes.
 */

#include <omni/physx/IOvxPhysicsWrite.h>
#include <omni/physx/IOvxPhysicsRead.h> // OvxAttr names, OvxObjectType, kOvx* scopes
#include <omni/physics/parse/IChangeFeed.h> // ChangeBatch/ColumnView -- the ovstage drain's apply input
#include <cstring> // std::memcpy (drain column gather/stage)

#include "OvxPhysicsShared.h" // enumeration + query resolution shared with the read

#include <OvstageOutput.h> // buildPathList; private to omni.physics.ovstage

#include "OmniPhysX.h"
#include "usdLoad/AttachedStage.h"
#include <common/foundation/MatrixTools.h> // affineInverse -- a prim transform may carry scale

#include "internal/Internal.h" // recordLifetimeEpoch
#include "internal/InternalActor.h" // InternalActorFlag::eLOCALSPACE_VELOCITIES -- the local-frame velocity guard
#include "internal/InternalParticle.h" // InternalParticleSet -- the particle write's destination
#include "internal/InternalDeformable.h" // InternalDeformableBody -- the sim-mesh reframe
#include "tensors/PointSetReframe.h" // PointSetTransform
#include "tensors/gpu/CudaKernels.h" // submitPointSetColumnOvStage

#include "tensors/SimulationBackend.h"
#include "tensors/GlobalsAreBad.h"
#include "tensors/gpu/GpuSimulationView.h"
#include "tensors/gpu/GpuRigidBodyView.h"
#include "tensors/cpu/CpuSimulationView.h"
#include "tensors/cpu/CpuRigidBodyView.h"
#include "tensors/base/BasePointInstancerView.h" // instancer instance scatter (ADR-0012)
#include "tensors/gpu/GpuArticulationView.h"
#include "tensors/cpu/CpuArticulationView.h"
#include "tensors/base/BaseRigidBodyView.h"
#include <common/utilities/CudaShimWrappers.h> // getCudaShim (consumer-stream handoff)
#include <omni/physics/tensors/TensorDesc.h>

#include <cudamanager/PxCudaContextManager.h>
#include <cudamanager/PxCudaContext.h>

#include <PxPhysicsAPI.h>

#include <carb/logging/Log.h>

#include <algorithm> // std::find
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace ::physx;
using omni::physx::cudaShimWrappers::getCudaShim;
using omni::physics::parse::IPhysicsSource;
using omni::physics::parse::ObjectKey;

namespace omni::physx
{
namespace
{

using omni::physx::ovx::ObjectKey;

// The rigid-body attributes a session can write, one row each: the column width, and whether the
// engine can set it on its own. Mirrors kRigidAttributes in OvxPhysicsRead.cpp -- adding an
// attribute is a row here plus a thin method on each view, with no dispatch to extend.
//
// `wholeTransform` marks the two that are slices of one PxTransform. PxRigidDynamicGPUAPIWriteType
// has a single eGLOBAL_POSE, so writing position alone must first read the current pose to keep the
// orientation -- a read-modify-write the velocity attributes do not need, because eLINEAR_VELOCITY
// and eANGULAR_VELOCITY are separate write types. Recorded per row rather than inferred from the
// name, so the cost is visible at the table rather than buried in the scatter.
struct WriteAttributeRow
{
    const char* token;
    // 0 for a per-shape column: the real width is the widest body in this session, resolved at plan
    // time from the view, exactly as the read resolves it at emit.
    int components;
    bool wholeTransform;
    // Element type of the column. float32 for everything with a device path; uint8 for the two actor
    // flags, which the tensor API reports as bytes.
    omni::physics::tensors::TensorDataType dtype;
    // True when the attribute has NO device destination, so its column is host-resident even on a
    // DirectGPU scene. These are simulation INPUTS PhysX never writes back, so there is nothing to
    // scatter through DirectGPU and one host setter serves both backends.
    bool hostOnly;
    bool perShape;
    // Writable on an ARTICULATION LINK, not just a standalone rigid dynamic.
    //
    // Stated per row rather than inferred from `hostOnly`, even though the two agree for every row
    // but one today. They agree by coincidence -- host-only means "no device source", link-writable
    // means "PhysX accepts this setter on a link" -- and `disableSimulation` is exactly where the
    // coincidence breaks: it is host-only but PxActorFlag documents it as "only supported by
    // PxRigidStatic and PxRigidDynamic actors". Inferring one from the other would have shipped that
    // as a silent wrong write.
    bool linkWritable;
    // Device-path attributes name a method per backend; host-only ones name a single
    // BaseRigidBodyView method in `baseWrite` instead, since there is no device variant to differ.
    // Exactly one of the two is set, which is what `hostOnly` selects between.
    bool (omni::physx::tensors::GpuRigidBodyView::*gpuWrite)(const omni::physics::tensors::TensorDesc*,
                                                             const PxU32*,
                                                             PxU32,
                                                             uint64_t);
    bool (omni::physx::tensors::CpuRigidBodyView::*cpuWrite)(const omni::physics::tensors::TensorDesc*,
                                                             const PxU32*,
                                                             PxU32,
                                                             uint64_t);
    bool (omni::physx::tensors::BaseRigidBodyView::*baseWrite)(const omni::physics::tensors::TensorDesc*,
                                                              const PxU32*,
                                                              PxU32,
                                                              uint64_t);
};

constexpr omni::physics::tensors::TensorDataType kF32 = omni::physics::tensors::TensorDataType::eFloat32;
constexpr omni::physics::tensors::TensorDataType kU8 = omni::physics::tensors::TensorDataType::eUint8;

constexpr WriteAttributeRow kRigidWriteAttributes[] = {
    // WRITE-ONLY (ADR-0012). linkWritable because addForce is a PxRigidBody method and PhysX has an
    // eLINK_FORCE write type, unlike the velocity rows directly below, which a link cannot take.
    //
    // No read oracle exists for this attribute in either direction: reading a force back would report
    // what the solver did with it, not what was written. Its tests assert an observable CONSEQUENCE.
    // WRITE-ONLY. 9 wide and deliberately NOT split into force / torque / point, unlike pose: the
    // split rule exists so each half can be read back and preserved, and a control input cleared
    // every step has no stored value to preserve. `point` is also not a load -- it is WHERE the load
    // applies -- so a session carrying it alone would mean nothing.
    { OvxAttr::kWrench, 9, false, kF32, false, false, true,
      &omni::physx::tensors::GpuRigidBodyView::setWrenchesOvStage,
      &omni::physx::tensors::CpuRigidBodyView::setWrenchesOvStage, nullptr },
    { OvxAttr::kForce, 3, false, kF32, false, false, true,
      &omni::physx::tensors::GpuRigidBodyView::setForcesOvStage,
      &omni::physx::tensors::CpuRigidBodyView::setForcesOvStage, nullptr },
    // Device path: pose and velocity, the two with a DirectGPU write behind them.
    { OvxAttr::kPosition, 3, true, kF32, false, false, false,
      &omni::physx::tensors::GpuRigidBodyView::setPositionsOvStage,
      &omni::physx::tensors::CpuRigidBodyView::setPositionsOvStage, nullptr },
    { OvxAttr::kOrientation, 4, true, kF32, false, false, false,
      &omni::physx::tensors::GpuRigidBodyView::setOrientationsOvStage,
      &omni::physx::tensors::CpuRigidBodyView::setOrientationsOvStage, nullptr },
    { OvxAttr::kLinearVelocity, 3, false, kF32, false, false, false,
      &omni::physx::tensors::GpuRigidBodyView::setLinearVelocitiesOvStage,
      &omni::physx::tensors::CpuRigidBodyView::setLinearVelocitiesOvStage, nullptr },
    { OvxAttr::kAngularVelocity, 3, false, kF32, false, false, false,
      &omni::physx::tensors::GpuRigidBodyView::setAngularVelocitiesOvStage,
      &omni::physx::tensors::CpuRigidBodyView::setAngularVelocitiesOvStage, nullptr },

    // Host-only: simulation inputs PhysX never writes back, so one base method serves both backends
    // (ADR-0012). Names are the read's -- it already serves every one of these.
    { OvxAttr::kMass, 1, false, kF32, true, false, true, nullptr, nullptr,
      &omni::physx::tensors::BaseRigidBodyView::setMassesOvStage },
    { OvxAttr::kInertia, 9, false, kF32, true, false, true, nullptr, nullptr,
      &omni::physx::tensors::BaseRigidBodyView::setInertiasOvStage },
    { OvxAttr::kCenterOfMassPosition, 3, false, kF32, true, false, true, nullptr, nullptr,
      &omni::physx::tensors::BaseRigidBodyView::setComPositionsOvStage },
    { OvxAttr::kCenterOfMassOrientation, 4, false, kF32, true, false, true, nullptr, nullptr,
      &omni::physx::tensors::BaseRigidBodyView::setComOrientationsOvStage },
    { OvxAttr::kDisableGravity, 1, false, kU8, true, false, true, nullptr, nullptr,
      &omni::physx::tensors::BaseRigidBodyView::setDisableGravitiesOvStage },
    { OvxAttr::kDisableSimulation, 1, false, kU8, true, false, false, nullptr, nullptr,
      &omni::physx::tensors::BaseRigidBodyView::setDisableSimulationsOvStage },

    // Per-shape: `components` is 0 because the width is the widest body in this session.
    { OvxAttr::kStaticFriction, 0, false, kF32, true, true, true, nullptr, nullptr,
      &omni::physx::tensors::BaseRigidBodyView::setStaticFrictionsOvStage },
    { OvxAttr::kDynamicFriction, 0, false, kF32, true, true, true, nullptr, nullptr,
      &omni::physx::tensors::BaseRigidBodyView::setDynamicFrictionsOvStage },
    { OvxAttr::kRestitution, 0, false, kF32, true, true, true, nullptr, nullptr,
      &omni::physx::tensors::BaseRigidBodyView::setRestitutionsOvStage },
    { OvxAttr::kContactOffset, 0, false, kF32, true, true, true, nullptr, nullptr,
      &omni::physx::tensors::BaseRigidBodyView::setContactOffsetsOvStage },
    { OvxAttr::kRestOffset, 0, false, kF32, true, true, true, nullptr, nullptr,
      &omni::physx::tensors::BaseRigidBodyView::setRestOffsetsOvStage },
};

// The joint-DOF attributes a session can write. Mirrors kJointAttributes in OvxPhysicsRead.cpp: a
// row names the view method that publishes it, so adding one is a row here plus a thin method on
// each articulation view. Both backends take the same arguments, so one row serves both.
struct JointWriteAttributeRow
{
    const char* token;
    bool (omni::physx::tensors::GpuArticulationView::*gpuWrite)(
        const omni::physics::tensors::TensorDesc*, const omni::physx::tensors::ArticulationDofOvStageRecord*, PxU32);
    bool (omni::physx::tensors::CpuArticulationView::*cpuWrite)(
        const omni::physics::tensors::TensorDesc*, const omni::physx::tensors::ArticulationDofOvStageRecord*, PxU32);
};

constexpr JointWriteAttributeRow kJointWriteAttributes[] = {
    { OvxAttr::kJointPosition, &omni::physx::tensors::GpuArticulationView::setDofPositionsOvStage,
      &omni::physx::tensors::CpuArticulationView::setDofPositionsOvStage },
    { OvxAttr::kJointVelocity, &omni::physx::tensors::GpuArticulationView::setDofVelocitiesOvStage,
      &omni::physx::tensors::CpuArticulationView::setDofVelocitiesOvStage },
    // The drive INPUTS (ADR-0012): what the caller ASKS the drive for, where the two above are what
    // the joint currently IS. The read emits all five, so each round-trips -- but only these three
    // survive a step unchanged, since state is whatever the solver produced.
    { OvxAttr::kJointPositionTarget, &omni::physx::tensors::GpuArticulationView::setDofPositionTargetsOvStage,
      &omni::physx::tensors::CpuArticulationView::setDofPositionTargetsOvStage },
    { OvxAttr::kJointVelocityTarget, &omni::physx::tensors::GpuArticulationView::setDofVelocityTargetsOvStage,
      &omni::physx::tensors::CpuArticulationView::setDofVelocityTargetsOvStage },
    { OvxAttr::kJointActuationForce, &omni::physx::tensors::GpuArticulationView::setDofActuationForcesOvStage,
      &omni::physx::tensors::CpuArticulationView::setDofActuationForcesOvStage },
};

// The joint-DOF PROPERTY table (ADR-0012): authored drive, limit and friction values PhysX never
// writes back. Names are the READ's -- every one carries the `joint` prefix it chose.
//
// A property enum rather than the method-pointer pair kJointWriteAttributes uses, mirroring the
// read's own split and for the same reason: there is no device destination to differ over, so one
// setDofPropertyOvStage on BaseArticulationView serves both backends. These columns are therefore
// HOST-resident even on a DirectGPU scene, exactly as the rigid-body properties are.
struct JointPropertyWriteRow
{
    const char* token;
    omni::physx::tensors::DofProperty prop;
};

constexpr JointPropertyWriteRow kJointPropertyWriteAttributes[] = {
    { OvxAttr::kJointStiffness, omni::physx::tensors::DofProperty::eStiffness },
    { OvxAttr::kJointDamping, omni::physx::tensors::DofProperty::eDamping },
    // (lower, upper) as ONE interval and one column of 2 lanes -- NOT split, like tendonLimit. Its
    // halves are not independently meaningful, so a session that carried only one of them would be
    // writing half an interval.
    { OvxAttr::kJointLimit, omni::physx::tensors::DofProperty::eLimit },
    { OvxAttr::kJointMaxVelocity, omni::physx::tensors::DofProperty::eMaxVelocity },
    { OvxAttr::kJointMaxForce, omni::physx::tensors::DofProperty::eMaxForce },
    { OvxAttr::kJointArmature, omni::physx::tensors::DofProperty::eArmature },
    { OvxAttr::kJointStaticFriction, omni::physx::tensors::DofProperty::eStaticFriction },
    { OvxAttr::kJointDynamicFriction, omni::physx::tensors::DofProperty::eDynamicFriction },
    { OvxAttr::kJointViscousFriction, omni::physx::tensors::DofProperty::eViscousFriction },
    { OvxAttr::kJointSpeedEffortGradient, omni::physx::tensors::DofProperty::eSpeedEffortGradient },
    { OvxAttr::kJointMaxActuatorVelocity, omni::physx::tensors::DofProperty::eMaxActuatorVelocity },
    { OvxAttr::kJointVelocityDependentResistance,
      omni::physx::tensors::DofProperty::eVelocityDependentResistance },
    { OvxAttr::kJointDriveType, omni::physx::tensors::DofProperty::eDriveType },
};

const JointPropertyWriteRow* findJointPropertyWriteAttribute(std::string_view token)
{
    for (const JointPropertyWriteRow& row : kJointPropertyWriteAttributes)
        if (token == row.token)
            return &row;
    return nullptr;
}

const JointWriteAttributeRow* findJointWriteAttribute(std::string_view token)
{
    for (const JointWriteAttributeRow& row : kJointWriteAttributes)
        if (token == row.token)
            return &row;
    return nullptr;
}

// The kOvxArticulation table (ADR-0012). The names are the READ's -- rootPosition, rootOrientation,
// rootLinearVelocity, rootAngularVelocity -- not the rigid ones reused.
//
// An earlier version here reused `position` on the reasoning that a name only has to be unique
// within a type. The read had not yet shipped this selector; when it did, it chose the root* prefix,
// and following it is not optional. Two spellings for one quantity across the two directions of the
// same API is the failure the naming doc exists to prevent, and this branch already made exactly
// that mistake once with the tendon names.
//
// Separate from kRigidWriteAttributes rather than a `linkWritable`-style flag on it, because the
// SOURCE differs and not just the permission: root state is per articulation and reached through the
// articulation view, where every rigid row is per body and reached through a rigid one.
struct ArtiWriteAttributeRow
{
    const char* token;
    int components;
    bool (omni::physx::tensors::GpuArticulationView::*gpuWrite)(
        const omni::physics::tensors::TensorDesc*, const PxU32*, PxU32, uint64_t);
    bool (omni::physx::tensors::CpuArticulationView::*cpuWrite)(
        const omni::physics::tensors::TensorDesc*, const PxU32*, PxU32, uint64_t);
};

constexpr ArtiWriteAttributeRow kArtiWriteAttributes[] = {
    { OvxAttr::kRootPosition, 3, &omni::physx::tensors::GpuArticulationView::setRootPositionsOvStage,
      &omni::physx::tensors::CpuArticulationView::setRootPositionsOvStage },
    { OvxAttr::kRootOrientation, 4, &omni::physx::tensors::GpuArticulationView::setRootOrientationsOvStage,
      &omni::physx::tensors::CpuArticulationView::setRootOrientationsOvStage },
    { OvxAttr::kRootLinearVelocity, 3, &omni::physx::tensors::GpuArticulationView::setRootLinearVelocitiesOvStage,
      &omni::physx::tensors::CpuArticulationView::setRootLinearVelocitiesOvStage },
    { OvxAttr::kRootAngularVelocity, 3, &omni::physx::tensors::GpuArticulationView::setRootAngularVelocitiesOvStage,
      &omni::physx::tensors::CpuArticulationView::setRootAngularVelocitiesOvStage },
};

// The tendon tables (ADR-0012). Names are the READ's -- adopted, not proposed: the tendon read
// shipped first and chose the `tendon` prefix so a tendon's stiffness stays distinguishable from a
// joint drive's in a flat vocabulary.
//
// ONE row per property rather than per (kind, property): the two kinds differ only in which PhysX
// struct the property sits in, and the view methods already take that as an argument. What DOES
// differ is coverage -- a spatial tendon has no limit and no rest length, because the schema puts
// both on its leaf attachment -- and that is the `fixedOnly` flag rather than a second table.
struct TendonWriteAttributeRow
{
    const char* token;
    omni::physx::tensors::TendonProperty prop;
    bool fixedOnly;
};

constexpr TendonWriteAttributeRow kTendonWriteAttributes[] = {
    { OvxAttr::kTendonStiffness, omni::physx::tensors::TendonProperty::eStiffness, false },
    { OvxAttr::kTendonDamping, omni::physx::tensors::TendonProperty::eDamping, false },
    { OvxAttr::kTendonLimitStiffness, omni::physx::tensors::TendonProperty::eLimitStiffness, false },
    { OvxAttr::kTendonOffset, omni::physx::tensors::TendonProperty::eOffset, false },
    { OvxAttr::kTendonLimit, omni::physx::tensors::TendonProperty::eLimit, true },
    { OvxAttr::kTendonRestLength, omni::physx::tensors::TendonProperty::eRestLength, true },
};

// The kOvxDeformableMaterial table (ADR-0012): the exact inverse of the read's kMaterialAttributes,
// row for row, using the read's own names.
//
// `surfaceOnly` carries the same rule the read applies, and for the same reason rather than for
// symmetry: PhysX puts thickness and the two bending terms on PxDeformableSurfaceMaterial only. The
// read omits those rows on a volume material instead of reporting 0.0, so the write must refuse to
// take them there instead of writing somewhere -- the setter would be a downcast to a type the
// object is not.
struct DeformableMaterialWriteRow
{
    const char* token;
    bool surfaceOnly;
    void (*write)(::physx::PxDeformableMaterial&, float);
};

constexpr DeformableMaterialWriteRow kDeformableMaterialWriteAttributes[] = {
    { OvxAttr::kDeformableDynamicFriction, false,
      [](::physx::PxDeformableMaterial& m, float v) { m.setDynamicFriction(v); } },
    { OvxAttr::kDeformableYoungsModulus, false,
      [](::physx::PxDeformableMaterial& m, float v) { m.setYoungsModulus(v); } },
    // PhysX spells this setPoissons(); the attribute keeps the schema's name, as the read's does.
    { OvxAttr::kDeformablePoissonsRatio, false,
      [](::physx::PxDeformableMaterial& m, float v) { m.setPoissons(v); } },
    { OvxAttr::kDeformableElasticityDamping, false,
      [](::physx::PxDeformableMaterial& m, float v) { m.setElasticityDamping(v); } },
    // The three surface-only rows downcast. Safe only because the row list is built on
    // PxBase::is<>() -- PhysX's own concrete type -- and never on the ovruntime record kind that
    // names it, which is the same guard the read states: a mislabelled record must cost a dropped
    // row, not undefined behaviour in these three lines.
    { OvxAttr::kDeformableBendingStiffness, true,
      [](::physx::PxDeformableMaterial& m, float v)
      { static_cast<::physx::PxDeformableSurfaceMaterial&>(m).setBendingStiffness(v); } },
    { OvxAttr::kDeformableThickness, true,
      [](::physx::PxDeformableMaterial& m, float v)
      { static_cast<::physx::PxDeformableSurfaceMaterial&>(m).setThickness(v); } },
    { OvxAttr::kDeformableBendingDamping, true,
      [](::physx::PxDeformableMaterial& m, float v)
      { static_cast<::physx::PxDeformableSurfaceMaterial&>(m).setBendingDamping(v); } },
};

const DeformableMaterialWriteRow* findDeformableMaterialWriteAttribute(std::string_view token)
{
    for (const DeformableMaterialWriteRow& row : kDeformableMaterialWriteAttributes)
        if (token == row.token)
            return &row;
    return nullptr;
}

// The kOvxVehicleWheel control table (ADR-0012).
//
// These serve vehicles WITHOUT a drive, whose control surface is per wheel. A vehicle WITH a drive is
// accelerated and steered as a whole, and its commands live on the VEHICLE prim -- which this
// selector does not address, and which needs a per-vehicle type the read has not defined.
struct VehicleWriteAttributeRow
{
    const char* token;
    omni::physx::tensors::BaseVehicleView::WheelControl control;
};

constexpr VehicleWriteAttributeRow kVehicleWriteAttributes[] = {
    { OvxAttr::kDriveTorque, omni::physx::tensors::BaseVehicleView::WheelControl::eDriveTorque },
    { OvxAttr::kBrakeTorque, omni::physx::tensors::BaseVehicleView::WheelControl::eBrakeTorque },
    { OvxAttr::kSteerAngle, omni::physx::tensors::BaseVehicleView::WheelControl::eSteerAngle },
};

const VehicleWriteAttributeRow* findVehicleWriteAttribute(std::string_view token)
{
    for (const VehicleWriteAttributeRow& row : kVehicleWriteAttributes)
        if (token == row.token)
            return &row;
    return nullptr;
}

const TendonWriteAttributeRow* findTendonWriteAttribute(std::string_view token)
{
    for (const TendonWriteAttributeRow& row : kTendonWriteAttributes)
        if (token == row.token)
            return &row;
    return nullptr;
}

const ArtiWriteAttributeRow* findArtiWriteAttribute(std::string_view token)
{
    for (const ArtiWriteAttributeRow& row : kArtiWriteAttributes)
        if (token == row.token)
            return &row;
    return nullptr;
}

// Null for a name rigid bodies do not accept -- a normal answer, not a failure.
const WriteAttributeRow* findRigidWriteAttribute(std::string_view token)
{
    for (const WriteAttributeRow& row : kRigidWriteAttributes)
        if (token == row.token)
            return &row;
    return nullptr;
}

// One writable group handed to the caller. The ADDRESS of `group` is the commit identity
// (ovstage_map_group_t has no write_group_id), so slots are heap-allocated individually and never
// freed before the session ends: a vector of values would move them on growth, and a freed slot
// could see its address handed out again for a later group, making a committed group
// indistinguishable from a live one.
struct GroupSlot
{
    ovstage_map_group_t group{};
    bool committed = false;

    // Storage the CALLER fills, aliased by group.data.tensors[0]. Host or device by the scene's
    // pipeline, matching what the read emits for the same scene, so a caller can hand a column
    // straight back without moving it across the bus.
    std::vector<float> floats; // host path
    CUdeviceptr deviceData = 0; // device path; freed with the session under ctxMgr's context
    int deviceOrdinal = -1;
    PxCudaContextManager* ctxMgr = nullptr;

    // The ovstage drain reuse: when the incoming column already lives where the scene's scatter reads
    // (device column on a DirectGPU scene, host column on a CPU scene), the planner allocates nothing
    // and `deviceData`/`floats` are pointed straight at that column -- a borrow, not owned storage, so
    // release must not free it. A residency MISMATCH (today's host-only ovstage feeding a GPU scene)
    // is not a borrow: the planner allocates and the drain stages the column across the bus into it.
    bool borrowed = false;

    // Stable backing for the DLTensors and the prim group; both hand out pointers that must outlive
    // the fetch that published them.
    //
    // Vectors rather than singletons because a group is one of two shapes. A rigid group is FIXED:
    // one tensor stacking every prim along its leading dimension. A joint group is an ARRAY: one
    // tensor PER PRIM, all pointing into one buffer, which is ovstage's per-prim shape and what the
    // joint read emits. Emitting a joint group per prim instead would multiply everything a group
    // costs -- a prim list, a fetch, a commit -- by the joint count.
    std::vector<int64_t> shapes;
    std::vector<DLTensor> tensors;
    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;

    // The scatter destination. Bodies are NOT dereferenced before commit -- they name what to
    // re-resolve, and commit re-resolves rows against a generation taken at that moment, because a
    // row resolved at plan time addresses the superset view as it was then (checklist section 5).
    PxScene* scene = nullptr;
    std::vector<PxRigidBody*> bodies;
    std::vector<ObjectKey> keys;
    const WriteAttributeRow* row = nullptr;
    // The resolved column width. Equals row->components except for a per-shape attribute, where it
    // is the widest body in this session and therefore a property of the session, not of the row.
    uint32_t comp = 0;

    // Joint groups only. `numOut` is the flat DOF count the column carries (the sum of every slice),
    // and `recsDev` is the uploaded per-output record list the scatter indexes through -- device
    // memory owned by this slot and freed with the session, like the column itself.
    const JointWriteAttributeRow* jointRow = nullptr;
    CUdeviceptr recsDev = 0;
    uint32_t numOut = 0;

    // Articulation-root groups only. `artis` names what to re-resolve at commit, exactly as `bodies`
    // does for a rigid group and for the same reason: a superset row resolved at plan time addresses
    // the view as it was then. `rowsVersion` is minted with the resolved list and identifies its
    // CONTENTS to the view's device row cache.
    const ArtiWriteAttributeRow* artiRow = nullptr;
    std::vector<PxArticulationReducedCoordinate*> artis;
    uint64_t rowsVersion = 0;

    // Tendon groups only. `tendonFixed` picks the kind, which decides both the PhysX struct and
    // whether limit / rest length are addressable at all.
    const TendonWriteAttributeRow* tendonRow = nullptr;
    bool tendonFixed = false;

    // Vehicle wheel groups only.
    const VehicleWriteAttributeRow* vehicleRow = nullptr;

    // Joint DOF PROPERTY groups only (ADR-0012). Host-resident by construction.
    const JointPropertyWriteRow* jointPropRow = nullptr;

    // Point-instancer groups only. The view is owned by the read's host cache and outlives the
    // session; `instancerIdx` names which of its instancers this group publishes, and `offsets` is
    // the single-entry offset list the view's scatter takes.
    bool isInstancer = false;
    // The BASE type: the scatter is virtual, so this path does not branch on which device's view it
    // was handed.
    omni::physx::tensors::BasePointInstancerView* instancerView = nullptr;
    omni::physx::tensors::BasePointInstancerView::InstancerColumn instancerColumn{};
    uint32_t instancerIdx = 0;

    // Deformable sim-mesh groups only (ADR-0012). One group per BODY, device-resident: the
    // destination is PhysX's own buffer, so this is a device scatter plus markDirty.
    bool isDeformable = false;
    bool deformableVolume = false;   // which concrete type `deformableBody` is
    bool deformableVelocity = false; // false = points
    // Parallel, one entry per body in this group. ONE group covers every body of the kind, with one
    // tensor each sharing a single allocation -- the shape the read emits, so a column can be read
    // and handed straight back. A group per body cost 128 device allocations, fills and frees where
    // this costs one, and could not take the read's own output at all.
    std::vector<::physx::PxDeformableBody*> deformableBodies;
    std::vector<omni::physx::internal::InternalDeformableBody*> deformableInternals;
    std::vector<uint32_t> deformableCounts;      // vertices per body
    std::vector<size_t> deformableFloatOffsets;  // where each body's slice starts in the column

    // Deformable-material groups only (ADR-0012). Host-resident and scene-less: a material belongs
    // to no scene, so this slot's `scene` stays null.
    const DeformableMaterialWriteRow* deformableMatRow = nullptr;
    std::vector<::physx::PxDeformableMaterial*> materials; // one per row, parallel to `keys`

    // Particle groups only (ADR-0012). HOST-resident by construction, unlike the read's particle
    // columns, which are device-resident on a GPU scene. The asymmetry is the destination's: the
    // read gathers from PhysX's device buffers, while the write's target is the set's PINNED HOST
    // staging pair, which PhysX uploads at the next step.
    // Parallel, one entry per SET. One group carries every set, as the deformable and read groups
    // do -- the interchange property is the point: the read hands back one group of N tensors.
    std::vector<omni::physx::internal::InternalParticleSet*> particleSets;
    std::vector<uint32_t> particleCounts;
    std::vector<size_t> particleFloatOffsets;
    bool isParticle = false;
    bool particleVelocity = false; // false = points

    // The shared cache entry this group was planned from, when the scope allows one. Commit resolves
    // rows through it: valid under an unchanged generation, re-resolved and re-stamped otherwise.
    // Held as a KEY rather than a pointer -- the map can rehash between plan and commit, and a
    // pointer into it would dangle.
    bool cacheable = false;
    ovx::RigidReadCacheKey cacheKey{};
};

struct WriteSession
{
    OvxOutputQueryHandle query = 0;
    std::string attribute; // one per session; the group type carries no attribute field
    std::vector<std::unique_ptr<GroupSlot>> groups;
    size_t nextFetch = 0;
    ovx_path_dictionary_t* dict = nullptr;
};

std::mutex g_mutex; // guards g_writes and g_nextWriteHandle, as OvxPhysicsRead.cpp does
std::unordered_map<OvxWriteHandle, WriteSession> g_writes;
OvxWriteHandle g_nextWriteHandle = 1; // 0 is the invalid sentinel

WriteSession* findSession(OvxWriteHandle h)
{
    const std::unordered_map<OvxWriteHandle, WriteSession>::iterator it = g_writes.find(h);
    return it == g_writes.end() ? nullptr : &it->second;
}

// A group pointer is LIVE for this session if the session issued it and it has not been committed.
// Anything else -- foreign pointer, already committed, or from another session -- is not live, and
// commit must refuse it rather than report success.
GroupSlot* findLiveGroup(WriteSession& s, const ovstage_map_group_t* g)
{
    if (!g)
        return nullptr;
    for (const std::unique_ptr<GroupSlot>& slot : s.groups)
    {
        if (&slot->group == g)
            return slot->committed ? nullptr : slot.get();
    }
    return nullptr;
}

// Build one scene's group: allocate the column the caller fills, name the prims it covers, and stamp
// the DLTensor. Residence follows the scene's pipeline, exactly as the read's does.
bool planRigidGroup(WriteSession& s,
                    IPhysicsSource& source,
                    PxScene* scene,
                    const std::vector<PxRigidBody*>& bodies,
                    const std::vector<ObjectKey>& keys,
                    const WriteAttributeRow& row,
                    uint32_t scope,
                    bool allocate = true)
{
    if (bodies.size() != keys.size())
    {
        CARB_LOG_ERROR("ovxWriteAttribute: rigid-body and key lists disagree for '%s' (%zu bodies, %zu keys); "
                       "nothing was planned.",
                       s.attribute.c_str(), bodies.size(), keys.size());
        return false;
    }

    // A DirectGPU scene has no state row for disabled rigid dynamics. Do not offer those rows to the
    // caller for ordinary attributes. disableSimulation is the deliberate exception: its host flag
    // is the route that can re-enable a body and restore its GPU row.
    //
    // The scene's disable hint is the third term for the reason the read gates on it too: detection
    // costs an actor-flag read per matched body, and on a scene with nothing disabled the filter can
    // only conclude that it has nothing to drop. Conservative, so only a whole-scene "none disabled"
    // skips the scan.
    omni::physx::tensors::SimulationBackend* disableHintBackend = omni::physx::tensors::GetSimulationBackend();
    const bool filterDisabled = scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API) &&
                                std::string_view(row.token) != OvxAttr::kDisableSimulation &&
                                (!disableHintBackend || disableHintBackend->sceneMayHaveDisabledRigidDynamics(scene));
    const ovx::EnabledRigidBodies enabled(bodies, keys, filterDisabled);
    const bool anyDisabled = enabled.anyDisabled();
    const std::vector<PxRigidBody*>& outputBodies = enabled.bodies();
    const std::vector<ObjectKey>& outputKeys = enabled.keys();
    const size_t n = outputBodies.size();
    if (n == 0)
        return true; // no group: none matched, or every match is disabled (REQ-INPUT-CORE-001 AC-10)

    std::unique_ptr<GroupSlot> slot(new GroupSlot());
    slot->scene = scene;
    slot->bodies = outputBodies;
    slot->keys = outputKeys;
    slot->row = &row;

    // Column width: fixed from the row, or -- for a per-shape attribute -- the widest body in this
    // session, so every row is the same length and a narrower body leaves a padded tail. The read
    // settled that shape (a padded fixed group beats one group per prim); this mirrors it.
    uint32_t comp = static_cast<uint32_t>(row.components);
    if (row.perShape)
    {
        for (PxRigidBody* b : outputBodies)
        {
            const PxRigidActor* actor = b;
            comp = std::max<uint32_t>(comp, actor ? actor->getNbShapes() : 0u);
        }
        if (comp == 0)
            return true; // no shapes anywhere in this scene -- nothing to write, not a failure
    }
    slot->comp = comp;

    const size_t elemSize = (row.dtype == omni::physics::tensors::TensorDataType::eUint8) ? 1u : 4u;
    const size_t bytes = n * size_t(comp) * elemSize;

    // Host-only attributes get a HOST column even on a DirectGPU scene: there is no device
    // destination to scatter into, so a device column would only force the caller to stage across
    // the bus for nothing. Residency follows the attribute, not the scene -- which is what the read
    // does for the same set.
    const size_t floats = bytes / 4 + (bytes % 4 ? 1 : 0); // host storage is float-granular
    // allocate == false is the ovstage drain's zero-copy borrow: the caller owns the column and points
    // deviceData/floats straight at it after planning, so the session allocates no storage here.
    if (allocate && !row.hostOnly && scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API))
    {
        // The scene's own manager, not the process default: under multi-GPU the backend view runs in
        // this scene's context, so the column must be allocated there to be addressable by the
        // kernels that will read it at commit.
        PxCudaContextManager* ctxMgr = scene->getCudaContextManager();
        PxCudaContext* cu = ctxMgr ? ctxMgr->getCudaContext() : nullptr;
        if (!cu)
        {
            CARB_LOG_ERROR("ovxWriteAttribute: DirectGPU scene has no CUDA context; no group emitted.");
            return false;
        }
        PxScopedCudaLock _lock(*ctxMgr);
        cu->memAlloc(&slot->deviceData, bytes);
        if (!slot->deviceData)
        {
            CARB_LOG_ERROR("ovxWriteAttribute: column allocation failed (%zu floats).", floats);
            return false;
        }
        slot->ctxMgr = ctxMgr;
        slot->deviceOrdinal = ctxMgr->getDevice();
    }
    else if (allocate)
    {
        slot->floats.assign(floats, 0.0f);
    }

    // Canonicalising a key costs a lock plus a cache lookup, and building the list walks every prim,
    // so the handles are reused when the cache already holds them for this key set under the current
    // generation -- the same trade the read makes. `generation` is what says the dictionary they were
    // interned into is still the one in use; the key comparison is what says they describe these
    // prims. Neither alone is enough.
    const ovx::RigidReadCacheKey cacheKey{
        scene, static_cast<int>(omni::physx::tensors::RigidBodyType::eRigidDynamic), scope
    };
    // Keep the cache's structural census intact while membership is temporarily filtered.
    const bool cacheable = (scope == kOvxAll && !anyDisabled);
    ovx_path_dictionary_t* dict = nullptr;
    bool built = false;
    if (cacheable)
    {
        omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
        uint64_t generation = 0;
        if (backend && backend->acquireSceneView(scene, &generation))
        {
            ovx::RigidReadCacheEntry& ce = ovx::g_rigidReadCache[cacheKey];
            if (ce.generation != generation || ce.keys != outputKeys || ce.handles.size() != outputKeys.size())
            {
                ce.generation = generation;
                ce.dbEpoch = omni::physx::internal::recordLifetimeEpoch();
                ce.keys = outputKeys;
                ce.bodies = outputBodies;
                ce.handles.clear();
                omni::physics::ovstage::canonicalisePathHandles(
                    source, outputKeys.data(), outputKeys.size(), ce.handles,
                                                               nullptr);
                ce.rows.clear();       // rows belong to this generation and are resolved at commit
                ce.rowsVersion = 0;
            }
            if (ce.handles.size() == outputKeys.size())
            {
                slot->list = omni::physics::ovstage::buildPathListFromHandles(source, ce.handles.data(),
                                                                             ce.handles.size(), &dict);
                built = true;
            }
        }
    }
    if (!built)
        slot->list =
            omni::physics::ovstage::buildPathList(source, outputKeys.data(), outputKeys.size(), &dict);
    slot->cacheable = cacheable;
    slot->cacheKey = cacheKey;
    if (slot->list == OVX_INVALID_PRIMPATH_LIST || !dict)
    {
        CARB_LOG_ERROR("ovxWriteAttribute: failed to build ovx_primpath_list_t for '%s' (%zu prims) -- "
                       "the active backend is not ovstage (this write is ovstage-only).",
                       s.attribute.c_str(), outputKeys.size());
        return false;
    }
    s.dict = dict;

    slot->shapes.assign(1, static_cast<int64_t>(n));
    slot->tensors.assign(1, DLTensor{});
    DLTensor& t = slot->tensors[0];
    t.data = slot->deviceData ? reinterpret_cast<void*>(slot->deviceData) : static_cast<void*>(slot->floats.data());
    t.device = slot->deviceOrdinal >= 0 ? DLDevice{ kDLCUDA, slot->deviceOrdinal } : DLDevice{ kDLCPU, 0 };
    t.ndim = 1;
    t.dtype = (row.dtype == omni::physics::tensors::TensorDataType::eUint8) ?
                  DLDataType{ kDLUInt, 8, static_cast<uint16_t>(comp) } :
                  DLDataType{ kDLFloat, 32, static_cast<uint16_t>(comp) };
    t.shape = slot->shapes.data();
    t.strides = nullptr; // contiguous row-major SoA, as the read emits
    t.byte_offset = 0;

    ovstage_map_group_t& g = slot->group;
    g.prims.list = slot->list;
    g.prims.offset = 0;
    g.prims.count = static_cast<uint32_t>(n);
    g.prims.index_map = nullptr;
    g.data.tensors = slot->tensors.data();
    g.data.tensor_count = static_cast<uint32_t>(slot->tensors.size());
    g.data.count = 0;
    g.data.index_map = nullptr;

    s.groups.push_back(std::move(slot));
    return true;
}

// One scene's joint group: an ARRAY group carrying one tensor per joint, all pointing into a single
// flat DOF column the caller fills. Mirrors the joint read's emit exactly, because a caller reading a
// joint column and writing it straight back must see the same shape in both directions.
//
// The records and slices come from the shared cache (ensureJointRecords), so this derives nothing the
// read does not already derive.
bool planJointGroup(WriteSession& s,
                    IPhysicsSource& source,
                    PxScene* scene,
                    const JointWriteAttributeRow& row)
{
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
        return false;

    const bool gpu = scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API);

    // The write RUNS the structural walk rather than requiring a read to have run first. It is the
    // read's walk, shared -- so the two directions still enumerate one joint set -- but a write API
    // has no business demanding the caller issue a read to make it work. Idempotent and epoch-gated,
    // so this costs an epoch comparison once the cache is warm.
    if (!ovx::refreshArticulationCache(ovx::allPhysicsScenes()))
    {
        CARB_LOG_ERROR("ovxWriteAttribute: the articulation structural walk failed; nothing was written.");
        return false;
    }
    ovx::ArticulationReadCacheEntry* jcPtr = ovx::articulationCacheEntry(scene);
    if (!jcPtr || jcPtr->joints.empty())
        return true; // this scene owns no joints -- not a failure
    ovx::ArticulationReadCacheEntry& jc = *jcPtr;
    const bool enumCached = true; // just refreshed above, by definition

    uint64_t generation = 0;
    const std::vector<omni::physx::tensors::ArticulationEntry>* entriesPtr = nullptr;
    const std::unordered_map<const PxArticulationReducedCoordinate*, PxU32>* artiRowMap = nullptr;
    if (gpu)
    {
        omni::physx::tensors::GpuSimulationView* sv = backend->acquireSceneView(scene, &generation);
        if (!sv || !sv->supersetArticulationView(&artiRowMap, &entriesPtr))
            return false;
    }
    else
    {
        omni::physx::tensors::CpuSimulationView* sv = backend->acquireCpuSceneView(scene, &generation);
        if (!sv || !sv->supersetArticulationView(&artiRowMap, &entriesPtr))
            return false;
    }
    if (!artiRowMap || !entriesPtr)
        return false;

    std::vector<uint32_t> localToRow(jc.artis.size(), 0xffffffffu);
    for (size_t i = 0; i < jc.artis.size(); ++i)
    {
        const std::unordered_map<const PxArticulationReducedCoordinate*, PxU32>::const_iterator it =
            artiRowMap->find(jc.artis[i]);
        if (it == artiRowMap->end())
            return false; // the cached view does not describe this articulation set
        localToRow[i] = it->second;
    }
    if (!ovx::ensureJointRecords(jc, source, jc.joints, *entriesPtr, localToRow, generation, enumCached))
        return false;
    if (jc.slices.empty())
        return true;

    const uint32_t numOut = static_cast<uint32_t>(jc.recs.size());
    std::unique_ptr<GroupSlot> slot(new GroupSlot());
    slot->scene = scene;
    slot->jointRow = &row;
    slot->numOut = numOut;

    if (gpu)
    {
        PxCudaContextManager* ctxMgr = scene->getCudaContextManager();
        PxCudaContext* cu = ctxMgr ? ctxMgr->getCudaContext() : nullptr;
        if (!cu)
            return false;
        PxScopedCudaLock _lock(*ctxMgr);
        cu->memAlloc(&slot->deviceData, size_t(numOut) * sizeof(float));
        cu->memAlloc(&slot->recsDev,
                     size_t(numOut) * sizeof(omni::physx::tensors::ArticulationDofOvStageRecord));
        if (!slot->deviceData || !slot->recsDev)
            return false;
        cu->memcpyHtoD(slot->recsDev, jc.recs.data(),
                       size_t(numOut) * sizeof(omni::physx::tensors::ArticulationDofOvStageRecord));
        slot->ctxMgr = ctxMgr;
        slot->deviceOrdinal = ctxMgr->getDevice();
    }
    else
    {
        slot->floats.assign(numOut, 0.0f);
    }

    ovx_path_dictionary_t* dict = nullptr;
    const bool haveHandles = jc.sliceHandles.size() == jc.slices.size();
    slot->list = haveHandles ? omni::physics::ovstage::buildPathListFromHandles(source, jc.sliceHandles.data(),
                                                                               jc.sliceHandles.size(), &dict) :
                               omni::physics::ovstage::buildPathList(source, jc.sliceKeys.data(),
                                                                     jc.sliceKeys.size(), &dict);
    if (slot->list == OVX_INVALID_PRIMPATH_LIST || !dict)
        return false;
    s.dict = dict;

    // One tensor per joint, each a window onto its own slice of the flat column.
    slot->shapes.resize(jc.slices.size());
    slot->tensors.assign(jc.slices.size(), DLTensor{});
    for (size_t si = 0; si < jc.slices.size(); ++si)
    {
        const ovx::JointSlice& sl = jc.slices[si];
        slot->shapes[si] = static_cast<int64_t>(sl.count);
        DLTensor& t = slot->tensors[si];
        t.data = gpu ? reinterpret_cast<void*>(slot->deviceData + size_t(sl.offset) * sizeof(float)) :
                       static_cast<void*>(slot->floats.data() + sl.offset);
        t.device = gpu ? DLDevice{ kDLCUDA, slot->deviceOrdinal } : DLDevice{ kDLCPU, 0 };
        t.ndim = 1;
        t.dtype = DLDataType{ kDLFloat, 32, 1 };
        t.shape = &slot->shapes[si];
        t.strides = nullptr;
        t.byte_offset = 0;
    }

    ovstage_map_group_t& g = slot->group;
    g.prims.list = slot->list;
    g.prims.offset = 0;
    g.prims.count = static_cast<uint32_t>(jc.slices.size());
    g.prims.index_map = nullptr;
    g.data.tensors = slot->tensors.data();
    g.data.tensor_count = static_cast<uint32_t>(slot->tensors.size());
    g.data.count = 0;
    g.data.index_map = nullptr;

    s.groups.push_back(std::move(slot));
    return true;
}

// One scene's articulation-root group: a FIXED group, one tensor stacking every matched articulation
// along its leading dimension, exactly the shape a rigid group has. Root state is per articulation,
// so there is nothing per-prim to spread across an array group.
//
// Rows are NOT resolved here. They are resolved at commit against the generation reported then,
// because a superset row resolved at plan time addresses the view as it was at plan time (checklist
// section 5) -- the same rule the rigid path follows.
bool planArticulationGroup(WriteSession& s,
                           IPhysicsSource& source,
                           PxScene* scene,
                           const std::vector<PxArticulationReducedCoordinate*>& artis,
                           const std::vector<ObjectKey>& keys,
                           const ArtiWriteAttributeRow& row)
{
    const size_t n = artis.size();
    if (n == 0)
        return true; // this scene owns no matching articulations -- not a failure

    std::unique_ptr<GroupSlot> slot(new GroupSlot());
    slot->scene = scene;
    slot->artis = artis;
    slot->keys = keys;
    slot->artiRow = &row;
    slot->comp = static_cast<uint32_t>(row.components);

    const size_t bytes = n * size_t(row.components) * sizeof(float);
    if (scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API))
    {
        // The scene's own manager, not the process default: under multi-GPU the backend view runs in
        // this scene's context, so the column must be allocated there to be addressable by the
        // scatter that will read it at commit.
        PxCudaContextManager* ctxMgr = scene->getCudaContextManager();
        PxCudaContext* cu = ctxMgr ? ctxMgr->getCudaContext() : nullptr;
        if (!cu)
        {
            CARB_LOG_ERROR("ovxWriteAttribute: DirectGPU scene has no CUDA context; no group emitted.");
            return false;
        }
        PxScopedCudaLock _lock(*ctxMgr);
        cu->memAlloc(&slot->deviceData, bytes);
        if (!slot->deviceData)
        {
            CARB_LOG_ERROR("ovxWriteAttribute: articulation column allocation failed (%zu bytes).", bytes);
            return false;
        }
        slot->ctxMgr = ctxMgr;
        slot->deviceOrdinal = ctxMgr->getDevice();
    }
    else
    {
        slot->floats.assign(n * size_t(row.components), 0.0f);
    }

    // No shared cache entry here. g_rigidReadCache is keyed by a RigidBodyType, which an articulation
    // is not, and the read has no articulation cache to share yet (its TASK-04 is what would add
    // one). Building the list outright is correct and leaves nothing to invalidate wrongly.
    ovx_path_dictionary_t* dict = nullptr;
    slot->list = omni::physics::ovstage::buildPathList(source, keys.data(), keys.size(), &dict);
    if (slot->list == OVX_INVALID_PRIMPATH_LIST || !dict)
    {
        CARB_LOG_ERROR("ovxWriteAttribute: failed to build ovx_primpath_list_t for '%s' (%zu articulations) -- "
                       "the active backend is not ovstage (this write is ovstage-only).",
                       s.attribute.c_str(), keys.size());
        return false;
    }
    s.dict = dict;

    slot->shapes.assign(1, static_cast<int64_t>(n));
    slot->tensors.assign(1, DLTensor{});
    DLTensor& t = slot->tensors[0];
    t.data = slot->deviceData ? reinterpret_cast<void*>(slot->deviceData) : static_cast<void*>(slot->floats.data());
    t.device = slot->deviceOrdinal >= 0 ? DLDevice{ kDLCUDA, slot->deviceOrdinal } : DLDevice{ kDLCPU, 0 };
    t.ndim = 1;
    t.dtype = DLDataType{ kDLFloat, 32, static_cast<uint16_t>(row.components) };
    t.shape = slot->shapes.data();
    t.strides = nullptr; // contiguous row-major SoA, as the read emits
    t.byte_offset = 0;

    ovstage_map_group_t& g = slot->group;
    g.prims.list = slot->list;
    g.prims.offset = 0;
    g.prims.count = static_cast<uint32_t>(n);
    g.prims.index_map = nullptr;
    g.data.tensors = slot->tensors.data();
    g.data.tensor_count = static_cast<uint32_t>(slot->tensors.size());
    g.data.count = 0;
    g.data.index_map = nullptr;

    s.groups.push_back(std::move(slot));
    return true;
}

// One scene's tendon group: a FIXED group, one row per tendon-root prim, carrying 1 float (or 2 for
// a fixed tendon's limit pair). Mirrors the tendon read's emit, which is fixed for the same reason:
// a tendon has one value per property, not a per-element array.
bool planTendonGroup(WriteSession& s,
                     IPhysicsSource& source,
                     PxScene* scene,
                     bool fixed,
                     const TendonWriteAttributeRow& row)
{
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
        return false;

    // The read's walk, through the shared accessor: the write enumerates no tendons of its own, so
    // the two directions cannot disagree about which prims are tendon roots or what order they take.
    ovx::TendonReadCacheEntry& tc = ovx::tendonCacheEntry(scene, fixed);
    if (tc.tendons.empty())
        return true; // this scene owns no tendons of this kind -- not a failure

    const bool gpu = scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API);
    uint64_t generation = 0;
    const std::vector<omni::physx::tensors::ArticulationEntry>* entriesPtr = nullptr;
    const std::unordered_map<const PxArticulationReducedCoordinate*, PxU32>* artiRowMap = nullptr;
    if (gpu)
    {
        omni::physx::tensors::GpuSimulationView* sv = backend->acquireSceneView(scene, &generation);
        if (!sv || !sv->supersetArticulationView(&artiRowMap, &entriesPtr))
            return false;
    }
    else
    {
        omni::physx::tensors::CpuSimulationView* sv = backend->acquireCpuSceneView(scene, &generation);
        if (!sv || !sv->supersetArticulationView(&artiRowMap, &entriesPtr))
            return false;
    }
    if (!artiRowMap || !entriesPtr)
        return false;

    // Derived here rather than through a shared helper, unlike the DOF records. That derivation
    // folds the degree convention and the body0IsParent sign and MUST NOT drift between directions;
    // this one is `record = (localToRow[viewArtiIdx], tendonIdx)` with no rules in it at all, so a
    // shared helper would carry the cost of a seam without protecting anything.
    tc.recs.clear();
    tc.keys.clear();
    tc.recs.reserve(tc.tendons.size());
    tc.keys.reserve(tc.tendons.size());
    for (const ovx::TendonRec& tr : tc.tendons)
    {
        if (tr.viewArtiIdx >= tc.artis.size())
            return false;
        const std::unordered_map<const PxArticulationReducedCoordinate*, PxU32>::const_iterator it =
            artiRowMap->find(tc.artis[tr.viewArtiIdx]);
        if (it == artiRowMap->end())
            return false; // the cached view does not describe this articulation set
        omni::physx::tensors::ArticulationTendonOvStageRecord r;
        r.viewArtiIdx = it->second;
        r.tendonIdx = tr.tendonIdx;
        tc.recs.push_back(r);
        tc.keys.push_back(tr.key);
    }
    tc.generation = generation;

    const uint32_t numOut = static_cast<uint32_t>(tc.recs.size());
    const uint32_t comp = omni::physx::tensors::tendonPropertyComponents(row.prop);

    std::unique_ptr<GroupSlot> slot(new GroupSlot());
    slot->scene = scene;
    slot->tendonRow = &row;
    slot->tendonFixed = fixed;
    slot->numOut = numOut;
    slot->comp = comp;
    slot->keys = tc.keys;

    const size_t bytes = size_t(numOut) * comp * sizeof(float);
    if (gpu)
    {
        PxCudaContextManager* ctxMgr = scene->getCudaContextManager();
        PxCudaContext* cu = ctxMgr ? ctxMgr->getCudaContext() : nullptr;
        if (!cu)
            return false;
        PxScopedCudaLock _lock(*ctxMgr);
        cu->memAlloc(&slot->deviceData, bytes);
        cu->memAlloc(&slot->recsDev,
                     size_t(numOut) * sizeof(omni::physx::tensors::ArticulationTendonOvStageRecord));
        if (!slot->deviceData || !slot->recsDev)
            return false;
        cu->memcpyHtoD(slot->recsDev, tc.recs.data(),
                       size_t(numOut) * sizeof(omni::physx::tensors::ArticulationTendonOvStageRecord));
        slot->ctxMgr = ctxMgr;
        slot->deviceOrdinal = ctxMgr->getDevice();
    }
    else
    {
        slot->floats.assign(size_t(numOut) * comp, 0.0f);
    }

    ovx_path_dictionary_t* dict = nullptr;
    const bool haveHandles = tc.handles.size() == tc.keys.size();
    slot->list = haveHandles ?
                     omni::physics::ovstage::buildPathListFromHandles(source, tc.handles.data(),
                                                                     tc.handles.size(), &dict) :
                     omni::physics::ovstage::buildPathList(source, tc.keys.data(), tc.keys.size(), &dict);
    if (slot->list == OVX_INVALID_PRIMPATH_LIST || !dict)
        return false;
    s.dict = dict;

    slot->shapes.assign(1, static_cast<int64_t>(numOut));
    slot->tensors.assign(1, DLTensor{});
    DLTensor& t = slot->tensors[0];
    t.data = gpu ? reinterpret_cast<void*>(slot->deviceData) : static_cast<void*>(slot->floats.data());
    t.device = gpu ? DLDevice{ kDLCUDA, slot->deviceOrdinal } : DLDevice{ kDLCPU, 0 };
    t.ndim = 1;
    t.dtype = DLDataType{ kDLFloat, 32, static_cast<uint16_t>(comp) };
    t.shape = slot->shapes.data();
    t.strides = nullptr;
    t.byte_offset = 0;

    ovstage_map_group_t& g = slot->group;
    g.prims.list = slot->list;
    g.prims.offset = 0;
    g.prims.count = numOut;
    g.prims.index_map = nullptr;
    g.data.tensors = slot->tensors.data();
    g.data.tensor_count = 1;
    g.data.count = 0;
    g.data.index_map = nullptr;

    s.groups.push_back(std::move(slot));
    return true;
}

// One scene's vehicle-wheel control group: a FIXED group, one row per wheel prim carrying one float.
// Mirrors the vehicle read's emit, which is fixed for the same reason -- a wheel has one value per
// control, not an array.
bool planVehicleGroup(WriteSession& s,
                      IPhysicsSource& source,
                      PxScene* scene,
                      const VehicleWriteAttributeRow& row)
{
    // The read's enumeration, shared. Not refreshed here, unlike the articulation and tendon
    // accessors: the vehicle walk needs a CpuSimulationView and the scene's InternalScene, which the
    // read holds at that point and this does not. So a session opened before any vehicle read on
    // this scene reports the miss rather than deriving a second enumeration that could disagree.
    ovx::VehicleReadCacheEntry* vc = ovx::vehicleCacheEntry(scene);
    if (!vc)
    {
        CARB_LOG_ERROR("ovxWriteAttribute: no vehicle enumeration cached for this scene -- read a wheel "
                       "attribute once before writing one, so both directions agree on the wheel set.");
        return false;
    }
    if (vc->recs.empty())
        return true; // this scene owns no vehicle wheels -- not a failure

    // Vehicles are CPU-only and permanently so: their suspension and sticky-tire constraints are
    // custom PxConstraints with a CPU solver-prep function PhysX refuses under DirectGPU. So there is
    // no device column to emit here and no GPU branch to write -- BaseVehicleView has no Gpu/Cpu
    // derivatives for the same reason.
    const uint32_t numOut = static_cast<uint32_t>(vc->recs.size());

    std::unique_ptr<GroupSlot> slot(new GroupSlot());
    slot->scene = scene;
    slot->vehicleRow = &row;
    slot->numOut = numOut;
    slot->comp = 1;
    slot->keys = vc->keys;
    slot->floats.assign(numOut, 0.0f);

    ovx_path_dictionary_t* dict = nullptr;
    const bool haveHandles = vc->handles.size() == vc->keys.size();
    slot->list = haveHandles ? omni::physics::ovstage::buildPathListFromHandles(source, vc->handles.data(),
                                                                               vc->handles.size(), &dict) :
                               omni::physics::ovstage::buildPathList(source, vc->keys.data(), vc->keys.size(),
                                                                    &dict);
    if (slot->list == OVX_INVALID_PRIMPATH_LIST || !dict)
        return false;
    s.dict = dict;

    slot->shapes.assign(1, static_cast<int64_t>(numOut));
    slot->tensors.assign(1, DLTensor{});
    DLTensor& t = slot->tensors[0];
    t.data = static_cast<void*>(slot->floats.data());
    t.device = DLDevice{ kDLCPU, 0 };
    t.ndim = 1;
    t.dtype = DLDataType{ kDLFloat, 32, 1 };
    t.shape = slot->shapes.data();
    t.strides = nullptr;
    t.byte_offset = 0;

    ovstage_map_group_t& g = slot->group;
    g.prims.list = slot->list;
    g.prims.offset = 0;
    g.prims.count = numOut;
    g.prims.index_map = nullptr;
    g.data.tensors = slot->tensors.data();
    g.data.tensor_count = 1;
    g.data.count = 0;
    g.data.index_map = nullptr;

    s.groups.push_back(std::move(slot));
    return true;
}

// One instancer's ARRAY group: a single tensor carrying that instancer's FULL instance array,
// placed by index, with holes where an instance has no live body. The read emits exactly this shape
// under the attribute's PLURAL name, which is why a `position` session produces both the standalone
// fixed groups and one of these per instancer.
bool planInstancerGroup(WriteSession& s,
                        IPhysicsSource& source,
                        PxScene* scene,
                        omni::physx::tensors::BasePointInstancerView* view,
                        const ObjectKey& key,
                        uint32_t arrayLength,
                        uint32_t instancerIdx,
                        omni::physx::tensors::BasePointInstancerView::InstancerColumn column,
                        uint32_t comp)
{
    if (arrayLength == 0)
        return true;

    std::unique_ptr<GroupSlot> slot(new GroupSlot());
    slot->isInstancer = true;
    slot->scene = scene;
    slot->instancerView = view;
    slot->instancerColumn = column;
    slot->instancerIdx = instancerIdx;
    slot->comp = comp;
    slot->numOut = arrayLength;
    // Residency follows the scene's pipeline, as every other group's does. This allocated a HOST
    // column unconditionally while the device path refused; once that path landed, the kernel was
    // handed a host pointer and faulted with an illegal memory access -- reported asynchronously at
    // the READ's next sync, which is what made it look like the read's bug.
    const size_t bytes = size_t(arrayLength) * comp * sizeof(float);
    if (scene && scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API))
    {
        PxCudaContextManager* ctxMgr = scene->getCudaContextManager();
        PxCudaContext* cu = ctxMgr ? ctxMgr->getCudaContext() : nullptr;
        if (!cu)
        {
            CARB_LOG_ERROR("ovxWriteAttribute: DirectGPU scene has no CUDA context; no instancer group.");
            return false;
        }
        PxScopedCudaLock _lock(*ctxMgr);
        cu->memAlloc(&slot->deviceData, bytes);
        if (!slot->deviceData)
        {
            CARB_LOG_ERROR("ovxWriteAttribute: instancer column allocation failed (%zu bytes).", bytes);
            return false;
        }
        slot->ctxMgr = ctxMgr;
        slot->deviceOrdinal = ctxMgr->getDevice();
    }
    else
    {
        slot->floats.assign(size_t(arrayLength) * comp, 0.0f);
    }

    ovx_path_dictionary_t* dict = nullptr;
    slot->list = omni::physics::ovstage::buildPathList(source, &key, 1, &dict);
    if (slot->list == OVX_INVALID_PRIMPATH_LIST || !dict)
        return false;
    s.dict = dict;

    // ONE prim -- the instancer -- carrying one tensor of arrayLength rows. That is the array shape:
    // the group's prim count is 1, not arrayLength, because the instances are not prims.
    slot->shapes.assign(1, static_cast<int64_t>(arrayLength));
    slot->tensors.assign(1, DLTensor{});
    DLTensor& t = slot->tensors[0];
    t.data = slot->deviceData ? reinterpret_cast<void*>(slot->deviceData) :
                                static_cast<void*>(slot->floats.data());
    t.device = slot->deviceOrdinal >= 0 ? DLDevice{ kDLCUDA, slot->deviceOrdinal } : DLDevice{ kDLCPU, 0 };
    t.ndim = 1;
    t.dtype = DLDataType{ kDLFloat, 32, static_cast<uint16_t>(comp) };
    t.shape = slot->shapes.data();
    t.strides = nullptr;
    t.byte_offset = 0;

    ovstage_map_group_t& g = slot->group;
    g.prims.list = slot->list;
    g.prims.offset = 0;
    g.prims.count = 1;
    g.prims.index_map = nullptr;
    g.data.tensors = slot->tensors.data();
    g.data.tensor_count = 1;
    g.data.count = 0;
    g.data.index_map = nullptr;

    s.groups.push_back(std::move(slot));
    return true;
}

// One deformable body's sim-mesh ARRAY group (ADR-0012): a single tensor carrying that body's full
// per-vertex column. The shape the read emits per body, so a column reads and writes back.
//
// Device-resident, and here that follows the destination rather than the scene: PhysX exposes the
// sim-mesh state only as device buffers (getSimPositionInvMassBufferD and friends), so there is no
// host form to publish even if a caller wanted one. Deformables are GPU-only anyway.
// The deformable sim-mesh group (ADR-0012): ONE group for the scene, one tensor per body, every
// tensor a view into a single allocation at its own byte offset.
//
// That is the read's shape and it is not merely tidier. A group per body meant one device
// allocation, one host-to-device fill and one free PER BODY -- measured at 128 envs, that was 80% of
// the write's cost, with the scatter itself only 20%. It also made the two directions
// non-interchangeable: the read hands back one group of N tensors, which a write wanting N groups
// cannot accept.
//
// Device-resident, which follows the destination rather than the scene: PhysX exposes sim-mesh
// state only as device buffers, so there is no host form to publish.
bool planDeformableGroup(WriteSession& s,
                         IPhysicsSource& source,
                         PxScene* scene,
                         bool isVolume,
                         const ovx::DeformableWriteTargets& targets,
                         bool velocity)
{
    if (targets.bodies.empty())
        return true;

    PxCudaContextManager* ctxMgr = scene ? scene->getCudaContextManager() : nullptr;
    PxCudaContext* cu = ctxMgr ? ctxMgr->getCudaContext() : nullptr;
    if (!cu)
    {
        CARB_LOG_ERROR("ovxWriteAttribute: deformable write needs a CUDA context -- none available.");
        return false;
    }

    std::unique_ptr<GroupSlot> slot(new GroupSlot());
    slot->isDeformable = true;
    slot->deformableVolume = isVolume;
    slot->deformableVelocity = velocity;
    slot->scene = scene;
    slot->comp = 3u;

    // The column is COMPACT and the meshes are ragged, so each body's slice starts where the last
    // one ended -- the same layout the read's gather writes into, and the reason a tensor carries a
    // byte offset rather than every body getting a buffer.
    size_t totalFloats = 0;
    for (size_t i = 0; i < targets.bodies.size(); ++i)
    {
        if (!targets.bodies[i] || !targets.internals[i] || targets.counts[i] == 0)
            continue;
        slot->deformableBodies.push_back(targets.bodies[i]);
        slot->deformableInternals.push_back(targets.internals[i]);
        slot->deformableCounts.push_back(targets.counts[i]);
        slot->deformableFloatOffsets.push_back(totalFloats);
        slot->keys.push_back(targets.keys[i]);
        totalFloats += size_t(targets.counts[i]) * 3u;
    }
    if (slot->deformableBodies.empty())
        return true;
    slot->numOut = static_cast<uint32_t>(slot->deformableBodies.size());

    {
        PxScopedCudaLock _lock(*ctxMgr);
        cu->memAlloc(&slot->deviceData, totalFloats * sizeof(float));
    }
    if (!slot->deviceData)
    {
        CARB_LOG_ERROR("ovxWriteAttribute: deformable column allocation failed (%zu floats).", totalFloats);
        return false;
    }
    slot->ctxMgr = ctxMgr;
    slot->deviceOrdinal = ctxMgr->getDevice();

    ovx_path_dictionary_t* dict = nullptr;
    slot->list = omni::physics::ovstage::buildPathList(source, slot->keys.data(), slot->keys.size(), &dict);
    if (slot->list == OVX_INVALID_PRIMPATH_LIST || !dict)
        return false;
    s.dict = dict;

    slot->shapes.assign(slot->numOut, 0);
    slot->tensors.assign(slot->numOut, DLTensor{});
    for (uint32_t i = 0; i < slot->numOut; ++i)
    {
        slot->shapes[i] = static_cast<int64_t>(slot->deformableCounts[i]);
        DLTensor& t = slot->tensors[i];
        t.data = reinterpret_cast<void*>(slot->deviceData);
        t.device = DLDevice{ kDLCUDA, slot->deviceOrdinal };
        t.ndim = 1;
        t.dtype = DLDataType{ kDLFloat, 32, 3 };
        t.shape = &slot->shapes[i];
        t.strides = nullptr;
        t.byte_offset = slot->deformableFloatOffsets[i] * sizeof(float);
    }

    ovstage_map_group_t& g = slot->group;
    g.prims.list = slot->list;
    g.prims.offset = 0;
    g.prims.count = slot->numOut;
    g.prims.index_map = nullptr;
    g.data.tensors = slot->tensors.data();
    g.data.tensor_count = slot->numOut;
    g.data.count = 0;
    g.data.index_map = nullptr;

    s.groups.push_back(std::move(slot));
    return true;
}

// Publish a deformable sim-mesh column into PhysX's device buffers (ADR-0012). One group carries
// every body, so this walks them.
// Defined below; forward-declared so these device/host scatter paths can honour the caller's producer
// handoff before reading the column, exactly as the sibling scatter paths do (ADR-0012 Decision 3).
void honourCallerSync(const GroupSlot& slot, ovstage_cuda_sync_t sync);

bool scatterDeformableGroup(GroupSlot& slot, ovstage_cuda_sync_t writeDoneSync)
{
    if (!slot.ctxMgr || slot.deformableBodies.size() != slot.numOut)
        return false;

    // Before the kernel below reads the caller's device column (slot.deviceData): order behind the
    // producer that filled it, as every sibling scatter path does. Without this, an async producer
    // fill can be read here mid-write.
    honourCallerSync(slot, writeDoneSync);

    PxScopedCudaLock _lock(*slot.ctxMgr);
    for (uint32_t i = 0; i < slot.numOut; ++i)
    {
        PxDeformableBody* body = slot.deformableBodies[i];
        omni::physx::internal::InternalDeformableBody* internalBody = slot.deformableInternals[i];
        if (!body || !internalBody)
            continue;

        // The concrete type decides both which buffer this is and which dirty flag announces it.
        // Taken from PhysX via is<>() rather than from the query type that led here, the same rule
        // the material path follows: these are downcasts, so a disagreement must cost a refused
        // write rather than undefined behaviour.
        PxVec4* dst = nullptr;
        PxDeformableVolume* volume = body->is<PxDeformableVolume>();
        PxDeformableSurface* surface = volume ? nullptr : body->is<PxDeformableSurface>();
        if (slot.deformableVolume != (volume != nullptr))
        {
            CARB_LOG_ERROR("ovxCommitGroup: deformable body is not the kind the query selected; nothing "
                           "written for it.");
            return false;
        }
        if (volume)
            dst = slot.deformableVelocity ? volume->getSimVelocityBufferD() :
                                            volume->getSimPositionInvMassBufferD();
        else if (surface)
            dst = slot.deformableVelocity ? surface->getVelocityBufferD() :
                                            surface->getPositionInvMassBufferD();
        if (!dst)
        {
            CARB_LOG_ERROR("ovxCommitGroup: deformable body has no device buffer for this column.");
            return false;
        }

        // Points arrive SIM-MESH-LOCAL, the frame the read publishes them in, so the stored world
        // value is the read's reframe inverted. affineInverse rather than a rigid inverse: the
        // transform may carry scale. Velocities are world on both directions and pass through.
        omni::physx::tensors::PointSetTransform xf{};
        if (!slot.deformableVelocity)
        {
            const PxMat44d l2w = omni::physx::affineInverse(internalBody->mWorldToSimMesh);
            // Element-wise narrow of PhysX's own storage, which is what the read's
            // toPointSetTransform does. Spelling the column layout out by hand would restate an
            // assumption that only has to be wrong once, and a transposed reframe is a
            // plausible-looking wrong answer.
            const double* const srcM = l2w.front();
            for (int c = 0; c < 16; ++c)
                xf.m[c] = float(srcM[c]);
        }

        const float* const src =
            reinterpret_cast<const float*>(slot.deviceData) + slot.deformableFloatOffsets[i];
        if (!omni::physx::tensors::submitPointSetColumnOvStage(dst, src, xf, slot.deformableCounts[i],
                                                              !slot.deformableVelocity))
        {
            CARB_LOG_ERROR("ovxCommitGroup: deformable column scatter failed.");
            return false;
        }

        // Mandatory, and stated in PhysX's own header: without it the buffer is written and the
        // solver never looks. The flag names the buffer, which is why it is chosen beside the getter.
        if (volume)
            volume->markDirty(slot.deformableVelocity ? PxDeformableVolumeDataFlag::eSIM_VELOCITY :
                                                        PxDeformableVolumeDataFlag::eSIM_POSITION_INVMASS);
        else
            surface->markDirty(slot.deformableVelocity ? PxDeformableSurfaceDataFlag::eVELOCITY :
                                                         PxDeformableSurfaceDataFlag::ePOSITION_INVMASS);
    }
    return true;
}

// The kOvxDeformableMaterial group (ADR-0012): one FIXED tensor stacking every material this
// attribute can serve, one f32 per prim.
//
// Scene-less, unlike every other planner here, and that is the object's nature rather than an
// omission: a PxDeformableMaterial belongs to no scene, so there is one group for the whole session
// instead of one per scene. The read walks the record database the same way, with no scene filter.
//
// Host-resident on every scene. Material properties are simulation INPUTS that PhysX never writes
// back, so there is no device copy to hand out -- the same reasoning that makes the rigid-body
// properties and the DOF properties host-only.
bool planDeformableMaterialGroup(WriteSession& s, IPhysicsSource& source, const DeformableMaterialWriteRow& row)
{
    std::vector<ObjectKey> keys;
    std::vector<PxDeformableMaterial*> materials;

    omni::physx::internal::InternalPhysXDatabase& db =
        omni::physx::OmniPhysX::getInstance().getInternalPhysXDatabase();
    for (const omni::physx::internal::InternalDatabase::Record& rec : db.getRecords())
    {
        const bool recordSaysSurface = rec.mType == omni::physx::ePTDeformableSurfaceMaterial;
        if ((rec.mType != omni::physx::ePTDeformableVolumeMaterial && !recordSaysSurface) || !rec.mPtr)
            continue;
        PxDeformableMaterial* material = static_cast<PxDeformableMaterial*>(rec.mPtr);

        // PhysX decides the kind, not the record -- the surface-only setters downcast on this, so
        // taking the record's word would turn a mislabelled record into undefined behaviour. The
        // read states the same rule; both must agree or an attribute could be readable on a prim it
        // is not writable on.
        const bool surface = material->is<PxDeformableSurfaceMaterial>();
        if (surface != recordSaysSurface)
        {
            CARB_LOG_WARN("ovxWriteAttribute: deformable material is recorded as %s but PhysX reports "
                          "%s -- its surface-only properties are not written.",
                          recordSaysSurface ? "surface" : "volume", surface ? "surface" : "volume");
        }
        if (row.surfaceOnly && !surface)
            continue; // a volume material has no such property, so it takes no row -- as in the read
        keys.push_back(rec.mKey);
        materials.push_back(material);
    }
    if (keys.empty())
        return true; // nothing this attribute can serve -- not a failure

    const uint32_t numOut = static_cast<uint32_t>(keys.size());

    std::unique_ptr<GroupSlot> slot(new GroupSlot());
    slot->deformableMatRow = &row;
    slot->materials = std::move(materials);
    slot->keys = keys;
    slot->numOut = numOut;
    slot->comp = 1u;
    slot->floats.assign(size_t(numOut), 0.0f);

    ovx_path_dictionary_t* dict = nullptr;
    slot->list = omni::physics::ovstage::buildPathList(source, keys.data(), keys.size(), &dict);
    if (slot->list == OVX_INVALID_PRIMPATH_LIST || !dict)
        return false;
    s.dict = dict;

    slot->shapes.assign(1, static_cast<int64_t>(numOut));
    slot->tensors.assign(1, DLTensor{});
    DLTensor& t = slot->tensors[0];
    t.data = static_cast<void*>(slot->floats.data());
    t.device = DLDevice{ kDLCPU, 0 };
    t.ndim = 1;
    t.dtype = DLDataType{ kDLFloat, 32, 1 };
    t.shape = slot->shapes.data();
    t.strides = nullptr;
    t.byte_offset = 0;

    ovstage_map_group_t& g = slot->group;
    g.prims.list = slot->list;
    g.prims.offset = 0;
    g.prims.count = numOut;
    g.prims.index_map = nullptr;
    g.data.tensors = slot->tensors.data();
    g.data.tensor_count = 1;
    g.data.count = 0;
    g.data.index_map = nullptr;

    s.groups.push_back(std::move(slot));
    return true;
}

// Publish one deformable-material column (ADR-0012).
bool scatterDeformableMaterialGroup(GroupSlot& slot, ovstage_cuda_sync_t writeDoneSync)
{
    if (!slot.deformableMatRow || slot.materials.size() != slot.numOut)
        return false;
    // Honour the caller's producer handoff before reading the column, even on the host path -- a caller
    // may have produced a host column with a CUDA op and handed the event over.
    honourCallerSync(slot, writeDoneSync);
    const float* const src = slot.floats.data();
    for (uint32_t i = 0; i < slot.numOut; ++i)
    {
        if (PxDeformableMaterial* m = slot.materials[i])
            slot.deformableMatRow->write(*m, src[i]);
    }
    return true;
}

// The particle group (ADR-0012): ONE group for the scene, one tensor per SET, each a view into a
// single column at its own byte offset -- the read's shape, so a column reads and writes back.
//
// HOST-resident on every scene, including DirectGPU, and that is the destination's property rather
// than an unfinished device path. A particle write lands in the set's pinned host staging pair and
// raises its upload flag; PhysX copies it to the device buffer at the next step. Publishing a device
// column would mean writing PhysX's live buffer behind that mechanism's back.
//
// Routing through the staging pair also makes the write READABLE before the next step: the read's
// pending path serves mPositions / mVelocities exactly when the matching flag is raised.
bool planParticleGroup(WriteSession& s,
                       IPhysicsSource& source,
                       PxScene* scene,
                       const ovx::ParticleWriteTargets& targets,
                       bool velocity)
{
    if (targets.sets.empty())
        return true;

    std::unique_ptr<GroupSlot> slot(new GroupSlot());
    slot->isParticle = true;
    slot->particleVelocity = velocity;
    slot->scene = scene;
    slot->comp = 3u;

    size_t totalFloats = 0;
    for (size_t i = 0; i < targets.sets.size(); ++i)
    {
        if (!targets.sets[i] || targets.counts[i] == 0)
            continue;
        slot->particleSets.push_back(targets.sets[i]);
        slot->particleCounts.push_back(targets.counts[i]);
        slot->particleFloatOffsets.push_back(totalFloats);
        slot->keys.push_back(targets.keys[i]);
        totalFloats += size_t(targets.counts[i]) * 3u;
    }
    if (slot->particleSets.empty())
        return true;
    slot->numOut = static_cast<uint32_t>(slot->particleSets.size());
    slot->floats.assign(totalFloats, 0.0f);

    ovx_path_dictionary_t* dict = nullptr;
    slot->list = omni::physics::ovstage::buildPathList(source, slot->keys.data(), slot->keys.size(), &dict);
    if (slot->list == OVX_INVALID_PRIMPATH_LIST || !dict)
        return false;
    s.dict = dict;

    slot->shapes.assign(slot->numOut, 0);
    slot->tensors.assign(slot->numOut, DLTensor{});
    for (uint32_t i = 0; i < slot->numOut; ++i)
    {
        slot->shapes[i] = static_cast<int64_t>(slot->particleCounts[i]);
        DLTensor& t = slot->tensors[i];
        t.data = static_cast<void*>(slot->floats.data());
        t.device = DLDevice{ kDLCPU, 0 };
        t.ndim = 1;
        t.dtype = DLDataType{ kDLFloat, 32, 3 };
        t.shape = &slot->shapes[i];
        t.strides = nullptr;
        t.byte_offset = slot->particleFloatOffsets[i] * sizeof(float);
    }

    ovstage_map_group_t& g = slot->group;
    g.prims.list = slot->list;
    g.prims.offset = 0;
    g.prims.count = slot->numOut;
    g.prims.index_map = nullptr;
    g.data.tensors = slot->tensors.data();
    g.data.tensor_count = slot->numOut;
    g.data.count = 0;
    g.data.index_map = nullptr;

    s.groups.push_back(std::move(slot));
    return true;
}

// The ovstage drain uses this to tell a PARTIAL COMMIT from "not handled". It is set at each scatter path's
// actual COMMIT point -- right before the backend setter, or (for the sequential particle path) right before
// the first per-set buffer write -- AFTER every pre-commit check has passed. Set it any earlier and a pre-commit failure
// that wrote NOTHING (no backend, no superset view, a record list that moved since planning) would be
// misreported as a partial commit, which holds the drain cursor forever on a deterministic failure instead of
// falling back. Each drain scatter path (rigid, vehicle, joint, particle) sets it at its own commit point; a
// new drain scatter path must do the same. applyOvstageValueBatch resets it before dispatch and reads it
// afterward: a scatter that committed part of a batch and then failed means the drain must NOT fall back
// (that would double-apply the committed part). The write-session commit path sets it too, harmlessly -- only
// applyOvstageValueBatch reads it, and it resets first. thread_local so concurrent instances do not race.
thread_local bool t_scatterCommitAttempted = false;

// Publish a particle column into each set's staging array (ADR-0012).
bool scatterParticleGroup(GroupSlot& slot, ovstage_cuda_sync_t writeDoneSync)
{
    if (slot.particleSets.size() != slot.numOut)
        return false;
    // Honour the caller's producer handoff before reading the column, even on the host path -- a caller
    // may have produced a host column with a CUDA op and handed the event over.
    honourCallerSync(slot, writeDoneSync);

    for (uint32_t g = 0; g < slot.numOut; ++g)
    {
        omni::physx::internal::InternalParticleSet* ps = slot.particleSets[g];
        if (!ps)
            continue;
        // Checked HERE and not at plan, matching where the read checks it: mEnabled is
        // runtime-mutable and disabling a set retires no database record, so a set disabled between
        // plan and commit must drop out at the commit or it would be published from a stale decision.
        if (!ps->mEnabled)
            continue;

        ::physx::PxVec4* const dst = slot.particleVelocity ? ps->mVelocities : ps->mPositions;
        if (!dst)
        {
            CARB_LOG_ERROR("ovxCommitGroup: particle set has no staging buffer for '%s'.",
                           slot.particleVelocity ? "velocities" : "points");
            return false;
        }
        // Commit point: this set's buffer is valid and about to be written, so flag it HERE, not before the
        // loop. A `!dst` bail on the FIRST set writes nothing; flagging earlier would report a partial commit
        // (Failed -> cursor held) for a non-commit failure. Once a set HAS been written, a later set's `!dst`
        // is a genuine partial commit -- the flag is already set, so it correctly stays Failed.
        t_scatterCommitAttempted = true;
        // The count the group was planned with can no longer exceed the set: resize() moves the
        // object lifetime epoch, which retires the plan. Clamped rather than trusted because the
        // cost is one comparison and the failure it prevents is a heap write past a pinned buffer.
        const uint32_t n = std::min<uint32_t>(slot.particleCounts[g], ps->mNumParticles);
        const float* const src = slot.floats.data() + slot.particleFloatOffsets[g];

        if (slot.particleVelocity)
        {
            for (uint32_t i = 0; i < n; ++i)
            {
                // .w is preserved, never written: the read drops it and the caller's column is vec3,
                // so a four-component store would put a coordinate where PhysX keeps its own lane.
                dst[i].x = src[i * 3 + 0];
                dst[i].y = src[i * 3 + 1];
                dst[i].z = src[i * 3 + 2];
            }
            ps->mUploadDirtyFlags |= omni::physx::internal::ParticleBufferFlags::eVELOCITIES;
            continue;
        }

        // Points arrive PRIM-LOCAL, because that is the frame the read publishes them in. The stored
        // array is world, so this is the read's reframe inverted -- and inverted with affineInverse
        // rather than a rigid inverse, since a prim transform may carry scale.
        const ::physx::PxMat44d l2w = omni::physx::affineInverse(ps->mWorldToLocal);
        for (uint32_t i = 0; i < n; ++i)
        {
            const double x = double(src[i * 3 + 0]);
            const double y = double(src[i * 3 + 1]);
            const double z = double(src[i * 3 + 2]);
            // Column-major: the product is l2w * v, and writing it out beats guessing which of
            // PxMat44T's transform overloads treats the vector as a point.
            const double wx = l2w.column0.x * x + l2w.column1.x * y + l2w.column2.x * z + l2w.column3.x;
            const double wy = l2w.column0.y * x + l2w.column1.y * y + l2w.column2.y * z + l2w.column3.y;
            const double wz = l2w.column0.z * x + l2w.column1.z * y + l2w.column2.z * z + l2w.column3.z;
            dst[i].x = float(wx);
            dst[i].y = float(wy);
            dst[i].z = float(wz);
        }
        // The flag is what makes the write visible: updateParticles copies the staging array into the
        // device buffer and raises PxParticleBufferFlag::eUPDATE_POSITION only when this is set.
        ps->mUploadDirtyFlags |= omni::physx::internal::ParticleBufferFlags::ePOSITIONS;
    }
    return true;
}

// Which instancer column an attribute writes, if any. The SINGULAR names, matching what the read
// accepts -- it publishes the instancer group under the plural one, but a caller asks with the
// singular, and the write mirrors that.
bool instancerColumnForWrite(std::string_view name,
                             omni::physx::tensors::BasePointInstancerView::InstancerColumn& out,
                             uint32_t& comp)
{
    using Column = omni::physx::tensors::BasePointInstancerView::InstancerColumn;
    if (name == OvxAttr::kPosition)
    {
        out = Column::eLocalPosition;
        comp = 3;
    }
    else if (name == OvxAttr::kOrientation)
    {
        out = Column::eLocalOrientation;
        comp = 4;
    }
    else if (name == OvxAttr::kLinearVelocity)
    {
        out = Column::eLinearVelocity;
        comp = 3;
    }
    else if (name == OvxAttr::kAngularVelocity)
    {
        out = Column::eAngularVelocity;
        comp = 3;
    }
    else
    {
        return false;
    }
    return true;
}

// One scene's joint-PROPERTY group. Same array shape as the DOF-state group -- one tensor per joint
// prim, all pointing into a flat per-axis column -- because it names the same slots. What differs is
// residency: these are simulation inputs with no device copy, so the column is HOST even on a
// DirectGPU scene, exactly as the rigid-body property columns are.
bool planJointPropertyGroup(WriteSession& s,
                            IPhysicsSource& source,
                            PxScene* scene,
                            const JointPropertyWriteRow& row)
{
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
        return false;

    if (!ovx::refreshArticulationCache(ovx::allPhysicsScenes()))
    {
        CARB_LOG_ERROR("ovxWriteAttribute: the articulation structural walk failed; nothing was written.");
        return false;
    }
    ovx::ArticulationReadCacheEntry* jcPtr = ovx::articulationCacheEntry(scene);
    if (!jcPtr || jcPtr->joints.empty())
        return true; // this scene owns no joints -- not a failure
    ovx::ArticulationReadCacheEntry& jc = *jcPtr;

    // The records name superset rows, so they are derived against the view that numbered them --
    // through the same helper the DOF-state path uses, so the two cannot disagree about which axis a
    // slot is. The properties are host-sourced, but the RECORDS are not: they still index the view.
    const bool gpu = scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API);
    uint64_t generation = 0;
    const std::vector<omni::physx::tensors::ArticulationEntry>* entriesPtr = nullptr;
    const std::unordered_map<const PxArticulationReducedCoordinate*, PxU32>* artiRowMap = nullptr;
    if (gpu)
    {
        omni::physx::tensors::GpuSimulationView* sv = backend->acquireSceneView(scene, &generation);
        if (!sv || !sv->supersetArticulationView(&artiRowMap, &entriesPtr))
            return false;
    }
    else
    {
        omni::physx::tensors::CpuSimulationView* sv = backend->acquireCpuSceneView(scene, &generation);
        if (!sv || !sv->supersetArticulationView(&artiRowMap, &entriesPtr))
            return false;
    }
    if (!artiRowMap || !entriesPtr)
        return false;

    std::vector<uint32_t> localToRow(jc.artis.size(), 0xffffffffu);
    for (size_t i = 0; i < jc.artis.size(); ++i)
    {
        const std::unordered_map<const PxArticulationReducedCoordinate*, PxU32>::const_iterator it =
            artiRowMap->find(jc.artis[i]);
        if (it == artiRowMap->end())
            return false;
        localToRow[i] = it->second;
    }
    if (!ovx::ensureJointRecords(jc, source, jc.joints, *entriesPtr, localToRow, generation, true))
        return false;
    if (jc.slices.empty())
        return true;

    const uint32_t numOut = static_cast<uint32_t>(jc.recs.size());
    const uint32_t comp = omni::physx::tensors::dofPropertyComponents(row.prop);
    const bool isByte = omni::physx::tensors::dofPropertyIsByte(row.prop);

    std::unique_ptr<GroupSlot> slot(new GroupSlot());
    slot->scene = scene;
    slot->jointPropRow = &row;
    slot->numOut = numOut;
    slot->comp = comp;
    const size_t bytes = size_t(numOut) * comp * (isByte ? 1u : 4u);
    slot->floats.assign(bytes / 4 + (bytes % 4 ? 1 : 0), 0.0f);

    ovx_path_dictionary_t* dict = nullptr;
    const bool haveHandles = jc.sliceHandles.size() == jc.slices.size();
    slot->list = haveHandles ? omni::physics::ovstage::buildPathListFromHandles(source, jc.sliceHandles.data(),
                                                                               jc.sliceHandles.size(), &dict) :
                               omni::physics::ovstage::buildPathList(source, jc.sliceKeys.data(),
                                                                     jc.sliceKeys.size(), &dict);
    if (slot->list == OVX_INVALID_PRIMPATH_LIST || !dict)
        return false;
    s.dict = dict;

    // One tensor per joint, each a window onto its own slice of the flat column -- the DOF-state
    // shape, because a property is per (joint, axis) just as a position is.
    slot->shapes.resize(jc.slices.size());
    slot->tensors.assign(jc.slices.size(), DLTensor{});
    char* const base = reinterpret_cast<char*>(slot->floats.data());
    for (size_t si = 0; si < jc.slices.size(); ++si)
    {
        const ovx::JointSlice& sl = jc.slices[si];
        slot->shapes[si] = static_cast<int64_t>(sl.count);
        DLTensor& t = slot->tensors[si];
        t.data = base + size_t(sl.offset) * comp * (isByte ? 1u : 4u);
        t.device = DLDevice{ kDLCPU, 0 };
        t.ndim = 1;
        t.dtype = isByte ? DLDataType{ kDLUInt, 8, static_cast<uint16_t>(comp) } :
                           DLDataType{ kDLFloat, 32, static_cast<uint16_t>(comp) };
        t.shape = &slot->shapes[si];
        t.strides = nullptr;
        t.byte_offset = 0;
    }

    ovstage_map_group_t& g = slot->group;
    g.prims.list = slot->list;
    g.prims.offset = 0;
    g.prims.count = static_cast<uint32_t>(jc.slices.size());
    g.prims.index_map = nullptr;
    g.data.tensors = slot->tensors.data();
    g.data.tensor_count = static_cast<uint32_t>(slot->tensors.size());
    g.data.count = 0;
    g.data.index_map = nullptr;

    s.groups.push_back(std::move(slot));
    return true;
}

// Plan the groups this session hands out: resolve the query to its matched bodies and split them by
// owning scene, the way the read splits its columns. Returns false if the attribute is not writable
// for the queried type, or if a group could not be built.
bool planGroups(WriteSession& s)
{
    uint32_t type = 0;
    uint32_t scope = 0;
    if (!ovx::queryTypeScope(s.query, type, scope))
    {
        CARB_LOG_ERROR("ovxWriteAttribute: invalid query handle %llu.", (unsigned long long)s.query);
        return false;
    }
    // Articulation LINKS accept the attributes PhysX can set on a link, and refuse the rest BY NAME.
    //
    // This was a type-level refusal, on the reasoning that PhysX exposes no link-pose write. True,
    // but far too coarse: a PxArticulationLink IS a PxRigidBody, so mass, inertia, the centre-of-mass
    // pair, disableGravity and the whole per-shape set are the same setters that serve a rigid body,
    // and the write API implements all of them on the base view. Refusing the type blocked a dozen
    // writable attributes to protect four unwritable ones.
    //
    // So the refusal moved to the row (`linkWritable`), and the error names what is wrong rather than
    // declaring the type shut. Move an articulation's pose through its root and joint DOFs.
    if (type == kOvxArticulationLink)
    {
        const WriteAttributeRow* lrow = findRigidWriteAttribute(s.attribute);
        if (!lrow)
        {
            CARB_LOG_ERROR("ovxWriteAttribute: articulation links do not accept attribute '%s'.",
                           s.attribute.c_str());
            return false;
        }
        if (!lrow->linkWritable)
        {
            CARB_LOG_ERROR("ovxWriteAttribute: '%s' is not writable on an articulation link. PhysX has "
                           "no link write for it -- move the articulation through its root pose and "
                           "jointPosition instead. Link mass, inertia, centreOfMass*, disableGravity "
                           "and the per-shape properties ARE writable.",
                           s.attribute.c_str());
            return false;
        }

        ovx::ActiveContext lctx;
        if (!ovx::getActiveContext("ovxWriteAttribute", lctx))
            return false;
        // The read's link enumeration, shared: a write must cover exactly the links a read covers --
        // including the ACTIVE set, which the scan now takes rather than rebuilding, so the two
        // directions cannot disagree about which actors moved.
        ovx::RigidRecordScan linkScan;
        const ovx::ActiveActorSet linkActive =
            (scope == kOvxActive) ? ovx::collectActiveActors() : ovx::ActiveActorSet{};
        ovx::scanLinkRecords(scope, linkActive, linkScan);
        for (PxScene* scene : ovx::allPhysicsScenes())
        {
            const ovx::RigidSceneBucket* bucket = ovx::bucketForScene(linkScan, scene);
            if (!bucket)
                continue;
            // kOvxAll only for the shared cache: its key carries a type, and a link session must not
            // collide with the rigid one over the same scene.
            if (!planRigidGroup(s, *lctx.source, scene, bucket->bodies, bucket->keys, *lrow, kOvxActive))
                return false;
        }
        return true;
    }

    if (type == kOvxArticulation)
    {
        const ArtiWriteAttributeRow* arow = findArtiWriteAttribute(s.attribute);
        if (!arow)
        {
            CARB_LOG_ERROR("ovxWriteAttribute: articulations do not accept attribute '%s'. Root state is "
                           "'rootPosition', 'rootOrientation', 'rootLinearVelocity' and 'rootAngularVelocity'; "
                           "joint state goes through kOvxArticulationJoint and link properties through "
                           "kOvxArticulationLink.",
                           s.attribute.c_str());
            return false;
        }
        ovx::ActiveContext actx;
        if (!ovx::getActiveContext("ovxWriteAttribute", actx))
            return false;
        // The read's structural walk, not a second one: it enumerates the same ePTArticulation
        // records AND detects duplicate or inconsistent ones, which an independent scan here did
        // not. Sharing it is what keeps a root write addressing the prims a root read reports.
        const std::vector<PxScene*> artiScenes = ovx::allPhysicsScenes();
        if (!ovx::refreshArticulationCache(artiScenes))
        {
            CARB_LOG_ERROR("ovxWriteAttribute: the articulation structural walk failed; nothing was written.");
            return false;
        }
        for (PxScene* scene : artiScenes)
        {
            ovx::ArticulationReadCacheEntry* entry = ovx::articulationCacheEntry(scene);
            if (!entry || entry->rootArtis.empty())
                continue;
            if (entry->duplicateRootPointers)
            {
                // The read refuses to emit under this, so the write must too: a duplicated
                // articulation pointer means the root list does not identify prims one-to-one, and a
                // scatter through it would write one articulation's values into another's row.
                CARB_LOG_ERROR("ovxWriteAttribute: duplicate or inconsistent ePTArticulation records "
                               "for this scene; nothing was written.");
                return false;
            }
            // kOvxActive narrows to what the solver moved. Deliberately NOT the active-actor set:
            // getActiveActors reports ACTORS, so an articulation's links appear in it and the
            // articulation never does -- testing membership there would narrow this to nothing.
            std::vector<PxArticulationReducedCoordinate*> artis;
            std::vector<ObjectKey> keys;
            artis.reserve(entry->rootArtis.size());
            keys.reserve(entry->rootArtis.size());
            for (size_t i = 0; i < entry->rootArtis.size(); ++i)
            {
                PxArticulationReducedCoordinate* arti = entry->rootArtis[i];
                if (!arti || arti->getScene() != scene)
                    continue;
                if (scope == kOvxActive && arti->isSleeping())
                    continue;
                artis.push_back(arti);
                keys.push_back(entry->rootKeys[i]);
            }
            if (!planArticulationGroup(s, *actx.source, scene, artis, keys, *arow))
                return false;
        }
        return true;
    }

    if (type == kOvxFixedTendon || type == kOvxSpatialTendon)
    {
        const bool fixed = (type == kOvxFixedTendon);
        const TendonWriteAttributeRow* trow = findTendonWriteAttribute(s.attribute);
        if (!trow)
        {
            CARB_LOG_ERROR("ovxWriteAttribute: %s tendons do not accept attribute '%s'.",
                           fixed ? "fixed" : "spatial", s.attribute.c_str());
            return false;
        }
        if (!fixed && trow->fixedOnly)
        {
            // Refused by NAME, and not because PhysX lacks the setter: the schema puts a spatial
            // tendon's limit and rest length on its LEAF ATTACHMENT, so there is no such property on
            // the tendon to write. The read refuses the same pair for the same reason.
            CARB_LOG_ERROR("ovxWriteAttribute: '%s' does not exist on a spatial tendon -- the schema "
                           "places it on the leaf attachment. It is writable on kOvxFixedTendon.",
                           s.attribute.c_str());
            return false;
        }
        ovx::ActiveContext tctx;
        if (!ovx::getActiveContext("ovxWriteAttribute", tctx))
            return false;
        for (PxScene* scene : ovx::allPhysicsScenes())
            if (!planTendonGroup(s, *tctx.source, scene, fixed, *trow))
                return false;
        return true;
    }

    if (type == kOvxArticulationJoint)
    {
        // Properties first: the two tables are disjoint, and a property is the larger set.
        if (const JointPropertyWriteRow* prow = findJointPropertyWriteAttribute(s.attribute))
        {
            ovx::ActiveContext pctx;
            if (!ovx::getActiveContext("ovxWriteAttribute", pctx))
                return false;
            for (PxScene* scene : ovx::allPhysicsScenes())
                if (!planJointPropertyGroup(s, *pctx.source, scene, *prow))
                    return false;
            return true;
        }
        const JointWriteAttributeRow* jrow = findJointWriteAttribute(s.attribute);
        if (!jrow)
        {
            CARB_LOG_ERROR("ovxWriteAttribute: articulation joints do not accept attribute '%s'.",
                           s.attribute.c_str());
            return false;
        }
        ovx::ActiveContext jctx;
        if (!ovx::getActiveContext("ovxWriteAttribute", jctx))
            return false;
        for (PxScene* scene : ovx::allPhysicsScenes())
            if (!planJointGroup(s, *jctx.source, scene, *jrow))
                return false;
        return true;
    }

    // Vehicle wheels are refused BY NAME with the reason, not through the catch-all below (ADR-0012).
    //
    // A wheel's world transform is DERIVED -- composed each step from the chassis body plus
    // suspension compression and steer angle -- so a written pose is overwritten by the next step.
    // Exactly the argument that makes articulation link poses unwritable, and BaseVehicleView
    // reflects it: the view exposes getWheelTransformsOvStage and no setter of any kind.
    //
    // This is a DECLARED EXCLUSION under REQ-INPUT-COVERAGE-001 AC-4, not an unimplemented type. The
    // distinction is the whole point of naming it here: "not writable yet" invites someone to
    // implement it, and there is nothing to implement.
    // Vehicle wheels refuse per ATTRIBUTE, the same shape kOvxArticulationLink settled into and for
    // the same reason: some of what this type exposes is an OUTPUT the solver produces and some is an
    // INPUT it consumes.
    //
    //   position / orientation   REFUSED, permanently. A wheel's transform is composed each step from
    //                            the chassis, the suspension and the steer angle, so a written value
    //                            is overwritten by the next step. BaseVehicleView reflects this: it
    //                            exposes a getter and, until W10, no setter at all. Declared under
    //                            REQ-INPUT-COVERAGE-001 AC-4 -- an exclusion, not a gap.
    //   driveTorque / brakeTorque / steerAngle   ACCEPTED. Inputs the next step reads.
    if (type == kOvxVehicleWheel)
    {
        const VehicleWriteAttributeRow* vrow = findVehicleWriteAttribute(s.attribute);
        if (!vrow)
        {
            CARB_LOG_ERROR("ovxWriteAttribute: '%s' is not writable on a vehicle wheel, and will not be. A "
                           "wheel's pose is derived from the chassis, its suspension and its steer angle "
                           "every step, so a written value is overwritten by the next one. The wheel's "
                           "CONTROLS are writable: 'driveTorque', 'brakeTorque' and 'steerAngle'.",
                           s.attribute.c_str());
            return false;
        }
        ovx::ActiveContext vctx;
        if (!ovx::getActiveContext("ovxWriteAttribute", vctx))
            return false;
        for (PxScene* scene : ovx::allPhysicsScenes())
            if (!planVehicleGroup(s, *vctx.source, scene, *vrow))
                return false;
        return true;
    }

    // Deformable simulation-mesh state (ADR-0012). Both kinds serve the same two columns the read
    // serves; they differ only in which PhysX type holds the buffers.
    if (type == kOvxDeformableVolume || type == kOvxDeformableSurface)
    {
        const bool velocity = (s.attribute == OvxAttr::kVelocities);
        if (!velocity && s.attribute != OvxAttr::kPoints)
        {
            // Two names are refused for DIFFERENT reasons, and both are declared exclusions under
            // REQ-INPUT-COVERAGE-001 AC-4 rather than gaps:
            //
            //   restPoints       authored geometry the solver never rewrites -- read-only.
            //   kinematicTarget  PhysX takes this as setKinematicTargetBufferD(const PxVec4*), which
            //                    ADOPTS a buffer the caller must keep alive for as long as the body
            //                    uses it. A write session's column is freed at release, so handing
            //                    one over would leave PhysX reading freed device memory. Serving it
            //                    needs an owned per-body buffer with a lifetime this API does not
            //                    have -- a design question, not an unwritten scatter.
            CARB_LOG_ERROR("ovxWriteAttribute: deformable bodies do not accept attribute '%s'. They accept "
                           "'points' and 'velocities'. 'restPoints' is authored geometry the solver never "
                           "rewrites. 'kinematicTarget' is set by handing PhysX a buffer it keeps, which a "
                           "write session cannot supply because its column is freed at release.",
                           s.attribute.c_str());
            return false;
        }
        const bool isVolume = (type == kOvxDeformableVolume);
        ovx::ActiveContext dctx;
        if (!ovx::getActiveContext("ovxWriteAttribute", dctx))
            return false;
        for (PxScene* scene : ovx::allPhysicsScenes())
        {
            ovx::DeformableWriteTargets targets;
            if (!ovx::deformableWriteTargets(scene, isVolume, targets))
                return false;
            if (!planDeformableGroup(s, *dctx.source, scene, isVolume, targets, velocity))
                return false;
        }
        return true;
    }

    // Deformable materials (ADR-0012). One group for the whole session: a material belongs to no
    // scene, so there is nothing to iterate scenes for.
    if (type == kOvxDeformableMaterial)
    {
        const DeformableMaterialWriteRow* mrow = findDeformableMaterialWriteAttribute(s.attribute);
        if (!mrow)
        {
            CARB_LOG_ERROR("ovxWriteAttribute: deformable materials do not accept attribute '%s'.",
                           s.attribute.c_str());
            return false;
        }
        ovx::ActiveContext mctx;
        if (!ovx::getActiveContext("ovxWriteAttribute", mctx))
            return false;
        return planDeformableMaterialGroup(s, *mctx.source, *mrow);
    }

    // Particle sets serve the two columns the read serves, and nothing else (ADR-0012). There is no
    // per-particle property set to add later: everything else a particle has -- phase, mass, the
    // material constants -- belongs to the set or its system, not to a particle.
    if (type == kOvxParticleSet)
    {
        const bool velocity = (s.attribute == OvxAttr::kVelocities);
        if (!velocity && s.attribute != OvxAttr::kPoints)
        {
            CARB_LOG_ERROR("ovxWriteAttribute: particle sets do not accept attribute '%s'. They accept "
                           "'points' and 'velocities'.",
                           s.attribute.c_str());
            return false;
        }
        ovx::ActiveContext pctx;
        if (!ovx::getActiveContext("ovxWriteAttribute", pctx))
            return false;
        for (PxScene* scene : ovx::allPhysicsScenes())
        {
            ovx::ParticleWriteTargets targets;
            if (!ovx::particleWriteTargets(scene, scope, targets))
                return false;
            if (!planParticleGroup(s, *pctx.source, scene, targets, velocity))
                return false;
        }
        return true;
    }

    if (type != kOvxRigidBody)
    {
        CARB_LOG_ERROR("ovxWriteAttribute: object type %u is not writable yet.", type);
        return false;
    }

    const WriteAttributeRow* row = findRigidWriteAttribute(s.attribute);
    if (!row)
    {
        CARB_LOG_ERROR("ovxWriteAttribute: rigid bodies do not accept attribute '%s'.", s.attribute.c_str());
        return false;
    }

    ovx::ActiveContext ctx;
    if (!ovx::getActiveContext("ovxWriteAttribute", ctx))
        return false;

    // Deferred: only walked if a scene actually needs it, and skipped entirely per scene when the
    // shared cache already describes it.
    ovx::LazyRigidScan lazy{ scope };
    const std::vector<PxScene*> scenes = ovx::allPhysicsScenes();
    ovx::purgeDeadRigidReadCacheEntries(scenes);
    for (PxScene* scene : scenes)
    {
        // The body set is derived from the object database and from nothing else, so it is unchanged
        // exactly while no object has been created or retired -- which the epoch answers on its own,
        // with no view to acquire. The read takes the same skip for the same reason; taking it here
        // removes most of a planned session's cost.
        const ovx::RigidReadCacheKey cacheKey{
            scene, static_cast<int>(omni::physx::tensors::RigidBodyType::eRigidDynamic), scope
        };
        if (scope == kOvxAll)
        {
            const std::unordered_map<ovx::RigidReadCacheKey, ovx::RigidReadCacheEntry,
                                     ovx::RigidReadCacheKeyHash>::iterator hit =
                ovx::g_rigidReadCache.find(cacheKey);
            if (hit != ovx::g_rigidReadCache.end() &&
                hit->second.dbEpoch == omni::physx::internal::recordLifetimeEpoch() &&
                !hit->second.bodies.empty() && hit->second.bodies.size() == hit->second.keys.size())
            {
                if (!planRigidGroup(s, *ctx.source, scene, hit->second.bodies, hit->second.keys, *row, scope))
                    return false;
                continue;
            }
        }

        const ovx::RigidSceneBucket* bucket = ovx::bucketForScene(lazy.get(), scene);
        if (!bucket)
            continue; // this scene owns no standalone bodies
        // Stored before the group is built, exactly as the read stores it before the emit: the emit
        // is what stamps the generation these belong to, alongside the rows and handles.
        if (scope == kOvxAll)
            ovx::g_rigidReadCache[cacheKey].bodies = bucket->bodies;
        if (!planRigidGroup(s, *ctx.source, scene, bucket->bodies, bucket->keys, *row, scope))
            return false;
    }

    // Point-instancer instances, published as ARRAY groups alongside the standalone fixed ones -- the
    // same session, because the read serves both under one attribute name and a caller reading a
    // column and writing it back must see the same shape in both directions.
    //
    // Attempted for every rigid attribute that HAS an instancer column; one that does not (mass, the
    // per-shape set) simply plans no instancer groups, which is not a failure -- an instancer's
    // instances have no such attribute either.
    omni::physx::tensors::BasePointInstancerView::InstancerColumn column{};
    uint32_t instComp = 0;
    if (instancerColumnForWrite(s.attribute, column, instComp))
    {
        for (PxScene* scene : scenes)
        {
            // Both devices now. The DirectGPU path goes through the read's GPU instancer view, the
            // host path through the CPU one, and instancerWriteTargets serves whichever the scene
            // has -- so a scene with no view of either kind yields no instancer groups rather than
            // failing the session, which is what the standalone half depends on.
            ovx::InstancerWriteTargets targets;
            if (!ovx::instancerWriteTargets(scene, scope, targets))
                return false;
            if (!targets.view)
                continue; // this scene has no instancers -- not a failure
            for (uint32_t i = 0; i < targets.keys.size(); ++i)
            {
                if (!planInstancerGroup(s, *ctx.source, scene, targets.view, targets.keys[i],
                                        targets.arrayLengths[i], i, column, instComp))
                    return false;
            }
        }
    }
    return true;
}

// Publish one filled group to the engine: re-resolve its bodies to superset rows, then hand the
// column to the view method its attribute names.
//
// Rows are resolved HERE, not at plan time. A row indexes the superset view's ordering, and the
// backend takes a fresh generation whenever that view is rebuilt -- so a row cached across the
// caller's fill would address the view as it was when the group was handed out, and scatter this
// group's values into whatever occupies those rows now. One retry, with the row mapping inside it:
// a null view means the cache was found stale and dropped, so a second attempt rebuilds.
// Honour the caller's producer handoff BEFORE anything reads the column. Both knobs, because either
// can carry the dependency (ovstage_cuda_sync_t: stream drains, wait_event waits). Applied on the
// host path too: a caller may have produced a HOST column with a CUDA kernel and handed the event
// over, and ignoring it there would read the buffer mid-write.
void honourCallerSync(const GroupSlot& slot, ovstage_cuda_sync_t sync)
{
    if ((!sync.stream && !sync.wait_event) || !getCudaShim())
        return;
    if (slot.ctxMgr)
    {
        PxScopedCudaLock _lock(*slot.ctxMgr);
        if (sync.wait_event)
            getCudaShim()->streamWaitEvent(uintptr_t(0), sync.wait_event, 0, nullptr);
        if (sync.stream)
            getCudaShim()->streamSynchronize(sync.stream == 1 ? uintptr_t(0) : sync.stream, nullptr);
    }
    else if (sync.wait_event)
    {
        // No context of ours to push: wait on the event outright rather than stream-order it.
        getCudaShim()->eventSynchronize(sync.wait_event, nullptr);
    }
}

// Publish a joint group: hand the flat DOF column and the record list to the articulation view.
// No row re-resolution here -- a DOF record names (view row, dof index), and the records were derived
// under the generation the view still reports; the acquire below is what re-checks that.
bool scatterJointGroup(GroupSlot& slot, ovstage_cuda_sync_t writeDoneSync)
{
    if (slot.numOut == 0)
        return true;
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
        return false;
    const bool gpu = slot.deviceOrdinal >= 0;

    honourCallerSync(slot, writeDoneSync);

    omni::physics::tensors::TensorDesc td;
    td.device = slot.deviceOrdinal;
    td.dtype = omni::physics::tensors::TensorDataType::eFloat32;
    td.numDims = 1;
    td.dims[0] = static_cast<int>(slot.numOut);
    td.data = gpu ? reinterpret_cast<void*>(slot.deviceData) : static_cast<void*>(slot.floats.data());

    const std::unordered_map<const PxArticulationReducedCoordinate*, PxU32>* artiRowMap = nullptr;
    const std::vector<omni::physx::tensors::ArticulationEntry>* entries = nullptr;
    if (gpu)
    {
        omni::physx::tensors::GpuSimulationView* sv = backend->acquireSceneView(slot.scene);
        omni::physx::tensors::GpuArticulationView* av =
            sv ? sv->supersetArticulationView(&artiRowMap, &entries) : nullptr;
        if (!av)
        {
            CARB_LOG_ERROR("ovxCommitGroup: superset articulation view unavailable; nothing was written.");
            return false;
        }
        t_scatterCommitAttempted = true; // past every pre-commit check; this setter is the commit point
        return (av->*slot.jointRow->gpuWrite)(
            &td, reinterpret_cast<const omni::physx::tensors::ArticulationDofOvStageRecord*>(slot.recsDev),
            slot.numOut);
    }
    omni::physx::tensors::CpuSimulationView* sv = backend->acquireCpuSceneView(slot.scene);
    omni::physx::tensors::CpuArticulationView* av =
        sv ? sv->supersetArticulationView(&artiRowMap, &entries) : nullptr;
    if (!av)
    {
        CARB_LOG_ERROR("ovxCommitGroup: superset articulation view unavailable; nothing was written.");
        return false;
    }
    // Host records live in the shared cache; the host path has no device upload to read them from.
    const ovx::ArticulationReadCacheEntry* jcPtr = ovx::articulationCacheEntry(slot.scene);
    if (!jcPtr)
    {
        CARB_LOG_ERROR("ovxCommitGroup: the articulation cache no longer describes this scene; "
                       "nothing was written.");
        return false;
    }
    const ovx::ArticulationReadCacheEntry& jc = *jcPtr;
    if (jc.recs.size() != slot.numOut)
    {
        CARB_LOG_ERROR("ovxCommitGroup: the joint record list moved since this group was planned; "
                       "nothing was written.");
        return false;
    }
    t_scatterCommitAttempted = true; // past every pre-commit check; this setter is the commit point
    return (av->*slot.jointRow->cpuWrite)(&td, jc.recs.data(), slot.numOut);
}

// Publish an articulation-root group: re-resolve its articulations to superset rows, then hand the
// column and the rows to the articulation view.
//
// Rows are resolved HERE, not at plan time, for the reason the rigid path resolves them here: a row
// indexes the superset view's ordering, and the backend takes a fresh generation whenever it rebuilds
// that view, so a row cached across the caller's fill would address the view as it was when the group
// was handed out and scatter this group's values into whatever occupies those rows now.
bool scatterArticulationGroup(GroupSlot& slot, ovstage_cuda_sync_t writeDoneSync)
{
    if (slot.artis.empty())
        return true; // nothing to publish -- not a failure
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
    {
        CARB_LOG_ERROR("ovxCommitGroup: no tensor backend.");
        return false;
    }
    const bool gpu = slot.deviceOrdinal >= 0;

    honourCallerSync(slot, writeDoneSync);

    const std::unordered_map<const PxArticulationReducedCoordinate*, PxU32>* artiRowMap = nullptr;
    const std::vector<omni::physx::tensors::ArticulationEntry>* entries = nullptr;
    omni::physx::tensors::GpuArticulationView* gpuView = nullptr;
    omni::physx::tensors::CpuArticulationView* cpuView = nullptr;
    if (gpu)
    {
        omni::physx::tensors::GpuSimulationView* sv = backend->acquireSceneView(slot.scene);
        gpuView = sv ? sv->supersetArticulationView(&artiRowMap, &entries) : nullptr;
    }
    else
    {
        omni::physx::tensors::CpuSimulationView* sv = backend->acquireCpuSceneView(slot.scene);
        cpuView = sv ? sv->supersetArticulationView(&artiRowMap, &entries) : nullptr;
    }
    if ((!gpuView && !cpuView) || !artiRowMap)
    {
        CARB_LOG_ERROR("ovxCommitGroup: superset articulation view unavailable; nothing was written.");
        return false;
    }

    std::vector<PxU32> rows;
    rows.reserve(slot.artis.size());
    for (PxArticulationReducedCoordinate* arti : slot.artis)
    {
        const std::unordered_map<const PxArticulationReducedCoordinate*, PxU32>::const_iterator it =
            artiRowMap->find(arti);
        if (it == artiRowMap->end())
        {
            CARB_LOG_ERROR("ovxCommitGroup: the superset articulation view does not describe this "
                           "articulation set; nothing was written.");
            return false;
        }
        rows.push_back(it->second);
    }

    // Minted fresh on every commit rather than held. The token has to change exactly when the row
    // CONTENTS do, and there is no articulation read cache to hang a held version off yet -- so this
    // errs towards re-uploading a list that has not moved, which costs one small memcpy, rather than
    // reusing a token for a list that has. The read's TASK-04 is what would add the cache to share.
    slot.rowsVersion = ovx::nextRowsVersion();

    omni::physics::tensors::TensorDesc td;
    td.device = slot.deviceOrdinal;
    td.dtype = omni::physics::tensors::TensorDataType::eFloat32;
    td.numDims = 1;
    td.dims[0] = static_cast<int>(rows.size() * slot.comp);
    td.data = gpu ? reinterpret_cast<void*>(slot.deviceData) : static_cast<void*>(slot.floats.data());

    const PxU32 numOut = static_cast<PxU32>(rows.size());
    if (gpu)
        return (gpuView->*slot.artiRow->gpuWrite)(&td, rows.data(), numOut, slot.rowsVersion);
    return (cpuView->*slot.artiRow->cpuWrite)(&td, rows.data(), numOut, slot.rowsVersion);
}

// Publish a tendon group: hand the column and the record list to the articulation view. No row
// re-resolution here -- a tendon record names (view row, tendon slot), derived under the generation
// the plan captured, and the acquire below is what re-checks it.
bool scatterTendonGroup(GroupSlot& slot, ovstage_cuda_sync_t writeDoneSync)
{
    if (slot.numOut == 0)
        return true;
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
        return false;
    const bool gpu = slot.deviceOrdinal >= 0;

    honourCallerSync(slot, writeDoneSync);

    omni::physics::tensors::TensorDesc td;
    td.device = slot.deviceOrdinal;
    td.dtype = omni::physics::tensors::TensorDataType::eFloat32;
    td.numDims = 1;
    td.dims[0] = static_cast<int>(slot.numOut * slot.comp);
    td.data = gpu ? reinterpret_cast<void*>(slot.deviceData) : static_cast<void*>(slot.floats.data());

    const std::unordered_map<const PxArticulationReducedCoordinate*, PxU32>* artiRowMap = nullptr;
    const std::vector<omni::physx::tensors::ArticulationEntry>* entries = nullptr;
    if (gpu)
    {
        omni::physx::tensors::GpuSimulationView* sv = backend->acquireSceneView(slot.scene);
        omni::physx::tensors::GpuArticulationView* av =
            sv ? sv->supersetArticulationView(&artiRowMap, &entries) : nullptr;
        if (!av)
        {
            CARB_LOG_ERROR("ovxCommitGroup: superset articulation view unavailable; nothing was written.");
            return false;
        }
        const omni::physx::tensors::ArticulationTendonOvStageRecord* recs =
            reinterpret_cast<const omni::physx::tensors::ArticulationTendonOvStageRecord*>(slot.recsDev);
        return slot.tendonFixed ?
                   av->setFixedTendonPropertiesOvStage(&td, recs, slot.numOut, slot.tendonRow->prop) :
                   av->setSpatialTendonPropertiesOvStage(&td, recs, slot.numOut, slot.tendonRow->prop);
    }
    omni::physx::tensors::CpuSimulationView* sv = backend->acquireCpuSceneView(slot.scene);
    omni::physx::tensors::CpuArticulationView* av =
        sv ? sv->supersetArticulationView(&artiRowMap, &entries) : nullptr;
    if (!av)
    {
        CARB_LOG_ERROR("ovxCommitGroup: superset articulation view unavailable; nothing was written.");
        return false;
    }
    // Host records live in the shared cache; the host path has no device upload to read them from.
    ovx::TendonReadCacheEntry& tc = ovx::tendonCacheEntry(slot.scene, slot.tendonFixed);
    if (tc.recs.size() != slot.numOut)
    {
        CARB_LOG_ERROR("ovxCommitGroup: the tendon record list moved since this group was planned; "
                       "nothing was written.");
        return false;
    }
    return slot.tendonFixed ?
               av->setFixedTendonPropertiesOvStage(&td, tc.recs.data(), slot.numOut, slot.tendonRow->prop) :
               av->setSpatialTendonPropertiesOvStage(&td, tc.recs.data(), slot.numOut, slot.tendonRow->prop);
}

// Publish a vehicle wheel-control group. Host-only by construction -- see planVehicleGroup.
bool scatterVehicleGroup(GroupSlot& slot, ovstage_cuda_sync_t writeDoneSync)
{
    if (slot.numOut == 0)
        return true;
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
        return false;

    honourCallerSync(slot, writeDoneSync);

    omni::physx::internal::InternalScene* internalScene = nullptr;
    for (const ovx::ScenePair& p : ovx::allPhysicsScenesWithInternal())
    {
        // ScenePair is a std::pair<PxScene*, InternalScene*>, not a named struct.
        if (p.first == slot.scene)
        {
            internalScene = p.second;
            break;
        }
    }
    omni::physx::tensors::CpuSimulationView* sv = backend->acquireCpuSceneView(slot.scene);
    omni::physx::tensors::BaseVehicleView* vv =
        (sv && internalScene) ? sv->vehicleView(*internalScene) : nullptr;
    if (!vv)
    {
        CARB_LOG_ERROR("ovxCommitGroup: vehicle view unavailable; nothing was written.");
        return false;
    }
    const ovx::VehicleReadCacheEntry* vc = ovx::vehicleCacheEntry(slot.scene);
    if (!vc || vc->recs.size() != slot.numOut)
    {
        CARB_LOG_ERROR("ovxCommitGroup: the vehicle wheel list moved since this group was planned; "
                       "nothing was written.");
        return false;
    }

    omni::physics::tensors::TensorDesc td;
    td.device = -1;
    td.dtype = omni::physics::tensors::TensorDataType::eFloat32;
    td.numDims = 1;
    td.dims[0] = static_cast<int>(slot.numOut);
    td.data = static_cast<void*>(slot.floats.data());

    t_scatterCommitAttempted = true; // past every pre-commit check; this setter is the commit point
    return vv->setWheelControlOvStage(*internalScene, vc->recs.data(), slot.numOut, slot.vehicleRow->control,
                                      &td);
}

// Publish one instancer's array. The view applies the hole contract -- a slot with no live body is
// skipped -- and inverts the reframe; see BasePointInstancerView::setInstancerColumnsOvStage.
bool scatterInstancerGroup(GroupSlot& slot, ovstage_cuda_sync_t writeDoneSync)
{
    if (slot.numOut == 0 || !slot.instancerView)
        return true;
    honourCallerSync(slot, writeDoneSync);

    const void* const src = slot.deviceData ? reinterpret_cast<const void*>(slot.deviceData) :
                                              static_cast<const void*>(slot.floats.data());
    return slot.instancerView->setInstancerColumnOvStage(slot.instancerColumn, slot.instancerIdx, src,
                                                         slot.numOut);
}

// Publish a joint-property group. Host-only by construction -- see planJointPropertyGroup.
bool scatterJointPropertyGroup(GroupSlot& slot, ovstage_cuda_sync_t writeDoneSync)
{
    if (slot.numOut == 0)
        return true;
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
        return false;
    honourCallerSync(slot, writeDoneSync);

    const ovx::ArticulationReadCacheEntry* jc = ovx::articulationCacheEntry(slot.scene);
    if (!jc || jc->recs.size() != slot.numOut)
    {
        CARB_LOG_ERROR("ovxCommitGroup: the joint record list moved since this group was planned; "
                       "nothing was written.");
        return false;
    }

    // BaseArticulationView, not the Gpu/Cpu derived one: a property has no device copy, so both
    // backends reach the same entry point -- which is also what stops them drifting on what an
    // attribute means.
    const bool gpu = slot.scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API);
    const std::unordered_map<const PxArticulationReducedCoordinate*, PxU32>* artiRowMap = nullptr;
    const std::vector<omni::physx::tensors::ArticulationEntry>* entries = nullptr;
    omni::physx::tensors::BaseArticulationView* av = nullptr;
    if (gpu)
    {
        omni::physx::tensors::GpuSimulationView* sv = backend->acquireSceneView(slot.scene);
        av = sv ? sv->supersetArticulationView(&artiRowMap, &entries) : nullptr;
    }
    else
    {
        omni::physx::tensors::CpuSimulationView* sv = backend->acquireCpuSceneView(slot.scene);
        av = sv ? sv->supersetArticulationView(&artiRowMap, &entries) : nullptr;
    }
    if (!av)
    {
        CARB_LOG_ERROR("ovxCommitGroup: superset articulation view unavailable; nothing was written.");
        return false;
    }

    omni::physics::tensors::TensorDesc td;
    td.device = -1;
    td.dtype = omni::physx::tensors::dofPropertyIsByte(slot.jointPropRow->prop) ?
                   omni::physics::tensors::TensorDataType::eUint8 :
                   omni::physics::tensors::TensorDataType::eFloat32;
    td.numDims = 1;
    td.dims[0] = static_cast<int>(slot.numOut * slot.comp);
    td.data = static_cast<void*>(slot.floats.data());

    return av->setDofPropertyOvStage(slot.jointPropRow->prop, jc->recs.data(), slot.numOut, &td);
}

bool scatterGroup(GroupSlot& slot, ovstage_cuda_sync_t writeDoneSync)
{
    if (slot.jointRow)
        return scatterJointGroup(slot, writeDoneSync);
    if (slot.jointPropRow)
        return scatterJointPropertyGroup(slot, writeDoneSync);
    if (slot.isDeformable)
        return scatterDeformableGroup(slot, writeDoneSync);
    if (slot.deformableMatRow)
        return scatterDeformableMaterialGroup(slot, writeDoneSync);
    if (slot.isParticle)
        return scatterParticleGroup(slot, writeDoneSync);
    if (slot.isInstancer)
        return scatterInstancerGroup(slot, writeDoneSync);
    if (slot.vehicleRow)
        return scatterVehicleGroup(slot, writeDoneSync);
    if (slot.tendonRow)
        return scatterTendonGroup(slot, writeDoneSync);
    if (slot.artiRow)
        return scatterArticulationGroup(slot, writeDoneSync);
    if (slot.bodies.empty())
        return true; // nothing to publish -- not a failure

    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
    {
        CARB_LOG_ERROR("ovxCommitGroup: no tensor backend.");
        return false;
    }
    const bool gpu = slot.deviceOrdinal >= 0; // the COLUMN's residency: what td below describes
    // Which BACKEND describes the SCENE, which is a different question and must not be answered
    // with the line above. A host-only property publishes a kDLCPU column on a DirectGPU scene,
    // where no CPU simulation view exists -- so deriving the view from the column's residency
    // asked for a view that is null there, and every mass / inertia / disableGravity /
    // disableSimulation write on such a scene failed at commit while its READ worked.
    const bool sceneIsGpu =
        slot.scene && slot.scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API);

    // Map this group's bodies onto their rows in the scene's superset view. Returns false when the
    // view does not describe this actor set, which the caller answers by dropping the cache and
    // trying once more.
    std::vector<PxU32> rows;
    auto resolveRows = [&](const std::unordered_map<const PxRigidBody*, PxU32>* rowMap)
    {
        rows.clear();
        rows.reserve(slot.bodies.size());
        for (PxRigidBody* b : slot.bodies)
        {
            const std::unordered_map<const PxRigidBody*, PxU32>::const_iterator it = rowMap->find(b);
            if (it == rowMap->end())
                return false;
            rows.push_back(it->second);
        }
        return true;
    };

    omni::physx::tensors::GpuRigidBodyView* gpuView = nullptr;
    omni::physx::tensors::CpuRigidBodyView* cpuView = nullptr;
    uint64_t rowsToken = 0;
    for (int attempt = 0; attempt < 2; ++attempt)
    {
        const std::unordered_map<const PxRigidBody*, PxU32>* rowMap = nullptr;
        uint64_t generation = 0;
        if (sceneIsGpu)
        {
            omni::physx::tensors::GpuSimulationView* sv = backend->acquireSceneView(slot.scene, &generation);
            gpuView = sv ? sv->supersetRigidView(&rowMap) : nullptr;
        }
        else
        {
            omni::physx::tensors::CpuSimulationView* sv = backend->acquireCpuSceneView(slot.scene, &generation);
            cpuView = sv ? sv->supersetRigidView(&rowMap) : nullptr;
        }
        const bool haveView = sceneIsGpu ? (gpuView != nullptr) : (cpuView != nullptr);
        if (haveView && rowMap)
        {
            // Rows taken under this generation still address this view: the backend takes a fresh
            // generation whenever the superset view is rebuilt, including when an actor it holds is
            // destroyed -- the removal a topology check alone cannot see. So a cached list is served
            // only under a generation obtained for THIS scene at this moment, never across one.
            ovx::RigidReadCacheEntry* ce =
                slot.cacheable ? &ovx::g_rigidReadCache[slot.cacheKey] : nullptr;
            if (ce && ce->generation == generation && ce->rowsVersion != 0 &&
                ce->rows.size() == slot.bodies.size() && ce->keys == slot.keys)
            {
                rows = ce->rows;
                rowsToken = ce->rowsVersion;
                break;
            }
            if (resolveRows(rowMap))
            {
                if (ce)
                {
                    // The canonical handles describe `keys`: a new key set (a new scene whose
                    // PxScene* reused this entry's address) cannot keep the old ones, or the next
                    // read publishes the previous scene's paths for these rows.
                    if (ce->keys != slot.keys)
                        ce->handles.clear();
                    ce->generation = generation;
                    ce->dbEpoch = omni::physx::internal::recordLifetimeEpoch();
                    ce->keys = slot.keys;
                    ce->bodies = slot.bodies;
                    ce->rows = rows;
                    // A NEW list, so any device copy of the old one is stale. Minted here and only
                    // here: the token has to change exactly when the content does, which is why it
                    // is not simply taken fresh on every write.
                    ce->rowsVersion = ovx::nextRowsVersion();
                    rowsToken = ce->rowsVersion;
                }
                else
                {
                    // kOvxActive -- and kOvxAll while a disabled body is filtered out -- rebuilds
                    // its rows every session by design, so there is nothing to compare against and
                    // no basis for reusing a device copy.
                    rowsToken = ovx::nextRowsVersion();
                }
                break;
            }
        }

        // Either the superset could not be built, or a body is missing from the map -- the cached
        // view predates it, or the scene's address was reused by a different scene. Drop the entry
        // and rebuild; a second failure is genuine.
        gpuView = nullptr;
        cpuView = nullptr;
        backend->invalidateSceneCache(slot.scene);
    }
    if (sceneIsGpu ? (gpuView == nullptr) : (cpuView == nullptr))
    {
        CARB_LOG_ERROR("ovxCommitGroup: superset rigid-body view unavailable; nothing was written.");
        return false;
    }

    if (sceneIsGpu)
    {
        // Probe the superset, not this group's filtered row list: a body disabled outside the
        // group still occupies a compacted DirectGPU slot.
        if (!gpuView->refreshDisabledRowsOvStage())
        {
            CARB_LOG_ERROR("ovxCommitGroup: DirectGPU row refresh failed; nothing was written.");
            return false;
        }
    }

    honourCallerSync(slot, writeDoneSync);

    // Past every pre-commit check; the setter below is the commit point (see t_scatterCommitAttempted).
    t_scatterCommitAttempted = true;

    omni::physics::tensors::TensorDesc td;
    td.device = slot.deviceOrdinal; // -1 on the host path, which is what the views check for
    td.dtype = slot.row->dtype;
    td.numDims = 2;
    td.dims[0] = static_cast<int>(slot.bodies.size());
    td.dims[1] = static_cast<int>(slot.comp);
    td.data = gpu ? reinterpret_cast<void*>(slot.deviceData) : static_cast<void*>(slot.floats.data());

    // Host-only attributes publish through the BASE view, which both backends share: there is no
    // device destination for them, so there is no per-backend method to choose between.
    if (slot.row->baseWrite)
    {
        omni::physx::tensors::BaseRigidBodyView* base =
            sceneIsGpu ? static_cast<omni::physx::tensors::BaseRigidBodyView*>(gpuView) :
                  static_cast<omni::physx::tensors::BaseRigidBodyView*>(cpuView);
        const bool written =
            (base->*slot.row->baseWrite)(&td, rows.data(), static_cast<uint32_t>(rows.size()), rowsToken);
        if (written && sceneIsGpu && std::string_view(slot.row->token) == OvxAttr::kDisableSimulation)
        {
            // The base setter changes a host actor flag and cannot notify the GPU view itself.
            // Advance the shared epoch so this superset and sibling views refresh the compacted
            // DirectGPU indices before their next operation.
            gpuView->markRdDisableDirty();
        }
        return written;
    }

    if (sceneIsGpu)
        return (gpuView->*slot.row->gpuWrite)(&td, rows.data(), static_cast<uint32_t>(rows.size()), rowsToken);
    return (cpuView->*slot.row->cpuWrite)(&td, rows.data(), static_cast<uint32_t>(rows.size()), rowsToken);
}

// Free everything a session's groups own: the device column each was allocated, and the prim list
// each built. A device pointer is only meaningful in its own context, so the free must go through the
// manager the allocation came from.
//
// Each group holds its OWN list -- groups are per scene and no two scenes cover the same prims -- so
// there is nothing to refcount here, unlike the read, where several attributes share one list.
void releaseGroupStorage(WriteSession& s)
{
    // Wait for any device write still in flight BEFORE freeing the columns it reads from. The RIGID
    // velocity paths hand the caller's column to PhysX directly, so a free here without this is a
    // use-after-free on device memory. The read blocks in the same place and for the same reason
    // (ovxReleaseRead): at release, never at the operation itself.
    //
    // Only rigid groups need it, and that is a property of WHAT PHYSX HOLDS rather than a shortcut.
    // A joint or articulation-root write scatters the caller's column into a PhysX-owned scratch and
    // hands PhysX only that scratch, so nothing PhysX may still be reading points at the column
    // being freed -- and cuMemFree synchronizes the device anyway, which covers our own kernel.
    // Stated rather than left to the loop's shape: relying on an unrelated view's wait to cover a
    // path is exactly how a use-after-free hides.
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (backend)
    {
        std::vector<PxScene*> waited;
        for (const std::unique_ptr<GroupSlot>& slot : s.groups)
        {
            if (!slot->committed || slot->deviceOrdinal < 0 || !slot->scene || !slot->row)
                continue;
            if (std::find(waited.begin(), waited.end(), slot->scene) != waited.end())
                continue;
            waited.push_back(slot->scene);
            omni::physx::tensors::GpuSimulationView* sv = backend->acquireSceneView(slot->scene);
            const std::unordered_map<const PxRigidBody*, PxU32>* rowMap = nullptr;
            if (omni::physx::tensors::GpuRigidBodyView* view = sv ? sv->supersetRigidView(&rowMap) : nullptr)
                view->waitOvStageWriteDone();
        }
    }

    for (const std::unique_ptr<GroupSlot>& slot : s.groups)
    {
        // A borrowed column is the caller's (the ovstage drain's zero-copy path) -- deviceData points
        // into it, so the session must not free it. recsDev, when present, is always session-owned.
        if (slot->deviceData && slot->ctxMgr && !slot->borrowed)
        {
            if (PxCudaContext* cu = slot->ctxMgr->getCudaContext())
            {
                PxScopedCudaLock _lock(*slot->ctxMgr);
                cu->memFree(slot->deviceData);
                if (slot->recsDev)
                    cu->memFree(slot->recsDev); // joint groups own their uploaded record list too
            }
            slot->deviceData = 0;
            slot->recsDev = 0;
        }
        if (slot->list != OVX_INVALID_PRIMPATH_LIST && s.dict)
        {
            ovx_path_dictionary_destroy_path_list(s.dict, slot->list);
            slot->list = OVX_INVALID_PRIMPATH_LIST;
        }
    }
}

} // namespace

extern "C"
{

OMNI_OVX_WRITE_API OvxWriteHandle ovxWriteAttribute(OvxOutputQueryHandle query, const ovx_string_or_token_t* attr)
{
    if (query == 0 || !attr)
        return 0;

    std::lock_guard<std::mutex> lock(g_mutex);

    // A session carries exactly one attribute, so a name is required rather than optional.
    // Interned-token-only attributes resolve once the dictionary lookup lands with the query
    // plumbing; a token with no string is not rejected here so that the eventual token path is not
    // accidentally forbidden by this stage.
    WriteSession s;
    s.query = query;
    if (attr->string.ptr && attr->string.length)
        s.attribute.assign(attr->string.ptr, attr->string.length);

    if (!planGroups(s))
    {
        releaseGroupStorage(s); // a later scene failed after earlier ones allocated
        return 0;
    }

    const OvxWriteHandle h = g_nextWriteHandle++;
    g_writes.emplace(h, std::move(s));
    return h;
}

OMNI_OVX_WRITE_API OvxWriteStatus ovxFetchWriteNext(OvxWriteHandle write, const ovstage_map_group_t** outGroup)
{
    if (!outGroup)
        return kOvxWriteStatusError;
    *outGroup = nullptr;

    std::lock_guard<std::mutex> lock(g_mutex);
    WriteSession* s = findSession(write);
    if (!s)
        return kOvxWriteStatusError; // a bad handle is a failure to iterate, not exhaustion

    if (s->nextFetch >= s->groups.size())
        return kOvxWriteStatusEndOfIteration;

    *outGroup = &s->groups[s->nextFetch++]->group;
    return kOvxWriteStatusOk;
}

OMNI_OVX_WRITE_API bool ovxCommitGroup(OvxWriteHandle write,
                                       const ovstage_map_group_t* group,
                                       ovstage_cuda_sync_t writeDoneSync,
                                       OvxCommitFailure* outFailure)
{
    const auto fail = [outFailure](OvxCommitFailure why)
    {
        if (outFailure)
            *outFailure = why;
        return false;
    };

    std::lock_guard<std::mutex> lock(g_mutex);
    WriteSession* s = findSession(write);
    if (!s)
        return fail(kOvxCommitFailureNotLive);

    GroupSlot* slot = findLiveGroup(*s, group);
    if (!slot)
        return fail(kOvxCommitFailureNotLive); // unknown, foreign, or already committed

    // Marked committed BEFORE the publish so a second commit of the same pointer is refused even if
    // the publish itself fails: commit is not retryable through this path.
    slot->committed = true;
    // Reported apart from the not-live cases above, which is the whole reason outFailure exists:
    // this group WAS accepted, the scatter ran, and a device scatter can fail after writing part of
    // its rows -- so no caller-facing layer may turn this into "nothing was published".
    if (!scatterGroup(*slot, writeDoneSync))
        return fail(kOvxCommitFailurePublish);
    return true;
}

OMNI_OVX_WRITE_API void ovxReleaseWrite(OvxWriteHandle write)
{
    // Uncommitted groups are DISCARDED, not published. Destroying the session is the whole of that:
    // nothing was handed to physics, so a caller that failed or threw mid-fill publishes nothing
    // from the group it was filling. Committed groups are not rolled back.
    //
    // Idempotent for an unknown or already-released handle, matching ovxReleaseRead.
    std::lock_guard<std::mutex> lock(g_mutex);
    const std::unordered_map<OvxWriteHandle, WriteSession>::iterator it = g_writes.find(write);
    if (it == g_writes.end())
        return;
    releaseGroupStorage(it->second);
    g_writes.erase(it);
}

} // extern "C"

// ================================================================================================
// ovstage drain value-apply. onSourceChange (PrimUpdate.cpp) routes a value ChangeBatch here; it is
// scattered through the SAME write backend a session commit uses (planGroup + scatterGroup), with the
// ovstage value column fed straight in -- replacing the per-object host-scalar apply for every
// attribute the backend covers. Reports NotHandled for anything it does not (transforms, apiSchemas,
// per-shape rows, structural changes) so onSourceChange keeps the parse-layer property path for those,
// and Failed (never NotHandled) once a scatter has committed part of the batch -- see DrainResult.
//
// Residency is data-driven, so accepting CUDA columns later is a no-op here: a column that already
// lives where the scene's scatter reads is borrowed in place (zero-copy); a mismatch -- today's
// host-only ovstage feeding a DirectGPU scene -- is staged across the bus into session storage.
namespace parse = ::omni::physics::parse;

// One scene's rigid group: plan it, land the column (borrow or stage), publish, release. `srcRows`
// selects the column rows feeding this group -- null = the whole column in batch order (the
// single-scene fast path, borrow-eligible), non-null = a gathered scattered subset (always copies).
// Component count carried by a feed ColumnView element type -- used to validate a drain column's width
// against the write row before copying, so a mis-authored / narrower column cannot be read out of bounds.
static uint32_t drainColumnComponents(parse::ColumnType t)
{
    switch (t)
    {
    case parse::ColumnType::eFloat2:
        return 2;
    case parse::ColumnType::eFloat3:
        return 3;
    case parse::ColumnType::eFloat4:
        return 4;
    case parse::ColumnType::eNone:
        return 0;
    default:
        return 1; // eFloat / eBool / eInt32 / eInt64 / eDouble / eToken / eObjectKey
    }
}

static bool drainScatterRigidGroup(parse::IPhysicsSource& source,
                                   PxScene* scene,
                                   const std::vector<PxRigidBody*>& bodies,
                                   const std::vector<ObjectKey>& keys,
                                   const WriteAttributeRow& row,
                                   const parse::ChangeBatch& batch,
                                   const std::vector<uint32_t>* srcRows,
                                   bool angular)
{
    const size_t n = bodies.size();
    if (n == 0)
        return true;
    const bool sceneGpu = !row.hostOnly && scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API);
    const bool colDevice = batch.values.device >= 0;
    // planRigidGroup DROPS disabled rigid dynamics on a DirectGPU scene (they have no device state row):
    // the returned slot is then a SUBSET of `bodies`, in slot order. When that happens we cannot borrow
    // the caller's column in place (it is wider and in the wrong order), and we cannot bulk-copy it either
    // -- both must go through the per-key gather below, which pulls each surviving body's row out of the
    // column by key. A batch that touches a disabled body is NOT failed or fallen back whole: its enabled
    // bodies still scatter, and the disabled ones are simply skipped (a velocity write to one is forbidden
    // by PhysX anyway). Statics / links never disable, so this is false for them.
    const bool willFilter = sceneGpu && ovx::anyDisabledRigidDynamic(bodies);

    // An angular column needs a deg->rad scale (below), which the zero-copy borrow cannot apply, so it
    // is always staged, never borrowed -- as is a filtered group (the borrow would alias a mismatched column).
    const bool borrow = colDevice && sceneGpu && !srcRows && !angular && !willFilter;

    // A device column we cannot borrow (a scattered subset, a filtered group, or a CPU scene) would need a
    // device-side gather / D2H stage this path does not do yet -- cannot arise while ovstage is host-only.
    if (colDevice && !borrow)
        return false;

    WriteSession s;
    if (!planRigidGroup(s, source, scene, bodies, keys, row, kOvxActive, /*allocate=*/!borrow))
        return false;
    if (s.groups.empty())
        return true; // every body in this scene was disabled -> nothing to scatter, correctly skipped
    GroupSlot& slot = *s.groups.back();

    if (borrow)
    {
        slot.deviceData = reinterpret_cast<CUdeviceptr>(const_cast<void*>(batch.values.data));
        slot.deviceOrdinal = batch.values.device;
        slot.borrowed = true;
    }
    else
    {
        // Host column -> session storage, then land the contiguous result (host->host on a CPU scene,
        // host->device on a DirectGPU one). Common case (nothing filtered): the slot is `bodies` in the same
        // order, so the column is copied positionally -- a single bulk copy (zero staging) for a whole
        // non-angular batch, a direct-indexed gather for a scattered subset (`srcRows`) or an angular batch
        // (which needs the deg->rad scale). ONLY when planRigidGroup dropped disabled bodies
        // (`nOut != keys.size()`) is the slot a reordered subset, and only then do we pay for a per-key gather.
        const size_t elemSize =
            (row.dtype == omni::physics::tensors::TensorDataType::eUint8) ? 1u : 4u;
        const size_t rowBytes = size_t(slot.comp) * elemSize;
        const size_t nOut = slot.bodies.size();
        const uint8_t* col = static_cast<const uint8_t*>(batch.values.data);
        std::vector<uint8_t> staging;
        const void* src = col;

        if (nOut != keys.size())
        {
            // Filtered: the slot is a subset of `bodies` in its own order, so look each surviving body up by
            // key. `srcRows` (if present) or the batch position gives its row in the column.
            std::unordered_map<ObjectKey, uint32_t, ObjectKey::Hash> keyToColRow;
            keyToColRow.reserve(keys.size());
            for (size_t i = 0; i < keys.size(); ++i)
                keyToColRow.emplace(keys[i], srcRows ? (*srcRows)[i] : static_cast<uint32_t>(i));
            staging.resize(nOut * rowBytes);
            for (size_t j = 0; j < nOut; ++j)
            {
                const std::unordered_map<ObjectKey, uint32_t, ObjectKey::Hash>::const_iterator it =
                    keyToColRow.find(slot.keys[j]);
                if (it == keyToColRow.end())
                    continue; // slot.keys is a subset of `keys`, so this cannot happen in practice
                std::memcpy(staging.data() + j * rowBytes, col + size_t(it->second) * rowBytes, rowBytes);
            }
            src = staging.data();
        }
        else if (srcRows || angular)
        {
            // Unfiltered scattered subset (or angular): slot order == group order, so gather by the group's
            // own column rows -- no key map needed.
            staging.resize(nOut * rowBytes);
            if (srcRows)
                for (size_t i = 0; i < nOut; ++i)
                    std::memcpy(staging.data() + i * rowBytes, col + size_t((*srcRows)[i]) * rowBytes, rowBytes);
            else
                std::memcpy(staging.data(), col, nOut * rowBytes);
            src = staging.data();
        }
        // else: unfiltered whole non-angular -> src stays `col`, no staging (the zero-copy fast path).

        if (angular)
        {
            // physics:angularVelocity is authored in DEGREES; the OvStage setter takes engine-native
            // radians (as write() does) and the change feed hands the raw column straight through, so
            // convert here -- exactly what the parse path does (degToRad) for the same attribute. angular
            // always lands in `staging` above, so this scales the session copy, never `col`. Rows are f32.
            float* f = reinterpret_cast<float*>(staging.data());
            const size_t floatCount = nOut * size_t(slot.comp);
            for (size_t i = 0; i < floatCount; ++i)
                f[i] *= 0.017453292519943295f; // pi / 180
        }
        if (slot.deviceData && slot.ctxMgr)
        {
            PxScopedCudaLock _lock(*slot.ctxMgr);
            slot.ctxMgr->getCudaContext()->memcpyHtoD(slot.deviceData, src, nOut * rowBytes);
        }
        else
        {
            std::memcpy(slot.floats.data(), src, nOut * rowBytes);
        }
    }

    slot.committed = true;
    const bool ok = scatterGroup(slot, ovstage_cuda_sync_t{});
    releaseGroupStorage(s);
    return ok;
}

// Vehicle wheel controls (driveTorque / brakeTorque / steerAngle). Vehicles are CPU-only and their
// write path scatters the WHOLE cached wheel set (planVehicleGroup + scatterVehicleGroup re-derive from
// the read cache and have no subset form), so the drain routes through it UNCHANGED only when the batch
// covers every wheel of a scene -- a per-frame control update does -- filling by cache position. A
// partial set (which would zero the omitted wheels) or a cold cache (no prior vehicle read) falls back.
// Returns true only when it both recognised a vehicle control and applied it.
static bool applyVehicleBatch(parse::IPhysicsSource& source, const parse::ChangeBatch& batch,
                              const std::string& usdAttr)
{
    const char* ovxName = nullptr;
    if (usdAttr == "physxVehicleWheelController:driveTorque")
        ovxName = OvxAttr::kDriveTorque;
    else if (usdAttr == "physxVehicleWheelController:brakeTorque")
        ovxName = OvxAttr::kBrakeTorque;
    else if (usdAttr == "physxVehicleWheelController:steerAngle")
        ovxName = OvxAttr::kSteerAngle;
    if (!ovxName)
        return false; // not a vehicle control -> caller tries the next object type
    const VehicleWriteAttributeRow* row = findVehicleWriteAttribute(ovxName);
    if (!row)
        return false;

    // Validate the column before the host memcpy below, like the sibling handlers do: a host f32 scalar,
    // one row per change. Require eFloat specifically, not merely a 1-component column: the scans below read
    // it as packed f32 (memcpy of sizeof(float) per row), so a narrower 1-component type (e.g. eBool at 1
    // byte) would overrun the feed buffer. A device / mis-shaped / mis-typed column falls back to the
    // parse-layer path rather than being memcpy'd as if it were packed host floats.
    if (batch.values.count != batch.numChanges || !batch.values.data ||
        batch.values.type != parse::ColumnType::eFloat || batch.values.device >= 0)
        return false;
    const bool brakeRow = (ovxName == OvxAttr::kBrakeTorque);

    // A negative brakeTorque is invalid, and the drain cannot reproduce the parse path's per-wheel handling:
    // updateVehicleWheelControllerBrakeTorque logs an error and KEEPS the wheel's prior torque (reject-and-
    // keep), whereas this whole-set scatter must write SOME value for every wheel. Rather than diverge -- and
    // have the applied value depend on cache state (a warm full-set drain clamping to 0 vs a cold/partial set
    // falling to the parse path that keeps the old torque) -- decline the whole batch so the parse path
    // applies its reject-and-keep uniformly. Checked before planVehicleGroup, so nothing is committed and the
    // batch falls back as NotHandled.
    if (brakeRow)
    {
        const uint8_t* vals = static_cast<const uint8_t*>(batch.values.data);
        for (size_t i = 0; i < batch.numChanges; ++i)
        {
            float v;
            std::memcpy(&v, vals + i * sizeof(float), sizeof(float));
            if (v < 0.0f)
                return false; // not drainable -> parse-path reject-and-keep, independent of cache state
        }
    }

    const ObjectKey* keys = static_cast<const ObjectKey*>(batch.keys.data);
    for (PxScene* scene : ovx::allPhysicsScenes())
    {
        const ovx::VehicleReadCacheEntry* vc = ovx::vehicleCacheEntry(scene);
        if (!vc || vc->keys.empty() || vc->keys.size() != batch.numChanges)
            continue; // no wheels here, or the batch is not this scene's full wheel set

        // Map each batch row to its wheel's position in the cache (= scatter) order.
        std::unordered_map<ObjectKey, uint32_t, ObjectKey::Hash> keyToPos;
        for (uint32_t i = 0; i < vc->keys.size(); ++i)
            keyToPos.emplace(vc->keys[i], i);
        std::vector<uint32_t> place(batch.numChanges);
        bool covers = true;
        for (size_t i = 0; i < batch.numChanges; ++i)
        {
            const std::unordered_map<ObjectKey, uint32_t, ObjectKey::Hash>::const_iterator it =
                keyToPos.find(keys[i]);
            if (it == keyToPos.end())
            {
                covers = false;
                break;
            }
            place[i] = it->second;
        }
        if (!covers)
            continue; // a batch key is not a wheel of this scene

        WriteSession s;
        if (!planVehicleGroup(s, source, scene, *row) || s.groups.empty())
            return false;
        GroupSlot& slot = *s.groups.back();
        if (slot.floats.size() != batch.numChanges)
        {
            releaseGroupStorage(s);
            return false;
        }
        // Place each batch value (batch order) at its wheel's cache position (host f32 scalars). A negative
        // brakeTorque already declined the whole batch above (reject-and-keep on the parse path), so every
        // value here is applied as authored.
        const uint8_t* col = static_cast<const uint8_t*>(batch.values.data);
        for (size_t i = 0; i < batch.numChanges; ++i)
        {
            float v;
            std::memcpy(&v, col + i * sizeof(float), sizeof(float));
            slot.floats[place[i]] = v;
        }
        slot.committed = true;
        const bool ok = scatterGroup(slot, ovstage_cuda_sync_t{});
        releaseGroupStorage(s);
        return ok;
    }
    return false; // no scene's full wheel set matched -> parse-layer fallback
}

// Articulation DOF drain. USD authors DOF as per-axis SCALARS on the joint prim -- `drive:<axis>:physics:
// targetPosition` / `:targetVelocity` and `state:<axis>:physics:position` / `:velocity`, one scalar per
// axis. The backend scatters the WHOLE scene's DOF column (planJointGroup zeros it), so this routes only
// when the batch covers every joint in the scene AND every joint is single-DOF -- then each joint's one
// DOF is filled from the batch and nothing is left zeroed. Multi-DOF joints (e.g. D6) fall back to the
// parse path: mapping a USD axis name onto a physxDofIdx within a multi-DOF slice needs the per-axis map
// only the parse layer has. Angular axes are authored in degrees; the deg->rad fold (and the joint sign) is
// applied by the setter scatterGroup runs, not here -- the raw column is scattered straight through, like
// the normal ovphysx_write joint path.
static bool applyArticulationBatch(parse::IPhysicsSource& source, const parse::ChangeBatch& batch,
                                   const std::string& usdAttr)
{
    // Match the EXACT registered drive/state property, only for the single-DOF prismatic (`linear`) and
    // revolute (`angular`) axes: their one DOF maps unambiguously to the joint, so discarding the axis below
    // is safe. Everything else falls back to the parse path: a D6 per-axis spelling (`rotX` / `transY` / ...)
    // needs the axis->physxDofIdx map only the parse layer has -- and a D6 with a single unlocked DOF still
    // has slice count 1, so the count check below cannot catch it -- while an arbitrary custom attribute the
    // wildcard feed surfaces (`state:banana:physics:position`) is not a control at all. Prefix/suffix matching
    // accepted both.
    const char* ovxName = nullptr;
    if (usdAttr == "drive:linear:physics:targetPosition" || usdAttr == "drive:angular:physics:targetPosition")
        ovxName = OvxAttr::kJointPositionTarget;
    else if (usdAttr == "drive:linear:physics:targetVelocity" || usdAttr == "drive:angular:physics:targetVelocity")
        ovxName = OvxAttr::kJointVelocityTarget;
    else if (usdAttr == "state:linear:physics:position" || usdAttr == "state:angular:physics:position")
        ovxName = OvxAttr::kJointPosition;
    else if (usdAttr == "state:linear:physics:velocity" || usdAttr == "state:angular:physics:velocity")
        ovxName = OvxAttr::kJointVelocity;
    if (!ovxName)
        return false;
    const JointWriteAttributeRow* row = findJointWriteAttribute(ovxName);
    if (!row)
        return false;
    // Per-axis DOF scalars arrive as a host F32 scalar column, one lane per joint prim. Require exactly
    // eFloat -- an int32 scalar also has one component but its bits would be read as a float below.
    if (batch.values.count != batch.numChanges || !batch.values.data ||
        batch.values.type != parse::ColumnType::eFloat || batch.values.device >= 0)
        return false;

    const ObjectKey* keys = static_cast<const ObjectKey*>(batch.keys.data);
    const float* col = static_cast<const float*>(batch.values.data);
    if (!ovx::refreshArticulationCache(ovx::allPhysicsScenes()))
        return false;
    for (PxScene* scene : ovx::allPhysicsScenes())
    {
        ovx::ArticulationReadCacheEntry* jcPtr = ovx::articulationCacheEntry(scene);
        if (!jcPtr || jcPtr->slices.empty())
            continue;
        ovx::ArticulationReadCacheEntry& jc = *jcPtr;
        // Single-DOF joints only: a multi-DOF joint (D6) would need the USD axis -> physxDofIdx map the
        // parse layer owns, so any scene carrying one falls back. Fixed / 0-DOF joints carry no DOF and
        // are simply not part of the set to cover. The batch must cover every DOF-bearing joint, since
        // planJointGroup zeros the whole column -- a partial batch would zero the DOFs it omits.
        std::unordered_map<ObjectKey, uint32_t, ObjectKey::Hash> keyToDof; // joint key -> flat DOF index
        bool multiDof = false;
        for (const ovx::JointSlice& sl : jc.slices)
        {
            if (sl.count > 1)
            {
                multiDof = true;
                break;
            }
            if (sl.count == 1)
                keyToDof.emplace(sl.key, sl.offset);
        }
        if (multiDof || keyToDof.size() != batch.numChanges)
            continue;
        // Candidacy only: every batch key is a single-DOF joint of this scene on the slices as they stand
        // now (this routes the batch to the right scene). The authoritative DOF offsets are taken AFTER
        // planJointGroup below -- it may rebuild the slices for a newer view generation, and an offset
        // captured here would then index a differently laid-out column.
        bool covers = true;
        for (size_t i = 0; i < batch.numChanges && covers; ++i)
            covers = keyToDof.count(keys[i]) != 0;
        if (!covers)
            continue;

        WriteSession s;
        if (!planJointGroup(s, source, scene, *row) || s.groups.empty())
            return false;
        GroupSlot& slot = *s.groups.back();

        // planJointGroup ran refreshArticulationCache, which can REPUBLISH g_articulationReadCache
        // (move-assign the whole map) -- that dangles the jcPtr / jc taken above the plan, so neither may be
        // read here. It can also rebuild THIS entry's slices/recs in place (ensureJointRecords) for the view
        // generation it acquired, sizing slot.numOut = jc.recs.size() to that layout. Re-fetch the entry and
        // take the authoritative DOF offsets from it, so every offset indexes the column slot.numOut
        // describes -- writing a pre-plan offset into a post-plan column is the stale-index bug the tendon
        // path had.
        jcPtr = ovx::articulationCacheEntry(scene);
        if (!jcPtr)
        {
            releaseGroupStorage(s);
            return false;
        }
        // A rebuild that reordered or shrank the single-DOF set out from under the batch (slot.numOut no
        // longer == numChanges, a key gone, or an offset out of range) falls back to the parse path; nothing
        // is committed yet.
        if (static_cast<size_t>(slot.numOut) != batch.numChanges)
        {
            releaseGroupStorage(s);
            return false;
        }
        std::unordered_map<ObjectKey, uint32_t, ObjectKey::Hash> keyToDofNow;
        for (const ovx::JointSlice& sl : jcPtr->slices)
            if (sl.count == 1)
                keyToDofNow.emplace(sl.key, sl.offset);
        // Fill every single-DOF joint from the batch with the RAW authored column, then scatter. The
        // deg->rad fold and the joint's sign are applied by the setter scatterGroup runs
        // (setDof*OvStage -> dofInverseScaleFor, eAngularSigned = invAngScale * sign) exactly as the normal
        // ovphysx_write joint path does -- pre-folding here would apply invAngScale twice (~57x too small on
        // a revolute axis; linear is x1, so it is invisible on a prismatic joint).
        std::vector<float> host(static_cast<size_t>(slot.numOut), 0.0f);
        bool placed = true;
        for (size_t i = 0; i < batch.numChanges && placed; ++i)
        {
            const std::unordered_map<ObjectKey, uint32_t, ObjectKey::Hash>::const_iterator it =
                keyToDofNow.find(keys[i]);
            placed = (it != keyToDofNow.end()) && (it->second < static_cast<uint32_t>(slot.numOut));
            if (placed)
                host[it->second] = col[i];
        }
        if (!placed)
        {
            releaseGroupStorage(s);
            return false;
        }
        if (slot.deviceData && slot.ctxMgr)
        {
            PxScopedCudaLock _lock(*slot.ctxMgr);
            slot.ctxMgr->getCudaContext()->memcpyHtoD(slot.deviceData, host.data(), host.size() * sizeof(float));
        }
        else
        {
            std::copy(host.begin(), host.end(), slot.floats.begin());
        }
        slot.committed = true;
        const bool ok = scatterGroup(slot, ovstage_cuda_sync_t{});
        releaseGroupStorage(s);
        return ok;
    }
    return false; // no scene's full single-DOF joint set matched -> parse-layer fallback
}

// Particle-set drain. USD authors particle positions / velocities as `points` / `velocities`
// (VtArray<GfVec3f>) on the set prim -- a ragged per-particle vec3 column. Routes only when the batch
// covers every particle set in the scene (planParticleGroup fills the whole compact column) and each
// set's delivered particle count matches, then fills each set's slice from the ragged column by set key.
static bool applyParticleBatch(parse::IPhysicsSource& source, const parse::ChangeBatch& batch,
                               const std::string& usdAttr)
{
    bool velocity = false;
    if (usdAttr == "points")
        velocity = false;
    else if (usdAttr == "velocities")
        velocity = true;
    else
        return false;
    // A ragged host vec3 column (the feed's array path) is required.
    if (!batch.valueRowOffsets || !batch.values.data || batch.values.device >= 0 ||
        drainColumnComponents(batch.values.type) != 3)
        return false;

    const ObjectKey* keys = static_cast<const ObjectKey*>(batch.keys.data);
    const float* col = static_cast<const float*>(batch.values.data);
    for (PxScene* scene : ovx::allPhysicsScenes())
    {
        ovx::ParticleWriteTargets targets;
        if (!ovx::particleWriteTargets(scene, 0u, targets) || targets.keys.empty())
            continue;
        if (targets.keys.size() != batch.numChanges)
            continue; // not this scene's full particle-set set
        std::unordered_map<ObjectKey, uint32_t, ObjectKey::Hash> keyToPos;
        for (uint32_t i = 0; i < targets.keys.size(); ++i)
            keyToPos.emplace(targets.keys[i], i);
        bool covers = true;
        for (size_t i = 0; i < batch.numChanges && covers; ++i)
        {
            const std::unordered_map<ObjectKey, uint32_t, ObjectKey::Hash>::const_iterator it = keyToPos.find(keys[i]);
            // Cover every set, and require the delivered per-set vertex count to match the set's size.
            if (it == keyToPos.end() ||
                (batch.valueRowOffsets[i + 1] - batch.valueRowOffsets[i]) != targets.counts[it->second])
                covers = false;
        }
        if (!covers)
            continue;

        WriteSession s;
        if (!planParticleGroup(s, source, scene, targets, velocity) || s.groups.empty())
            return false;
        GroupSlot& slot = *s.groups.back();
        // Map each batch row's set key to its slot slice, then copy the ragged points into slot.floats.
        std::unordered_map<ObjectKey, uint32_t, ObjectKey::Hash> slotKeyToIdx;
        for (uint32_t j = 0; j < slot.numOut; ++j)
            slotKeyToIdx.emplace(slot.keys[j], j);
        for (size_t i = 0; i < batch.numChanges; ++i)
        {
            const std::unordered_map<ObjectKey, uint32_t, ObjectKey::Hash>::const_iterator it =
                slotKeyToIdx.find(keys[i]);
            if (it == slotKeyToIdx.end())
                continue; // a set filtered out of the group (count 0) -- nothing to fill
            const uint32_t j = it->second;
            const size_t floats = size_t(slot.particleCounts[j]) * 3u;
            std::memcpy(&slot.floats[slot.particleFloatOffsets[j]], col + size_t(batch.valueRowOffsets[i]) * 3u,
                        floats * sizeof(float));
        }
        slot.committed = true;
        const bool ok = scatterGroup(slot, ovstage_cuda_sync_t{});
        releaseGroupStorage(s);
        return ok;
    }
    return false; // no scene's full particle set matched -> parse-layer fallback
}


// Standalone dynamic bodies whose authored linear/angular velocity is expressed in the body's OWN local
// frame (the `physics:localSpaceVelocities` metadata, tracked as InternalActorFlag::eLOCALSPACE_VELOCITIES).
// The parse-layer velocity update rotates such a velocity by the body's world orientation and per-axis
// scales it into world space before applying (PhysXRigidBodyPropertiesUpdate.cpp updateLinearVelocity); this
// columnar drain scatters the raw column and does neither, so a velocity batch touching one of these bodies
// must fall back whole to the parse path -- otherwise a local-frame velocity would land as if it were
// world-frame, silently wrong. One DB walk, built only for a velocity drain.
static std::unordered_set<const PxRigidBody*> collectLocalSpaceVelocityBodies()
{
    std::unordered_set<const PxRigidBody*> out;
    const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    for (const internal::InternalDatabase::Record& rec : db.getRecords())
    {
        if (rec.mType != ePTActor || !rec.mPtr || !rec.mInternalPtr)
            continue;
        const internal::InternalActor* ia = reinterpret_cast<const internal::InternalActor*>(rec.mInternalPtr);
        if (!(ia->mFlags & internal::InternalActorFlag::eLOCALSPACE_VELOCITIES))
            continue;
        if (PxRigidDynamic* dyn = reinterpret_cast<PxRigidActor*>(rec.mPtr)->is<PxRigidDynamic>())
            out.insert(dyn);
    }
    return out;
}

// Unit / frame conversion contract for the drain (ADR-0001 unit contract: a value reaches PhysX in
// engine-native units, each conversion applied EXACTLY ONCE). Every bridged attribute either has its
// conversion baked into the scatter setter -- in which case the drain scatters the RAW column -- or needs the
// conversion applied HERE before scattering. Both drain bug classes are a mismatch with this table: a MISSING
// conversion (velocity into a local frame, COM without the local scale) or a DUPLICATED one (the angular DOF
// double-fold), each landing the value wrong and slipping past a test that happens to use a x1 axis. Check a
// new bridged attribute against this before adding it -- the fold's HOME differs by object type:
//
//   attribute                          conversion         applied by
//   physics:velocity                   none (world) [1]   raw scatter
//   physics:angularVelocity            deg->rad           THE DRAIN (drainScatterRigidGroup, angular=true);
//                                                          the velocity setter takes engine-native rad
//   physxRigidBody:disableGravity      none               raw scatter
//   physics:mass / centerOfMass /      n/a                NOT DRAINED -- coupled mass frame; falls back to the
//     diagonalInertia / principalAxes                     parse path's holistic recompute (at next step)
//   drive|state:<axis> DOF             deg->rad * sign    THE SETTER (setDof*OvStage -> dofInverseScaleFor,
//     (target/state pos + vel)         (angular)          eAngularSigned); the drain scatters the RAW column
//   vehicle drive/brakeTorque,         none               raw scatter (the setter takes the value directly,
//     steerAngle                                          as the ovphysx_write vehicle path does)
//   tendon stiffness/damping/          n/a                NOT DRAINED -- coupled to `tendonEnabled` + needs a
//     limitStiffness/offset                               wake; falls back to the parse path (follow-up MR)
//   particle points                    sim-local->world   THE SCATTER (submitPointSetColumnOvStage /
//                                       reframe            the particle scatter loop apply affineInverse of
//                                                          the world->local / world->sim-mesh transform --
//                                                          the inverse of the read's reframe, scale-aware)
//   particle velocities                none               raw ragged scatter (world in both directions)
//   deformablePose:*:points /          n/a                NOT DRAINED -- bind-pose authoring, not live state;
//     :velocities                                         falls back to the parse-path resync (re-cook at step)
//
// [1] a body carrying physics:localSpaceVelocities authors velocity in its LOCAL frame; the parse path
//     reframes it (rotate + scale), which the raw scatter cannot, so those bodies fall back whole.
OMNI_OVX_WRITE_API DrainResult applyOvstageValueBatch(parse::IPhysicsSource& source, const parse::ChangeBatch& batch,
                                                      std::vector<uint32_t>* unresolvedRows)
{
    // Value changes only. Deletes / new objects / typeName resyncs (structural) and attributes with no
    // backend setter are NotHandled -> onSourceChange keeps the parse-layer property path for those.
    if (batch.isDelete || batch.numChanges == 0 || !batch.property.valid() ||
        !batch.values.data || !batch.keys.data)
        return DrainResult::NotHandled;

    // Canonical USD-attribute -> rigid write-row bridge. `batch.property` is the raw USD name
    // (conv::toUsdAttributeName is identity for these). These are the fixed-width per-prim rigid attributes
    // the change feed delivers as a dense value column (appendRigidBodyAttrs) and the backend can scatter
    // as-is: velocity / angularVelocity / disableGravity. The rigid mass frame is NOT bridged -- see the
    // dispatch below. Pose is the transform path. Other object types add their own bridges as their planners
    // gain subset support.
    //
    // A body carrying `physics:localSpaceVelocities` has its velocity authored in the body's local frame;
    // the parse path rotates (and per-axis scales) it into world space before applying, which this raw
    // columnar scatter does not. Rather than reframe here, a velocity batch touching any such body falls
    // back whole to the parse path (see the eLOCALSPACE_VELOCITIES guard in the resolution loop), so it is
    // reframed correctly there -- never applied raw in the wrong frame.
    struct UsdOvxRow
    {
        const char* usd;
        const char* ovx;
        bool angular; // authored in USD degrees -> convert deg->rad on the way in (as the parse path does)
    };
    static constexpr UsdOvxRow kRigidBridge[] = {
        { "physics:velocity", OvxAttr::kLinearVelocity, false },
        { "physics:angularVelocity", OvxAttr::kAngularVelocity, true },
        { "physxRigidBody:disableGravity", OvxAttr::kDisableGravity, false },
        // The mass frame (physics:mass / centerOfMass / diagonalInertia / principalAxes) is intentionally
        // absent -- see the dispatch below; it falls back to the parse path's holistic recompute.
    };
    const std::string usdAttr(source.tokenToString(batch.property));

    // Reset the commit-attempt flag before any handler runs (each scatter path sets it at its commit point,
    // AFTER its pre-commit checks -- so a scatter that failed before writing anything leaves it clear).
    // Reaching the fall-through below with it set means a handler committed part of the batch and then failed
    // -> Failed (must NOT fall back, that would double-apply the committed part); otherwise nothing was
    // committed -> NotHandled (fall back). A handler that fully applied returns true here (Applied).
    t_scatterCommitAttempted = false;
    const auto notHandledOrFailed = []
    { return t_scatterCommitAttempted ? DrainResult::Failed : DrainResult::NotHandled; };

    // Vehicle wheel controls route through the CPU-only vehicle write path (whole-wheel-set).
    if (applyVehicleBatch(source, batch, usdAttr))
        return DrainResult::Applied;
    // Articulation tendon knobs (physxTendon:<inst>:{stiffness,damping,limitStiffness,offset}) are NOT
    // drained: applying them correctly is coupled to the sibling `tendonEnabled` (a disabled tendon must
    // keep its coefficients at 0) and needs the articulation woken -- semantics the raw write-backend
    // scatter skips. They are left NotHandled and fall back to the parse path's tendon updaters. NOTE: on a
    // DirectGPU scene the parse path's `tendon->setStiffness()` is flagged illegal by PhysX and writes only
    // the CPU core, so a runtime tendon change does not reliably take effect there -- the same limitation as
    // before this drain existed. A follow-up will restore GPU support with the gating + wake applied through
    // the DirectGPU tendon write.
    // The rigid MASS FRAME (physics:mass / centerOfMass / diagonalInertia / principalAxes) is deliberately
    // NOT drained: those attributes are coupled -- the parse path routes all of them through
    // updateBodyDensity -> addDirtyMassActor, a holistic recompute of mass + inertia + COM together (from the
    // body's shapes and every authored mass attribute), applied at the next step. A per-attribute write-row
    // scatter cannot reproduce that coupling (e.g. changing mass alone must rescale auto-derived inertia), so
    // the whole frame is left NotHandled and falls back to that recompute.
    // Articulation DOF (per-axis drive targets / joint state) route through the joint write path
    // (whole single-DOF set of a scene).
    if (applyArticulationBatch(source, batch, usdAttr))
        return DrainResult::Applied;
    // Particle sets deliver ragged per-vertex `points` / `velocities` columns; those ARE live sim state,
    // so the drain scatters them.
    if (applyParticleBatch(source, batch, usdAttr))
        return DrainResult::Applied;
    // Deformable `deformablePose:<inst>:omniphysics:points` is deliberately NOT handled here: it is the
    // sim mesh's BIND POSE (the descriptor field is `simMeshBindPoseToken`), not live solver state. The
    // parse path registers it as a structural resync (physicsDeformableBodyHierarchyResyncCheck) that
    // re-cooks the body from the new rest configuration, so it is left as NotHandled and falls through to
    // that path -- scattering it as live vertex state would teleport the running mesh and leave the rest
    // frame stale. The change is still DELIVERED (the OmniPhysics{Volume,Surface}DeformableSimAPI schemas
    // stay on the feed's known list) so that resync fires; the re-cook is applied at the next step.

    const char* ovxName = nullptr;
    bool angular = false;
    for (const UsdOvxRow& e : kRigidBridge)
        if (usdAttr == e.usd)
        {
            ovxName = e.ovx;
            angular = e.angular;
            break;
        }
    if (!ovxName)
        return notHandledOrFailed();

    // RIGID standalone bodies. (Articulation links -- which share kRigidWriteAttributes -- and the
    // other object types fan out here on the same shape.)
    const WriteAttributeRow* row = findRigidWriteAttribute(ovxName);
    if (!row || (!row->gpuWrite && !row->cpuWrite && !row->baseWrite) || row->perShape)
        return notHandledOrFailed(); // not a fixed-width rigid backend attribute -> parse-layer fallback

    // A kinematic body cannot take a (angular)velocity -- PxRigidDynamic::set(Angular)Velocity FATAL-errors
    // on one, and a velocity authored on a kinematic body is a surface (conveyor) velocity the parse path
    // routes to physxSurfaceVelocity. So a velocity batch touching any kinematic body falls back whole.
    const bool velocityRow = (ovxName == OvxAttr::kLinearVelocity || ovxName == OvxAttr::kAngularVelocity);

    // The column must be exactly numChanges rows of row.components; otherwise a narrower / mis-authored
    // column would be read out of bounds in the copy below. Reject a mismatch to the parse-layer path.
    if (batch.values.count != batch.numChanges ||
        drainColumnComponents(batch.values.type) != static_cast<uint32_t>(row->components))
        return notHandledOrFailed();

    const ObjectKey* keys = static_cast<const ObjectKey*>(batch.keys.data);

    // Resolve the changed keys against the read's own rigid AND articulation-link enumerations, so the
    // drain covers exactly what a read would: a value batch may target a standalone body or a link. A
    // link accepts only the attributes PhysX can set on a link (`row->linkWritable`) -- the rest fall
    // through to the parse-layer path. A key maps to at most one (scene, body).
    struct BodyHit
    {
        PxScene* scene;
        PxRigidBody* body;
        bool link;
    };
    ovx::RigidRecordScan rigidScan;
    ovx::scanRigidRecords(kOvxAll, ovx::ActiveActorSet{}, rigidScan);
    std::unordered_map<ObjectKey, BodyHit, ObjectKey::Hash> keyToBody;
    for (const ovx::RigidSceneBucket& b : rigidScan.byScene)
        for (size_t i = 0; i < b.keys.size(); ++i)
            keyToBody.emplace(b.keys[i], BodyHit{ b.scene, b.bodies[i], false });
    // Scan links only for an attribute PhysX can set on one. A velocity row has linkWritable == false, so
    // every link hit is discarded in the resolution loop below -- scanning the whole link set
    // (collectActiveActors' bitset + DB walk, then scanLinkRecords' walk) is pure waste for it.
    if (row->linkWritable)
    {
        ovx::RigidRecordScan linkScan;
        ovx::scanLinkRecords(kOvxActive, ovx::collectActiveActors(), linkScan);
        for (const ovx::RigidSceneBucket& b : linkScan.byScene)
            for (size_t i = 0; i < b.keys.size(); ++i)
                keyToBody.emplace(b.keys[i], BodyHit{ b.scene, b.bodies[i], true });
    }

    // Partition by owning scene in batch order (so the column, in batch order, aligns), keeping each
    // row's column index for the gather.
    struct SceneGroup
    {
        std::vector<PxRigidBody*> bodies;
        std::vector<ObjectKey> keys;
        std::vector<uint32_t> rows;
    };
    std::unordered_map<PxScene*, SceneGroup> byScene;

    // Bodies whose velocity is authored in their own local frame need the parse path's reframe; a velocity
    // batch touching one falls back whole (via the all-or-nothing check below). Built once, velocity only.
    const std::unordered_set<const PxRigidBody*> localSpaceVelBodies =
        velocityRow ? collectLocalSpaceVelocityBodies() : std::unordered_set<const PxRigidBody*>{};

    // A batch row the drain will not service: record it (when the caller supplied a sink) so onSourceChange
    // runs its per-object parse fallback for exactly that key, then skip it here.
    const auto skipRow = [&](size_t i)
    { if (unresolvedRows) unresolvedRows->push_back(static_cast<uint32_t>(i)); };

    size_t resolved = 0;
    for (size_t i = 0; i < batch.numChanges; ++i)
    {
        const std::unordered_map<ObjectKey, BodyHit, ObjectKey::Hash>::const_iterator it =
            keyToBody.find(keys[i]);
        if (it == keyToBody.end() || !it->second.body)
        {
            skipRow(i);
            continue; // not a rigid body or link -> left to the parse-layer property path
        }
        if (it->second.link && !row->linkWritable)
        {
            skipRow(i);
            continue; // this attribute is not writable on an articulation link
        }
        if (velocityRow)
        {
            const PxRigidDynamic* dyn = it->second.body->is<PxRigidDynamic>();
            if (dyn && (dyn->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
            {
                skipRow(i);
                continue; // kinematic -> surface velocity on the parse path, not setLinearVelocity here
            }
            if (localSpaceVelBodies.count(it->second.body))
            {
                skipRow(i);
                continue; // velocity authored in the body's local frame -> parse path reframes it (rotate + scale)
            }
        }
        SceneGroup& g = byScene[it->second.scene];
        g.bodies.push_back(it->second.body);
        g.keys.push_back(keys[i]);
        g.rows.push_back(static_cast<uint32_t>(i));
        ++resolved;
    }
    // Split, don't abandon. Scatter the resolved keys through the backend and let onSourceChange run the
    // parse-layer path for the skipped ones (recorded above) -- keeping the drain's DirectGPU-correct path
    // for every serviceable key in a batch that also carries a skip (else a whole-batch fallback would route
    // the serviceable bodies to the per-object velocity setter, which is inert on DirectGPU, dropping them).
    // Two cases still fall back WHOLE: no key resolved (nothing to scatter), or the caller passed no sink for
    // the skipped rows (`unresolvedRows` null) -- there a partial Applied would make onSourceChange silently
    // drop them, so the older all-or-nothing is preserved. A skipped key is never committed, so the fallback
    // cannot double-apply.
    if (resolved == 0 || (resolved != batch.numChanges && !unresolvedRows))
        return notHandledOrFailed();

    bool ok = true;
    for (const std::pair<PxScene* const, SceneGroup>& e : byScene)
    {
        const bool whole = (e.second.bodies.size() == batch.numChanges);
        ok = drainScatterRigidGroup(source, e.first, e.second.bodies, e.second.keys, *row, batch,
                                    whole ? nullptr : &e.second.rows, angular) &&
             ok;
    }
    // Re-enabling gravity (disableGravity -> false) must wake a sleeping non-kinematic dynamic, or it stays
    // asleep instead of beginning to fall. The parse path (updateBodyDisableGravity) wakes it, but the backend
    // setter only flips the actor flag -- so wake here, reading the flag the scatter just set (gravity is now
    // enabled iff eDISABLE_GRAVITY is clear). Matches the parse path, which wakes whenever the applied value is
    // false, regardless of the prior value.
    if (ovxName == OvxAttr::kDisableGravity)
    {
        for (const std::pair<PxScene* const, SceneGroup>& e : byScene)
            for (PxRigidBody* b : e.second.bodies)
            {
                // Disabled dynamics were dropped from the scatter (no device state row), so their gravity
                // flag was not changed here; skip them -- PxRigidDynamic::wakeUp() is forbidden on an
                // eDISABLE_SIMULATION body anyway.
                if (ovx::isDisabledRigidDynamic(b))
                    continue;
                PxRigidDynamic* dyn = b->is<PxRigidDynamic>();
                if (dyn && !(dyn->getActorFlags() & PxActorFlag::eDISABLE_GRAVITY) &&
                    !(dyn->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
                    dyn->wakeUp();
            }
    }
    // ok is false if any scene's scatter failed. Route it through notHandledOrFailed() rather than a bare
    // Failed: a scatter that committed part of the set set t_scatterCommitAttempted, so that stays Failed (do
    // NOT fall back -- it would double-apply the committed part); but a NON-commit failure
    // (drainScatterRigidGroup bailing on a device column it can't borrow, or a planRigidGroup CUDA-context /
    // alloc failure -- nothing scattered) comes back NotHandled and falls back to the parse path. A bare
    // Failed there is unrecoverable: onSourceChange returns false -> drainRange holds the cursor -> a
    // deterministic failure grows the held range every frame until it falls off the retained history and
    // ovstage changes stop reaching physics for good.
    return ok ? DrainResult::Applied : notHandledOrFailed();
}

} // namespace omni::physx
