// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-3 AC-4

/**
 * @implements REQ-INPUT-CORE-001
 * @covers AC-1 AC-5 AC-6 AC-10 AC-12 AC-13 AC-14 AC-16
 *
 * @implements REQ-INPUT-DEVICE-001
 * @covers AC-1 AC-1a AC-1b AC-3 AC-9 AC-10 AC-11 AC-12 AC-13 AC-14
 *
 * @implements REQ-INPUT-COVERAGE-001
 * @covers AC-5 AC-6 AC-7 AC-8 AC-9 AC-12 AC-13 AC-14 AC-15 AC-16 AC-18 AC-19 AC-20 AC-21
 *
 * @implements REQ-SIM-OVSTAGE-WRITEAPPLY-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9 AC-10 AC-12 AC-13 AC-14 AC-18 AC-19 AC-20 AC-21
 *
 * @implements REQ-PROPS-LINK-001
 * @covers AC-4
 *
 * @implements REQ-RUNTIME-ERROR-001
 * @covers AC-3 AC-4
 *
 * @implements REQ-READ-SKIN-001
 * @covers AC-7
 * @implements REQ-READ-CCT-001
 * @covers AC-2 AC-4 AC-5
 *
 * @implements REQ-READ-SELECT-001
 * @covers AC-4
 *
 * @implements REQ-READ-CORE-001
 * @covers AC-11
 *
 * @implements REQ-READ-SDF-001
 * @covers AC-2
 *
 * ADR-0012: the app -> physics write, the return direction of OvxPhysicsRead.cpp.
 *
 * WHAT IS HERE: the session -- handle table, group registry, the commit/discard lifecycle, the
 * address-stability rule that makes commit-by-pointer decidable, and rigid-body group planning.
 *
 * SCOPE: each object type has its own attribute table because each reaches different state
 * through a different view or runtime record -- not because the vocabulary differs.
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

/**
 * @implements REQ-ATTACH-IO-001
 * @covers AC-2
 *
 * @implements REQ-READ-TENDON-001
 * @covers AC-5
 */

#include "RuntimeExecution.h"
#include "DebugRenderSession.h"
#include "usdLoad/LoadUsd.h"
#include <omni/physx/IOvxPhysicsWrite.h>
#include <characterkinematic/PxController.h>
#include <omni/physx/RuntimeError.h>
#include <omni/physx/IOvxPhysicsRead.h> // OvxAttr names, OvxObjectType, kOvx* scopes
#include <omni/physics/parse/IChangeFeed.h> // ChangeBatch/ColumnView -- the ovstage drain's apply input
#include <cstring> // std::memcpy (drain column gather/stage)

#include "OvxPhysicsShared.h" // enumeration + query resolution shared with the read
#include "CallerStream.h"
#include "QueryDeviceBuffers.h" // device buffers reused across steps over a held query

#include <OvstageOutput.h> // buildPathList; private to omni.physics.ovstage
#include <OvstageSource.h> // conv::kFabricWorldMatrix; private to omni.physics.ovstage

#include "OmniPhysX.h"
#include "usdLoad/AttachedStage.h"
#include "usdLoad/LoadUsd.h" // UsdLoad::getUsdLoad / getAttachedStageByHandle
#include <common/foundation/MatrixTools.h> // affineInverse -- a prim transform may carry scale

#include "internal/Internal.h" // recordLifetimeEpoch
#include "internal/InternalVehicle.h"
#include "internal/InternalScene.h"
#include "tensors/base/BaseVehicleView.h"
#include "internal/InternalActor.h" // InternalActorFlag::eLOCALSPACE_VELOCITIES -- the local-frame velocity guard
#include "PhysXScene.h" // scene timestep for PxController::move
#include "internal/InternalParticle.h" // InternalParticleSet -- the particle write's destination
#include "internal/InternalDeformable.h" // InternalDeformableBody -- the sim-mesh reframe
#include "tensors/PointSetReframe.h" // PointSetTransform
#include "tensors/gpu/CudaKernels.h" // submitPointSetColumnOvStage
#include "PhysXTools.h" // getValue -- the tendon drain reads as the parse path does

#include "tensors/SimulationBackend.h"
#include "tensors/GlobalsAreBad.h"
#include "tensors/gpu/GpuSimulationView.h"
#include "tensors/gpu/GpuRigidBodyView.h"
#include "tensors/cpu/CpuSimulationView.h"
#include "tensors/cpu/CpuRigidBodyView.h"
#include "tensors/base/BasePointInstancerView.h" // instancer instance scatter (ADR-0012)
#include "tensors/gpu/GpuPointInstancerView.h" // a session's instancer write is prepared at open (ADR-0052)
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
#include <cmath> // std::fmod (wrapped revolute drive target)
#include <cstdio>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <array>
#include <map>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace ::physx;
using omni::physx::usdparser::AttachedStage;
using omni::physx::usdparser::UsdLoad;
using omni::physx::internal::InternalScene;
using omni::physx::cudaShimWrappers::getCudaShim;
using omni::physics::parse::IPhysicsSource;
using omni::physics::parse::ObjectKey;

// The robotics diagnostic channel the parse path warns on (registered in OmniPhysX.cpp) -- the joint
// drain wraps a wrapped-revolute drive target and warns on it here, matching the parse path.
OMNI_LOG_DECLARE_CHANNEL(kRoboticsLogChannel)

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
    // The trailing bool is `wholeSet`: true when the write names every row the attachment owns (scope
    // kOvxAll, no selection, no disabled/controller filtering), so a setter that confirms this covers its
    // own view can skip subset bookkeeping. Only the link-wrench/force path reads it today; the others
    // accept and ignore it.
    bool (omni::physx::tensors::GpuRigidBodyView::*gpuWrite)(const omni::physics::tensors::TensorDesc*,
                                                             const PxU32*,
                                                             PxU32,
                                                             uint64_t,
                                                             bool,
                                                             const omni::physx::tensors::OvStageDeviceWriteArgs&);
    bool (omni::physx::tensors::CpuRigidBodyView::*cpuWrite)(const omni::physics::tensors::TensorDesc*,
                                                             const PxU32*,
                                                             PxU32,
                                                             uint64_t,
                                                             bool);
    bool (omni::physx::tensors::BaseRigidBodyView::*baseWrite)(const omni::physics::tensors::TensorDesc*,
                                                              const PxU32*,
                                                              PxU32,
                                                              uint64_t,
                                                              bool);
};

constexpr omni::physics::tensors::TensorDataType kF32 = omni::physics::tensors::TensorDataType::eFloat32;

constexpr WriteAttributeRow kRigidWriteAttributes[] = {
#include "generated/kRigidWriteAttributes.inc"
};

// The joint-DOF attributes a session can write. Mirrors kJointAttributes in OvxPhysicsRead.cpp: a
// row names the view method that publishes it, so adding one is a row here plus a thin method on
// each articulation view. The GPU method also takes how the write runs on the device.
struct JointWriteAttributeRow
{
    const char* token;
    bool (omni::physx::tensors::GpuArticulationView::*gpuWrite)(
        const omni::physics::tensors::TensorDesc*, const omni::physx::tensors::ArticulationDofOvStageRecord*, PxU32,
        const omni::physx::tensors::OvStageDeviceWriteArgs&);
    bool (omni::physx::tensors::CpuArticulationView::*cpuWrite)(
        const omni::physics::tensors::TensorDesc*, const omni::physx::tensors::ArticulationDofOvStageRecord*, PxU32);
};

constexpr JointWriteAttributeRow kJointWriteAttributes[] = {
#include "generated/kJointWriteAttributes.inc"
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
#include "generated/kJointPropertyWriteAttributes.inc"
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
        const omni::physics::tensors::TensorDesc*, const PxU32*, PxU32, uint64_t,
        const omni::physx::tensors::OvStageDeviceWriteArgs&);
    bool (omni::physx::tensors::CpuArticulationView::*cpuWrite)(
        const omni::physics::tensors::TensorDesc*, const PxU32*, PxU32, uint64_t);
};

constexpr ArtiWriteAttributeRow kArtiWriteAttributes[] = {
#include "generated/kArtiWriteAttributes.inc"
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
#include "generated/kTendonWriteAttributes.inc"
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

constexpr auto ioWriteDeformableDynamicFriction = [](::physx::PxDeformableMaterial& m, float v) { m.setDynamicFriction(v); };
constexpr auto ioWriteDeformableYoungsModulus = [](::physx::PxDeformableMaterial& m, float v) { m.setYoungsModulus(v); };
constexpr auto ioWriteDeformablePoissonsRatio = [](::physx::PxDeformableMaterial& m, float v) { m.setPoissons(v); };
constexpr auto ioWriteDeformableElasticityDamping = [](::physx::PxDeformableMaterial& m, float v) { m.setElasticityDamping(v); };
constexpr auto ioWriteDeformableBendingStiffness = [](::physx::PxDeformableMaterial& m, float v) { static_cast<::physx::PxDeformableSurfaceMaterial&>(m).setBendingStiffness(v); };
constexpr auto ioWriteDeformableThickness = [](::physx::PxDeformableMaterial& m, float v) { static_cast<::physx::PxDeformableSurfaceMaterial&>(m).setThickness(v); };
constexpr auto ioWriteDeformableBendingDamping = [](::physx::PxDeformableMaterial& m, float v) { static_cast<::physx::PxDeformableSurfaceMaterial&>(m).setBendingDamping(v); };

constexpr DeformableMaterialWriteRow kDeformableMaterialWriteAttributes[] = {
#include "generated/kDeformableMaterialWriteAttributes.inc"
};

const DeformableMaterialWriteRow* findDeformableMaterialWriteAttribute(std::string_view token)
{
    for (const DeformableMaterialWriteRow& row : kDeformableMaterialWriteAttributes)
        if (token == row.token)
            return &row;
    return nullptr;
}

// Raw-wheel controls and whole-vehicle controls use separate selectors.
struct VehicleWriteAttributeRow
{
    const char* token;
    omni::physx::tensors::BaseVehicleView::WheelControl control;
};

constexpr VehicleWriteAttributeRow kVehicleWriteAttributes[] = {
#include "generated/kVehicleWriteAttributes.inc"
};

const VehicleWriteAttributeRow* findVehicleWriteAttribute(std::string_view token)
{
    for (const VehicleWriteAttributeRow& row : kVehicleWriteAttributes)
        if (token == row.token)
            return &row;
    return nullptr;
}

// Character-controller movement. Host-only: PxController::move is a CPU call, and on a
// DirectGPU scene its kinematic target is not uploaded. The written vector is a world-space
// displacement applied once, at commit, with the owning scene's timestep.
struct CctWriteAttributeRow
{
    const char* token;
    uint32_t components;
};

constexpr CctWriteAttributeRow kCctWriteAttributes[] = {
#include "generated/kCctWriteAttributes.inc"
};

const CctWriteAttributeRow* findCctWriteAttribute(std::string_view token)
{
    for (const CctWriteAttributeRow& row : kCctWriteAttributes)
        if (token == row.token)
            return &row;
    return nullptr;
}

struct VehicleControlAttributeRow
{
    const char* token;
    omni::physx::tensors::BaseVehicleView::VehicleControl control;
    omni::physics::tensors::TensorDataType dtype;
};

constexpr VehicleControlAttributeRow kVehicleControlAttributes[] = {
#include "generated/kVehicleControlAttributes.inc"
};

const VehicleControlAttributeRow* findVehicleControlAttribute(std::string_view name)
{
    for (const VehicleControlAttributeRow& row : kVehicleControlAttributes)
        if (name == row.token)
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
    explicit GroupSlot(AttachHandle owner) : attach(owner)
    {
    }

    GroupSlot(const GroupSlot&) = delete;
    GroupSlot& operator=(const GroupSlot&) = delete;

    ovstage_map_group_t group{};
    size_t attributeIndex = 0;
    bool committed = false;
    // Set while an ACCEPTED commit's producer handoff (ovstage_cuda_sync_t) is not proven complete:
    // the caller's stream or event may still be writing this slot's storage, host floats included,
    // so release quarantines the whole slot instead of freeing it. Cleared only once every supplied
    // dependency has completed on the host (honourCallerSync).
    bool producerUnproven = false;

    // Storage the CALLER fills, aliased by group.data.tensors[0]. Host or device by the scene's
    // pipeline, matching what the read emits for the same scene, so a caller can hand a column
    // straight back without moving it across the bus.
    std::vector<float> floats; // host path
    CUdeviceptr deviceData = 0; // device path; freed with the session under ctxMgr's context
    int deviceOrdinal = -1;
    PxCudaContextManager* ctxMgr = nullptr;
    // Set when the column came from the query's buffers (allocateColumn): release parks it there again
    // instead of freeing it. `columnBytes` is its size; `recsVersion` is the content version of the records
    // `recsDev` holds (0 = upload again).
    std::shared_ptr<ovx::QueryDeviceBuffers> columnCache;
    ovx::QueryDeviceBuffers::Key columnKey{};
    size_t columnBytes = 0;
    uint64_t recsVersion = 0;

    // A device group whose commit is ordered on the caller's stream (ADR-0052). Its device work runs in the
    // session's own buffers -- the column, the record list, `scratchDev` (laid out by the view) -- so a
    // captured commit replays against buffers nothing else resizes or overwrites. The views' tables it also
    // reads are rewritten on the host only once it is done (GpuSimulationData::noteTableReader). PhysX forks from the
    // caller's stream through `forkEvent` and the stream rejoins through `joinEvent`. `readyEvent` is
    // recorded once the commit's device work is queued and travels with the parked buffers: their next
    // user waits on it before writing them.
    bool streamOrdered = false;
    CUdeviceptr scratchDev = 0;
    size_t scratchBytes = 0;
    // The content version of the leading list `scratchDev` holds (0 = upload again).
    uint64_t scratchVersion = 0;
    uintptr_t readyEvent = 0;
    // True once this session has waited for `readyEvent` on the host, before writing reused buffers.
    bool readyWaited = false;
    uintptr_t forkEvent = 0;
    uintptr_t joinEvent = 0;
    // Resolved at open: the superset rows the group writes, their content token, the view generation they
    // address, the scene's disable epoch they were checked against, and what the view prepared.
    std::vector<PxU32> hostRows;
    uint64_t hostRowsToken = 0;
    uint64_t viewGeneration = 0;
    uint64_t disableEpoch = 0;
    // True when the rows are rigid dynamics, whose DirectGPU index a disable or re-enable moves.
    bool rigidDynamicRows = false;
    omni::physx::tensors::OvStageDeviceWriteArgs prepared;
    // Set once a commit was queued on a capturing stream: the graph, not this session, runs its device
    // work, so no completion was recorded for it.
    bool captured = false;

    // The ovstage drain reuse: when the incoming column already lives where the scene's scatter reads
    // (device column on a DirectGPU scene, host column on a CPU scene), the planner allocates nothing
    // and `deviceData`/`floats` are pointed straight at that column -- a borrow, not owned storage, so
    // release must not free it. A residency MISMATCH (today's host-only ovstage feeding a GPU scene)
    // is not a borrow: the planner allocates and the drain stages the column across the bus into it.
    bool borrowed = false;

    // Stable backing for the DLTensors and the prim group; both hand out pointers that must outlive
    // the fetch that published them.
    //
    // Vectors rather than singletons because a group is one of two shapes. A rigid or joint group is
    // FIXED: one tensor stacking every row along its leading dimension. A deformable group is an ARRAY:
    // one tensor PER PRIM, all pointing into one buffer, which is ovstage's per-prim shape and what the
    // deformable read emits.
    std::vector<int64_t> shapes;
    std::vector<DLTensor> tensors;
    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
    ovx_path_dictionary_t* listDictionary = nullptr;
    AttachHandle attach;

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
    // True when this group names every row the attachment owns (scope kOvxAll, no selection, no disabled /
    // controller filtering). Handed to the write setter, which takes the link-wrench whole-view fast path
    // only when that also covers its view -- an articulation not yet in a scene when the write was opened is
    // left out of the group and joins the view at the next step.
    // Conservative: any filtering leaves it false (the sparse path is always correct, just slower).
    bool wholeSet = false;

    // Joint groups only. `numOut` is the flat DOF count the column carries (the sum of every slice),
    // and `recsDev` is the uploaded per-output record list the scatter indexes through -- device
    // memory owned by this slot and freed with the session, like the column itself.
    const JointWriteAttributeRow* jointRow = nullptr;
    CUdeviceptr recsDev = 0;
    uint32_t numOut = 0;
    // When a prim-set selection compacted the joint records, the CPU scatter re-derives the shared cache
    // by default and would mismatch the compacted column; this holds the filtered records so the scatter
    // applies exactly the selected DOFs. Empty for a whole-set write (the scatter keeps re-deriving).
    std::vector<omni::physx::tensors::ArticulationDofOvStageRecord> recsHost;

    // Articulation-root groups only. `artis` names what to re-resolve at commit, exactly as `bodies`
    // does for a rigid group and for the same reason: a superset row resolved at plan time addresses
    // the view as it was then. `rowsVersion` is minted with the resolved list and identifies its
    // CONTENTS to the view's device row cache.
    const ArtiWriteAttributeRow* artiRow = nullptr;
    std::vector<PxArticulationReducedCoordinate*> artis;
    uint64_t rowsVersion = 0;

    // Tendon groups only. `tendonFixed` picks the kind, which decides both the PhysX struct and
    // whether limit / rest length are addressable at all. `tendonRecsHost`, like the joint `recsHost`,
    // holds the prim-set-compacted record list so the host scatter applies exactly the selected tendons
    // instead of re-reading the full shared cache by row index; empty for a whole-set write.
    const TendonWriteAttributeRow* tendonRow = nullptr;
    bool tendonFixed = false;
    std::vector<omni::physx::tensors::ArticulationTendonOvStageRecord> tendonRecsHost;

    // Vehicle groups capture their selected wheel records or whole-vehicle indices at planning.
    // Commit validates the captured epoch and generation before using these row mappings.
    const VehicleWriteAttributeRow* vehicleRow = nullptr;
    const VehicleControlAttributeRow* vehicleControlRow = nullptr;
    std::vector<omni::physx::tensors::VehicleWheelOvStageRecord> vehicleRecsHost;
    std::vector<uint32_t> wholeVehicleRecs;
    std::vector<int32_t> vehicleGears;
    std::vector<uint8_t> vehicleCommands;
    uint64_t vehicleSetEpoch = 0;
    uint64_t vehicleGeneration = 0;

    // Character-controller groups only. Host-only, one fixed group. `keys` names
    // controllers to re-resolve at commit; no runtime pointer crosses the session.
    const CctWriteAttributeRow* cctRow = nullptr;

    // Joint DOF PROPERTY groups only (ADR-0012). Host-resident by construction.
    const JointPropertyWriteRow* jointPropRow = nullptr;

    // Point-instancer groups only. `instancerIdx` names which of the view's instancers this group publishes.
    bool isInstancer = false;
    // The BASE type: the scatter is virtual, so this path does not branch on which device's view it
    // was handed. The view is a child of the scene view it was taken from, cached by the read, and a rebuild
    // of that scene view deletes it, while the session is still live: an articulation the stage drain created
    // enters its scene at the next step, and that rebuilds the scene view. So it is used only while the scene
    // view still reports `instancerGeneration` (instancerViewCurrent).
    omni::physx::tensors::BasePointInstancerView* instancerView = nullptr;
    uint64_t instancerGeneration = 0;
    omni::physx::tensors::BasePointInstancerView::InstancerColumn instancerColumn{};
    uint32_t instancerIdx = 0;

    // Deformable sim-mesh groups only (ADR-0012). One group per BODY, device-resident: the
    // destination is PhysX's own buffer, so this is a device scatter plus markDirty.
    bool isDeformable = false;
    bool deformableVolume = false;          // which concrete type `deformableBody` is
    bool deformableVelocity = false;        // false = points
    bool deformableKinematicTarget = false; // vec4 per node into the runtime-owned buffer; volume only
    // Parallel, one entry per body in this group. ONE group covers every body of the kind, with one
    // tensor each sharing a single allocation -- the shape the read emits, so a column can be read
    // and handed straight back. A group per body cost 128 device allocations, fills and frees where
    // this costs one, and could not take the read's own output at all.
    std::vector<::physx::PxDeformableBody*> deformableBodies;
    std::vector<omni::physx::internal::InternalDeformableBody*> deformableInternals;
    std::vector<uint32_t> deformableCounts;      // vertices per body
    std::vector<size_t> deformableFloatOffsets;  // where each body's slice starts in the column
    // Kinematic-target groups only: the most nodes of any body, which sizes the commit's launch.
    uint32_t kinematicTargetMaxNodes = 0;

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

    void releaseList()
    {
        if (list != OVX_INVALID_PRIMPATH_LIST)
        {
            // The dictionary is borrowed from this exact attachment. A late native release
            // must not dereference it after detach has destroyed the owner.
            UsdLoad* load = UsdLoad::getUsdLoadIfCreated();
            if (listDictionary && load && load->getAttachedStageByHandle(attach))
                ovx_path_dictionary_destroy_path_list(listDictionary, list);
            list = OVX_INVALID_PRIMPATH_LIST;
        }
    }

    ~GroupSlot()
    {
        // Own the list before any subsequent vector allocation or session adoption can throw.
        releaseList();
        // Covers both an unpublished plan that failed and storage retained past
        // detach. Normal session release clears the allocations first.
        if (!ctxMgr)
            return;
        if (ctxMgr->getCudaContext())
        {
            PxScopedCudaLock lock(*ctxMgr);
            ovx::QueryDeviceBuffers::Buffer owned{
                borrowed ? 0 : deviceData, recsDev, 0, scratchDev, 0, 0, readyEvent
            };
            ovx::QueryDeviceBuffers::freeBuffer(owned);
            if (IOptionalCuda* const cuda = getCudaShim())
            {
                if (forkEvent)
                    cuda->eventDestroy(forkEvent, nullptr);
                if (joinEvent)
                    cuda->eventDestroy(joinEvent, nullptr);
            }
        }
        ctxMgr->release();
    }
};

struct WriteSession
{
    uint64_t attach = 0;
    uint64_t recordEpoch = 0;
    std::string attribute; // Current attribute while building a plan.
    std::vector<std::string> attributeNames;
    std::vector<OvxWriteAttributeResult> attributeResults;
    std::vector<std::unique_ptr<GroupSlot>> groups;
    size_t nextFetch = 0;
    ovx_path_dictionary_t* dict = nullptr;
    // The query's reusable device buffers; shared because the session may outlive its query (ADR-0045).
    std::shared_ptr<ovx::QueryDeviceBuffers> columns;

    // The query's explicit selection or its ACTIVE membership snapshot, captured
    // once at session open. All group planners compact against this same set.
    // An empty set with hasSelection is a valid selection matching nothing.
    bool hasSelection = false;
    std::unordered_set<ObjectKey, ObjectKey::Hash> selectionKeys;
    // Each scene's articulation roots this session writes, filtered once and shared by every root attribute, and the
    // view rows they resolve to under `rowsGeneration` (0 until resolved). The record epoch is fixed for the open
    // (checked at its end), and nothing here outlives the session.
    struct RootPlan
    {
        std::vector<PxArticulationReducedCoordinate*> artis;
        std::vector<ObjectKey> keys;
        std::vector<ovx_primpath_t> handles;
        uint64_t rowsGeneration = 0;
        std::vector<PxU32> rows;
        uint64_t rowsVersion = 0;
    };
    std::unordered_map<const PxScene*, RootPlan> rootPlans;
    bool debugRender = false;
    uint64_t debugOwner = 0;
    debug_render::WriteKind debugKind = debug_render::WriteKind::Scale;
    int debugParam = -1;

    bool admits(const ObjectKey& k) const
    {
        return !hasSelection || selectionKeys.find(k) != selectionKeys.end();
    }
};

 // guards g_writes and g_nextWriteHandle, as OvxPhysicsRead.cpp does
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

// Sentinel for planRigidGroup's wholeSetScope: "no distinct write scope was supplied, use `scope`."
// 0xFFFFFFFF is neither kOvxAll (0) nor kOvxActive (1), so it can never be a real scope.
constexpr uint32_t kWholeSetScopeFromCache = 0xFFFFFFFFu;

// The ovstage drain uses this to tell a PARTIAL COMMIT from "not handled". It is set at each scatter path's
// actual COMMIT point -- right before the backend setter, or (for the sequential particle path) right before
// the first per-set buffer write -- AFTER every pre-commit check has passed. Set it any earlier and a pre-commit failure
// that wrote NOTHING (no backend, no superset view, a record list that moved since planning) would be
// misreported as a partial commit, which holds the drain cursor forever on a deterministic failure instead of
// falling back. Each drain scatter path (rigid, vehicle, joint, particle) sets it at its own commit point; a
// new drain scatter path must do the same. applyOvstageValueBatch resets it before dispatch and reads it
// afterward: a scatter that committed part of a batch and then failed means the drain must NOT fall back
// (that would double-apply the committed part). ovxCommitGroup also resets it before dispatch and reads it
// to distinguish vehicle preflight rejection from a possible partial write. thread_local avoids instance races.
thread_local bool t_scatterCommitAttempted = false;

// Set when a drain SELECTED the DirectGPU path but a transient device-prep step failed before any commit --
// a PLANNING allocation (the session column in allocateColumn, or the joint records buffer / its upload), a
// permutation / producer-ordering sync, or a scatter-kernel launch returned a CUDA error (e.g.
// CUDA_ERROR_OUT_OF_MEMORY). Nothing reached the solver, so this is NOT a partial commit, but it
// must NOT fall back either: the per-object parse-path setter is inert on a DirectGPU scene, so NotHandled
// would advance the consumed ordinal while applying nothing, losing the update with no retry. The dispatcher
// maps it to Failed instead, which holds the cursor so the SAME ordinal is retried once the device recovers.
// Reset before dispatch, like t_scatterCommitAttempted. Distinct from unsupported input (a non-device-eligible
// attribute, a multi-DOF joint, an unresolvable key), which stays NotHandled and correctly falls back.
thread_local bool t_devicePrepFailed = false;

// Point the slot at a device column of `bytes`: the query's parked column for this scene, attribute and
// `sub` group when it has one of that size on this context, otherwise a fresh allocation. The slot owns it
// until release, which parks it again (releaseGroupStorage). A parked column that could not be freed fails
// the allocation, leaving `slot.deviceData` null. Call under the slot's context lock.
void allocateColumn(
    WriteSession& s, GroupSlot& slot, PxCudaContext* cu, const void* row, size_t bytes, uint64_t sub = 0)
{
    slot.columnBytes = bytes;
    // This is the EARLIEST device-prep step a drain reaches (the planning column allocation, before any scatter
    // prep), so a failure here must set t_devicePrepFailed too -- the caller returns false, which maps to
    // NotHandled -> the inert-on-DirectGPU parse path -> the write is lost; the flag makes it retryable instead
    // (AC-12).
    if (s.columns)
    {
        slot.columnCache = s.columns;
        slot.columnKey = ovx::QueryDeviceBuffers::Key{ slot.scene, row, sub };
        ovx::QueryDeviceBuffers::Buffer parked;
        bool freeFailed = false;
        if (s.columns->take(slot.columnKey, slot.ctxMgr, bytes, parked, freeFailed))
        {
            slot.deviceData = parked.data;
            slot.recsDev = parked.recs;
            slot.recsVersion = parked.recsVersion;
            slot.scratchDev = parked.scratch;
            slot.scratchBytes = parked.scratchBytes;
            slot.scratchVersion = parked.scratchVersion;
            slot.readyEvent = parked.ready;
            return;
        }
        if (freeFailed)
        {
            t_devicePrepFailed = true; // transient: could not reclaim a parked device buffer -> retryable
            OVX_RUNTIME_ERROR("ovxWriteAttribute: could not reclaim a parked CUDA buffer; no group emitted.");
            return;
        }
    }
    if (cu->memAlloc(&slot.deviceData, bytes) != 0)
        t_devicePrepFailed = true; // device OOM -> retryable (the dispatcher maps t_devicePrepFailed -> Failed)
}

// Waits, once per session, for the last queued device use of the buffers `slot` reused, before the host
// writes them: a previous session's commit may still be reading them on its caller's stream. Call under the
// slot's context lock.
bool waitBeforeHostWrite(GroupSlot& slot)
{
    if (!slot.readyEvent || slot.readyWaited)
        return true;
    IOptionalCuda* const cuda = getCudaShim();
    if (!cuda || !cuda->eventSynchronize(slot.readyEvent, nullptr))
        return false;
    slot.readyWaited = true;
    return true;
}

// Uploads a group's record list through the optional CUDA interface, under the group's context lock. The
// caller marks the list current only when this succeeds, so a failed copy is never parked as the new version. A
// copy from pageable memory may return before it reaches the device, and a stream-ordered commit runs on the
// caller's stream, which does not wait for the legacy default stream: the copy is waited for.
bool uploadRecords(GroupSlot& slot, CUdeviceptr recsDev, const void* records, size_t bytes)
{
    IOptionalCuda* const cuda = getCudaShim();
    return cuda && waitBeforeHostWrite(slot) &&
           cuda->memcpyHtoD(static_cast<uintptr_t>(recsDev), records, bytes, nullptr) &&
           cuda->streamSynchronize(0, nullptr);
}

// The content version of a selection's records, which are the slices `kept` of shared records whose content version
// is `shared`: a held query's next session finds its parked copy current and uploads nothing. Zero, meaning upload
// again, while the shared records have no version.
uint64_t selectedRecordsVersion(uint64_t shared, const std::vector<size_t>& kept)
{
    if (shared == 0)
        return 0;
    uint64_t hash = 1469598103934665603ull ^ shared;
    for (const size_t index : kept)
        hash = (hash ^ index) * 1099511628211ull;
    return hash ? hash : 1;
}

// The DirectGPU data of a group's scene, which holds the tables its views share across operations; null when the
// scene has no DirectGPU view.
omni::physx::tensors::GpuSimulationDataPtr sceneGpuData(const GroupSlot& slot)
{
    omni::physx::tensors::SimulationBackend* const backend = omni::physx::tensors::GetSimulationBackend();
    omni::physx::tensors::GpuSimulationView* const sv = backend ? backend->acquireSceneView(slot.scene) : nullptr;
    return sv ? sv->getGpuSimulationData() : nullptr;
}

// Build one scene's group: allocate the column the caller fills, name the prims it covers, and stamp
// the DLTensor. Residence follows the scene's pipeline, exactly as the read's does.
//
// `scope` is the CACHE discriminator (cache key + cacheable gate). `wholeSetScope` is the query's REAL
// scope, used only for the whole-set test: the link path forces `scope` to kOvxActive so a link session
// cannot share the rigid cache, which would otherwise report every link write as a subset and keep the
// host dedup on even for a kOvxAll write. It defaults to `scope` -- the rigid callers, where the two
// coincide -- so only the link path need pass its true query scope.
bool planRigidGroup(WriteSession& s,
                    IPhysicsSource& source,
                    PxScene* scene,
                    const std::vector<PxRigidBody*>& bodies,
                    const std::vector<ObjectKey>& keys,
                    const WriteAttributeRow& row,
                    uint32_t scope,
                    bool allocate = true,
                    uint32_t wholeSetScope = kWholeSetScopeFromCache,
                    bool buildOutputGroup = true)
{
    if (bodies.size() != keys.size())
    {
        OVX_RUNTIME_ERROR("ovxWriteAttribute: rigid-body and key lists disagree for '%s' (%zu bodies, %zu keys); "
                   "nothing was planned.",
                   s.attribute.c_str(), bodies.size(), keys.size());
        return false;
    }

    // Kinematic target is INERT on a DirectGPU scene, so refuse it there rather than accept a silent
    // no-op. PhysX leaves PxRigidDynamic::setKinematicTarget unguarded (it has no counterpart in
    // PxRigidDynamicGPUAPIWriteType), so the CPU call succeeds and updates the CPU actor -- but under
    // eENABLE_DIRECT_GPU_API the CPU-integrated kinematic pose is never uploaded to the device, so the
    // body does not move in the device-authoritative simulation (proven: the driven platform's device
    // pose stays put and a resting box is not carried). CPU scenes and GPU scenes with CPU readback
    // drive it normally.
    if (scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API) &&
        (std::string_view(row.token) == OvxAttr::kKinematicTargetPosition ||
         std::string_view(row.token) == OvxAttr::kKinematicTargetOrientation))
    {
        OVX_RUNTIME_ERROR("ovxWriteAttribute: '%s' is not applied on a DirectGPU scene -- a kinematic target is "
                   "integrated on the CPU and not uploaded to the device. Drive the body on a CPU scene "
                   "or a GPU scene with CPU readback.",
                   row.token);
        return false;
    }

    // DirectGPU requires eDISABLE_SLEEPING and does not support this CPU-side wake/sleep command.
    // Preserve the conditional rejection. CPU scenes and GPU scenes with CPU readback support it.
    if (scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API) &&
        std::string_view(row.token) == OvxAttr::kWake)
    {
        OVX_RUNTIME_ERROR("ovxWriteAttribute: '%s' is not applied on a DirectGPU scene -- sleeping is "
                   "disabled under eENABLE_DIRECT_GPU_API. Use this command on a CPU scene or a GPU "
                   "scene with CPU readback.",
                   row.token);
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
    // Prim-set selection: plan only the selected bodies; unselected superset rows are never scattered and
    // stay bit-identical (REQ-READ-SELECT-001 AC-4). Covers the standalone rigid AND articulation-link
    // write paths, which both reach here.
    std::vector<PxRigidBody*> selBodies;
    std::vector<ObjectKey> selKeys;
    const std::vector<PxRigidBody*>* useBodies = &bodies;
    const std::vector<ObjectKey>* useKeys = &keys;
    if (s.hasSelection)
    {
        const std::vector<size_t> keep = ovx::selectedIndices(s, keys);
        selBodies = ovx::gatherByIndices(bodies, keep);
        selKeys = ovx::gatherByIndices(keys, keep);
        useBodies = &selBodies;
        useKeys = &selKeys;
    }

    // A controller proxy is a kinematic dynamic, but rigid-body writes must not drive it:
    // position / velocity / kinematic-target setters desync PxController from its actor.
    // The read reports the prim's pose. The scan reads every body's record, so it only
    // runs when the scene's controller map holds a controller, or when the scene cannot be resolved.
    AttachedStage* cctOwner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(s.attach);
    PhysXScene* cctScene = OmniPhysX::getInstance().getPhysXSetup().findPhysXScene(scene);
    const bool sceneMayHoldControllers =
        !cctScene || !cctScene->getInternalScene() || !cctScene->getInternalScene()->mCctMap.empty();
    const std::vector<omni::physx::internal::InternalDatabase::Record>* cctRecords =
        cctOwner && sceneMayHoldControllers ? &cctOwner->getInternalPhysXDatabase().getRecords() : nullptr;
    std::vector<PxRigidBody*> withoutControllers;
    std::vector<ObjectKey> withoutControllerKeys;
    bool droppedControllers = false;
    if (cctRecords)
    {
        withoutControllers.reserve(useBodies->size());
        withoutControllerKeys.reserve(useKeys->size());
        for (size_t i = 0; i < useBodies->size(); ++i)
        {
            PxRigidBody* body = (*useBodies)[i];
            const size_t id = body ? reinterpret_cast<size_t>(body->userData) : static_cast<size_t>(-1);
            if (id < cctRecords->size() && (*cctRecords)[id].mType == ePTCct)
            {
                auto* cct = static_cast<omni::physx::internal::InternalCct*>((*cctRecords)[id].mInternalPtr);
                if (cct && cct->mActor == body)
                    continue;
            }
            withoutControllers.push_back(body);
            withoutControllerKeys.push_back((*useKeys)[i]);
        }
        // Dropping controller proxies is a subset, same as a selection: storing it in
        // g_rigidReadCache would make the next whole-set rigid read omit those prims.
        droppedControllers = withoutControllers.size() != useBodies->size();
        useBodies = &withoutControllers;
        useKeys = &withoutControllerKeys;
    }

    const ovx::EnabledRigidBodies enabled(*useBodies, *useKeys, filterDisabled);
    const bool anyDisabled = enabled.anyDisabled();
    const std::vector<PxRigidBody*>& outputBodies = enabled.bodies();
    const std::vector<ObjectKey>& outputKeys = enabled.keys();
    const size_t n = outputBodies.size();
    if (n == 0)
        return true; // no group: none matched, or every match is disabled (REQ-INPUT-CORE-001 AC-10)

    std::unique_ptr<GroupSlot> slot(new GroupSlot(s.attach));
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
            OVX_RUNTIME_ERROR("ovxWriteAttribute: DirectGPU scene has no CUDA context; no group emitted.");
            return false;
        }
        slot->ctxMgr = ctxMgr;
        ctxMgr->acquireReference();
        slot->deviceOrdinal = ctxMgr->getDevice();
        PxScopedCudaLock _lock(*ctxMgr);
        allocateColumn(s, *slot, cu, &row, bytes);
        if (!slot->deviceData)
        {
            OVX_RUNTIME_ERROR("ovxWriteAttribute: column allocation failed (%zu floats).", floats);
            return false;
        }
    }
    else if (allocate)
    {
        slot->floats.assign(floats, 0.0f);
    }

    // Keep the cache's structural census intact while membership is temporarily filtered. cacheKey and
    // cacheable feed the scatter's device-row cache (g_rigidReadCache), so they are set even when the
    // output group below is skipped.
    const ovx::RigidReadCacheKey cacheKey{
        scene, static_cast<int>(omni::physx::tensors::RigidBodyType::eRigidDynamic), scope
    };
    // Keep the cache's structural census intact while membership is temporarily filtered: a prim-set
    // selection plans only its subset, so folding that subset into the shared g_rigidReadCache would make
    // a later whole read (or write) see only the selected prims (REQ-READ-SELECT-001 AC-5). The
    // controller filter is the same kind of subset (REQ-READ-CCT-001 AC-3).
    const bool cacheable = (scope == kOvxAll && !anyDisabled && !s.hasSelection && !droppedControllers);
    // wholeSet keys on the query's REAL scope: the link path forces `scope` to kOvxActive for the cache
    // key only. The link-wrench setter skips its host dedup only when this group also names every link
    // record in its view. (For links anyDisabled/droppedControllers are always false, so this reduces to
    // the real scope and the selection flag.)
    const uint32_t effWholeSetScope = (wholeSetScope == kWholeSetScopeFromCache) ? scope : wholeSetScope;
    slot->wholeSet = (effWholeSetScope == kOvxAll && !anyDisabled && !s.hasSelection && !droppedControllers);
    slot->cacheable = cacheable;
    slot->cacheKey = cacheKey;

    // The ovstage output group (the prim-path list + DLTensor + map group) is what the OUTPUT-QUERY and
    // session-commit callers publish. The whole-superset drain (ovstage -> physics) does NOT use it: its
    // scatter re-resolves rows from `slot->bodies` against the backend superset view and writes
    // `slot->deviceData` straight in, never touching slot->list / tensors / group. Building the path
    // list walks every prim and releaseGroupStorage then destroys it -- pure per-drain waste. So the
    // drain passes buildOutputGroup=false and skips it entirely; other callers keep the default.
    if (buildOutputGroup)
    {
        // Canonicalising a key costs a lock plus a cache lookup, and building the list walks every prim,
        // so the handles are reused when the cache already holds them for this key set under the current
        // generation -- the same trade the read makes. `generation` is what says the dictionary they were
        // interned into is still the one in use; the key comparison is what says they describe these
        // prims. Neither alone is enough. A held query's write is planned from the cache's own key list,
        // which needs no comparing.
        ovx_path_dictionary_t*& dict = slot->listDictionary;
        bool built = false;
        if (cacheable)
        {
            omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
            uint64_t generation = 0;
            if (backend && backend->acquireSceneView(scene, &generation))
            {
                ovx::RigidReadCacheEntry& ce = ovx::g_rigidReadCache[cacheKey];
                if (ce.generation != generation || (&ce.keys != &outputKeys && ce.keys != outputKeys) ||
                    ce.handles.size() != outputKeys.size())
                {
                    ce.generation = generation;
                    ce.dbEpoch = omni::physx::internal::recordLifetimeEpoch(scene);
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
        if (slot->list == OVX_INVALID_PRIMPATH_LIST || !dict)
        {
            OVX_RUNTIME_ERROR("ovxWriteAttribute: failed to build ovx_primpath_list_t for '%s' (%zu prims) -- "
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
    }

    s.groups.push_back(std::move(slot));
    return true;
}

// The superset articulation view of `scene` and the view row of each articulation in `artis`. A cached view
// built while an articulation awaited scene insertion does not describe it, and the insertion does not retire
// the view, so a view that misses one of `artis` is rebuilt once, as the read does in acquireArticulationBinding.
bool acquireArticulationRows(omni::physx::tensors::SimulationBackend& backend,
                             PxScene* scene,
                             bool gpu,
                             const std::vector<PxArticulationReducedCoordinate*>& artis,
                             uint64_t& generation,
                             const std::vector<omni::physx::tensors::ArticulationEntry>*& entries,
                             std::vector<uint32_t>& localToRow,
                             omni::physx::tensors::GpuArticulationView** gpuView = nullptr,
                             omni::physx::tensors::CpuArticulationView** cpuView = nullptr)
{
    for (int attempt = 0; attempt < 2; ++attempt)
    {
        if (attempt > 0)
            backend.invalidateSceneCache(scene);
        const std::unordered_map<const PxArticulationReducedCoordinate*, PxU32>* artiRowMap = nullptr;
        entries = nullptr;
        generation = 0;
        if (gpu)
        {
            omni::physx::tensors::GpuSimulationView* sv = backend.acquireSceneView(scene, &generation);
            omni::physx::tensors::GpuArticulationView* view =
                sv ? sv->supersetArticulationView(&artiRowMap, &entries) : nullptr;
            if (!view)
                return false;
            if (gpuView)
                *gpuView = view;
        }
        else
        {
            omni::physx::tensors::CpuSimulationView* sv = backend.acquireCpuSceneView(scene, &generation);
            omni::physx::tensors::CpuArticulationView* view =
                sv ? sv->supersetArticulationView(&artiRowMap, &entries) : nullptr;
            if (!view)
                return false;
            if (cpuView)
                *cpuView = view;
        }
        if (!artiRowMap || !entries)
            return false;

        localToRow.assign(artis.size(), 0xffffffffu);
        bool resolved = true;
        for (size_t i = 0; i < artis.size() && resolved; ++i)
        {
            const std::unordered_map<const PxArticulationReducedCoordinate*, PxU32>::const_iterator it =
                artiRowMap->find(artis[i]);
            resolved = it != artiRowMap->end();
            if (resolved)
                localToRow[i] = it->second;
        }
        if (resolved)
            return true;
    }
    return false;
}

// A prim-set selection over a scene's joints: the slices of `jc` it keeps, their records compacted into the order a
// joint group's column carries them -- otherwise a scatter would apply uninitialised column entries to the joints left
// out -- and the joint key and, when every joint has one, handle of each row. The shared jc cache is never mutated
// (REQ-READ-SELECT-001 AC-4/AC-5), so the joints left out stay bit-identical.
std::vector<size_t> selectJointRows(const WriteSession& s,
                                    const ovx::ArticulationReadCacheEntry& jc,
                                    std::vector<omni::physx::tensors::ArticulationDofOvStageRecord>& recs,
                                    std::vector<ObjectKey>& rowKeys,
                                    std::vector<ovx_primpath_t>& rowHandles)
{
    std::vector<size_t> keep = ovx::selectedIndices(s, jc.sliceKeys);
    const std::vector<ovx::JointSlice> slices = ovx::gatherByIndices(jc.slices, keep);
    for (const ovx::JointSlice& sl : slices)
        recs.insert(recs.end(), jc.recs.begin() + sl.offset, jc.recs.begin() + sl.offset + sl.count);
    rowKeys = ovx::jointRowsPerDof(slices, ovx::gatherByIndices(jc.sliceKeys, keep));
    if (jc.sliceHandles.size() == jc.slices.size())
        rowHandles = ovx::jointRowsPerDof(slices, ovx::gatherByIndices(jc.sliceHandles, keep));
    return keep;
}

// One scene's joint group: a FIXED group, one tensor over the flat DOF column the caller fills, one row per DOF,
// each row's prim the joint the DOF belongs to (ADR-0059). Mirrors the joint read's emit exactly, because a caller
// reading a joint column and writing it straight back must see the same shape in both directions.
//
// The records and slices come from the shared cache (ensureJointRecords), so this derives nothing the
// read does not already derive.
bool planJointGroup(WriteSession& s,
                    IPhysicsSource& source,
                    PxScene* scene,
                    const JointWriteAttributeRow& row)
{
    InternalScene* owningScene = ovx::internalSceneOf(scene);
    if (!owningScene)
        return false;
    AttachedStage& owner = owningScene->getAttachedStage();
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
        return false;

    const bool gpu = scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API);

    // The write RUNS the structural walk rather than requiring a read to have run first. It is the
    // read's walk, shared -- so the two directions still enumerate one joint set -- but a write API
    // has no business demanding the caller issue a read to make it work. Idempotent and epoch-gated,
    // so this costs an epoch comparison once the cache is warm.
    if (!ovx::refreshArticulationCache(owner, ovx::allPhysicsScenes(owner)))
    {
        OVX_RUNTIME_ERROR("ovxWriteAttribute: the articulation structural walk failed; nothing was written.");
        return false;
    }
    ovx::ArticulationReadCacheEntry* jcPtr = ovx::articulationCacheEntry(scene);
    if (!jcPtr || jcPtr->joints.empty())
        return true; // this scene owns no joints -- not a failure
    ovx::ArticulationReadCacheEntry& jc = *jcPtr;
    const bool enumCached = true; // just refreshed above, by definition

    uint64_t generation = 0;
    const std::vector<omni::physx::tensors::ArticulationEntry>* entriesPtr = nullptr;
    std::vector<uint32_t> localToRow;
    if (!acquireArticulationRows(*backend, scene, gpu, jc.artis, generation, entriesPtr, localToRow))
        return false;
    if (!ovx::ensureJointRecords(jc, source, jc.joints, *entriesPtr, localToRow, generation, enumCached))
        return false;
    if (jc.slices.empty())
        return true;

    std::vector<omni::physx::tensors::ArticulationDofOvStageRecord> selRecs;
    std::vector<ObjectKey> selRowKeys;
    std::vector<ovx_primpath_t> selRowHandles;
    const std::vector<omni::physx::tensors::ArticulationDofOvStageRecord>* jRecs = &jc.recs;
    // Each row's joint, which makes the group's prim list.
    const std::vector<ObjectKey>* rowKeys = &jc.dofKeys;
    const std::vector<ovx_primpath_t>* rowHandles = &jc.dofHandles;
    uint64_t recsVersion = jc.recsVersion;
    if (s.hasSelection)
    {
        const std::vector<size_t> keep = selectJointRows(s, jc, selRecs, selRowKeys, selRowHandles);
        if (selRecs.empty())
            return true; // none of this scene's joints are in the selection
        recsVersion = selectedRecordsVersion(jc.recsVersion, keep);
        jRecs = &selRecs;
        rowKeys = &selRowKeys;
        rowHandles = &selRowHandles;
    }

    const uint32_t numOut = static_cast<uint32_t>(jRecs->size());
    std::unique_ptr<GroupSlot> slot(new GroupSlot(s.attach));
    slot->scene = scene;
    slot->jointRow = &row;
    slot->numOut = numOut;

    if (gpu)
    {
        PxCudaContextManager* ctxMgr = scene->getCudaContextManager();
        PxCudaContext* cu = ctxMgr ? ctxMgr->getCudaContext() : nullptr;
        if (!cu)
            return false;
        slot->ctxMgr = ctxMgr;
        ctxMgr->acquireReference();
        slot->deviceOrdinal = ctxMgr->getDevice();
        PxScopedCudaLock _lock(*ctxMgr);
        allocateColumn(s, *slot, cu, &row, size_t(numOut) * sizeof(float));
        // A reused column brings its uploaded records along. They stay valid while they hold the current contents
        // of the shared records, or of the selection's slices of them.
        const size_t recsBytes = size_t(numOut) * sizeof(omni::physx::tensors::ArticulationDofOvStageRecord);
        // The DOF column (allocateColumn, above) already marks t_devicePrepFailed on OOM; the records buffer and
        // its upload are the joint path's other transient device-prep steps, so a failure of either is retryable
        // too, not a NotHandled fall-back to the inert-on-DirectGPU parse path (AC-12).
        if (!slot->recsDev && cu->memAlloc(&slot->recsDev, recsBytes) != 0)
            t_devicePrepFailed = true;
        if (!slot->deviceData || !slot->recsDev)
            return false;
        if (recsVersion == 0 || slot->recsVersion != recsVersion)
        {
            slot->recsVersion = 0; // current again only once the copy has landed
            if (!uploadRecords(*slot, slot->recsDev, jRecs->data(), recsBytes))
            {
                t_devicePrepFailed = true; // transient record upload (H2D) failure -> retryable
                OVX_RUNTIME_ERROR("ovxWriteAttribute: joint record upload failed; no group emitted.");
                return false;
            }
            slot->recsVersion = recsVersion;
        }
    }
    else
    {
        slot->floats.assign(numOut, 0.0f);
        // See recsHost: the CPU scatter re-derives the shared cache by default, so hand it the compacted
        // records when a selection filtered them.
        if (s.hasSelection)
            slot->recsHost = selRecs;
    }

    ovx_path_dictionary_t*& dict = slot->listDictionary;
    const bool haveHandles = rowHandles->size() == rowKeys->size();
    slot->list = haveHandles ? omni::physics::ovstage::buildPathListFromHandles(source, rowHandles->data(),
                                                                               rowHandles->size(), &dict) :
                               omni::physics::ovstage::buildPathList(source, rowKeys->data(), rowKeys->size(), &dict);
    if (slot->list == OVX_INVALID_PRIMPATH_LIST || !dict)
        return false;
    s.dict = dict;

    slot->shapes.assign(1, static_cast<int64_t>(numOut));
    slot->tensors.assign(1, DLTensor{});
    DLTensor& t = slot->tensors[0];
    t.data = gpu ? reinterpret_cast<void*>(slot->deviceData) : static_cast<void*>(slot->floats.data());
    t.device = gpu ? DLDevice{ kDLCUDA, slot->deviceOrdinal } : DLDevice{ kDLCPU, 0 };
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
                           const std::vector<ovx_primpath_t>& handles,
                           const ArtiWriteAttributeRow& row)
{
    const size_t n = artis.size();
    if (n == 0)
        return true; // this scene owns no matching articulations -- not a failure

    std::unique_ptr<GroupSlot> slot(new GroupSlot(s.attach));
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
            OVX_RUNTIME_ERROR("ovxWriteAttribute: DirectGPU scene has no CUDA context; no group emitted.");
            return false;
        }
        slot->ctxMgr = ctxMgr;
        ctxMgr->acquireReference();
        slot->deviceOrdinal = ctxMgr->getDevice();
        PxScopedCudaLock _lock(*ctxMgr);
        allocateColumn(s, *slot, cu, &row, bytes);
        if (!slot->deviceData)
        {
            OVX_RUNTIME_ERROR("ovxWriteAttribute: articulation column allocation failed (%zu bytes).", bytes);
            return false;
        }
    }
    else
    {
        slot->floats.assign(n * size_t(row.components), 0.0f);
    }

    // `handles` are the read's canonical root handles for `keys`, when the source had them; from them the list
    // skips canonicalising every key again.
    ovx_path_dictionary_t*& dict = slot->listDictionary;
    slot->list = handles.size() == keys.size() ?
                     omni::physics::ovstage::buildPathListFromHandles(source, handles.data(), handles.size(), &dict) :
                     omni::physics::ovstage::buildPathList(source, keys.data(), keys.size(), &dict);
    if (slot->list == OVX_INVALID_PRIMPATH_LIST || !dict)
    {
        OVX_RUNTIME_ERROR("ovxWriteAttribute: failed to build ovx_primpath_list_t for '%s' (%zu articulations) -- "
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
    //
    // The entry is shared with the read, so it is rebuilt and claimed by the read's rules: only when the view
    // build moved (a new tendon set retires `generation`), retired first so a failure part-way leaves nothing
    // claimed, handles rebuilt with the keys, and a fresh content version for every device upload keyed on it.
    if (tc.generation != generation || generation == 0)
    {
        tc.generation = 0;
        std::vector<uint32_t> localToRow;
        if (!acquireArticulationRows(*backend, scene, gpu, tc.artis, generation, entriesPtr, localToRow))
            return false;
        tc.recs.clear();
        tc.keys.clear();
        tc.handles.clear();
        tc.recs.reserve(tc.tendons.size());
        tc.keys.reserve(tc.tendons.size());
        for (const ovx::TendonRec& tr : tc.tendons)
        {
            if (tr.viewArtiIdx >= tc.artis.size())
                return false;
            omni::physx::tensors::ArticulationTendonOvStageRecord r;
            r.viewArtiIdx = localToRow[tr.viewArtiIdx];
            r.tendonIdx = tr.tendonIdx;
            tc.recs.push_back(r);
            tc.keys.push_back(tr.key);
        }
        omni::physics::ovstage::canonicalisePathHandles(source, tc.keys.data(), tc.keys.size(), tc.handles, nullptr);
        tc.generation = generation;
        tc.recsVersion = ovx::mintRecordContentVersion();
    }

    // Prim-set selection compacts the record list to the admitted tendons. The shared cache (tc.recs /
    // tc.keys / tc.handles) stays FULL -- folding a subset into it would corrupt a later whole-set read
    // or write over the same scene (REQ-READ-SELECT-001 AC-5). The compacted records ride on the slot:
    // recsDev for the GPU scatter, tendonRecsHost for the host one, both indexed [0, numOut) against the
    // caller's compacted column.
    std::vector<omni::physx::tensors::ArticulationTendonOvStageRecord> selRecs;
    std::vector<ObjectKey> selKeys;
    std::vector<ovx_primpath_t> selHandles;
    const bool cacheHaveHandles = tc.handles.size() == tc.keys.size();
    uint64_t recsVersion = tc.recsVersion;
    if (s.hasSelection)
    {
        const std::vector<size_t> keep = ovx::selectedIndices(s, tc.keys);
        recsVersion = selectedRecordsVersion(tc.recsVersion, keep);
        selRecs = ovx::gatherByIndices(tc.recs, keep);
        selKeys = ovx::gatherByIndices(tc.keys, keep);
        if (cacheHaveHandles)
            selHandles = ovx::gatherByIndices(tc.handles, keep);
    }
    const std::vector<omni::physx::tensors::ArticulationTendonOvStageRecord>& recsUse =
        s.hasSelection ? selRecs : tc.recs;
    const std::vector<ObjectKey>& keysUse = s.hasSelection ? selKeys : tc.keys;
    const std::vector<ovx_primpath_t>& handlesUse = s.hasSelection ? selHandles : tc.handles;

    const uint32_t numOut = static_cast<uint32_t>(recsUse.size());
    if (numOut == 0)
        return true; // a selection that admits none of this scene's tendons emits no group -- not a failure
    const uint32_t comp = omni::physx::tensors::tendonPropertyComponents(row.prop);

    std::unique_ptr<GroupSlot> slot(new GroupSlot(s.attach));
    slot->scene = scene;
    slot->tendonRow = &row;
    slot->tendonFixed = fixed;
    slot->numOut = numOut;
    slot->comp = comp;
    slot->keys = keysUse;
    if (s.hasSelection)
        slot->tendonRecsHost = selRecs; // the host scatter applies exactly these, not the full cache

    const size_t bytes = size_t(numOut) * comp * sizeof(float);
    if (gpu)
    {
        PxCudaContextManager* ctxMgr = scene->getCudaContextManager();
        PxCudaContext* cu = ctxMgr ? ctxMgr->getCudaContext() : nullptr;
        if (!cu)
            return false;
        slot->ctxMgr = ctxMgr;
        ctxMgr->acquireReference();
        slot->deviceOrdinal = ctxMgr->getDevice();
        PxScopedCudaLock _lock(*ctxMgr);
        allocateColumn(s, *slot, cu, &row, bytes, fixed ? 1 : 0);
        const size_t recsBytes = size_t(numOut) * sizeof(omni::physx::tensors::ArticulationTendonOvStageRecord);
        if (!slot->recsDev)
            cu->memAlloc(&slot->recsDev, recsBytes);
        if (!slot->deviceData || !slot->recsDev)
            return false;
        if (recsVersion == 0 || slot->recsVersion != recsVersion)
        {
            slot->recsVersion = 0; // current again only once the copy has landed
            if (!uploadRecords(*slot, slot->recsDev, recsUse.data(), recsBytes))
            {
                OVX_RUNTIME_ERROR("ovxWriteAttribute: tendon record upload failed; no group emitted.");
                return false;
            }
            slot->recsVersion = recsVersion;
        }
    }
    else
    {
        slot->floats.assign(size_t(numOut) * comp, 0.0f);
    }

    ovx_path_dictionary_t*& dict = slot->listDictionary;
    const bool haveHandles = handlesUse.size() == keysUse.size();
    slot->list = haveHandles ?
                     omni::physics::ovstage::buildPathListFromHandles(source, handlesUse.data(),
                                                                     handlesUse.size(), &dict) :
                     omni::physics::ovstage::buildPathList(source, keysUse.data(), keysUse.size(), &dict);
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

// One CPU scalar column per scene, with immutable row identity until commit.
bool planVehicleDataGroup(WriteSession& s, IPhysicsSource& source, PxScene* scene,
                          const VehicleWriteAttributeRow* wheel,
                          const VehicleControlAttributeRow* vehicle)
{
    omni::physx::tensors::BaseVehicleView* view = nullptr;
    ovx::VehicleReadCacheEntry* cache = ovx::refreshVehicleCache(source, scene, &view);
    if (!cache)
    {
        OVX_RUNTIME_ERROR("ovxWriteAttributes: vehicle host view unavailable; nothing was opened.");
        return false;
    }
    const std::vector<ObjectKey>& keys = vehicle ? cache->vehicleKeys : cache->keys;
    const std::vector<ovx_primpath_t>& handles = vehicle ? cache->vehicleHandles : cache->handles;
    const std::vector<size_t> selected = ovx::selectedIndices(s, keys);
    if (selected.empty())
        return true;
    if (!view)
        return false;
    using VehicleControl = omni::physx::tensors::BaseVehicleView::VehicleControl;
    const std::vector<omni::physx::tensors::VehicleEntry>& entries = view->getEntries();
    for (size_t index : selected)
    {
        const uint32_t row = vehicle ? static_cast<uint32_t>(index) : cache->recs[index].viewVehicleIdx;
        const omni::physx::internal::InternalVehicle& target = *entries[row].vehicle;
        const bool compatible = wheel ? !target.hasDrive() :
            vehicle->control == VehicleControl::eResetToRestState ? true :
            (vehicle->control == VehicleControl::eThrust0 || vehicle->control == VehicleControl::eThrust1) ?
                target.isTank() : target.hasDrive();
        if (!compatible)
        {
            OVX_RUNTIME_ERROR("ovxWriteAttributes: '%s' is incompatible with a selected vehicle; nothing was opened.",
                              wheel ? wheel->token : vehicle->token);
            return false;
        }
    }
    std::unique_ptr<GroupSlot> slot(new GroupSlot(s.attach));
    slot->scene = scene;
    slot->vehicleRow = wheel;
    slot->vehicleControlRow = vehicle;
    slot->vehicleSetEpoch = cache->setEpoch;
    slot->vehicleGeneration = cache->generation;
    slot->numOut = static_cast<uint32_t>(selected.size());
    slot->comp = 1;
    slot->keys = ovx::gatherByIndices(keys, selected);
    if (vehicle)
        slot->wholeVehicleRecs.assign(selected.begin(), selected.end());
    else
        slot->vehicleRecsHost = ovx::gatherByIndices(cache->recs, selected);

    ovx_path_dictionary_t*& dict = slot->listDictionary;
    if (handles.size() == keys.size())
    {
        const std::vector<ovx_primpath_t> selectedHandles = ovx::gatherByIndices(handles, selected);
        slot->list = omni::physics::ovstage::buildPathListFromHandles(source, selectedHandles.data(),
                                                                     selectedHandles.size(), &dict);
    }
    else
        slot->list = omni::physics::ovstage::buildPathList(source, slot->keys.data(), slot->keys.size(), &dict);
    if (slot->list == OVX_INVALID_PRIMPATH_LIST || !dict)
        return false;
    s.dict = dict;
    slot->shapes.assign(1, static_cast<int64_t>(slot->numOut));
    slot->tensors.assign(1, DLTensor{});
    DLTensor& tensor = slot->tensors[0];
    tensor.dtype = { kDLFloat, 32, 1 };
    if (vehicle && vehicle->dtype == omni::physics::tensors::TensorDataType::eInt32)
    {
        slot->vehicleGears.assign(slot->numOut, 0);
        tensor.data = slot->vehicleGears.data();
        tensor.dtype = { kDLInt, 32, 1 };
    }
    else if (vehicle && vehicle->dtype == omni::physics::tensors::TensorDataType::eUint8)
    {
        slot->vehicleCommands.assign(slot->numOut, 0);
        tensor.data = slot->vehicleCommands.data();
        tensor.dtype = { kDLUInt, 8, 1 };
    }
    else
    {
        slot->floats.assign(slot->numOut, 0.0f);
        tensor.data = slot->floats.data();
    }
    tensor.device = { kDLCPU, 0 };
    tensor.ndim = 1;
    tensor.shape = slot->shapes.data();
    ovstage_map_group_t& group = slot->group;
    group.prims.list = slot->list;
    group.prims.count = slot->numOut;
    group.data.tensors = slot->tensors.data();
    group.data.tensor_count = 1;
    s.groups.push_back(std::move(slot));
    return true;
}

// PxController::move drives the proxy with setKinematicTarget. That CPU target is not
// uploaded on a DirectGPU scene, so the write is refused before a group exists.
bool cctMoveRejectedOnDirectGpu(const std::vector<omni::physx::ovx::CctOvStageRow>& rows, const char* api)
{
    for (const omni::physx::ovx::CctOvStageRow& cctRow : rows)
    {
        const PxRigidActor* actor = cctRow.internal ? cctRow.internal->mActor : nullptr;
        const PxScene* scene = actor ? actor->getScene() : nullptr;
        if (scene && scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API))
        {
            OVX_RUNTIME_ERROR("%s: 'move' is not applied on a DirectGPU scene -- "
                              "PxController::move updates its proxy through setKinematicTarget, and that "
                              "CPU target is not uploaded under eENABLE_DIRECT_GPU_API.",
                              api);
            return true;
        }
    }
    return false;
}

// One fixed host group covering the in-scope character controllers this session admits.
bool planCctGroup(WriteSession& s, AttachedStage& owner, IPhysicsSource& source, uint32_t scope,
                  const CctWriteAttributeRow& row)
{
    std::vector<omni::physx::ovx::CctOvStageRow> rows;
    if (!omni::physx::ovx::enumerateCctRows(owner, scope, rows))
        return false;
    if (s.hasSelection)
    {
        std::vector<omni::physx::ovx::CctOvStageRow> admitted;
        admitted.reserve(rows.size());
        for (const omni::physx::ovx::CctOvStageRow& cctRow : rows)
            if (s.admits(cctRow.key))
                admitted.push_back(cctRow);
        rows.swap(admitted);
    }
    if (rows.empty())
        return true;
    if (cctMoveRejectedOnDirectGpu(rows, "ovxWriteAttribute"))
        return false;

    const uint32_t numOut = static_cast<uint32_t>(rows.size());
    std::unique_ptr<GroupSlot> slot(new GroupSlot(s.attach));
    slot->cctRow = &row;
    slot->numOut = numOut;
    slot->comp = row.components;
    slot->keys.reserve(numOut);
    for (const omni::physx::ovx::CctOvStageRow& cctRow : rows)
        slot->keys.push_back(cctRow.key);

    slot->floats.assign(size_t(numOut) * row.components, 0.0f);

    ovx_path_dictionary_t*& dict = slot->listDictionary;
    slot->list = omni::physics::ovstage::buildPathList(source, slot->keys.data(), slot->keys.size(), &dict);
    if (slot->list == OVX_INVALID_PRIMPATH_LIST || !dict)
        return false;
    s.dict = dict;

    slot->shapes.assign(1, static_cast<int64_t>(numOut));
    slot->tensors.assign(1, DLTensor{});
    DLTensor& t = slot->tensors[0];
    t.data = static_cast<void*>(slot->floats.data());
    t.device = DLDevice{ kDLCPU, 0 };
    t.ndim = 1;
    t.dtype = DLDataType{ kDLFloat, 32, static_cast<uint16_t>(row.components) };
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
                        uint64_t viewGeneration,
                        const ObjectKey& key,
                        uint32_t arrayLength,
                        uint32_t instancerIdx,
                        omni::physx::tensors::BasePointInstancerView::InstancerColumn column,
                        uint32_t comp,
                        const WriteAttributeRow& row)
{
    if (arrayLength == 0)
        return true;

    std::unique_ptr<GroupSlot> slot(new GroupSlot(s.attach));
    slot->isInstancer = true;
    slot->scene = scene;
    slot->instancerView = view;
    slot->instancerGeneration = viewGeneration;
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
            OVX_RUNTIME_ERROR("ovxWriteAttribute: DirectGPU scene has no CUDA context; no instancer group.");
            return false;
        }
        slot->ctxMgr = ctxMgr;
        ctxMgr->acquireReference();
        slot->deviceOrdinal = ctxMgr->getDevice();
        PxScopedCudaLock _lock(*ctxMgr);
        // Keyed by the instancer's own identity: the view is rebuilt, at a new address, on a record-epoch change.
        allocateColumn(s, *slot, cu, &row, bytes, key.handle);
        if (!slot->deviceData)
        {
            OVX_RUNTIME_ERROR("ovxWriteAttribute: instancer column allocation failed (%zu bytes).", bytes);
            return false;
        }
    }
    else
    {
        slot->floats.assign(size_t(arrayLength) * comp, 0.0f);
    }

    ovx_path_dictionary_t*& dict = slot->listDictionary;
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
                         bool velocity,
                         bool kinematicTarget = false)
{
    if (targets.bodies.empty())
        return true;

    PxCudaContextManager* ctxMgr = scene ? scene->getCudaContextManager() : nullptr;
    PxCudaContext* cu = ctxMgr ? ctxMgr->getCudaContext() : nullptr;
    if (!cu)
    {
        OVX_RUNTIME_ERROR("ovxWriteAttribute: deformable write needs a CUDA context -- none available.");
        return false;
    }

    std::unique_ptr<GroupSlot> slot(new GroupSlot(s.attach));
    slot->isDeformable = true;
    slot->deformableVolume = isVolume;
    slot->deformableVelocity = velocity;
    slot->deformableKinematicTarget = kinematicTarget;
    slot->scene = scene;
    // Kinematic target is a PxVec4 per node (xyz + w active-flag); points/velocities are vec3.
    const uint32_t comp = kinematicTarget ? 4u : 3u;
    slot->comp = comp;

    // The column is COMPACT and the meshes are ragged, so each body's slice starts where the last
    // one ended -- the same layout the read's gather writes into, and the reason a tensor carries a
    // byte offset rather than every body getting a buffer.
    size_t totalFloats = 0;
    for (size_t i = 0; i < targets.bodies.size(); ++i)
    {
        if (!targets.bodies[i] || !targets.internals[i] || targets.counts[i] == 0)
            continue;
        if (s.hasSelection && !s.admits(targets.keys[i]))
            continue; // prim-set selection: leave unselected deformable bodies out of the group
        slot->deformableBodies.push_back(targets.bodies[i]);
        slot->deformableInternals.push_back(targets.internals[i]);
        slot->deformableCounts.push_back(targets.counts[i]);
        slot->deformableFloatOffsets.push_back(totalFloats);
        slot->keys.push_back(targets.keys[i]);
        totalFloats += size_t(targets.counts[i]) * comp;
    }
    if (slot->deformableBodies.empty())
        return true;
    slot->numOut = static_cast<uint32_t>(slot->deformableBodies.size());

    slot->ctxMgr = ctxMgr;
    ctxMgr->acquireReference();
    slot->deviceOrdinal = ctxMgr->getDevice();
    {
        PxScopedCudaLock _lock(*ctxMgr);
        const uint64_t use = ovx::QueryDeviceBuffers::kDeformableColumn | (isVolume ? 1u : 0u) | (velocity ? 2u : 0u) |
                             (kinematicTarget ? 4u : 0u);
        allocateColumn(s, *slot, cu, nullptr, totalFloats * sizeof(float), use);
    }
    if (!slot->deviceData)
    {
        OVX_RUNTIME_ERROR("ovxWriteAttribute: deformable column allocation failed (%zu floats).", totalFloats);
        return false;
    }

    ovx_path_dictionary_t*& dict = slot->listDictionary;
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
        t.dtype = DLDataType{ kDLFloat, 32, static_cast<uint16_t>(comp) };
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
bool scatterDeformableGroup(GroupSlot& slot)
{
    if (!slot.ctxMgr || slot.deformableBodies.size() != slot.numOut)
        return false;

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
            OVX_RUNTIME_ERROR("ovxCommitGroup: deformable body is not the kind the query selected; nothing "
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
            OVX_RUNTIME_ERROR("ovxCommitGroup: deformable body has no device buffer for this column.");
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
            OVX_RUNTIME_ERROR("ovxCommitGroup: deformable column scatter failed.");
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

// Where a kinematic-target group's scratch keeps what its commit reads, laid out at open
// (prepareKinematicTargetGroup): one record per body, then, for a commit applied on the host, its row mask and the
// count of pinned nodes it kept pinned.
struct KinematicTargetScratchLayout
{
    explicit KinematicTargetScratchLayout(uint32_t bodies)
        : recordBytes(size_t(bodies) * sizeof(omni::physx::tensors::KinematicTargetBodyRecord)),
          mask(omni::physx::tensors::OvStageScratch::alignUp(recordBytes)),
          keptPinned(mask + omni::physx::tensors::OvStageScratch::alignUp(bodies)),
          bytes(keptPinned + sizeof(unsigned long long))
    {
    }
    size_t recordBytes;
    size_t mask;
    size_t keptPinned;
    size_t bytes;
};

// Whether another path than this write drives `volume`'s kinematic targets (REQ-INPUT-COVERAGE-001 AC-16): a
// whole-body kinematic volume (eKINEMATIC, kinematicBody=true), which this write never raises, or
// ePARTIALLY_KINEMATIC raised by something else, such as the tensor binding's GpuVolumeDeformableBodyView. The
// flags and the adopted buffer are one per-body slot, which this write must not take over.
bool kinematicTargetDrivenElsewhere(const PxDeformableVolume& volume,
                                    const omni::physx::internal::InternalVolumeDeformableBody& internal)
{
    return volume.getDeformableBodyFlags().isSet(PxDeformableBodyFlag::eKINEMATIC) ||
           (!internal.mKinematicTargetActive &&
            volume.getDeformableVolumeFlags().isSet(PxDeformableVolumeFlag::ePARTIALLY_KINEMATIC));
}

// The kinematic-target commit (volume only). Each body was set up when the session opened
// (prepareKinematicTargetGroup): a runtime-owned buffer PhysX adopted, every node free, ePARTIALLY_KINEMATIC raised.
// The commit is one kernel over every body, from the caller's column into those buffers, which PhysX re-reads at
// each step: no allocation, no host call into PhysX and nothing read back.
//
// Pinning is monotonic: PhysX pins a node by zeroing its inverse mass and never restores it, so a released node
// would stay frozen. The kernel keeps a pinned node the column marks free at its last target and counts it; the
// count is logged the next time ovphysx looks, failing no call.
//
// On a DirectGPU scene the commit is ordered on the caller's stream, with `deviceArgs`, and can be captured into a
// CUDA graph; nothing sleeps there. Its kernel runs on the scene's deformable copy stream, forked from the caller's
// stream and joined back like a PhysX call: the commits of every session through the scene land in call order, and
// the step waits for that stream before simulating. Elsewhere it runs on the legacy default stream, reads the count
// back at once, and wakes the bodies it wrote: PhysX skips a sleeping body, and setting a target does not wake it.
bool scatterKinematicTargetGroup(GroupSlot& slot,
                                 const omni::physx::tensors::OvStageDeviceWriteArgs* deviceArgs,
                                 const uint8_t* hostMask)
{
    if (!slot.ctxMgr || slot.deformableBodies.size() != slot.numOut)
        return false;
    if (slot.numOut == 0)
        return true; // a masked commit that selects no body writes nothing

    // Before anything is written: a body another path drives refuses the whole commit (AC-16 point 5).
    for (uint32_t i = 0; i < slot.numOut; ++i)
    {
        PxDeformableBody* body = slot.deformableBodies[i];
        omni::physx::internal::InternalVolumeDeformableBody* internal =
            static_cast<omni::physx::internal::InternalVolumeDeformableBody*>(slot.deformableInternals[i]);
        PxDeformableVolume* volume = body ? body->is<PxDeformableVolume>() : nullptr;
        if (!body || !internal || !volume)
        {
            OVX_RUNTIME_ERROR("ovxCommitGroup: kinematic target is a volume-deformable write; body is not one.");
            return false;
        }
        if (volume->getDeformableBodyFlags().isSet(PxDeformableBodyFlag::eKINEMATIC))
        {
            OVX_RUNTIME_ERROR("ovxCommitGroup: a volume deformable in this kinematic-target write is authored "
                              "fully kinematic (PxDeformableBodyFlag::eKINEMATIC, i.e. kinematicBody=true): PhysX "
                              "already treats every node as a target, so this per-node write would add "
                              "ePARTIALLY_KINEMATIC and swap the adopted target buffer over a body another path "
                              "already drives. Refused; nothing was written.");
            return false;
        }
        if (kinematicTargetDrivenElsewhere(*volume, *internal))
        {
            OVX_RUNTIME_ERROR("ovxCommitGroup: a volume deformable in this kinematic-target write is already "
                              "kinematically driven through another path (ePARTIALLY_KINEMATIC is raised, e.g. by "
                              "a GpuVolumeDeformableBodyView from the tensor binding). The two share one per-body "
                              "target slot, so this write refuses to take it over rather than clobber the other "
                              "drive. Nothing was written.");
            return false;
        }
    }

    PxCudaContext* const cu = slot.ctxMgr->getCudaContext();
    if (!cu || !slot.scratchDev)
        return false;
    PxScopedCudaLock lock(*slot.ctxMgr);
    const KinematicTargetScratchLayout layout(slot.numOut);
    const omni::physx::tensors::KinematicTargetBodyRecord* const records =
        reinterpret_cast<const omni::physx::tensors::KinematicTargetBodyRecord*>(slot.scratchDev);
    const float* const column = reinterpret_cast<const float*>(slot.deviceData);

    if (deviceArgs)
    {
        // Allocated when the session opened, so a captured commit allocates nothing here.
        const omni::physx::tensors::GpuSimulationDataPtr gpuData = sceneGpuData(slot);
        unsigned long long* const keptPinned = gpuData ? gpuData->keptPinnedTargetsCounter() : nullptr;
        PhysXScene* const physxScene = OmniPhysX::getInstance().getPhysXSetup().findPhysXScene(slot.scene);
        const CUstream copy = physxScene ? physxScene->getInternalScene()->getDeformableCopyStream() : nullptr;
        if (!keptPinned || !copy)
        {
            OVX_RUNTIME_ERROR("ovxCommitGroup: the kinematic-target commit has no kept-pinned count or no deformable "
                              "copy stream; nothing was written.");
            return false;
        }
        gpuData->reportKeptPinnedTargets();
        return omni::physx::tensors::physxOnStream(
            *deviceArgs,
            [&](CUevent fork, CUevent join)
            {
                IOptionalCuda* const cuda = getCudaShim();
                int status = 0;
                const bool ordered = cuda->streamWaitEvent(
                    reinterpret_cast<uintptr_t>(copy), reinterpret_cast<uintptr_t>(fork), 0, &status);
                if (!ordered)
                    OVX_RUNTIME_ERROR("ovxCommitGroup: the kinematic-target commit could not be ordered after the "
                                      "caller's stream (CUresult %d); nothing was written.",
                                      status);
                const bool submitted =
                    ordered && omni::physx::tensors::submitKinematicTargetsOvStage(
                                   records, slot.numOut, slot.kinematicTargetMaxNodes, column, deviceArgs->mask,
                                   keptPinned, copy);
                // Recorded even when nothing was launched, so the caller's stream never waits on an earlier commit's.
                if (!cuda->eventRecord(reinterpret_cast<uintptr_t>(join), reinterpret_cast<uintptr_t>(copy), &status))
                {
                    OVX_RUNTIME_ERROR("ovxCommitGroup: the kinematic-target commit's completion could not be recorded "
                                      "(CUresult %d).",
                                      status);
                    if (deviceArgs->unjoined)
                        *deviceArgs->unjoined = true;
                    return false;
                }
                return submitted;
            });
    }

    unsigned long long* const keptPinned =
        reinterpret_cast<unsigned long long*>(slot.scratchDev + layout.keptPinned);
    const CUdeviceptr maskDev = slot.scratchDev + layout.mask;
    // Checked: a lost mask writes rows the caller left out, and a lost count reports pins that were not kept.
    if ((hostMask && cu->memcpyHtoD(maskDev, hostMask, slot.numOut) != CUDA_SUCCESS) ||
        cu->memsetD32(reinterpret_cast<CUdeviceptr>(keptPinned), 0, 2) != CUDA_SUCCESS)
    {
        OVX_RUNTIME_ERROR("ovxCommitGroup: the kinematic-target commit could not be prepared; nothing was written.");
        return false;
    }
    unsigned long long kept = 0;
    if (!omni::physx::tensors::submitKinematicTargetsOvStage(
            records, slot.numOut, slot.kinematicTargetMaxNodes, column,
            hostMask ? reinterpret_cast<const uint8_t*>(maskDev) : nullptr, keptPinned, nullptr) ||
        cu->memcpyDtoH(&kept, reinterpret_cast<CUdeviceptr>(keptPinned), sizeof(kept)) != CUDA_SUCCESS)
    {
        OVX_RUNTIME_ERROR("ovxCommitGroup: kinematic-target scatter failed.");
        return false;
    }
    omni::physx::tensors::logKeptPinnedTargets(kept);
    if (slot.scene && slot.scene->getFlags().isSet(PxSceneFlag::eDISABLE_SLEEPING))
        return true;
    for (uint32_t i = 0; i < slot.numOut; ++i)
    {
        if (hostMask && !hostMask[i])
            continue;
        PxDeformableVolume* const volume = slot.deformableBodies[i]->is<PxDeformableVolume>();
        if (PxScene* const scene = volume->getScene())
            volume->setWakeCounter(scene->getWakeCounterResetValue());
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
    ovx::SourceContext context;
    if (!ovx::getSourceContext(s.attach, "material write", context))
        return false;
    std::vector<ObjectKey> keys;
    std::vector<PxDeformableMaterial*> materials;

    omni::physx::internal::InternalPhysXDatabase& db =
        context.stage->getInternalPhysXDatabase();
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
        if (s.hasSelection && !s.admits(rec.mKey))
            continue; // prim-set selection: only admitted materials take a row
        keys.push_back(rec.mKey);
        materials.push_back(material);
    }
    if (keys.empty())
        return true; // nothing this attribute can serve -- not a failure

    const uint32_t numOut = static_cast<uint32_t>(keys.size());

    std::unique_ptr<GroupSlot> slot(new GroupSlot(s.attach));
    slot->deformableMatRow = &row;
    slot->materials = std::move(materials);
    slot->keys = keys;
    slot->numOut = numOut;
    slot->comp = 1u;
    slot->floats.assign(size_t(numOut), 0.0f);

    ovx_path_dictionary_t*& dict = slot->listDictionary;
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
bool scatterDeformableMaterialGroup(GroupSlot& slot)
{
    if (!slot.deformableMatRow || slot.materials.size() != slot.numOut)
        return false;
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

    std::unique_ptr<GroupSlot> slot(new GroupSlot(s.attach));
    slot->isParticle = true;
    slot->particleVelocity = velocity;
    slot->scene = scene;
    slot->comp = 3u;

    size_t totalFloats = 0;
    for (size_t i = 0; i < targets.sets.size(); ++i)
    {
        if (!targets.sets[i] || targets.counts[i] == 0)
            continue;
        if (s.hasSelection && !s.admits(targets.keys[i]))
            continue; // prim-set selection: leave unselected particle sets out of the group
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

    ovx_path_dictionary_t*& dict = slot->listDictionary;
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

// Publish a particle column into each set's staging array (ADR-0012).
bool scatterParticleGroup(GroupSlot& slot)
{
    if (slot.particleSets.size() != slot.numOut)
        return false;

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
            OVX_RUNTIME_ERROR("ovxCommitGroup: particle set has no staging buffer for '%s'.",
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

// One scene's joint-PROPERTY group. Same shape as the DOF-state group -- one tensor over a flat column, one row
// per DOF (ADR-0059) -- because it names the same slots. What differs is residency: these are simulation inputs
// with no device copy, so the column is HOST even on a DirectGPU scene, exactly as the rigid-body property columns
// are.
bool planJointPropertyGroup(WriteSession& s,
                            IPhysicsSource& source,
                            PxScene* scene,
                            const JointPropertyWriteRow& row)
{
    InternalScene* owningScene = ovx::internalSceneOf(scene);
    if (!owningScene)
        return false;
    AttachedStage& owner = owningScene->getAttachedStage();
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
        return false;

    if (!ovx::refreshArticulationCache(owner, ovx::allPhysicsScenes(owner)))
    {
        OVX_RUNTIME_ERROR("ovxWriteAttribute: the articulation structural walk failed; nothing was written.");
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
    std::vector<uint32_t> localToRow;
    if (!acquireArticulationRows(*backend, scene, gpu, jc.artis, generation, entriesPtr, localToRow))
        return false;
    if (!ovx::ensureJointRecords(jc, source, jc.joints, *entriesPtr, localToRow, generation, true))
        return false;
    if (jc.slices.empty())
        return true;

    // Prim-set selection: plan/scatter only the admitted joints. Host-only, so the compacted records ride on
    // recsHost.
    std::vector<omni::physx::tensors::ArticulationDofOvStageRecord> selRecs;
    std::vector<ObjectKey> selRowKeys;
    std::vector<ovx_primpath_t> selRowHandles;
    // Each row's joint, which makes the group's prim list.
    const std::vector<ObjectKey>* rowKeys = &jc.dofKeys;
    const std::vector<ovx_primpath_t>* rowHandles = &jc.dofHandles;
    if (s.hasSelection)
    {
        selectJointRows(s, jc, selRecs, selRowKeys, selRowHandles);
        if (selRecs.empty())
            return true; // none of this scene's joints are in the selection
        rowKeys = &selRowKeys;
        rowHandles = &selRowHandles;
    }

    const uint32_t numOut =
        s.hasSelection ? static_cast<uint32_t>(selRecs.size()) : static_cast<uint32_t>(jc.recs.size());
    const uint32_t comp = omni::physx::tensors::dofPropertyComponents(row.prop);
    const bool isByte = omni::physx::tensors::dofPropertyIsByte(row.prop);

    std::unique_ptr<GroupSlot> slot(new GroupSlot(s.attach));
    slot->scene = scene;
    slot->jointPropRow = &row;
    slot->numOut = numOut;
    slot->comp = comp;
    const size_t bytes = size_t(numOut) * comp * (isByte ? 1u : 4u);
    slot->floats.assign(bytes / 4 + (bytes % 4 ? 1 : 0), 0.0f);
    // See recsHost: the CPU property scatter re-derives the shared cache by default; hand it the compacted
    // records when a selection filtered them.
    if (s.hasSelection)
        slot->recsHost = selRecs;

    ovx_path_dictionary_t*& dict = slot->listDictionary;
    const bool haveHandles = rowHandles->size() == rowKeys->size();
    slot->list = haveHandles ? omni::physics::ovstage::buildPathListFromHandles(source, rowHandles->data(),
                                                                               rowHandles->size(), &dict) :
                               omni::physics::ovstage::buildPathList(source, rowKeys->data(), rowKeys->size(), &dict);
    if (slot->list == OVX_INVALID_PRIMPATH_LIST || !dict)
        return false;
    s.dict = dict;

    slot->shapes.assign(1, static_cast<int64_t>(numOut));
    slot->tensors.assign(1, DLTensor{});
    DLTensor& t = slot->tensors[0];
    t.data = slot->floats.data();
    t.device = DLDevice{ kDLCPU, 0 };
    t.ndim = 1;
    t.dtype = isByte ? DLDataType{ kDLUInt, 8, static_cast<uint16_t>(comp) } :
                       DLDataType{ kDLFloat, 32, static_cast<uint16_t>(comp) };
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

// Validate the complete request before allocating any writable group. The same
// authoritative write rows are used below to select each scatter implementation.
bool acceptsWriteAttribute(uint32_t type, const std::string& name)
{
    switch (type)
    {
    case kOvxRigidBody:
        return findRigidWriteAttribute(name) != nullptr;
    case kOvxArticulationLink:
    {
        const WriteAttributeRow* row = findRigidWriteAttribute(name);
        return row && row->linkWritable;
    }
    case kOvxArticulation:
        return findArtiWriteAttribute(name) != nullptr;
    case kOvxArticulationJoint:
        return findJointPropertyWriteAttribute(name) || findJointWriteAttribute(name);
    case kOvxFixedTendon:
    case kOvxSpatialTendon:
    {
        const TendonWriteAttributeRow* row = findTendonWriteAttribute(name);
        return row && (type == kOvxFixedTendon || !row->fixedOnly);
    }
    case kOvxVehicleWheel:
        return findVehicleWriteAttribute(name) != nullptr;
    case kOvxCharacterController:
        return findCctWriteAttribute(name) != nullptr;
    case kOvxVehicle:
        return findVehicleControlAttribute(name) != nullptr;
    case kOvxDeformableVolume:
        return name == OvxAttr::kPoints || name == OvxAttr::kVelocities || name == OvxAttr::kKinematicTarget;
    case kOvxDeformableSurface:
    case kOvxParticleSet:
        return name == OvxAttr::kPoints || name == OvxAttr::kVelocities;
    case kOvxDeformableMaterial:
        return findDeformableMaterialWriteAttribute(name) != nullptr;
    default:
        return false;
    }
}

// Keep whole-request validation as informative as the per-type group planners.
// The request index and resolved name are added by the caller.
const char* writeAttributeRejectionReason(uint32_t type, const std::string& name)
{
    switch (type)
    {
    case kOvxRigidBody:
        return "rigid bodies do not accept this attribute.";
    case kOvxArticulationLink:
        return findRigidWriteAttribute(name) ?
            "this attribute is not writable on an articulation link. PhysX has no link write for it -- "
            "move the articulation through its root pose and jointPosition instead. Link mass, inertia, "
            "centreOfMass*, disableGravity and the per-shape properties are writable." :
            "articulation links do not accept this attribute.";
    case kOvxArticulation:
        return "articulations do not accept this attribute. Root state is 'rootPosition', 'rootOrientation', "
            "'rootLinearVelocity' and 'rootAngularVelocity'; joint state goes through "
            "kOvxArticulationJoint and link properties through kOvxArticulationLink.";
    case kOvxArticulationJoint:
        return "articulation joints do not accept this attribute.";
    case kOvxFixedTendon:
        return "fixed tendons do not accept this attribute.";
    case kOvxSpatialTendon:
        return findTendonWriteAttribute(name) ?
            "this attribute does not exist on a spatial tendon -- the schema places it on the leaf "
            "attachment. It is writable on kOvxFixedTendon." :
            "spatial tendons do not accept this attribute.";
    case kOvxVehicle:
        return "vehicles accept accelerator, brake0, brake1, steer, requestedGear, tank thrust0/thrust1, "
            "and resetToRestState; other drive state is read-only.";
    case kOvxVehicleWheel:
        return "vehicle wheels do not accept this attribute. A wheel's pose is derived from the chassis, "
            "its suspension and its steer angle every step. The wheel's controls are writable: "
            "'driveTorque', 'brakeTorque' and 'steerAngle'.";
    case kOvxDeformableVolume:
    case kOvxDeformableSurface:
        return "deformable bodies do not accept this attribute. They accept 'points' and 'velocities' "
            "(and 'kinematicTarget' on a volume deformable). 'restPoints' is authored geometry the solver "
            "never rewrites.";
    case kOvxParticleSet:
        return "particle sets do not accept this attribute. They accept 'points' and 'velocities'.";
    case kOvxDeformableMaterial:
        return "deformable materials do not accept this attribute.";
    case kOvxCharacterController:
        return "character controllers accept only 'move'.";
    case kOvxSdfShape:
        // Query-only (ADR-0048): nothing about the shape is an input a write could carry.
        return "SDF shapes have no writable attribute; evaluate them with ovxEvaluateSdfShapes.";
    case kOvxRigidActor:
        return "rigid actors are not writable.";
    case kOvxShape:
        return "shapes are not writable.";
    case kOvxPhysxMaterial:
        return "PhysX materials are not writable.";
    case kOvxScene:
        return "scenes are not writable.";
    case kOvxPhysics:
        return "the physics object is not writable.";
    case kOvxDeformableSkinMesh:
        // Skinning output: the post-solve callback rewrites it every step, so a written value would not
        // survive the next solve (REQ-READ-SKIN-001 AC-7). Read-only by design, not pending.
        return "deformable skin meshes are read-only, since 'points' is skinning output recomputed every step. "
            "Write the owning deformable's sim-mesh 'points' (kOvxDeformableVolume / kOvxDeformableSurface) instead.";
    default:
        return "this object type is not writable.";
    }
}

// Plan the groups this session hands out: resolve the query to its matched bodies and split them by
// owning scene, the way the read splits its columns. Returns false if the attribute is not writable
// for the queried type, or if a group could not be built.
bool planGroups(WriteSession& s, uint32_t type, uint32_t scope, AttachedStage& owner)
{
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
            OVX_RUNTIME_ERROR("ovxWriteAttribute: articulation links do not accept attribute '%s'.",
                       s.attribute.c_str());
            return false;
        }
        if (!lrow->linkWritable)
        {
            OVX_RUNTIME_ERROR("ovxWriteAttribute: '%s' is not writable on an articulation link. PhysX has "
                       "no link write for it -- move the articulation through its root pose and "
                       "jointPosition instead. Link mass, inertia, centreOfMass*, disableGravity "
                       "and the per-shape properties ARE writable.",
                       s.attribute.c_str());
            return false;
        }

        ovx::SourceContext lctx;
        if (!ovx::getSourceContext(s.attach, "ovxWriteAttribute", lctx))
            return false;
        // The read's link enumeration, shared: a write must cover exactly the links a read covers --
        // including the ACTIVE set, which the scan now takes rather than rebuilding, so the two
        // directions cannot disagree about which actors moved.
        ovx::RigidRecordScan linkScan;
        const ovx::ActiveActorSet linkActive =
            (scope == kOvxActive) ? ovx::collectActiveActors(owner) : ovx::ActiveActorSet{};
        ovx::scanLinkRecords(owner, scope, linkActive, linkScan);
        for (PxScene* scene : ovx::allPhysicsScenes(owner))
        {
            const ovx::RigidSceneBucket* bucket = ovx::bucketForScene(linkScan, scene);
            if (!bucket)
                continue;
            // kOvxActive only for the shared cache: its key carries a type, and a link session must not
            // collide with the rigid one over the same scene. The whole-set test, though, needs the REAL
            // query scope -- passed separately so a kOvxAll link write can still skip the host dedup.
            if (!planRigidGroup(s, *lctx.source, scene, bucket->bodies, bucket->keys, *lrow, kOvxActive,
                                /*allocate=*/true, /*wholeSetScope=*/scope))
                return false;
        }
        return true;
    }

    if (type == kOvxArticulation)
    {
        const ArtiWriteAttributeRow* arow = findArtiWriteAttribute(s.attribute);
        if (!arow)
        {
            OVX_RUNTIME_ERROR("ovxWriteAttribute: articulations do not accept attribute '%s'. Root state is "
                       "'rootPosition', 'rootOrientation', 'rootLinearVelocity' and 'rootAngularVelocity'; "
                       "joint state goes through kOvxArticulationJoint and link properties through "
                       "kOvxArticulationLink.",
                       s.attribute.c_str());
            return false;
        }
        ovx::SourceContext actx;
        if (!ovx::getSourceContext(s.attach, "ovxWriteAttribute", actx))
            return false;
        // The read's structural walk, not a second one: it enumerates the same ePTArticulation
        // records AND detects duplicate or inconsistent ones, which an independent scan here did
        // not. Sharing it is what keeps a root write addressing the prims a root read reports.
        const std::vector<PxScene*> artiScenes = ovx::allPhysicsScenes(owner);
        if (!ovx::refreshArticulationCache(owner, artiScenes))
        {
            OVX_RUNTIME_ERROR("ovxWriteAttribute: the articulation structural walk failed; nothing was written.");
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
                OVX_RUNTIME_ERROR("ovxWriteAttribute: duplicate or inconsistent ePTArticulation records "
                           "for this scene; nothing was written.");
                return false;
            }
            // The roots this scene's write covers, the same for every root attribute of the session: filtered by
            // its first one.
            std::unordered_map<const PxScene*, WriteSession::RootPlan>::iterator plan = s.rootPlans.find(scene);
            if (plan == s.rootPlans.end())
            {
                // The read's canonical root handles, kept in the shared entry beside the keys they come from and
                // filled here when no read has yet. A structural refresh rebuilds the entry, so they never
                // outlive their keys.
                if (entry->rootHandles.size() != entry->rootKeys.size())
                {
                    entry->rootHandles.clear();
                    omni::physics::ovstage::canonicalisePathHandles(
                        *actx.source, entry->rootKeys.data(), entry->rootKeys.size(), entry->rootHandles, nullptr);
                }
                const bool haveHandles = entry->rootHandles.size() == entry->rootKeys.size();
                // ACTIVE membership was resolved from the query's shared root/active-link
                // rule at session open; do not replace it with a current sleeping check.
                WriteSession::RootPlan roots;
                roots.artis.reserve(entry->rootArtis.size());
                roots.keys.reserve(entry->rootArtis.size());
                if (haveHandles)
                    roots.handles.reserve(entry->rootArtis.size());
                for (size_t i = 0; i < entry->rootArtis.size(); ++i)
                {
                    PxArticulationReducedCoordinate* arti = entry->rootArtis[i];
                    if (!arti || arti->getScene() != scene)
                        continue;
                    if (!s.admits(entry->rootKeys[i]))
                        continue; // prim-set selection: leave unselected articulations unwritten
                    roots.artis.push_back(arti);
                    roots.keys.push_back(entry->rootKeys[i]);
                    if (haveHandles)
                        roots.handles.push_back(entry->rootHandles[i]);
                }
                plan = s.rootPlans.emplace(scene, std::move(roots)).first;
            }
            if (!planArticulationGroup(s, *actx.source, scene, plan->second.artis, plan->second.keys,
                                       plan->second.handles, *arow))
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
            OVX_RUNTIME_ERROR("ovxWriteAttribute: %s tendons do not accept attribute '%s'.",
                       fixed ? "fixed" : "spatial", s.attribute.c_str());
            return false;
        }
        if (!fixed && trow->fixedOnly)
        {
            // Refused by NAME, and not because PhysX lacks the setter: the schema puts a spatial
            // tendon's limit and rest length on its LEAF ATTACHMENT, so there is no such property on
            // the tendon to write. The read refuses the same pair for the same reason.
            OVX_RUNTIME_ERROR("ovxWriteAttribute: '%s' does not exist on a spatial tendon -- the schema "
                       "places it on the leaf attachment. It is writable on kOvxFixedTendon.",
                       s.attribute.c_str());
            return false;
        }
        ovx::SourceContext tctx;
        if (!ovx::getSourceContext(s.attach, "ovxWriteAttribute", tctx))
            return false;
        for (PxScene* scene : ovx::allPhysicsScenes(owner))
            if (!planTendonGroup(s, *tctx.source, scene, fixed, *trow))
                return false;
        return true;
    }

    if (type == kOvxArticulationJoint)
    {
        // Properties first: the two tables are disjoint, and a property is the larger set.
        if (const JointPropertyWriteRow* prow = findJointPropertyWriteAttribute(s.attribute))
        {
            ovx::SourceContext pctx;
            if (!ovx::getSourceContext(s.attach, "ovxWriteAttribute", pctx))
                return false;
            for (PxScene* scene : ovx::allPhysicsScenes(owner))
                if (!planJointPropertyGroup(s, *pctx.source, scene, *prow))
                    return false;
            return true;
        }
        const JointWriteAttributeRow* jrow = findJointWriteAttribute(s.attribute);
        if (!jrow)
        {
            OVX_RUNTIME_ERROR("ovxWriteAttribute: articulation joints do not accept attribute '%s'.",
                       s.attribute.c_str());
            return false;
        }
        ovx::SourceContext jctx;
        if (!ovx::getSourceContext(s.attach, "ovxWriteAttribute", jctx))
            return false;
        for (PxScene* scene : ovx::allPhysicsScenes(owner))
            if (!planJointGroup(s, *jctx.source, scene, *jrow))
                return false;
        return true;
    }

    if (type == kOvxVehicleWheel || type == kOvxVehicle)
    {
        const VehicleWriteAttributeRow* wheel = type == kOvxVehicleWheel ?
            findVehicleWriteAttribute(s.attribute) : nullptr;
        const VehicleControlAttributeRow* vehicle = type == kOvxVehicle ?
            findVehicleControlAttribute(s.attribute) : nullptr;
        if (!wheel && !vehicle)
        {
            OVX_RUNTIME_ERROR("ovxWriteAttribute: '%s' is not a writable vehicle control.", s.attribute.c_str());
            return false;
        }
        ovx::SourceContext context;
        if (!ovx::getSourceContext(s.attach, "ovxWriteAttribute", context))
            return false;
        for (PxScene* scene : ovx::allPhysicsScenes(owner))
            if (!planVehicleDataGroup(s, *context.source, scene, wheel, vehicle))
                return false;
        return true;
    }

    // Deformable simulation-mesh state (ADR-0012). Both kinds serve the same two columns the read
    // serves; they differ only in which PhysX type holds the buffers.
    if (type == kOvxDeformableVolume || type == kOvxDeformableSurface)
    {
        const bool velocity = (s.attribute == OvxAttr::kVelocities);
        const bool isVolume = (type == kOvxDeformableVolume);
        // kinematicTarget is a VOLUME-only write: PxDeformableVolume::setKinematicTargetBufferD, driven
        // through a runtime-owned per-body buffer (InternalVolumeDeformableBody::mSimMeshKinematicTargetsD)
        // that outlives the write session -- the ownership the AC-4 exclusion lacked. Surface deformables
        // have no such API here, so the name is refused for them.
        const bool kinematicTarget = (s.attribute == OvxAttr::kKinematicTarget) && isVolume;
        if (!velocity && s.attribute != OvxAttr::kPoints && !kinematicTarget)
        {
            //   restPoints       authored geometry the solver never rewrites -- read-only.
            //   kinematicTarget  volume-only; refused here only on a SURFACE deformable.
            OVX_RUNTIME_ERROR("ovxWriteAttribute: deformable bodies do not accept attribute '%s'. They accept "
                       "'points' and 'velocities' (and 'kinematicTarget' on a volume deformable). "
                       "'restPoints' is authored geometry the solver never rewrites.",
                       s.attribute.c_str());
            return false;
        }
        ovx::SourceContext dctx;
        if (!ovx::getSourceContext(s.attach, "ovxWriteAttribute", dctx))
            return false;
        for (PxScene* scene : ovx::allPhysicsScenes(owner))
        {
            ovx::DeformableWriteTargets targets;
            if (!ovx::deformableWriteTargets(scene, isVolume, targets))
                return false;
            if (!planDeformableGroup(s, *dctx.source, scene, isVolume, targets, velocity, kinematicTarget))
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
            OVX_RUNTIME_ERROR("ovxWriteAttribute: deformable materials do not accept attribute '%s'.",
                       s.attribute.c_str());
            return false;
        }
        ovx::SourceContext mctx;
        if (!ovx::getSourceContext(s.attach, "ovxWriteAttribute", mctx))
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
            OVX_RUNTIME_ERROR("ovxWriteAttribute: particle sets do not accept attribute '%s'. They accept "
                       "'points' and 'velocities'.",
                       s.attribute.c_str());
            return false;
        }
        ovx::SourceContext pctx;
        if (!ovx::getSourceContext(s.attach, "ovxWriteAttribute", pctx))
            return false;
        for (PxScene* scene : ovx::allPhysicsScenes(owner))
        {
            ovx::ParticleWriteTargets targets;
            if (!ovx::particleWriteTargets(scene, scope, targets))
                return false;
            if (!planParticleGroup(s, *pctx.source, scene, targets, velocity))
                return false;
        }
        return true;
    }

    if (type == kOvxCharacterController)
    {
        const CctWriteAttributeRow* crow = findCctWriteAttribute(s.attribute);
        if (!crow)
        {
            OVX_RUNTIME_ERROR("ovxWriteAttribute: character controllers do not accept attribute '%s'. "
                              "They accept 'move'. Pose is the rigid-body position and orientation "
                              "of the controller prim.",
                              s.attribute.c_str());
            return false;
        }
        ovx::SourceContext cctx;
        if (!ovx::getSourceContext(s.attach, "ovxWriteAttributes", cctx))
            return false;
        return planCctGroup(s, owner, *cctx.source, scope, *crow);
    }

    if (type != kOvxRigidBody)
    {
        OVX_RUNTIME_ERROR("ovxWriteAttribute: object type %u is not writable yet.", type);
        return false;
    }

    const WriteAttributeRow* row = findRigidWriteAttribute(s.attribute);
    if (!row)
    {
        OVX_RUNTIME_ERROR("ovxWriteAttribute: rigid bodies do not accept attribute '%s'.", s.attribute.c_str());
        return false;
    }

    ovx::SourceContext ctx;
    if (!ovx::getSourceContext(s.attach, "ovxWriteAttribute", ctx))
        return false;

    // Deferred: only walked if a scene actually needs it, and skipped entirely per scene when the
    // shared cache already describes it.
    ovx::LazyRigidScan lazy{ owner, scope };
    const std::vector<PxScene*> scenes = ovx::allPhysicsScenes(owner);
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
                hit->second.dbEpoch == omni::physx::internal::recordLifetimeEpoch(scene) &&
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
        // is what stamps the generation these belong to, alongside the rows and handles. A selected
        // write stays off the shared cache (REQ-READ-SELECT-001 AC-5); planRigidGroup filters the bucket.
        if (scope == kOvxAll && !s.hasSelection)
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
            // A complete instancer list, empty included, names every instancer the scene holds now: a column parked
            // for one it no longer lists belongs to a removed instancer and is freed rather than kept until the
            // query goes. An unknown list frees nothing.
            if (s.columns && targets.listed)
            {
                std::unordered_set<uint64_t> liveInstancers;
                for (const ObjectKey& key : targets.keys)
                    liveInstancers.insert(key.handle);
                if (!s.columns->dropAbsent(scene, row, liveInstancers))
                {
                    OVX_RUNTIME_ERROR("ovxWriteAttributes: could not reclaim the parked CUDA buffer of a removed "
                                      "instancer.");
                    return false;
                }
            }
            if (!targets.view)
                continue; // this scene has no instancers, or its device view is not built yet -- not a failure
            for (uint32_t i = 0; i < targets.keys.size(); ++i)
            {
                if (s.hasSelection && !s.admits(targets.keys[i]))
                    continue; // prim-set selection: only admitted instancers publish their array
                if (!planInstancerGroup(s, *ctx.source, scene, targets.view, targets.generation, targets.keys[i],
                                        targets.arrayLengths[i], i, column, instComp, *row))
                    return false;
            }
        }
    }
    return true;
}

// The caller's producer handoff for one ACCEPTED commit (ovstage_cuda_sync_t). The two fields are
// independent dependencies and both are honoured when both are set: a nonzero `stream` is drained, a
// nonzero `wait_event` is waited for. `stream` names the PRODUCER's stream; it is never used as a
// stream for this runtime's own work.
//
// Blocking for every column residency: true means every supplied dependency has COMPLETED, observed
// on the host. That orders every consumer that follows on any stream or context -- the host scatters,
// the view kernels on the engine's default stream, and the PhysX DirectGPU work gated behind them --
// without relying on any stream's implicit ordering. The calling thread pays for it: it waits for the
// event, and for ALL work queued on the named stream, not only the fill.
//
// No engine context is pushed. A context-relative sentinel such as 1 (the legacy default stream) is
// therefore resolved in the CALLER's current context, the one its producer's default stream belongs
// to; an explicit stream or event handle names its own context. When the driver rejects a handle --
// no current context for a sentinel included -- that rejection is the result, not a retry elsewhere.
//
// Called once per accepted commit, before any preflight can refuse the spent group. Any failure,
// including a nonempty handoff with no CUDA driver, leaves `producerUnproven` set: nothing proves the
// producer has stopped writing this slot's storage, so release quarantines it. The handles are
// borrowed for this call only and are never retried later -- the caller may destroy them once the
// commit returns.
bool honourCallerSync(GroupSlot& slot, const char* attributeName, ovstage_cuda_sync_t sync)
{
    slot.producerUnproven = sync.stream != 0 || sync.wait_event != 0;
    if (!slot.producerUnproven)
        return true;
    char who[192];
    std::snprintf(who, sizeof(who), "ovxCommitGroup: request %zu ('%.128s')", slot.attributeIndex, attributeName);
    if (!ovx::waitForCallerProducer(sync, who, "nothing was read or written"))
        return false;
    slot.producerUnproven = false;
    return true;
}

// Publish a joint group: hand the flat DOF column and the record list to the articulation view.
// No row re-resolution here: a DOF record names (view row, dof index), derived when the write opened, and the
// rows stay valid while the session is live (the commit refuses a moved record epoch). A view rebuilt in between
// only appends the articulations a step inserted, so the others keep their rows, and the plan refuses an
// articulation not yet in its scene. A structural change through a native pointer is not covered (ADR-0074).
bool scatterJointGroup(GroupSlot& slot, const omni::physx::tensors::OvStageDeviceWriteArgs* deviceArgs)
{
    if (slot.numOut == 0)
        return true;
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
        return false;
    const bool gpu = slot.deviceOrdinal >= 0;

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
            OVX_RUNTIME_ERROR("ovxCommitGroup: superset articulation view unavailable; nothing was written.");
            return false;
        }
        t_scatterCommitAttempted = true; // past every pre-commit check; this setter is the commit point
        return (av->*slot.jointRow->gpuWrite)(
            &td, reinterpret_cast<const omni::physx::tensors::ArticulationDofOvStageRecord*>(slot.recsDev),
            slot.numOut, deviceArgs ? *deviceArgs : omni::physx::tensors::OvStageDeviceWriteArgs());
    }
    omni::physx::tensors::CpuSimulationView* sv = backend->acquireCpuSceneView(slot.scene);
    omni::physx::tensors::CpuArticulationView* av =
        sv ? sv->supersetArticulationView(&artiRowMap, &entries) : nullptr;
    if (!av)
    {
        OVX_RUNTIME_ERROR("ovxCommitGroup: superset articulation view unavailable; nothing was written.");
        return false;
    }
    // Host records live in the shared cache; the host path has no device upload to read them from.
    const ovx::ArticulationReadCacheEntry* jcPtr = ovx::articulationCacheEntry(slot.scene);
    if (!jcPtr)
    {
        OVX_RUNTIME_ERROR("ovxCommitGroup: the articulation cache no longer describes this scene; "
                   "nothing was written.");
        return false;
    }
    const ovx::ArticulationReadCacheEntry& jc = *jcPtr;
    // A selected write carries its own compacted records (recsHost, snapshotted at plan time); a whole-set
    // write re-derives jc.recs here. Scatter through whichever describes this group's column.
    const bool useSlotRecs = !slot.recsHost.empty();
    const omni::physx::tensors::ArticulationDofOvStageRecord* const recs =
        useSlotRecs ? slot.recsHost.data() : jc.recs.data();
    const size_t recCount = useSlotRecs ? slot.recsHost.size() : jc.recs.size();
    if (recCount != slot.numOut)
    {
        OVX_RUNTIME_ERROR("ovxCommitGroup: the joint record list moved since this group was planned; "
                   "nothing was written.");
        return false;
    }
    t_scatterCommitAttempted = true; // past every pre-commit check; this setter is the commit point
    return (av->*slot.jointRow->cpuWrite)(&td, recs, slot.numOut);
}

// Publish an articulation-root group: re-resolve its articulations to superset rows, then hand the
// column and the rows to the articulation view.
//
// Rows are resolved HERE, not at plan time, for the reason the rigid path resolves them here: a row
// indexes the superset view's ordering, and the backend takes a fresh generation whenever it rebuilds
// that view, so a row cached across the caller's fill would address the view as it was when the group
// was handed out and scatter this group's values into whatever occupies those rows now.
bool scatterArticulationGroup(GroupSlot& slot, const omni::physx::tensors::OvStageDeviceWriteArgs* deviceArgs)
{
    if (slot.artis.empty())
        return true; // nothing to publish -- not a failure
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
    {
        OVX_RUNTIME_ERROR("ovxCommitGroup: no tensor backend.");
        return false;
    }
    const bool gpu = slot.deviceOrdinal >= 0;

    uint64_t generation = 0;
    const std::vector<omni::physx::tensors::ArticulationEntry>* entries = nullptr;
    omni::physx::tensors::GpuArticulationView* gpuView = nullptr;
    omni::physx::tensors::CpuArticulationView* cpuView = nullptr;
    // A stream-ordered group resolved its rows when the session opened (ADR-0052); the commit checked the
    // view was not rebuilt since.
    std::vector<PxU32> rows;
    if (slot.streamOrdered)
    {
        const std::unordered_map<const PxArticulationReducedCoordinate*, PxU32>* artiRowMap = nullptr;
        omni::physx::tensors::GpuSimulationView* sv = backend->acquireSceneView(slot.scene);
        gpuView = sv ? sv->supersetArticulationView(&artiRowMap, &entries) : nullptr;
        if (!gpuView)
        {
            OVX_RUNTIME_ERROR("ovxCommitGroup: superset articulation view unavailable; nothing was written.");
            return false;
        }
        rows = slot.hostRows;
        slot.rowsVersion = slot.hostRowsToken;
    }
    else
    {
        if (!acquireArticulationRows(*backend, slot.scene, gpu, slot.artis, generation, entries, rows, &gpuView,
                                     &cpuView))
        {
            OVX_RUNTIME_ERROR("ovxCommitGroup: no superset articulation view describes this articulation set; "
                              "nothing was written.");
            return false;
        }

        // Minted fresh on every commit rather than held. The token has to change exactly when the row
        // CONTENTS do, and there is no articulation read cache to hang a held version off yet -- so this
        // errs towards re-uploading a list that has not moved, which costs one small memcpy, rather than
        // reusing a token for a list that has. Sharing a held version waits on a read-side articulation read cache.
        slot.rowsVersion = ovx::nextRowsVersion();
    }

    omni::physics::tensors::TensorDesc td;
    td.device = slot.deviceOrdinal;
    td.dtype = omni::physics::tensors::TensorDataType::eFloat32;
    td.numDims = 1;
    td.dims[0] = static_cast<int>(rows.size() * slot.comp);
    td.data = gpu ? reinterpret_cast<void*>(slot.deviceData) : static_cast<void*>(slot.floats.data());

    const PxU32 numOut = static_cast<PxU32>(rows.size());
    if (gpu)
        return (gpuView->*slot.artiRow->gpuWrite)(
            &td, rows.data(), numOut, slot.rowsVersion,
            deviceArgs ? *deviceArgs : omni::physx::tensors::OvStageDeviceWriteArgs());
    return (cpuView->*slot.artiRow->cpuWrite)(&td, rows.data(), numOut, slot.rowsVersion);
}

// Publish a tendon group: hand the column and the record list to the articulation view. No row
// re-resolution here: a tendon record names (view row, tendon slot), derived when the write opened, and the rows
// stay valid while the session is live, as for a joint group (scatterJointGroup); the tendon walk skips an
// articulation not yet in its scene. A structural change through a native pointer is not covered (ADR-0074).
bool scatterTendonGroup(GroupSlot& slot, const omni::physx::tensors::OvStageDeviceWriteArgs* deviceArgs)
{
    if (slot.numOut == 0)
        return true;
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
        return false;
    const bool gpu = slot.deviceOrdinal >= 0;

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
            OVX_RUNTIME_ERROR("ovxCommitGroup: superset articulation view unavailable; nothing was written.");
            return false;
        }
        const omni::physx::tensors::ArticulationTendonOvStageRecord* recs =
            reinterpret_cast<const omni::physx::tensors::ArticulationTendonOvStageRecord*>(slot.recsDev);
        const omni::physx::tensors::OvStageDeviceWriteArgs run =
            deviceArgs ? *deviceArgs : omni::physx::tensors::OvStageDeviceWriteArgs();
        return slot.tendonFixed ?
                   av->setFixedTendonPropertiesOvStage(&td, recs, slot.numOut, slot.tendonRow->prop, run) :
                   av->setSpatialTendonPropertiesOvStage(&td, recs, slot.numOut, slot.tendonRow->prop, run);
    }
    omni::physx::tensors::CpuSimulationView* sv = backend->acquireCpuSceneView(slot.scene);
    omni::physx::tensors::CpuArticulationView* av =
        sv ? sv->supersetArticulationView(&artiRowMap, &entries) : nullptr;
    if (!av)
    {
        OVX_RUNTIME_ERROR("ovxCommitGroup: superset articulation view unavailable; nothing was written.");
        return false;
    }
    // A selected write carries its own compacted records (tendonRecsHost, snapshotted at plan time); a
    // whole-set write re-reads the shared cache (the host path has no device upload to read them from).
    ovx::TendonReadCacheEntry& tc = ovx::tendonCacheEntry(slot.scene, slot.tendonFixed);
    const bool useSlotRecs = !slot.tendonRecsHost.empty();
    const omni::physx::tensors::ArticulationTendonOvStageRecord* recs =
        useSlotRecs ? slot.tendonRecsHost.data() : tc.recs.data();
    const size_t recCount = useSlotRecs ? slot.tendonRecsHost.size() : tc.recs.size();
    if (recCount != slot.numOut)
    {
        OVX_RUNTIME_ERROR("ovxCommitGroup: the tendon record list moved since this group was planned; "
                   "nothing was written.");
        return false;
    }
    return slot.tendonFixed ?
               av->setFixedTendonPropertiesOvStage(&td, recs, slot.numOut, slot.tendonRow->prop) :
               av->setSpatialTendonPropertiesOvStage(&td, recs, slot.numOut, slot.tendonRow->prop);
}

// Publish only the row mapping captured at open; equal counts do not prove identity.
bool scatterVehicleGroup(GroupSlot& slot)
{
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    omni::physx::internal::InternalScene* internalScene = ovx::internalSceneOf(slot.scene);
    uint64_t generation = 0;
    omni::physx::tensors::CpuSimulationView* sceneView =
        backend ? backend->acquireCpuSceneView(slot.scene, &generation) : nullptr;
    if (!internalScene || !sceneView || generation != slot.vehicleGeneration ||
        internalScene->mVehicleSetEpoch != slot.vehicleSetEpoch)
    {
        OVX_RUNTIME_ERROR("ovxCommitGroup: vehicle membership changed; nothing was written. Reopen the write.");
        return false;
    }
    omni::physx::tensors::BaseVehicleView* view = sceneView->vehicleView(*internalScene);
    if (!view)
        return false;
    omni::physics::tensors::TensorDesc tensor;
    tensor.device = -1;
    tensor.dtype = slot.vehicleControlRow ? slot.vehicleControlRow->dtype :
        omni::physics::tensors::TensorDataType::eFloat32;
    tensor.numDims = 1;
    tensor.dims[0] = static_cast<int>(slot.numOut);
    tensor.data = slot.tensors[0].data;
    t_scatterCommitAttempted = true;
    const bool applied = slot.vehicleControlRow ?
        view->setVehicleControlOvStage(*internalScene, slot.wholeVehicleRecs.data(), slot.numOut,
                                       slot.vehicleControlRow->control, &tensor) :
        view->setWheelControlOvStage(*internalScene, slot.vehicleRecsHost.data(), slot.numOut,
                                     slot.vehicleRow->control, &tensor);
    // Both setters return false only during whole-batch preflight. An exception after entering
    // a setter leaves the flag raised, preserving the conservative partial-publication handling.
    if (!applied)
        t_scatterCommitAttempted = false;
    return applied;
}

// Whether an instancer group's view is still alive: the scene view it was taken from reports `generation`. A rebuilt
// scene view has deleted it, so the group is refused, nothing written, and the caller reopens the write
// (REQ-INPUT-CORE-001 AC-16). Not re-resolved: the supported trigger is rare (an articulation the stage drain created
// entering its scene at a step), and reopening resolves the instancer against the rebuilt view.
bool instancerViewCurrent(const GroupSlot& slot, uint64_t generation)
{
    if (generation != 0 && generation == slot.instancerGeneration)
        return true;
    OVX_RUNTIME_ERROR("Point-instancer write: the scene's view was rebuilt since this write was opened (for example "
                      "when an articulation entered its scene at a step), which retires the instancer view the group "
                      "was planned against; nothing was written. Reopen the write.");
    return false;
}

// Publish one instancer's array. The view applies the hole contract -- a slot with no live body is
// skipped -- and inverts the reframe; see BasePointInstancerView::setInstancerColumnsOvStage.
bool scatterInstancerGroup(GroupSlot& slot,
                           const omni::physx::tensors::OvStageDeviceWriteArgs* deviceArgs,
                           const uint8_t* hostMask)
{
    if (slot.numOut == 0 || !slot.instancerView)
        return true;
    // A stream-ordered group was checked when its commit began (streamOrderedGroupCurrent re-prepares it).
    if (!slot.streamOrdered)
    {
        omni::physx::tensors::SimulationBackend* const backend = omni::physx::tensors::GetSimulationBackend();
        uint64_t generation = 0;
        if (backend)
        {
            if (slot.scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API))
                backend->acquireSceneView(slot.scene, &generation);
            else
                backend->acquireCpuSceneView(slot.scene, &generation);
        }
        if (!instancerViewCurrent(slot, generation))
            return false;
    }

    const void* const src = slot.deviceData ? reinterpret_cast<const void*>(slot.deviceData) :
                                              static_cast<const void*>(slot.floats.data());
    return slot.instancerView->setInstancerColumnOvStage(slot.instancerColumn, slot.instancerIdx, src,
                                                         slot.numOut, hostMask, deviceArgs);
}

// Publish a joint-property group. Host-only by construction -- see planJointPropertyGroup.
bool scatterJointPropertyGroup(GroupSlot& slot)
{
    if (slot.numOut == 0)
        return true;
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
        return false;

    const ovx::ArticulationReadCacheEntry* jc = ovx::articulationCacheEntry(slot.scene);
    // A selected write carries its own compacted records (recsHost, snapshotted at plan time); a whole-set
    // write re-derives jc->recs here. Scatter through whichever describes this group's column.
    const bool useSlotRecs = !slot.recsHost.empty();
    const omni::physx::tensors::ArticulationDofOvStageRecord* const recs =
        useSlotRecs ? slot.recsHost.data() : (jc ? jc->recs.data() : nullptr);
    const size_t recCount = useSlotRecs ? slot.recsHost.size() : (jc ? jc->recs.size() : 0);
    if ((!jc && !useSlotRecs) || recCount != slot.numOut)
    {
        OVX_RUNTIME_ERROR("ovxCommitGroup: the joint record list moved since this group was planned; "
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
        OVX_RUNTIME_ERROR("ovxCommitGroup: superset articulation view unavailable; nothing was written.");
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

    return av->setDofPropertyOvStage(slot.jointPropRow->prop, recs, slot.numOut, &td);
}

// Each committed move calls PxController::move once, with the owning scene's timestep.
// Nothing is stored for a later step to replay. Collision flags are intentionally discarded:
// exposing them is still an open product question.
bool scatterCctGroup(AttachedStage& owner, GroupSlot& slot)
{
    if (slot.numOut == 0 || !slot.cctRow)
        return true;

    std::vector<omni::physx::ovx::CctOvStageRow> currentRows;
    if (!omni::physx::ovx::enumerateCctRows(owner, kOvxAll, currentRows))
        return false;
    std::unordered_map<ObjectKey, uint32_t, ObjectKey::Hash> keyToRow;
    keyToRow.reserve(currentRows.size());
    for (uint32_t i = 0; i < currentRows.size(); ++i)
        keyToRow.emplace(currentRows[i].key, i);
    std::vector<omni::physx::ovx::CctOvStageRow> resolved;
    resolved.reserve(slot.keys.size());
    for (const ObjectKey& key : slot.keys)
    {
        const auto found = keyToRow.find(key);
        if (found == keyToRow.end())
        {
            OVX_RUNTIME_ERROR("ovxCommitGroup: a character controller was removed since this group was planned; "
                              "nothing was written.");
            return false;
        }
        resolved.push_back(currentRows[found->second]);
    }
    const float* values = reinterpret_cast<const float*>(slot.floats.data());
    const size_t count = size_t(slot.numOut) * slot.cctRow->components;
    for (size_t k = 0; k < count; ++k)
    {
        if (!PxIsFinite(values[k]))
        {
            OVX_RUNTIME_ERROR("ovxCommitGroup: character-controller '%s' values must be finite; "
                              "nothing was written.",
                              slot.cctRow->token);
            return false;
        }
    }
    if (cctMoveRejectedOnDirectGpu(resolved, "ovxCommitGroup"))
        return false;

    std::vector<float> timesteps(slot.numOut, 0.0f);
    for (uint32_t i = 0; i < slot.numOut; ++i)
    {
        const omni::physx::internal::InternalCct* cct = resolved[i].internal;
        const PhysXScene* scene = cct ? cct->mPhysXScene : nullptr;
        const uint32_t stepsPerSecond = scene ? scene->getTimeStepsPerSeconds() : 0;
        if (!resolved[i].controller || stepsPerSecond == 0)
        {
            OVX_RUNTIME_ERROR("ovxCommitGroup: character-controller '%s' has no simulator timestep; "
                              "nothing was written.",
                              slot.cctRow->token);
            return false;
        }
        timesteps[i] = 1.0f / static_cast<float>(stepsPerSecond);
    }

    t_scatterCommitAttempted = true;
    const PxControllerFilters filters;
    for (uint32_t i = 0; i < slot.numOut; ++i)
    {
        const float* v = values + size_t(i) * 3;
        const PxVec3 displacement(v[0], v[1], v[2]);
        // A finite zero still calls move and may ride a previously touched platform.
        // Sustained grounding needs caller-supplied downward displacement; no gravity is added here.
        // Collision flags stay unpublished. Exposing them is still an open product question.
        resolved[i].controller->move(displacement, 0.0f, timesteps[i], filters);
    }
    return true;
}

// A nonzero content version for a row list: equal lists resolved under the same view generation share it, so
// a held query's next session finds its parked copy current, and a drain that writes the same bodies again finds
// its copy in the view's row cache; neither uploads anything. The top bit is set, which the minted versions
// (nextOvStageRowsVersion) never reach, so a content version never equals a minted one.
uint64_t rowListVersion(const std::vector<PxU32>& rows, uint64_t generation)
{
    uint64_t hash = 1469598103934665603ull ^ generation;
    for (const PxU32 row : rows)
        hash = (hash ^ row) * 1099511628211ull;
    return hash | (1ull << 63);
}

// Publish one filled group to the engine: re-resolve its bodies to superset rows, then hand the
// column to the view method its attribute names.
//
// Rows are resolved HERE, not at plan time. A row indexes the superset view's ordering, and the
// backend takes a fresh generation whenever that view is rebuilt -- so a row cached across the
// caller's fill would address the view as it was when the group was handed out, and scatter this
// group's values into whatever occupies those rows now. One retry, with the row mapping inside it:
// a null view means the cache was found stale and dropped, so a second attempt rebuilds.
// Maps a rigid group's bodies onto their rows in the scene's superset view, under the generation the view reports
// now: the backend takes a fresh generation whenever it rebuilds that view, so a row is only meaningful under the
// generation it was resolved in. `rowsToken` identifies the list's contents to the view's device row cache.
bool resolveRigidRows(GroupSlot& slot,
                      bool sceneIsGpu,
                      std::vector<PxU32>& rows,
                      uint64_t& rowsToken,
                      uint64_t& generationOut,
                      omni::physx::tensors::GpuRigidBodyView*& gpuView,
                      omni::physx::tensors::CpuRigidBodyView*& cpuView)
{
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
    {
        OVX_RUNTIME_ERROR("ovxCommitGroup: no tensor backend.");
        return false;
    }
    gpuView = nullptr;
    cpuView = nullptr;
    rowsToken = 0;
    generationOut = 0;
    // Map this group's bodies onto their rows in the scene's superset view. Returns false when the
    // view does not describe this actor set, which the caller answers by dropping the cache and
    // trying once more.
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
                generationOut = generation;
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
                    ce->dbEpoch = omni::physx::internal::recordLifetimeEpoch(slot.scene);
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
                    // kOvxActive -- and kOvxAll while a disabled body is filtered out -- resolves its rows
                    // every commit, so its token is the list's content. A drain plans kOvxActive, and one
                    // that writes the same bodies again reuses the device copy of its list.
                    rowsToken = rowListVersion(rows, generation);
                }
                generationOut = generation;
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
        OVX_RUNTIME_ERROR("ovxCommitGroup: superset rigid-body view unavailable; nothing was written.");
        return false;
    }

    return true;
}

// The groups whose commit is ordered on the caller's stream (ADR-0052): DirectGPU device columns PhysX applies
// through its direct-GPU API. Everything else -- host columns, deformables and kinematic targets, whose
// application finishes on the host -- keeps the host wait.
bool isStreamOrderedKind(const GroupSlot& slot)
{
    if (slot.deviceOrdinal < 0 || !slot.ctxMgr || !slot.scene || slot.borrowed ||
        !slot.scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API))
        return false;
    if (slot.isDeformable)
        return slot.deformableKinematicTarget;
    if (slot.isParticle || slot.cctRow || slot.jointPropRow || slot.deformableMatRow || slot.vehicleRow ||
        slot.vehicleControlRow)
        return false;
    if (slot.row)
        return !slot.row->hostOnly;
    return slot.jointRow || slot.artiRow || slot.tendonRow || slot.isInstancer;
}

using OvStageRigidWrite = omni::physx::tensors::GpuRigidBodyView::OvStageRigidWrite;
using OvStageArtiWrite = omni::physx::tensors::GpuArticulationView::OvStageArtiWrite;

OvStageRigidWrite rigidWriteKind(const WriteAttributeRow& row, bool links)
{
    const std::string_view token(row.token);
    if (token == OvxAttr::kPosition || token == OvxAttr::kOrientation)
        return OvStageRigidWrite::ePose;
    if (token == OvxAttr::kWrench)
        return links ? OvStageRigidWrite::eLinkWrench : OvStageRigidWrite::eWrench;
    if (token == OvxAttr::kForce)
        return links ? OvStageRigidWrite::eLinkForce : OvStageRigidWrite::eVec3;
    return OvStageRigidWrite::eVec3; // a linear or angular velocity
}

// Sizes the group's scratch, keeping the parked one when it fits, and uploads `head` -- the list the view
// expects at its start -- unless the scratch already holds this version of it. Call under the slot's context
// lock.
bool ensureScratch(GroupSlot& slot, size_t bytes, const void* head, size_t headBytes, uint64_t headVersion)
{
    IOptionalCuda* const cuda = getCudaShim();
    if (!cuda)
        return false;
    bytes = std::max<size_t>(bytes, 1); // a zero-size allocation fails; a zero-width view still needs a pointer
    if (slot.scratchDev && slot.scratchBytes != bytes)
    {
        if (!waitBeforeHostWrite(slot) || !cuda->memFree(slot.scratchDev, nullptr))
        {
            slot.scratchDev = 0; // lost rather than freed twice
            return false;
        }
        slot.scratchDev = 0;
    }
    if (!slot.scratchDev)
    {
        slot.scratchBytes = 0;
        slot.scratchVersion = 0;
        uintptr_t scratch = 0;
        if (!cuda->memAlloc(&scratch, bytes, nullptr) || !scratch)
            return false;
        slot.scratchDev = static_cast<CUdeviceptr>(scratch);
        slot.scratchBytes = bytes;
    }
    if (headBytes && (headVersion == 0 || slot.scratchVersion != headVersion))
    {
        slot.scratchVersion = 0; // current again only once the copy has landed
        // Waited for as uploadRecords waits for its copy.
        if (!waitBeforeHostWrite(slot) || !cuda->memcpyHtoD(slot.scratchDev, head, headBytes, nullptr) ||
            !cuda->streamSynchronize(0, nullptr))
            return false;
        slot.scratchVersion = headVersion;
    }
    return true;
}

// The host work a kinematic-target group does once, when its write session opens, so its commit is one kernel and
// nothing on the host (REQ-INPUT-COVERAGE-001 AC-16). Sets up every body not set up yet -- a runtime-owned buffer
// holding each node free at its current position, adopted by PhysX with ePARTIALLY_KINEMATIC raised, which changes
// nothing until a commit pins a node -- and uploads the bodies' records at the head of the group's scratch. A body
// another path drives gets no record, and its commit refuses. A session reopened over bodies already set up uploads
// nothing: their records keep their content. Call under the slot's context lock, outside a capture.
bool prepareKinematicTargetGroup(GroupSlot& slot)
{
    PxCudaContext* const cu = slot.ctxMgr ? slot.ctxMgr->getCudaContext() : nullptr;
    IOptionalCuda* const cuda = getCudaShim();
    if (!cu || !cuda)
        return false;
    std::vector<omni::physx::tensors::KinematicTargetBodyRecord> records(slot.numOut);
    std::vector<uint32_t> fresh; // filled here, adopted by PhysX once the fill has landed
    uint32_t maxNodes = 0;
    for (uint32_t i = 0; i < slot.numOut; ++i)
    {
        PxDeformableVolume* const volume =
            slot.deformableBodies[i] ? slot.deformableBodies[i]->is<PxDeformableVolume>() : nullptr;
        omni::physx::internal::InternalVolumeDeformableBody* const internal =
            static_cast<omni::physx::internal::InternalVolumeDeformableBody*>(slot.deformableInternals[i]);
        if (!volume || !internal || slot.deformableFloatOffsets[i] > UINT32_MAX)
            return false;
        if (kinematicTargetDrivenElsewhere(*volume, *internal))
            continue;
        const uint32_t count = slot.deformableCounts[i];
        if (!internal->mKinematicTargetActive)
        {
            if (!internal->mSimMeshKinematicTargetsD)
            {
                CUdeviceptr buffer = 0;
                if (cu->memAlloc(&buffer, size_t(count) * sizeof(PxVec4)) != CUDA_SUCCESS || !buffer)
                {
                    OVX_RUNTIME_ERROR("ovxWriteAttributes: kinematic-target buffer allocation failed (%u nodes).",
                                      count);
                    return false;
                }
                internal->mSimMeshKinematicTargetsD = reinterpret_cast<PxVec4*>(buffer);
            }
            if (!omni::physx::tensors::initKinematicTargetBufferOvStage(internal->mSimMeshKinematicTargetsD,
                                                                        volume->getSimPositionInvMassBufferD(),
                                                                        count))
                return false;
            fresh.push_back(i);
        }
        omni::physx::tensors::KinematicTargetBodyRecord& record = records[i];
        record.dst = internal->mSimMeshKinematicTargetsD;
        record.floatOffset = static_cast<PxU32>(slot.deformableFloatOffsets[i]);
        record.nodeCount = count;
        // Targets arrive in the sim mesh's frame, the one the read publishes them in.
        const PxMat44d columnToBuffer = omni::physx::affineInverse(internal->mWorldToSimMesh);
        for (int c = 0; c < 16; ++c)
            record.columnToBuffer.m[c] = float(columnToBuffer.front()[c]);
        maxNodes = std::max(maxNodes, count);
    }
    if (!fresh.empty() && !cuda->streamSynchronize(0, nullptr))
        return false;
    for (const uint32_t i : fresh)
    {
        PxDeformableVolume* const volume = slot.deformableBodies[i]->is<PxDeformableVolume>();
        omni::physx::internal::InternalVolumeDeformableBody* const internal =
            static_cast<omni::physx::internal::InternalVolumeDeformableBody*>(slot.deformableInternals[i]);
        volume->setDeformableVolumeFlag(PxDeformableVolumeFlag::ePARTIALLY_KINEMATIC, true);
        volume->setKinematicTargetBufferD(internal->mSimMeshKinematicTargetsD);
        internal->mKinematicTargetActive = true;
    }

    const KinematicTargetScratchLayout layout(slot.numOut);
    uint64_t version = 1469598103934665603ull;
    const uint8_t* const bytes = reinterpret_cast<const uint8_t*>(records.data());
    for (size_t b = 0; b < layout.recordBytes; ++b)
        version = (version ^ bytes[b]) * 1099511628211ull;
    if (!ensureScratch(slot, layout.bytes, records.data(), layout.recordBytes, version | (1ull << 63)))
        return false;
    slot.kinematicTargetMaxNodes = maxNodes;
    return true;
}

// prepareKinematicTargetGroup for a group applied on the host: a GPU-dynamics scene without the direct-GPU API.
bool prepareHostKinematicTargetGroup(GroupSlot& slot)
{
    if (!slot.ctxMgr)
        return false;
    PxScopedCudaLock lock(*slot.ctxMgr);
    return prepareKinematicTargetGroup(slot);
}

// The host work a stream-ordered group does once, when the session opens, so its commit only queues device work
// (ADR-0052): resolves its rows, sizes its scratch and uploads the list at its head, lets the view prepare what
// it needs, and creates the events the commit orders with. Also what a commit reruns, outside a capture, when the
// view was rebuilt since. `roots`, for an articulation-root group at open, is its scene's plan, whose rows the
// session's other root attributes reuse.
bool prepareStreamOrderedGroup(GroupSlot& slot, WriteSession::RootPlan* roots = nullptr)
{
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    IOptionalCuda* const cuda = getCudaShim();
    if (!backend || !cuda)
        return false;
    uint64_t generation = 0;
    omni::physx::tensors::GpuSimulationView* sv = backend->acquireSceneView(slot.scene, &generation);
    if (!sv)
        return false;
    PxScopedCudaLock lock(*slot.ctxMgr);
    slot.prepared = omni::physx::tensors::OvStageDeviceWriteArgs();
    const auto scratchOf = [&slot]() { return reinterpret_cast<void*>(slot.scratchDev); };

    if (slot.row)
    {
        std::vector<PxU32> rows;
        uint64_t token = 0;
        omni::physx::tensors::GpuRigidBodyView* rigidView = nullptr;
        omni::physx::tensors::CpuRigidBodyView* unusedCpuView = nullptr;
        if (!resolveRigidRows(slot, true, rows, token, generation, rigidView, unusedCpuView))
            return false;
        const PxU32 n = static_cast<PxU32>(rows.size());
        // The GPU index map is refreshed here, so a commit captured right after the session opens has nothing left
        // to upload.
        if (!rigidView->refreshDisabledRowsOvStage())
            return false;
        const OvStageRigidWrite kind = rigidWriteKind(*slot.row, rigidView->ovStageRowsAreLinks(rows.data(), n));
        // `token` identifies the rows' contents: minted when a cached list changes, the list's content version
        // otherwise (resolveRigidRows), so a held query's reused scratch keeps its copy with no hash per open.
        if (!ensureScratch(slot, rigidView->ovStageWriteScratchBytes(kind, n), rows.data(), rows.size() * sizeof(PxU32),
                           token))
            return false;
        slot.prepared.scratch = scratchOf();
        slot.prepared.scratchBytes = slot.scratchBytes;
        // Waits for the last commit only when the preparation writes device memory that commit may still read: a
        // link write uploads which articulations it applies into the scratch, and a wrench refreshes a stale COM
        // table. Any other kind writes nothing there, so reopening a session each step over unchanged rows does not
        // block the host.
        const bool linkWrite = kind == OvStageRigidWrite::eLinkForce || kind == OvStageRigidWrite::eLinkWrench;
        const bool wrench = kind == OvStageRigidWrite::eWrench || kind == OvStageRigidWrite::eLinkWrench;
        if (((linkWrite || (wrench && !rigidView->getComsCacheStateValid())) && !waitBeforeHostWrite(slot)) ||
            !rigidView->prepareOvStageWrite(kind, rows.data(), n, slot.wholeSet, slot.prepared))
            return false;
        slot.disableEpoch = rigidView->rdDisableEpoch();
        slot.rigidDynamicRows = !linkWrite;
        slot.hostRows = std::move(rows);
        slot.hostRowsToken = token;
    }
    else if (slot.artiRow || slot.jointRow || slot.tendonRow)
    {
        const std::unordered_map<const PxArticulationReducedCoordinate*, PxU32>* artiRowMap = nullptr;
        const std::vector<omni::physx::tensors::ArticulationEntry>* entries = nullptr;
        omni::physx::tensors::GpuArticulationView* av = sv->supersetArticulationView(&artiRowMap, &entries);
        if (!av || !artiRowMap)
            return false;
        if (slot.artiRow)
        {
            // As a commit does, rebuilds once a view built before an articulation's scene insertion. The session's
            // first root attribute resolves the rows; the others take them under the same view generation.
            std::vector<PxU32> rows;
            uint64_t version = 0;
            if (roots && roots->rowsVersion != 0 && roots->rowsGeneration == generation &&
                roots->rows.size() == slot.artis.size())
            {
                rows = roots->rows;
                version = roots->rowsVersion;
            }
            else
            {
                if (!acquireArticulationRows(*backend, slot.scene, true, slot.artis, generation, entries, rows, &av))
                    return false;
                version = rowListVersion(rows, generation);
                if (roots)
                {
                    roots->rows = rows;
                    roots->rowsGeneration = generation;
                    roots->rowsVersion = version;
                }
            }
            const std::string_view token(slot.artiRow->token);
            const OvStageArtiWrite kind = (token == OvxAttr::kRootPosition || token == OvxAttr::kRootOrientation) ?
                                              OvStageArtiWrite::eRootPose :
                                              OvStageArtiWrite::eRootVelocity;
            if (!ensureScratch(slot, av->ovStageWriteScratchBytes(kind, static_cast<PxU32>(rows.size())), rows.data(),
                               rows.size() * sizeof(PxU32), version))
                return false;
            slot.hostRows = std::move(rows);
            slot.hostRowsToken = version;
        }
        else
        {
            // A masked joint commit selects DOFs, the group's rows (ADR-0059), so neither a joint nor a tendon write
            // uploads a list.
            const OvStageArtiWrite kind = slot.jointRow     ? OvStageArtiWrite::eDof :
                                          slot.tendonFixed ? OvStageArtiWrite::eFixedTendon :
                                                             OvStageArtiWrite::eSpatialTendon;
            if (!ensureScratch(slot, av->ovStageWriteScratchBytes(kind, slot.numOut), nullptr, 0, 0))
                return false;
        }
    }
    else if (slot.isInstancer)
    {
        if (!instancerViewCurrent(slot, generation))
            return false;
        omni::physx::tensors::GpuPointInstancerView* const instancerView =
            dynamic_cast<omni::physx::tensors::GpuPointInstancerView*>(slot.instancerView);
        const std::unordered_map<const PxRigidBody*, PxU32>* rowMap = nullptr;
        omni::physx::tensors::GpuRigidBodyView* rigidView = sv->supersetRigidView(&rowMap);
        const PxU32* rows = nullptr;
        PxU32 count = 0;
        if (!instancerView || !rigidView || !rigidView->refreshDisabledRowsOvStage() ||
            !instancerView->prepareInstancerWriteOvStage(slot.instancerIdx, rows, count))
            return false;
        using Column = omni::physx::tensors::BasePointInstancerView::InstancerColumn;
        const bool pose = slot.instancerColumn == Column::eLocalPosition ||
                          slot.instancerColumn == Column::eLocalOrientation;
        const std::vector<PxU32> rowList(rows, rows + count);
        if (!ensureScratch(slot,
                           rigidView->ovStageWriteScratchBytes(pose ? OvStageRigidWrite::eInstancerPose :
                                                                      OvStageRigidWrite::eInstancerVelocity,
                                                               count),
                           rowList.data(), rowList.size() * sizeof(PxU32), rowListVersion(rowList, generation)))
            return false;
        slot.disableEpoch = rigidView->rdDisableEpoch();
        slot.rigidDynamicRows = true;
        slot.hostRows = rowList;
    }
    else if (slot.deformableKinematicTarget)
    {
        // The count its commit adds kept pins to, allocated here so a captured commit allocates nothing.
        const omni::physx::tensors::GpuSimulationDataPtr gpuData = sv->getGpuSimulationData();
        if (!gpuData || !gpuData->keptPinnedTargetsCounter() || !prepareKinematicTargetGroup(slot))
            return false;
    }
    else
    {
        return false;
    }

    if ((!slot.forkEvent && !cuda->eventCreate(&slot.forkEvent, CU_EVENT_DISABLE_TIMING, nullptr)) ||
        (!slot.joinEvent && !cuda->eventCreate(&slot.joinEvent, CU_EVENT_DISABLE_TIMING, nullptr)) ||
        (!slot.readyEvent && !cuda->eventCreate(&slot.readyEvent, CU_EVENT_DISABLE_TIMING, nullptr)))
        return false;
    // The caller's fill of a reused column waits on the device for the commit that last used it.
    slot.group.data.cuda_sync = ovstage_cuda_sync_t{ 0, slot.readyEvent };
    slot.viewGeneration = generation;
    slot.streamOrdered = true;
    return true;
}

// `deviceArgs` is how a stream-ordered commit runs on the device (ADR-0052), null for the ovstage drain and the host
// paths. `hostMask` is a masked commit's selection of a host-resident group, one byte per row.
bool scatterGroup(AttachedStage& owner,
                  GroupSlot& slot,
                  const omni::physx::tensors::OvStageDeviceWriteArgs* deviceArgs = nullptr,
                  const uint8_t* hostMask = nullptr)
{
    if (slot.cctRow)
        return scatterCctGroup(owner, slot);
    if (slot.jointRow)
        return scatterJointGroup(slot, deviceArgs);
    if (slot.jointPropRow)
        return scatterJointPropertyGroup(slot);
    if (slot.isDeformable)
        return slot.deformableKinematicTarget ? scatterKinematicTargetGroup(slot, deviceArgs, hostMask) :
                                                scatterDeformableGroup(slot);
    if (slot.deformableMatRow)
        return scatterDeformableMaterialGroup(slot);
    if (slot.isParticle)
        return scatterParticleGroup(slot);
    if (slot.isInstancer)
        return scatterInstancerGroup(slot, deviceArgs, hostMask);
    if (slot.vehicleRow || slot.vehicleControlRow)
        return scatterVehicleGroup(slot);
    if (slot.tendonRow)
        return scatterTendonGroup(slot, deviceArgs);
    if (slot.artiRow)
        return scatterArticulationGroup(slot, deviceArgs);
    if (slot.bodies.empty())
        return true; // nothing to publish -- not a failure

    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (!backend)
    {
        OVX_RUNTIME_ERROR("ovxCommitGroup: no tensor backend.");
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

    std::vector<PxU32> rows;
    omni::physx::tensors::GpuRigidBodyView* gpuView = nullptr;
    omni::physx::tensors::CpuRigidBodyView* cpuView = nullptr;
    uint64_t rowsToken = 0;
    uint64_t generation = 0;
    if (slot.streamOrdered && deviceArgs)
    {
        // Rows resolved when the session opened (ADR-0052); the commit checked the view was not rebuilt
        // since. The setter refreshes the disabled rows itself, refusing under capture when that would upload.
        const std::unordered_map<const PxRigidBody*, PxU32>* rowMap = nullptr;
        omni::physx::tensors::GpuSimulationView* sv = backend->acquireSceneView(slot.scene);
        gpuView = sv ? sv->supersetRigidView(&rowMap) : nullptr;
        if (!gpuView)
        {
            OVX_RUNTIME_ERROR("ovxCommitGroup: superset rigid-body view unavailable; nothing was written.");
            return false;
        }
        t_scatterCommitAttempted = true;
        omni::physics::tensors::TensorDesc td;
        td.device = slot.deviceOrdinal;
        td.dtype = slot.row->dtype;
        td.numDims = 2;
        td.dims[0] = static_cast<int>(slot.hostRows.size());
        td.dims[1] = static_cast<int>(slot.comp);
        td.data = reinterpret_cast<void*>(slot.deviceData);
        return (gpuView->*slot.row->gpuWrite)(&td, slot.hostRows.data(), static_cast<uint32_t>(slot.hostRows.size()),
                                              slot.hostRowsToken, slot.wholeSet, *deviceArgs);
    }
    if (!resolveRigidRows(slot, sceneIsGpu, rows, rowsToken, generation, gpuView, cpuView))
        return false;

    if (sceneIsGpu)
    {
        // Probe the superset, not this group's filtered row list: a body disabled outside the
        // group still occupies a compacted DirectGPU slot. A refresh failure is a transient device-table
        // rewrite/sync failure on a selected device path -- before t_scatterCommitAttempted, so it would map to
        // NotHandled and fall back to the per-object setter (inert on DirectGPU, advancing the ordinal while
        // dropping the write). Set t_devicePrepFailed so it reports a retryable Failed instead (AC-12).
        if (!gpuView->refreshDisabledRowsOvStage())
        {
            OVX_RUNTIME_ERROR("ovxCommitGroup: DirectGPU row refresh failed; nothing was written.");
            t_devicePrepFailed = true;
            return false;
        }
    }

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
            (base->*slot.row->baseWrite)(&td, rows.data(), static_cast<uint32_t>(rows.size()), rowsToken, slot.wholeSet);
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
        return (gpuView->*slot.row->gpuWrite)(
            &td, rows.data(), static_cast<uint32_t>(rows.size()), rowsToken, slot.wholeSet,
            deviceArgs ? *deviceArgs : omni::physx::tensors::OvStageDeviceWriteArgs());
    return (cpuView->*slot.row->cpuWrite)(&td, rows.data(), static_cast<uint32_t>(rows.size()), rowsToken,
                                          slot.wholeSet);
}

// Free everything a session's groups own: the device column each was allocated, and the prim list
// each built. A device pointer is only meaningful in its own context, so the free must go through the
// manager the allocation came from.
//
// Each group holds its OWN list -- groups are per scene and no two scenes cover the same prims -- so
// there is nothing to refcount here, unlike the read, where several attributes share one list.
bool releaseGroupStorage(WriteSession& s)
{
    // Wait for any device write still in flight BEFORE freeing the columns it reads from. The RIGID
    // velocity paths hand the caller's column to PhysX directly, so a free here without this is a
    // use-after-free on device memory. The read blocks in the same place and for the same reason
    // (ovxReleaseRead): at release, never at the operation itself.
    //
    // These waits protect columns consumed directly by the rigid scene view. Joint and
    // articulation-root writes publish through PhysX-owned scratch instead; their own scatter
    // work retains the existing context-scoped deallocation path.
    bool released = true;
    std::vector<std::pair<PxScene*, bool>> sceneCompletion;
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    if (backend)
    {
        for (const std::unique_ptr<GroupSlot>& slot : s.groups)
        {
            // A stream-ordered group needs no wait here: its buffers are parked with the event its commit
            // recorded, which their next user waits on (ADR-0052).
            if (!slot || !slot->committed || slot->deviceOrdinal < 0 || !slot->row || slot->streamOrdered)
                continue;
            if (!slot->scene)
            {
                released = false;
                continue;
            }
            if (std::find_if(sceneCompletion.begin(), sceneCompletion.end(),
                             [&slot](const std::pair<PxScene*, bool>& entry) { return entry.first == slot->scene; }) !=
                sceneCompletion.end())
                continue;
            sceneCompletion.emplace_back(slot->scene, false);
            PhysXScene* sceneOwner = OmniPhysX::getInstance().getPhysXSetup().findPhysXScene(slot->scene);
            if (sceneOwner && sceneOwner->getAttachedStage().getAttachHandle() == s.attach &&
                sceneOwner->getAttachedStage().recordVersion.load(std::memory_order_relaxed) == s.recordEpoch)
            {
                omni::physx::tensors::GpuSimulationView* sv = backend->acquireSceneView(slot->scene);
                const std::unordered_map<const PxRigidBody*, PxU32>* rowMap = nullptr;
                omni::physx::tensors::GpuRigidBodyView* view = sv ? sv->supersetRigidView(&rowMap) : nullptr;
                sceneCompletion.back().second = view && view->waitOvStageWriteDone();
            }
            else if (slot->ctxMgr && slot->ctxMgr->getCudaContext())
            {
                // Retirement destroys the view and its events, and reset can reuse a scene address
                // within the same attachment. Confirm completion on the retained context when the
                // original scene generation no longer matches; teardown waits can have failed.
                PxScopedCudaLock lock(*slot->ctxMgr);
                IOptionalCuda* const cuda = getCudaShim();
                sceneCompletion.back().second = cuda && cuda->cudaAvailable() && cuda->ctxSynchronize(nullptr);
            }
            if (!sceneCompletion.back().second)
                released = false;
        }
    }

    if (!backend)
    {
        for (const std::unique_ptr<GroupSlot>& slot : s.groups)
            if (slot && slot->committed && slot->deviceOrdinal >= 0 && slot->row && !slot->streamOrdered)
                released = false;
    }
    if (!released)
    {
        OVX_RUNTIME_ERROR("Write release could not confirm device completion; affected committed rigid buffers are quarantined");
    }
    // Columns are parked for reuse only on a clean release, and only for a scene still live in this
    // attachment generation: anything else could never be taken again, so it is freed as before.
    const bool cleanRelease = released;
    auto sceneStillLive = [&s](PxScene* scene) {
        PhysXScene* sceneOwner = scene ? OmniPhysX::getInstance().getPhysXSetup().findPhysXScene(scene) : nullptr;
        return sceneOwner && sceneOwner->getAttachedStage().getAttachHandle() == s.attach &&
               sceneOwner->getAttachedStage().recordVersion.load(std::memory_order_relaxed) == s.recordEpoch;
    };
    // A captured commit's device work runs from the caller's graph and records no completion the session could
    // wait on. Release waits for each such context once, which covers every replay launched before it; a graph
    // replayed after its session is released reads buffers the session no longer owns (ADR-0052).
    std::vector<std::pair<PxCudaContextManager*, bool>> replaysSettled;
    // Whether `ctxMgr` was waited for, and with what outcome; null when it never was.
    const auto settledEntry =
        [&replaysSettled](const PxCudaContextManager* ctxMgr) -> const std::pair<PxCudaContextManager*, bool>* {
        for (const std::pair<PxCudaContextManager*, bool>& entry : replaysSettled)
        {
            if (entry.first == ctxMgr)
                return &entry;
        }
        return nullptr;
    };
    for (const std::unique_ptr<GroupSlot>& slot : s.groups)
    {
        if (!slot || !slot->captured || !slot->ctxMgr || settledEntry(slot->ctxMgr))
            continue;
        bool settled = false;
        if (slot->ctxMgr->getCudaContext())
        {
            PxScopedCudaLock lock(*slot->ctxMgr);
            IOptionalCuda* const cuda = getCudaShim();
            settled = cuda && cuda->cudaAvailable() && cuda->ctxSynchronize(nullptr);
        }
        replaysSettled.emplace_back(slot->ctxMgr, settled);
    }
    // Pinned nodes a kinematic-target commit kept pinned, counted on the device: what has landed by now, the replays
    // waited for above included, is logged here; the rest at a later commit or release.
    for (const std::unique_ptr<GroupSlot>& slot : s.groups)
    {
        if (!backend || !slot || !slot->committed || !slot->deformableKinematicTarget || !slot->streamOrdered)
            continue;
        if (omni::physx::tensors::GpuSimulationView* const sv = backend->currentSceneView(slot->scene))
            if (const omni::physx::tensors::GpuSimulationDataPtr gpuData = sv->getGpuSimulationData())
                gpuData->reportKeptPinnedTargets();
    }
    for (std::unique_ptr<GroupSlot>& slot : s.groups)
    {
        if (!slot)
            continue; // quarantined by an earlier pass
        slot->releaseList();
        const std::pair<PxCudaContextManager*, bool>* const settled =
            slot->captured ? settledEntry(slot->ctxMgr) : nullptr;
        if (slot->captured && !(settled && settled->second))
        {
            // Nothing proves the replays reading this slot's buffers finished, including when its context could
            // not be waited for at all: kept, as a quarantined group is.
            released = false;
            OVX_RUNTIME_ERROR("Write release could not confirm that the replays of a captured commit finished; its "
                              "buffers are quarantined");
            slot->columnCache.reset();
            static_cast<void>(slot.release());
            continue;
        }
        if (slot->producerUnproven)
        {
            // An accepted commit whose producer handoff was never proven complete. The caller's stream
            // or event may still be writing this slot's storage, and the caller may already have
            // destroyed those borrowed handles, so there is nothing left to wait on. Keep everything --
            // host floats, device column, record buffer and the context reference -- by leaking the
            // whole slot on purpose: its destructor must not run. Allocation-free, so it cannot fail.
            released = false;
            OVX_RUNTIME_ERROR("Write release kept a committed group's storage: its producer handoff was never "
                              "proven complete, so the buffer is quarantined");
            // Only this slot's own storage is kept: the query's other parked buffers still go with the query.
            slot->columnCache.reset();
            static_cast<void>(slot.release());
            continue;
        }
        bool deviceUseComplete = true;
        if (slot->committed && slot->isDeformable && !slot->deformableKinematicTarget)
        {
            // The deformable scatter consumes the column on the default stream. It has no
            // rigid-view completion event, and release must check completion before freeing it.
            deviceUseComplete = false;
            if (slot->ctxMgr && slot->ctxMgr->getCudaContext())
            {
                PxScopedCudaLock lock(*slot->ctxMgr);
                IOptionalCuda* const cuda = getCudaShim();
                deviceUseComplete = cuda && cuda->cudaAvailable() && cuda->streamSynchronize(0, nullptr);
            }
            if (!deviceUseComplete)
            {
                released = false;
                OVX_RUNTIME_ERROR("Write release could not confirm deformable scatter completion; buffers are quarantined");
            }
        }
        // A borrowed column is the caller's (the ovstage drain's zero-copy path) -- deviceData points
        // into it, so the session must not free it. recsDev, when present, is always session-owned.
        if (slot->deviceData && !slot->borrowed)
        {
            // A failed rigid scene wait cannot quarantine an uncommitted group or a group whose
            // own scene completed. Deformable columns use the default-stream result above.
            if (slot->committed && slot->deviceOrdinal >= 0 && slot->row && !slot->streamOrdered)
            {
                const auto completion = std::find_if(
                    sceneCompletion.begin(), sceneCompletion.end(),
                    [&slot](const std::pair<PxScene*, bool>& entry) { return entry.first == slot->scene; });
                deviceUseComplete = completion != sceneCompletion.end() && completion->second;
            }
            PxCudaContext* cu = slot->ctxMgr ? slot->ctxMgr->getCudaContext() : nullptr;
            if (cu && deviceUseComplete)
            {
                PxScopedCudaLock lock(*slot->ctxMgr);
                // A column from the query's cache is parked for the next session instead of freed. The free
                // synchronised the device implicitly, which is what guaranteed that every scatter reading
                // the column -- and any fill still running on the caller's stream -- had finished; the
                // scene wait above covers only the rigid-dynamic apply. Wait for the context once, as the
                // free did, before the next session can hand the column out to be filled again.
                bool parked = false;
                const ovx::QueryDeviceBuffers::Buffer owned{ slot->deviceData, slot->recsDev, slot->recsVersion,
                                                             slot->scratchDev, slot->scratchBytes,
                                                             slot->scratchVersion, slot->readyEvent };
                if (slot->columnCache && cleanRelease && sceneStillLive(slot->scene))
                {
                    // A stream-ordered group parks its buffers at once, with the event its commit recorded
                    // (ADR-0052). Any other group waits for the context once, as a free did, before the next
                    // session can hand the column out to be filled again.
                    IOptionalCuda* const cuda = getCudaShim();
                    const bool settled =
                        slot->streamOrdered || (cuda && cuda->cudaAvailable() && cuda->ctxSynchronize(nullptr));
                    if (!settled)
                    {
                        // Freed below instead: the free waits for the device by itself. The wait's failure is
                        // still the release's.
                        released = false;
                        OVX_RUNTIME_ERROR("Write release could not confirm device completion before keeping a "
                                          "column for reuse; the column is freed instead");
                    }
                    else
                    {
                        // The cache owns the buffers from here: parked, or freed when the key was taken.
                        if (!slot->columnCache->give(slot->columnKey, slot->ctxMgr, slot->columnBytes, owned))
                        {
                            released = false;
                            OVX_RUNTIME_ERROR("Write release could not reclaim a CUDA buffer");
                        }
                        parked = true;
                    }
                }
                if (!parked && !ovx::QueryDeviceBuffers::freeBuffer(owned))
                {
                    released = false;
                    OVX_RUNTIME_ERROR("Write release could not reclaim a CUDA buffer");
                }
            }
            else if (!cu)
            {
                released = false;
                OVX_RUNTIME_ERROR("Write release lost its CUDA context; buffers are quarantined");
            }
            slot->deviceData = 0;
            slot->recsDev = 0;
            slot->scratchDev = 0;
            slot->readyEvent = 0;
        }
        // The fork and join events are the session's; the destructor destroys them.
    }
    return released;
}

} // namespace

static AttachHandle writeOwner(OvxWriteHandle write)
{
    std::lock_guard<carb::tasking::RecursiveMutexWrapper> lock(omni::physx::ovx::outputCacheMutex());
    auto it = g_writes.find(write);
    if (it == g_writes.end())
        return omni::physics::kNoAttach;
    // Debug settings follow the instance's current attach, matching queryOwner().
    if (it->second.debugRender)
        return debug_render::currentAttach(it->second.debugOwner);
    return it->second.attach;
}
static AttachHandle writeQueryOwner(OvxOutputQueryHandle query)
{
    uint64_t attach = 0;
    uint32_t type = 0, scope = 0;
    omni::physx::ovx::queryTypeScope(query, attach, type, scope);
    return attach;
}

namespace
{
// The rows a masked commit selects among (ADR-0052): a ragged group's tensors -- one per deformable body -- or the
// leading dimension of a fixed group's single tensor, which for a joint group is one row per DOF (ADR-0059).
uint32_t maskRows(const GroupSlot& slot)
{
    if (slot.isDeformable || slot.tensors.empty())
        return static_cast<uint32_t>(slot.tensors.size());
    return static_cast<uint32_t>(slot.shapes.empty() ? 0 : slot.shapes[0]);
}

// A masked commit's selection: a contiguous 1-D bool or uint8 tensor with one entry per row, on the group's
// device (host memory for a host-resident group). Null when it is acceptable, else why not.
const char* rowMaskRejection(const GroupSlot& slot, const DLTensor& mask)
{
    // The kinds the tensor binding's masked write served take a mask; these never had one.
    if (slot.cctRow || slot.vehicleRow || slot.vehicleControlRow || slot.isParticle)
        return "character controller, vehicle and particle groups take no row mask";
    if (!mask.data && maskRows(slot) != 0)
        return "its data is null";
    if (mask.ndim != 1 || !mask.shape)
        return "it must be one-dimensional";
    if (mask.dtype.lanes != 1 || mask.dtype.bits != 8 || (mask.dtype.code != kDLBool && mask.dtype.code != kDLUInt))
        return "its dtype must be bool or uint8";
    if (mask.shape[0] != static_cast<int64_t>(maskRows(slot)))
        return "it must hold one entry per row of the group";
    if (mask.strides && mask.shape[0] > 1 && mask.strides[0] != 1)
        return "it must be contiguous";
    const bool deviceGroup = slot.deviceOrdinal >= 0;
    if (deviceGroup && (mask.device.device_type != kDLCUDA || mask.device.device_id != slot.deviceOrdinal))
        return "it must live on the group's CUDA device";
    if (!deviceGroup && mask.device.device_type != kDLCPU && mask.device.device_type != kDLCUDAHost)
        return "it must live in host memory, as the group does";
    return nullptr;
}

const uint8_t* maskBytes(const DLTensor* mask)
{
    return mask ? static_cast<const uint8_t*>(mask->data) + mask->byte_offset : nullptr;
}

// How the stream a commit is given stands with a CUDA graph capture (ADR-0052).
enum class CommitStreamCapture
{
    eNone,
    // The stream is being captured.
    eCapturing,
    // The stream is the legacy default stream while a blocking stream of its context is being captured: CUDA refuses
    // any use of it, and the attempt would invalidate that capture (currentLegacyStreamHeldByCapture).
    eLegacyHeld,
    // The driver could not say whether the stream is being captured.
    eUnknown,
};

// How a commit of `slot` given `sync.stream` stands with a capture. The stream itself is asked first. A group in
// device memory then runs its device work on the legacy default stream of the scene's context unless it is ordered on
// a stream of that context -- given a zero stream, applied on the host after its host wait, or given a stream of
// another context -- so that stream is asked too, with the context current. A group applied on the host uses the
// stream only for its host wait. Without a CUDA driver nothing can be captured: the commit reports the missing driver
// when it waits for the stream.
CommitStreamCapture commitStreamCapture(const GroupSlot& slot, const ovstage_cuda_sync_t& sync)
{
    IOptionalCuda* const cuda = getCudaShim();
    if (!cuda)
        return CommitStreamCapture::eNone;
    if (sync.stream)
    {
        bool capturing = false;
        int status = 0;
        if (!cuda->streamIsCapturing(sync.stream, &capturing, &status))
        {
            if (status == CUDA_ERROR_STREAM_CAPTURE_IMPLICIT)
                return CommitStreamCapture::eLegacyHeld;
            return cuda->cudaAvailable() ? CommitStreamCapture::eUnknown : CommitStreamCapture::eNone;
        }
        if (capturing)
            return CommitStreamCapture::eCapturing;
    }
    if (slot.deviceOrdinal < 0 || !slot.ctxMgr)
        return CommitStreamCapture::eNone;
    uintptr_t streamContext = 0;
    if (sync.stream && slot.streamOrdered && cuda->streamGetCtx(sync.stream, &streamContext, nullptr) &&
        streamContext == reinterpret_cast<uintptr_t>(slot.ctxMgr->getContext()))
        return CommitStreamCapture::eNone;
    PxScopedCudaLock contextLock(*slot.ctxMgr);
    return currentLegacyStreamHeldByCapture() ? CommitStreamCapture::eLegacyHeld : CommitStreamCapture::eNone;
}

// Whether a commit of the live group `slot` is refused as given, before anything is read or spent: a row mask
// that does not fit the group, a group applied on the host committed on a capturing stream -- its host wait for the
// stream would fail and end the capture -- any group whose stream or device work is the legacy default stream while a
// capture holds it (commitStreamCapture), or a stream whose capture status the driver cannot report. Reports the cause
// as a runtime error; the group stays live. Shared by the commit and ovxValidateCommit, so a caller can refuse the
// commit before admitting it. Call under the output-cache mutex.
bool commitRefused(const WriteSession& s, const GroupSlot& slot, const DLTensor* mask, const ovstage_cuda_sync_t& sync)
{
    if (s.debugRender)
    {
        if (mask)
        {
            OVX_RUNTIME_ERROR("ovxCommitGroupMasked: a debug-render write has no rows to select; commit it unmasked.");
            return true;
        }
        switch (commitStreamCapture(slot, sync))
        {
        case CommitStreamCapture::eCapturing:
            OVX_RUNTIME_ERROR("ovxCommitGroup: a debug-render write is applied on the host and cannot be committed on "
                              "a stream being captured into a CUDA graph; nothing was read or written. The group is "
                              "still live: commit it outside the capture.");
            return true;
        case CommitStreamCapture::eLegacyHeld:
            OVX_RUNTIME_ERROR("ovxCommitGroup: a debug-render write cannot use the legacy default stream while a "
                              "blocking stream of its CUDA context is being captured into a CUDA graph, and using it "
                              "would invalidate that capture; nothing was read or written. The group is still live: "
                              "commit it on another stream, or after the capture.");
            return true;
        case CommitStreamCapture::eUnknown:
            OVX_RUNTIME_ERROR("ovxCommitGroup: whether stream 0x%llx is being captured into a CUDA graph could not be "
                              "determined; nothing was read or written. The group is still live.",
                              static_cast<unsigned long long>(sync.stream));
            return true;
        case CommitStreamCapture::eNone:
            break;
        }
        return false;
    }
    const char* const attributeName = s.attributeNames[slot.attributeIndex].c_str();
    if (mask)
    {
        if (const char* why = rowMaskRejection(slot, *mask))
        {
            OVX_RUNTIME_ERROR("ovxCommitGroupMasked: request %zu ('%.128s'): the row mask is rejected: %s. The group "
                              "is still live.",
                              slot.attributeIndex, attributeName, why);
            return true;
        }
    }
    const CommitStreamCapture capture = commitStreamCapture(slot, sync);
    if (!slot.streamOrdered && capture == CommitStreamCapture::eCapturing)
    {
        OVX_RUNTIME_ERROR("ovxCommitGroup: request %zu ('%.128s'): a group applied on the host cannot be committed on "
                          "a stream being captured into a CUDA graph; nothing was read or written. The group is still "
                          "live: commit it outside the capture.",
                          slot.attributeIndex, attributeName);
        return true;
    }
    if (capture == CommitStreamCapture::eLegacyHeld)
    {
        OVX_RUNTIME_ERROR("ovxCommitGroup: request %zu ('%.128s'): the commit would use the legacy default stream of a "
                          "CUDA context while a blocking stream of that context is being captured into a CUDA graph, "
                          "which CUDA refuses and which would invalidate that capture; nothing was read or written. "
                          "The group is still live: commit it after the capture or, for a DirectGPU device group, on "
                          "another stream of the scene's context.",
                          slot.attributeIndex, attributeName);
        return true;
    }
    if (capture == CommitStreamCapture::eUnknown)
    {
        OVX_RUNTIME_ERROR("ovxCommitGroup: request %zu ('%.128s'): whether stream 0x%llx is being captured into a CUDA "
                          "graph could not be determined; nothing was read or written. The group is still live.",
                          slot.attributeIndex, attributeName, static_cast<unsigned long long>(sync.stream));
        return true;
    }
    // Refused before the commit resolves its stream, which a driver older than CUDA 12.5 refuses for a stream being
    // captured, invalidating the capture (resolveCallerStream).
    if (capture == CommitStreamCapture::eCapturing && !driverSupportsCapture(cudaDriverVersion()))
    {
        const int driverVersion = cudaDriverVersion();
        OVX_RUNTIME_ERROR("ovxCommitGroup: request %zu ('%.128s'): stream 0x%llx is being captured into a CUDA graph, "
                          "which needs an NVIDIA driver for CUDA 12.5 or newer, and this driver supports CUDA %d.%d; "
                          "nothing was read or written. The group is still live and the capture still valid.",
                          slot.attributeIndex, attributeName, static_cast<unsigned long long>(sync.stream),
                          driverVersion / 1000, driverVersion % 1000 / 10);
        return true;
    }
    return false;
}

// A masked commit's selection, on the host, for a group applied there. Called after the producer handoff, so a
// device mask is complete when it is copied.
bool hostRowMask(const GroupSlot& slot, const DLTensor& mask, std::vector<uint8_t>& out)
{
    const uint32_t rows = maskRows(slot);
    out.assign(rows, 0);
    if (rows == 0)
        return true;
    const uint8_t* const src = maskBytes(&mask);
    if (mask.device.device_type == kDLCUDA)
    {
        IOptionalCuda* const cuda = getCudaShim();
        if (!cuda || !slot.ctxMgr)
            return false;
        PxScopedCudaLock contextLock(*slot.ctxMgr);
        return cuda->memcpyDtoH(out.data(), reinterpret_cast<uintptr_t>(src), rows, nullptr);
    }
    std::memcpy(out.data(), src, rows);
    return true;
}

// Narrows a group applied on the host to the rows a masked commit selects (ADR-0052), in place: the group is spent,
// and its setter then writes exactly the rows it still holds, leaving every other row as it is. Each kind keeps
// its rows in its own lists; a fixed column is packed down to match, a ragged one keeps its offsets.
bool applyHostRowMask(GroupSlot& slot, const uint8_t* mask)
{
    const uint32_t rows = maskRows(slot);
    std::vector<size_t> keep;
    keep.reserve(rows);
    for (uint32_t r = 0; r < rows; ++r)
        if (mask[r])
            keep.push_back(r);
    // The point instancer and the kinematic target take the mask themselves: the instancer addresses instances inside
    // one row, and the kinematic target's records were laid out over every body at open.
    if (keep.size() == rows || slot.isInstancer || slot.deformableKinematicTarget)
        return true;
    const uint32_t kept = static_cast<uint32_t>(keep.size());
    // Moves the kept rows of a fixed column to its front, in order.
    const auto packRows = [&keep](void* data, size_t rowBytes) {
        char* const base = static_cast<char*>(data);
        for (size_t i = 0; i < keep.size(); ++i)
            if (keep[i] != i)
                std::memmove(base + i * rowBytes, base + keep[i] * rowBytes, rowBytes);
    };
    const auto setFixedRows = [&slot, kept]() {
        if (!slot.shapes.empty())
            slot.shapes[0] = static_cast<int64_t>(kept);
    };

    if (slot.row)
    {
        const size_t elem = slot.row->dtype == omni::physics::tensors::TensorDataType::eUint8 ? 1u : 4u;
        packRows(slot.floats.data(), size_t(slot.comp) * elem);
        slot.bodies = ovx::gatherByIndices(slot.bodies, keep);
        slot.keys = ovx::gatherByIndices(slot.keys, keep);
        // A subset must neither refresh the shared row cache nor take the whole-view fast path.
        slot.cacheable = false;
        slot.wholeSet = false;
        setFixedRows();
        return true;
    }
    if (slot.artiRow)
    {
        packRows(slot.floats.data(), size_t(slot.comp) * sizeof(float));
        slot.artis = ovx::gatherByIndices(slot.artis, keep);
        slot.keys = ovx::gatherByIndices(slot.keys, keep);
        setFixedRows();
        return true;
    }
    if (slot.tendonRow)
    {
        const std::vector<omni::physx::tensors::ArticulationTendonOvStageRecord>& recs =
            slot.tendonRecsHost.empty() ? ovx::tendonCacheEntry(slot.scene, slot.tendonFixed).recs :
                                          slot.tendonRecsHost;
        if (recs.size() != rows)
            return false;
        slot.tendonRecsHost = ovx::gatherByIndices(recs, keep);
        packRows(slot.floats.data(), size_t(slot.comp) * sizeof(float));
        slot.numOut = kept;
        setFixedRows();
        return true;
    }
    if (slot.deformableMatRow)
    {
        packRows(slot.floats.data(), size_t(slot.comp) * sizeof(float));
        slot.materials = ovx::gatherByIndices(slot.materials, keep);
        slot.keys = ovx::gatherByIndices(slot.keys, keep);
        slot.numOut = kept;
        setFixedRows();
        return true;
    }
    if (slot.jointRow || slot.jointPropRow)
    {
        // The records a host joint scatter reads: the session's own when a selection compacted them, else the
        // shared cache's, which describe the whole column.
        const ovx::ArticulationReadCacheEntry* jc = ovx::articulationCacheEntry(slot.scene);
        const std::vector<omni::physx::tensors::ArticulationDofOvStageRecord>* recs =
            !slot.recsHost.empty() ? &slot.recsHost : (jc ? &jc->recs : nullptr);
        if (!recs || recs->size() != rows)
            return false;
        // A row is one DOF (ADR-0059), so the kept rows are the kept outputs.
        const size_t outputBytes = slot.jointPropRow ?
            size_t(slot.comp) * (omni::physx::tensors::dofPropertyIsByte(slot.jointPropRow->prop) ? 1u : 4u) :
            sizeof(float);
        slot.recsHost = ovx::gatherByIndices(*recs, keep);
        packRows(slot.floats.data(), outputBytes);
        slot.numOut = kept;
        setFixedRows();
        return true;
    }
    if (slot.isDeformable)
    {
        slot.deformableBodies = ovx::gatherByIndices(slot.deformableBodies, keep);
        slot.deformableInternals = ovx::gatherByIndices(slot.deformableInternals, keep);
        slot.deformableCounts = ovx::gatherByIndices(slot.deformableCounts, keep);
        slot.deformableFloatOffsets = ovx::gatherByIndices(slot.deformableFloatOffsets, keep);
        slot.numOut = kept;
        return true;
    }
    return false;
}

// Re-resolves a stream-ordered group whose view was rebuilt since the session opened, which a capture cannot do,
// and refuses a captured commit whose rigid rows lost or regained their DirectGPU index since (ADR-0052).
bool streamOrderedGroupCurrent(GroupSlot& slot, bool capturing, const char* attributeName)
{
    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    uint64_t generation = 0;
    omni::physx::tensors::GpuSimulationView* sv =
        backend ? backend->acquireSceneView(slot.scene, &generation) : nullptr;
    if (!sv)
    {
        OVX_RUNTIME_ERROR("ovxCommitGroup: '%s': the scene's GPU view is unavailable; nothing was written.",
                          attributeName);
        return false;
    }
    if (generation != slot.viewGeneration)
    {
        if (capturing)
        {
            OVX_RUNTIME_ERROR("ovxCommitGroup: '%s': the scene changed since this write was opened, and a captured "
                              "commit cannot resolve its rows again. Reopen the write and capture again; nothing "
                              "was written.",
                              attributeName);
            return false;
        }
        if (!prepareStreamOrderedGroup(slot))
        {
            OVX_RUNTIME_ERROR("ovxCommitGroup: '%s': the group could not be resolved against the rebuilt scene; "
                              "nothing was written.",
                              attributeName);
            return false;
        }
    }
    if (capturing && slot.rigidDynamicRows)
    {
        const std::unordered_map<const PxRigidBody*, PxU32>* rowMap = nullptr;
        omni::physx::tensors::GpuRigidBodyView* rigidView = sv->supersetRigidView(&rowMap);
        if (!rigidView || rigidView->rdDisableEpoch() != slot.disableEpoch)
        {
            OVX_RUNTIME_ERROR("ovxCommitGroup: '%s': a rigid body was enabled or disabled since this write was "
                              "opened, and a captured commit cannot follow its new index. Reopen the write and "
                              "capture again; nothing was written.",
                              attributeName);
            return false;
        }
    }
    return true;
}

// Records `slot.readyEvent` after everything the caller queued for a spent stream-ordered group: its stream and wait
// event. The next user of the group's buffers waits for that event. False on a capturing stream, which records nothing
// the host could wait on, or when the record fails.
bool handOnProducer(const GroupSlot& slot, const CallerStream& caller)
{
    IOptionalCuda* const cuda = getCudaShim();
    if (caller.capturing || !cuda || !slot.ctxMgr || !slot.readyEvent)
        return false;
    PxScopedCudaLock contextLock(*slot.ctxMgr);
    return (!caller.waitEvent || cuda->streamWaitEvent(caller.stream, caller.waitEvent, 0, nullptr)) &&
           cuda->eventRecord(slot.readyEvent, caller.stream, nullptr);
}

// The commit both entry points share. `mask` is a masked commit's row selection (ADR-0052); null writes every row.
bool commitGroup(OvxWriteHandle write,
                 const ovstage_map_group_t* group,
                 const DLTensor* mask,
                 ovstage_cuda_sync_t writeDoneSync,
                 OvxCommitFailure* outFailure)
{
    RuntimeExecutionScope execution = RuntimeExecutionScope::output(writeOwner(write));
    if (!execution)
    {
        if (outFailure)
            *outFailure = kOvxCommitFailureBusy;
        return false;
    }

    const auto fail = [outFailure](OvxCommitFailure why)
    {
        if (outFailure)
            *outFailure = why;
        return false;
    };

    std::unique_lock<carb::tasking::RecursiveMutexWrapper> lock(omni::physx::ovx::outputCacheMutex());
    WriteSession* s = findSession(write);
    if (s && s->debugRender)
    {
        if (s->groups.size() != 1 || group != &s->groups[0]->group || s->groups[0]->committed)
            return fail(kOvxCommitFailureNotLive);
        GroupSlot* slot = s->groups[0].get();
        if (commitRefused(*s, *slot, mask, writeDoneSync))
            return fail(kOvxCommitFailureInvalidArgument);
        slot->tensors[0].shape = slot->shapes.data();
        // Spent before publication, including a rejected value, so a second commit is refused.
        slot->committed = true;
        // Wait for the caller's producer before the payload is read, including a host column.
        if (!honourCallerSync(*slot, s->attribute.c_str(), writeDoneSync))
            return fail(kOvxCommitFailurePublish);
        debug_render::WritePlan plan;
        plan.kind = s->debugKind;
        plan.param = s->debugParam;
        std::string error;
        if (!debug_render::commit(s->debugOwner, plan, slot->tensors[0], error))
        {
            OVX_RUNTIME_ERROR("%s", error.c_str());
            return fail(kOvxCommitFailurePublish);
        }
        debug_render::applyStoredHere(debug_render::copyState(s->debugOwner));
        return true;
    }
    AttachedStage* owner = s ? UsdLoad::getUsdLoad()->getAttachedStageByHandle(s->attach) : nullptr;
    if (!owner || owner->getInternalPhysXDatabase().recordEpoch() != s->recordEpoch)
        return fail(kOvxCommitFailureNotLive);

    GroupSlot* slot = findLiveGroup(*s, group);
    if (!slot)
        return fail(kOvxCommitFailureNotLive); // unknown, foreign, or already committed
    if (commitRefused(*s, *slot, mask, writeDoneSync))
        return fail(kOvxCommitFailureInvalidArgument);
    const char* const attributeName = s->attributeNames[slot->attributeIndex].c_str();

    // Marked committed BEFORE the publish so a second commit of the same pointer is refused even if
    // the publish itself fails: commit is not retryable through this path.
    slot->committed = true;
    // Only a stream-ordered group resolves the caller's stream; a group applied on the host drains it, a capturing
    // one having been refused above. Resolved before the scene's context is pushed, as the host drain resolves it.
    CallerStream caller;
    const uintptr_t sceneContext = slot->ctxMgr ? reinterpret_cast<uintptr_t>(slot->ctxMgr->getContext()) : 0;
    if (slot->streamOrdered)
    {
        int status = 0;
        if (!resolveCallerStream(writeDoneSync.stream, writeDoneSync.wait_event, caller, status))
        {
            slot->producerUnproven = true;
            OVX_RUNTIME_ERROR("ovxCommitGroup: request %zu ('%.128s'): the producer stream 0x%llx could not be "
                              "resolved (CUresult %d); nothing was read or written.",
                              slot->attributeIndex, attributeName,
                              static_cast<unsigned long long>(writeDoneSync.stream), status);
            return fail(kOvxCommitFailurePublish);
        }
        // Spent and kept at release, unlike a host group refused on a capturing stream: the caller's fill is most
        // likely captured on this stream too, so each replay of its graph writes this column, from a context the
        // release cannot wait on. Parked for reuse, the column could be overwritten under the next session. Captured
        // work cannot write a host group's column: a copy into pageable host memory cannot be captured.
        if (caller.capturing && !caller.sharesContext(sceneContext))
        {
            slot->producerUnproven = true;
            OVX_RUNTIME_ERROR("ovxCommitGroup: request %zu ('%.128s'): the producer stream 0x%llx is being captured "
                              "but belongs to another CUDA context than the scene's, so the capture cannot record "
                              "this commit; nothing was read or written. Capture on a stream of the scene's context.",
                              slot->attributeIndex, attributeName,
                              static_cast<unsigned long long>(writeDoneSync.stream));
            return fail(kOvxCommitFailurePublish);
        }
        // A stream of another context cannot carry the commit (CallerStream.h): its producer is waited for on the
        // host, as for a group applied there, and the commit runs on the scene's own stream.
        if (!caller.sharesContext(sceneContext))
        {
            lock.unlock();
            if (!honourCallerSync(*slot, attributeName, writeDoneSync))
                return fail(kOvxCommitFailurePublish);
            lock.lock();
            caller = CallerStream{};
        }
    }
    // Fails a spent group before its completion is recorded. Nothing on the host waited for a stream-ordered group's
    // producer, which may still be writing its buffers: their next user waits for it instead, or release quarantines
    // them when that cannot be arranged.
    const auto failSpent = [&]()
    {
        if (slot->streamOrdered && !handOnProducer(*slot, caller))
            slot->producerUnproven = true;
        return fail(kOvxCommitFailurePublish);
    };

    omni::physx::tensors::OvStageDeviceWriteArgs deviceArgs;
    std::vector<uint8_t> hostMask;
    bool unjoined = false;
    if (slot->streamOrdered)
    {
        // Ordered on the device after the caller's stream (ADR-0052): the producer handoff becomes a device wait,
        // nothing here blocks the host, and a capture records the work instead of running it.
        if (!streamOrderedGroupCurrent(*slot, caller.capturing, attributeName))
            return failSpent();
        deviceArgs = slot->prepared;
        deviceArgs.fork = reinterpret_cast<CUevent>(slot->forkEvent);
        deviceArgs.join = reinterpret_cast<CUevent>(slot->joinEvent);
        deviceArgs.mask = maskBytes(mask);
        deviceArgs.scratch = reinterpret_cast<void*>(slot->scratchDev);
        deviceArgs.scratchBytes = slot->scratchBytes;
        deviceArgs.capturing = caller.capturing;
        deviceArgs.unjoined = &unjoined;

        PxScopedCudaLock contextLock(*slot->ctxMgr);
        IOptionalCuda* const cuda = getCudaShim();
        int status = 0;
        if (caller.waitEvent && !cuda->streamWaitEvent(caller.stream, caller.waitEvent, 0, &status))
        {
            slot->producerUnproven = true;
            OVX_RUNTIME_ERROR("ovxCommitGroup: request %zu ('%.128s'): ordering after the producer event 0x%llx "
                              "failed (CUresult %d); nothing was read or written.",
                              slot->attributeIndex, attributeName,
                              static_cast<unsigned long long>(caller.waitEvent), status);
            return fail(kOvxCommitFailurePublish);
        }
        deviceArgs.stream = reinterpret_cast<CUstream>(caller.stream);
    }
    else
    {
        // The producer handoff, exactly once and BEFORE every preflight below: the group is spent whether
        // or not it publishes, so no refusal may return while the caller's producer could still be writing
        // its storage. A failed handoff reads and publishes nothing and leaves the slot for release to
        // quarantine (producerUnproven).
        // Owner admission keeps this session and its group alive while the producer completes. Release
        // the shared cache lock so other owners can progress; unordered_map rehash preserves s/slot.
        lock.unlock();
        if (!honourCallerSync(*slot, attributeName, writeDoneSync))
            return fail(kOvxCommitFailurePublish);
        lock.lock();
        if (mask && (!hostRowMask(*slot, *mask, hostMask) || !applyHostRowMask(*slot, hostMask.data())))
        {
            OVX_RUNTIME_ERROR("ovxCommitGroupMasked: request %zu ('%.128s'): the row mask could not be applied; "
                              "nothing was written.",
                              slot->attributeIndex, attributeName);
            return fail(kOvxCommitFailurePublish);
        }
    }
    // Match the rigid GPU setter's readiness rule here so only session commits gain this
    // diagnostic. The stage drain keeps its own fallback/commit-point behavior. Articulation root,
    // joint-state and tendon columns scatter through the same DirectGPU state, which the first step creates.
    if ((slot->row || slot->artiRow || slot->jointRow || slot->tendonRow) && slot->deviceOrdinal >= 0)
    {
        omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
        if (backend && backend->getStepCount(s->attach) < 1)
        {
            OVX_RUNTIME_ERROR("ovxCommitGroup: request %zu ('%.128s'): DirectGPU write requires at least one "
                              "simulation step. Call warmup() or step_sync() before writing; nothing was written.",
                              slot->attributeIndex, attributeName);
            return failSpent();
        }
    }
    // Reported apart from the not-live cases above, which is the whole reason outFailure exists:
    // this group WAS accepted and is spent. The diagnostic can identify a preflight refusal, but
    // a later device scatter can fail after writing part of its rows.
    char cause[2048];
    {
        // CPU setters can reject a row through the SDK callback while their batch still returns true.
        // A fresh scope makes native commit status independent of any caller's existing diagnostic.
        RuntimeErrorScope error;
        t_scatterCommitAttempted = false;
        const bool published = slot->streamOrdered ?
                                   scatterGroup(*owner, *slot, &deviceArgs, nullptr) :
                                   scatterGroup(*owner, *slot, nullptr, mask ? hostMask.data() : nullptr);
        bool settled = true;
        if (slot->streamOrdered)
        {
            if (caller.capturing)
            {
                // The graph runs this device work, not the session, so there is no completion to record.
                slot->captured = true;
            }
            else
            {
                // Recorded even after a failed scatter, which may have queued part of its work: the parked
                // buffers' next user waits on it. Without it nothing proves when they are idle, so release
                // keeps them. A stream that could not be made to wait for PhysX proves nothing either.
                const omni::physx::tensors::GpuSimulationDataPtr gpuData = sceneGpuData(*slot);
                PxScopedCudaLock contextLock(*slot->ctxMgr);
                int status = 0;
                settled = !unjoined && getCudaShim()->eventRecord(
                                           slot->readyEvent, reinterpret_cast<uintptr_t>(deviceArgs.stream), &status);
                // The commit also read its views' tables on that stream: the host rewrites them only once it is done.
                settled = (!gpuData || gpuData->noteTableReader(deviceArgs.stream)) && settled;
                if (!settled)
                {
                    slot->producerUnproven = true;
                    OVX_RUNTIME_ERROR("ovxCommitGroup: request %zu ('%.128s'): the commit's completion could not be "
                                      "recorded; its buffers are quarantined at release.",
                                      slot->attributeIndex, attributeName);
                }
            }
        }
        if (published && settled && error.message()[0] == '\0')
            return true;
        std::snprintf(cause, sizeof(cause), "%s", error.message());
    }
    // The inner scope has ended; forward its cause without logging the same failure twice.
    char contextualCause[2048];
    const char* publication = (slot->vehicleRow || slot->vehicleControlRow) && !t_scatterCommitAttempted ?
        "nothing was written" : "some rows may have been applied";
    std::snprintf(contextualCause, sizeof(contextualCause),
                  "ovxCommitGroup: request %zu ('%.128s') failed; %s: %.1792s",
                  slot->attributeIndex, attributeName, publication, cause);
    recordRuntimeError(contextualCause);
    return fail(kOvxCommitFailurePublish);
}
} // namespace

extern "C"
{

OMNI_OVX_WRITE_API OvxWriteHandle ovxWriteAttributes(OvxOutputQueryHandle query,
                                                      const ovx_token_or_string_t* attributes,
                                                      size_t attributeCount,
                                                      OvxWriteResult* outSummary)
try
{
    if (outSummary)
        *outSummary = {};
    RuntimeExecutionScope execution = RuntimeExecutionScope::output(writeQueryOwner(query));
    if (!execution)
        return 0;
    if (!query || !attributes || !attributeCount)
    {
        OVX_RUNTIME_ERROR("ovxWriteAttributes: a query and nonempty attribute array are required.");
        return 0;
    }
    std::lock_guard<carb::tasking::RecursiveMutexWrapper> lock(ovx::outputCacheMutex());
    WriteSession s;
    uint64_t debugInstance = 0;
    uint64_t debugAttach = 0;
    if (omni::physx::ovx::debugRenderQuery(query, debugInstance, debugAttach))
    {
        if (attributeCount != 1)
        {
            OVX_RUNTIME_ERROR("ovxWriteAttributes: a debug-render write takes one attribute.");
            return 0;
        }
        const ovx_token_or_string_t* attr = &attributes[0];
        if (attr->string.ptr && attr->string.length)
            s.attribute.assign(attr->string.ptr, attr->string.length);
        ovx_path_dictionary_t* dictionary = nullptr;
        if (debugAttach != 0)
        {
            ovx::SourceContext stage;
            if (ovx::getSourceContext(debugAttach, "ovxWriteAttribute", stage))
                omni::physics::ovstage::buildPathList(*stage.source, nullptr, 0, &dictionary);
        }
        debug_render::WritePlan plan;
        std::string error;
        if (!debug_render::planWrite(debugInstance, dictionary, attr, plan, error))
        {
            OVX_RUNTIME_ERROR("%s", error.c_str());
            return 0;
        }
        auto slot = std::make_unique<GroupSlot>(debugAttach);
        slot->floats.clear();
        slot->shapes.push_back(plan.column.shape[0]);
        if (plan.column.ndim > 1)
            slot->shapes.push_back(plan.column.shape[1]);
        slot->tensors.assign(1, DLTensor{});
        DLTensor& tensor = slot->tensors[0];
        tensor.device = DLDevice{ kDLCPU, 0 };
        tensor.ndim = plan.column.ndim;
        tensor.dtype = plan.column.dtype;
        tensor.strides = nullptr;
        tensor.byte_offset = 0;
        if (plan.column.dtype.code == kDLFloat)
        {
            slot->floats.resize(plan.column.bytes.size() / sizeof(float));
            if (!plan.column.bytes.empty())
                std::memcpy(slot->floats.data(), plan.column.bytes.data(), plan.column.bytes.size());
            tensor.data = slot->floats.empty() ? nullptr : static_cast<void*>(slot->floats.data());
        }
        else
        {
            slot->floats.resize((plan.column.bytes.size() + sizeof(float) - 1) / sizeof(float));
            if (!plan.column.bytes.empty())
                std::memcpy(slot->floats.data(), plan.column.bytes.data(), plan.column.bytes.size());
            tensor.data = plan.column.bytes.empty() ? nullptr : static_cast<void*>(slot->floats.data());
        }
        slot->group.data.tensors = slot->tensors.data();
        slot->group.data.tensor_count = 1;
        s.debugRender = true;
        s.debugOwner = debugInstance;
        s.debugKind = plan.kind;
        s.debugParam = plan.param;
        s.attach = debugAttach;
        s.groups.push_back(std::move(slot));
        if (outSummary)
            s.attributeResults.assign(1, OvxWriteAttributeResult{ plan.column.attribute, 1 });
        const OvxWriteHandle handle = g_nextWriteHandle++;
        const WriteSession& published = g_writes.emplace(handle, std::move(s)).first->second;
        if (outSummary)
        {
            outSummary->totalPrimCount = 0;
            outSummary->attributeCount = published.attributeResults.size();
            outSummary->attributes = published.attributeResults.data();
        }
        return handle;
    }

    uint32_t type = 0, scope = 0;
    if (!ovx::queryTypeScope(query, s.attach, type, scope))
    {
        OVX_RUNTIME_ERROR("ovxWriteAttributes: invalid query handle.");
        return 0;
    }
    s.columns = ovx::queryDeviceBuffers(query);
    s.dict = ovx::queryDictionaryLocked(query);
    if (!s.dict)
    {
        OVX_RUNTIME_ERROR("ovxWriteAttributes: query has no live owner dictionary.");
        return 0;
    }
    s.hasSelection = ovx::querySelection(query, s.selectionKeys);
    AttachedStage* owner = UsdLoad::getUsdLoad()->getAttachedStageByHandle(s.attach);
    if (!owner)
        return 0;
    s.recordEpoch = owner->getInternalPhysXDatabase().recordEpoch();
    if (s.columns && !s.columns->dropRetiredScenes(ovx::allPhysicsScenes(*owner)))
    {
        OVX_RUNTIME_ERROR("ovxWriteAttributes: could not reclaim the parked CUDA buffers of a retired scene.");
        return 0;
    }
    s.attributeNames.reserve(attributeCount);
    if (outSummary)
        s.attributeResults.resize(attributeCount);
    std::unordered_set<std::string> seen;
    for (size_t i = 0; i < attributeCount; ++i)
    {
        const ovx_token_or_string_t& attribute = attributes[i];
        // A non-NULL string wins; otherwise the token names the attribute.
        ovx_string_t name = attribute.string;
        if (!name.ptr && attribute.token != OVX_INVALID_TOKEN &&
            ovx_path_dictionary_token_to_string(s.dict, attribute.token, &name) != OVX_OK)
        {
            OVX_RUNTIME_ERROR("ovxWriteAttributes: request %zu has an unresolvable token.", i);
            return 0;
        }
        if (!name.ptr || !name.length || std::memchr(name.ptr, 0, name.length))
        {
            OVX_RUNTIME_ERROR("ovxWriteAttributes: request %zu has an invalid attribute name.", i);
            return 0;
        }
        s.attributeNames.emplace_back(name.ptr, name.length);
        const std::string& canonical = s.attributeNames.back();
        if (!seen.insert(canonical).second)
        {
            OVX_RUNTIME_ERROR("ovxWriteAttributes: request %zu duplicates '%s'.", i, canonical.c_str());
            return 0;
        }
        if (!acceptsWriteAttribute(type, canonical))
        {
            OVX_RUNTIME_ERROR("ovxWriteAttributes: request %zu ('%s'): %s",
                              i, canonical.c_str(), writeAttributeRejectionReason(type, canonical));
            return 0;
        }
        if (outSummary &&
            ovx_path_dictionary_intern_token(s.dict, name, &s.attributeResults[i].attribute) != OVX_OK)
        {
            OVX_RUNTIME_ERROR("ovxWriteAttributes: request %zu token interning failed.", i);
            return 0;
        }
    }
    size_t totalPrimCount = 0;
    const bool snapshotActiveMembership = scope == kOvxActive &&
        (type == kOvxArticulation || type == kOvxVehicleWheel || type == kOvxVehicle ||
         type == kOvxDeformableVolume || type == kOvxDeformableSurface);
    if (snapshotActiveMembership)
    {
        // These planners share ALL caches with reads. Compact from the query's
        // authoritative active set rather than treating every cached row as active.
        if (!ovx::queryMembershipLocked(query, s.selectionKeys))
            return 0;
        s.hasSelection = true;
        if (outSummary)
            totalPrimCount = s.selectionKeys.size();
    }
    else if (outSummary && !ovx::queryTotalPrimCountLocked(query, totalPrimCount))
    {
        return 0;
    }
    // Unpublished groups own their allocations directly; unwinding or an early return
    // reclaims them without the waits needed for a committed session.
    for (size_t i = 0; i < attributeCount; ++i)
    {
        s.attribute = s.attributeNames[i];
        // One attribute may emit several groups; label only the groups it appends.
        const size_t firstGroup = s.groups.size();
        // Empty selections need no scene/backend preflight after name validation.
        if ((!s.hasSelection || !s.selectionKeys.empty()) && !planGroups(s, type, scope, *owner))
        {
            OVX_RUNTIME_ERROR("ovxWriteAttributes: planning request %zu ('%s') failed; no session was published.",
                              i, s.attribute.c_str());
            return 0;
        }
        if (outSummary)
            s.attributeResults[i].groupCount = s.groups.size() - firstGroup;
        for (size_t group = firstGroup; group < s.groups.size(); ++group)
        {
            GroupSlot& slot = *s.groups[group];
            slot.attributeIndex = i;
            // A DirectGPU device group commits on the caller's stream; everything that needs the host is done
            // here, so its commit only queues device work (ADR-0052).
            const std::unordered_map<const PxScene*, WriteSession::RootPlan>::iterator roots =
                slot.artiRow ? s.rootPlans.find(slot.scene) : s.rootPlans.end();
            const bool prepared =
                isStreamOrderedKind(slot) ?
                    prepareStreamOrderedGroup(slot, roots != s.rootPlans.end() ? &roots->second : nullptr) :
                    !slot.deformableKinematicTarget || prepareHostKinematicTargetGroup(slot);
            if (!prepared)
            {
                OVX_RUNTIME_ERROR("ovxWriteAttributes: request %zu ('%s'): preparing its device commit failed; no "
                                  "session was published.",
                                  i, s.attribute.c_str());
                return 0;
            }
        }
    }
    if (owner->getInternalPhysXDatabase().recordEpoch() != s.recordEpoch)
    {
        OVX_RUNTIME_ERROR("ovxWriteAttributes: attachment changed while planning.");
        return 0;
    }
    // Reserve the map node before transferring any owned group storage.
    const OvxWriteHandle h = g_nextWriteHandle;
    if (!h)
    {
        OVX_RUNTIME_ERROR("ovxWriteAttributes: write handle space exhausted.");
        return 0;
    }
    const std::pair<std::unordered_map<OvxWriteHandle, WriteSession>::iterator, bool> inserted =
        g_writes.try_emplace(h);
    if (!inserted.second)
    {
        OVX_RUNTIME_ERROR("ovxWriteAttributes: write handle space exhausted.");
        return 0;
    }
    ++g_nextWriteHandle;
    inserted.first->second = std::move(s);
    const WriteSession& published = inserted.first->second;
    if (outSummary)
    {
        outSummary->totalPrimCount = totalPrimCount;
        outSummary->attributeCount = published.attributeResults.size();
        outSummary->attributes = published.attributeResults.data();
    }
    return h;
}
catch (const std::exception& error)
{
    if (outSummary)
        *outSummary = {};
    OVX_RUNTIME_ERROR("ovxWriteAttributes: write creation failed: %s", error.what());
    return 0;
}
catch (...)
{
    if (outSummary)
        *outSummary = {};
    OVX_RUNTIME_ERROR("ovxWriteAttributes: write creation failed with an exception.");
    return 0;
}

OMNI_OVX_WRITE_API OvxWriteStatus ovxFetchWriteNext(OvxWriteHandle write, OvxFetchedWriteGroup* outGroup)
{
    RuntimeExecutionScope execution = RuntimeExecutionScope::output(writeOwner(write));
    if (!execution)
        return kOvxWriteStatusBusy;
    if (!outGroup)
    {
        OVX_RUNTIME_ERROR("ovxFetchWriteNext: outGroup is null.");
        return kOvxWriteStatusError;
    }
    *outGroup = {};

    std::lock_guard<carb::tasking::RecursiveMutexWrapper> lock(omni::physx::ovx::outputCacheMutex());
    WriteSession* s = findSession(write);
    if (s && s->debugRender)
    {
        if (s->nextFetch >= s->groups.size())
            return kOvxWriteStatusEndOfIteration;
        GroupSlot* slot = s->groups[s->nextFetch++].get();
        slot->tensors[0].shape = slot->shapes.data();
        slot->group.data.tensors = slot->tensors.data();
        slot->group.data.tensor_count = 1;
        *outGroup = {&slot->group, 0};
        return kOvxWriteStatusOk;
    }
    AttachedStage* owner = s ? UsdLoad::getUsdLoad()->getAttachedStageByHandle(s->attach) : nullptr;
    if (!owner || owner->getInternalPhysXDatabase().recordEpoch() != s->recordEpoch)
    {
        OVX_RUNTIME_ERROR("ovxFetchWriteNext: invalid or retired write session owner.");
        return kOvxWriteStatusError; // a bad handle is a failure to iterate, not exhaustion
    }

    if (s->nextFetch >= s->groups.size())
        return kOvxWriteStatusEndOfIteration;

    const GroupSlot& slot = *s->groups[s->nextFetch++];
    *outGroup = {&slot.group, slot.attributeIndex};
    return kOvxWriteStatusOk;
}

OMNI_OVX_WRITE_API bool ovxCommitGroup(OvxWriteHandle write,
                                       const ovstage_map_group_t* group,
                                       ovstage_cuda_sync_t writeDoneSync,
                                       OvxCommitFailure* outFailure)
{
    return commitGroup(write, group, nullptr, writeDoneSync, outFailure);
}

OMNI_OVX_WRITE_API bool ovxValidateCommit(OvxWriteHandle write,
                                          const ovstage_map_group_t* group,
                                          const DLTensor* rowMask,
                                          ovstage_cuda_sync_t writeDoneSync)
{
    RuntimeExecutionScope execution = RuntimeExecutionScope::output(writeOwner(write));
    if (!execution)
        return true; // the commit reports the busy owner
    std::lock_guard<carb::tasking::RecursiveMutexWrapper> lock(omni::physx::ovx::outputCacheMutex());
    WriteSession* s = findSession(write);
    if (!s)
        return true;
    // Groups the commit would refuse as not live are left to it.
    const GroupSlot* slot = nullptr;
    if (!s->debugRender)
        slot = findLiveGroup(*s, group);
    else if (s->groups.size() == 1 && group == &s->groups[0]->group && !s->groups[0]->committed)
        slot = s->groups[0].get();
    return !slot || !commitRefused(*s, *slot, rowMask, writeDoneSync);
}

OMNI_OVX_WRITE_API bool ovxCommitGroupMasked(OvxWriteHandle write,
                                             const ovstage_map_group_t* group,
                                             const DLTensor* rowMask,
                                             ovstage_cuda_sync_t writeDoneSync,
                                             OvxCommitFailure* outFailure)
{
    if (!rowMask)
    {
        OVX_RUNTIME_ERROR("ovxCommitGroupMasked: the row mask is null; commit the group unmasked to write every row.");
        if (outFailure)
            *outFailure = kOvxCommitFailureInvalidArgument;
        return false;
    }
    return commitGroup(write, group, rowMask, writeDoneSync, outFailure);
}

OMNI_OVX_WRITE_API bool ovxReleaseWrite(OvxWriteHandle write)
{
    RuntimeExecutionScope execution = RuntimeExecutionScope::output(writeOwner(write), true);
    if (!execution)
        return false;
    // Uncommitted groups are DISCARDED, not published. Destroying the session is the whole of that:
    // nothing was handed to physics, so a caller that failed or threw mid-fill publishes nothing
    // from the group it was filling. Committed groups are not rolled back.
    //
    // Idempotent for an unknown or already-released handle, matching ovxReleaseRead.
    std::lock_guard<carb::tasking::RecursiveMutexWrapper> lock(omni::physx::ovx::outputCacheMutex());
    const std::unordered_map<OvxWriteHandle, WriteSession>::iterator it = g_writes.find(write);
    if (it == g_writes.end())
        return true;
    bool released = releaseGroupStorage(it->second);
    // A write can outlive its query (ADR-0045). When it is the last user of the query's buffers, they are freed
    // here, where a failed free still fails this release, rather than in the cache's destructor.
    const std::shared_ptr<ovx::QueryDeviceBuffers> columns = it->second.columns;
    g_writes.erase(it);
    if (columns && columns.use_count() == 1 && !columns->drain())
    {
        released = false;
        OVX_RUNTIME_ERROR("Write release could not reclaim the released query's parked CUDA buffers");
    }
    return released;
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

// Cache of the planned borrow session (the bodies/keys/comp GroupSlot) per (scene, attribute, row count).
// With the output group skipped, a borrow slot holds nothing releaseGroupStorage destroys, so it is reused
// across drains -- skipping planRigidGroup entirely -- while the object set (recordLifetimeEpoch), the
// kinematic state (kinematicToggleEpoch) and the key set are unchanged. Only the borrowed device column is
// re-pointed each drain.
struct DrainSlotCache
{
    uint64_t recordEpoch = ~uint64_t(0);
    uint64_t flagEpoch = ~uint64_t(0);
    std::vector<ObjectKey> keys;
    WriteSession session;
    bool valid = false;
};

using DrainSlotCacheMap = std::map<std::tuple<uint64_t, const PxScene*, const WriteAttributeRow*, uint32_t>, DrainSlotCache>;
static DrainSlotCache& drainSlotCacheFor(AttachedStage& owner, const PxScene* scene, const WriteAttributeRow* row, uint32_t count)
{
    static DrainSlotCacheMap cacheBySlot;
    // Key by the attach handle, not the scene address alone. A scene address can be reused by a later
    // attachment, and the hit-test epochs (recordLifetimeEpoch / kinematicToggleEpoch) resolve the address to
    // its current attachment's recordVersion / kinematicVersion, which restart per attachment. So a detach +
    // re-attach of identical content would reach the same epochs and key set and hit this stale session --
    // scattering through, then releasing, an already-freed context. Attach handles are never reused and
    // recordVersion only grows within one attachment, so (attach, scene) can never produce a false hit.
    const uint64_t attach = owner.getAttachHandle();
    const std::tuple<uint64_t, const PxScene*, const WriteAttributeRow*, uint32_t> key{ attach, scene, row, count };
    const DrainSlotCacheMap::iterator hit = cacheBySlot.find(key);
    if (hit != cacheBySlot.end())
        return hit->second; // warm hit: steady state pays nothing
    // Cold key: drop entries for attachments that have detached (handle no longer resolves), mirroring
    // drainPartitionCacheFor, so a retired attachment's WriteSession is not retained for the process lifetime.
    if (UsdLoad* load = UsdLoad::getUsdLoadIfCreated())
        for (DrainSlotCacheMap::iterator it = cacheBySlot.begin(); it != cacheBySlot.end();)
            it = (std::get<0>(it->first) == attach || load->getAttachedStageByHandle(std::get<0>(it->first)))
                     ? std::next(it)
                     : cacheBySlot.erase(it);
    // The count (and dead-scene) dimension still grows on a live attachment whose per-drain row count varies.
    // Cap it like the partition cache: past a small cap, clear and let the next drain rebuild the few it needs.
    constexpr size_t kMaxSlots = 16;
    if (cacheBySlot.size() >= kMaxSlots)
        cacheBySlot.clear(); // returns a fresh entry below; no caller holds a reference across this call
    return cacheBySlot[key];
}

static bool drainScatterRigidGroup(AttachedStage& owner, parse::IPhysicsSource& source,
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
    // The disabled-body scan is O(bodies) and runs every drain. Gate it on the backend's per-scene hint,
    // exactly as planRigidGroup gates its own filter: a DirectGPU scene with nothing disabled (the steady
    // state) then pays O(1) instead of a virtual actor-flag read per body.
    omni::physx::tensors::SimulationBackend* disableHintBackend = omni::physx::tensors::GetSimulationBackend();
    const bool mayHaveDisabled = !disableHintBackend || disableHintBackend->sceneMayHaveDisabledRigidDynamics(scene);
    const bool willFilter = sceneGpu && mayHaveDisabled && ovx::anyDisabledRigidDynamic(bodies);

    // An angular column needs a deg->rad scale, which the zero-copy borrow cannot apply -- but on a
    // DirectGPU scene it still stays on device (deviceScaleAngular below): the session allocates its own
    // column and a scale kernel writes the conversion into it from the borrowed ovstage column, no host
    // stage. A filtered group (the borrow would alias a mismatched column) is excluded from both.
    const bool borrow = colDevice && sceneGpu && !srcRows && !angular && !willFilter;
    const bool deviceScaleAngular = colDevice && angular && sceneGpu && !srcRows && !willFilter;

    // A device column we can neither borrow nor scale on-device (a scattered subset, a filtered group with
    // a kinematic/disabled/local-space body, or a CPU scene) is staged through host in the else branch
    // below: D2H the column once, gather exactly the serviceable rows, H2D the result into the session
    // column. The whole-column device paths (borrow, device scale) above stay zero-copy; only this rarer
    // filtered case pays the round trip -- but its serviceable bodies are applied instead of lost.

    // The drain scatters through the backend superset view, not the ovstage output group, so planRigidGroup
    // skips building the (unused) prim-path list / DLTensor / map group. For the zero-copy borrow the whole
    // planned slot (bodies/keys/comp) is stable while the object set and write-flags are, so cache the
    // session per (scene, attribute, count) and reuse it -- skipping planRigidGroup entirely -- re-pointing
    // only the borrowed device column. Angular scale / host / filtered groups plan fresh each drain.
    WriteSession localSession;
    WriteSession* sp = &localSession;
    DrainSlotCache* borrowCache = nullptr; // the reused borrow session's cache entry; a failed producer
                                           // handoff below invalidates it so the next drain re-plans instead
                                           // of dereferencing a slot releaseGroupStorage already released.
    if (borrow)
    {
        DrainSlotCache& sc = drainSlotCacheFor(owner, scene, &row, static_cast<uint32_t>(keys.size()));
        borrowCache = &sc;
        const uint64_t recEpoch = omni::physx::internal::recordLifetimeEpoch(scene);
        const uint64_t flagEpoch = omni::physx::internal::kinematicToggleEpoch(scene);
        const bool hit = sc.valid && sc.recordEpoch == recEpoch && sc.flagEpoch == flagEpoch &&
                         sc.keys.size() == keys.size() &&
                         (keys.empty() ||
                          std::memcmp(sc.keys.data(), keys.data(), keys.size() * sizeof(ObjectKey)) == 0);
        if (!hit)
        {
            sc.valid = false;
            sc.session = WriteSession{};
            sc.session.attach = owner.getAttachHandle();
            sc.session.recordEpoch = owner.getInternalPhysXDatabase().recordEpoch();
            if (!planRigidGroup(sc.session, source, scene, bodies, keys, row, kOvxActive, /*allocate=*/false,
                                kWholeSetScopeFromCache, /*buildOutputGroup=*/false))
                return false;
            if (sc.session.groups.empty())
                return true; // every body disabled -> nothing to scatter (borrow implies !willFilter, so rare)
            sc.recordEpoch = recEpoch;
            sc.flagEpoch = flagEpoch;
            sc.keys.assign(keys.begin(), keys.end());
            sc.valid = true;
        }
        sp = &sc.session;
    }
    else
    {
        localSession.attach = owner.getAttachHandle();
        localSession.recordEpoch = owner.getInternalPhysXDatabase().recordEpoch();
        if (!planRigidGroup(localSession, source, scene, bodies, keys, row, kOvxActive, /*allocate=*/!borrow,
                            kWholeSetScopeFromCache, /*buildOutputGroup=*/false))
            return false;
        if (localSession.groups.empty())
            return true; // every body in this scene was disabled -> nothing to scatter, correctly skipped
    }
    WriteSession& s = *sp;
    GroupSlot& slot = *s.groups.back();

    if (borrow)
    {
        slot.deviceData = reinterpret_cast<CUdeviceptr>(const_cast<void*>(batch.values.data));
        slot.deviceOrdinal = batch.values.device;
        slot.borrowed = true;
    }
    else if (deviceScaleAngular)
    {
        // Angular velocity is authored in DEGREES; convert to engine-native radians ON DEVICE. The
        // session owns a device column (planRigidGroup allocate=true above); scale the borrowed ovstage
        // column straight into it (device->device, no host copy) after waiting on the ovstage producer
        // event. slot order == group order == column order here (unfiltered, no srcRows), so the scale is
        // positional. releaseGroupStorage frees the session column since slot.borrowed stays false.
        PxScopedCudaLock _lock(*slot.ctxMgr);
        // Run the deg->rad scale on the producer's stream, like the other ovstage write kernels (they all take
        // the write's stream rather than hardcoding the legacy default stream, which serializes with other
        // streams in the context and cannot be CUDA-graph captured). Order it after the producer handoff: wait
        // on the producer event when one was given; otherwise the scale is already after the producer's work on
        // this same stream. Both sync fields are 0 on the current self-synchronizing ovstage read, so the scale
        // runs on the null stream today and the wait does not fire.
        const cudaStream_t scaleStream =
            reinterpret_cast<cudaStream_t>(static_cast<uintptr_t>(batch.values.deviceStream));
        if (batch.values.deviceWaitEvent && getCudaShim())
        {
            if (!getCudaShim()->streamWaitEvent(reinterpret_cast<uintptr_t>(scaleStream),
                                                static_cast<uintptr_t>(batch.values.deviceWaitEvent), 0, nullptr))
            {
                t_devicePrepFailed = true; // producer wait failed on the angular deg->rad scale -> retryable
                return false;
            }
        }
        const PxU32 floatCount = static_cast<PxU32>(slot.bodies.size() * size_t(slot.comp));
        // The scale IS the fill for the session column (planRigidGroup only allocated it): a failed launch
        // leaves it uninitialized, so set t_devicePrepFailed and report a retryable Failed rather than scatter garbage
        // angular velocity while reporting Applied. The check catches launch failures, not a fault while the
        // kernel runs (which surfaces on a later sync) -- but the trivial dst=k*src kernel only faults on an
        // already-bad pointer or count, a logic bug.
        if (!omni::physx::tensors::scaleFloatColumnOvStage(reinterpret_cast<float*>(slot.deviceData),
                                                           static_cast<const float*>(batch.values.data), floatCount,
                                                           0.017453292519943295f, scaleStream)) // pi / 180
        {
            t_devicePrepFailed = true; // angular scale launch failed -> retryable; the DirectGPU parse fallback is inert
            return false;
        }
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
        // A device column cannot be read host-side: stage it to host once (D2H), ordered behind the ovstage
        // producer handoff, so the per-key / subset gather below selects the serviceable rows. The whole
        // column is copied, since srcRows indexes into it.
        std::vector<uint8_t> deviceHostCopy;
        if (colDevice)
        {
            if (!slot.ctxMgr || !getCudaShim())
                return false; // no CUDA context to stage a device column through -> parse-layer fallback
            PxScopedCudaLock _lock(*slot.ctxMgr);
            // Honour the producer handoff before reading; fail closed on error. stream and wait_event are
            // INDEPENDENT dependencies -- a {stream, event} handoff requires draining the producer stream AND
            // waiting on its event -- so do BOTH when both are set, not one or the other, stream first. (Both are
            // 0 on the current self-synchronizing ovstage read, so neither fires today; this completes the
            // explicit future handoff contract.)
            if (batch.values.deviceStream &&
                !getCudaShim()->streamSynchronize(static_cast<uintptr_t>(batch.values.deviceStream), nullptr))
            {
                t_devicePrepFailed = true; // producer sync failed staging the device column -> retryable
                return false;
            }
            if (batch.values.deviceWaitEvent &&
                !getCudaShim()->streamWaitEvent(uintptr_t(0),
                                                static_cast<uintptr_t>(batch.values.deviceWaitEvent), 0, nullptr))
            {
                t_devicePrepFailed = true; // producer wait failed staging the device column -> retryable
                return false;
            }
            deviceHostCopy.resize(size_t(batch.numChanges) * rowBytes);
            // A failed D2H leaves deviceHostCopy zero-filled; scattering that while reporting Applied is the
            // failure the angular-scale path above also guards. Report a retryable Failed instead (the DirectGPU
            // parse fallback is inert and would drop it).
            if (slot.ctxMgr->getCudaContext()->memcpyDtoH(
                    deviceHostCopy.data(),
                    reinterpret_cast<CUdeviceptr>(const_cast<void*>(batch.values.data)), deviceHostCopy.size()) != 0)
            {
                t_devicePrepFailed = true; // D2H staging copy failed -> retryable, must not fall back
                return false;
            }
        }
        const uint8_t* col = colDevice ? deviceHostCopy.data() : static_cast<const uint8_t*>(batch.values.data);
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
            // Check the staging H2D: a discarded failure would stage stale/partial data, then commit it and
            // report Applied while the authored velocity never reached the device. Fail retryable instead (the
            // DirectGPU parse fallback is inert and would advance the ordinal dropping it, AC-12).
            if (slot.ctxMgr->getCudaContext()->memcpyHtoD(slot.deviceData, src, nOut * rowBytes) != 0)
            {
                t_devicePrepFailed = true; // staged H2D copy failed -> retryable, must not fall back
                releaseGroupStorage(s);
                return false;
            }
        }
        else
        {
            std::memcpy(slot.floats.data(), src, nOut * rowBytes);
        }
    }

    slot.committed = true;
    // A borrowed device column aliases the ovstage producer's output; prove its handoff (the ColumnView's
    // opaque stream/event, carried as an ovstage_cuda_sync_t) complete before the scatter reads it. Staged
    // and device-scaled columns pass no producer sync here: a staged column's host->device copy and its scatter
    // both run on the legacy stream (drainArgs.stream below is set only for the borrow/scale paths), and the
    // scale waits on the producer event then runs on the producer stream the scatter also uses.
    // honourCallerSync is a no-op when the feed delivered the column synchronously (stream/event both 0), i.e.
    // the parity read path.
    if (borrow &&
        !honourCallerSync(slot, row.token,
                          ovstage_cuda_sync_t{ static_cast<uintptr_t>(batch.values.deviceStream),
                                               static_cast<uintptr_t>(batch.values.deviceWaitEvent) }))
    {
        // The borrowed column's producer handoff failed before the scatter read it -> retryable (hold the cursor),
        // not a fall back to the DirectGPU-inert parse path. releaseGroupStorage quarantines (releases) the slot;
        // drop the cached session so the next matching drain re-plans instead of reusing an entry whose only group
        // is now gone (null deref otherwise).
        t_devicePrepFailed = true;
        if (borrowCache)
            borrowCache->valid = false;
        releaseGroupStorage(s);
        return false;
    }
    // Scatter on the producer's stream for the zero-copy paths (borrow / on-device scale): their kernels ran
    // there, so PhysX forks its write from the same stream the column was written on -- ordered by the stream,
    // no extra event. The staged branch keeps the legacy default stream: it fills the session column with a
    // host->device memcpy on that stream, a pageable copy a non-blocking producer stream would not wait for
    // (GpuSimulationData::rewriteTable syncs for the same reason), so its copy and scatter must share one
    // stream. On the current synchronous read the producer stream is 0, the legacy default stream, so this is a
    // no-op today.
    omni::physx::tensors::OvStageDeviceWriteArgs drainArgs;
    if (borrow || deviceScaleAngular)
        drainArgs.stream = reinterpret_cast<CUstream>(static_cast<uintptr_t>(batch.values.deviceStream));
    const bool ok = scatterGroup(owner, slot, &drainArgs);
    releaseGroupStorage(s);
    return ok;
}

// ObjectKey -> (scene, body) for every standalone rigid, cached against the object-lifetime epoch.
// A drain that rewrites the same bodies each time (the dense control-loop pattern) resolves the SAME key
// set every drain -- the change-range read returns the rows written that range -- and this map is purely
// topology-derived (recordLifetimeEpoch bumps on every rigid add/remove), so it is rebuilt only on a
// topology change instead of every drain. Standalone rigids ONLY: a link-writable row scans articulation links
// fresh each drain (kOvxAll, topology-derived like this map), not cached here -- the hot-path velocity
// drain is not link-writable, so this standalone-only cache covers it.
using StandaloneKeyToBody = std::unordered_map<ObjectKey, std::pair<PxScene*, PxRigidBody*>, ObjectKey::Hash>;

// The standalone-rigid lookups the value and pose drains resolve keys against. Both are fixed between object
// creations and removals, so both are cached against the object-lifetime epoch.
struct StandaloneRigidLookup
{
    StandaloneKeyToBody keyToBody;
    // Dynamic bodies whose authored velocity is in the body's own frame (`physics:localSpaceVelocities`, tracked as
    // InternalActorFlag::eLOCALSPACE_VELOCITIES). The parse-layer velocity update rotates and scales such a velocity
    // into world space (PhysXRigidBodyPropertiesUpdate.cpp updateLinearVelocity); the drain scatters the raw column,
    // so a velocity row on one of these bodies is left to the parse path -- otherwise a local-frame velocity would
    // land as if it were world-frame. The flag is set when the body is created. Filled only for a caller that asks
    // for it (the velocity drain), so the pose drain's rebuild does not pay for a second record walk.
    std::unordered_set<const PxRigidBody*> localSpaceVelocityBodies;
};

static const StandaloneRigidLookup& cachedStandaloneRigids(AttachedStage& owner, bool withLocalSpaceVelocityBodies)
{
    // One entry PER ATTACHMENT, rebuilt only when that attachment's recordEpoch() moves. A single shared slot
    // used to rebuild the whole map whenever the owner changed, so two attachments draining alternately
    // thrashed it every drain. On a cold/stale lookup, entries for detached attachments (whose handle no
    // longer resolves) are dropped -- mirroring the detached-attachment purge in drainSlotCacheFor -- so the map does
    // not grow across attach/detach cycles. A steady drain loop finds its slot current and pays nothing.
    struct Slot
    {
        uint64_t epoch = ~uint64_t(0);
        StandaloneRigidLookup lookup;
        bool localSpaceBuilt = false;
    };
    static std::unordered_map<uint64_t, Slot> byAttach;
    const uint64_t attach = owner.getAttachHandle();
    const internal::InternalPhysXDatabase& db = owner.getInternalPhysXDatabase();
    const uint64_t epoch = db.recordEpoch();
    const std::unordered_map<uint64_t, Slot>::iterator hit = byAttach.find(attach);
    Slot* slot = (hit != byAttach.end() && hit->second.epoch == epoch) ? &hit->second : nullptr;
    if (!slot)
    {
        if (UsdLoad* load = UsdLoad::getUsdLoadIfCreated())
            for (std::unordered_map<uint64_t, Slot>::iterator it = byAttach.begin(); it != byAttach.end();)
                it = (it->first == attach || load->getAttachedStageByHandle(it->first)) ? std::next(it)
                                                                                        : byAttach.erase(it);
        slot = &byAttach[attach];
        slot->lookup.keyToBody.clear();
        slot->lookup.localSpaceVelocityBodies.clear();
        slot->localSpaceBuilt = false;
        // Whole-scene (kOvxAll) record walk, O(records). It runs only on a cold/stale (attach, epoch) -- the warm
        // hit above skips it -- so a scan cache keyed the same way could never hit; walk directly.
        ovx::RigidRecordScan scan;
        ovx::scanRigidRecords(owner, kOvxAll, ovx::ActiveActorSet{}, scan);
        for (const ovx::RigidSceneBucket& b : scan.byScene)
            for (size_t i = 0; i < b.keys.size(); ++i)
                slot->lookup.keyToBody.emplace(b.keys[i], std::make_pair(b.scene, b.bodies[i]));
        slot->epoch = epoch;
    }
    if (withLocalSpaceVelocityBodies && !slot->localSpaceBuilt)
    {
        for (const internal::InternalDatabase::Record& rec : db.getRecords())
        {
            if (rec.mType != ePTActor || !rec.mPtr || !rec.mInternalPtr)
                continue;
            const internal::InternalActor* ia = reinterpret_cast<const internal::InternalActor*>(rec.mInternalPtr);
            if (!(ia->mFlags & internal::InternalActorFlag::eLOCALSPACE_VELOCITIES))
                continue;
            if (PxRigidDynamic* dyn = reinterpret_cast<PxRigidActor*>(rec.mPtr)->is<PxRigidDynamic>())
                slot->lookup.localSpaceVelocityBodies.insert(dyn);
        }
        slot->localSpaceBuilt = true;
    }
    return slot->lookup;
}

// Device POSE drain. ovstage authors pose as local-frame / reset-stack edits; the change feed resolves
// each to a column-major 4x4 world matrix (omni:fabric:worldMatrix, 16 doubles or floats/row) and -- on a DirectGPU
// sim -- delivers it ON DEVICE for EXACTLY the prims whose local frame changed this range (the feed gates
// the device world-matrix read to the authored-local-frame path set, so a hierarchy recompute of an
// unrelated body is never in the column). PhysX takes position (vec3) + orientation (quat): decompose the
// device matrix column into a pos column and a quat column ON DEVICE (decomposeWorldMatrixOvStage, matching
// the host decomposeMatrix), then scatter each through the proven borrow path -- no host copy on the payload.
//
// Split, don't abandon -- the pose path's realization of the AC-13 split (applyOvstageValueBatch's dispatch
// comment is the canonical rationale: the per-object/per-prim parse fallback is INERT on a DirectGPU scene, so
// a whole-batch fallback silently drops the serviceable keys). Here the inert fallback is the per-prim
// transform path (checkPrimChange -> addTransformChange, which IS the pose path on a CPU scene). A row the
// device scatter cannot serve (a link, a non-body prim, a different-scene or disabled body) is recorded
// UNRESOLVED and the serviceable rows are still posed on device. Two cases still fall back WHOLE: nothing
// serviceable, or the caller gave no sink for the skipped rows.
static DrainResult applyPoseBatch(AttachedStage& owner, parse::IPhysicsSource& source, const parse::ChangeBatch& batch,
                                  const std::string& usdAttr, std::vector<uint32_t>* unresolvedRows)
{
    if (usdAttr != omni::physics::ovstage::conv::kFabricWorldMatrix || batch.values.device < 0 ||
        batch.numChanges == 0 || !batch.values.data || !batch.keys.data ||
        (batch.values.type != parse::ColumnType::eDouble && batch.values.type != parse::ColumnType::eFloat))
        return DrainResult::NotHandled;

    const ObjectKey* keys = static_cast<const ObjectKey*>(batch.keys.data);
    // Epoch-cached standalone key->body map: the device pose drain re-resolves the same key set every frame
    // a body is re-posed, so the O(records) map build is served once per topology change, not per drain.
    const StandaloneKeyToBody& keyToBody =
        cachedStandaloneRigids(owner, /*withLocalSpaceVelocityBodies=*/false).keyToBody;

    // Controller proxies (ePTCct) are standalone kinematic dynamics the rigid-body READ reports, so the cached map
    // includes them -- but a pose WRITE must not drive one (it desyncs PxController from its actor), which is why
    // planRigidGroup drops them from the write plan. Detect a proxy so the loop below treats it as NOT serviceable:
    // counting it serviceable would decompose its row into the device column and then planRigidGroup would drop it
    // from the scatter, shifting every FOLLOWING body onto the wrong (preceding) column row.
    const std::vector<omni::physx::internal::InternalDatabase::Record>& records =
        owner.getInternalPhysXDatabase().getRecords();
    const auto isControllerProxy = [&records](PxRigidBody* body) -> bool
    {
        const size_t id = body ? reinterpret_cast<size_t>(body->userData) : static_cast<size_t>(-1);
        if (id >= records.size() || records[id].mType != ePTCct)
            return false;
        const auto* cct = static_cast<omni::physx::internal::InternalCct*>(records[id].mInternalPtr);
        return cct && cct->mActor == body;
    };

    // Resolve each row and PARTITION BY SCENE: every serviceable body (a standalone, non-disabled, non-controller
    // rigid in a DirectGPU scene) is grouped under its OWN scene, and each scene is decomposed + scattered on
    // device in turn, so a multi-scene attach poses all of its scenes rather than only the first. Every other row
    // -- a non-standalone body, a disabled body, a controller proxy, or a not-yet-created key -- is recorded skipped for the parse-layer
    // fallback. This path runs only for a DEVICE column, and a device column is delivered only when EVERY scene
    // in the attach is DirectGPU on one device (ovxDirectGpuDeviceOrdinal >= 0), so there is no CPU scene here --
    // a mixed attach gets ordinal -1 and the transform column stays on the host read.
    //
    // A KINEMATIC body IS served -- its pose is written through the same device setGlobalPose the dynamic
    // bodies use. That moves it to the authored pose WITHOUT the sweep-and-push of the CPU setKinematicTarget,
    // but on a DirectGPU scene there is no alternative: the GPU rigid-write API has no kinematic-target entry
    // (PxRigidDynamicGPUAPIWriteType is global-pose / velocities / force / torque only, and its set-global-pose
    // kernel is a pure pose overwrite -- no velocity), and the per-prim host path that would call
    // setKinematicTarget is inert on DirectGPU. So the device pose is the only way to move a kinematic body
    // there, and it is what IsaacLab does too (it moves kinematic rigid bodies by writing their pose; the
    // push-contact behaviour is a CPU-dynamics-only feature upstream).
    struct PoseSceneGroup
    {
        std::vector<PxRigidBody*> bodies;
        std::vector<ObjectKey> gkeys;
        std::vector<uint32_t> ok; // logical rows served on device (indices into the batch / its rowMap)
    };
    std::unordered_map<PxScene*, PoseSceneGroup> byScene;
    std::vector<uint32_t> skipped; // logical rows left for the parse-layer fallback
    size_t resolved = 0;
    for (size_t i = 0; i < batch.numChanges; ++i)
    {
        const StandaloneKeyToBody::const_iterator it = keyToBody.find(keys[i]);
        PxRigidBody* const body = (it != keyToBody.end()) ? it->second.second : nullptr;
        PxScene* const bodyScene = body ? it->second.first : nullptr;
        if (body && bodyScene && !isControllerProxy(body) &&
            bodyScene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API) &&
            !ovx::isDisabledRigidDynamic(body))
        {
            PoseSceneGroup& g = byScene[bodyScene];
            g.bodies.push_back(body);
            g.gkeys.push_back(keys[i]);
            g.ok.push_back(static_cast<uint32_t>(i));
            ++resolved;
        }
        else
            skipped.push_back(static_cast<uint32_t>(i));
    }
    // Fall back WHOLE when nothing resolved, or when the caller passed no sink for the skipped rows (a partial
    // Applied would make onSourceChange silently drop them). Mirrors the velocity split's gate.
    if (resolved == 0 || (!skipped.empty() && !unresolvedRows))
        return DrainResult::NotHandled;

    const WriteAttributeRow* posRow = findRigidWriteAttribute(OvxAttr::kPosition);
    const WriteAttributeRow* oriRow = findRigidWriteAttribute(OvxAttr::kOrientation);
    if (!posRow || !oriRow)
        return DrainResult::NotHandled;

    // Decompose + scatter EACH scene's serviceable group on device. Each scene owns its scratch: its bodies are
    // compact in `g.bodies` order, so the scatter is dense (no srcRows gather). Decompose reads the shared
    // ovstage matrix column (gathered through subMap) into that scene's own pos/quat columns.
    const uint32_t* rowMap = batch.values.rowMap;
    bool scattered = true;
    bool prepFailed = false;
    // A TRANSIENT device-prep failure (alloc / producer sync / rowMap upload / decompose) on a selected device
    // path, as opposed to the persistent no-CUDA-context bail -- the two map to different DrainResults below.
    bool transientPrepFailed = false;
    for (const std::pair<PxScene* const, PoseSceneGroup>& e : byScene)
    {
        PxScene* const scene = e.first;
        const PoseSceneGroup& g = e.second;
        PxCudaContextManager* const ctxMgr = scene->getCudaContextManager();
        PxCudaContext* const cu = ctxMgr ? ctxMgr->getCudaContext() : nullptr;
        if (!cu)
        {
            prepFailed = true;
            break;
        }
        const uint32_t rows = static_cast<uint32_t>(g.ok.size());
        const int devOrd = ctxMgr->getDevice();

        // The device column stores logical row i's matrix at storage row rowMap[i] when the snapshot came back
        // index-mapped (a HOST array of indices into a shared population buffer), or at storage row i when it
        // came back dense (null rowMap). This scene's serviceable set is a SUBSET of the batch, so compose ONE
        // gather map -- subMap[j] = rowMap ? rowMap[g.ok[j]] : g.ok[j] -- and the decompose reads exactly those
        // storage rows into a dense pos/quat column aligned with g.bodies. Always uploaded: the subset drops
        // rows, so the gather is never the identity even for a dense column.
        std::vector<uint32_t> subMap(rows);
        for (uint32_t j = 0; j < rows; ++j)
            subMap[j] = rowMap ? rowMap[g.ok[j]] : g.ok[j];

        CUdeviceptr posDev = 0, quatDev = 0, rowMapDev = 0;
        {
            PxScopedCudaLock _lock(*ctxMgr);
            cu->memAlloc(&posDev, size_t(rows) * 3 * sizeof(float));
            cu->memAlloc(&quatDev, size_t(rows) * 4 * sizeof(float));
            cu->memAlloc(&rowMapDev, size_t(rows) * sizeof(uint32_t));
            if (!posDev || !quatDev || !rowMapDev)
            {
                if (posDev) cu->memFree(posDev);
                if (quatDev) cu->memFree(quatDev);
                if (rowMapDev) cu->memFree(rowMapDev);
                prepFailed = true;
                transientPrepFailed = true;
                break;
            }
            // Order behind the ovstage producer before reading its matrix column, then decompose it into the two
            // session-owned columns. stream and wait_event are independent dependencies, so honour BOTH when both
            // are set (stream first), as the rigid and joint device drains do; both are 0 on the synchronous
            // (parity) read path. Fail CLOSED on any error (the producer wait, the subMap upload, or the decompose
            // launch): nothing is committed for this scene yet, so a failed decompose must not scatter the
            // uninitialised pos/quat scratch into the bodies as poses.
            bool prepared = true;
            if (batch.values.deviceStream)
                prepared = getCudaShim() &&
                           getCudaShim()->streamSynchronize(static_cast<uintptr_t>(batch.values.deviceStream), nullptr);
            if (prepared && batch.values.deviceWaitEvent)
                prepared = getCudaShim() &&
                           getCudaShim()->streamWaitEvent(uintptr_t(0),
                                                          static_cast<uintptr_t>(batch.values.deviceWaitEvent), 0,
                                                          nullptr);
            prepared = prepared && (cu->memcpyHtoD(rowMapDev, subMap.data(), size_t(rows) * sizeof(uint32_t)) == 0);
            const PxU32* const srcRowDev = reinterpret_cast<const PxU32*>(rowMapDev);
            float* const posOut = reinterpret_cast<float*>(posDev);
            float* const quatOut = reinterpret_cast<float*>(quatDev);
            prepared = prepared &&
                       (batch.values.type == parse::ColumnType::eFloat ?
                            omni::physx::tensors::decomposeWorldMatrixOvStage(
                                static_cast<const float*>(batch.values.data), srcRowDev, posOut, quatOut, rows) :
                            omni::physx::tensors::decomposeWorldMatrixOvStage(
                                static_cast<const double*>(batch.values.data), srcRowDev, posOut, quatOut, rows));
            if (!prepared)
            {
                cu->memFree(posDev);
                cu->memFree(quatDev);
                cu->memFree(rowMapDev);
                prepFailed = true;
                transientPrepFailed = true;
                break;
            }
        }

        // Scatter the two derived columns as borrowed device columns (no host copy). The columns are COMPACT (one
        // row per serviceable body, in g.bodies order), so the scatter is dense -- no srcRows gather needed.
        parse::ChangeBatch posBatch = batch;
        posBatch.values =
            parse::ColumnView{ parse::ColumnType::eFloat3, devOrd, rows, reinterpret_cast<const void*>(posDev) };
        posBatch.numChanges = rows;
        parse::ChangeBatch oriBatch = batch;
        oriBatch.values =
            parse::ColumnView{ parse::ColumnType::eFloat4, devOrd, rows, reinterpret_cast<const void*>(quatDev) };
        oriBatch.numChanges = rows;

        bool sceneOk =
            drainScatterRigidGroup(owner, source, scene, g.bodies, g.gkeys, *posRow, posBatch, nullptr, false);
        sceneOk = drainScatterRigidGroup(owner, source, scene, g.bodies, g.gkeys, *oriRow, oriBatch, nullptr,
                                         false) &&
                  sceneOk;
        {
            PxScopedCudaLock _lock(*ctxMgr);
            cu->memFree(posDev);
            cu->memFree(quatDev);
            cu->memFree(rowMapDev);
        }
        scattered = scattered && sceneOk;
        if (!sceneOk)
            break; // any failed scatter -> Failed below (held + retried); stop before posing more scenes
    }

    // Failed holds the drain cursor and retries the same ordinal; NotHandled falls back to the per-prim transform
    // path, which is INERT on a DirectGPU scene (the per-prim transform setter writes only the CPU core the solver
    // ignores), so NotHandled for a serviceable body would advance the ordinal while applying nothing. Hence every
    // failure after the device path was selected reports Failed: a TRANSIENT device-prep failure here (alloc /
    // producer sync / rowMap upload / decompose), and ANY failed drainScatterRigidGroup call -- whether it committed
    // part of a batch (must not fall back: a re-run double-applies) or failed before its setter (scatterGroup's row
    // resolve / row refresh, the plan, a producer handoff). drainScatterRigidGroup returns true for "nothing to
    // scatter", so a false is always a real failure. Only the persistent no-CUDA-context bail is NotHandled.
    // On success, report the skipped rows so onSourceChange runs the parse path for exactly those keys.
    if (!scattered || prepFailed)
        return (t_scatterCommitAttempted || transientPrepFailed || !scattered) ? DrainResult::Failed :
                                                                                 DrainResult::NotHandled;
    if (unresolvedRows)
        unresolvedRows->insert(unresolvedRows->end(), skipped.begin(), skipped.end());
    return DrainResult::Applied;
}

// Vehicle wheel controls (driveTorque / brakeTorque / steerAngle). This drain fills the whole cached
// wheel set by cache position and only accepts batches covering every wheel in the scene. Partial
// batches and cold caches fall back to parsing. Generic sessions independently support selected rows.
// Returns true only when it both recognised a vehicle control and applied it.
static bool applyVehicleBatch(usdparser::AttachedStage& owner, parse::IPhysicsSource& source, const parse::ChangeBatch& batch,
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

    // Preserve the parse path's authored-value policy for values the generic setter rejects.
    // Decline before planning/scatter so invalid input cannot pin the change feed at this ordinal.
    const uint8_t* vals = static_cast<const uint8_t*>(batch.values.data);
    for (size_t i = 0; i < batch.numChanges; ++i)
    {
        float value;
        std::memcpy(&value, vals + i * sizeof(float), sizeof(float));
        if (!std::isfinite(value) || (brakeRow && value < 0.0f))
            return false;
    }

    const ObjectKey* keys = static_cast<const ObjectKey*>(batch.keys.data);
    for (PxScene* scene : ovx::allPhysicsScenes(owner))
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
        s.attach = owner.getAttachHandle();
        if (!planVehicleDataGroup(s, source, scene, row, nullptr) || s.groups.empty())
            return false;
        GroupSlot& slot = *s.groups.back();
        if (slot.floats.size() != batch.numChanges)
        {
            releaseGroupStorage(s);
            return false;
        }
        // Place the prevalidated authored values at their wheels' cache positions.
        const uint8_t* col = static_cast<const uint8_t*>(batch.values.data);
        for (size_t i = 0; i < batch.numChanges; ++i)
        {
            float v;
            std::memcpy(&v, col + i * sizeof(float), sizeof(float));
            slot.floats[place[i]] = v;
        }
        slot.committed = true;
        const bool ok = scatterGroup(owner, slot);
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
static bool applyArticulationBatch(usdparser::AttachedStage& owner, parse::IPhysicsSource& source, const parse::ChangeBatch& batch,
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
    // Per-axis DOF scalars: one lane per joint prim, exactly eFloat (an int32 scalar also has one
    // component but its bits would be read as a float below). The column may be host- OR device-resident: a
    // device column (batch.values.device >= 0, delivered by the ovstage.placement device read) is scattered
    // into the session's DOF-order device column device-to-device by scatterDofColumnOvStage, with no host
    // round trip; a host column takes the host fill below.
    if (batch.values.count != batch.numChanges || !batch.values.data ||
        batch.values.type != parse::ColumnType::eFloat)
        return false;
    const bool colDevice = batch.values.device >= 0;

    const ObjectKey* keys = static_cast<const ObjectKey*>(batch.keys.data);
    const float* col = colDevice ? nullptr : static_cast<const float*>(batch.values.data);
    if (!ovx::refreshArticulationCache(owner, ovx::allPhysicsScenes(owner)))
        return false;
    for (PxScene* scene : ovx::allPhysicsScenes(owner))
    {
        // Plan FIRST: planJointGroup runs ensureJointRecords, which is what POPULATES jc.slices/recs. The
        // coverage check below reads those slices, so it must run against a planned cache -- on a fresh
        // drain with no prior joint read the slices are otherwise empty and the batch falls through to the
        // per-object parse path. planJointGroup returns true with no group for a scene that owns no joints.
        WriteSession s;
        s.attach = owner.getAttachHandle(); // GroupSlot::releaseList frees the plan's path list only for a
                                            // resolvable attach; left 0, every joint drain leaks one list.
        if (!planJointGroup(s, source, scene, *row))
            return false;
        if (s.groups.empty())
            continue; // this scene owns no joints
        GroupSlot& slot = *s.groups.back();

        ovx::ArticulationReadCacheEntry* jcPtr = ovx::articulationCacheEntry(scene);
        if (!jcPtr)
        {
            releaseGroupStorage(s);
            return false;
        }
        // Single-DOF joints only: a multi-DOF joint (D6) needs the USD axis -> physxDofIdx map only the
        // parse layer has. The batch must cover every single-DOF joint of the scene, since planJointGroup
        // zeroed the whole column -- an omitted DOF would be written 0. A multi-DOF joint or a partial batch
        // is released and skipped (next scene, or the parse fallback); nothing was committed.
        std::unordered_map<ObjectKey, uint32_t, ObjectKey::Hash> keyToDof; // joint key -> flat DOF index
        bool multiDof = false;
        for (const ovx::JointSlice& sl : jcPtr->slices)
        {
            if (sl.count > 1)
            {
                multiDof = true;
                break;
            }
            if (sl.count == 1)
                keyToDof.emplace(sl.key, sl.offset);
        }
        bool covers = !multiDof && keyToDof.size() == batch.numChanges &&
                      static_cast<size_t>(slot.numOut) == batch.numChanges;
        for (size_t i = 0; i < batch.numChanges && covers; ++i)
            covers = keyToDof.count(keys[i]) != 0;
        if (!covers)
        {
            releaseGroupStorage(s);
            continue;
        }
        // Fill every single-DOF joint from the batch with the RAW authored column, then scatter. The
        // deg->rad fold and the joint's sign are applied by the setter scatterGroup runs
        // (setDof*OvStage -> dofInverseScaleFor, eAngularSigned = invAngScale * sign) exactly as the normal
        // ovphysx_write joint path does -- pre-folding here would apply invAngScale twice (~57x too small on
        // a revolute axis; linear is x1, so it is invisible on a prismatic joint).
        //
        // A wrapped revolute joint (eREVOLUTE, no limits) wraps its drive POSITION target into [-360, 360]
        // and warns, matching the parse path's InternalJoint::setArticulationDrivePositionTarget. The raw
        // tensor setter passes the value straight through: a CPU scene rejects a target past +-2Pi (it stays
        // at 0), while a DirectGPU scene accepts it, so there the wrap is about matching the parse path, not
        // avoiding a rejection. The record's cached isWrappedRevolute gates it, so it is a compare per DOF and
        // a fmod only out of range.
        const bool wrapTargets = (ovxName == OvxAttr::kJointPositionTarget);
        if (colDevice)
        {
            // Device drain: the input column is device-resident (ovstage.placement read). Build the DOF-offset
            // permutation and the wrapped-revolute flags on the host, upload them, and scatter the column into
            // the session's DOF-order device column device-to-device -- no host round trip. The deg->rad fold +
            // sign are applied by the setter scatterGroup runs, exactly as on the host path. A per-value wrap
            // warning cannot be emitted from the device (the values are not host-readable); the wrap itself is
            // applied on device (scatterDofColumnOvStage), so the result still matches the parse path.
            // One packed index per row: the DOF offset in the low 31 bits, the wrapped-revolute wrap flag in
            // the top bit (kScatterDofWrapBit). Offsets are bounded by slot.numOut (the scene's single-DOF
            // count) and so fit in 31 bits, so the scatter takes one uploaded column: one allocation and one
            // H2D upload per drain.
            std::vector<PxU32> perm(batch.numChanges);
            bool placed = true;
            for (size_t i = 0; i < batch.numChanges && placed; ++i)
            {
                const std::unordered_map<ObjectKey, uint32_t, ObjectKey::Hash>::const_iterator it =
                    keyToDof.find(keys[i]);
                placed = (it != keyToDof.end()) && (it->second < static_cast<uint32_t>(slot.numOut));
                if (!placed)
                    break;
                PxU32 packed = it->second;
                if (wrapTargets && static_cast<size_t>(it->second) < jcPtr->recs.size() &&
                    jcPtr->recs[it->second].isWrappedRevolute)
                    packed |= omni::physx::tensors::kScatterDofWrapBit;
                perm[i] = packed;
            }
            if (!placed || !slot.deviceData || !slot.ctxMgr)
            {
                releaseGroupStorage(s);
                return false;
            }
            PxScopedCudaLock _lock(*slot.ctxMgr);
            PxCudaContext* const cu = slot.ctxMgr->getCudaContext();
            // Order the scatter after the ovstage producer's write of the column, then fail closed on error
            // (both sync fields are 0 on the current self-synchronizing ovstage read, so neither wait fires
            // today). The scatter's memcpyHtoD and scatterDofColumnOvStage run on the legacy default stream, so
            // honour the producer handoff the same way the host-staged rigid path does. stream and wait_event are
            // INDEPENDENT dependencies -- a {stream, event} handoff requires draining the producer stream AND
            // waiting on its event -- so do BOTH when both are set (stream first), not one or the other: a
            // {stream, 0} handoff from a non-blocking producer stream would otherwise be unordered against this
            // default-stream read (AC-14, extended to joints).
            if (batch.values.deviceStream &&
                (!getCudaShim() ||
                 !getCudaShim()->streamSynchronize(static_cast<uintptr_t>(batch.values.deviceStream), nullptr)))
            {
                t_devicePrepFailed = true; // producer-ordering sync failed -> retryable, must not fall back
                releaseGroupStorage(s);
                return false;
            }
            if (batch.values.deviceWaitEvent &&
                (!getCudaShim() ||
                 !getCudaShim()->streamWaitEvent(uintptr_t(0),
                                                 static_cast<uintptr_t>(batch.values.deviceWaitEvent), 0, nullptr)))
            {
                t_devicePrepFailed = true; // producer-ordering wait failed -> retryable, must not fall back
                releaseGroupStorage(s);
                return false;
            }
            CUdeviceptr permDev = 0;
            bool ok = cu->memAlloc(&permDev, perm.size() * sizeof(PxU32)) == 0 &&
                      cu->memcpyHtoD(permDev, perm.data(), perm.size() * sizeof(PxU32)) == 0 &&
                      omni::physx::tensors::scatterDofColumnOvStage(
                          reinterpret_cast<float*>(slot.deviceData),
                          static_cast<const float*>(batch.values.data),
                          reinterpret_cast<const PxU32*>(permDev), static_cast<PxU32>(batch.numChanges), nullptr);
            // The scatter reads perm and writes the session column; wait for it before freeing perm and before
            // the setter reads the column. A host wait, like the host-staged rigid path -- the device joint
            // drain is not yet stream-ordered.
            if (ok && getCudaShim())
                ok = getCudaShim()->streamSynchronize(uintptr_t(0), nullptr);
            if (permDev)
                cu->memFree(permDev);
            if (!ok)
            {
                // The permutation alloc / H2D / scatter launch failed (e.g. CUDA_ERROR_OUT_OF_MEMORY). Nothing
                // committed, but hold the cursor so the same ordinal retries once the device recovers rather
                // than falling back to the inert-on-DirectGPU parse path (which would consume it, losing the DOF).
                t_devicePrepFailed = true;
                releaseGroupStorage(s);
                return false;
            }
        }
        else
        {
            std::vector<float> host(static_cast<size_t>(slot.numOut), 0.0f);
            bool placed = true;
            for (size_t i = 0; i < batch.numChanges && placed; ++i)
            {
                const std::unordered_map<ObjectKey, uint32_t, ObjectKey::Hash>::const_iterator it =
                    keyToDof.find(keys[i]);
                placed = (it != keyToDof.end()) && (it->second < static_cast<uint32_t>(slot.numOut));
                if (!placed)
                    break;
                float value = col[i];
                if (wrapTargets && static_cast<size_t>(it->second) < jcPtr->recs.size() &&
                    jcPtr->recs[it->second].isWrappedRevolute && (value >= 360.0f || value <= -360.0f))
                {
                    OMNI_LOG_WARN(kRoboticsLogChannel,
                                  "Physics USD: Drive position target set to %2.f on %s will be wrapped in "
                                  "[-360, 360] range.Consider setting explicit limits to enable use of unwrapped joints",
                                  value, owner.textFor(keys[i]));
                    value = std::fmod(value, 360.0f);
                }
                host[it->second] = value;
            }
            if (!placed)
            {
                releaseGroupStorage(s);
                return false;
            }
            if (slot.deviceData && slot.ctxMgr)
            {
                PxScopedCudaLock _lock(*slot.ctxMgr);
                // Check the staging H2D: a discarded failure would commit stale data and report Applied while the
                // authored target never reached the device DOF column. Fail retryable instead (the DirectGPU parse
                // fallback is inert and would advance the ordinal dropping it, AC-12).
                if (slot.ctxMgr->getCudaContext()->memcpyHtoD(slot.deviceData, host.data(),
                                                              host.size() * sizeof(float)) != 0)
                {
                    t_devicePrepFailed = true; // staged H2D copy failed -> retryable, must not fall back
                    releaseGroupStorage(s);
                    return false;
                }
            }
            else
            {
                std::copy(host.begin(), host.end(), slot.floats.begin());
            }
        }
        slot.committed = true;
        const bool ok = scatterGroup(owner, slot);
        releaseGroupStorage(s);
        return ok;
    }
    return false; // no scene's full single-DOF joint set matched -> parse-layer fallback
}

// Particle-set drain. USD authors particle positions / velocities as `points` / `velocities`
// (VtArray<GfVec3f>) on the set prim -- a ragged per-particle vec3 column. Routes only when the batch
// covers every particle set in the scene (planParticleGroup fills the whole compact column) and each
// set's delivered particle count matches, then fills each set's slice from the ragged column by set key.
static bool applyParticleBatch(usdparser::AttachedStage& owner, parse::IPhysicsSource& source, const parse::ChangeBatch& batch,
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
    for (PxScene* scene : ovx::allPhysicsScenes(owner))
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
    s.attach = owner.getAttachHandle();
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
        const bool ok = scatterGroup(owner, slot);
        releaseGroupStorage(s);
        return ok;
    }
    return false; // no scene's full particle set matched -> parse-layer fallback
}

// ---- Tendon drain (DirectGPU) -------------------------------------------------------------------------------------
//
// PhysX flags every tendon setter illegal on a DirectGPU scene, so the parse path's tendon updaters
// (PhysXTendonPropertiesUpdate.cpp) write nothing there. This applies the tendon-level properties -- stiffness,
// damping, limit stiffness, offset and the enabled switch of either kind, and a fixed tendon's rest length and
// limits -- through the DirectGPU tendon structs instead, with the updaters' semantics: the enabled gating, the
// clamps and the limits' push. A CPU scene's rows keep going to those updaters, which are legal there and also wake
// the articulation; a DirectGPU scene needs no wake, since eENABLE_DIRECT_GPU_API forces eDISABLE_SLEEPING. Values
// are read through the source exactly as the updaters read them.
//
// The per-axis and per-attachment attributes (gearing, forceCoefficient, localPos, a spatial tendon's leaf rest
// length and limits) are left to the parse path: PhysX's DirectGPU tendon-joint and attachment blocks address every
// tendon of an articulation as if it were the first.

enum class TendonDrainProp
{
    eStiffness,
    eDamping,
    eLimitStiffness,
    eOffset,
    eEnabled,
    eRestLength,
    eLowerLimit,
    eUpperLimit
};

static bool tendonDrainPropFor(std::string_view name, TendonDrainProp& out)
{
    static constexpr std::pair<std::string_view, TendonDrainProp> kProps[] = {
        { "stiffness", TendonDrainProp::eStiffness },
        { "damping", TendonDrainProp::eDamping },
        { "limitStiffness", TendonDrainProp::eLimitStiffness },
        { "offset", TendonDrainProp::eOffset },
        { "tendonEnabled", TendonDrainProp::eEnabled },
        { "restLength", TendonDrainProp::eRestLength },
        { "lowerLimit", TendonDrainProp::eLowerLimit },
        { "upperLimit", TendonDrainProp::eUpperLimit },
    };
    for (const std::pair<std::string_view, TendonDrainProp>& p : kProps)
        if (p.first == name)
        {
            out = p.second;
            return true;
        }
    return false;
}

// One property write the drain issues, and how its value lands (a plain set, or a limit side that pushes the far
// side). Rows carry indices into the scene's tendon cache of that kind, one value each.
struct TendonWriteKey
{
    omni::physx::tensors::TendonProperty prop;
    omni::physx::tensors::TendonFieldWrite mode;
    bool operator<(const TendonWriteKey& o) const
    {
        return std::tie(prop, mode) < std::tie(o.prop, o.mode);
    }
};

struct TendonWriteRows
{
    std::vector<size_t> tendons; // into TendonReadCacheEntry::tendons
    std::vector<float> values;
};

// Collect the writes the parse updater for `prop` would make on one tendon, read and clamped as it reads and clamps
// them. Appends nothing where the updater writes nothing (a disabled tendon's coefficient, an unreadable value, a
// NaN).
static void collectTendonWrites(usdparser::AttachedStage& owner,
                                parse::IPhysicsSource& source,
                                const ovx::TendonRec& tendon,
                                size_t tendonIndex,
                                const std::string& instance,
                                const std::string& usdAttr,
                                const TendonDrainProp prop,
                                std::map<TendonWriteKey, TendonWriteRows>& out)
{
    using omni::physx::tensors::TendonFieldWrite;
    using omni::physx::tensors::TendonProperty;
    const parse::ReadTime t = parse::ReadTime::defaultTime();
    const auto tok = [&](const char* suffix) { return source.internToken("physxTendon:" + instance + ":" + suffix); };
    const auto readFloat = [&](parse::TokenId name, float lo, float hi, float& v) {
        if (!internal::getValue<float>(owner, tendon.key, name, t, v))
            return false;
        // A NaN survives the clamp. The updaters' setters refuse it with an error and the DirectGPU write would not,
        // so it is refused and reported here, and the tendon keeps its value.
        if (std::isnan(v))
        {
            const std::string_view attr = source.tokenToString(name);
            OVX_RUNTIME_ERROR("update_from_ovstage: %.*s is NaN; the tendon keeps its current value.",
                              static_cast<int>(attr.size()), attr.data());
            return false;
        }
        v = v > hi ? hi : (v < lo ? lo : v);
        return true;
    };
    const auto emit = [&](TendonProperty field, TendonFieldWrite mode, float v) {
        TendonWriteRows& rows = out[TendonWriteKey{ field, mode }];
        rows.tendons.push_back(tendonIndex);
        rows.values.push_back(v);
    };
    const parse::TokenId self = source.internToken(usdAttr);
    float v = 0.0f;
    switch (prop)
    {
    case TendonDrainProp::eStiffness:
    case TendonDrainProp::eDamping:
    case TendonDrainProp::eLimitStiffness:
    {
        // A disabled tendon keeps these at 0, as the updaters do. An unauthored `tendonEnabled` means enabled, as the
        // parser builds the tendon.
        bool enabled = false;
        if (!internal::getValue<bool>(owner, tendon.key, tok("tendonEnabled"), t, enabled))
            enabled = true;
        if (!enabled || !readFloat(self, 0.0f, FLT_MAX, v))
            return;
        emit(prop == TendonDrainProp::eStiffness ? TendonProperty::eStiffness :
             prop == TendonDrainProp::eDamping   ? TendonProperty::eDamping :
                                                   TendonProperty::eLimitStiffness,
             TendonFieldWrite::eSet, v);
        return;
    }
    case TendonDrainProp::eOffset:
        if (readFloat(self, -FLT_MAX, FLT_MAX, v))
            emit(TendonProperty::eOffset, TendonFieldWrite::eSet, v);
        return;
    case TendonDrainProp::eEnabled:
    {
        // Enabling re-applies the three authored coefficients (each only if readable); disabling zeroes them.
        bool enabled = false;
        if (!internal::getValue<bool>(owner, tendon.key, self, t, enabled))
            return;
        static constexpr std::pair<const char*, TendonProperty> kGated[] = {
            { "stiffness", TendonProperty::eStiffness },
            { "limitStiffness", TendonProperty::eLimitStiffness },
            { "damping", TendonProperty::eDamping },
        };
        for (const std::pair<const char*, TendonProperty>& g : kGated)
        {
            v = 0.0f;
            if (!enabled || readFloat(tok(g.first), 0.0f, FLT_MAX, v))
                emit(g.second, TendonFieldWrite::eSet, v);
        }
        return;
    }
    case TendonDrainProp::eRestLength:
        if (readFloat(self, -FLT_MAX, FLT_MAX, v))
            emit(TendonProperty::eRestLength, TendonFieldWrite::eSet, v);
        return;
    case TendonDrainProp::eLowerLimit:
    case TendonDrainProp::eUpperLimit:
        if (readFloat(self, -FLT_MAX, FLT_MAX, v))
            emit(TendonProperty::eLimit,
                 prop == TendonDrainProp::eLowerLimit ? TendonFieldWrite::eLowPushHigh : TendonFieldWrite::eHighPushLow,
                 v);
        return;
    }
}

// Upload one property's records and values for one scene and write them through the view's tendon setter. Any
// failure is Failed (retryable): the parse fallback writes nothing on a DirectGPU scene, so falling back would drop
// the change.
static bool writeTendonProperty(PxScene* scene,
                                omni::physx::tensors::GpuArticulationView& view,
                                bool fixed,
                                const TendonWriteKey& write,
                                const std::vector<omni::physx::tensors::ArticulationTendonOvStageRecord>& records,
                                const std::vector<float>& values)
{
    PxCudaContextManager* const ctxMgr = scene->getCudaContextManager();
    PxCudaContext* const cu = ctxMgr ? ctxMgr->getCudaContext() : nullptr;
    if (!cu)
        return false;
    PxScopedCudaLock _lock(*ctxMgr);
    const size_t recBytes = records.size() * sizeof(records[0]);
    const size_t valBytes = values.size() * sizeof(float);
    CUdeviceptr recDev = 0, valDev = 0;
    const bool staged = cu->memAlloc(&recDev, recBytes) == 0 && cu->memAlloc(&valDev, valBytes) == 0 &&
                        cu->memcpyHtoD(recDev, records.data(), recBytes) == 0 &&
                        cu->memcpyHtoD(valDev, values.data(), valBytes) == 0;
    bool ok = false;
    if (staged)
    {
        omni::physics::tensors::TensorDesc td;
        td.device = ctxMgr->getDevice();
        td.dtype = omni::physics::tensors::TensorDataType::eFloat32;
        td.numDims = 1;
        td.dims[0] = static_cast<int>(values.size());
        td.data = reinterpret_cast<void*>(valDev);
        const omni::physx::tensors::ArticulationTendonOvStageRecord* recs =
            reinterpret_cast<const omni::physx::tensors::ArticulationTendonOvStageRecord*>(recDev);
        const PxU32 n = PxU32(records.size());
        ok = fixed ? view.setFixedTendonPropertiesOvStage(&td, recs, n, write.prop,
                                                          omni::physx::tensors::OvStageDeviceWriteArgs(), write.mode) :
                     view.setSpatialTendonPropertiesOvStage(&td, recs, n, write.prop,
                                                            omni::physx::tensors::OvStageDeviceWriteArgs());
    }
    if (recDev)
        cu->memFree(recDev);
    if (valDev)
        cu->memFree(valDev);
    return ok;
}

// The tendon drain. Rows whose prim and instance name a tendon of a DirectGPU scene that has stepped -- the root axis
// of a fixed tendon, the root attachment of a spatial one -- are written there; every other row (a CPU scene's, a
// not-yet-stepped scene's, a per-axis or per-attachment one, or one this drain cannot place) is reported for the parse
// path. Falls back whole when no row reached such a scene, or when the caller gave no sink for the rest. Every
// failure after a row is placed returns Failed directly, so the dispatcher's commit / prep flags are not set here.
static DrainResult applyTendonBatch(usdparser::AttachedStage& owner,
                                    parse::IPhysicsSource& source,
                                    const parse::ChangeBatch& batch,
                                    const std::string& usdAttr,
                                    std::vector<uint32_t>* unresolvedRows)
{
    static constexpr std::string_view kPrefix = "physxTendon:";
    if (usdAttr.compare(0, kPrefix.size(), kPrefix) != 0)
        return DrainResult::NotHandled;
    const size_t sep = usdAttr.rfind(':');
    if (sep <= kPrefix.size())
        return DrainResult::NotHandled;
    const std::string instance = usdAttr.substr(kPrefix.size(), sep - kPrefix.size());
    TendonDrainProp prop;
    if (!tendonDrainPropFor(std::string_view(usdAttr).substr(sep + 1), prop))
        return DrainResult::NotHandled;
    // A spatial tendon's rest length and limits are its leaf attachment's, not tendon-level.
    const bool fixedOnly = prop == TendonDrainProp::eRestLength || prop == TendonDrainProp::eLowerLimit ||
                           prop == TendonDrainProp::eUpperLimit;

    // Until a DirectGPU scene's first step, PhysX still accepts the tendon setters there and the DirectGPU tendon write
    // refuses, so that scene's rows are left to the parse updaters, as a CPU scene's are. Decided per scene, not by the
    // attachment's step count, since scenes can be stepped one at a time; a scene's timestamp starts at 1 and passes it
    // once a step completes.
    std::vector<PxScene*> gpuScenes;
    for (PxScene* scene : ovx::allPhysicsScenes(owner))
        if (scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API) && scene->getTimestamp() > 1)
            gpuScenes.push_back(scene);
    if (gpuScenes.empty())
        return DrainResult::NotHandled;

    // The read's tendon walk, through the shared accessor, so the drain and the read agree on which prims are
    // tendon roots. Indexed here by prim for this instance; kind 0 is fixed, 1 spatial.
    using TendonsByKey = std::unordered_multimap<ObjectKey, size_t, ObjectKey::Hash>;
    std::vector<std::array<TendonsByKey, 2>> byKey(gpuScenes.size());
    for (size_t s = 0; s < gpuScenes.size(); ++s)
        for (int kind = 0; kind < (fixedOnly ? 1 : 2); ++kind)
        {
            const ovx::TendonReadCacheEntry& tc = ovx::tendonCacheEntry(gpuScenes[s], kind == 0);
            for (size_t i = 0; i < tc.tendons.size(); ++i)
                if (!tc.tendons[i].instance.empty() && tc.tendons[i].instance == instance)
                    byKey[s][kind].emplace(tc.tendons[i].key, i);
        }

    // Place every row first, so a whole fallback is decided before anything is written.
    const ObjectKey* keys = static_cast<const ObjectKey*>(batch.keys.data);
    std::vector<std::array<std::vector<size_t>, 2>> placed(gpuScenes.size());
    std::vector<uint32_t> unplaced;
    for (size_t i = 0; i < batch.numChanges; ++i)
    {
        bool hit = false;
        for (size_t s = 0; s < gpuScenes.size(); ++s)
            for (int kind = 0; kind < 2; ++kind)
            {
                const std::pair<TendonsByKey::const_iterator, TendonsByKey::const_iterator> range =
                    byKey[s][kind].equal_range(keys[i]);
                for (TendonsByKey::const_iterator it = range.first; it != range.second; ++it)
                {
                    placed[s][kind].push_back(it->second);
                    hit = true;
                }
            }
        if (!hit)
            unplaced.push_back(static_cast<uint32_t>(i));
    }
    if (unplaced.size() == batch.numChanges || (!unplaced.empty() && !unresolvedRows))
        return DrainResult::NotHandled;

    omni::physx::tensors::SimulationBackend* backend = omni::physx::tensors::GetSimulationBackend();
    for (size_t s = 0; s < gpuScenes.size(); ++s)
        for (int kind = 0; kind < 2; ++kind)
        {
            if (placed[s][kind].empty())
                continue;
            const bool fixed = kind == 0;
            const ovx::TendonReadCacheEntry& tc = ovx::tendonCacheEntry(gpuScenes[s], fixed);
            std::map<TendonWriteKey, TendonWriteRows> writes;
            for (const size_t ti : placed[s][kind])
                collectTendonWrites(owner, source, tc.tendons[ti], ti, instance, usdAttr, prop, writes);
            if (writes.empty())
                continue; // every placed row is one its updater would not write (a disabled tendon, ...)

            // Records as the tendon write derives them: (the articulation's superset view row, the tendon index).
            uint64_t generation = 0;
            const std::vector<omni::physx::tensors::ArticulationEntry>* entries = nullptr;
            std::vector<uint32_t> localToRow;
            omni::physx::tensors::GpuArticulationView* view = nullptr;
            if (!backend ||
                !acquireArticulationRows(*backend, gpuScenes[s], true, tc.artis, generation, entries, localToRow,
                                         &view) ||
                !view)
                return DrainResult::Failed;
            for (const std::pair<const TendonWriteKey, TendonWriteRows>& w : writes)
            {
                std::vector<omni::physx::tensors::ArticulationTendonOvStageRecord> records;
                records.reserve(w.second.tendons.size());
                for (const size_t ti : w.second.tendons)
                {
                    const ovx::TendonRec& tr = tc.tendons[ti];
                    if (tr.viewArtiIdx >= localToRow.size())
                        return DrainResult::Failed;
                    omni::physx::tensors::ArticulationTendonOvStageRecord r;
                    r.viewArtiIdx = localToRow[tr.viewArtiIdx];
                    r.tendonIdx = tr.tendonIdx;
                    records.push_back(r);
                }
                if (!writeTendonProperty(gpuScenes[s], *view, fixed, w.first, records, w.second.values))
                    return DrainResult::Failed;
            }
        }
    if (unresolvedRows)
        unresolvedRows->insert(unresolvedRows->end(), unplaced.begin(), unplaced.end());
    return DrainResult::Applied;
}

// The CUDA device ordinal shared by the attachment's scenes, or -1 unless EVERY attached scene is
// DirectGPU on the same device (a mixed DirectGPU + CPU or multi-device stage, or no scenes, gives -1).
// The ovstage value drain uses it (via OvstageChangeFeed::setDeviceOrdinal) to read device-eligible
// columns straight from that device: the device read must target the same device the scatter writes to.
OMNI_OVX_WRITE_API int ovxDirectGpuDeviceOrdinal(usdparser::AttachedStage& owner)
{
    // Device delivery is all-or-nothing for the attachment: the feed sets ONE device ordinal and serves
    // every body on it. Return a device only when EVERY physics scene is DirectGPU on the SAME device;
    // otherwise -1, so a mixed DirectGPU + CPU (or multi-device) stage stages every column on the host and no
    // CPU-scene group reaches the device staging branch, which needs a CUDA context the CPU scene lacks.
    int device = -1;
    for (PxScene* scene : ovx::allPhysicsScenes(owner))
    {
        if (!scene)
            continue;
        if (!scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API))
            return -1; // a CPU scene -> host staging for the whole attachment
        PxCudaContextManager* ctxMgr = scene->getCudaContextManager();
        if (!ctxMgr)
            return -1; // DirectGPU but no context manager -> cannot serve on device
        const int sceneDevice = static_cast<int>(ctxMgr->getDevice());
        if (device < 0)
            device = sceneDevice;
        else if (device != sceneDevice)
            return -1; // DirectGPU scenes on different devices -> one ordinal cannot cover both
    }
    return device; // -1 when the attachment has no scenes
}

// Per-scene partition of a drained value batch: the bodies to scatter, their keys, and their row indices
// into the batch column. Hoisted to file scope so a stable drain can cache and reuse it.
struct DrainSceneGroup
{
    std::vector<PxRigidBody*> bodies;
    std::vector<ObjectKey> keys;
    std::vector<uint32_t> rows;
};

// Cache of a value batch's resolve+partition, keyed by write attribute (row). A drain that rewrites the
// same bodies each time (the dense control-loop pattern) hands the SAME key set every drain, and the
// partition is a pure function of (key set, object set, kinematic flags), so on a stable drain it is reused
// instead of re-resolving every key. Validity:
//   - recordLifetimeEpoch: an object was created/retired (also covers creation-time localSpaceVelocities).
//   - kinematicToggleEpoch: a body toggled kinematic (it can no longer take an authored velocity).
//   - the key set itself (count + bytes): a re-discovered authoring set forces a rebuild.
// Only rows that do NOT scan articulation links are cacheable: a link-writable row scans links fresh each
// drain (kOvxAll). These are not the per-frame hot path (velocity is not link-writable), so they are left
// uncached -- a link-writable row builds fresh every drain and never publishes a reusable plan.
struct DrainPartitionCache
{
    uint64_t recordEpoch = ~uint64_t(0);
    uint64_t flagEpoch = ~uint64_t(0);
    std::vector<ObjectKey> keys;
    std::unordered_map<PxScene*, DrainSceneGroup> byScene;
    std::vector<uint32_t> unresolvedRows;
    size_t resolved = 0;
    bool valid = false;
};

// Keyed by (attach, attribute row, batch row count): one device value read delivers the SAME attribute as
// more than one batch in a drain -- the query's base handle (the bulk) plus its delta -- so a slot per row
// alone would let the two overwrite each other every drain. The row count separates them (a real key-set
// change of the same count still forces a rebuild via the byte compare below), and the attach separates two
// attachments draining the same count so they do not collide on one entry.
//
// Neither the count nor the attach dimension is self-bounding: a producer whose per-drain row count varies
// (per-env resets, event-driven edits) would keep one entry per distinct count, and detached attachments would
// linger, for the process lifetime. On a cold key, drop entries for attachments that have detached (handle no
// longer resolves) -- mirroring drainSlotCacheFor's detached-attachment purge -- then, if the map is still past a small
// cap, clear it and let the next drain rebuild the handful it needs. A stable control loop keeps a constant
// (attach, count), takes the warm-hit early return, and pays nothing.
using DrainPartitionCacheMap = std::map<std::tuple<uint64_t, const WriteAttributeRow*, uint32_t>, DrainPartitionCache>;
static DrainPartitionCache& drainPartitionCacheFor(AttachedStage& owner, const WriteAttributeRow* row, uint32_t count)
{
    static DrainPartitionCacheMap cacheBySlot;
    const uint64_t attach = owner.getAttachHandle();
    const std::tuple<uint64_t, const WriteAttributeRow*, uint32_t> key{ attach, row, count };
    const DrainPartitionCacheMap::iterator hit = cacheBySlot.find(key);
    if (hit != cacheBySlot.end())
        return hit->second; // warm hit: steady state pays nothing
    if (UsdLoad* load = UsdLoad::getUsdLoadIfCreated())
        for (DrainPartitionCacheMap::iterator it = cacheBySlot.begin(); it != cacheBySlot.end();)
            it = (std::get<0>(it->first) == attach || load->getAttachedStageByHandle(std::get<0>(it->first)))
                     ? std::next(it)
                     : cacheBySlot.erase(it);
    constexpr size_t kMaxSlots = 16;
    if (cacheBySlot.size() >= kMaxSlots)
        cacheBySlot.clear(); // returns a fresh entry below; no caller holds a reference across this call
    return cacheBySlot[key];
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
//   physxTendon:<inst>:* tendon-level  none               THE DRAIN (applyTendonBatch): the parse updaters'
//     on a DirectGPU scene                                gating, clamps and limit push through the DirectGPU
//                                                         tendon structs; a CPU scene's rows go to the updaters
//   physxTendon:<inst>:* per axis /    axis gearing       NOT DRAINED -- the parse path's updaters (which fold
//     attachment                       rad2deg, localPos  these), legal on a CPU scene only
//                                      x scale
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
OMNI_OVX_WRITE_API DrainResult applyOvstageValueBatch(usdparser::AttachedStage& owner, parse::IPhysicsSource& source, const parse::ChangeBatch& batch,
                                                      std::vector<uint32_t>* unresolvedRows)
{
    // Internal change-feed drains share the articulation cache with public reads/writes.
    std::lock_guard<carb::tasking::RecursiveMutexWrapper> cacheLock(ovx::outputCacheMutex());
    // Value changes only. Deletes / new objects / typeName resyncs (structural) and attributes with no
    // backend setter are NotHandled -> onSourceChange keeps the parse-layer property path for those.
    if (batch.isDelete || batch.numChanges == 0 || !batch.property.valid() || !batch.keys.data)
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
    t_devicePrepFailed = false;
    // A committed partial batch, or a selected-device-path prep failure, both become Failed (hold the cursor):
    // the first must not double-apply on fallback, the second must stay retryable instead of being lost to the
    // inert-on-DirectGPU parse path. Otherwise NotHandled, which falls back.
    const auto notHandledOrFailed = []
    { return (t_scatterCommitAttempted || t_devicePrepFailed) ? DrainResult::Failed : DrainResult::NotHandled; };

    // Tendon-level attributes (physxTendon:<inst>:*) on a DirectGPU scene, where the parse path's tendon setters
    // are illegal and write nothing. They read their values through the source as the parse updaters do, so they
    // run before the value-column check below; a CPU scene's rows go to those updaters (applyTendonBatch).
    const DrainResult tendonResult = applyTendonBatch(owner, source, batch, usdAttr, unresolvedRows);
    if (tendonResult != DrainResult::NotHandled)
        return tendonResult;
    if (!batch.values.data)
        return DrainResult::NotHandled;

    // Pose (omni:fabric:worldMatrix) routes through the DirectGPU device decompose + scatter when the change
    // feed delivered a device column gated to the authored-local-frame set. It SPLITS: the serviceable bodies
    // are posed on device and any row it cannot serve is reported in `unresolvedRows` for the per-prim transform
    // path (which applies poses on a CPU scene; it is inert on DirectGPU, so a whole-batch fallback there would
    // drop the serviceable poses -- see applyPoseBatch). NotHandled only when nothing was serviceable.
    const DrainResult poseResult = applyPoseBatch(owner, source, batch, usdAttr, unresolvedRows);
    if (poseResult != DrainResult::NotHandled)
        return poseResult;
    // Vehicle wheel controls route through the CPU-only vehicle write path (whole-wheel-set).
    if (applyVehicleBatch(owner, source, batch, usdAttr))
        return DrainResult::Applied;
    // The rigid MASS FRAME (physics:mass / centerOfMass / diagonalInertia / principalAxes) is deliberately
    // NOT drained: those attributes are coupled -- the parse path routes all of them through
    // updateBodyDensity -> addDirtyMassActor, a holistic recompute of mass + inertia + COM together (from the
    // body's shapes and every authored mass attribute), applied at the next step. A per-attribute write-row
    // scatter cannot reproduce that coupling (e.g. changing mass alone must rescale auto-derived inertia), so
    // the whole frame is left NotHandled and falls back to that recompute.
    // Articulation DOF (per-axis drive targets / joint state) route through the joint write path
    // (whole single-DOF set of a scene).
    if (applyArticulationBatch(owner, source, batch, usdAttr))
        return DrainResult::Applied;
    // Particle sets deliver ragged per-vertex `points` / `velocities` columns; those ARE live sim state,
    // so the drain scatters them.
    if (applyParticleBatch(owner, source, batch, usdAttr))
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
    // Standalone rigids come from the epoch-cached map (rebuilt only on a topology change, not per
    // drain). Articulation links are scanned fresh into a separate map each drain (kOvxAll) only for an
    // attribute PhysX can set on one (a velocity row has linkWritable == false, so no link scan). A key
    // maps to at most one body, standalone taking precedence.
    //
    // Links are scanned kOvxAll, like the standalone-rigid scan: this is a WRITE, and a write must reach a
    // SLEEPING link exactly as it reaches a sleeping standalone body. Scanning kOvxActive here dropped every
    // link of a sleeping articulation, so a drained `physxRigidBody:disableGravity` on one fell through to
    // the per-object parse path instead of the write backend, and that path did not wake the articulation,
    // so re-enabling gravity on a sleeping articulation left it asleep and motionless.
    const StandaloneRigidLookup& standalone = cachedStandaloneRigids(owner, velocityRow);
    const StandaloneKeyToBody& standaloneKeyToBody = standalone.keyToBody;
    StandaloneKeyToBody linkKeyToBody;
    if (row->linkWritable)
    {
        ovx::RigidRecordScan linkScan;
        ovx::scanLinkRecords(owner, kOvxAll, ovx::ActiveActorSet{}, linkScan);
        for (const ovx::RigidSceneBucket& b : linkScan.byScene)
            for (size_t i = 0; i < b.keys.size(); ++i)
                linkKeyToBody.emplace(b.keys[i], std::make_pair(b.scene, b.bodies[i]));
    }

    // Resolve+partition is a pure function of the key set and the object/flag epochs, and a drain that
    // rewrites the same bodies each time (the dense control-loop pattern) re-sends the same key set every
    // drain, so it is cached per attribute and reused on a stable drain instead of re-resolving every key. A
    // link-scanning row is not the per-frame hot path, so it is left uncached (builds fresh, publishes no plan).
    const bool cacheable = !row->linkWritable;
    DrainPartitionCache& pc = drainPartitionCacheFor(owner, row, static_cast<uint32_t>(batch.numChanges));
    const uint64_t recEpoch = owner.getInternalPhysXDatabase().recordEpoch();
    const uint64_t flagEpoch = owner.kinematicVersion.load(std::memory_order_relaxed);

    const bool hit = cacheable && pc.valid && pc.recordEpoch == recEpoch && pc.flagEpoch == flagEpoch &&
                     pc.keys.size() == batch.numChanges &&
                     (batch.numChanges == 0 ||
                      std::memcmp(pc.keys.data(), keys, batch.numChanges * sizeof(ObjectKey)) == 0);

    size_t resolved = 0;
    if (hit)
    {
        // Same key set, same objects, same kinematic state: the recorded partition stands. The bodies are
        // reused; only the fresh device values (batch.values) differ. The skip sink is re-emitted so
        // onSourceChange still runs the parse path for the same unserviceable rows.
        resolved = pc.resolved;
    }
    else
    {
        pc.byScene.clear();
        pc.unresolvedRows.clear();
        // A batch row the drain will not service (not a body/link, kinematic, or local-frame velocity) is
        // recorded so onSourceChange runs its per-object parse fallback for exactly that key.
        for (size_t i = 0; i < batch.numChanges; ++i)
        {
            PxScene* hitScene = nullptr;
            PxRigidBody* hitBody = nullptr;
            const StandaloneKeyToBody::const_iterator sit = standaloneKeyToBody.find(keys[i]);
            if (sit != standaloneKeyToBody.end())
            {
                hitScene = sit->second.first;
                hitBody = sit->second.second;
            }
            else if (row->linkWritable)
            {
                const StandaloneKeyToBody::const_iterator lit = linkKeyToBody.find(keys[i]);
                if (lit != linkKeyToBody.end())
                {
                    hitScene = lit->second.first;
                    hitBody = lit->second.second;
                }
            }
            if (!hitBody)
            {
                pc.unresolvedRows.push_back(static_cast<uint32_t>(i));
                continue; // not a rigid body or (link-writable) link -> parse-layer property path
            }
            if (velocityRow)
            {
                const PxRigidDynamic* dyn = hitBody->is<PxRigidDynamic>();
                if (dyn && (dyn->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
                {
                    pc.unresolvedRows.push_back(static_cast<uint32_t>(i));
                    continue; // kinematic -> surface velocity on the parse path, not setLinearVelocity here
                }
                if (standalone.localSpaceVelocityBodies.count(hitBody))
                {
                    pc.unresolvedRows.push_back(static_cast<uint32_t>(i));
                    continue; // local-frame velocity -> parse path reframes it (rotate + scale)
                }
            }
            DrainSceneGroup& g = pc.byScene[hitScene];
            g.bodies.push_back(hitBody);
            g.keys.push_back(keys[i]);
            g.rows.push_back(static_cast<uint32_t>(i));
            ++resolved;
        }
        pc.resolved = resolved;
        pc.keys.assign(keys, keys + batch.numChanges);
        pc.recordEpoch = recEpoch;
        pc.flagEpoch = flagEpoch;
        pc.valid = cacheable; // a link-scanning row built fresh, but its plan is not reusable
    }
    const std::unordered_map<PxScene*, DrainSceneGroup>& byScene = pc.byScene;

    // Split, don't abandon. Scatter the resolved keys through the backend and let onSourceChange run the
    // parse-layer path for the skipped ones -- keeping the drain's DirectGPU-correct path for every
    // serviceable key in a batch that also carries a skip (else a whole-batch fallback would route the
    // serviceable bodies to the per-object velocity setter, which is inert on DirectGPU, dropping them).
    // Two cases still fall back WHOLE: no key resolved (nothing to scatter), or the caller passed no sink for
    // the skipped rows (`unresolvedRows` null) -- there a partial Applied would make onSourceChange silently
    // drop them, so the older all-or-nothing is preserved. A skipped key is never committed, so the fallback
    // cannot double-apply.
    if (resolved == 0 || (resolved != batch.numChanges && !unresolvedRows))
        return notHandledOrFailed();
    if (unresolvedRows)
        unresolvedRows->insert(unresolvedRows->end(), pc.unresolvedRows.begin(), pc.unresolvedRows.end());

    bool ok = true;
    for (const std::pair<PxScene* const, DrainSceneGroup>& e : byScene)
    {
        const bool whole = (e.second.bodies.size() == batch.numChanges);
        ok = drainScatterRigidGroup(owner, source, e.first, e.second.bodies, e.second.keys, *row, batch,
                                    whole ? nullptr : &e.second.rows, angular) &&
             ok;
    }
    // ok is false if any scene's scatter failed. Route it through notHandledOrFailed() rather than a bare
    // Failed: a scatter that committed part of the set set t_scatterCommitAttempted, and a TRANSIENT device-prep
    // failure on a selected device path (a producer-ordering wait/sync, the deg->rad scale launch, a D2H staging
    // copy, or a borrowed column's handoff) set t_devicePrepFailed -- both map to Failed, so the cursor holds and
    // the SAME ordinal retries. Do NOT fall back in either case: a committed part would double-apply, and a
    // selected-device prep failure's parse fallback is INERT on a DirectGPU scene (the per-object velocity setter
    // writes only the CPU core the solver ignores), so falling back would advance the ordinal while applying
    // nothing -- losing the write with no retry (the bug this guards, same class as the joint device drain). A
    // genuine non-handled failure -- nothing was selected or scattered: a device column it was never going to
    // borrow, or a planRigidGroup planning / cold-cache / key-resolution failure -- leaves both flags clear and
    // comes back NotHandled, falling back to the parse path. A bare Failed for THAT case would be unrecoverable:
    // onSourceChange returns false -> drainRange holds the cursor -> a deterministic (persistent) failure grows the
    // held range every frame until it falls off the retained history and ovstage changes stop reaching physics.
    return ok ? DrainResult::Applied : notHandledOrFailed();
}

} // namespace omni::physx
