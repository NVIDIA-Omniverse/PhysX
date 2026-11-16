// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-BENCHMARK-001
 * @covers AC-1 AC-4 AC-5
 */

// Authoring.kinematic_drive_*: driving a KINEMATIC body every tick through the
// two control planes, measured per step. A kinematic slab carries a resting
// dynamic rider, so each tick's control update is a real kinematic-target move
// that the rider follows.
//
//   kinematic_drive_ovstage_cpu  (Path A, import)  author omni:xform on the
//       kinematic slab at the next ordinal -> advance_write_floor + wait(seal)
//       -> ovphysx_update_from_ovstage({start,end}) -> ovphysx_step_sync. This
//       is the OVStage control-plane path: each drain re-runs the bridge's
//       discovery of the slab's binding/contact state before physics sees the
//       new target.
//
//   kinematic_drive_direct_cpu   (Path B, direct)  ovphysx_write of
//       [kinematicTargetPosition, kinematicTargetOrientation] through a write
//       session over a query opened ONCE, fill + commit + release -> step_sync.
//       No OVStage drain, no per-tick rediscovery.
//
// The pair prices the import control plane against the direct kinematic-target
// write for the identical visible result (slab moves, rider rides), which is
// the question a front end driving a kinematic body every tick faces. It is the
// AuthoringTeleport pattern specialized to a kinematic body via the real direct
// kinematic-target write rather than the deprecated tensor binding.
//
// CPU ONLY. The kinematicTargetPosition/Orientation attributes are CONDITIONAL
// (CPU-only) in ovphysx_writability, so Path B has no device destination. The
// whole scene authors physxScene:enableGPUDynamics = false, and both rows gate
// on forceGpu() being off, matching the Step.*_cpu / Authoring convention.
//
// Scene load and per-path setup happen in startRun(); the correctness gate (the
// slab moved AND the rider was carried) runs in endRun(), outside the measured
// window. One measured step is one drive iteration, so the harness reports the
// per-tick cost directly.

#include "framework/UsdPCH.h"

#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"
#include "../framework/BmUtils.h"
#include "../BenchmarkFailure.h"
#include "../TaskCompletion.h"
#include "AuthoringCommon.h"

#include <ovphysx/experimental/ovphysx.hpp>
#include <ovphysx/ovphysx_attributes.h>

#include <cstring>
#include <string>


void initAuthoringKinematicDrive()
{
}


namespace
{

const char* const kSlabPath = "/World/KinematicSlab0";
const char* const kRiderPath = "/World/DynamicRider0";

// The slab is driven along +X. These bound the untimed verify gate: the slab is
// commanded to a distinctive X and must arrive there, and the rider (resting on
// top) must have been carried a meaningful fraction of the way.
const float kVerifySlabX = 1.25f;
const float kVerifyTolerance = 0.05f;
const float kRiderCarryMin = 0.3f;


// Row-major identity transform with translation at indices 12-14 (USD/PhysX
// row-vector convention), scale left at identity: the slab's authored scale is
// folded into omni:xform by the bridge from the loaded prim, and the drive only
// changes the translation.
void fillSlabXform(double* matrix16, float x)
{
    std::memset(matrix16, 0, 16 * sizeof(double));
    matrix16[0] = 1.0;
    matrix16[5] = 1.0;
    matrix16[10] = 1.0;
    matrix16[12] = static_cast<double>(x);
    matrix16[13] = 0.0;
    matrix16[14] = 0.0;
    matrix16[15] = 1.0;
}


// Per-iteration target X, cycling so successive writes differ and a stale-value
// regression cannot pass unnoticed. Monotone within the cycle so the rider is
// actually dragged.
float iterationX(int iteration)
{
    return 0.02f * static_cast<float>((iteration % 23) + 1);
}


// ---------------------------------------------------------------------------
// Shared base.
// ---------------------------------------------------------------------------

class KinematicDriveBase : public BmBenchmark
{
public:
    explicit KinematicDriveBase(const char* rowName) : mRowName(rowName)
    {
    }

    // CPU-only, by scene construction. Reject the GPU pass outright.
    bool isValid() const override
    {
        if (BmGlobals::getInstance().forceGpu())
        {
            return false;
        }
        return BmGlobals::getInstance().getPhysX() != nullptr;
    }

    uint32_t getNbSteps() const override
    {
        return 100;
    }

    uint32_t getNbRuns() const override
    {
        return 5;
    }

    void startRun() override
    {
        mPhysX = BmGlobals::getInstance().getPhysX();
        mIteration = 0;
        mStepOk = false;
        mSetupOk = false;
        if (!authoringbm::prepareStageAttachmentForRun(
                mStageAttachment, mRowName, &mLeftoverAttachmentReported))
        {
            return;
        }
        resetCase();
        if (!mPhysX)
        {
            return;
        }

        mAttached = false;
        authoringbm::GpuFallbackScope gpuScope(/*arm=*/false, mRowName);

        const std::string path = BmGlobals::getInstance().getDataFolder() +
                                 "/../benchmarks/data/" + sceneAsset();
        if (!authoringbm::populateAndAttachChecked(mPhysX, authoringbm::UsdSource::eFile, path,
                                                   "ovphysx-kinematic-drive", mStageAttachment, mRowName,
                                                   &mAttached))
        {
            return;
        }
        if (!authoringbm::checkDeviceEvidenceAndWarmup(mPhysX->handle(), /*gpu=*/false, gpuScope, mRowName))
        {
            return;
        }

        // Settle the resting rider onto the slab so it is in live contact before
        // the timed drive begins. Outside the measured window.
        for (int s = 0; s < 10; ++s)
        {
            if (!stepSync())
            {
                return;
            }
        }

        mSetupOk = setup();
        mStepOk = mSetupOk;
        // Untimed same-stage warmup: the harness dummy run warms a different
        // stage that is then cleared, so without this the first drive of every
        // recorded run would be timed.
        if (mSetupOk)
        {
            for (int w = 0; w < 5; ++w)
            {
                if (!driveOnce(iterationX(w)))
                {
                    mStepOk = false;
                    mSetupOk = false;
                    break;
                }
            }
        }
    }

    void preStep() override
    {
    }

    void endRun() override
    {
        if (mSetupOk && mStepOk)
        {
            verify();
        }
        else if (mSetupOk && !mStepOk)
        {
            bmRecordFailure(mRowName, "kinematic drive sequence did not complete");
        }
        teardown();
        if (!authoringbm::clearOvstageChecked(mAttached ? mPhysX : nullptr, mStageAttachment, mRowName))
        {
            bmRecordFailure(mRowName, "ovstage teardown failed; retaining the stage attachment");
            mLeftoverAttachmentReported = true;
        }
        else
        {
            mAttached = false;
        }
    }

protected:
    void step() override
    {
        if (!mSetupOk || !mStepOk)
        {
            return;
        }
        if (!driveOnce(iterationX(mIteration)))
        {
            mStepOk = false;
        }
        ++mIteration;
    }

    // Per-case hooks.
    virtual void resetCase()
    {
    }
    virtual bool setup() = 0;
    virtual bool driveOnce(float targetX) = 0;
    virtual bool driveVerify(float targetX) = 0;
    virtual void teardown()
    {
    }

    virtual const char* sceneAsset() const
    {
        return "kinematic_drive_cpu.usda";
    }

    // The body whose carry the gate checks. Default is the single rigid rider;
    // articulated rows override this to a chain link fixed to the slab.
    virtual const char* riderPath() const
    {
        return kRiderPath;
    }

    // Fail-closed gate, outside the timed window: drive the slab to a
    // distinctive X and confirm both the slab arrived AND the rider on top was
    // carried. A control path that silently no-ops still costs time, so timing
    // alone cannot catch it.
    void verify()
    {
        if (!driveVerify(kVerifySlabX))
        {
            bmRecordFailure(mRowName, "verify drive did not complete");
            return;
        }

        int64_t slabCount = 0;
        float slabX = 0.0f;
        if (!readBodyXAt(kSlabPath, &slabCount, &slabX))
        {
            bmRecordFailure(mRowName, "slab pose readback failed");
            return;
        }
        if (slabCount < 1)
        {
            bmRecordFailure(mRowName, "no rigid body at %s", kSlabPath);
            return;
        }
        if (slabX < kVerifySlabX - kVerifyTolerance || slabX > kVerifySlabX + kVerifyTolerance)
        {
            bmRecordFailure(mRowName, "slab did not reach target (expected x ~= %.3f, got x = %.3f)",
                            static_cast<double>(kVerifySlabX), static_cast<double>(slabX));
            return;
        }

        const char* const rider = riderPath();
        int64_t riderCount = 0;
        float riderX = 0.0f;
        if (!readBodyXAt(rider, &riderCount, &riderX))
        {
            bmRecordFailure(mRowName, "rider pose readback failed");
            return;
        }
        if (riderCount < 1)
        {
            bmRecordFailure(mRowName, "no rigid body at %s", rider);
            return;
        }
        if (riderX < kRiderCarryMin)
        {
            bmRecordFailure(mRowName, "rider was not carried (expected x >= %.3f, got x = %.3f)",
                            static_cast<double>(kRiderCarryMin), static_cast<double>(riderX));
        }
    }

    // Read the world X of the first rigid body at `path`, via a pose tensor
    // binding (same mechanism as AuthoringCommon's readBodyPoseAt, specialized
    // to the X column).
    bool readBodyXAt(const char* path, int64_t* outCount, float* outX)
    {
        *outCount = 0;

        ovphysx_tensor_binding_handle_t binding = 0;
        ovphysx_tensor_binding_desc_t desc;
        std::memset(&desc, 0, sizeof(desc));
        desc.pattern = ovphysx_cstr(path);
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
        if (ovphysx_create_tensor_binding(mPhysX->handle(), &desc, &binding).status != OVPHYSX_API_SUCCESS)
        {
            return false;
        }

        ovphysx_tensor_spec_t spec;
        std::memset(&spec, 0, sizeof(spec));
        if (ovphysx_get_tensor_binding_spec(mPhysX->handle(), binding, &spec).status != OVPHYSX_API_SUCCESS)
        {
            authoringbm::destroyTensorBindingChecked(mPhysX->handle(), binding, mRowName, "X readback binding");
            return false;
        }
        const int64_t count = (spec.ndim >= 1) ? spec.shape[0] : 0;
        *outCount = count;
        if (count < 1)
        {
            return authoringbm::destroyTensorBindingChecked(mPhysX->handle(), binding, mRowName, "X readback binding");
        }

        float pose[7] = { 0.0f };
        int64_t shape[2] = { 1, 7 };
        DLTensor tensor;
        std::memset(&tensor, 0, sizeof(tensor));
        tensor.data = pose;
        tensor.ndim = 2;
        tensor.shape = shape;
        tensor.dtype.code = kDLFloat;
        tensor.dtype.bits = 32;
        tensor.dtype.lanes = 1;
        tensor.device.device_type = kDLCPU;

        const bool ok = ovphysx_read_tensor_binding(mPhysX->handle(), binding, &tensor).status == OVPHYSX_API_SUCCESS;
        const bool destroyed =
            authoringbm::destroyTensorBindingChecked(mPhysX->handle(), binding, mRowName, "X readback binding");
        if (!ok || !destroyed)
        {
            return false;
        }
        if (outX)
        {
            *outX = pose[0];
        }
        return true;
    }

    bool stepSync()
    {
        if (ovphysx_step_sync(mPhysX->handle(), 1.0f / 60.0f).status != OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t err = ovphysx_get_last_error();
            bmRecordFailure(mRowName, "step_sync failed: %.*s", static_cast<int>(err.length), err.ptr ? err.ptr : "");
            return false;
        }
        return true;
    }

    const char* mRowName = nullptr;
    ovphysx::PhysX* mPhysX = nullptr;
    ovphysx_sample_stage_attachment_t mStageAttachment{};
    bool mAttached = false;
    bool mLeftoverAttachmentReported = false;
    bool mSetupOk = false;
    bool mStepOk = true;
    int mIteration = 0;
};


// ---------------------------------------------------------------------------
// Path A: OVStage omni:xform drive, sealed and drained (the import path).
// ---------------------------------------------------------------------------

class KinematicDriveOvstage : public KinematicDriveBase
{
public:
    explicit KinematicDriveOvstage(const char* rowName) : KinematicDriveBase(rowName)
    {
    }

protected:
    void resetCase() override
    {
        mQuery = OVSTAGE_INVALID_QUERY_HANDLE;
        mXformAttr = {};
        mOrdinal = 0;
        mLastDrained = 0;
        std::memset(mMatrix, 0, sizeof(mMatrix));
        mWriteShape = 1;
        std::memset(&mWriteTensor, 0, sizeof(mWriteTensor));
        std::memset(&mWriteData, 0, sizeof(mWriteData));
    }

    bool setup() override
    {
        if (!authoringbm::queryPath(mStageAttachment.stage, kSlabPath, &mQuery, mRowName))
        {
            bmRecordFailure(mRowName, "query %s failed", kSlabPath);
            return false;
        }
        ovx_token_t token = 0;
        if (!authoringbm::internAttr(mStageAttachment.stage, "omni:xform", &token))
        {
            bmRecordFailure(mRowName, "intern omni:xform failed");
            return false;
        }
        mXformAttr = authoringbm::nameToken(token);
        mOrdinal = mStageAttachment.ordinal;
        mLastDrained = mStageAttachment.ordinal;
        return true;
    }

    bool driveOnce(float targetX) override
    {
        return writeSealDrainStep(targetX);
    }

    bool driveVerify(float targetX) override
    {
        return writeSealDrainStep(targetX);
    }

    void teardown() override
    {
        if (mQuery != OVSTAGE_INVALID_QUERY_HANDLE && mStageAttachment.stage)
        {
            authoringbm::releaseQueryChecked(mStageAttachment.stage, mQuery, mRowName);
        }
        mQuery = OVSTAGE_INVALID_QUERY_HANDLE;
    }

private:
    bool writeSealDrainStep(float targetX)
    {
        fillSlabXform(mMatrix, targetX);

        mWriteTensor.data = mMatrix;
        mWriteTensor.device.device_type = kDLCPU;
        mWriteTensor.ndim = 1;
        mWriteTensor.shape = &mWriteShape;
        mWriteTensor.dtype.code = kDLFloat;
        mWriteTensor.dtype.bits = 64;
        mWriteTensor.dtype.lanes = 16;

        mWriteData.tensors = &mWriteTensor;
        mWriteData.tensor_count = 1;
        mWriteData.is_array = false;

        ++mOrdinal;
        const ovstage_enqueue_result_t enqueue = ovstage_write_attribute(
            mStageAttachment.stage, mQuery, mXformAttr, mOrdinal, mWriteData, OVSTAGE_PRIM_MODE_UPSERT);
        if (enqueue.status != OVSTAGE_OK)
        {
            bmRecordFailure(mRowName, "write_attribute(omni:xform) rejected: %d", static_cast<int>(enqueue.status));
            return false;
        }

        const ovstage_op_id_t writeOp = enqueue.op_index;
        bool sealCompleted = false;
        if (!authoringbm::sealAndDrain(
                mPhysX->handle(), mStageAttachment.stage, &mLastDrained, mOrdinal, mRowName, &sealCompleted))
        {
            authoringbm::retireWriteOp(mStageAttachment.stage, writeOp, sealCompleted, mRowName);
            return false;
        }
        if (!authoringbm::retireWriteOp(mStageAttachment.stage, writeOp, /*completionProven=*/true, mRowName))
        {
            return false;
        }
        return stepSync();
    }

    ovstage_query_handle_t mQuery = OVSTAGE_INVALID_QUERY_HANDLE;
    ovx_string_or_token_t mXformAttr{};
    ovstage_ordinal_t mOrdinal = 0;
    ovstage_ordinal_t mLastDrained = 0;
    double mMatrix[16] = { 0.0 };
    int64_t mWriteShape = 1;
    DLTensor mWriteTensor{};
    ovstage_write_data_t mWriteData{};
};


// ---------------------------------------------------------------------------
// Path B: direct kinematic-target write session (the direct path).
// ---------------------------------------------------------------------------

class KinematicDriveDirect : public KinematicDriveBase
{
public:
    explicit KinematicDriveDirect(const char* rowName) : KinematicDriveBase(rowName)
    {
    }

protected:
    void resetCase() override
    {
        mQuery = 0;
    }

    bool setup() override
    {
        // Query opened ONCE and held across steps: a query is a reusable
        // selector, so the per-tick cost is the write session alone, which is
        // what the import path is being priced against.
        ovphysx_query_selection_t selection;
        std::memset(&selection, 0, sizeof(selection));
        selection.pattern = ovphysx_cstr(kSlabPath);
        if (bmCompleteTask(mPhysX->handle(),
                           ovphysx_query_selection(mPhysX->handle(), OVPHYSX_QUERY_TARGET_RIGID_BODY, &selection,
                                                   &mQuery, nullptr))
                    .status != OVPHYSX_API_SUCCESS ||
            mQuery == 0)
        {
            const ovphysx_string_t err = ovphysx_get_last_error();
            bmRecordFailure(mRowName, "query_selection(%s) failed: %.*s", kSlabPath, static_cast<int>(err.length),
                            err.ptr ? err.ptr : "");
            return false;
        }
        return true;
    }

    bool driveOnce(float targetX) override
    {
        return writeCommitStep(targetX);
    }

    bool driveVerify(float targetX) override
    {
        return writeCommitStep(targetX);
    }

    void teardown() override
    {
        if (mQuery != 0 && mPhysX)
        {
            authoringbm::releaseOvphysxResultChecked(
                bmCompleteTask(mPhysX->handle(), ovphysx_release_query(mPhysX->handle(), mQuery)), mRowName,
                "kinematic-target query");
        }
        mQuery = 0;
    }

private:
    // Fill one group's kinematicTargetPosition (vec3 f32) + kinematicTargetOrientation
    // (quat xyzw f32) from the commanded X. The two attributes arrive as separate
    // tensors in request order; identify them by lane width (3 vs 4).
    bool fillGroup(const ovstage_map_group_t& group, float targetX)
    {
        if (!group.data.tensors || group.data.tensor_count == 0)
        {
            return false;
        }
        for (uint32_t t = 0; t < group.data.tensor_count; ++t)
        {
            const DLTensor& tensor = group.data.tensors[t];
            if (!tensor.data || tensor.device.device_type != kDLCPU || tensor.dtype.code != kDLFloat ||
                tensor.dtype.bits != 32 || !tensor.shape || tensor.ndim < 1)
            {
                return false;
            }
            const int64_t rows = tensor.shape[0];
            const uint32_t lanes = tensor.dtype.lanes ? tensor.dtype.lanes : 1;
            char* base = static_cast<char*>(tensor.data) + tensor.byte_offset;
            if (lanes == 3)
            {
                // kinematicTargetPosition: drive X, hold Y/Z at the slab origin.
                for (int64_t r = 0; r < rows; ++r)
                {
                    float* p = reinterpret_cast<float*>(base) + static_cast<size_t>(r) * 3;
                    p[0] = targetX;
                    p[1] = 0.0f;
                    p[2] = 0.0f;
                }
            }
            else if (lanes == 4)
            {
                // kinematicTargetOrientation: identity quaternion xyzw.
                for (int64_t r = 0; r < rows; ++r)
                {
                    float* q = reinterpret_cast<float*>(base) + static_cast<size_t>(r) * 4;
                    q[0] = 0.0f;
                    q[1] = 0.0f;
                    q[2] = 0.0f;
                    q[3] = 1.0f;
                }
            }
            else
            {
                return false;
            }
        }
        return true;
    }

    bool writeCommitStep(float targetX)
    {
        const ovx_string_or_token_t attrs[2] = {
            { 0, { OVPHYSX_ATTR_KINEMATIC_TARGET_POSITION, sizeof(OVPHYSX_ATTR_KINEMATIC_TARGET_POSITION) - 1 } },
            { 0, { OVPHYSX_ATTR_KINEMATIC_TARGET_ORIENTATION,
                   sizeof(OVPHYSX_ATTR_KINEMATIC_TARGET_ORIENTATION) - 1 } },
        };
        ovphysx_write_handle_t write = 0;
        if (bmCompleteTask(mPhysX->handle(), ovphysx_write(mPhysX->handle(), mQuery, attrs, 2, &write, nullptr)).status !=
                OVPHYSX_API_SUCCESS ||
            write == 0)
        {
            const ovphysx_string_t err = ovphysx_get_last_error();
            bmRecordFailure(mRowName, "ovphysx_write(kinematic target) failed: %.*s", static_cast<int>(err.length),
                            err.ptr ? err.ptr : "");
            return false;
        }

        bool ok = true;
        bool drained = false;
        for (;;)
        {
            ovphysx_fetched_write_group_t fetched{};
            const ovphysx_api_status_t status =
                bmCompleteTask(mPhysX->handle(), ovphysx_fetch_write_next(mPhysX->handle(), write, &fetched)).status;
            const ovstage_map_group_t* group = fetched.group;
            if (status == OVPHYSX_API_SUCCESS && !group)
            {
                drained = true;
                break;
            }
            if (status != OVPHYSX_API_SUCCESS || !group)
            {
                bmRecordFailure(mRowName, "ovphysx_fetch_write_next failed");
                ok = false;
                break;
            }
            if (!fillGroup(*group, targetX))
            {
                bmRecordFailure(mRowName, "kinematic-target group fill failed");
                ok = false;
                break;
            }
            const ovstage_cuda_sync_t noSync{ 0, 0 };
            if (bmCompleteTask(mPhysX->handle(), ovphysx_commit_group(mPhysX->handle(), write, group, noSync)).status !=
                OVPHYSX_API_SUCCESS)
            {
                bmRecordFailure(mRowName, "ovphysx_commit_group failed");
                ok = false;
                break;
            }
        }

        if (bmCompleteTask(mPhysX->handle(), ovphysx_release_write(mPhysX->handle(), write)).status !=
            OVPHYSX_API_SUCCESS)
        {
            ok = false;
        }
        if (!ok || !drained)
        {
            return false;
        }
        return stepSync();
    }

    ovphysx_query_handle_t mQuery = 0;
};


// Hidden diagnostics, CPU-only. The two rows share the fixture and the per-step
// step; they differ only in the control plane they drive the slab through.
struct KinematicDriveOvstageCpu : KinematicDriveOvstage
{
    KinematicDriveOvstageCpu() : KinematicDriveOvstage("Authoring.kinematic_drive_ovstage_cpu")
    {
    }
};

struct KinematicDriveDirectCpu : KinematicDriveDirect
{
    KinematicDriveDirectCpu() : KinematicDriveDirect("Authoring.kinematic_drive_direct_cpu")
    {
    }
};


// Higher-count variants on the heavy fixture (32 slab/rider pairs = 64 bodies).
// Only KinematicSlab0 is driven, but Path A's ovstage drain re-imports the whole
// scene's state each tick while Path B touches only the driven body, so the gap
// widens with the scene's carried joint/binding/contact state -- the
// faithfulness point. Same driven body and verify gate as the count-1 rows.
struct KinematicDriveOvstageHeavyCpu : KinematicDriveOvstage
{
    KinematicDriveOvstageHeavyCpu() : KinematicDriveOvstage("Authoring.kinematic_drive_ovstage_heavy_cpu")
    {
    }

protected:
    const char* sceneAsset() const override
    {
        return "kinematic_drive_heavy_cpu.usda";
    }
};

struct KinematicDriveDirectHeavyCpu : KinematicDriveDirect
{
    KinematicDriveDirectHeavyCpu() : KinematicDriveDirect("Authoring.kinematic_drive_direct_heavy_cpu")
    {
    }

protected:
    const char* sceneAsset() const override
    {
        return "kinematic_drive_heavy_cpu.usda";
    }
};


// ---------------------------------------------------------------------------
// Articulated-rider variants, swept over joint count.
// ---------------------------------------------------------------------------
//
// The rigid-rider fixture carries only a resting contact; this one replaces the
// rider with an ARTICULATION (a serial revolute chain) fixed-jointed to the
// kinematic slab. Path A's ovstage drain re-imports the whole scene each tick,
// including the articulation's joint/binding state, so if joint rediscovery is
// what drives the production import cost the A-B gap should SCALE with the joint
// count. Path B still writes only the slab's kinematicTarget; the articulation
// rides along via the fixed joint.
//
// The verify gate confirms link0 (fixed to the slab) was carried. The link is a
// rigid body, so the same pose readback works.

const char* const kArticulatedRiderPath = "/World/Articulation0/link0";

// Base link is fixed to the slab, so it tracks the slab almost exactly; use the
// same carry threshold as the rigid rider.

template <int Joints>
struct KinematicDriveOvstageArticulated : KinematicDriveOvstage
{
    explicit KinematicDriveOvstageArticulated(const char* rowName) : KinematicDriveOvstage(rowName)
    {
    }

protected:
    const char* sceneAsset() const override
    {
        return kSceneAsset;
    }
    const char* riderPath() const override
    {
        return kArticulatedRiderPath;
    }
    static const char* const kSceneAsset;
};

template <int Joints>
struct KinematicDriveDirectArticulated : KinematicDriveDirect
{
    explicit KinematicDriveDirectArticulated(const char* rowName) : KinematicDriveDirect(rowName)
    {
    }

protected:
    const char* sceneAsset() const override
    {
        return kSceneAsset;
    }
    const char* riderPath() const override
    {
        return kArticulatedRiderPath;
    }
    static const char* const kSceneAsset;
};

template <> const char* const KinematicDriveOvstageArticulated<4>::kSceneAsset =
    "kinematic_drive_articulated_j4_cpu.usda";
template <> const char* const KinematicDriveOvstageArticulated<16>::kSceneAsset =
    "kinematic_drive_articulated_j16_cpu.usda";
template <> const char* const KinematicDriveOvstageArticulated<64>::kSceneAsset =
    "kinematic_drive_articulated_j64_cpu.usda";
template <> const char* const KinematicDriveDirectArticulated<4>::kSceneAsset =
    "kinematic_drive_articulated_j4_cpu.usda";
template <> const char* const KinematicDriveDirectArticulated<16>::kSceneAsset =
    "kinematic_drive_articulated_j16_cpu.usda";
template <> const char* const KinematicDriveDirectArticulated<64>::kSceneAsset =
    "kinematic_drive_articulated_j64_cpu.usda";

struct KinematicDriveOvstageArtJ4Cpu : KinematicDriveOvstageArticulated<4>
{
    KinematicDriveOvstageArtJ4Cpu()
        : KinematicDriveOvstageArticulated<4>("Authoring.kinematic_drive_articulated_ovstage_j4_cpu")
    {
    }
};
struct KinematicDriveDirectArtJ4Cpu : KinematicDriveDirectArticulated<4>
{
    KinematicDriveDirectArtJ4Cpu()
        : KinematicDriveDirectArticulated<4>("Authoring.kinematic_drive_articulated_direct_j4_cpu")
    {
    }
};
struct KinematicDriveOvstageArtJ16Cpu : KinematicDriveOvstageArticulated<16>
{
    KinematicDriveOvstageArtJ16Cpu()
        : KinematicDriveOvstageArticulated<16>("Authoring.kinematic_drive_articulated_ovstage_j16_cpu")
    {
    }
};
struct KinematicDriveDirectArtJ16Cpu : KinematicDriveDirectArticulated<16>
{
    KinematicDriveDirectArtJ16Cpu()
        : KinematicDriveDirectArticulated<16>("Authoring.kinematic_drive_articulated_direct_j16_cpu")
    {
    }
};
struct KinematicDriveOvstageArtJ64Cpu : KinematicDriveOvstageArticulated<64>
{
    KinematicDriveOvstageArtJ64Cpu()
        : KinematicDriveOvstageArticulated<64>("Authoring.kinematic_drive_articulated_ovstage_j64_cpu")
    {
    }
};
struct KinematicDriveDirectArtJ64Cpu : KinematicDriveDirectArticulated<64>
{
    KinematicDriveDirectArtJ64Cpu()
        : KinematicDriveDirectArticulated<64>("Authoring.kinematic_drive_articulated_direct_j64_cpu")
    {
    }
};


Register<KinematicDriveOvstageCpu, true> sKinematicDriveOvstageCpu("Authoring.kinematic_drive_ovstage_cpu");
Register<KinematicDriveDirectCpu, true> sKinematicDriveDirectCpu("Authoring.kinematic_drive_direct_cpu");
Register<KinematicDriveOvstageHeavyCpu, true> sKinematicDriveOvstageHeavyCpu(
    "Authoring.kinematic_drive_ovstage_heavy_cpu");
Register<KinematicDriveDirectHeavyCpu, true> sKinematicDriveDirectHeavyCpu(
    "Authoring.kinematic_drive_direct_heavy_cpu");
Register<KinematicDriveOvstageArtJ4Cpu, true> sKinematicDriveOvstageArtJ4Cpu(
    "Authoring.kinematic_drive_articulated_ovstage_j4_cpu");
Register<KinematicDriveDirectArtJ4Cpu, true> sKinematicDriveDirectArtJ4Cpu(
    "Authoring.kinematic_drive_articulated_direct_j4_cpu");
Register<KinematicDriveOvstageArtJ16Cpu, true> sKinematicDriveOvstageArtJ16Cpu(
    "Authoring.kinematic_drive_articulated_ovstage_j16_cpu");
Register<KinematicDriveDirectArtJ16Cpu, true> sKinematicDriveDirectArtJ16Cpu(
    "Authoring.kinematic_drive_articulated_direct_j16_cpu");
Register<KinematicDriveOvstageArtJ64Cpu, true> sKinematicDriveOvstageArtJ64Cpu(
    "Authoring.kinematic_drive_articulated_ovstage_j64_cpu");
Register<KinematicDriveDirectArtJ64Cpu, true> sKinematicDriveDirectArtJ64Cpu(
    "Authoring.kinematic_drive_articulated_direct_j64_cpu");

} // namespace
