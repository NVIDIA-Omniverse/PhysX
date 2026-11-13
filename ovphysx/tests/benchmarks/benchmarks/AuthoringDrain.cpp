// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// DEPRECATED (tensor-binding-deprecation): the binding (tensor) lane retires with the binding. The ovstage lane stays.

/**
 * @implements REQ-CAPI-BENCHMARK-001
 * @covers AC-1 AC-4 AC-5
 */

// Authoring.* in-process control-plane component diagnostics.
//
// The same rigid-body velocity change is applied two ways and stepped, next to
// a step-only baseline:
//
//   drain_ovstage_cpu     ovstage_write_attribute(physics:velocity, UPSERT)
//                         -> advance_write_floor -> wait(seal)
//                         -> ovphysx_update_from_ovstage -> step
//
//   drain_tensor_cpu      ovphysx_write_tensor_binding(RIGID_BODY_VELOCITY)
//                         -> step        (no drain)
//
//   drain_step_only_cpu   step           (no control write at all)
//
//   drain_ovstage_gpu     the drain_ovstage_cpu workflow on a DirectGPU scene: physics:velocity +
//                         physics:angularVelocity authored as device (kDLCUDA) columns and drained
//                         zero-copy on the device path.
//
//   drain_ovstage_scaled_{4096,16384,65536}_gpu
//                         the same device drain over N bodies, no sim step, timing the full device
//                         write + drain path as it scales with body count.
//
// These are diagnostic rows. They explain where the KPI totals in the
// population_add_* and teleport_* rows go and do not replace them. The
// step-only row is the floor both control paths are measured against. Without
// it, a change in raw simulate cost is indistinguishable from a change in
// control-plane cost.
//
// The ovstage path waits once on the seal, which transitively covers the write
// at that ordinal, rather than waiting per operation. authoringbm::sealAndDrain
// preserves that.
//
// Device selection uses the harness model (scene-authored
// physxScene:enableGPUDynamics plus the forceGpu() gate). ovphysx_set_cpu_mode
// is process-wide and would poison every other row in this binary.

#include "framework/UsdPCH.h"

#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"
#include "../framework/BmUtils.h"
#include "../BenchmarkFailure.h"
#include "AuthoringCommon.h"

#include <ovphysx/experimental/ovphysx.hpp>

#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
#include <cuda_runtime_api.h>
#endif

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>


void initAuthoringDrain()
{
}


namespace
{

const char* const kAuthoringDrainBodyPath = "/World/Cube1";

// Distinctive velocity used by the post-timing gate, on the gravity-free X
// axis so the check is not entangled with fall speed.
const float kAuthoringVerifyVelocityX = 12.0f;
const float kAuthoringVerifyVelocityTolerance = 0.5f;


// Per-iteration X velocity, so all three rows stay on an equivalent physical
// trajectory and the step-only floor really is the floor. Cycles so successive
// writes differ.
float authoringIterationVelocity(int iteration)
{
    return 0.5f + 0.05f * static_cast<float>((iteration % 11) + 1);
}


// Reads linear + angular velocity (6 floats: lin.xyz then ang.xyz) for an arbitrary body path. The scaled
// GPU drain's verify uses it to confirm the device columns reached PhysX on more than one body, angular
// included (so the on-device deg->rad scale is covered).
bool authoringReadVelocity6(ovphysx_instance_handle_t handle, const char* path, const char* row, float out6[6])
{
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc;
    std::memset(&desc, 0, sizeof(desc));
    desc.pattern = ovphysx_cstr(path);
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32;
    if (ovphysx_create_tensor_binding(handle, &desc, &binding).status != OVPHYSX_API_SUCCESS)
    {
        bmRecordFailure(row, "create_tensor_binding(velocity read %s) failed", path);
        return false;
    }

    int64_t shape[2] = { 1, 6 };
    DLTensor tensor;
    std::memset(&tensor, 0, sizeof(tensor));
    tensor.data = out6;
    tensor.ndim = 2;
    tensor.shape = shape;
    tensor.dtype.code = kDLFloat;
    tensor.dtype.bits = 32;
    tensor.dtype.lanes = 1;
    tensor.device.device_type = kDLCPU;

    const bool ok = ovphysx_read_tensor_binding(handle, binding, &tensor).status == OVPHYSX_API_SUCCESS;
    const bool destroyed = authoringbm::destroyTensorBindingChecked(handle, binding, row, "velocity read binding");
    if (!ok)
    {
        bmRecordFailure(row, "read_tensor_binding(velocity %s) failed", path);
        return false;
    }
    return destroyed;
}


// Reads back linear velocity X for the default body under test, the gravity-free axis the control paths write.
bool authoringReadVelocityX(ovphysx_instance_handle_t handle, const char* row, float* outX)
{
    float v6[6] = { 0.0f };
    if (!authoringReadVelocity6(handle, kAuthoringDrainBodyPath, row, v6))
    {
        return false;
    }
    *outX = v6[0];
    return true;
}


// ---------------------------------------------------------------------------
// Shared base for the diagnostic rows.
// ---------------------------------------------------------------------------

class AuthoringDrainBase : public BmBenchmark
{
public:
    explicit AuthoringDrainBase(const char* rowName) : mRowName(rowName)
    {
    }

    // Device selection: the CPU rows run in the default pass and skip under --forceGpu; a GPU row runs
    // only in the GPU pass (--forceGpu --directGpu), mirroring the OutputRead _gpu lanes.
    bool isValid() const override
    {
        if (isGpu())
        {
            return BmGlobals::getInstance().forceGpu() && BmGlobals::getInstance().directGpu() &&
                   BmGlobals::getInstance().getPhysX() != nullptr;
        }
        if (BmGlobals::getInstance().forceGpu())
        {
            return false;
        }
        return BmGlobals::getInstance().getPhysX() != nullptr;
    }

    // Overridden by the GPU row. Drives scene selection, the GPU-fallback evidence arming, and the
    // device-evidence check in startRun, plus the isValid gate above.
    virtual bool isGpu() const
    {
        return false;
    }

    // The scene asset the row attaches. The GPU row swaps in the enableGPUDynamics variant.
    virtual const char* sceneFileName() const
    {
        return "simple_physics_scene_cpu.usda";
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

        // Capture must be live across the attach. The CPU-fallback diagnostic
        // is emitted while the scene is being realized, not afterwards.
        mAttached = false;
        authoringbm::GpuFallbackScope gpuScope(/*arm=*/isGpu(), mRowName);

        const std::string path = BmGlobals::getInstance().getDataFolder() + "/" + sceneFileName();
        if (!authoringbm::populateAndAttachChecked(mPhysX, authoringbm::UsdSource::eFile, path,
                                                   "ovphysx-authoring-drain", mStageAttachment, mRowName, &mAttached))
        {
            return;
        }
        if (!authoringbm::checkDeviceEvidenceAndWarmup(mPhysX->handle(), /*gpu=*/isGpu(), gpuScope, mRowName))
        {
            return;
        }
        mSetupOk = setup();
        mStepOk = mSetupOk;
        // Ten untimed same-stage warmup iterations.
        if (mSetupOk)
        {
            for (int w = 0; w < 10; ++w)
            {
                if (!applyOnce(authoringIterationVelocity(w)))
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
            bmRecordFailure(mRowName, "control sequence did not complete");
        }
        teardown();
        // A Stage ovphysx never took needs the plain destroy, not the reset and
        // detach path. Attempting to detach a never-attached Stage reports a
        // failure that misattributes the original fault.
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
        if (!applyOnce(authoringIterationVelocity(mIteration)))
        {
            mStepOk = false;
        }
        ++mIteration;
    }

    virtual void resetCase()
    {
    }
    virtual bool setup() = 0;
    virtual bool applyOnce(float velocityX) = 0;
    virtual void teardown()
    {
    }

    // Fail-closed gate, outside the timed window. The control rows prove the
    // written velocity reached PhysX. The step-only row has no control write,
    // so it proves only that the body is present and simulating.
    virtual void verify()
    {
        int64_t count = 0;
        float y = 0.0f;
        if (!authoringbm::readBodyPoseAt(mPhysX->handle(), kAuthoringDrainBodyPath, &count, &y, mRowName))
        {
            bmRecordFailure(mRowName, "pose readback failed");
            return;
        }
        if (count < 1)
        {
            bmRecordFailure(mRowName, "no rigid body at %s", kAuthoringDrainBodyPath);
        }
    }

    // Shared by both control rows. Applies a distinctive velocity through the
    // row's own path, then confirms PhysX has it. A control path that silently
    // no-ops still costs time, so timing alone cannot catch it.
    void verifyControlPathApplied()
    {
        if (!applyOnce(kAuthoringVerifyVelocityX))
        {
            bmRecordFailure(mRowName, "verify write did not complete");
            return;
        }

        float x = 0.0f;
        if (!authoringReadVelocityX(mPhysX->handle(), mRowName, &x))
        {
            bmRecordFailure(mRowName, "velocity readback failed");
            return;
        }
        // X is the gravity-free axis, so the written value survives a step
        // essentially intact. The window is far tighter than a no-op.
        if (x < kAuthoringVerifyVelocityX - kAuthoringVerifyVelocityTolerance ||
            x > kAuthoringVerifyVelocityX + kAuthoringVerifyVelocityTolerance)
        {
            bmRecordFailure(mRowName, "velocity not applied (expected ~%.2f, got %.2f)",
                            static_cast<double>(kAuthoringVerifyVelocityX), static_cast<double>(x));
        }
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
// Path A: ovstage control plane (write physics:velocity, seal, drain, step).
// ---------------------------------------------------------------------------

class AuthoringDrainOvstage : public AuthoringDrainBase
{
public:
    AuthoringDrainOvstage() : AuthoringDrainBase("Authoring.drain_ovstage_cpu")
    {
    }

protected:
    void resetCase() override
    {
        mQuery = OVSTAGE_INVALID_QUERY_HANDLE;
        mVelocityAttr = {};
        mOrdinal = 0;
        mLastDrained = 0;
        std::memset(mValues, 0, sizeof(mValues));
        mWriteShape = 1;
        std::memset(&mWriteTensor, 0, sizeof(mWriteTensor));
        std::memset(&mWriteData, 0, sizeof(mWriteData));
    }

    bool setup() override
    {
        if (!authoringbm::queryPath(mStageAttachment.stage, kAuthoringDrainBodyPath, &mQuery, mRowName))
        {
            bmRecordFailure(mRowName, "query %s failed", kAuthoringDrainBodyPath);
            return false;
        }
        ovx_token_t token{0};
        if (!authoringbm::internAttr(mStageAttachment.stage, "physics:velocity", &token))
        {
            bmRecordFailure(mRowName, "intern physics:velocity failed");
            return false;
        }
        mVelocityAttr = authoringbm::nameToken(token);

        mOrdinal = mStageAttachment.ordinal;
        mLastDrained = mStageAttachment.ordinal;
        return true;
    }

    bool applyOnce(float velocityX) override
    {
        mValues[0] = velocityX;
        mValues[1] = 0.0f;
        mValues[2] = 0.0f;
        mWriteTensor.data = mValues;
        mWriteTensor.device.device_type = kDLCPU;
        mWriteTensor.ndim = 1;
        mWriteTensor.shape = &mWriteShape;
        mWriteTensor.dtype.code = kDLFloat;
        mWriteTensor.dtype.bits = 32;
        mWriteTensor.dtype.lanes = 3;

        mWriteData.tensors = &mWriteTensor;
        mWriteData.tensor_count = 1;
        mWriteData.is_array = false;
        mWriteData.semantic = OVSTAGE_SEMANTIC_NONE;

        ++mOrdinal;
        // Only the seal is waited on. It transitively covers this write at this
        // ordinal.
        const ovstage_enqueue_result_t enqueue = ovstage_write_attribute(
            mStageAttachment.stage, mQuery, mVelocityAttr, mOrdinal, mWriteData, OVSTAGE_PRIM_MODE_UPSERT);
        if (enqueue.status != OVSTAGE_OK)
        {
            bmRecordFailure(mRowName, "write_attribute(physics:velocity) rejected: %d", static_cast<int>(enqueue.status));
            return false;
        }

        // Retain the write op so completed tracking is released. A bounded
        // timeout enters the final completion wait while this object-owned
        // payload remains live. The outer process timeout is the hard stop.
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

    void verify() override
    {
        AuthoringDrainBase::verify();
        verifyControlPathApplied();
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
    ovstage_query_handle_t mQuery = OVSTAGE_INVALID_QUERY_HANDLE;
    ovx_token_or_string_t mVelocityAttr{};
    ovstage_ordinal_t mOrdinal = 0;
    ovstage_ordinal_t mLastDrained = 0;
    float mValues[3] = { 0.0f };
    int64_t mWriteShape = 1;
    DLTensor mWriteTensor{};
    ovstage_write_data_t mWriteData{};
};


// ---------------------------------------------------------------------------
// Path A on GPU: the ovstage control plane on a DirectGPU scene, authoring the device-native velocity
// state each step as DEVICE-RESIDENT (kDLCUDA) columns -- physics:velocity and physics:angularVelocity
// (deg) -- written at one ordinal and drained together in one updateFromOvStage. This is the workflow the
// device value drain targets: a GPU policy produces the next state already on the GPU and
// pushes it every step. The lane queries EXACTLY the three dynamic bodies (not the whole stage): the
// device read serves a column on device only when every change key resolves to a body and the drain
// rewrites every row, so authoring velocity on non-body prims (PhysicsScene, Ground, colliders) would
// leave keys unresolved, flip `whole` false, and route the column through the host-staging gather. It
// seals SCOPE_INCLUDE over the two written floors (not SCOPE_ALL), so each drain is the per-frame steady
// state -- a value-only drain that does not advance the structural floors and so skips the structural
// reconcile. Only the seal is waited
// on; it transitively covers both writes at that ordinal. checkDeviceEvidenceAndWarmup (gpu=true, via
// isGpu()) refuses to publish the row if the scene silently fell back to CPU; the inherited-style
// verify() confirms the device-written velocity reached PhysX.
// ---------------------------------------------------------------------------

class AuthoringDrainOvstageGpu : public AuthoringDrainBase
{
public:
    AuthoringDrainOvstageGpu() : AuthoringDrainBase("Authoring.drain_ovstage_gpu")
    {
    }

    bool isGpu() const override
    {
        return true;
    }

    const char* sceneFileName() const override
    {
        return "simple_physics_scene_gpu.usda";
    }

protected:
    void resetCase() override
    {
        mQuery = OVSTAGE_INVALID_QUERY_HANDLE;
        mVelAttr = {};
        mAngAttr = {};
        mVelTok = ovx_token_t{0};
        mAngTok = ovx_token_t{0};
        mOrdinal = 0;
        mLastDrained = 0;
        freeDeviceBuffers();
    }

    bool setup() override
    {
        // Query EXACTLY the three dynamic bodies, not the whole stage. The device read serves a column on
        // device only when every change key resolves to a body and the drain rewrites every row of the
        // column (`whole`, OvxPhysicsWrite.cpp). An empty-filter (whole-stage) query would also author
        // velocity on PhysicsScene, Ground and the collider Geoms, whose keys never resolve to bodies --
        // `whole` goes false and the column drains through the host-staging gather, so this "gpu" row would
        // silently measure the host path. query_from_path_list is synchronous (no op to wait on) and needs
        // no result fetch: the write shape is just the body count.
        static const char* const kBodyPaths[] = { "/World/Cube1", "/World/Cube2", "/World/Sphere1" };
        constexpr size_t kBodyCount = sizeof(kBodyPaths) / sizeof(kBodyPaths[0]);
        path_dictionary_instance_t* dict = ovstage_get_path_dictionary(mStageAttachment.stage);
        if (!dict)
        {
            bmRecordFailure(mRowName, "no path dictionary for the body query");
            return false;
        }
        ovx_string_t paths[kBodyCount];
        for (size_t i = 0; i < kBodyCount; ++i)
        {
            paths[i] = authoringbm::stringView(kBodyPaths[i]);
        }
        ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
        const ovx_api_result_t built = path_dictionary_create_path_list_from_strings(dict, paths, kBodyCount, &list);
        if (!authoringbm::queryPathListIsUsable(built.status, list))
        {
            bmRecordFailure(mRowName, "create_path_list_from_strings failed: %d", static_cast<int>(built.status));
            return false;
        }
        const ovstage_api_status_t qstatus = ovstage_query_from_path_list(mStageAttachment.stage, list, &mQuery);
        const bool released = authoringbm::releasePathListChecked(dict, list, mRowName, "body query path-list");
        if (qstatus != OVSTAGE_OK || mQuery == OVSTAGE_INVALID_QUERY_HANDLE || !released)
        {
            bmRecordFailure(mRowName, "query_from_path_list failed: %d", static_cast<int>(qstatus));
            return false;
        }
        mWriteShape = static_cast<int64_t>(kBodyCount);

        if (!internColumn("physics:velocity", &mVelAttr, &mVelTok) ||
            !internColumn("physics:angularVelocity", &mAngAttr, &mAngTok))
        {
            return false;
        }
        mOrdinal = mStageAttachment.ordinal;
        mLastDrained = mStageAttachment.ordinal;
#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
        const size_t n = static_cast<size_t>(mWriteShape);
        mHostVel.assign(n * 3, 0.0f);
        mHostAng.assign(n * 3, 0.0f);
        if (cudaMalloc(&mDevVel, sizeof(float) * 3 * n) != cudaSuccess ||
            cudaMalloc(&mDevAng, sizeof(float) * 3 * n) != cudaSuccess)
        {
            bmRecordFailure(mRowName, "cudaMalloc(device columns) failed");
            return false;
        }
        // The buffers live on the process-current device; the DLTensors must advertise that same
        // ordinal, not a hard-coded 0, so the read lands on the device that owns the memory.
        if (cudaGetDevice(&mCudaDevice) != cudaSuccess)
        {
            bmRecordFailure(mRowName, "cudaGetDevice failed");
            return false;
        }
        return true;
#else
        bmRecordFailure(mRowName, "ovphysx_benchmarks built without CUDA; the GPU device-drain row cannot run");
        return false;
#endif
    }

    // A GPU policy authors the device-native velocity state this step: velocity + angular velocity, both
    // device-resident, written at one ordinal, then drained together. `value` varies each step so
    // successive drains carry a real change.
    bool applyOnce(float value) override
    {
#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
        ++mOrdinal;

        // Author the whole-body superset: the same moving value on every prim, so the drain rewrites every
        // row and the device read returns a dense device column (the whole point of the lane).
        const size_t n = static_cast<size_t>(mWriteShape);
        // mHostVel / mHostAng are sized once in setup(); refill the x lane (y/z stay 0) with no per-step alloc.
        for (size_t i = 0; i < n; ++i)
        {
            mHostVel[i * 3 + 0] = value;
            mHostAng[i * 3 + 0] = value;
        }

        ovstage_op_id_t ops[2] = { OVSTAGE_INVALID_OP_ID, OVSTAGE_INVALID_OP_ID };
        if (!writeColumn(mTensors[0], mDatas[0], mVelAttr, mDevVel, mHostVel.data(), mHostVel.size() * sizeof(float), 32, 3, &ops[0]) ||
            !writeColumn(mTensors[1], mDatas[1], mAngAttr, mDevAng, mHostAng.data(), mHostAng.size() * sizeof(float), 32, 3, &ops[1]))
        {
            // A later writeColumn can fail after an earlier one already queued an op. Retire every
            // enqueued op (invalid ids no-op) so no write-completion tracking survives the failure;
            // the seal never ran, so completion is unproven and each queued op is waited on first.
            for (const ovstage_op_id_t op : ops)
            {
                authoringbm::retireWriteOp(mStageAttachment.stage, op, /*completionProven=*/false, mRowName);
            }
            return false;
        }

        // Seal only the two written floors (SCOPE_INCLUDE) so each drain is a value-only drain; SCOPE_ALL
        // would advance the structural floors and force the structural reconcile every drain. Only the seal
        // is waited on; it transitively covers both writes at this ordinal.
        const ovx_token_t sealFloors[2] = { mVelTok, mAngTok };
        bool sealCompleted = false;
        if (!authoringbm::sealAndDrainScoped(mPhysX->handle(), mStageAttachment.stage, &mLastDrained, mOrdinal,
                                             sealFloors, 2, mRowName, &sealCompleted))
        {
            for (const ovstage_op_id_t op : ops)
            {
                authoringbm::retireWriteOp(mStageAttachment.stage, op, sealCompleted, mRowName);
            }
            return false;
        }
        for (const ovstage_op_id_t op : ops)
        {
            if (!authoringbm::retireWriteOp(mStageAttachment.stage, op, /*completionProven=*/true, mRowName))
            {
                return false;
            }
        }
        return stepSync();
#else
        (void)value;
        return false;
#endif
    }

    // The velocity column is one of the two written, so the base control-path gate still proves the
    // device-resident write reached PhysX.
    void verify() override
    {
        AuthoringDrainBase::verify();
        verifyControlPathApplied();
    }

    void teardown() override
    {
        if (mQuery != OVSTAGE_INVALID_QUERY_HANDLE && mStageAttachment.stage)
        {
            authoringbm::releaseQueryChecked(mStageAttachment.stage, mQuery, mRowName);
        }
        mQuery = OVSTAGE_INVALID_QUERY_HANDLE;
        freeDeviceBuffers();
    }

private:
    bool internColumn(const char* name, ovx_token_or_string_t* outAttr, ovx_token_t* outTok = nullptr)
    {
        ovx_token_t token{0};
        if (!authoringbm::internAttr(mStageAttachment.stage, name, &token))
        {
            bmRecordFailure(mRowName, "intern %s failed", name);
            return false;
        }
        *outAttr = authoringbm::nameToken(token);
        if (outTok)
        {
            *outTok = token; // kept for the SCOPE_INCLUDE floor seal (value-only drain)
        }
        return true;
    }

#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
    // Copy one column's values to its device buffer and enqueue a device-resident write at mOrdinal. The
    // tensor descriptor and write_data are persistent members so they outlive the enqueue until the seal
    // executes the copy-in.
    bool writeColumn(DLTensor& tensor, ovstage_write_data_t& data, const ovx_token_or_string_t& attr, void* devBuf,
                     const void* host, size_t bytes, int bits, int lanes, ovstage_op_id_t* outOp)
    {
        if (cudaMemcpy(devBuf, host, bytes, cudaMemcpyHostToDevice) != cudaSuccess)
        {
            bmRecordFailure(mRowName, "cudaMemcpy(device column H2D) failed");
            return false;
        }
        std::memset(&tensor, 0, sizeof(tensor));
        tensor.data = devBuf;
        tensor.device.device_type = kDLCUDA;
        tensor.device.device_id = mCudaDevice;
        tensor.ndim = 1;
        tensor.shape = &mWriteShape;
        tensor.dtype.code = kDLFloat;
        tensor.dtype.bits = static_cast<uint8_t>(bits);
        tensor.dtype.lanes = static_cast<uint16_t>(lanes);

        std::memset(&data, 0, sizeof(data));
        data.tensors = &tensor;
        data.tensor_count = 1;
        data.is_array = false;
        data.semantic = OVSTAGE_SEMANTIC_NONE;

        const ovstage_enqueue_result_t e =
            ovstage_write_attribute(mStageAttachment.stage, mQuery, attr, mOrdinal, data, OVSTAGE_PRIM_MODE_UPSERT);
        if (e.status != OVSTAGE_OK)
        {
            bmRecordFailure(mRowName, "write_attribute rejected: %d", static_cast<int>(e.status));
            return false;
        }
        *outOp = e.op_index;
        return true;
    }
#endif

    void freeDeviceBuffers()
    {
#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
        for (void** buf : { &mDevVel, &mDevAng })
        {
            if (*buf)
            {
                cudaFree(*buf);
                *buf = nullptr;
            }
        }
#endif
    }

    ovstage_query_handle_t mQuery = OVSTAGE_INVALID_QUERY_HANDLE;
    ovx_token_or_string_t mVelAttr{};
    ovx_token_or_string_t mAngAttr{};
    ovx_token_t mVelTok{0}; // raw tokens kept for the SCOPE_INCLUDE floor seal (value-only drain)
    ovx_token_t mAngTok{0};
    ovstage_ordinal_t mOrdinal = 0;
    ovstage_ordinal_t mLastDrained = 0;
    int64_t mWriteShape = 1;
    std::vector<float> mHostVel; // sized once in setup(), refilled per step (no per-step allocation)
    std::vector<float> mHostAng;
    DLTensor mTensors[2]{};
    ovstage_write_data_t mDatas[2]{};
#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
    void* mDevVel = nullptr;
    void* mDevAng = nullptr;
    int mCudaDevice = 0;
#endif
};


// ---------------------------------------------------------------------------
// Path B: tensor escape hatch (write velocity binding, step, no drain).
// ---------------------------------------------------------------------------

class AuthoringDrainTensor : public AuthoringDrainBase
{
public:
    AuthoringDrainTensor() : AuthoringDrainBase("Authoring.drain_tensor_cpu")
    {
    }

protected:
    void resetCase() override
    {
        mBinding = 0;
        std::memset(mVelocity, 0, sizeof(mVelocity));
        mShape[0] = 1;
        mShape[1] = 6;
        std::memset(&mTensor, 0, sizeof(mTensor));
    }

    bool setup() override
    {
        ovphysx_tensor_binding_desc_t desc;
        std::memset(&desc, 0, sizeof(desc));
        desc.pattern = ovphysx_cstr(kAuthoringDrainBodyPath);
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32;
        if (ovphysx_create_tensor_binding(mPhysX->handle(), &desc, &mBinding).status != OVPHYSX_API_SUCCESS)
        {
            bmRecordFailure(mRowName, "create_tensor_binding(velocity) failed");
            return false;
        }

        ovphysx_tensor_spec_t spec;
        std::memset(&spec, 0, sizeof(spec));
        if (ovphysx_get_tensor_binding_spec(mPhysX->handle(), mBinding, &spec).status != OVPHYSX_API_SUCCESS)
        {
            bmRecordFailure(mRowName, "get_tensor_binding_spec(velocity) failed");
            return false;
        }
        if (spec.ndim < 2 || spec.shape[0] < 1 || spec.shape[1] < 6)
        {
            bmRecordFailure(mRowName, "unexpected velocity tensor shape ndim=%d", static_cast<int>(spec.ndim));
            return false;
        }

        mTensor.data = mVelocity;
        mTensor.ndim = 2;
        mTensor.shape = mShape;
        mTensor.dtype.code = kDLFloat;
        mTensor.dtype.bits = 32;
        mTensor.dtype.lanes = 1;
        mTensor.device.device_type = kDLCPU;
        return true;
    }

    bool applyOnce(float velocityX) override
    {
        std::memset(mVelocity, 0, sizeof(mVelocity));
        mVelocity[0] = velocityX;
        if (ovphysx_write_tensor_binding(mPhysX->handle(), mBinding, &mTensor, nullptr).status != OVPHYSX_API_SUCCESS)
        {
            bmRecordFailure(mRowName, "write_tensor_binding(velocity) failed");
            return false;
        }
        return stepSync();
    }

    void verify() override
    {
        AuthoringDrainBase::verify();
        verifyControlPathApplied();
    }

    void teardown() override
    {
        authoringbm::destroyTensorBindingChecked(
            mPhysX ? mPhysX->handle() : 0, mBinding, mRowName, "velocity write binding");
    }

private:
    ovphysx_tensor_binding_handle_t mBinding = 0;
    float mVelocity[6] = { 0.0f };
    int64_t mShape[2] = { 1, 6 };
    DLTensor mTensor{};
};


// ---------------------------------------------------------------------------
// Path C: step only. The floor both control paths are measured against.
// ---------------------------------------------------------------------------

class AuthoringDrainStepOnly : public AuthoringDrainBase
{
public:
    AuthoringDrainStepOnly() : AuthoringDrainBase("Authoring.drain_step_only_cpu")
    {
    }

protected:
    bool setup() override
    {
        return true;
    }

    bool applyOnce(float /*velocityX*/) override
    {
        return stepSync();
    }
};


// Generate a DirectGPU scene of `bodyCount` dynamic rigid bodies on a non-overlapping grid (gravity off,
// so no contacts and no motion), each carrying physics:velocity + physics:angularVelocity. Body paths are
// /World/Body_NNNNNN. Cached per count (the text is built at most once per size).
const std::string& makeGpuDrainScene(uint32_t bodyCount)
{
    static std::map<uint32_t, std::string> cache;
    const std::map<uint32_t, std::string>::iterator hit = cache.find(bodyCount);
    if (hit != cache.end())
    {
        return hit->second;
    }
    const uint32_t side = static_cast<uint32_t>(std::ceil(std::sqrt(static_cast<double>(bodyCount))));
    std::ostringstream s;
    s << "#usda 1.0\n(\n    defaultPrim = \"World\"\n    metersPerUnit = 1\n    upAxis = \"Y\"\n)\n\n"
      << "def Xform \"World\"\n{\n"
      << "    def PhysicsScene \"PhysicsScene\" (\n        prepend apiSchemas = [\"PhysxSceneAPI\"]\n    )\n    {\n"
      << "        vector3f physics:gravityDirection = (0, -1, 0)\n"
      << "        float physics:gravityMagnitude = 0\n"
      << "        bool physxScene:enableGPUDynamics = true\n"
      << "        uniform token physxScene:broadphaseType = \"GPU\"\n"
      << "        uint physxScene:gpuFoundLostAggregatePairsCapacity = 1048576\n"
      << "        uint physxScene:gpuTotalAggregatePairsCapacity = 1048576\n"
      << "    }\n\n";
    for (uint32_t i = 0; i < bodyCount; ++i)
    {
        const uint32_t r = i / side;
        const uint32_t c = i % side;
        s << "    def Cube \"Body_" << std::setw(6) << std::setfill('0') << i << "\" (\n"
          << "        prepend apiSchemas = [\"PhysicsRigidBodyAPI\", \"PhysicsCollisionAPI\", \"PhysicsMassAPI\"]\n    )\n    {\n"
          << "        double size = 0.25\n"
          << "        float physics:mass = 1\n"
          << "        vector3f physics:velocity = (0, 0, 0)\n"
          << "        vector3f physics:angularVelocity = (0, 0, 0)\n"
          << "        double3 xformOp:translate = (" << static_cast<double>(c) * 2.0 << ", 0, "
          << static_cast<double>(r) * 2.0 << ")\n"
          << "        uniform token[] xformOpOrder = [\"xformOp:translate\"]\n    }\n\n";
    }
    s << "}\n";
    return cache.emplace(bodyCount, s.str()).first->second;
}

// ---------------------------------------------------------------------------
// SCALED device drain: the drain_ovstage_gpu workload at RL-relevant body counts. Authors
// physics:velocity + physics:angularVelocity as device (kDLCUDA) columns on ALL N bodies each iteration,
// seals SCOPE_INCLUDE, and drains via update_from_ovstage. No sim step -- the timed window is the full
// device write path (host fill + H2D upload, then the seal and the zero-copy borrow + on-device deg->rad
// scale drain), measured as it scales with body count N.
// ---------------------------------------------------------------------------
class AuthoringDrainOvstageGpuScaled : public AuthoringDrainBase
{
public:
    AuthoringDrainOvstageGpuScaled(const char* rowName, uint32_t bodyCount)
        : AuthoringDrainBase(rowName), mBodyCount(bodyCount)
    {
    }

    bool isGpu() const override
    {
        return true;
    }

    uint32_t getNbSteps() const override
    {
        return mBodyCount >= 65536u ? 20u : 50u;
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
        if (!authoringbm::prepareStageAttachmentForRun(mStageAttachment, mRowName, &mLeftoverAttachmentReported))
        {
            return;
        }
        resetCase();
        if (!mPhysX)
        {
            return;
        }
        mAttached = false;
        authoringbm::GpuFallbackScope gpuScope(/*arm=*/true, mRowName);
        const std::string& usda = makeGpuDrainScene(mBodyCount);
        if (!authoringbm::populateAndAttachChecked(mPhysX, authoringbm::UsdSource::eString, usda,
                                                   "ovphysx-authoring-drain-scaled", mStageAttachment, mRowName,
                                                   &mAttached))
        {
            return;
        }
        if (!authoringbm::checkDeviceEvidenceAndWarmup(mPhysX->handle(), /*gpu=*/true, gpuScope, mRowName))
        {
            return;
        }
        mSetupOk = setup();
        mStepOk = mSetupOk;
        if (mSetupOk)
        {
            for (int w = 0; w < 10; ++w)
            {
                if (!applyOnce(authoringIterationVelocity(w)))
                {
                    mStepOk = false;
                    mSetupOk = false;
                    break;
                }
            }
        }
    }

protected:
    void resetCase() override
    {
        mQuery = OVSTAGE_INVALID_QUERY_HANDLE;
        mVelAttr = {};
        mAngAttr = {};
        mVelTok = ovx_token_t{0};
        mAngTok = ovx_token_t{0};
        mOrdinal = 0;
        mLastDrained = 0;
        freeDeviceBuffers();
    }

    bool setup() override
    {
        // Query ALL N bodies (dense) so every change key resolves -> whole-superset device serve.
        std::vector<std::string> paths;
        paths.reserve(mBodyCount);
        char buf[32];
        for (uint32_t i = 0; i < mBodyCount; ++i)
        {
            std::snprintf(buf, sizeof(buf), "/World/Body_%06u", i);
            paths.emplace_back(buf);
        }
        std::vector<ovx_string_t> views;
        views.reserve(mBodyCount);
        for (const std::string& p : paths)
        {
            views.push_back(authoringbm::stringView(p));
        }
        path_dictionary_instance_t* dict = ovstage_get_path_dictionary(mStageAttachment.stage);
        if (!dict)
        {
            bmRecordFailure(mRowName, "no path dictionary for the N-body query");
            return false;
        }
        ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
        const ovx_api_result_t built =
            path_dictionary_create_path_list_from_strings(dict, views.data(), views.size(), &list);
        if (!authoringbm::queryPathListIsUsable(built.status, list))
        {
            bmRecordFailure(mRowName, "path-list creation failed for N=%u (status=%d)", mBodyCount,
                            static_cast<int>(built.status));
            return false;
        }
        const ovstage_api_status_t qs = ovstage_query_from_path_list(mStageAttachment.stage, list, &mQuery);
        const bool released = authoringbm::releasePathListChecked(dict, list, mRowName, "scaled drain path-list");
        if (qs != OVSTAGE_OK || mQuery == OVSTAGE_INVALID_QUERY_HANDLE || !released)
        {
            bmRecordFailure(mRowName, "query_from_path_list failed: %d", static_cast<int>(qs));
            return false;
        }
        ovx_token_t vt{0};
        ovx_token_t at{0};
        if (!authoringbm::internAttr(mStageAttachment.stage, "physics:velocity", &vt) ||
            !authoringbm::internAttr(mStageAttachment.stage, "physics:angularVelocity", &at))
        {
            bmRecordFailure(mRowName, "intern velocity/angularVelocity failed");
            return false;
        }
        mVelAttr = authoringbm::nameToken(vt);
        mAngAttr = authoringbm::nameToken(at);
        mVelTok = vt;
        mAngTok = at;
        mOrdinal = mStageAttachment.ordinal;
        mLastDrained = mStageAttachment.ordinal;
        const size_t n = static_cast<size_t>(mBodyCount);
        mHostVel.assign(n * 3, 0.0f);
        mHostAng.assign(n * 3, 0.0f);
        mWriteShape = static_cast<int64_t>(mBodyCount);
#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
        if (cudaMalloc(&mDevVel, sizeof(float) * 3 * n) != cudaSuccess ||
            cudaMalloc(&mDevAng, sizeof(float) * 3 * n) != cudaSuccess)
        {
            bmRecordFailure(mRowName, "cudaMalloc(%zu device rows) failed", n);
            return false;
        }
        if (cudaGetDevice(&mCudaDevice) != cudaSuccess)
        {
            bmRecordFailure(mRowName, "cudaGetDevice failed");
            return false;
        }
        return true;
#else
        bmRecordFailure(mRowName, "ovphysx_benchmarks built without CUDA; the scaled GPU device-drain row cannot run");
        return false;
#endif
    }

    // Author device velocity + angularVelocity on ALL N bodies, seal SCOPE_INCLUDE, drain. NO sim step --
    // the timed window is the producer write + seal + update_from_ovstage, isolating the drain.
    bool applyOnce(float velocityX) override
    {
#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
        ++mOrdinal;
        const size_t n = static_cast<size_t>(mBodyCount);
        for (size_t i = 0; i < n; ++i)
        {
            mHostVel[i * 3 + 0] = velocityX;
            mHostAng[i * 3 + 0] = velocityX;
        }
        ovstage_op_id_t ops[2] = { OVSTAGE_INVALID_OP_ID, OVSTAGE_INVALID_OP_ID };
        if (!writeDeviceColumn(mTensors[0], mDatas[0], mVelAttr, mDevVel, mHostVel.data(), &ops[0]) ||
            !writeDeviceColumn(mTensors[1], mDatas[1], mAngAttr, mDevAng, mHostAng.data(), &ops[1]))
        {
            for (const ovstage_op_id_t op : ops)
            {
                authoringbm::retireWriteOp(mStageAttachment.stage, op, /*completionProven=*/false, mRowName);
            }
            return false;
        }
        const ovx_token_t floors[2] = { mVelTok, mAngTok };
        bool sealed = false;
        if (!authoringbm::sealAndDrainScoped(mPhysX->handle(), mStageAttachment.stage, &mLastDrained, mOrdinal,
                                             floors, 2, mRowName, &sealed))
        {
            for (const ovstage_op_id_t op : ops)
            {
                authoringbm::retireWriteOp(mStageAttachment.stage, op, sealed, mRowName);
            }
            return false;
        }
        for (const ovstage_op_id_t op : ops)
        {
            if (!authoringbm::retireWriteOp(mStageAttachment.stage, op, /*completionProven=*/true, mRowName))
            {
                return false;
            }
        }
        return true; // no sim step: isolate the drain
#else
        (void)velocityX;
        return false;
#endif
    }

    // Fail-closed, outside the timed window. A drain that silently dropped the device columns would make
    // this row FASTER and still pass a bare existence check (REQ-CAPI-BENCHMARK-001 AC-4 asks for the
    // pose/velocity effect). So author a known velocity + angular velocity through the row's own path, then
    // read both back on the FIRST and LAST body: a dropped column reads ~0 and a partial scatter misses one
    // end. Angular is authored in degrees, so a correct read is the deg->rad-scaled value -- the on-device
    // scale the branch adds.
    void verify() override
    {
        if (!applyOnce(kAuthoringVerifyVelocityX))
        {
            bmRecordFailure(mRowName, "verify write did not complete");
            return;
        }
        constexpr float kDegToRad = 3.14159265358979323846f / 180.0f;
        const float expectedAng = kAuthoringVerifyVelocityX * kDegToRad;
        char last[32];
        std::snprintf(last, sizeof(last), "/World/Body_%06u", mBodyCount - 1);
        const char* paths[2] = { "/World/Body_000000", last };
        for (const char* path : paths)
        {
            float v6[6] = { 0.0f };
            if (!authoringReadVelocity6(mPhysX->handle(), path, mRowName, v6))
            {
                return; // failure already recorded
            }
            // X is the authored lane; no sim step runs, so the drained value survives intact.
            if (std::fabs(v6[0] - kAuthoringVerifyVelocityX) > kAuthoringVerifyVelocityTolerance)
            {
                bmRecordFailure(mRowName, "linear velocity not applied on %s (expected ~%.2f, got %.2f)", path,
                                static_cast<double>(kAuthoringVerifyVelocityX), static_cast<double>(v6[0]));
            }
            if (std::fabs(v6[3] - expectedAng) > kAuthoringVerifyVelocityTolerance * kDegToRad)
            {
                bmRecordFailure(mRowName, "angular velocity not deg->rad scaled on %s (expected ~%.4f, got %.4f)",
                                path, static_cast<double>(expectedAng), static_cast<double>(v6[3]));
            }
        }
    }

    void teardown() override
    {
        if (mQuery != OVSTAGE_INVALID_QUERY_HANDLE && mStageAttachment.stage)
        {
            authoringbm::releaseQueryChecked(mStageAttachment.stage, mQuery, mRowName);
        }
        mQuery = OVSTAGE_INVALID_QUERY_HANDLE;
        freeDeviceBuffers();
    }

private:
#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
    bool writeDeviceColumn(DLTensor& tensor, ovstage_write_data_t& data, const ovx_token_or_string_t& attr,
                           void* devBuf, const void* host, ovstage_op_id_t* outOp)
    {
        if (cudaMemcpy(devBuf, host, static_cast<size_t>(mWriteShape) * 3 * sizeof(float),
                       cudaMemcpyHostToDevice) != cudaSuccess)
        {
            bmRecordFailure(mRowName, "cudaMemcpy(device column H2D) failed");
            return false;
        }
        std::memset(&tensor, 0, sizeof(tensor));
        tensor.data = devBuf;
        tensor.device.device_type = kDLCUDA;
        tensor.device.device_id = mCudaDevice;
        tensor.ndim = 1;
        tensor.shape = &mWriteShape;
        tensor.dtype.code = kDLFloat;
        tensor.dtype.bits = 32;
        tensor.dtype.lanes = 3;
        std::memset(&data, 0, sizeof(data));
        data.tensors = &tensor;
        data.tensor_count = 1;
        data.is_array = false;
        data.semantic = OVSTAGE_SEMANTIC_NONE;
        const ovstage_enqueue_result_t e =
            ovstage_write_attribute(mStageAttachment.stage, mQuery, attr, mOrdinal, data, OVSTAGE_PRIM_MODE_UPSERT);
        if (e.status != OVSTAGE_OK)
        {
            bmRecordFailure(mRowName, "write_attribute rejected: %d", static_cast<int>(e.status));
            return false;
        }
        *outOp = e.op_index;
        return true;
    }
#endif

    void freeDeviceBuffers()
    {
#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
        for (void** buf : { &mDevVel, &mDevAng })
        {
            if (*buf)
            {
                cudaFree(*buf);
                *buf = nullptr;
            }
        }
#endif
    }

    uint32_t mBodyCount = 0;
    ovstage_query_handle_t mQuery = OVSTAGE_INVALID_QUERY_HANDLE;
    ovx_token_or_string_t mVelAttr{};
    ovx_token_or_string_t mAngAttr{};
    ovx_token_t mVelTok{0};
    ovx_token_t mAngTok{0};
    ovstage_ordinal_t mOrdinal = 0;
    ovstage_ordinal_t mLastDrained = 0;
    int64_t mWriteShape = 1;
    std::vector<float> mHostVel;
    std::vector<float> mHostAng;
    DLTensor mTensors[2]{};
    ovstage_write_data_t mDatas[2]{};
#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
    void* mDevVel = nullptr;
    void* mDevAng = nullptr;
    int mCudaDevice = 0;
#endif
};

struct AuthoringDrainOvstageGpu4096 : AuthoringDrainOvstageGpuScaled
{
    AuthoringDrainOvstageGpu4096() : AuthoringDrainOvstageGpuScaled("Authoring.drain_ovstage_scaled_4096_gpu", 4096)
    {
    }
};
struct AuthoringDrainOvstageGpu16384 : AuthoringDrainOvstageGpuScaled
{
    AuthoringDrainOvstageGpu16384() : AuthoringDrainOvstageGpuScaled("Authoring.drain_ovstage_scaled_16384_gpu", 16384)
    {
    }
};
struct AuthoringDrainOvstageGpu65536 : AuthoringDrainOvstageGpuScaled
{
    AuthoringDrainOvstageGpu65536() : AuthoringDrainOvstageGpuScaled("Authoring.drain_ovstage_scaled_65536_gpu", 65536)
    {
    }
};

Register<AuthoringDrainOvstage, true> sAuthoringDrainOvstage("Authoring.drain_ovstage_cpu");
Register<AuthoringDrainOvstageGpu, true> sAuthoringDrainOvstageGpu("Authoring.drain_ovstage_gpu");
Register<AuthoringDrainTensor, true> sAuthoringDrainTensor("Authoring.drain_tensor_cpu");
Register<AuthoringDrainStepOnly, true> sAuthoringDrainStepOnly("Authoring.drain_step_only_cpu");
// The scaled GPU rows register unconditionally (like drain_ovstage_gpu) and fail in setup without CUDA, so
// a CPU-only benchmark build still lists all of producer_inventory.json's rows (the contract's step 1).
Register<AuthoringDrainOvstageGpu4096, true> sAuthoringDrainOvstageGpu4096("Authoring.drain_ovstage_scaled_4096_gpu");
Register<AuthoringDrainOvstageGpu16384, true> sAuthoringDrainOvstageGpu16384("Authoring.drain_ovstage_scaled_16384_gpu");
Register<AuthoringDrainOvstageGpu65536, true> sAuthoringDrainOvstageGpu65536("Authoring.drain_ovstage_scaled_65536_gpu");

} // namespace
