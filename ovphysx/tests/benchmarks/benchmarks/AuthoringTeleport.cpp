// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// DEPRECATED (tensor-binding-deprecation): the binding (tensor) lane retires with the binding. The ovstage lane stays.

/**
 * @implements REQ-CAPI-BENCHMARK-001
 * @covers AC-1 AC-4 AC-5
 */

// Authoring.*: in-process transform-refresh benchmarks. Existing rigid bodies
// move without changing scene topology.
//
//   teleport_tensor_cpu   ovphysx_write_tensor_binding(RIGID_BODY_POSE) + step
//                         (the tensor escape hatch, no ovstage drain)
//
//   teleport_ovstage_cpu  ovstage_write_attribute(omni:xform, UPSERT)
//                         -> advance_write_floor -> wait(seal)
//                         -> ovphysx_update_from_ovstage -> step
//                         (the OVStage control-plane path)
//
// The pair prices the ovstage control plane against the direct tensor write for
// the identical visible result, which is the question a front end refreshing
// transforms every tick faces.
//
// ovphysx consumes existing-body pose control from `omni:xform`, NOT from
// `xformOp:translate`. Writing translate alone leaves the PhysX actor unmoved
// while still paying the full seal and drain cost, a silent no-op that still
// looks like work.
//
// Design choices:
//
//   - No ovphysx_set_cpu_mode(). It is process-wide and non-revertible
//     in-process (see ovphysx/AGENTS.md), so it would poison every other row in
//     this binary. The device comes from the scene instead:
//     simple_physics_scene_cpu.usda authors physxScene:enableGPUDynamics =
//     false, and the row gates on forceGpu(), matching the Step.*_cpu
//     convention. The GPU-requested rows are hidden diagnostics. The CPU rows
//     are canonical.
//   - One measured step is one teleport iteration, so the harness reports
//     per-refresh cost directly.
//   - Scene load and setup happen in startRun() and the correctness gate runs
//     in endRun(), so neither is inside the measured window.

#include "framework/UsdPCH.h"

#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"
#include "../framework/BmUtils.h"
#include "../BenchmarkFailure.h"
#include "AuthoringCommon.h"

#include <ovphysx/experimental/ovphysx.hpp>

#include <cstring>
#include <string>


void initAuthoringTeleport()
{
}


namespace
{

const char* const kAuthoringBodyPath = "/World/Cube1";

// Distinctive height used by the post-timing gate. One sim step from rest drops
// only ~g*dt^2/2 (~1.4 mm), so a 0.05 window is tight but not flaky.
const float kAuthoringVerifyY = 7.5f;
const float kAuthoringVerifyTolerance = 0.05f;


// [N,7] = position xyz + quaternion xyzw (identity).
void authoringFillPose(float* pose7, float x, float y, float z)
{
    pose7[0] = x;
    pose7[1] = y;
    pose7[2] = z;
    pose7[3] = 0.0f;
    pose7[4] = 0.0f;
    pose7[5] = 0.0f;
    pose7[6] = 1.0f;
}


// Row-major identity transform with translation at indices 12-14, matching the
// USD/PhysX row-vector convention.
void authoringFillOmniXform(double* matrix16, float x, float y, float z)
{
    std::memset(matrix16, 0, 16 * sizeof(double));
    matrix16[0] = 1.0;
    matrix16[5] = 1.0;
    matrix16[10] = 1.0;
    matrix16[12] = static_cast<double>(x);
    matrix16[13] = static_cast<double>(y);
    matrix16[14] = static_cast<double>(z);
    matrix16[15] = 1.0;
}


// The per-iteration target height, cycling so successive writes differ and a
// stale-value regression cannot pass unnoticed.
float authoringIterationY(int iteration)
{
    return 5.0f + 0.05f * static_cast<float>((iteration % 11) + 1);
}


// Zeroes linear+angular velocity so residual fall speed from the timed loop
// cannot drag the body below the written height during the single verify step.
bool authoringZeroVelocity(ovphysx_handle_t handle, const char* row)
{
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc;
    std::memset(&desc, 0, sizeof(desc));
    desc.pattern = ovphysx_cstr(kAuthoringBodyPath);
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32;
    if (ovphysx_create_tensor_binding(handle, &desc, &binding).status != OVPHYSX_API_SUCCESS)
    {
        bmRecordFailure(row, "create_tensor_binding(velocity) failed");
        return false;
    }

    float velocity[6] = { 0.0f };
    int64_t shape[2] = { 1, 6 };
    DLTensor tensor;
    std::memset(&tensor, 0, sizeof(tensor));
    tensor.data = velocity;
    tensor.ndim = 2;
    tensor.shape = shape;
    tensor.dtype.code = kDLFloat;
    tensor.dtype.bits = 32;
    tensor.dtype.lanes = 1;
    tensor.device.device_type = kDLCPU;

    const bool ok = ovphysx_write_tensor_binding(handle, binding, &tensor, nullptr).status == OVPHYSX_API_SUCCESS;
    const bool destroyed = authoringbm::destroyTensorBindingChecked(handle, binding, row, "velocity binding");
    if (!ok)
    {
        bmRecordFailure(row, "write_tensor_binding(velocity) failed");
    }
    return ok && destroyed;
}


// ---------------------------------------------------------------------------
// Shared base for the transform-refresh rows.
// ---------------------------------------------------------------------------

class AuthoringTeleportBase : public BmBenchmark
{
public:
    AuthoringTeleportBase(const char* rowName, bool gpu) : mRowName(rowName), mGpu(gpu)
    {
    }

    // Device is authored per scene, so each row belongs to exactly one pass:
    // the _gpu rows load the plain scene (PhysicsScene with no PhysxSceneAPI
    // defaults to GPU dynamics) and the _cpu rows load the _cpu overlay.
    bool isValid() const override
    {
        if (BmGlobals::getInstance().forceGpu() != mGpu)
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

        // Capture must be live across the attach: the CPU-fallback diagnostic
        // is emitted while the scene is being realized, not afterwards.
        mAttached = false;
        authoringbm::GpuFallbackScope gpuScope(mGpu, mRowName);

        const std::string path = BmGlobals::getInstance().getDataFolder() +
                                 (mGpu ? "/simple_physics_scene.usda" : "/simple_physics_scene_cpu.usda");
        if (!authoringbm::populateAndAttachChecked(mPhysX, authoringbm::UsdSource::eFile, path,
                                                   "ovphysx-authoring-teleport", mStageAttachment, mRowName,
                                                   &mAttached))
        {
            return;
        }
        if (!authoringbm::checkDeviceEvidenceAndWarmup(mPhysX->handle(), mGpu, gpuScope, mRowName))
        {
            return;
        }
        mSetupOk = setup();
        mStepOk = mSetupOk;
        // Untimed same-stage warmup. The harness dummy run warms a different
        // stage that is then cleared, so without this the first tensor write of
        // every recorded run would be timed.
        if (mSetupOk)
        {
            for (int w = 0; w < 5; ++w)
            {
                if (!teleportOnce(authoringIterationY(w)))
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
            bmRecordFailure(mRowName, "teleport sequence did not complete");
        }
        teardown();
        // A Stage ovphysx never took needs the plain destroy, not the reset +
        // detach path: attempting to detach a never-attached Stage reports a
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
        if (!teleportOnce(authoringIterationY(mIteration)))
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
    virtual bool teleportOnce(float targetY) = 0;
    virtual bool verifyTeleport(float targetY) = 0;
    virtual void teardown()
    {
    }

    // Fail-closed gate, outside the timed window: teleport to a distinctive
    // height and confirm the PhysX actor actually moved there. A control path
    // that silently no-ops still costs time, so timing alone cannot catch it.
    void verify()
    {
        if (!authoringZeroVelocity(mPhysX->handle(), mRowName))
        {
            bmRecordFailure(mRowName, "could not zero velocity for verify");
            return;
        }
        if (!verifyTeleport(kAuthoringVerifyY))
        {
            bmRecordFailure(mRowName, "verify teleport did not complete");
            return;
        }

        int64_t count = 0;
        float y = 0.0f;
        if (!authoringbm::readBodyPoseAt(mPhysX->handle(), kAuthoringBodyPath, &count, &y, mRowName))
        {
            bmRecordFailure(mRowName, "pose readback failed");
            return;
        }
        if (count < 1)
        {
            bmRecordFailure(mRowName, "no rigid body at %s", kAuthoringBodyPath);
            return;
        }
        if (y < kAuthoringVerifyY - kAuthoringVerifyTolerance || y > kAuthoringVerifyY + kAuthoringVerifyTolerance)
        {
            bmRecordFailure(mRowName, "body did not move (expected y ~= %.3f, got y = %.3f)",
                            static_cast<double>(kAuthoringVerifyY), static_cast<double>(y));
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
    bool mGpu = false;
    int mIteration = 0;
};


// ---------------------------------------------------------------------------
// Case 1: tensor pose write, no ovstage drain.
// ---------------------------------------------------------------------------

class AuthoringTeleportTensor : public AuthoringTeleportBase
{
public:
    AuthoringTeleportTensor(const char* rowName, bool gpu) : AuthoringTeleportBase(rowName, gpu)
    {
    }

protected:
    void resetCase() override
    {
        mBinding = 0;
        std::memset(mPose, 0, sizeof(mPose));
        mShape[0] = 1;
        mShape[1] = 7;
        std::memset(&mTensor, 0, sizeof(mTensor));
    }

    bool setup() override
    {
        ovphysx_tensor_binding_desc_t desc;
        std::memset(&desc, 0, sizeof(desc));
        desc.pattern = ovphysx_cstr(kAuthoringBodyPath);
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
        if (ovphysx_create_tensor_binding(mPhysX->handle(), &desc, &mBinding).status != OVPHYSX_API_SUCCESS)
        {
            bmRecordFailure(mRowName, "create_tensor_binding(pose) failed");
            return false;
        }

        ovphysx_tensor_spec_t spec;
        std::memset(&spec, 0, sizeof(spec));
        if (ovphysx_get_tensor_binding_spec(mPhysX->handle(), mBinding, &spec).status != OVPHYSX_API_SUCCESS)
        {
            bmRecordFailure(mRowName, "get_tensor_binding_spec(pose) failed");
            return false;
        }
        if (spec.ndim < 2 || spec.shape[0] < 1 || spec.shape[1] < 7)
        {
            bmRecordFailure(mRowName, "unexpected pose tensor shape ndim=%d", static_cast<int>(spec.ndim));
            return false;
        }

        mTensor.data = mPose;
        mTensor.ndim = 2;
        mTensor.shape = mShape;
        mTensor.dtype.code = kDLFloat;
        mTensor.dtype.bits = 32;
        mTensor.dtype.lanes = 1;
        mTensor.device.device_type = kDLCPU;
        return true;
    }

    bool teleportOnce(float targetY) override
    {
        authoringFillPose(mPose, 0.0f, targetY, 0.0f);
        if (ovphysx_write_tensor_binding(mPhysX->handle(), mBinding, &mTensor, nullptr).status != OVPHYSX_API_SUCCESS)
        {
            bmRecordFailure(mRowName, "write_tensor_binding(pose) failed");
            return false;
        }
        return stepSync();
    }

    bool verifyTeleport(float targetY) override
    {
        authoringFillPose(mPose, 0.0f, targetY, 0.0f);
        if (ovphysx_write_tensor_binding(mPhysX->handle(), mBinding, &mTensor, nullptr).status != OVPHYSX_API_SUCCESS)
        {
            return false;
        }
        return stepSync();
    }

    void teardown() override
    {
        authoringbm::destroyTensorBindingChecked(
            mPhysX ? mPhysX->handle() : 0, mBinding, mRowName, "pose binding");
    }

private:
    ovphysx_tensor_binding_handle_t mBinding = 0;
    float mPose[7] = { 0.0f };
    int64_t mShape[2] = { 1, 7 };
    DLTensor mTensor{};
};


// ---------------------------------------------------------------------------
// Case 2: ovstage omni:xform write, sealed and drained.
// ---------------------------------------------------------------------------

class AuthoringTeleportOvstage : public AuthoringTeleportBase
{
public:
    AuthoringTeleportOvstage(const char* rowName, bool gpu) : AuthoringTeleportBase(rowName, gpu)
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
        if (!authoringbm::queryPath(mStageAttachment.stage, kAuthoringBodyPath, &mQuery, mRowName))
        {
            bmRecordFailure(mRowName, "query %s failed", kAuthoringBodyPath);
            return false;
        }
        ovx_token_t token = 0;
        if (!authoringbm::internAttr(mStageAttachment.stage, "omni:xform", &token))
        {
            bmRecordFailure(mRowName, "intern omni:xform failed");
            return false;
        }
        mXformAttr = authoringbm::nameToken(token);

        // Ordinals continue from wherever the attach left the stage.
        mOrdinal = mStageAttachment.ordinal;
        mLastDrained = mStageAttachment.ordinal;
        return true;
    }

    bool teleportOnce(float targetY) override
    {
        return writeSealDrainStep(targetY);
    }

    bool verifyTeleport(float targetY) override
    {
        return writeSealDrainStep(targetY);
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
    bool writeSealDrainStep(float targetY)
    {
        authoringFillOmniXform(mMatrix, 0.0f, targetY, 0.0f);

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

    ovstage_query_handle_t mQuery = OVSTAGE_INVALID_QUERY_HANDLE;
    ovx_string_or_token_t mXformAttr{};
    ovstage_ordinal_t mOrdinal = 0;
    ovstage_ordinal_t mLastDrained = 0;
    double mMatrix[16] = { 0.0 };
    int64_t mWriteShape = 1;
    DLTensor mWriteTensor{};
    ovstage_write_data_t mWriteData{};
};


// The CPU pair is canonical. The GPU-requested pair is retained as hidden,
// unscheduled diagnostics with fallback detection but no positive
// realized-device query.
struct AuthoringTeleportTensorCpu : AuthoringTeleportTensor
{
    AuthoringTeleportTensorCpu() : AuthoringTeleportTensor("Authoring.teleport_tensor_cpu", false)
    {
    }
};

struct AuthoringTeleportTensorGpu : AuthoringTeleportTensor
{
    AuthoringTeleportTensorGpu() : AuthoringTeleportTensor("Authoring.teleport_tensor_gpu", true)
    {
    }
};

struct AuthoringTeleportOvstageCpu : AuthoringTeleportOvstage
{
    AuthoringTeleportOvstageCpu() : AuthoringTeleportOvstage("Authoring.teleport_ovstage_cpu", false)
    {
    }
};

struct AuthoringTeleportOvstageGpu : AuthoringTeleportOvstage
{
    AuthoringTeleportOvstageGpu() : AuthoringTeleportOvstage("Authoring.teleport_ovstage_gpu", true)
    {
    }
};


Register<AuthoringTeleportTensorCpu, true> sAuthoringTeleportTensorCpu("Authoring.teleport_tensor_cpu");
Register<AuthoringTeleportTensorGpu, true> sAuthoringTeleportTensorGpu("Authoring.teleport_tensor_gpu");
Register<AuthoringTeleportOvstageCpu, true> sAuthoringTeleportOvstageCpu("Authoring.teleport_ovstage_cpu");
Register<AuthoringTeleportOvstageGpu, true> sAuthoringTeleportOvstageGpu("Authoring.teleport_ovstage_gpu");

} // namespace
