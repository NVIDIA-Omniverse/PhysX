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

#include <cstring>
#include <string>


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


// Reads back linear velocity X for the body under test, the gravity-free axis
// the control paths write.
bool authoringReadVelocityX(ovphysx_handle_t handle, const char* row, float* outX)
{
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc;
    std::memset(&desc, 0, sizeof(desc));
    desc.pattern = ovphysx_cstr(kAuthoringDrainBodyPath);
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32;
    if (ovphysx_create_tensor_binding(handle, &desc, &binding).status != OVPHYSX_API_SUCCESS)
    {
        bmRecordFailure(row, "create_tensor_binding(velocity read) failed");
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

    const bool ok = ovphysx_read_tensor_binding(handle, binding, &tensor).status == OVPHYSX_API_SUCCESS;
    const bool destroyed = authoringbm::destroyTensorBindingChecked(handle, binding, row, "velocity read binding");
    if (!ok)
    {
        bmRecordFailure(row, "read_tensor_binding(velocity) failed");
        return false;
    }
    if (!destroyed)
    {
        return false;
    }
    *outX = velocity[0];
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

        // Capture must be live across the attach. The CPU-fallback diagnostic
        // is emitted while the scene is being realized, not afterwards.
        mAttached = false;
        authoringbm::GpuFallbackScope gpuScope(/*arm=*/false, mRowName);

        const std::string path = BmGlobals::getInstance().getDataFolder() + "/simple_physics_scene_cpu.usda";
        if (!authoringbm::populateAndAttachChecked(mPhysX, authoringbm::UsdSource::eFile, path,
                                                   "ovphysx-authoring-drain", mStageAttachment, mRowName, &mAttached))
        {
            return;
        }
        if (!authoringbm::checkDeviceEvidenceAndWarmup(mPhysX->handle(), /*gpu=*/false, gpuScope, mRowName))
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
        ovx_token_t token = 0;
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
    ovx_string_or_token_t mVelocityAttr{};
    ovstage_ordinal_t mOrdinal = 0;
    ovstage_ordinal_t mLastDrained = 0;
    float mValues[3] = { 0.0f };
    int64_t mWriteShape = 1;
    DLTensor mWriteTensor{};
    ovstage_write_data_t mWriteData{};
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


Register<AuthoringDrainOvstage, true> sAuthoringDrainOvstage("Authoring.drain_ovstage_cpu");
Register<AuthoringDrainTensor, true> sAuthoringDrainTensor("Authoring.drain_tensor_cpu");
Register<AuthoringDrainStepOnly, true> sAuthoringDrainStepOnly("Authoring.drain_step_only_cpu");

} // namespace
