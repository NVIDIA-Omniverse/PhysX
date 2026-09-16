// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-BENCHMARK-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-CAPI-BENCHMARK-005
 * @covers AC-1 AC-2 AC-4 AC-5
 */
//
// WriteScaling.* and WriteScalingHighN.*: bulk rigid-body velocity update
// through the two public ovphysx write paths, at N CPU bodies.
//
//   velocity_ovstage_<N>_cpu  ovstage_write_attribute(physics:velocity, UPSERT)
//                             -> advance_write_floor(INCLUDE physics:velocity)
//                             -> updateFromOvstage -> step
//
//   velocity_tensor_<N>_cpu   persistent tensor binding write -> step
//
// Three scales are registered:
//
//   WriteScaling.*       4,096 bodies. These two rows are the L1B contract.
//   WriteScalingHighN.*  8,192 and 16,384 bodies. Hidden diagnostics, because
//                        the 4,096-body point sits before the observed OVStage
//                        slowdown becomes obvious. They share the measured
//                        sequence with the 4,096-body pair and differ only in
//                        N. The family prefix differs on purpose: the L1B lane
//                        selects `WriteScaling.*_cpu`, and a row matching that
//                        glob would widen the L1B row set.
//
// Each pair prices the OVStage control plane against the direct tensor write,
// so both lanes have to keep the shape they were measured in.
//
// What the two lanes write is NOT the same state. The tensor binding is [N,6]
// (linear and angular velocity) and each write sets the linear component and
// zeroes the angular one. The OVStage lane writes only the three-lane linear
// physics:velocity and never touches angular velocity. The comparison targets
// the same linear-velocity outcome through two control paths, not an identical
// full-state write, and the correctness gate checks exactly that: the linear
// vx every body ends up with.
//
// Timing boundary:
//
//   outside  scene generation, ovstage populate + attach, path-list creation,
//            attribute-token interning, persistent tensor-binding creation,
//            per-step value fill (preStep), and the correctness gate
//   inside   the bulk write for this row's lane plus one 1/60 s step
//
// The OVStage lane creates and releases its query inside the timed region. The
// query is part of what a front end pays to address N prims through the
// control plane, so hoisting it out of the measurement would flatter that lane.
//
// These numbers are NOT comparable with the Authoring.* OVStage rows:
//
//   - This lane's timed region contains three waits (on the write, on the
//     release-query, and on the seal). The Authoring.* OVStage rows wait only
//     on the seal and retire the write op afterwards on the strength of that
//     seal's completion, so their numbers are lower for reasons unrelated to
//     body count.
//   - This lane steps with step() + waitAll(). The Authoring.* rows use
//     ovphysx_step_sync(), which is cheaper because it bypasses the async
//     operation machinery.
//
// Read a WriteScaling row against the other WriteScaling row, not against an
// Authoring one. Re-basing this lane onto the Authoring boundary would change
// what it reports and is deliberately not done.
//
// Robustness choices:
//
//   - Every ovstage/population/ovphysx wait goes through the shared
//     AuthoringCommon helpers (authoringbm::waitStage / waitPopulation /
//     waitOvphysxAllChecked / releaseQueryChecked). Those use the bounded
//     kWaitTimeoutNs and, on a timeout, record the failure and block for final
//     completion instead of releasing a still-pending op. The outer
//     benchmark-process timeout stays the hard bound.
//   - The OVStage write payload (DLTensor + ovstage_write_data_t) lives in the
//     benchmark object rather than on the stack of the write function. A
//     bounded wait that times out returns while the op is still pending, so
//     stack-owned payload would be freed under a live operation.
//   - Every setup, write, seal, drain, step and readback failure routes through
//     bmRecordFailure(), so a row that did not complete its work publishes no
//     number and the process exits non-zero. That includes every waitAll(),
//     which is the only place an asynchronous failure is reported.
//   - The generated scene explicitly applies PhysxSceneAPI and authors
//     physxScene:enableGPUDynamics = false and physxScene:broadphaseType =
//     "MBP", matching the CPU Authoring fixture
//     (tests/data/simple_physics_scene_cpu.usda). All three authored lines are
//     verified inside the PhysicsScene block before the stage is populated.
//     ovphysx exposes no realized-device query, so the row is CPU by
//     construction from that authoring.
//   - After the timed steps, the row writes a distinctive velocity through its
//     own write path, steps, and reads the rigid-body velocity back. A control
//     path that silently no-ops still costs time, so timing alone cannot catch
//     it. The gate runs in endRun(), outside the measured window.
//   - Stage lifetime follows the Authoring convention:
//     prepareStageAttachmentForRun() refuses to start a run over a retained
//     attachment, and clearOvstageChecked() keeps the attachment on a
//     failed teardown rather than orphaning a caller-owned Stage. A Stage that
//     ovphysx never took is torn down by the plain destroy instead, so a
//     failure before the attach is not followed by a detach error that
//     misattributes it.

#include "framework/UsdPCH.h"

#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"
#include "../framework/BmUtils.h"
#include "../BenchmarkFailure.h"
#include "AuthoringCommon.h"

#include <ovphysx/experimental/ovphysx.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>


void initWriteScaling()
{
}


namespace
{

// The L1B scale plus the two hidden high-N diagnostic scales.
const uint32_t kWriteScalingBodyCount = 4096;
const uint32_t kWriteScalingHighNBodyCount8192 = 8192;
const uint32_t kWriteScalingHighNBodyCount16384 = 16384;

const float kStepDt = 1.0f / 60.0f;
const char* const kVelocityAttribute = "physics:velocity";
const char* const kBodyPathPattern = "/World/Body_*";

// The timed loop only ever writes +1 or -1 m/s, so this value cannot be
// produced by a stale or ignored write. The scene authors zero gravity and the
// bodies are spaced so they never touch, so one step preserves the written
// linear velocity exactly. The tolerance only absorbs float32 round-trip.
const float kVerifyVelocityX = 3.5f;
const float kVerifyTolerance = 0.05f;

// Authored lines the generated scene must carry for the _cpu label to mean
// anything. Emitted from these constants and checked against them, so a future
// edit to the generator cannot quietly drop the CPU configuration. The schema
// line matters as much as the two attributes: physxScene:* opinions on a prim
// without PhysxSceneAPI applied are inert, so the scene would silently fall
// back to the GPU dynamics default while the row still called itself _cpu.
const char* const kCpuSceneApiAuthoring = "prepend apiSchemas = [\"PhysxSceneAPI\"]";
const char* const kCpuDynamicsAuthoring = "bool physxScene:enableGPUDynamics = false";
const char* const kCpuBroadphaseAuthoring = "uniform token physxScene:broadphaseType = \"MBP\"";
const char* const kPhysicsScenePrim = "def PhysicsScene \"PhysicsScene\"";


enum class WriteLane
{
    eOvstage,
    eTensor,
};


std::string bodyPath(uint32_t index)
{
    std::ostringstream stream;
    stream << "/World/Body_" << std::setw(6) << std::setfill('0') << index;
    return stream.str();
}


// The scene text is identical for both rows of a scale and is rebuilt for every
// run, so it is cached per scale. One process can run all three scales, and the
// harness's sorted row order alternates between them, so a single-slot cache
// would regenerate the 16,384-body text (a few megabytes) on every alternation.
// Returned by reference.
const std::string& makeScene(uint32_t bodyCount)
{
    static std::map<uint32_t, std::string> cachedScenes;
    const auto cached = cachedScenes.find(bodyCount);
    if (cached != cachedScenes.end())
    {
        return cached->second;
    }

    const uint32_t side = static_cast<uint32_t>(std::ceil(std::sqrt(static_cast<double>(bodyCount))));
    std::ostringstream stream;
    stream << "#usda 1.0\n"
              "(\n"
              "    defaultPrim = \"World\"\n"
              "    metersPerUnit = 1\n"
              "    upAxis = \"Y\"\n"
              ")\n\n"
              "def Xform \"World\"\n"
              "{\n"
              "    "
           << kPhysicsScenePrim << " (\n"
           << "        " << kCpuSceneApiAuthoring << "\n"
           << "    )\n"
              "    {\n"
              "        vector3f physics:gravityDirection = (0, -1, 0)\n"
              "        float physics:gravityMagnitude = 0\n"
              "        uint physxScene:timeStepsPerSecond = 240\n"
              "        "
           << kCpuDynamicsAuthoring << "\n"
           << "        " << kCpuBroadphaseAuthoring << "\n"
           << "    }\n\n";

    for (uint32_t index = 0; index < bodyCount; ++index)
    {
        const uint32_t row = index / side;
        const uint32_t column = index % side;
        stream << "    def Cube \"Body_" << std::setw(6) << std::setfill('0') << index << "\" (\n"
               << "        prepend apiSchemas = "
                  "[\"PhysicsRigidBodyAPI\", \"PhysicsCollisionAPI\", \"PhysicsMassAPI\"]\n"
               << "    )\n"
               << "    {\n"
               << "        double size = 0.25\n"
               << "        float physics:mass = 1\n"
               << "        vector3f physics:velocity = (0, 0, 0)\n"
               << "        vector3f physics:angularVelocity = (0, 0, 0)\n"
               << "        double3 xformOp:translate = (" << static_cast<double>(column) * 0.5 << ", 0, "
               << static_cast<double>(row) * 0.5 << ")\n"
               << "        uniform token[] xformOpOrder = [\"xformOp:translate\"]\n"
               << "    }\n\n";
    }
    stream << "}\n";
    return cachedScenes.emplace(bodyCount, stream.str()).first->second;
}


// Scoped to the PhysicsScene block (from the prim declaration to the first
// body prim) so a matching line authored anywhere else in the generated text
// cannot satisfy the check, and so the schema is required to be applied to the
// prim that carries the two attributes rather than merely present somewhere.
bool sceneAuthorsCpuConfiguration(const std::string& usda)
{
    const size_t sceneStart = usda.find(kPhysicsScenePrim);
    if (sceneStart == std::string::npos)
    {
        return false;
    }
    const size_t bodyStart = usda.find("def Cube", sceneStart);
    const std::string sceneBlock =
        usda.substr(sceneStart, bodyStart == std::string::npos ? std::string::npos : bodyStart - sceneStart);

    return sceneBlock.find(kCpuSceneApiAuthoring) != std::string::npos &&
           sceneBlock.find(kCpuDynamicsAuthoring) != std::string::npos &&
           sceneBlock.find(kCpuBroadphaseAuthoring) != std::string::npos;
}


class WriteScalingBase : public BmBenchmark
{
public:
    WriteScalingBase(const char* rowName, WriteLane lane, uint32_t bodyCount)
        : mRow(rowName), mLane(lane), mBodyCount(bodyCount)
    {
    }

    // CPU-only rows: the scene authors CPU dynamics, so they belong to the
    // default pass and are skipped under --forceGpu, matching the Step.*_cpu
    // and Authoring.*_cpu convention.
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
        return 20;
    }

    uint32_t getNbRuns() const override
    {
        return 5;
    }

    void startRun() override
    {
        mSetupOk = false;
        mStepOk = false;
        mStepIndex = 0;
        mPhysX = BmGlobals::getInstance().getPhysX();
        if (!authoringbm::prepareStageAttachmentForRun(mAttachment, mRow, &mLeftoverAttachmentReported))
        {
            // A retained attachment keeps whatever attach state it was left
            // with, so mAttached must not be reset here: teardown still has to
            // pick the right retry path for that stage.
            return;
        }
        mAttached = false;
        if (!mPhysX)
        {
            bmRecordFailure(mRow, "no ovphysx instance available");
            return;
        }

        const std::string& usda = makeScene(mBodyCount);
        if (!sceneAuthorsCpuConfiguration(usda))
        {
            bmRecordFailure(mRow,
                            "generated scene does not author the CPU configuration on its "
                            "%s block (expected '%s', '%s' and '%s')",
                            kPhysicsScenePrim, kCpuSceneApiAuthoring, kCpuDynamicsAuthoring,
                            kCpuBroadphaseAuthoring);
            return;
        }

        // The scope is inert for a CPU row (nothing is armed), but it keeps the
        // shared device-evidence helper's contract: it must be alive across the
        // attach that a GPU row would be probing.
        authoringbm::GpuFallbackScope gpuScope(false, mRow);
        if (!authoringbm::populateAndAttachChecked(mPhysX, authoringbm::UsdSource::eString, usda,
                                                   "ovphysx-write-scaling", mAttachment, mRow, &mAttached))
        {
            return;
        }
        if (!authoringbm::checkDeviceEvidenceAndWarmup(mPhysX->handle(), /*gpu=*/false, gpuScope, mRow))
        {
            return;
        }
        printFormatted(
            "%s: authored CPU configuration verified on the PhysicsScene prim "
            "(PhysxSceneAPI applied, enableGPUDynamics=false, broadphaseType=MBP)",
            mRow);

        if (!buildWritePath())
        {
            return;
        }

        mSetupOk = true;
        mStepOk = true;

        // Untimed warmup on this same stage. The harness's dummy run warms a
        // different stage that is then cleared, so without this the first write
        // and the first step of every recorded run would be timed cold.
        preStep();
        if (!writeOnce() || !stepOnce())
        {
            mSetupOk = false;
            mStepOk = false;
        }
    }

    // Value generation is deliberately outside the measured window: the
    // benchmark prices the write and the step, not the arithmetic that fills
    // the buffer.
    void preStep() override
    {
        fillVelocities((mStepIndex++ & 1u) == 0 ? 1.0f : -1.0f);
    }

    void endRun() override
    {
        if (mSetupOk && mStepOk)
        {
            verify();
        }
        else if (mSetupOk && !mStepOk)
        {
            bmRecordFailure(mRow, "write/step sequence did not complete");
        }
        teardown();
    }

protected:
    void step() override
    {
        if (!mSetupOk || !mStepOk)
        {
            return;
        }
        if (!writeOnce() || !stepOnce())
        {
            mStepOk = false;
        }
    }

private:
    bool buildWritePath()
    {
        mPaths.clear();
        mPathViews.clear();
        mPaths.reserve(mBodyCount);
        mPathViews.reserve(mBodyCount);
        for (uint32_t index = 0; index < mBodyCount; ++index)
        {
            mPaths.push_back(bodyPath(index));
        }
        for (const std::string& path : mPaths)
        {
            mPathViews.push_back(authoringbm::stringView(path));
        }

        mDictionary = ovstage_get_path_dictionary(mAttachment.stage);
        if (!mDictionary)
        {
            bmRecordFailure(mRow, "get_path_dictionary returned null");
            return false;
        }
        if (path_dictionary_create_path_list_from_strings(mDictionary, mPathViews.data(), mPathViews.size(),
                                                         &mPathList)
                    .status != OVX_API_SUCCESS ||
            mPathList == OVX_INVALID_PRIMPATH_LIST)
        {
            bmRecordFailure(mRow, "path-list creation failed for N=%u", mBodyCount);
            return false;
        }

        const ovx_string_t velocityName = authoringbm::stringView(kVelocityAttribute);
        if (path_dictionary_create_tokens_from_strings(mDictionary, &velocityName, 1, &mVelocityToken).status !=
                OVX_API_SUCCESS ||
            mVelocityToken == OVX_INVALID_TOKEN)
        {
            bmRecordFailure(mRow, "intern %s failed", kVelocityAttribute);
            return false;
        }

        mStageVelocities.assign(static_cast<size_t>(mBodyCount) * 3, 0.0f);
        mTensorVelocities.assign(static_cast<size_t>(mBodyCount) * 6, 0.0f);
        mStageShape[0] = static_cast<int64_t>(mBodyCount);

        if (mLane == WriteLane::eTensor && !createTensorBinding())
        {
            return false;
        }
        return true;
    }

    void teardown()
    {
        // Destroy the binding before the stage reset in clearOvstageChecked():
        // a binding's lifetime is tied to the realized physics objects it was
        // created against.
        //
        // mBinding is only ever set by createTensorBinding(), which startRun()
        // reaches only after checking mPhysX, so a live binding without an
        // instance is not a reachable state. Gate on the binding rather than
        // passing handle 0, which would report a release failure against this
        // row instead of whatever actually went wrong.
        if (mBinding != 0)
        {
            authoringbm::destroyTensorBindingChecked(
                mPhysX->handle(), mBinding, mRow, "persistent velocity binding");
        }
        authoringbm::releasePathListChecked(mDictionary, mPathList, mRow, "body path-list");
        mVelocityToken = OVX_INVALID_TOKEN;
        mDictionary = nullptr;
        // A Stage ovphysx never took needs the plain destroy, not the reset +
        // detach path: attempting to detach a never-attached Stage reports a
        // failure that misattributes the original fault. Passing a null
        // instance selects exactly that, and still retains the Stage if the
        // destroy fails.
        if (!authoringbm::clearOvstageChecked(mAttached ? mPhysX : nullptr, mAttachment, mRow))
        {
            bmRecordFailure(mRow, "ovstage teardown failed; retaining the stage attachment");
            mLeftoverAttachmentReported = true;
        }
        else
        {
            mAttached = false;
        }
    }

    void fillVelocities(float velocityX)
    {
        for (uint32_t index = 0; index < mBodyCount; ++index)
        {
            float* stage = mStageVelocities.data() + static_cast<size_t>(index) * 3;
            stage[0] = velocityX;
            stage[1] = 0.0f;
            stage[2] = 0.0f;

            float* tensor = mTensorVelocities.data() + static_cast<size_t>(index) * 6;
            tensor[0] = velocityX;
            tensor[1] = 0.0f;
            tensor[2] = 0.0f;
            tensor[3] = 0.0f;
            tensor[4] = 0.0f;
            tensor[5] = 0.0f;
        }
    }

    bool writeOnce()
    {
        return mLane == WriteLane::eOvstage ? writeThroughOvstage() : writeThroughTensor();
    }

    // The raw C handle rather than ovphysx::TensorBinding: the RAII wrapper's
    // destroy() returns void, so a teardown that used it could not report a
    // failed destroy, and calling the C destroy alongside it would release the
    // same handle twice.
    // DEPRECATED (tensor-binding-deprecation): the eTensor lane measures the deprecated binding write. It retires with it.
    bool createVelocityBinding(ovphysx_tensor_binding_handle_t& binding, const char* what)
    {
        ovphysx_tensor_binding_desc_t desc;
        std::memset(&desc, 0, sizeof(desc));
        desc.pattern = ovphysx_cstr(kBodyPathPattern);
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32;
        if (ovphysx_create_tensor_binding(mPhysX->handle(), &desc, &binding).status != OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t err = ovphysx_get_last_error();
            bmRecordFailure(mRow, "%s creation failed for N=%u: %.*s", what, mBodyCount,
                            static_cast<int>(err.length), err.ptr ? err.ptr : "");
            return false;
        }
        return true;
    }

    bool velocityBindingShapeOk(ovphysx_tensor_binding_handle_t binding, ovphysx_tensor_spec_t& spec, const char* what)
    {
        std::memset(&spec, 0, sizeof(spec));
        if (ovphysx_get_tensor_binding_spec(mPhysX->handle(), binding, &spec).status != OVPHYSX_API_SUCCESS ||
            spec.ndim != 2 || spec.shape[0] != static_cast<int64_t>(mBodyCount) || spec.shape[1] != 6)
        {
            bmRecordFailure(mRow, "unexpected %s shape for N=%u: ndim=%d", what, mBodyCount,
                            static_cast<int>(spec.ndim));
            return false;
        }
        return true;
    }

    bool createTensorBinding()
    {
        if (!createVelocityBinding(mBinding, "velocity binding"))
        {
            return false;
        }

        ovphysx_tensor_spec_t spec;
        if (!velocityBindingShapeOk(mBinding, spec, "velocity binding"))
        {
            return false;
        }

        mTensorShape[0] = spec.shape[0];
        mTensorShape[1] = spec.shape[1];
        std::memset(&mTensorView, 0, sizeof(mTensorView));
        mTensorView.data = mTensorVelocities.data();
        mTensorView.device.device_type = kDLCPU;
        mTensorView.ndim = 2;
        mTensorView.shape = mTensorShape;
        mTensorView.dtype.code = static_cast<uint8_t>(kDLFloat);
        mTensorView.dtype.bits = 32;
        mTensorView.dtype.lanes = 1;
        return true;
    }

    bool writeThroughTensor()
    {
        if (ovphysx_write_tensor_binding(mPhysX->handle(), mBinding, &mTensorView, nullptr).status !=
            OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t err = ovphysx_get_last_error();
            bmRecordFailure(mRow, "tensor velocity write failed for N=%u: %.*s", mBodyCount,
                            static_cast<int>(err.length), err.ptr ? err.ptr : "");
            return false;
        }
        return authoringbm::waitOvphysxAllChecked(mPhysX->handle(), mRow, "tensor velocity write");
    }

    bool writeThroughOvstage()
    {
        ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
        const ovstage_api_status_t queryStatus = ovstage_query_from_path_list(mAttachment.stage, mPathList, &query);
        if (queryStatus != OVSTAGE_OK || query == OVSTAGE_INVALID_QUERY_HANDLE)
        {
            bmRecordFailure(mRow, "query_from_path_list failed for N=%u: status=%d (%s)", mBodyCount,
                            static_cast<int>(queryStatus), ovstage_get_error_string(mAttachment.stage, queryStatus));
            return false;
        }

        // Object-owned payload: a bounded wait that times out returns while the
        // write is still pending, so this storage must outlive the call.
        std::memset(&mStageTensor, 0, sizeof(mStageTensor));
        mStageTensor.data = mStageVelocities.data();
        mStageTensor.device.device_type = kDLCPU;
        mStageTensor.ndim = 1;
        mStageTensor.shape = mStageShape;
        mStageTensor.dtype.code = static_cast<uint8_t>(kDLFloat);
        mStageTensor.dtype.bits = 32;
        mStageTensor.dtype.lanes = 3;

        std::memset(&mStageWrite, 0, sizeof(mStageWrite));
        mStageWrite.tensors = &mStageTensor;
        mStageWrite.tensor_count = 1;
        mStageWrite.is_array = false;
        mStageWrite.semantic = OVSTAGE_SEMANTIC_NONE;

        const ovstage_ordinal_t ordinal = ++mAttachment.ordinal;
        const ovstage_enqueue_result_t enqueue =
            ovstage_write_attribute(mAttachment.stage, query, authoringbm::nameToken(mVelocityToken), ordinal,
                                    mStageWrite, OVSTAGE_PRIM_MODE_UPSERT);
        if (enqueue.status != OVSTAGE_OK)
        {
            bmRecordFailure(mRow, "write_attribute(%s) rejected: %d", kVelocityAttribute,
                            static_cast<int>(enqueue.status));
            authoringbm::releaseQueryChecked(mAttachment.stage, query, mRow);
            return false;
        }

        // The query is released whichever way the write went, so a failed write
        // cannot leak a handle the pinned ovstage contract requires retired
        // before ovstage_destroy_instance().
        const bool writeOk = authoringbm::waitStage(mAttachment.stage, enqueue.op_index, mRow, "write_attribute");
        const bool releaseOk = authoringbm::releaseQueryChecked(mAttachment.stage, query, mRow);
        if (!writeOk || !releaseOk)
        {
            return false;
        }

        // Sealing only physics:velocity is the measured shape: this lane prices
        // a single-attribute control-plane refresh, not an end-of-frame
        // SCOPE_ALL seal.
        ovstage_write_floor_desc_t floorDesc;
        std::memset(&floorDesc, 0, sizeof(floorDesc));
        floorDesc.ordinal = ordinal;
        floorDesc.scope = OVSTAGE_SCOPE_INCLUDE;
        floorDesc.attributes = &mVelocityToken;
        floorDesc.attribute_count = 1;
        const ovstage_enqueue_result_t seal = ovstage_advance_write_floor(mAttachment.stage, &floorDesc);
        if (seal.status != OVSTAGE_OK)
        {
            bmRecordFailure(mRow, "advance_write_floor enqueue rejected: %d", static_cast<int>(seal.status));
            return false;
        }
        if (!authoringbm::waitStage(mAttachment.stage, seal.op_index, mRow, "advance_write_floor"))
        {
            return false;
        }

        ovstage_ordinal_range_t range;
        std::memset(&range, 0, sizeof(range));
        range.has_start_ordinal = true;
        range.start_ordinal = ordinal;
        range.end_ordinal = ordinal;
        if (mPhysX->updateFromOvstage(range) != OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t err = ovphysx_get_last_error();
            bmRecordFailure(mRow, "update_from_ovstage failed: %.*s", static_cast<int>(err.length),
                            err.ptr ? err.ptr : "");
            return false;
        }
        return true;
    }

    // step() + waitAll() is the measured shape. ovphysx_step_sync() would be
    // cheaper, but swapping it in would change the number this row reports.
    bool stepOnce()
    {
        if (mPhysX->step(kStepDt) != OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t err = ovphysx_get_last_error();
            bmRecordFailure(mRow, "step failed for N=%u: %.*s", mBodyCount, static_cast<int>(err.length),
                            err.ptr ? err.ptr : "");
            return false;
        }
        return authoringbm::waitOvphysxAllChecked(mPhysX->handle(), mRow, "step");
    }

    // Fail-closed gate, outside the timed window: push a velocity no timed step
    // ever writes through this row's own path, step, and read the simulated
    // bodies back. A write path that parses and costs time but never reaches
    // the PhysX actor fails here instead of publishing a fast number.
    void verify()
    {
        fillVelocities(kVerifyVelocityX);
        if (!writeOnce() || !stepOnce())
        {
            bmRecordFailure(mRow, "verify write/step did not complete");
            return;
        }

        std::vector<float> velocities;
        if (!readVelocities(velocities))
        {
            return;
        }
        for (uint32_t index = 0; index < mBodyCount; ++index)
        {
            const float velocityX = velocities[static_cast<size_t>(index) * 6];
            if (velocityX < kVerifyVelocityX - kVerifyTolerance || velocityX > kVerifyVelocityX + kVerifyTolerance)
            {
                bmRecordFailure(mRow, "body %u did not take the written velocity (expected vx ~= %.3f, got %.3f)",
                                index, static_cast<double>(kVerifyVelocityX), static_cast<double>(velocityX));
                return;
            }
        }
    }

    // Readback binding, created and destroyed inside the gate so it is never
    // part of what either lane is measured on.
    bool readVelocities(std::vector<float>& out)
    {
        ovphysx_tensor_binding_handle_t readBinding = 0;
        if (!createVelocityBinding(readBinding, "verify velocity binding"))
        {
            return false;
        }

        ovphysx_tensor_spec_t spec;
        if (!velocityBindingShapeOk(readBinding, spec, "verify velocity binding"))
        {
            authoringbm::destroyTensorBindingChecked(mPhysX->handle(), readBinding, mRow, "verify velocity binding");
            return false;
        }

        out.assign(static_cast<size_t>(mBodyCount) * 6, 0.0f);
        int64_t shape[2] = { spec.shape[0], spec.shape[1] };
        DLTensor tensor;
        std::memset(&tensor, 0, sizeof(tensor));
        tensor.data = out.data();
        tensor.device.device_type = kDLCPU;
        tensor.ndim = 2;
        tensor.shape = shape;
        tensor.dtype.code = static_cast<uint8_t>(kDLFloat);
        tensor.dtype.bits = 32;
        tensor.dtype.lanes = 1;

        bool ok = true;
        if (ovphysx_read_tensor_binding(mPhysX->handle(), readBinding, &tensor).status != OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t err = ovphysx_get_last_error();
            bmRecordFailure(mRow, "verify velocity readback failed: %.*s", static_cast<int>(err.length),
                            err.ptr ? err.ptr : "");
            ok = false;
        }
        return authoringbm::destroyTensorBindingChecked(
                   mPhysX->handle(), readBinding, mRow, "verify velocity binding") &&
               ok;
    }

    const char* mRow = nullptr;
    WriteLane mLane = WriteLane::eOvstage;
    uint32_t mBodyCount = 0;

    ovphysx::PhysX* mPhysX = nullptr;
    ovphysx_sample_stage_attachment_t mAttachment{};
    bool mAttached = false;
    bool mLeftoverAttachmentReported = false;
    bool mSetupOk = false;
    bool mStepOk = false;
    uint32_t mStepIndex = 0;

    path_dictionary_instance_t* mDictionary = nullptr;
    ovx_primpath_list_t mPathList = OVX_INVALID_PRIMPATH_LIST;
    ovx_token_t mVelocityToken = OVX_INVALID_TOKEN;
    std::vector<std::string> mPaths;
    std::vector<ovx_string_t> mPathViews;

    std::vector<float> mStageVelocities;
    std::vector<float> mTensorVelocities;
    int64_t mStageShape[1] = { 0 };
    int64_t mTensorShape[2] = { 0, 0 };
    DLTensor mStageTensor{};
    ovstage_write_data_t mStageWrite{};
    ovphysx_tensor_binding_handle_t mBinding = 0;
    DLTensor mTensorView{};
};


struct WriteScalingVelocityOvstage4096Cpu : WriteScalingBase
{
    WriteScalingVelocityOvstage4096Cpu()
        : WriteScalingBase("WriteScaling.velocity_ovstage_4096_cpu", WriteLane::eOvstage, kWriteScalingBodyCount)
    {
    }
};

struct WriteScalingVelocityTensor4096Cpu : WriteScalingBase
{
    WriteScalingVelocityTensor4096Cpu()
        : WriteScalingBase("WriteScaling.velocity_tensor_4096_cpu", WriteLane::eTensor, kWriteScalingBodyCount)
    {
    }
};


Register<WriteScalingVelocityOvstage4096Cpu, true> sWriteScalingVelocityOvstage4096Cpu(
    "WriteScaling.velocity_ovstage_4096_cpu");
Register<WriteScalingVelocityTensor4096Cpu, true> sWriteScalingVelocityTensor4096Cpu(
    "WriteScaling.velocity_tensor_4096_cpu");


// The hidden high-N diagnostics: the identical measured sequence at 8,192 and
// 16,384 bodies. Registered under their own family prefix so the frozen L1B
// `WriteScaling.*_cpu` selection keeps returning exactly the two rows above.
struct WriteScalingHighNVelocityOvstage8192Cpu : WriteScalingBase
{
    WriteScalingHighNVelocityOvstage8192Cpu()
        : WriteScalingBase("WriteScalingHighN.velocity_ovstage_8192_cpu", WriteLane::eOvstage,
                           kWriteScalingHighNBodyCount8192)
    {
    }
};

struct WriteScalingHighNVelocityTensor8192Cpu : WriteScalingBase
{
    WriteScalingHighNVelocityTensor8192Cpu()
        : WriteScalingBase("WriteScalingHighN.velocity_tensor_8192_cpu", WriteLane::eTensor,
                           kWriteScalingHighNBodyCount8192)
    {
    }
};

struct WriteScalingHighNVelocityOvstage16384Cpu : WriteScalingBase
{
    WriteScalingHighNVelocityOvstage16384Cpu()
        : WriteScalingBase("WriteScalingHighN.velocity_ovstage_16384_cpu", WriteLane::eOvstage,
                           kWriteScalingHighNBodyCount16384)
    {
    }
};

struct WriteScalingHighNVelocityTensor16384Cpu : WriteScalingBase
{
    WriteScalingHighNVelocityTensor16384Cpu()
        : WriteScalingBase("WriteScalingHighN.velocity_tensor_16384_cpu", WriteLane::eTensor,
                           kWriteScalingHighNBodyCount16384)
    {
    }
};


Register<WriteScalingHighNVelocityOvstage8192Cpu, true> sWriteScalingHighNVelocityOvstage8192Cpu(
    "WriteScalingHighN.velocity_ovstage_8192_cpu");
Register<WriteScalingHighNVelocityTensor8192Cpu, true> sWriteScalingHighNVelocityTensor8192Cpu(
    "WriteScalingHighN.velocity_tensor_8192_cpu");
Register<WriteScalingHighNVelocityOvstage16384Cpu, true> sWriteScalingHighNVelocityOvstage16384Cpu(
    "WriteScalingHighN.velocity_ovstage_16384_cpu");
Register<WriteScalingHighNVelocityTensor16384Cpu, true> sWriteScalingHighNVelocityTensor16384Cpu(
    "WriteScalingHighN.velocity_tensor_16384_cpu");

} // namespace
