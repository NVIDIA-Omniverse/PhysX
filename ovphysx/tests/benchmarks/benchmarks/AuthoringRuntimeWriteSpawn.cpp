// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-BENCHMARK-001
 * @covers AC-1 AC-4 AC-5
 *
 * @implements REQ-CAPI-BENCHMARK-005
 * @covers AC-1 AC-3 AC-4 AC-5
 */

// Authoring.runtime_write_spawn_*: in-process runtime spawn through OVStage
// writes only. RuntimeSpawnScaling.collider_heavy_1280_cpu, at the end of this
// file, reuses the same column writes to price the drain of one spawned body
// into a scene that already holds 1,280 static colliders.
//
// This creation path uses no population USD references at all. Instead of
// referencing an authored layer, it writes the exact column layout a
// population-created body ends up with (usd-prim-type, usd-schemas, xforms,
// mass, velocities, and a Cube collider child) straight through
// ovstage_write_attribute in UPSERT mode:
//
//   ovstage_write_attribute(... UPSERT ...)  x13 per body
//   ovstage_advance_write_floor(ordinal)
//   ovphysx_update_from_ovstage([prev+1, ordinal])
//   ovphysx_step_sync(dt)
//
// It uses the same fixture and the same 80 bodies / 5 drain cycles as
// population_add_packed_*, so the two are directly comparable: same result,
// same scale, reference path versus pure write path.
//
// The drip shape (one body per seal/drain) is covered by population_add_drip_*
// rather than duplicated here. Drip creation wedges after about 63 create/drain
// cycles even with a single population reference, so the wall is counted in
// cycles, not references. This row prices the batched write path itself.
//
// The device comes from the scene plus the forceGpu() gate, as in the sibling
// rows. ovphysx_set_cpu_mode is not called.

#include "framework/UsdPCH.h"

#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"
#include "../framework/BmUtils.h"
#include "../BenchmarkFailure.h"
#include "AuthoringCommon.h"

#include <ovphysx/experimental/ovphysx.hpp>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>


void initAuthoringRuntimeWriteSpawn()
{
}


namespace
{

// Matched to population_add_packed_* so the two creation paths are comparable.
const int kAuthoringSpawnCount = 80;
const int kAuthoringSpawnBatch = 16;


// One reusable OVStage write payload. Every write completes before this storage
// is reused. If the bounded wait times out, the final completion wait keeps this
// benchmark-owned storage alive until the operation settles or the outer
// process timeout terminates the run.
struct AuthoringWritePayload
{
    static constexpr uint16_t kF32Capacity = 4;
    float f32[kF32Capacity] = { 0.0f };
    double f64 = 0.0;
    std::vector<uint64_t> tokens;
    int64_t shape = 1;
    DLTensor tensor{};
    ovstage_write_data_t write{};

    void reset()
    {
        std::memset(f32, 0, sizeof(f32));
        f64 = 0.0;
        tokens.clear();
        shape = 1;
        std::memset(&tensor, 0, sizeof(tensor));
        std::memset(&write, 0, sizeof(write));
    }
};
static_assert(AuthoringWritePayload::kF32Capacity >= 4, "spawn writes require one four-lane quaternion payload");


void authoringSpawnPose(int index, float& x, float& y, float& z)
{
    x = static_cast<float>(index % 8) * 1.5f - 6.0f;
    y = 3.0f + static_cast<float>(index / 8) * 1.5f;
    z = static_cast<float>((index / 8) % 8) * 1.5f - 6.0f;
}


// One float column. `lanes` carries the component count (3 for a translate, 4
// for a quaternion, 1 for a scalar).
bool authoringWriteF32(ovstage_instance_t* stage,
                       const char* path,
                       const char* attr,
                       ovstage_ordinal_t ordinal,
                       const float* values,
                       uint16_t lanes,
                       ovstage_attribute_semantic_t semantic,
                       const char* row,
                       AuthoringWritePayload* payload)
{
    if (!payload || !values || lanes == 0 || lanes > AuthoringWritePayload::kF32Capacity)
    {
        bmRecordFailure(row, "invalid float payload lane count: %u (capacity %u)", static_cast<unsigned>(lanes),
                        static_cast<unsigned>(AuthoringWritePayload::kF32Capacity));
        return false;
    }

    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    if (!authoringbm::queryPath(stage, path, &query, row))
    {
        bmRecordFailure(row, "query %s failed", path);
        return false;
    }

    payload->reset();
    for (uint16_t i = 0; i < lanes; ++i)
    {
        payload->f32[i] = values[i];
    }
    payload->tensor.data = payload->f32;
    payload->tensor.device.device_type = kDLCPU;
    payload->tensor.ndim = 1;
    payload->tensor.shape = &payload->shape;
    payload->tensor.dtype.code = kDLFloat;
    payload->tensor.dtype.bits = 32;
    payload->tensor.dtype.lanes = lanes;

    payload->write.tensors = &payload->tensor;
    payload->write.tensor_count = 1;
    payload->write.is_array = false;
    payload->write.semantic = semantic;

    const ovstage_enqueue_result_t enqueue = ovstage_write_attribute(
        stage, query, authoringbm::nameString(attr), ordinal, payload->write, OVSTAGE_PRIM_MODE_UPSERT);
    const bool ok = enqueue.status == OVSTAGE_OK && authoringbm::waitStage(stage, enqueue.op_index, row, attr);
    const bool released = authoringbm::releaseQueryChecked(stage, query, row);
    return ok && released;
}


bool authoringWriteF64Scalar(ovstage_instance_t* stage,
                             const char* path,
                             const char* attr,
                             ovstage_ordinal_t ordinal,
                             double value,
                             const char* row,
                             AuthoringWritePayload* payload)
{
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    if (!authoringbm::queryPath(stage, path, &query, row))
    {
        bmRecordFailure(row, "query %s failed", path);
        return false;
    }

    payload->reset();
    payload->f64 = value;
    payload->tensor.data = &payload->f64;
    payload->tensor.device.device_type = kDLCPU;
    payload->tensor.ndim = 1;
    payload->tensor.shape = &payload->shape;
    payload->tensor.dtype.code = kDLFloat;
    payload->tensor.dtype.bits = 64;
    payload->tensor.dtype.lanes = 1;

    payload->write.tensors = &payload->tensor;
    payload->write.tensor_count = 1;
    payload->write.is_array = false;
    payload->write.semantic = OVSTAGE_SEMANTIC_NONE;

    const ovstage_enqueue_result_t enqueue = ovstage_write_attribute(
        stage, query, authoringbm::nameString(attr), ordinal, payload->write, OVSTAGE_PRIM_MODE_UPSERT);
    const bool ok = enqueue.status == OVSTAGE_OK && authoringbm::waitStage(stage, enqueue.op_index, row, attr);
    const bool released = authoringbm::releaseQueryChecked(stage, query, row);
    return ok && released;
}


// Interned tokens travel as uint64 payloads. A scalar is one element, an array
// is `is_array = true` with one element per token.
bool authoringWriteTokens(ovstage_instance_t* stage,
                          const char* path,
                          const char* attr,
                          ovstage_ordinal_t ordinal,
                          const char* const* tokens,
                          size_t tokenCount,
                          bool isArray,
                          ovstage_attribute_semantic_t semantic,
                          const char* row,
                          AuthoringWritePayload* payload)
{
    path_dictionary_instance_t* dict = ovstage_get_path_dictionary(stage);
    if (!dict)
    {
        return false;
    }

    std::vector<ovx_string_t> strings(tokenCount);
    std::vector<ovx_token_t> interned(tokenCount, 0);
    for (size_t i = 0; i < tokenCount; ++i)
    {
        strings[i] = authoringbm::stringView(tokens[i]);
    }
    if (path_dictionary_create_tokens_from_strings(dict, strings.data(), tokenCount, interned.data()).status !=
        OVX_API_SUCCESS)
    {
        bmRecordFailure(row, "intern tokens for %s failed", attr);
        return false;
    }

    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    if (!authoringbm::queryPath(stage, path, &query, row))
    {
        bmRecordFailure(row, "query %s failed", path);
        return false;
    }

    payload->reset();
    payload->tokens.resize(tokenCount);
    for (size_t i = 0; i < tokenCount; ++i)
    {
        payload->tokens[i] = static_cast<uint64_t>(interned[i]);
    }

    payload->shape = static_cast<int64_t>(tokenCount);
    payload->tensor.data = payload->tokens.data();
    payload->tensor.device.device_type = kDLCPU;
    payload->tensor.ndim = 1;
    payload->tensor.shape = &payload->shape;
    payload->tensor.dtype.code = kDLUInt;
    payload->tensor.dtype.bits = 64;
    payload->tensor.dtype.lanes = 1;

    payload->write.tensors = &payload->tensor;
    payload->write.tensor_count = 1;
    payload->write.is_array = isArray;
    payload->write.semantic = semantic;

    const ovstage_enqueue_result_t enqueue = ovstage_write_attribute(
        stage, query, authoringbm::nameString(attr), ordinal, payload->write, OVSTAGE_PRIM_MODE_UPSERT);
    const bool ok = enqueue.status == OVSTAGE_OK && authoringbm::waitStage(stage, enqueue.op_index, row, attr);
    const bool released = authoringbm::releaseQueryChecked(stage, query, row);
    return ok && released;
}


// The exact column layout a population-created body ends up with, reproduced
// write-for-write.
bool authoringSpawnBody(
    ovstage_instance_t* stage, ovstage_ordinal_t ordinal, int index, const char* row, AuthoringWritePayload* payload)
{
    char bodyPath[96];
    char colliderPath[112];
    std::snprintf(bodyPath, sizeof(bodyPath), "/World/Runtime/Body_%04d", index);
    std::snprintf(colliderPath, sizeof(colliderPath), "%s/Collider", bodyPath);

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    authoringSpawnPose(index, x, y, z);

    const float translate[3] = { x, y, z };
    const float orient[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    const float mass = 1.0f;
    const float scale[3] = { 0.5f, 0.5f, 0.5f };
    const float zero3[3] = { 0.0f, 0.0f, 0.0f };

    const char* const bodySchemas[] = { "PhysicsMassAPI", "PhysicsRigidBodyAPI" };
    const char* const bodyOrder[] = { "xformOp:translate", "xformOp:orient" };
    const char* const colliderSchemas[] = { "PhysicsCollisionAPI" };
    const char* const colliderOrder[] = { "xformOp:scale" };
    const char* const xformType[] = { "Xform" };
    const char* const cubeType[] = { "Cube" };

    bool ok = true;
    ok = ok && authoringWriteTokens(
                   stage, bodyPath, "usd-prim-type", ordinal, xformType, 1, false, OVSTAGE_SEMANTIC_NONE, row, payload);
    ok = ok && authoringWriteTokens(
                   stage, bodyPath, "usd-schemas", ordinal, bodySchemas, 2, true, OVSTAGE_SEMANTIC_NONE, row, payload);
    ok = ok && authoringWriteF32(
                   stage, bodyPath, "xformOp:translate", ordinal, translate, 3, OVSTAGE_SEMANTIC_POINT, row, payload);
    ok = ok && authoringWriteF32(
                   stage, bodyPath, "xformOp:orient", ordinal, orient, 4, OVSTAGE_SEMANTIC_QUATERNION, row, payload);
    ok = ok && authoringWriteTokens(stage, bodyPath, "xformOpOrder", ordinal, bodyOrder, 2, true,
                                    OVSTAGE_SEMANTIC_TOKEN_ID, row, payload);
    ok = ok && authoringWriteF32(stage, bodyPath, "physics:mass", ordinal, &mass, 1, OVSTAGE_SEMANTIC_NONE, row, payload);
    ok = ok && authoringWriteF32(
                   stage, bodyPath, "physics:velocity", ordinal, zero3, 3, OVSTAGE_SEMANTIC_VECTOR, row, payload);
    ok = ok && authoringWriteF32(stage, bodyPath, "physics:angularVelocity", ordinal, zero3, 3, OVSTAGE_SEMANTIC_VECTOR,
                                 row, payload);

    ok = ok && authoringWriteTokens(stage, colliderPath, "usd-prim-type", ordinal, cubeType, 1, false,
                                    OVSTAGE_SEMANTIC_NONE, row, payload);
    ok = ok && authoringWriteTokens(stage, colliderPath, "usd-schemas", ordinal, colliderSchemas, 1, true,
                                    OVSTAGE_SEMANTIC_NONE, row, payload);
    ok = ok && authoringWriteF64Scalar(stage, colliderPath, "size", ordinal, 1.0, row, payload);
    ok = ok &&
         authoringWriteF32(stage, colliderPath, "xformOp:scale", ordinal, scale, 3, OVSTAGE_SEMANTIC_NONE, row, payload);
    ok = ok && authoringWriteTokens(stage, colliderPath, "xformOpOrder", ordinal, colliderOrder, 1, true,
                                    OVSTAGE_SEMANTIC_TOKEN_ID, row, payload);

    if (!ok)
    {
        bmRecordFailure(row, "failed to spawn body %d at %s", index, bodyPath);
    }
    return ok;
}


class AuthoringRuntimeWriteSpawn : public BmBenchmark
{
public:
    AuthoringRuntimeWriteSpawn(const char* rowName, bool gpu) : mRow(rowName), mGpu(gpu)
    {
    }

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
        return 1;
    }

    uint32_t getNbRuns() const override
    {
        return 5;
    }

    void startRun() override
    {
        mPhysX = BmGlobals::getInstance().getPhysX();
        mScenePath = BmGlobals::getInstance().getDataFolder() +
                     (mGpu ? "/empty_dynamic_boxes.usda" : "/empty_dynamic_boxes_cpu.usda");
    }

    // Untimed: every measured step starts from a freshly loaded scene so the
    // spawned bodies do not accumulate across steps.
    void preStep() override
    {
        mSetupOk = false;
        mStepOk = false;
        mSeedBodies = 0;
        mSpawned = 0;
        mOrdinal = 0;
        mLastDrained = 0;
        if (!authoringbm::prepareStageAttachmentForRun(
                mStageAttachment, mRow, &mLeftoverAttachmentReported))
        {
            return;
        }
        mWritePayload.reset();
        if (!mPhysX)
        {
            return;
        }
        mAttached = false;
        authoringbm::GpuFallbackScope gpuScope(mGpu, mRow);
        if (!authoringbm::populateAndAttachChecked(mPhysX, authoringbm::UsdSource::eFile, mScenePath,
                                                   "ovphysx-authoring-runtime-write-spawn", mStageAttachment, mRow,
                                                   &mAttached))
        {
            return;
        }
        if (!authoringbm::checkDeviceEvidenceAndWarmup(mPhysX->handle(), mGpu, gpuScope, mRow))
        {
            return;
        }

        if (!authoringbm::countRigidBodies(mPhysX->handle(), &mSeedBodies, mRow))
        {
            bmRecordFailure(mRow, "seed rigid-body readback failed");
            return;
        }
        mOrdinal = mStageAttachment.ordinal;
        mLastDrained = mStageAttachment.ordinal;
        mStepOk = true;
        mSetupOk = true;
    }

    Time::Second timedStep() override
    {
        Time timer;
        step();
        const Time::Second elapsed = timer.getElapsedSeconds();
        verify();
        return elapsed;
    }

    void endRun() override
    {
        // A Stage ovphysx never took needs the plain destroy, not the reset +
        // detach path: attempting to detach a never-attached Stage reports a
        // failure that misattributes the original fault.
        if (!authoringbm::clearOvstageChecked(mAttached ? mPhysX : nullptr, mStageAttachment, mRow))
        {
            bmRecordFailure(mRow, "ovstage teardown failed; retaining the stage attachment");
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
        if (!mSetupOk)
        {
            return;
        }
        ovstage_instance_t* stage = mStageAttachment.stage;
        const ovphysx_handle_t handle = mPhysX->handle();

        for (int first = 0; first < kAuthoringSpawnCount; first += kAuthoringSpawnBatch)
        {
            int last = first + kAuthoringSpawnBatch;
            if (last > kAuthoringSpawnCount)
            {
                last = kAuthoringSpawnCount;
            }

            ++mOrdinal;
            for (int i = first; i < last; ++i)
            {
                if (!authoringSpawnBody(stage, mOrdinal, i, mRow, &mWritePayload))
                {
                    mStepOk = false;
                    return;
                }
            }

            if (!authoringbm::sealAndDrain(handle, stage, &mLastDrained, mOrdinal, mRow))
            {
                mStepOk = false;
                return;
            }
            mSpawned += (last - first);

            if (ovphysx_step_sync(handle, 1.0f / 60.0f).status != OVPHYSX_API_SUCCESS)
            {
                const ovphysx_string_t err = ovphysx_get_last_error();
                bmRecordFailure(mRow, "step_sync failed: %.*s", static_cast<int>(err.length), err.ptr ? err.ptr : "");
                mStepOk = false;
                return;
            }
        }
    }

private:
    // Fail-closed: every written body must have become a simulating rigid body.
    // A column layout that parses but does not produce an actor would otherwise
    // look like a very fast spawn.
    void verify()
    {
        if (!mSetupOk)
        {
            return;
        }
        if (!mStepOk)
        {
            bmRecordFailure(mRow, "spawn sequence did not complete");
            return;
        }

        uint32_t simulated = 0;
        if (!authoringbm::countRigidBodies(mPhysX->handle(), &simulated, mRow))
        {
            bmRecordFailure(mRow, "rigid-body readback failed");
            return;
        }
        const uint32_t expected = mSeedBodies + static_cast<uint32_t>(mSpawned);
        if (simulated != expected)
        {
            bmRecordFailure(mRow, "expected %u simulated rigid bodies (%u seed + %d written), found %u", expected,
                            mSeedBodies, mSpawned, simulated);
        }
    }

    const char* mRow = nullptr;
    bool mGpu = false;

    ovphysx::PhysX* mPhysX = nullptr;
    ovphysx_sample_stage_attachment_t mStageAttachment{};
    std::string mScenePath;
    bool mAttached = false;
    bool mLeftoverAttachmentReported = false;
    bool mSetupOk = false;
    bool mStepOk = true;
    uint32_t mSeedBodies = 0;
    int mSpawned = 0;
    ovstage_ordinal_t mOrdinal = 0;
    ovstage_ordinal_t mLastDrained = 0;
    AuthoringWritePayload mWritePayload;
};


struct AuthoringRuntimeWriteSpawnCpu : AuthoringRuntimeWriteSpawn
{
    AuthoringRuntimeWriteSpawnCpu() : AuthoringRuntimeWriteSpawn("Authoring.runtime_write_spawn_cpu", false)
    {
    }
};

struct AuthoringRuntimeWriteSpawnGpu : AuthoringRuntimeWriteSpawn
{
    AuthoringRuntimeWriteSpawnGpu() : AuthoringRuntimeWriteSpawn("Authoring.runtime_write_spawn_gpu", true)
    {
    }
};


Register<AuthoringRuntimeWriteSpawnCpu, true> sAuthoringRuntimeWriteSpawnCpu("Authoring.runtime_write_spawn_cpu");
Register<AuthoringRuntimeWriteSpawnGpu, true> sAuthoringRuntimeWriteSpawnGpu("Authoring.runtime_write_spawn_gpu");


// ---------------------------------------------------------------------------
// RuntimeSpawnScaling.collider_heavy_1280_cpu: structural ingestion of one
// runtime-spawned body into a scene that already holds 1,280 static colliders.
// ---------------------------------------------------------------------------
//
// Factory scenes spawn objects into large existing collider sets. This row
// measures how the seal/drain that realizes one new body scales with what is
// already in the scene. It reuses authoringSpawnBody() unchanged and is
// registered outside the `Authoring.*` selection so that it does not widen
// that fixed row set.
//
// Timing boundary:
//
//   outside  the generated 1,280-collider scene (text built once and cached),
//            ovstage populate + attach, ovphysx_warmup(), the seed rigid-body
//            readback, the thirteen waited UPSERT column writes that author the
//            one runtime body (authoringSpawnBody), the one waited omni:xform
//            write that places it, and the correctness gate
//   inside   authoringbm::sealAndDrain (ovstage_advance_write_floor(ordinal)
//            plus its completion wait and
//            ovphysx_update_from_ovstage([prev+1, ordinal])) followed by one
//            ovphysx_step_sync(1/60 s)
//
// The row prices what a front end pays after it has finished authoring one
// object into a populated scene: the seal, the structural drain that realizes
// the new body, and the first step that simulates it. The column writes
// themselves are priced by Authoring.runtime_write_spawn_cpu.
//
// The extra omni:xform write is needed because ovphysx resolves a prim's local
// transform from the ovstage data plane's omni:xform matrix (which the column
// population writes for every xformable prim, see
// OvstageSource::getLocalToWorldTransform), not from the xformOp:* columns the
// spawn layout carries. A body authored through the thirteen spawn columns
// alone realizes at the origin, which for this fixture would sit inside a
// collider and charge the timed step for contact work. The row writes the same
// pose as omni:xform, outside the timer, so the body realizes where
// authoringSpawnPose(0) puts it.
//
// The colliders are static Cube prims carrying PhysicsCollisionAPI and no
// rigid-body schema, on a y = 0 grid the spawned body never touches. The body
// is placed at y = 3 by authoringSpawnPose(0), and the scene authors zero
// gravity, so one step leaves it where it was written. The PhysicsScene authors
// the same CPU dynamics and MBP lines as the CPU Authoring fixture, verified in
// the generated text before population, so the row is CPU by construction.
// Like that fixture it leaves physxScene:timeStepsPerSecond unauthored (the
// schema default, 60 Hz).
//
// Fail-closed gates, all outside the timer. Before population the generated
// text must carry the CPU scene lines, exactly 1,280 collider prims, and no
// rigid-body schema. After attach the scene must hold zero dynamic rigid
// bodies. After the timed drain and step it must hold exactly one rigid body,
// ovphysx_get_object_type() must classify the authored path as a rigid body,
// and the pose readback of that path must return one match at the authored
// height. Together those prove the drain was neither dropped nor deferred past
// the measured step. The public API has no query for realized static shapes,
// so the collider count is established from the authored text rather than
// from a physics-side readback.

const uint32_t kColliderHeavyCount = 1280;
const int kColliderHeavyBodyIndex = 0;
const char* const kColliderHeavyBodyPath = "/World/Runtime/Body_0000";
const float kColliderHeavyPoseTolerance = 0.01f;

// Authored lines the generated scene must carry, emitted from these constants
// and checked against them so a generator edit cannot quietly drop the CPU
// configuration or change the collider count.
const char* const kColliderHeavyScenePrim = "def PhysicsScene \"PhysicsScene\"";
const char* const kColliderHeavySceneApiAuthoring = "prepend apiSchemas = [\"PhysxSceneAPI\"]";
const char* const kColliderHeavyDynamicsAuthoring = "bool physxScene:enableGPUDynamics = false";
const char* const kColliderHeavyBroadphaseAuthoring = "uniform token physxScene:broadphaseType = \"MBP\"";
const char* const kColliderHeavyColliderSchema = "prepend apiSchemas = [\"PhysicsCollisionAPI\"]";


const std::string& makeColliderHeavyScene()
{
    static std::string cachedScene;
    if (!cachedScene.empty())
    {
        return cachedScene;
    }

    const uint32_t side = static_cast<uint32_t>(std::ceil(std::sqrt(static_cast<double>(kColliderHeavyCount))));
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
           << kColliderHeavyScenePrim << " (\n"
           << "        " << kColliderHeavySceneApiAuthoring << "\n"
           << "    )\n"
              "    {\n"
              "        vector3f physics:gravityDirection = (0, -1, 0)\n"
              "        float physics:gravityMagnitude = 0\n"
              "        "
           << kColliderHeavyDynamicsAuthoring << "\n"
           << "        " << kColliderHeavyBroadphaseAuthoring << "\n"
           << "    }\n\n";

    for (uint32_t index = 0; index < kColliderHeavyCount; ++index)
    {
        const uint32_t row = index / side;
        const uint32_t column = index % side;
        stream << "    def Cube \"Collider_" << std::setw(4) << std::setfill('0') << index << "\" (\n"
               << "        " << kColliderHeavyColliderSchema << "\n"
               << "    )\n"
               << "    {\n"
               << "        double size = 0.25\n"
               << "        double3 xformOp:translate = (" << static_cast<double>(column) * 0.5 << ", 0, "
               << static_cast<double>(row) * 0.5 << ")\n"
               << "        uniform token[] xformOpOrder = [\"xformOp:translate\"]\n"
               << "    }\n\n";
    }
    stream << "}\n";
    cachedScene = stream.str();
    return cachedScene;
}


size_t countOccurrences(const std::string& text, const char* needle)
{
    const std::string pattern(needle);
    size_t count = 0;
    for (size_t at = text.find(pattern); at != std::string::npos; at = text.find(pattern, at + pattern.size()))
    {
        ++count;
    }
    return count;
}


// Object-owned payload for the one omni:xform write, shaped like the teleport
// row's: a row-major double[16] local matrix with the translation in the last
// row, which is what the column population writes for every xformable prim.
struct ColliderHeavyXformPayload
{
    double matrix[16] = { 0.0 };
    int64_t shape = 1;
    DLTensor tensor{};
    ovstage_write_data_t write{};
};


bool colliderHeavyWriteOmniXform(ovstage_instance_t* stage,
                                 const char* path,
                                 ovstage_ordinal_t ordinal,
                                 float x,
                                 float y,
                                 float z,
                                 const char* row,
                                 ColliderHeavyXformPayload* payload)
{
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    if (!authoringbm::queryPath(stage, path, &query, row))
    {
        bmRecordFailure(row, "query %s failed", path);
        return false;
    }

    std::memset(payload->matrix, 0, sizeof(payload->matrix));
    payload->matrix[0] = 1.0;
    payload->matrix[5] = 1.0;
    payload->matrix[10] = 1.0;
    payload->matrix[12] = static_cast<double>(x);
    payload->matrix[13] = static_cast<double>(y);
    payload->matrix[14] = static_cast<double>(z);
    payload->matrix[15] = 1.0;
    payload->shape = 1;
    std::memset(&payload->tensor, 0, sizeof(payload->tensor));
    payload->tensor.data = payload->matrix;
    payload->tensor.device.device_type = kDLCPU;
    payload->tensor.ndim = 1;
    payload->tensor.shape = &payload->shape;
    payload->tensor.dtype.code = kDLFloat;
    payload->tensor.dtype.bits = 64;
    payload->tensor.dtype.lanes = 16;
    std::memset(&payload->write, 0, sizeof(payload->write));
    payload->write.tensors = &payload->tensor;
    payload->write.tensor_count = 1;
    payload->write.is_array = false;
    payload->write.semantic = OVSTAGE_SEMANTIC_NONE;

    const ovstage_enqueue_result_t enqueue = ovstage_write_attribute(
        stage, query, authoringbm::nameString("omni:xform"), ordinal, payload->write, OVSTAGE_PRIM_MODE_UPSERT);
    const bool ok = enqueue.status == OVSTAGE_OK && authoringbm::waitStage(stage, enqueue.op_index, row, "omni:xform");
    const bool released = authoringbm::releaseQueryChecked(stage, query, row);
    if (!ok)
    {
        bmRecordFailure(row, "omni:xform write for %s did not complete", path);
    }
    return ok && released;
}


// Records the first defect it finds against the row and returns false.
bool colliderHeavySceneIsWellFormed(const std::string& usda, const char* row)
{
    const size_t sceneStart = usda.find(kColliderHeavyScenePrim);
    if (sceneStart == std::string::npos)
    {
        bmRecordFailure(row, "generated collider scene has no %s block", kColliderHeavyScenePrim);
        return false;
    }
    const size_t firstCollider = usda.find("def Cube", sceneStart);
    const std::string sceneBlock =
        usda.substr(sceneStart, firstCollider == std::string::npos ? std::string::npos : firstCollider - sceneStart);
    if (sceneBlock.find(kColliderHeavySceneApiAuthoring) == std::string::npos ||
        sceneBlock.find(kColliderHeavyDynamicsAuthoring) == std::string::npos ||
        sceneBlock.find(kColliderHeavyBroadphaseAuthoring) == std::string::npos)
    {
        bmRecordFailure(row,
                        "generated collider scene does not author the CPU configuration on its %s block "
                        "(expected '%s', '%s' and '%s')",
                        kColliderHeavyScenePrim, kColliderHeavySceneApiAuthoring, kColliderHeavyDynamicsAuthoring,
                        kColliderHeavyBroadphaseAuthoring);
        return false;
    }
    const size_t colliders = countOccurrences(usda, kColliderHeavyColliderSchema);
    if (colliders != kColliderHeavyCount)
    {
        bmRecordFailure(row, "generated collider scene authors %zu collider prims; expected exactly %u", colliders,
                        kColliderHeavyCount);
        return false;
    }
    if (usda.find("PhysicsRigidBodyAPI") != std::string::npos)
    {
        bmRecordFailure(
            row, "generated collider scene authors a rigid body; the only rigid body must be the spawned one");
        return false;
    }
    return true;
}


class RuntimeSpawnScalingColliderHeavy : public BmBenchmark
{
public:
    explicit RuntimeSpawnScalingColliderHeavy(const char* rowName) : mRow(rowName)
    {
    }

    // CPU-only: the generated scene authors CPU dynamics, so the row belongs to
    // the default pass and is skipped under --forceGpu, like WriteScaling.
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
        return 1;
    }

    uint32_t getNbRuns() const override
    {
        return 5;
    }

    void startRun() override
    {
        mPhysX = BmGlobals::getInstance().getPhysX();
    }

    // Untimed: every measured step starts from a freshly populated collider
    // scene with the one runtime body already authored and every column write
    // complete, so the timer sees only the seal/drain and the first step.
    void preStep() override
    {
        mSetupOk = false;
        mStepOk = false;
        mSeedBodies = 0;
        mOrdinal = 0;
        mLastDrained = 0;
        if (!authoringbm::prepareStageAttachmentForRun(mStageAttachment, mRow, &mLeftoverAttachmentReported))
        {
            return;
        }
        mWritePayload.reset();
        if (!mPhysX)
        {
            bmRecordFailure(mRow, "no ovphysx instance available");
            return;
        }

        const std::string& usda = makeColliderHeavyScene();
        if (!colliderHeavySceneIsWellFormed(usda, mRow))
        {
            return;
        }

        mAttached = false;
        authoringbm::GpuFallbackScope gpuScope(false, mRow);
        if (!authoringbm::populateAndAttachChecked(mPhysX, authoringbm::UsdSource::eString, usda,
                                                   "ovphysx-runtime-spawn-scaling", mStageAttachment, mRow, &mAttached))
        {
            return;
        }
        if (!authoringbm::checkDeviceEvidenceAndWarmup(mPhysX->handle(), /*gpu=*/false, gpuScope, mRow))
        {
            return;
        }

        if (!authoringbm::countRigidBodies(mPhysX->handle(), &mSeedBodies, mRow))
        {
            bmRecordFailure(mRow, "seed rigid-body readback failed");
            return;
        }
        if (mSeedBodies != 0)
        {
            bmRecordFailure(
                mRow, "collider scene realized %u dynamic rigid bodies before the spawn; expected none", mSeedBodies);
            return;
        }

        // Ordinals continue from where the attach left the stage, as in the
        // runtime-write-spawn row above.
        mLastDrained = mStageAttachment.ordinal;
        mOrdinal = mStageAttachment.ordinal + 1;
        if (!authoringSpawnBody(mStageAttachment.stage, mOrdinal, kColliderHeavyBodyIndex, mRow, &mWritePayload))
        {
            return;
        }
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        authoringSpawnPose(kColliderHeavyBodyIndex, x, y, z);
        mExpectedY = y;
        if (!colliderHeavyWriteOmniXform(
                mStageAttachment.stage, kColliderHeavyBodyPath, mOrdinal, x, y, z, mRow, &mXformPayload))
        {
            return;
        }
        mSetupOk = true;
        mStepOk = true;
    }

    Time::Second timedStep() override
    {
        Time timer;
        step();
        const Time::Second elapsed = timer.getElapsedSeconds();
        verify();
        return elapsed;
    }

    void endRun() override
    {
        // A Stage ovphysx never took needs the plain destroy, not the reset +
        // detach path: attempting to detach a never-attached Stage reports a
        // failure that misattributes the original fault.
        if (!authoringbm::clearOvstageChecked(mAttached ? mPhysX : nullptr, mStageAttachment, mRow))
        {
            bmRecordFailure(mRow, "ovstage teardown failed; retaining the stage attachment");
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
        if (!mSetupOk)
        {
            return;
        }
        const ovphysx_handle_t handle = mPhysX->handle();
        if (!authoringbm::sealAndDrain(handle, mStageAttachment.stage, &mLastDrained, mOrdinal, mRow))
        {
            mStepOk = false;
            return;
        }
        if (ovphysx_step_sync(handle, 1.0f / 60.0f).status != OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t err = ovphysx_get_last_error();
            bmRecordFailure(mRow, "step_sync failed: %.*s", static_cast<int>(err.length), err.ptr ? err.ptr : "");
            mStepOk = false;
            return;
        }
    }

private:
    // Fail-closed: the drained body must be simulating at its exact authored
    // path. A drain that is dropped or deferred past the measured step would
    // otherwise look like a very fast ingestion.
    void verify()
    {
        if (!mSetupOk)
        {
            return;
        }
        if (!mStepOk)
        {
            bmRecordFailure(mRow, "seal/drain/step sequence did not complete");
            return;
        }

        const ovphysx_handle_t handle = mPhysX->handle();
        uint32_t simulated = 0;
        if (!authoringbm::countRigidBodies(handle, &simulated, mRow))
        {
            bmRecordFailure(mRow, "rigid-body readback failed");
            return;
        }
        if (simulated != 1)
        {
            bmRecordFailure(mRow, "expected exactly 1 simulated rigid body after the drained spawn, found %u", simulated);
            return;
        }

        ovphysx_object_type_t type = OVPHYSX_OBJECT_TYPE_INVALID;
        const ovphysx_result_t typed = ovphysx_get_object_type(handle, ovphysx_cstr(kColliderHeavyBodyPath), &type);
        if (typed.status != OVPHYSX_API_SUCCESS || type != OVPHYSX_OBJECT_TYPE_RIGID_BODY)
        {
            bmRecordFailure(mRow, "%s is not a realized rigid body after the drained spawn (status=%d, type=%d)",
                            kColliderHeavyBodyPath, static_cast<int>(typed.status), static_cast<int>(type));
            return;
        }

        int64_t matches = 0;
        float y = 0.0f;
        if (!authoringbm::readBodyPoseAt(handle, kColliderHeavyBodyPath, &matches, &y, mRow) || matches != 1)
        {
            bmRecordFailure(mRow, "pose readback of %s failed or matched %lld bodies; expected exactly 1",
                            kColliderHeavyBodyPath, static_cast<long long>(matches));
            return;
        }
        if (std::fabs(y - mExpectedY) > kColliderHeavyPoseTolerance)
        {
            bmRecordFailure(mRow, "%s did not keep its authored height (expected y ~= %.3f, got %.3f)",
                            kColliderHeavyBodyPath, static_cast<double>(mExpectedY), static_cast<double>(y));
        }
    }

    const char* mRow = nullptr;

    ovphysx::PhysX* mPhysX = nullptr;
    ovphysx_sample_stage_attachment_t mStageAttachment{};
    bool mAttached = false;
    bool mLeftoverAttachmentReported = false;
    bool mSetupOk = false;
    bool mStepOk = false;
    uint32_t mSeedBodies = 0;
    float mExpectedY = 0.0f;
    ovstage_ordinal_t mOrdinal = 0;
    ovstage_ordinal_t mLastDrained = 0;
    AuthoringWritePayload mWritePayload;
    ColliderHeavyXformPayload mXformPayload;
};


struct RuntimeSpawnScalingColliderHeavy1280Cpu : RuntimeSpawnScalingColliderHeavy
{
    RuntimeSpawnScalingColliderHeavy1280Cpu()
        : RuntimeSpawnScalingColliderHeavy("RuntimeSpawnScaling.collider_heavy_1280_cpu")
    {
    }
};


Register<RuntimeSpawnScalingColliderHeavy1280Cpu, true> sRuntimeSpawnScalingColliderHeavy1280Cpu(
    "RuntimeSpawnScaling.collider_heavy_1280_cpu");

} // namespace
