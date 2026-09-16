// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-BENCHMARK-001
 * @covers AC-1 AC-4 AC-5
 */

// Authoring.population_churn_* in-process object lifecycle: create a dynamic
// rigid body while ovphysx stays attached, then remove it again.
//
// One measured step is one complete create-then-remove cycle:
//
//   create   population_add_usd_reference_from_string -> wait
//            population_apply_usd_changes(N)          -> wait
//            advance_write_floor(N) -> wait -> update_from_ovstage
//   remove   population_remove_usd_reference          -> wait
//            population_apply_usd_changes(N+1)        -> wait
//            delete_attributes(prim, N+1)             -> wait
//            advance_write_floor(N+1) -> wait -> update_from_ovstage
//   step     ovphysx_step_sync
//
// This path uses population add/remove. It never calls
// OVSTAGE_PRIM_MODE_UPSERT, so "population_churn" describes the timed work.
//
// This is the row that covers churn rather than growth. population_add_* only
// ever adds. A front end that spawns and retires objects exercises the remove
// half too, and a leak there shows up as a body count that never returns to
// its floor rather than as a slow row.
//
// The prim path cycles over box_0..box_7 (i % 8), so a stale-handle regression
// on reused paths surfaces here.
//
// Device comes from the scene plus the forceGpu() gate, as in the sibling rows.
// The CPU row is canonical. The GPU-requested row is retained as a hidden,
// unscheduled diagnostic whose fallback detector provides negative evidence
// only.

#include "framework/UsdPCH.h"

#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"
#include "../framework/BmUtils.h"
#include "../BenchmarkFailure.h"
#include "AuthoringCommon.h"

#include <ovphysx/experimental/ovphysx.hpp>

#include <cstdio>
#include <cstring>
#include <string>


void initAuthoringPopulationChurn()
{
}


namespace
{

const char* const kAuthoringChurnParent = "/World/DynamicBoxes";

// Distinct height used by the post-timing gate, clear of the per-iteration band.
const float kAuthoringChurnVerifyY = 4.25f;


std::string authoringFormatDouble(double value)
{
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.9g", value);
    return std::string(buffer);
}


// One body as its own referenced layer, with a non-uniform collider scale.
std::string authoringMakeBodyUsda(float x, float y, float z)
{
    std::string usda;
    usda += "#usda 1.0\n";
    usda += "(defaultPrim = \"Body\")\n";
    usda += "def Xform \"Body\" (prepend apiSchemas = [\"PhysicsRigidBodyAPI\", \"PhysicsMassAPI\"])\n";
    usda += "{\n";
    usda += "    double3 xformOp:translate = (" + authoringFormatDouble(x) + ", " + authoringFormatDouble(y) + ", " +
            authoringFormatDouble(z) + ")\n";
    usda += "    quatf xformOp:orient = (1, 0, 0, 0)\n";
    usda += "    uniform token[] xformOpOrder = [\"xformOp:translate\", \"xformOp:orient\"]\n";
    usda += "    float physics:mass = 1.0\n";
    usda += "    def Cube \"Collider\" (prepend apiSchemas = [\"PhysicsCollisionAPI\"])\n";
    usda += "    {\n";
    usda += "        double size = 1.0\n";
    usda += "        double3 xformOp:scale = (0.4, 0.3, 0.25)\n";
    usda += "        uniform token[] xformOpOrder = [\"xformOp:scale\"]\n";
    usda += "    }\n";
    usda += "}\n";
    return usda;
}


class AuthoringPopulationChurn : public BmBenchmark
{
public:
    AuthoringPopulationChurn(const char* rowName, bool gpu) : mRow(rowName), mGpu(gpu)
    {
    }

    // Device is authored per scene, so each row belongs to exactly one pass.
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
        return 25;
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
        mBaselineBodies = 0;
        mOrdinal = 0;
        mLastDrained = 0;
        mVerifyRef = 0;
        if (!authoringbm::prepareStageAttachmentForRun(mStageAttachment, mRow, &mLeftoverAttachmentReported))
        {
            return;
        }
        if (!mPhysX)
        {
            return;
        }

        // Capture must be live across the attach. The CPU-fallback diagnostic
        // is emitted while the scene is being realized, not afterwards.
        mAttached = false;
        authoringbm::GpuFallbackScope gpuScope(mGpu, mRow);

        const std::string path = BmGlobals::getInstance().getDataFolder() +
                                 (mGpu ? "/empty_dynamic_boxes.usda" : "/empty_dynamic_boxes_cpu.usda");
        if (!authoringbm::populateAndAttachChecked(mPhysX, authoringbm::UsdSource::eFile, path,
                                                   "ovphysx-authoring-population-churn", mStageAttachment, mRow,
                                                   &mAttached))
        {
            return;
        }
        if (!authoringbm::checkDeviceEvidenceAndWarmup(mPhysX->handle(), mGpu, gpuScope, mRow))
        {
            return;
        }

        // The scene's own bodies (the ground plane) are the floor the body
        // count must return to after every remove.
        if (!authoringbm::countRigidBodies(mPhysX->handle(), &mBaselineBodies, mRow))
        {
            bmRecordFailure(mRow, "baseline rigid-body readback failed");
            return;
        }

        mOrdinal = mStageAttachment.ordinal;
        mLastDrained = mStageAttachment.ordinal;
        mStepOk = true;
        mSetupOk = true;
    }

    void preStep() override
    {
    }

    Time::Second timedStep() override
    {
        Time timer;
        step();
        const Time::Second elapsed = timer.getElapsedSeconds();
        return elapsed;
    }

    void endRun() override
    {
        if (mSetupOk && mStepOk)
        {
            verify();
        }
        else if (mSetupOk && !mStepOk)
        {
            bmRecordFailure(mRow, "lifecycle sequence did not complete");
        }
        // A Stage ovphysx never took needs the plain destroy, not the reset and
        // detach path. Attempting to detach a never-attached Stage reports a
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
        if (!mSetupOk || !mStepOk)
        {
            return;
        }
        const float y = 1.0f + 0.1f * static_cast<float>((mIteration % 5) + 1);
        if (!cycleOnce(mIteration % 8, y))
        {
            mStepOk = false;
        }
        ++mIteration;
    }

private:
    // One create-then-remove cycle at the given slot.
    bool cycleOnce(int slot, float y)
    {
        ovstage_instance_t* stage = mStageAttachment.stage;
        const ovphysx_handle_t handle = mPhysX->handle();

        const std::string usda = authoringMakeBodyUsda(0.0f, y, 0.0f);
        const std::string target = std::string(kAuthoringChurnParent) + "/box_" + std::to_string(slot);

        // --- create ---
        ++mOrdinal;
        ovstage_population_usd_reference_handle_t reference = 0;
        const ovstage_population_enqueue_result_t add = ovstage_population_add_usd_reference_from_string(
            stage, authoringbm::stringView(usda), authoringbm::stringView(target), &reference);
        if (add.status != OVSTAGE_OK)
        {
            bmRecordFailure(mRow, "add_usd_reference enqueue rejected: %d", static_cast<int>(add.status));
            return false;
        }
        if (!authoringbm::waitPopulation(stage, add.op_index, mRow, "add_usd_reference"))
        {
            return false;
        }

        const ovstage_population_enqueue_result_t apply = ovstage_population_apply_usd_changes(stage, mOrdinal);
        if (apply.status != OVSTAGE_OK)
        {
            bmRecordFailure(mRow, "apply_usd_changes(create) rejected: %d", static_cast<int>(apply.status));
            return false;
        }
        if (!authoringbm::waitPopulation(stage, apply.op_index, mRow, "apply_usd_changes(create)"))
        {
            return false;
        }
        if (!authoringbm::sealAndDrain(handle, stage, &mLastDrained, mOrdinal, mRow))
        {
            return false;
        }

        // --- remove ---
        ++mOrdinal;
        if (reference != 0)
        {
            const ovstage_population_enqueue_result_t remove = ovstage_population_remove_usd_reference(stage, reference);
            if (remove.status != OVSTAGE_OK)
            {
                bmRecordFailure(mRow, "remove_usd_reference enqueue rejected: %d", static_cast<int>(remove.status));
                return false;
            }
            if (!authoringbm::waitPopulation(stage, remove.op_index, mRow, "remove_usd_reference"))
            {
                return false;
            }

            const ovstage_population_enqueue_result_t applyRemove = ovstage_population_apply_usd_changes(stage, mOrdinal);
            if (applyRemove.status != OVSTAGE_OK)
            {
                bmRecordFailure(mRow, "apply_usd_changes(remove) rejected: %d", static_cast<int>(applyRemove.status));
                return false;
            }
            if (!authoringbm::waitPopulation(stage, applyRemove.op_index, mRow, "apply_usd_changes(remove)"))
            {
                return false;
            }
        }

        // The prim may already be gone after the population remove and apply.
        // The query then legitimately fails and there is nothing to delete,
        // which counts as success.
        ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
        if (authoringbm::queryPath(stage, target.c_str(), &query, mRow))
        {
            const ovstage_enqueue_result_t del = ovstage_delete_attributes(stage, query, nullptr, 0, mOrdinal);
            if (del.status != OVSTAGE_OK)
            {
                bmRecordFailure(mRow, "delete_attributes enqueue rejected: %d", static_cast<int>(del.status));
                authoringbm::releaseQueryChecked(stage, query, mRow);
                return false;
            }
            if (!authoringbm::waitStage(stage, del.op_index, mRow, "delete_attributes"))
            {
                authoringbm::releaseQueryChecked(stage, query, mRow);
                return false;
            }
            authoringbm::releaseQueryChecked(stage, query, mRow);
        }

        if (!authoringbm::sealAndDrain(handle, stage, &mLastDrained, mOrdinal, mRow))
        {
            return false;
        }

        if (ovphysx_step_sync(handle, 1.0f / 60.0f).status != OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t err = ovphysx_get_last_error();
            bmRecordFailure(mRow, "step_sync failed: %.*s", static_cast<int>(err.length), err.ptr ? err.ptr : "");
            return false;
        }
        return true;
    }

    // Fail-closed gate, outside the timed window. After a run of create/remove
    // cycles the population must have returned to its baseline (no leaked
    // actors), and a fresh create must still produce a simulating body at the
    // intended height.
    void verify()
    {
        uint32_t settled = 0;
        if (!authoringbm::countRigidBodies(mPhysX->handle(), &settled, mRow))
        {
            bmRecordFailure(mRow, "rigid-body readback failed");
            return;
        }
        if (settled != mBaselineBodies)
        {
            bmRecordFailure(mRow, "%u rigid bodies after remove cycles, expected baseline %u", settled, mBaselineBodies);
            return;
        }

        // A fresh create must still land, and land where it was authored.
        if (!cycleCreateOnlyForVerify())
        {
            bmRecordFailure(mRow, "post-churn create did not complete");
            return;
        }

        int64_t count = 0;
        float y = 0.0f;
        const std::string target = std::string(kAuthoringChurnParent) + "/box_0";
        if (!authoringbm::readBodyPoseAt(mPhysX->handle(), target.c_str(), &count, &y, mRow))
        {
            bmRecordFailure(mRow, "verify pose readback failed");
            return;
        }
        if (count != 1)
        {
            bmRecordFailure(
                mRow, "expected 1 body at %s after create, found %lld", target.c_str(), static_cast<long long>(count));
            return;
        }
        // One settling step from rest drops only ~1.4 mm.
        if (y < kAuthoringChurnVerifyY - 0.05f || y > kAuthoringChurnVerifyY + 0.05f)
        {
            bmRecordFailure(mRow, "created body at y = %.3f, expected ~%.3f", static_cast<double>(y),
                            static_cast<double>(kAuthoringChurnVerifyY));
            return;
        }

        // Remove the actor just proven live and require zero at that exact path.
        // The aggregate baseline check above is not equivalent. Paired
        // create/remove no-ops preserve the baseline, and an identity or scope
        // bug can remove a different actor while the total still matches.
        if (!removeVerifyActor())
        {
            bmRecordFailure(mRow, "post-verify removal did not complete");
            return;
        }

        int64_t goneCount = -1;
        float goneY = 0.0f;
        if (!authoringbm::readBodyPoseAt(mPhysX->handle(), target.c_str(), &goneCount, &goneY, mRow))
        {
            bmRecordFailure(mRow, "post-removal readback failed");
            return;
        }
        if (goneCount != 0)
        {
            bmRecordFailure(mRow, "expected 0 bodies at %s after removal, found %lld", target.c_str(),
                            static_cast<long long>(goneCount));
        }
    }

    // Remove half of the gate. Drops the verification reference, applies,
    // deletes the prim's attributes, then seals and drains so the topology
    // change reaches physics before the post-removal read.
    bool removeVerifyActor()
    {
        ovstage_instance_t* stage = mStageAttachment.stage;
        const ovphysx_handle_t handle = mPhysX->handle();
        const std::string target = std::string(kAuthoringChurnParent) + "/box_0";

        ++mOrdinal;
        if (mVerifyRef != 0)
        {
            const ovstage_population_enqueue_result_t remove = ovstage_population_remove_usd_reference(stage, mVerifyRef);
            if (remove.status != OVSTAGE_OK ||
                !authoringbm::waitPopulation(stage, remove.op_index, mRow, "remove_usd_reference(verify)"))
            {
                return false;
            }
            const ovstage_population_enqueue_result_t apply = ovstage_population_apply_usd_changes(stage, mOrdinal);
            if (apply.status != OVSTAGE_OK ||
                !authoringbm::waitPopulation(stage, apply.op_index, mRow, "apply_usd_changes(verify remove)"))
            {
                return false;
            }
            mVerifyRef = 0;
        }

        ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
        if (authoringbm::queryPath(stage, target.c_str(), &query, mRow))
        {
            const ovstage_enqueue_result_t del = ovstage_delete_attributes(stage, query, nullptr, 0, mOrdinal);
            const bool ok = del.status == OVSTAGE_OK &&
                            authoringbm::waitStage(stage, del.op_index, mRow, "delete_attributes(verify)");
            authoringbm::releaseQueryChecked(stage, query, mRow);
            if (!ok)
            {
                return false;
            }
        }

        return authoringbm::sealAndDrain(handle, stage, &mLastDrained, mOrdinal, mRow) &&
               ovphysx_step_sync(handle, 1.0f / 60.0f).status == OVPHYSX_API_SUCCESS;
    }

    // Create half only, used by the gate to prove the path still works.
    bool cycleCreateOnlyForVerify()
    {
        ovstage_instance_t* stage = mStageAttachment.stage;
        const ovphysx_handle_t handle = mPhysX->handle();

        const std::string usda = authoringMakeBodyUsda(0.0f, kAuthoringChurnVerifyY, 0.0f);
        const std::string target = std::string(kAuthoringChurnParent) + "/box_0";

        ++mOrdinal;
        mVerifyRef = 0;
        const ovstage_population_enqueue_result_t add = ovstage_population_add_usd_reference_from_string(
            stage, authoringbm::stringView(usda), authoringbm::stringView(target), &mVerifyRef);
        if (add.status != OVSTAGE_OK ||
            !authoringbm::waitPopulation(stage, add.op_index, mRow, "add_usd_reference(verify)"))
        {
            return false;
        }
        const ovstage_population_enqueue_result_t apply = ovstage_population_apply_usd_changes(stage, mOrdinal);
        if (apply.status != OVSTAGE_OK ||
            !authoringbm::waitPopulation(stage, apply.op_index, mRow, "apply_usd_changes(verify)"))
        {
            return false;
        }
        if (!authoringbm::sealAndDrain(handle, stage, &mLastDrained, mOrdinal, mRow))
        {
            return false;
        }
        return ovphysx_step_sync(handle, 1.0f / 60.0f).status == OVPHYSX_API_SUCCESS;
    }

    const char* mRow = nullptr;
    bool mGpu = false;

    ovphysx::PhysX* mPhysX = nullptr;
    ovphysx_sample_stage_attachment_t mStageAttachment{};
    bool mAttached = false;
    bool mLeftoverAttachmentReported = false;
    bool mSetupOk = false;
    bool mStepOk = true;
    int mIteration = 0;
    uint32_t mBaselineBodies = 0;
    ovstage_ordinal_t mOrdinal = 0;
    ovstage_ordinal_t mLastDrained = 0;
    ovstage_population_usd_reference_handle_t mVerifyRef = 0;
};

struct AuthoringPopulationChurnCpu : AuthoringPopulationChurn
{
    AuthoringPopulationChurnCpu() : AuthoringPopulationChurn("Authoring.population_churn_cpu", false)
    {
    }
};

struct AuthoringPopulationChurnGpu : AuthoringPopulationChurn
{
    AuthoringPopulationChurnGpu() : AuthoringPopulationChurn("Authoring.population_churn_gpu", true)
    {
    }
};


Register<AuthoringPopulationChurnCpu, true> sAuthoringPopulationChurnCpu("Authoring.population_churn_cpu");
Register<AuthoringPopulationChurnGpu, true> sAuthoringPopulationChurnGpu("Authoring.population_churn_gpu");

} // namespace
