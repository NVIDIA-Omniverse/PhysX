// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-BENCHMARK-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 */

// Authoring.* in-process object-creation benchmarks: add new dynamic rigid
// bodies while ovphysx stays attached.
//
// Operation sequence per population reference:
//
//   ovstage_population_add_usd_reference_from_string  (author the new prim)
//   ovstage_population_wait_op                        (retain the result)
//   ovstage_population_apply_usd_changes(ordinal)     (apply the USD changes)
//   ovstage_advance_write_floor(ordinal)              (seal the write floor)
//   ovphysx_update_from_ovstage([prev+1, ordinal])    (drain into physics)
//   ovphysx_step_sync(dt)                             (simulate)
//
// The registered rows differ only in how many bodies share one population
// reference:
//
//   population_add_packed_cpu  80 bodies, 16 per reference ->  5 references
//   population_add_drip_cpu    80 bodies,  1 per reference -> 80 references
//
// The drip row is the shape a front end that spawns one object per tick
// produces. Before OMPE-104209 it wedged inside ovphysx_step_sync after roughly
// 63 create/drain cycles. 80 cycles runs deliberately past that wall so a
// regression shows up here rather than at a customer.
//
// Device selection uses the harness model, authored per scene through
// physxScene:enableGPUDynamics and gated by forceGpu().
// ovphysx_set_cpu_mode() is process-wide and cannot be reverted within the
// process, so a single call would poison every other row in this binary.
// Scene load happens in preStep() and the correctness gate runs after the
// timer stops, so neither is inside the measured window.

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


void initAuthoringPopulationAdd()
{
}


namespace
{

// Total bodies added per measured step, shared by both rows so the packed and
// drip numbers are directly comparable. 80 is an exact multiple of the pack
// size (5 references) and is comfortably past the ~63 create/drain wall
// described in the file comment.
const int kAuthoringBodyCount = 80;
const int kAuthoringPackedPack = 16;
const int kAuthoringDripPack = 1;

// Steps simulated after each reference is drained.
const int kAuthoringStepsBetween = 1;

const char* const kAuthoringParentPath = "/World/DynamicBoxes";

// The collider block every authored body carries.
const char* const kAuthoringColliderUsda =
    "        def Cube \"Collider\" (prepend apiSchemas = [\"PhysicsCollisionAPI\"])\n"
    "        {\n"
    "            double size = 1.0\n"
    "            double3 xformOp:scale = (0.5, 0.5, 0.5)\n"
    "            uniform token[] xformOpOrder = [\"xformOp:scale\"]\n"
    "        }\n";


// Deterministic lattice for the authored poses.
void authoringBodyPose(int index, double& x, double& y, double& z)
{
    x = static_cast<double>(index % 8) * 1.5 - 6.0;
    y = 3.0 + static_cast<double>(index / 8) * 1.5;
    z = static_cast<double>((index / 8) % 8) * 1.5 - 6.0;
}


// Formats one double with "%.9g", without a caller-supplied buffer bound.
std::string authoringFormatDouble(double value)
{
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.9g", value);
    return std::string(buffer);
}


// One body as its own referenced layer, targeted at an absolute prim path.
std::string authoringAuthorSingle(int index)
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    authoringBodyPose(index, x, y, z);

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
    usda += kAuthoringColliderUsda;
    usda += "}\n";
    return usda;
}


// The packed form: several sibling bodies in one referenced layer, targeted at
// their shared parent, so N bodies cost a single population reference.
std::string authoringAuthorPacked(int first, int last)
{
    std::string usda;
    usda += "#usda 1.0\n(defaultPrim = \"Bodies\")\ndef Scope \"Bodies\"\n{\n";
    for (int i = first; i < last; ++i)
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        authoringBodyPose(i, x, y, z);

        usda += "    def Xform \"box_" + std::to_string(i) +
                "\" (prepend apiSchemas = [\"PhysicsRigidBodyAPI\", \"PhysicsMassAPI\"])\n";
        usda += "    {\n";
        usda += "        double3 xformOp:translate = (" + authoringFormatDouble(x) + ", " + authoringFormatDouble(y) +
                ", " + authoringFormatDouble(z) + ")\n";
        usda += "        quatf xformOp:orient = (1, 0, 0, 0)\n";
        usda += "        uniform token[] xformOpOrder = [\"xformOp:translate\", \"xformOp:orient\"]\n";
        usda += "        float physics:mass = 1.0\n";
        usda += kAuthoringColliderUsda;
        usda += "    }\n";
    }
    usda += "}\n";
    return usda;
}


// ---------------------------------------------------------------------------
// Shared base for the create rows.
// ---------------------------------------------------------------------------

class AuthoringPopulationAddBase : public BmBenchmark
{
public:
    AuthoringPopulationAddBase(const char* rowName, int pack, bool gpu) : mRowName(rowName), mPack(pack), mGpu(gpu)
    {
    }

    // Device is authored per scene, so each row belongs to exactly one pass.
    // empty_dynamic_boxes.usda leaves PhysxSceneAPI unset and so defaults to GPU
    // dynamics, while the _cpu overlay declares the CPU pipeline. Also gate on
    // PhysX bootstrap so step() never dereferences a null instance.
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

    // Untimed. Every measured step starts from a freshly loaded scene, so the
    // body count and the reference count do not accumulate across steps.
    void preStep() override
    {
        mSetupOk = false;
        mStepOk = false;
        mOrdinal = 0;
        mLastDrained = 0;
        mSeedBodies = 0;
        mAddedBodies = 0;

        if (!authoringbm::prepareStageAttachmentForRun(
                mStageAttachment, mRowName, &mLeftoverAttachmentReported))
        {
            return;
        }
        if (!mPhysX)
        {
            return;
        }
        mAttached = false;
        authoringbm::GpuFallbackScope gpuScope(mGpu, mRowName);
        if (!authoringbm::populateAndAttachChecked(mPhysX, authoringbm::UsdSource::eFile, mScenePath,
                                                   "ovphysx-authoring-population-add", mStageAttachment, mRowName,
                                                   &mAttached))
        {
            return;
        }
        if (!authoringbm::checkDeviceEvidenceAndWarmup(mPhysX->handle(), mGpu, gpuScope, mRowName))
        {
            return;
        }

        // The seed scene contributes its own rigid bodies (the ground plane),
        // so the gate compares against seed plus spawned rather than a
        // hardcoded total. Read once per step, outside the timed window.
        if (!authoringbm::countRigidBodies(mPhysX->handle(), &mSeedBodies, mRowName))
        {
            bmRecordFailure(mRowName, "seed rigid-body readback failed");
            return;
        }

        mSetupOk = true;
        mStepOk = true;
        // Ordinals continue from wherever the attach left the stage. Starting
        // from zero would re-target ordinals the stage has already passed and
        // apply_usd_changes then rejects the batch.
        mOrdinal = mStageAttachment.ordinal;
        mLastDrained = mStageAttachment.ordinal;
        mAddedBodies = 0;
    }

    // Times only the create sequence. The correctness gate runs after the
    // timer has stopped so validation never lands inside the measured window.
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
        if (!mSetupOk || !mPhysX)
        {
            return;
        }

        ovstage_instance_t* stage = mStageAttachment.stage;
        const ovphysx_handle_t handle = mPhysX->handle();

        for (int first = 0; first < kAuthoringBodyCount; first += mPack)
        {
            int last = first + mPack;
            if (last > kAuthoringBodyCount)
            {
                last = kAuthoringBodyCount;
            }

            std::string usda;
            std::string target;
            if (last - first == 1)
            {
                usda = authoringAuthorSingle(first);
                target = std::string(kAuthoringParentPath) + "/box_" + std::to_string(first);
            }
            else
            {
                usda = authoringAuthorPacked(first, last);
                target = kAuthoringParentPath;
            }

            // Author the new prim(s) as one population reference.
            ovstage_population_usd_reference_handle_t reference = 0;
            const ovstage_population_enqueue_result_t add = ovstage_population_add_usd_reference_from_string(
                stage, authoringbm::stringView(usda), authoringbm::stringView(target), &reference);
            if (add.status != OVSTAGE_OK)
            {
                bmRecordFailure(mRowName, "add_usd_reference enqueue rejected: %d", static_cast<int>(add.status));
                mStepOk = false;
                return;
            }
            if (!authoringbm::waitPopulation(stage, add.op_index, mRowName, "add_usd_reference"))
            {
                mStepOk = false;
                return;
            }

            ++mOrdinal;

            // Apply the fresh USD changes at this ordinal.
            const ovstage_population_enqueue_result_t apply = ovstage_population_apply_usd_changes(stage, mOrdinal);
            if (apply.status != OVSTAGE_OK)
            {
                bmRecordFailure(mRowName, "apply_usd_changes enqueue rejected: %d", static_cast<int>(apply.status));
                mStepOk = false;
                return;
            }
            if (!authoringbm::waitPopulation(stage, apply.op_index, mRowName, "apply_usd_changes"))
            {
                mStepOk = false;
                return;
            }

            // Seal the write floor through this ordinal, then drain the
            // matching change interval into physics.
            if (!authoringbm::sealAndDrain(handle, stage, &mLastDrained, mOrdinal, mRowName))
            {
                mStepOk = false;
                return;
            }
            mAddedBodies += (last - first);

            // Simulate. This is where the OMPE-104209 wedge surfaced.
            for (int s = 0; s < kAuthoringStepsBetween; ++s)
            {
                if (ovphysx_step_sync(handle, 1.0f / 60.0f).status != OVPHYSX_API_SUCCESS)
                {
                    const ovphysx_string_t err = ovphysx_get_last_error();
                    bmRecordFailure(
                        mRowName, "step_sync failed: %.*s", static_cast<int>(err.length), err.ptr ? err.ptr : "");
                    mStepOk = false;
                    return;
                }
            }
        }
    }

private:
    // Fail-closed gate. Every body must have reached physics. Runs outside the
    // timed window. A miscount is reported rather than silently accepted, so a
    // regression cannot present as a suspiciously fast row.
    void verify()
    {
        if (!mSetupOk || !mPhysX)
        {
            return;
        }
        if (!mStepOk)
        {
            bmRecordFailure(mRowName, "create sequence did not complete");
            return;
        }

        uint32_t simulated = 0;
        if (!authoringbm::countRigidBodies(mPhysX->handle(), &simulated, mRowName))
        {
            bmRecordFailure(mRowName, "rigid-body readback failed");
            return;
        }
        const uint32_t expected = mSeedBodies + static_cast<uint32_t>(mAddedBodies);
        if (simulated != expected)
        {
            bmRecordFailure(mRowName, "expected %u simulated rigid bodies (%u seed + %d added), found %u", expected,
                            mSeedBodies, mAddedBodies, simulated);
        }
    }

    const char* mRowName = nullptr;
    int mPack = 1;
    bool mGpu = false;

    ovphysx::PhysX* mPhysX = nullptr;
    ovphysx_sample_stage_attachment_t mStageAttachment{};
    std::string mScenePath;

    bool mAttached = false;
    bool mLeftoverAttachmentReported = false;
    bool mSetupOk = false;
    bool mStepOk = true;
    ovstage_ordinal_t mOrdinal = 0;
    ovstage_ordinal_t mLastDrained = 0;
    uint32_t mSeedBodies = 0;
    int mAddedBodies = 0;
};


// 80 bodies, 16 per reference -> 5 population references. The shape a front
// end takes when it batches siblings under a common parent.
struct AuthoringPopulationAddPackedCpu : AuthoringPopulationAddBase
{
    AuthoringPopulationAddPackedCpu()
        : AuthoringPopulationAddBase("Authoring.population_add_packed_cpu", kAuthoringPackedPack, false)
    {
    }
};

struct AuthoringPopulationAddPackedGpu : AuthoringPopulationAddBase
{
    AuthoringPopulationAddPackedGpu()
        : AuthoringPopulationAddBase("Authoring.population_add_packed_gpu", kAuthoringPackedPack, true)
    {
    }
};

// 80 bodies, 1 per reference -> 80 create/drain cycles. The one-object-per-tick
// shape, run deliberately past the ~63 wall.
struct AuthoringPopulationAddDripCpu : AuthoringPopulationAddBase
{
    AuthoringPopulationAddDripCpu()
        : AuthoringPopulationAddBase("Authoring.population_add_drip_cpu", kAuthoringDripPack, false)
    {
    }
};

struct AuthoringPopulationAddDripGpu : AuthoringPopulationAddBase
{
    AuthoringPopulationAddDripGpu()
        : AuthoringPopulationAddBase("Authoring.population_add_drip_gpu", kAuthoringDripPack, true)
    {
    }
};


Register<AuthoringPopulationAddPackedCpu, true> sAuthoringPopulationAddPackedCpu("Authoring.population_add_packed_cpu");
Register<AuthoringPopulationAddPackedGpu, true> sAuthoringPopulationAddPackedGpu("Authoring.population_add_packed_gpu");
Register<AuthoringPopulationAddDripCpu, true> sAuthoringPopulationAddDripCpu("Authoring.population_add_drip_cpu");
Register<AuthoringPopulationAddDripGpu, true> sAuthoringPopulationAddDripGpu("Authoring.population_add_drip_gpu");

} // namespace
