// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-BENCHMARK-001
 * @covers AC-1 AC-4 AC-5
 */

// Authoring.population_scale_cpu incremental spawn scaling. Spawns one body at
// a time into an accumulating attached scene that is never reset between
// measured steps, so the per-step time series is the per-op cost as a function
// of how many bodies are already present. This is the axis NVBugs 6664721
// reports on: per-spawn cost grows with scene size while a teleport-from-pool
// lane in the same process stays flat.
//
// The Authoring.population_add_* rows reload a fresh scene every measured step,
// so they report per-op cost at a fixed scene size and cannot see this growth.
// Here the scene is kept across steps, so step k measures the cost of adding
// the (k+1)-th body to a scene that already holds k. endRun() fits
// cost(k)=a+b*k and prints the ticket's drift metric.
//
// The single-body author/drain sequence is the drip row's, reused from
// AuthoringCommon.h helpers. Only the accumulate-vs-reset lifetime differs.

#include "framework/UsdPCH.h"

#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"
#include "../framework/BmUtils.h"
#include "../BenchmarkFailure.h"
#include "AuthoringCommon.h"

#include <ovphysx/experimental/ovphysx.hpp>

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>


void initAuthoringPopulationScale()
{
}


namespace
{

// n=150 matches the ticket's spawn count. The first few ops are first-touch
// warm-up (allocator growth, cache fill) and are excluded from the fit, as the
// ticket excludes its first 3 ops.
const int kScaleSpawnCount = 150;
const int kScaleWarmup = 3;
const double kScaleDriftBudget = 0.50;
const int kScaleStepsBetween = 1;
const char* const kScaleParentPath = "/World/DynamicBoxes";

const char* const kScaleColliderUsda =
    "        def Cube \"Collider\" (prepend apiSchemas = [\"PhysicsCollisionAPI\"])\n"
    "        {\n"
    "            double size = 1.0\n"
    "            double3 xformOp:scale = (0.5, 0.5, 0.5)\n"
    "            uniform token[] xformOpOrder = [\"xformOp:scale\"]\n"
    "        }\n";

// Deterministic lattice, matching the population-add rows so authored poses
// stay comparable across the create benchmarks.
void scaleBodyPose(int index, double& x, double& y, double& z)
{
    x = static_cast<double>(index % 8) * 1.5 - 6.0;
    y = 3.0 + static_cast<double>(index / 8) * 1.5;
    z = static_cast<double>((index / 8) % 8) * 1.5 - 6.0;
}

std::string scaleFormatDouble(double value)
{
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.9g", value);
    return std::string(buffer);
}

// One body as its own referenced layer, targeted at an absolute prim path.
std::string scaleAuthorSingle(int index)
{
    double x = 0.0, y = 0.0, z = 0.0;
    scaleBodyPose(index, x, y, z);
    std::string usda;
    usda += "#usda 1.0\n";
    usda += "(defaultPrim = \"Body\")\n";
    usda += "def Xform \"Body\" (prepend apiSchemas = [\"PhysicsRigidBodyAPI\", \"PhysicsMassAPI\"])\n";
    usda += "{\n";
    usda += "    double3 xformOp:translate = (" + scaleFormatDouble(x) + ", " + scaleFormatDouble(y) + ", " +
            scaleFormatDouble(z) + ")\n";
    usda += "    quatf xformOp:orient = (1, 0, 0, 0)\n";
    usda += "    uniform token[] xformOpOrder = [\"xformOp:translate\", \"xformOp:orient\"]\n";
    usda += "    float physics:mass = 1.0\n";
    usda += kScaleColliderUsda;
    usda += "}\n";
    return usda;
}

double medianOf(std::vector<double> v)
{
    if (v.empty())
    {
        return 0.0;
    }
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    return (n % 2) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}


class AuthoringPopulationScaleCpu : public BmBenchmark
{
public:
    bool isValid() const override
    {
        if (BmGlobals::getInstance().forceGpu())
        {
            return false; // CPU-only row
        }
        return BmGlobals::getInstance().getPhysX() != nullptr;
    }

    uint32_t getNbSteps() const override
    {
        return static_cast<uint32_t>(kScaleSpawnCount);
    }

    // One accumulating series per run. Min-over-runs would hide the growth, so
    // a single run is what this row reports on.
    uint32_t getNbRuns() const override
    {
        return 1;
    }

    // Attach once per run and keep the scene. Steps accumulate, unlike the
    // population-add rows which reload a fresh scene in preStep().
    void startRun() override
    {
        ++mRunCounter;
        mSetupOk = false;
        mStepOk = true;
        mAttached = false;
        mStepIndex = 0;
        mTimesMs.clear();
        mTimesMs.reserve(kScaleSpawnCount);

        mPhysX = BmGlobals::getInstance().getPhysX();
        mScenePath = BmGlobals::getInstance().getDataFolder() + "/empty_dynamic_boxes_cpu.usda";
        if (!mPhysX)
        {
            return;
        }
        if (!authoringbm::prepareStageAttachmentForRun(mStageAttachment, kRowName, &mLeftoverAttachmentReported))
        {
            return;
        }
        authoringbm::GpuFallbackScope gpuScope(false, kRowName);
        if (!authoringbm::populateAndAttachChecked(mPhysX, authoringbm::UsdSource::eFile, mScenePath,
                                                   "ovphysx-authoring-population-scale", mStageAttachment, kRowName,
                                                   &mAttached))
        {
            return;
        }
        if (!authoringbm::checkDeviceEvidenceAndWarmup(mPhysX->handle(), false, gpuScope, kRowName))
        {
            return;
        }
        if (!authoringbm::countRigidBodies(mPhysX->handle(), &mSeedBodies, kRowName))
        {
            bmRecordFailure(kRowName, "seed rigid-body readback failed");
            return;
        }
        mOrdinal = mStageAttachment.ordinal;
        mLastDrained = mStageAttachment.ordinal;
        mSetupOk = true;
    }

    // Untimed. Authors the USDA for the body this step will spawn. There is no
    // reset, so the scene keeps every body added by the previous steps.
    void preStep() override
    {
        if (!mSetupOk)
        {
            return;
        }
        mPendingUsda = scaleAuthorSingle(mStepIndex);
        mPendingTarget = std::string(kScaleParentPath) + "/box_" + std::to_string(mStepIndex);
    }

    // Times the single spawn and records it into the per-step series so
    // endRun() can fit the growth. Verification runs after the timer stops.
    Time::Second timedStep() override
    {
        Time timer;
        step();
        const Time::Second elapsed = timer.getElapsedSeconds();
        mTimesMs.push_back(static_cast<double>(elapsed) * 1e3);
        verify();
        ++mStepIndex;
        return elapsed;
    }

    void endRun() override
    {
        printScalingReport();
        if (!authoringbm::clearOvstageChecked(mAttached ? mPhysX : nullptr, mStageAttachment, kRowName))
        {
            bmRecordFailure(kRowName, "ovstage teardown failed; retaining the stage attachment");
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

        ovstage_population_usd_reference_handle_t reference = 0;
        const ovstage_population_enqueue_result_t add = ovstage_population_add_usd_reference_from_string(
            stage, authoringbm::stringView(mPendingUsda), authoringbm::stringView(mPendingTarget), &reference);
        if (add.status != OVSTAGE_OK)
        {
            bmRecordFailure(kRowName, "add_usd_reference enqueue rejected: %d", static_cast<int>(add.status));
            mStepOk = false;
            return;
        }
        if (!authoringbm::waitPopulation(stage, add.op_index, kRowName, "add_usd_reference"))
        {
            mStepOk = false;
            return;
        }

        ++mOrdinal;
        const ovstage_population_enqueue_result_t apply = ovstage_population_apply_usd_changes(stage, mOrdinal);
        if (apply.status != OVSTAGE_OK)
        {
            bmRecordFailure(kRowName, "apply_usd_changes enqueue rejected: %d", static_cast<int>(apply.status));
            mStepOk = false;
            return;
        }
        if (!authoringbm::waitPopulation(stage, apply.op_index, kRowName, "apply_usd_changes"))
        {
            mStepOk = false;
            return;
        }

        if (!authoringbm::sealAndDrain(handle, stage, &mLastDrained, mOrdinal, kRowName))
        {
            mStepOk = false;
            return;
        }

        for (int s = 0; s < kScaleStepsBetween; ++s)
        {
            if (ovphysx_step_sync(handle, 1.0f / 60.0f).status != OVPHYSX_API_SUCCESS)
            {
                const ovphysx_string_t err = ovphysx_get_last_error();
                bmRecordFailure(kRowName, "step_sync failed: %.*s", static_cast<int>(err.length),
                                err.ptr ? err.ptr : "");
                mStepOk = false;
                return;
            }
        }
    }

private:
    // Accumulating gate. After spawning the (k+1)-th body the scene must hold
    // seed + (k+1) rigid bodies. Runs outside the timed window.
    void verify()
    {
        if (!mSetupOk || !mPhysX)
        {
            return;
        }
        if (!mStepOk)
        {
            bmRecordFailure(kRowName, "spawn sequence did not complete at index %d", mStepIndex);
            return;
        }
        uint32_t simulated = 0;
        if (!authoringbm::countRigidBodies(mPhysX->handle(), &simulated, kRowName))
        {
            bmRecordFailure(kRowName, "rigid-body readback failed");
            return;
        }
        const uint32_t expected = mSeedBodies + static_cast<uint32_t>(mStepIndex + 1);
        if (simulated != expected)
        {
            bmRecordFailure(kRowName, "expected %u simulated rigid bodies (%u seed + %d spawned), found %u", expected,
                            mSeedBodies, mStepIndex + 1, simulated);
        }
    }

    // Diagnostic. Fits cost(k)=a+b*k over the post-warm-up per-step series and
    // prints the ticket's drift metric.
    // drift = slope * n / median(first-quartile per-op cost).
    void printScalingReport()
    {
        const int n = static_cast<int>(mTimesMs.size());
        if (n <= kScaleWarmup + 4)
        {
            printFormatted("%s [run %d]: too few samples for a scaling fit (n=%d)", kRowName, mRunCounter, n);
            return;
        }

        // Least-squares fit over k = warmup..n-1, cost in ms.
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        int m = 0;
        for (int k = kScaleWarmup; k < n; ++k)
        {
            const double x = static_cast<double>(k);
            const double y = mTimesMs[k];
            sx += x; sy += y; sxx += x * x; sxy += x * y; ++m;
        }
        const double denom = m * sxx - sx * sx;
        const double slope = denom != 0.0 ? (m * sxy - sx * sy) / denom : 0.0;
        const double intercept = m != 0 ? (sy - slope * sx) / m : 0.0;

        // Quartile medians over the post-warm-up series, in index order.
        std::vector<double> post(mTimesMs.begin() + kScaleWarmup, mTimesMs.end());
        const int q = (std::max)(1, static_cast<int>(post.size()) / 4);
        auto qMed = [&](int idx) {
            const int lo = idx * q;
            const int hi = (std::min)(static_cast<int>(post.size()), lo + q);
            return medianOf(std::vector<double>(post.begin() + lo, post.begin() + hi));
        };
        const double q1 = qMed(0), q2 = qMed(1), q3 = qMed(2), q4 = qMed(3);
        const double drift = q1 != 0.0 ? slope * static_cast<double>(n) / q1 : 0.0;
        const double ratio = q1 != 0.0 ? q4 / q1 : 0.0;

        printFormatted("%s [run %d]: n=%d - fit %.2f + %.4f*k ms", kRowName, mRunCounter, n, intercept, slope);
        printFormatted("%s [run %d]: drift %.2f (budget <%.2f) - quartile medians %.2f -> %.2f -> %.2f -> %.2f ms = %.2fx",
                       kRowName, mRunCounter, drift, kScaleDriftBudget, q1, q2, q3, q4, ratio);
        printFormatted("%s [run %d]: SCALING %s", kRowName, mRunCounter,
                       drift <= kScaleDriftBudget ? "FLAT (per-op cost ~constant with scene size)" :
                                                    "GROWS (per-op cost rises with scene size)");
    }

    static constexpr const char* kRowName = "Authoring.population_scale_cpu";

    ovphysx::PhysX* mPhysX = nullptr;
    ovphysx_sample_stage_attachment_t mStageAttachment{};
    std::string mScenePath;
    std::string mPendingUsda;
    std::string mPendingTarget;
    std::vector<double> mTimesMs;

    bool mAttached = false;
    bool mLeftoverAttachmentReported = false;
    bool mSetupOk = false;
    bool mStepOk = true;
    int mStepIndex = 0;
    int mRunCounter = 0;
    ovstage_ordinal_t mOrdinal = 0;
    ovstage_ordinal_t mLastDrained = 0;
    uint32_t mSeedBodies = 0;
};


Register<AuthoringPopulationScaleCpu, true> sAuthoringPopulationScaleCpu("Authoring.population_scale_cpu");

} // namespace
