// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// Low-load benchmarks measuring per-call overhead (dispatch, init, reset)
// rather than steady-state simulation cost. Useful for catching regressions
// in cold paths that the Step.* benchmarks amortize away.

#include "framework/UsdPCH.h"

#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"
#include "../framework/BmUtils.h"
#include "../OvstageLoad.h"

#include <ovphysx/experimental/ovphysx.hpp>

#include <string>


void initLowLoad()
{
}

namespace
{

// Shared base: caches the PhysX* once in startRun() so step() doesn't pay
// for a global lookup + null check on every measured iteration -- defensive
// coding has no place in the hot path.
class LowLoadBase : public BmBenchmark
{
public:
    // Gate on PhysX bootstrap success — if BmGlobals' init failed, getPhysX()
    // returns null and the cached mPhysX in startRun() would also be null;
    // step() then dereferences it. Cleanest place to skip is here, called
    // once before any step() runs.
    bool isValid() const override
    {
        return BmGlobals::getInstance().getPhysX() != nullptr;
    }

    void startRun() override
    {
        mPhysX = BmGlobals::getInstance().getPhysX();
    }

    void endRun() override
    {
        if (mPhysX)
        {
            benchmarkClearOvstage(mPhysX, mStageAttachment);
        }
    }

    void preStep() override {}

protected:
    ovphysx::PhysX* mPhysX = nullptr;
    ovphysx_sample_stage_attachment_t mStageAttachment{};
};


// first_step_after_reload: time to step() once after re-loading the scene.
// This is NOT a true process-restart cold start (the harness pays the
// PhysX::create() cost once in bmCreateGlobals, then re-uses it across
// runs). What we actually measure is: re-load the scene, reset it, then
// time the first step. True process-restart cold-start lives on the Python
// side via subprocess (see bench_process_cold_start.py).
class LowLoad_FirstStepAfterReload : public LowLoadBase
{
public:
    uint32_t getNbSteps() const override { return 1; }
    uint32_t getNbRuns() const override { return 20; }

    void startRun() override
    {
        LowLoadBase::startRun();
        if (!mPhysX) return;

        const std::string path =
            BmGlobals::getInstance().getDataFolder() + "/basic_simulation.usda";
        if (!benchmarkLoadUsdWithOvstage(mPhysX, path, mStageAttachment))
        {
            printFormatted("LowLoad.first_step_after_reload: ovstage load failed");
            return;
        }
    }

    void step() override
    {
        mPhysX->step(1.0f / 60.0f);
        mPhysX->waitAll();
    }
};

// empty_step: per-step cost of a step() call against an empty scene that's
// already loaded. Floors out per-call dispatch overhead — the cost that
// every other Step.* benchmark pays before doing any simulation work.
class LowLoad_EmptyStep : public LowLoadBase
{
public:
    uint32_t getNbSteps() const override { return 500; }
    uint32_t getNbRuns() const override { return 5; }

    void step() override
    {
        mPhysX->step(1.0f / 60.0f);
        mPhysX->waitAll();
    }
};

// noop_ovstage_attach: per-call cost of ovstage population + attach/update on a minimal USDA.
// Isolates parse/cooking/registration overhead from rigid-body count.
class LowLoad_NoopOvstageAttach : public LowLoadBase
{
public:
    uint32_t getNbSteps() const override { return 50; }
    uint32_t getNbRuns() const override { return 5; }

    void startRun() override
    {
        LowLoadBase::startRun();
        // Cache the path string so step() doesn't pay for a heap-allocated
        // std::string concat per measured iteration.
        mPath = BmGlobals::getInstance().getDataFolder() + "/basic_simulation.usda";
    }

    void step() override
    {
        // Check the status so a regression in ovstage attach/update (e.g. reset doesn't
        // fully clear stage between iterations) surfaces as a printed
        // failure rather than as a silently-low "noop_ovstage_attach" measurement
        // that looks like an improvement.
        if (!benchmarkLoadUsdWithOvstage(mPhysX, mPath, mStageAttachment))
            printFormatted("LowLoad.noop_ovstage_attach: ovstage load failed");
        // Per-step teardown so each step measures a clean ovstage attach.
        benchmarkClearOvstage(mPhysX, mStageAttachment);
    }

private:
    std::string mPath;
};

// reset: per-call cost of ovphysx_reset_stage against a loaded scene. Matters
// for RL-style episode resets that hit this path frequently.
class LowLoad_Reset : public LowLoadBase
{
public:
    uint32_t getNbSteps() const override { return 50; }
    uint32_t getNbRuns() const override { return 5; }

    void endRun() override {}

    void preStep() override
    {
        // Load the scene before each measured reset.
        if (!mPhysX) return;
        const std::string path =
            BmGlobals::getInstance().getDataFolder() + "/basic_simulation.usda";
        (void)benchmarkLoadUsdWithOvstage(mPhysX, path, mStageAttachment);
    }

    void step() override
    {
        (void)ovphysx_reset_stage(mPhysX->handle());
        mPhysX->waitAll();
    }
};


Register<LowLoad_FirstStepAfterReload> sLowLoadFirstStepAfterReload("LowLoad.first_step_after_reload");
Register<LowLoad_EmptyStep>            sLowLoadEmptyStep("LowLoad.empty_step");
Register<LowLoad_NoopOvstageAttach>    sLowLoadNoopOvstageAttach("LowLoad.noop_ovstage_attach");
Register<LowLoad_Reset>                sLowLoadReset("LowLoad.reset");

} // namespace
