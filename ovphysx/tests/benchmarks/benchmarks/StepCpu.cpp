// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// Per-step simulation cost on CPU device. Loads a fixture once per run and
// measures step+waitAll over getNbSteps() steps.
//
// These benchmarks declare themselves invalid (isValid() == false) when the
// process was started with --forceGpu, so the GPU pass skips them; the
// matching StepGpu.cpp does the opposite.

#include "framework/UsdPCH.h"

#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"
#include "../framework/BmUtils.h"
#include "../OvstageLoad.h"

#include <ovphysx/experimental/ovphysx.hpp>

#include <string>


void initStepCpu()
{
}

namespace
{

class StepCpuBase : public BmBenchmark
{
public:
    explicit StepCpuBase(const char* fixture) : mFixture(fixture) {}

    bool isValid() const override
    {
        // Skip on the GPU pass (the StepGpu variant covers that mode).
        // Also skip if PhysX bootstrap failed — see LowLoad.cpp::isValid for
        // rationale.
        if (BmGlobals::getInstance().forceGpu()) return false;
        return BmGlobals::getInstance().getPhysX() != nullptr;
    }

    uint32_t getNbSteps() const override { return 60; }
    uint32_t getNbRuns() const override { return 5; }

    void startRun() override
    {
        // Cache the PhysX* once so step() doesn't pay for a global lookup
        // on every measured iteration (defensive coding has no place in
        // the hot path — review on MR !7247).
        mPhysX = BmGlobals::getInstance().getPhysX();
        if (!mPhysX) return;

        const std::string path = BmGlobals::getInstance().getDataFolder() + "/" + mFixture;
        if (!benchmarkLoadUsdWithOvstage(mPhysX, path, mStageAttachment))
        {
            printFormatted("StepCpu: ovstage load(%s) failed", mFixture.c_str());
            return;
        }
    }

    void endRun() override
    {
        if (!mPhysX) return;
        benchmarkClearOvstage(mPhysX, mStageAttachment);
    }

    void preStep() override {}

protected:
    void step() override
    {
        mPhysX->step(kDt);
        mPhysX->waitAll();
    }

private:
    static constexpr float kDt = 1.0f / 60.0f;
    std::string mFixture;
    ovphysx::PhysX* mPhysX = nullptr;
    ovphysx_sample_stage_attachment_t mStageAttachment{};
};


class StepCpu_BasicSimulation : public StepCpuBase
{
public:
    StepCpu_BasicSimulation() : StepCpuBase("basic_simulation.usda") {}
};

// Articulation pileup: 16 floating-root articulations (3..18 links each)
// fall onto a scatter of 100 large rigid cubes and spheres on a ground plane.
// Adapted from omni.physxdemos.scenes.ArticulationDemo with the demo context
// stripped out so it loads as a plain USDA fixture.
class StepCpu_ArticulationPileup : public StepCpuBase
{
public:
    StepCpu_ArticulationPileup()
        : StepCpuBase("../benchmarks/data/articulation_pileup.usda") {}
};

// Minimal stepping fixture — 20 falling cubes + ground plane. Sized as
// an overhead-probe scene: triangulates per-step fixed cost vs body-count
// scaling when paired with LowLoad.empty_step (~5us, no scene) and
// Step.basic_simulation_cpu (~1ms, 1 body).
class StepCpu_Cubes20 : public StepCpuBase
{
public:
    StepCpu_Cubes20()
        : StepCpuBase("../benchmarks/data/cubes20.usda") {}
};

// Warehouse stepping (~3.5k dynamic). Heavy fixture for steady-state throughput.
class StepCpu_Warehouse : public StepCpuBase
{
public:
    StepCpu_Warehouse()
        : StepCpuBase("../benchmarks/data/warehouse.usda") {}
};


Register<StepCpu_BasicSimulation>    sStepCpuBasicSimulation("Step.basic_simulation_cpu");
Register<StepCpu_ArticulationPileup> sStepCpuArticulationPileup("Step.articulation_pileup_cpu");
Register<StepCpu_Warehouse>          sStepCpuWarehouse("Step.warehouse_cpu");
Register<StepCpu_Cubes20>            sStepCpuCubes20("Step.cubes20_cpu");

} // namespace
