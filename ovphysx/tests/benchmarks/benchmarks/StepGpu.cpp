// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// Per-step simulation cost on GPU device. Only runs when --forceGpu was set.

#include "framework/UsdPCH.h"

#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"
#include "../framework/BmUtils.h"
#include "../OvstageLoad.h"

#include <ovphysx/experimental/ovphysx.hpp>

#include <string>


void initStepGpu()
{
}

namespace
{

class StepGpuBase : public BmBenchmark
{
public:
    explicit StepGpuBase(const char* fixture) : mFixture(fixture) {}

    bool isValid() const override
    {
        if (!BmGlobals::getInstance().forceGpu()) return false;
        return BmGlobals::getInstance().getPhysX() != nullptr;
    }

    uint32_t getNbSteps() const override { return 60; }
    uint32_t getNbRuns() const override { return 5; }

    void startRun() override
    {
        // Cache PhysX* once — see StepCpuBase for rationale (MR !7247).
        mPhysX = BmGlobals::getInstance().getPhysX();
        if (!mPhysX) return;

        const std::string path = BmGlobals::getInstance().getDataFolder() + "/" + mFixture;
        if (!benchmarkLoadUsdWithOvstage(mPhysX, path, mStageAttachment))
        {
            printFormatted("StepGpu: ovstage load(%s) failed", mFixture.c_str());
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


class StepGpu_TwoArticulations : public StepGpuBase
{
public:
    StepGpu_TwoArticulations() : StepGpuBase("two_articulations_gpu.usda") {}
};

class StepGpu_ArticulationPileup : public StepGpuBase
{
public:
    StepGpu_ArticulationPileup()
        : StepGpuBase("../benchmarks/data/articulation_pileup.usda") {}
};

// Minimal stepping fixture, GPU. See StepCpu.cpp::StepCpu_Cubes20 for rationale.
class StepGpu_Cubes20 : public StepGpuBase
{
public:
    StepGpu_Cubes20()
        : StepGpuBase("../benchmarks/data/cubes20.usda") {}
};

// Warehouse stepping (~3.5k dynamic), GPU. Heavy fixture for steady-state throughput.
class StepGpu_Warehouse : public StepGpuBase
{
public:
    StepGpu_Warehouse()
        : StepGpuBase("../benchmarks/data/warehouse.usda") {}
};


Register<StepGpu_TwoArticulations>   sStepGpuTwoArticulations("Step.two_articulations_gpu");
Register<StepGpu_ArticulationPileup> sStepGpuArticulationPileup("Step.articulation_pileup_gpu");
Register<StepGpu_Warehouse>          sStepGpuWarehouse("Step.warehouse_gpu");
Register<StepGpu_Cubes20>            sStepGpuCubes20("Step.cubes20_gpu");

} // namespace
