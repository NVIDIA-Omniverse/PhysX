// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// USD load benchmarks: time the cost of ovstage population plus attach/update across
// representative scene fixtures. Each step performs one full load cycle;
// the prior load is cleared in preStep() so the timed step measures only
// ovstage population + sync, not the prior load's state.

#include "framework/UsdPCH.h"

#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"
#include "../framework/BmUtils.h"
#include "../OvstageLoad.h"

#include <ovphysx/ovphysx.h>
#include <ovphysx/experimental/ovphysx.hpp>

#include <string>


void initUsdLoad()
{
}

namespace
{

class UsdLoadBase : public BmBenchmark
{
public:
    explicit UsdLoadBase(const char* fixture) : mFixture(fixture) {}

    uint32_t getNbSteps() const override { return 1; }
    uint32_t getNbRuns() const override { return 7; }

    void startRun() override {}
    void endRun() override {}

    void preStep() override
    {
        // Start each step from an empty stage. We use the C API directly
        // because the C++ wrapper's two reset() overloads are ambiguous at
        // the call site.
        ovphysx::PhysX* physx = BmGlobals::getInstance().getPhysX();
        if (!physx) return;
        benchmarkClearOvstage(physx, mStageAttachment);
    }

protected:
    void step() override
    {
        ovphysx::PhysX* physx = BmGlobals::getInstance().getPhysX();
        if (!physx) return;

        const std::string path = BmGlobals::getInstance().getDataFolder() + "/" + mFixture;
        if (!benchmarkLoadUsdWithOvstage(physx, path, mStageAttachment))
        {
            printFormatted("UsdLoad: ovstage load(%s) failed", mFixture.c_str());
            return;
        }
    }

private:
    std::string mFixture;
    ovphysx_sample_stage_attachment_t mStageAttachment{};
};


class UsdLoad_BasicSimulation : public UsdLoadBase
{
public:
    UsdLoad_BasicSimulation() : UsdLoadBase("basic_simulation.usda") {}
};

class UsdLoad_ArticulationPileup : public UsdLoadBase
{
public:
    UsdLoad_ArticulationPileup()
        : UsdLoadBase("../benchmarks/data/articulation_pileup.usda") {}
};

// ~2.4k dynamic + ~400 static via gen_warehouse.py. "Startup time on a
// larger factory" coverage (OMPE-94463).
class UsdLoad_Warehouse : public UsdLoadBase
{
public:
    UsdLoad_Warehouse()
        : UsdLoadBase("../benchmarks/data/warehouse.usda") {}
};

// Cubes20 — minimal overhead-probe fixture. 20 falling cubes + ground plane.
class UsdLoad_Cubes20 : public UsdLoadBase
{
public:
    UsdLoad_Cubes20()
        : UsdLoadBase("../benchmarks/data/cubes20.usda") {}
};

// Cartpole — the single-env fixture used by Lab.cartpole_* (OMPE-94463).
// Loading cost itself is small; this bench mostly serves as a sanity check
// that the fixture parses correctly and to track parse-cost regressions.
class UsdLoad_Cartpole : public UsdLoadBase
{
public:
    UsdLoad_Cartpole()
        : UsdLoadBase("../benchmarks/data/cartpole.usda") {}
};


Register<UsdLoad_BasicSimulation>    sUsdLoadBasicSimulation("UsdLoad.basic_simulation");
Register<UsdLoad_ArticulationPileup> sUsdLoadArticulationPileup("UsdLoad.articulation_pileup");
Register<UsdLoad_Warehouse>          sUsdLoadWarehouse("UsdLoad.warehouse");
Register<UsdLoad_Cartpole>           sUsdLoadCartpole("UsdLoad.cartpole");
Register<UsdLoad_Cubes20>            sUsdLoadCubes20("UsdLoad.cubes20");

} // namespace
