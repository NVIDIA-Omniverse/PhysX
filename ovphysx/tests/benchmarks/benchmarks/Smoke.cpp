// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// Trivial smoke benchmark used to validate the harness loop, output and
// golden compare. Does no real work -- it just spends a fixed amount of CPU
// time so the result has a stable, non-zero mean.

#include "framework/UsdPCH.h"

#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"

#include <chrono>
#include <thread>


void initSmoke()
{
}

namespace
{

class SmokeNoOp : public BmBenchmark
{
public:
    uint32_t getNbSteps() const override { return 10; }
    uint32_t getNbRuns() const override { return 5; }

    void startRun() override {}
    void endRun() override {}
    void preStep() override {}

protected:
    void step() override
    {
        // ~1 ms of sleep gives us a stable, non-zero baseline that does not
        // exercise any real ovphysx code.
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
};

Register<SmokeNoOp> sSmokeNoOp("Smoke.no_op");

} // namespace
