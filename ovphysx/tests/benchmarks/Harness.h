// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Adapted from omni.physx/tests/test.benchmarks/Harness.h to add the
// OVPhysX-specific DirectGPU opt-in.

/**
 * @implements REQ-CAPI-BENCHMARK-002
 * @covers AC-1 AC-3
 */

#ifndef HARNESS_H
#define HARNESS_H

#include "BenchmarkList.h"
#include "framework/BmBenchmark.h"
#include "framework/BmGlobals.h"
#include "framework/BmOutput.h"
#include "framework/BmTime.h"


namespace BenchmarkHarness
{
void harnessImpl(int argc, char** argv, PrintfCbFunc cb);
}

struct CmdLineParameters
{
    const char* filterString; // prefix filter string
    const char* reportFile; // output file for reports
    const char* timingDiagnosticsFile; // optional untrimmed timing diagnostics JSONL output
    const char* dataFolder; // data folder for the benchmarks
    const char* goldenFilePath; // the path of goldenFiles
    const char* kitArguments; // arguments to pass into kit on startup
    uint32_t steps; // step count, overrides the benchmark choice when non-zero
    uint32_t runs; // run count, overrides the benchmark choice when non-zero
    uint32_t slop;
    int32_t numThreads;
    bool forceGpu;
    // Enables GPU tensors. Suppresses GPU readback, PhysX to USD writes,
    // and USD prim edits flowing through Fabric to PhysX.
    bool directGpu;
    bool sanity; // whether the benchmark should return a sanity value
    bool profile;
    bool regenerate;
    bool detail;
    bool help;
    bool list;
    bool uniquify;
    bool verbose;
    bool runHidden;
    bool enableTracy;
    bool enableNvtx;

    CmdLineParameters()
        : filterString(NULL),
          reportFile("_report.txt"),
          timingDiagnosticsFile(NULL),
          dataFolder(NULL),
          goldenFilePath(NULL),
          kitArguments(NULL),
          steps(0),
          runs(0),
          slop(10),
          numThreads(-1),
          forceGpu(false),
          directGpu(false),
          sanity(false),
          profile(false),
          regenerate(false),
          detail(false),
          help(false),
          list(false),
          uniquify(false),
          verbose(false),
          runHidden(false),
          enableTracy(false),
          enableNvtx(false)
    {
    }
};

#endif
