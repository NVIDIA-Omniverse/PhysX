// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

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
    const char* dataFolder; // output file for reports
    const char* goldenFilePath; // the path of goldenFiles
    const char* kitArguments; // arguments to pass into kit on startup
    uint32_t steps; // step count - if non-0, this overrides the benchmark choice
    uint32_t runs; // run count - if non-0, this overrides the benchmark choice
    uint32_t slop;
    int32_t numThreads;
    bool forceGpu;
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

    CmdLineParameters()
        : filterString(NULL),
          reportFile("_report.txt"),
          dataFolder(NULL),
          goldenFilePath(NULL),
          kitArguments(NULL),
          steps(0),
          runs(0),
          slop(10),
          numThreads(-1),
          forceGpu(false),
          sanity(false),
          profile(false),
          regenerate(false),
          detail(false),
          help(false),
          list(false),
          uniquify(false),
          verbose(false),
          runHidden(false),
          enableTracy(false)
    {
    }
};

#endif
