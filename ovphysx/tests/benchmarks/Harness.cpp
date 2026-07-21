// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// Adapted from omni.physx/tests/test.benchmarks/Harness.cpp.
//
// The CLI parsing, harness loop, statistics (trimmed mean / stddev) and
// main() shape are preserved verbatim so upstream improvements in the
// omni.physx harness drop in cleanly. The only diff from upstream is at
// the very top: omni.physx pulls a precompiled USD header and the
// OMNI_APP_GLOBALS macro plus carb::extras::adjustWindowsDllSearchPaths().
// We don't need either here because ovphysx::PhysX::create() (called
// inside bmInitialize -> BmGlobals) handles its own Carbonite bootstrap.

// _GNU_SOURCE enables the glibc extensions malloc_trim (in <malloc.h>) and
// the C99 math symbols (used by <cmath>) that carb's headers otherwise hide.
// Define before any include.
#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include <carb/Defines.h>
#include <carb/ClientUtils.h>

#include "framework/UsdPCH.h"

#include "Harness.h"
#include "framework/BmUtils.h"

// Satisfies carb's linker requirement for a globals define somewhere in the
// binary. The benchmark executable initializes its own Carbonite built-ins for
// file/log helpers without shipping libcarb with ovphysx.
CARB_STATIC_BINARY_GLOBALS("ovphysx_benchmarks")

#if !CARB_PLATFORM_WINDOWS
#include <malloc.h>
#endif

bool match(const char* opt, const char* ref)
{
#if CARB_PLATFORM_WINDOWS
    return !(::_strnicmp(opt, ref, strlen(ref)));
#else
    return !(::strncasecmp(opt, ref, strlen(ref)));
#endif
}

void parse(CmdLineParameters& result, const char* arg)
{
    if (match(arg, "--filter="))
        result.filterString = arg + 9;
    else if (match(arg, "--report="))
        result.reportFile = arg + 9;
    else if (match(arg, "--data="))
        result.dataFolder = arg + 7;
    else if (match(arg, "--steps="))
        result.steps = atol(arg + 8);
    else if (match(arg, "--runs="))
        result.runs = atol(arg + 7);
    else if (match(arg, "--slop="))
        result.slop = atol(arg + 7);
    else if (match(arg, "--sanity_check"))
        result.sanity = true;
    else if (match(arg, "--profile"))
        result.profile = true;
    else if (match(arg, "--threads="))
        result.numThreads = atol(arg + 10);
    else if (match(arg, "--forceGpu"))
        result.forceGpu = true;
    else if (match(arg, "--regenerate"))
        result.regenerate = true;
    else if (match(arg, "--detail"))
        result.detail = true;
    else if (match(arg, "--help"))
        result.help = true;
    else if (match(arg, "--list"))
        result.list = true;
    else if (match(arg, "--verbose"))
        result.verbose = true;
    else if (match(arg, "--hidden"))
        result.runHidden = true;
    else if (match(arg, "--tracy"))
        result.enableTracy = true;
    else if (match(arg, "--kit_arguments="))
        result.kitArguments = arg + 16;
}

CmdLineParameters getCommandLineOptions(int argc, char** argv, PrintfCbFunc printCb)
{
    CmdLineParameters result;

    for (int i = 0; i < argc; i++)
        parse(result, argv[i]);

    if (result.help)
    {
        puts(
            "--help                       Print this help text\n"
            "--list                       List benchmarks\n"
            "\n"
            "--filter=<string>            Limit to tests matching <string>\n"
            "--regenerate                 Regenerate baseline file in data/BenchmarkData\n"
            "--profile                    Enables profiling if supported by the benchmark\n"
            "--detail                     Generate csv per-frame files for each benchmark\n"
            "--data=<directory>           Data folder for the benchmarks\n"
            "--report=<filename>          Dump results to <filename>\n"
            "--threads=<N>                Run benchmarks on N threads if supported by the benchmark\n"
            "--forceGpu                   Run benchmarks on GPU if supported by the benchmark\n"
            "--steps=<N>                  Run benchmarks for N steps\n"
            "--runs=<N>	                  Run benchmark N times, taking the minimum time for each step\n"
            "--slop=<N>                   Percentage tolerance used for initializing tolerances in the regenerated baseline file (default is 10)\n"
            "--sanity_check               Request sanity check output from benchmarks that support it\n"
            "--verbose                    Print the name of each benchmark as it executes (useful if something crashes!)\n"
            "--hidden                     Run benchmarks classified as 'hidden' as well.\n"
            "--tracy                      Enable Tracy profiler and connect to Tracy server\n"
            "--kit_arguments=<arguments>  Command line arguments to be passed into Kit (separated by ';')"
            "\n");

        exit(0);
    }
    else if (result.list)
    {
        std::vector<BmRegistrable*> reg;
        bmGetRegister(reg, result.filterString, true);
        for (uint32_t i = 0; i < reg.size(); i++)
        {
            printFormatted("%s", reg[i]->getName());
            puts(reg[i]->isHidden() ? " (hidden)" : "");
            if (printCb)
            {
                (printCb)("%s", reg[i]->getName());
            }
        }
        exit(0);
    }

    return result;
}

class Trace
{
public:
    static void start(uint32_t /*frame*/, const CmdLineParameters& /*cmdLine*/)
    {
    }

    static void stop(uint32_t /*frame*/, const CmdLineParameters& /*cmdLine*/)
    {
    }
};

namespace BenchmarkHarness
{

void harnessImpl(int argc, char** argv, PrintfCbFunc printCb)
{
    CmdLineParameters cmdLine = getCommandLineOptions(argc, argv, printCb);

    std::vector<char> charBuffer;
    std::vector<const char*> kitArguments;
    if (cmdLine.kitArguments)
    {
        size_t len = strlen(cmdLine.kitArguments);
        charBuffer.resize(2 * len);
        char* buf = charBuffer.data();
        uint32_t argumentStart = 0;
        for (uint32_t i = 0; i < len; i++)
        {
            const bool isSeparator = cmdLine.kitArguments[i] == ';';
            if (isSeparator || ((i+1) == len))
            {
                uint32_t argumentLength;
                if (isSeparator)
                    argumentLength = i - argumentStart;
                else
                    argumentLength = i - argumentStart + 1;

                kitArguments.push_back(buf);

                memcpy(buf, cmdLine.kitArguments + argumentStart, argumentLength);
                buf += argumentLength;
                *buf = '\0';
                buf++;

                argumentStart += argumentLength + 1;
            }
        }
    }

    bmInitialize(cmdLine.sanity, cmdLine.dataFolder, cmdLine.numThreads, cmdLine.forceGpu, cmdLine.profile,
        cmdLine.enableTracy, kitArguments.data(), static_cast<uint32_t>(kitArguments.size()));

    // Compute the device/thread decoration up front (it used to be computed below,
    // after filtering). Reported record names get this postfix appended (e.g.
    // "Step.x_gpu" + "_GPU" -> "Step.x_gpu_GPU"), but --filter is matched against the
    // UNdecorated registered name. So feeding a full reported name back as --filter
    // (which FrameCore's regression sanity step does when it re-runs a flagged test by
    // its reported name) matched nothing -> empty report -> "no valid performance
    // records found". Strip a trailing copy of the active postfix from an exact filter
    // so the base name matches; the postfix is re-applied to the reported name below,
    // so the emitted record name is unchanged.
    std::string postfix("");
    if (cmdLine.numThreads != -1)
    {
        postfix = "_" + std::to_string(cmdLine.numThreads) + "T";
    }
    if (cmdLine.forceGpu)
    {
        postfix = postfix + "_GPU";
    }

    std::string effectiveFilter(cmdLine.filterString ? cmdLine.filterString : "");
    if (!postfix.empty() && effectiveFilter.size() > postfix.size() &&
        effectiveFilter.compare(effectiveFilter.size() - postfix.size(), postfix.size(), postfix) == 0)
    {
        effectiveFilter.erase(effectiveFilter.size() - postfix.size());
    }

    std::vector<BmRegistrable*> reg;
    bmGetRegister(reg, effectiveFilter.empty() ? cmdLine.filterString : effectiveFilter.c_str(),
                  cmdLine.runHidden);

    std::vector<uint64_t> times, runs;
    times.reserve(2048);
    runs.reserve(2048);

    BmRecord* records = new BmRecord[reg.size()];

    const uint32_t minRuns = 3; // this is so we can always discard 1 slowest and 1 fastest runs

    // Apply the device/thread decoration (computed above, before filtering) to the
    // reported names.
    for (uint32_t i = 0; i < reg.size(); i++)
    {
        reg[i]->addPostfix(postfix.c_str());
    }

    for (uint32_t i = 0; i < reg.size(); i++)
    {
        BmRecord& record = records[i];
        BmBenchmark& b = *reg[i]->create();

        if (!b.isValid())
        {
            printFormatted("Benchmark %s failed to initialize, skipping.\n", reg[i]->getName());
            delete &b;
            continue;
        }

        uint32_t runCount = cmdLine.runs ? cmdLine.runs : b.getNbRuns();
        if (runCount)
        {
            if (runCount < minRuns)
            {
                printFormatted("runCount should be at least 3, increasing from %d\n", runCount);
                if (printCb)
                {
                    (printCb)("runCount should be at least 3, increasing from %d\n", runCount);
                }
                runCount = minRuns;
            }
        }

        if (cmdLine.verbose)
        {
            printFormatted("Running %s %d times\n", reg[i]->getName(), runCount);
            if (printCb)
            {
                (printCb)("Running %s %d times\n", reg[i]->getName(), runCount);
            }
        }

        uint32_t stepCount = cmdLine.steps ? cmdLine.steps : b.getNbSteps();

        record.time.resize(stepCount, uint64_t(1) << 30);
        record.stdDev.resize(stepCount, uint64_t(1) << 30);

        // do one dummy unrecorded run with stepCount steps to initialize the instruction cache
        // this significantly improves the noise when running multiple benchmarks
        if (runCount)
        {
            b.startRun();
            for (uint32_t s = 0; s < stepCount; s++)
            {
                b.preStep();
                b.timedStep();
            }
            b.endRun();

            times.clear();
            // run the test (s steps) x (r runs)
            for (uint32_t r = 0; r < runCount; r++)
            {
                b.startRun();

                for (uint32_t s = 0; s < stepCount; s++)
                {
                    b.preStep();

                    Trace::start(s, cmdLine);
                    uint64_t stepTimeMS = uint64_t(b.timedStep() * 1e6f);
                    Trace::stop(s, cmdLine);

                    times.push_back(stepTimeMS);

                    record.time[s] = BmMin(record.time[s], stepTimeMS);
                }

                b.endRun();
            }
        }
        else
        {
            // special run for heavy scenes just run once
            times.clear();
            // run the test (s steps) x (r runs)
            b.startRun();

            for (uint32_t s = 0; s < stepCount; s++)
            {
                b.preStep();

                Trace::start(s, cmdLine);
                uint64_t stepTimeMS = uint64_t(b.timedStep() * 1e6f);
                Trace::stop(s, cmdLine);

                // push back three time to get enough data
                times.push_back(stepTimeMS);
                times.push_back(stepTimeMS);
                times.push_back(stepTimeMS);

                record.time[s] = BmMin(record.time[s], stepTimeMS);
            }

            b.endRun();

            runCount = 3;
        }

        // transpose the array from [R][S] to [S][R]
        runs.resize(times.size());
        for (uint32_t s = 0; s < stepCount; s++)
            for (uint32_t r = 0; r < runCount; r++)
                runs[r + runCount * s] = times[r * stepCount + s];

        for (uint32_t s = 0; s < stepCount; s++)
        {
            // sort section of the array for this step
            std::sort(runs.begin() + s * runCount, runs.begin() + (s + 1) * runCount);

            // discard the fastest 20% and slowest 30% of runs, discard at least 1 slowest and 1 fastest run
            // maxRun is inclusive
            uint32_t minRun = BmMax<uint32_t>(1, uint32_t(runCount * 0.2f));
            uint32_t maxRun = BmMax<uint32_t>(BmMin(uint32_t(runCount * 0.7f), runCount - 2), 1);
            double mean = 0.0f;

#define GETTIME(r, s) runs[r + s * runCount]

            // for each step compute mean over multiple runs
            for (uint32_t r = minRun; r <= maxRun; r++)
            {
                mean += GETTIME(r, s);
            }
            mean *= 1.0f / (maxRun - minRun + 1);
            record.time[s] = uint64_t(mean); // store in record array

            // compute standard deviation
            uint64_t variance = 0;
            const uint64_t avg = record.time[s];
            for (uint32_t r = minRun; r <= maxRun; r++)
            {
                const uint64_t value = GETTIME(r, s);

                if (avg >= value)
                    variance += (avg - value) * (avg - value);
                else
                    variance += (value - avg) * (value - avg);
            }
            record.stdDev[s] = (uint64_t)std::sqrt((double)variance);
        }

        delete &b;

#if !CARB_PLATFORM_WINDOWS
        malloc_trim(0);
#endif
    }

    BmOutput* output = new BmOutput(cmdLine.regenerate, cmdLine.slop, true, cmdLine.reportFile, printCb);
    output->printHeaders();
    for (uint32_t i = 0; i < reg.size(); i++)
    {
        if (cmdLine.detail)
            output->dump(reg[i]->getName(), records[i]);
        output->emit(reg[i]->getName(), bmGetDefaultResult(records[i]));
    }

    delete output;

    delete[] records;
    bmTerminate();
}
} // namespace BenchmarkHarness


int main(int argc, char** argv)
{
    BenchmarkHarness::harnessImpl(argc, argv, NULL);
    return 0;
}
