// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-BENCHMARK-001
 * @covers AC-1 AC-2 AC-4
 *
 * @implements REQ-CAPI-BENCHMARK-002
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-CAPI-BENCHMARK-003
 * @covers AC-2
 */

// Adapted from omni.physx/tests/test.benchmarks/Harness.cpp.
//
// The CLI parsing, harness loop, and statistics (trimmed mean / stddev) are
// preserved verbatim so upstream improvements in the omni.physx harness drop
// in cleanly. The bootstrap and main error boundary are ovphysx-specific.

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
#include <exception>
#include <filesystem>
#include <memory>
#include <stdexcept>

#include <carb/Defines.h>
#include <carb/ClientUtils.h>

#include "framework/UsdPCH.h"

#include "Harness.h"
#include "BenchmarkFailure.h"
#include "TimingDiagnostics.h"
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
    else if (match(arg, "--timing-diagnostics="))
        result.timingDiagnosticsFile = arg + 21;
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
    else if (match(arg, "--directGpu"))
        result.directGpu = true;
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
    else if (match(arg, "--nvtx"))
        result.enableNvtx = true;
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
            "--timing-diagnostics=<file>  Write untrimmed timing diagnostics as JSONL\n"
            "--threads=<N>                Run benchmarks on N threads if supported by the benchmark\n"
            "--forceGpu                   Run benchmarks on GPU if supported by the benchmark\n"
            "--directGpu                  Use OVPhysX DirectGPU settings (requires --forceGpu)\n"
            "--steps=<N>                  Run benchmarks for N steps\n"
            "--runs=<N>	                  Run benchmark N times, using a per-step trimmed mean across runs\n"
            "--slop=<N>                   Percentage tolerance used for initializing tolerances in the regenerated baseline file (default is 10)\n"
            "--sanity_check               Request sanity check output from benchmarks that support it\n"
            "--verbose                    Print the name of each benchmark as it executes (useful if something crashes!)\n"
            "--hidden                     Run benchmarks classified as 'hidden' as well.\n"
            "--tracy                      Enable Tracy profiler and connect to Tracy server\n"
            "--nvtx                       Emit NVTX ranges for capture with Nsight Systems\n"
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
namespace
{

std::filesystem::path normalizeOutputPath(const std::filesystem::path& path, const char* option, uint32_t depth = 0)
{
    constexpr uint32_t kMaxSymlinkDepth = 40;
    if (depth == kMaxSymlinkDepth)
    {
        throw std::invalid_argument(std::string("could not resolve ") + option + " path: too many symbolic links");
    }

    std::error_code error;
    const std::filesystem::path absolutePath = std::filesystem::absolute(path, error);
    if (error)
    {
        throw std::invalid_argument(std::string("could not resolve ") + option + " path");
    }

    const std::filesystem::file_status status = std::filesystem::symlink_status(absolutePath, error);
    if (!error && std::filesystem::is_symlink(status))
    {
        const std::filesystem::path target = std::filesystem::read_symlink(absolutePath, error);
        if (error)
        {
            throw std::invalid_argument(std::string("could not resolve ") + option + " symbolic link");
        }
        const std::filesystem::path resolvedTarget =
            target.is_absolute() ? target : absolutePath.parent_path() / target;
        return normalizeOutputPath(resolvedTarget, option, depth + 1);
    }

    error.clear();
    const std::filesystem::path normalizedPath = std::filesystem::weakly_canonical(absolutePath, error);
    if (error)
    {
        throw std::invalid_argument(std::string("could not resolve ") + option + " path");
    }
    return normalizedPath;
}

bool outputPathsMatch(const char* reportPath, const char* diagnosticsPath)
{
    std::error_code error;
    if (std::filesystem::equivalent(reportPath, diagnosticsPath, error))
    {
        return true;
    }

    const std::filesystem::path normalizedReport = normalizeOutputPath(reportPath, "--report");
    const std::filesystem::path normalizedDiagnostics =
        normalizeOutputPath(diagnosticsPath, "--timing-diagnostics");
#if CARB_PLATFORM_WINDOWS
    return ::_stricmp(normalizedReport.generic_string().c_str(), normalizedDiagnostics.generic_string().c_str()) == 0;
#else
    return normalizedReport == normalizedDiagnostics;
#endif
}

} // namespace

void harnessImpl(int argc, char** argv, PrintfCbFunc printCb)
{
    CmdLineParameters cmdLine = getCommandLineOptions(argc, argv, printCb);

    if (cmdLine.directGpu && !cmdLine.forceGpu)
    {
        throw std::invalid_argument("--directGpu requires --forceGpu");
    }
    if (cmdLine.timingDiagnosticsFile != nullptr && cmdLine.reportFile != nullptr)
    {
        if (outputPathsMatch(cmdLine.reportFile, cmdLine.timingDiagnosticsFile))
        {
            throw std::invalid_argument("--timing-diagnostics and --report must name different files");
        }
    }

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

    // --nvtx sets the OVPHYSX_NVTX environment variable rather than passing a
    // config entry down, so that bmInitialize keeps the signature it shares with
    // the omni.physx harness. It has to happen before bmInitialize creates the
    // PhysX instance. The setting is read while the PhysX SDK is created and
    // cannot be turned on afterwards.
    if (cmdLine.enableNvtx)
    {
#if CARB_PLATFORM_WINDOWS
        ::_putenv_s("OVPHYSX_NVTX", "1");
#else
        ::setenv("OVPHYSX_NVTX", "1", 1);
#endif
    }

    bmInitialize(cmdLine.sanity, cmdLine.dataFolder, cmdLine.numThreads, cmdLine.forceGpu, cmdLine.directGpu,
        cmdLine.profile,
        cmdLine.enableTracy, kitArguments.data(), static_cast<uint32_t>(kitArguments.size()));

    // BmGlobals leaves the shared instance null when ovphysx_initialize() or
    // PhysX::create() fails, having only printed a diagnostic. Every row in
    // this binary measures ovphysx, so nothing can be published. The fault is
    // recorded so the process exits non-zero, before any row runs. The name is
    // not a registered row, so it cannot suppress one.
    if (BmGlobals::getInstance().getPhysX() == nullptr)
    {
        bmRecordFailure("<bootstrap>", "ovphysx bootstrap failed; no benchmark can run and no row is published");
        bmTerminate();
        return;
    }

    TimingDiagnosticsWriter timingDiagnosticsWriter(cmdLine.timingDiagnosticsFile);
    if (!timingDiagnosticsWriter.isReady())
    {
        bmRecordFailure("<timing-diagnostics>", "%s", timingDiagnosticsWriter.getError().c_str());
        bmTerminate();
        return;
    }

    // The device/thread postfix is computed before filtering. Reported record
    // names carry it (for example "Step.x_gpu" + "_GPU" -> "Step.x_gpu_GPU"), but
    // --filter is matched against the undecorated registered name. FrameCore's
    // regression sanity step re-runs a flagged test by its reported name, which
    // would otherwise match nothing and produce an empty report. A trailing copy
    // of the active postfix is therefore stripped from the filter. The postfix is
    // re-applied to the reported name below, so the emitted record name is
    // unchanged.
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

    std::vector<uint64_t> times, runs, actualTimes;
    times.reserve(2048);
    runs.reserve(2048);

    BmRecord* records = new BmRecord[reg.size()];
    std::vector<TimingDiagnostics> timingDiagnostics;
    if (timingDiagnosticsWriter.isEnabled())
    {
        actualTimes.reserve(2048);
        timingDiagnostics.resize(reg.size());
    }

    const uint32_t minRuns = 3; // always leaves one slowest and one fastest run to discard

    // Apply the device/thread postfix to the reported names.
    for (uint32_t i = 0; i < reg.size(); i++)
    {
        reg[i]->addPostfix(postfix.c_str());
    }

    for (uint32_t i = 0; i < reg.size(); i++)
    {
        BmRecord& record = records[i];
        std::unique_ptr<BmBenchmark> benchmark(reg[i]->create());
        BmBenchmark& b = *benchmark;

        // record.executed stays false, so the emit loop below publishes nothing
        // for this row. Device gating is the common reason to land here and is
        // not a failure, so the exit code is untouched. A bootstrap failure is
        // recorded above, before any row is created.
        if (!b.isValid())
        {
            printFormatted("Benchmark %s failed to initialize, skipping.\n", reg[i]->getName());
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
        const bool singleActualRun = runCount == 0;

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

        // One unrecorded warm-up run with stepCount steps initializes the instruction
        // cache, which reduces the noise when running multiple benchmarks.
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
            // Run the test for (s steps) x (r runs).
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
            // Heavy scenes run only once.
            times.clear();
            b.startRun();

            for (uint32_t s = 0; s < stepCount; s++)
            {
                b.preStep();

                Trace::start(s, cmdLine);
                uint64_t stepTimeMS = uint64_t(b.timedStep() * 1e6f);
                Trace::stop(s, cmdLine);

                // Each sample is pushed three times so the trim below has enough data.
                times.push_back(stepTimeMS);
                times.push_back(stepTimeMS);
                times.push_back(stepTimeMS);

                record.time[s] = BmMin(record.time[s], stepTimeMS);
            }

            b.endRun();

            runCount = 3;
        }

        if (timingDiagnosticsWriter.isEnabled())
        {
            if (singleActualRun)
            {
                // The heavy-scene path duplicates each sample only for the trim
                // calculation. Each timedStep call is counted once here.
                actualTimes.clear();
                for (uint32_t s = 0; s < stepCount; ++s)
                {
                    actualTimes.push_back(times[s * 3]);
                }
                timingDiagnostics[i] = computeTimingDiagnostics(actualTimes);
            }
            else
            {
                timingDiagnostics[i] = computeTimingDiagnostics(times);
            }
        }

        // Transpose the array from [R][S] to [S][R].
        runs.resize(times.size());
        for (uint32_t s = 0; s < stepCount; s++)
            for (uint32_t r = 0; r < runCount; r++)
                runs[r + runCount * s] = times[r * stepCount + s];

        for (uint32_t s = 0; s < stepCount; s++)
        {
            // Sort the section of the array for this step.
            std::sort(runs.begin() + s * runCount, runs.begin() + (s + 1) * runCount);

            // Keep the inclusive zero-based index range max(1, floor(0.2 * runCount))
            // through min(floor(0.7 * runCount), runCount - 2), discarding at least one run at each end.
            uint32_t minRun = BmMax<uint32_t>(1, uint32_t(runCount * 0.2f));
            uint32_t maxRun = BmMax<uint32_t>(BmMin(uint32_t(runCount * 0.7f), runCount - 2), 1);
            double mean = 0.0f;

#define GETTIME(r, s) runs[r + s * runCount]

            // Mean over the kept runs for this step.
            for (uint32_t r = minRun; r <= maxRun; r++)
            {
                mean += GETTIME(r, s);
            }
            mean *= 1.0f / (maxRun - minRun + 1);
            record.time[s] = uint64_t(mean);

            // Population standard deviation over the kept runs. The sum of squared
            // deviations is divided by the kept count before the square root, since
            // the root of the bare sum overstates the spread by sqrt(keptRuns) and
            // makes spreads incomparable between run counts. Population rather than
            // sample, because the kept set is an order-statistic trim of a fully
            // observed set, and a three-run configuration keeps exactly one run,
            // where dividing by keptRuns - 1 divides by zero.
            uint64_t sumSquares = 0;
            uint32_t keptRuns = 0;
            const uint64_t avg = record.time[s];
            for (uint32_t r = minRun; r <= maxRun; r++)
            {
                const uint64_t value = GETTIME(r, s);

                if (avg >= value)
                    sumSquares += (avg - value) * (avg - value);
                else
                    sumSquares += (value - avg) * (value - avg);

                keptRuns++;
            }
            record.stdDev[s] =
                keptRuns ? (uint64_t)std::sqrt((double)sumSquares / (double)keptRuns) : 0;
        }

        record.executed = true;

        // The benchmark is destroyed before malloc_trim. The unique_ptr also
        // guarantees cleanup if a benchmark operation throws.
        benchmark.reset();

#if !CARB_PLATFORM_WINDOWS
        malloc_trim(0);
#endif
    }

    BmOutput* output = new BmOutput(cmdLine.regenerate, cmdLine.slop, true, cmdLine.reportFile, printCb);
    output->printHeaders();
    bool timingDiagnosticsFailureRecorded = false;
    for (uint32_t i = 0; i < reg.size(); i++)
    {
        // A row the harness never ran has no sample to publish. Emitting its
        // default-constructed record would print a zero that reads as a real
        // measurement and, under --regenerate, write a zero baseline entry that
        // silently disables comparison for that row from then on.
        if (!records[i].executed)
        {
            printFormatted("%s: skipped -- no timing record published", reg[i]->getName());
            continue;
        }
        // Discard the sample for any row that recorded a failure. Publishing it
        // would advertise an unusually fast number produced by work that did
        // not complete.
        if (bmRowHasFailure(reg[i]->getName()))
        {
            printFormatted("%s: FAILED -- timing record discarded", reg[i]->getName());
            continue;
        }
        if (cmdLine.detail)
            output->dump(reg[i]->getName(), records[i]);
        // A false return is a local non-regenerate baseline breach. emit()
        // already printed and possibly wrote the failed comparison, so this only
        // records the process failure. FrameCore always passes --regenerate and
        // does not compare baselines.
        if (!output->emit(reg[i]->getName(), bmGetDefaultResult(records[i])))
        {
            bmRecordFailure(reg[i]->getName(), "baseline comparison failed");
            continue;
        }
        if (!timingDiagnosticsFailureRecorded && timingDiagnosticsWriter.isEnabled() &&
            !timingDiagnosticsWriter.append(reg[i]->getName(), timingDiagnostics[i]))
        {
            bmRecordFailure(reg[i]->getName(), "%s", timingDiagnosticsWriter.getError().c_str());
            timingDiagnosticsFailureRecorded = true;
            continue;
        }
    }

    if (!timingDiagnosticsWriter.finish() && !timingDiagnosticsFailureRecorded)
    {
        bmRecordFailure("<timing-diagnostics>", "%s", timingDiagnosticsWriter.getError().c_str());
    }

    delete output;

    delete[] records;
    bmTerminate();
}
} // namespace BenchmarkHarness


int main(int argc, char** argv)
{
    try
    {
        BenchmarkHarness::harnessImpl(argc, argv, NULL);

        // Any setup, operation-sequence, correctness-gate or baseline failure
        // makes the process exit non-zero. scripts/test_benchmarks_cpp.cmake
        // turns a non-zero pass exit code into a hard FATAL_ERROR, which makes
        // the suite fail closed end to end. This sits inside the exception
        // boundary because an exception and a recorded failure are different
        // faults and both must exit non-zero.
        bmPrintFailureSummary();
        return bmFailureCount() ? 1 : 0;
    }
    catch (const std::exception& error)
    {
        printFormatted("Benchmark run failed: %s\n", error.what());
        return 1;
    }
    catch (...)
    {
        printFormatted("Benchmark run failed with an unknown error\n");
        return 1;
    }
}
