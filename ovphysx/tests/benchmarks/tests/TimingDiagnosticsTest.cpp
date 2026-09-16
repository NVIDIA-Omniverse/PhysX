// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-BENCHMARK-002
 * @covers AC-1 AC-2 AC-3
 */

#include "../TimingDiagnostics.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace
{

bool expect(bool condition, const char* message)
{
    if (!condition)
    {
        std::fprintf(stderr, "%s\n", message);
    }
    return condition;
}

bool expectNear(double actual, double expected, double tolerance, const char* message)
{
    return expect(std::fabs(actual - expected) <= tolerance, message);
}

bool expectOneThroughOneHundredSummary()
{
    std::vector<uint64_t> samples;
    for (uint64_t value = 1; value <= 100; ++value)
    {
        samples.push_back(value);
    }
    const std::vector<uint64_t> original = samples;
    const BenchmarkHarness::TimingDiagnostics diagnostics = BenchmarkHarness::computeTimingDiagnostics(samples);

    bool ok = true;
    ok = expect(diagnostics.allStepsCount == 100, "all-steps count is incorrect") && ok;
    ok = expect(diagnostics.allStepsMeanUs == 50.5, "fractional all-steps mean is incorrect") && ok;
    ok = expectNear(diagnostics.allStepsPopulationSdUs, std::sqrt(833.25), 1.0e-12,
                    "population all-steps standard deviation is incorrect") &&
         ok;
    ok = expect(diagnostics.allStepsMinUs == 1, "all-steps minimum is incorrect") && ok;
    ok = expect(diagnostics.allStepsMaxUs == 100, "all-steps maximum is incorrect") && ok;
    ok = expect(samples == original, "timing diagnostics calculation changed its input") && ok;
    return ok;
}

bool expectConstantSummary()
{
    const std::vector<uint64_t> samples(8, 42);
    const BenchmarkHarness::TimingDiagnostics diagnostics = BenchmarkHarness::computeTimingDiagnostics(samples);
    return expect(diagnostics.allStepsCount == 8, "constant-sample count is incorrect") &&
           expect(diagnostics.allStepsMeanUs == 42.0, "constant-sample mean is incorrect") &&
           expect(diagnostics.allStepsPopulationSdUs == 0.0, "constant-sample standard deviation is not zero") &&
           expect(diagnostics.allStepsMinUs == 42, "constant-sample minimum is incorrect") &&
           expect(diagnostics.allStepsMaxUs == 42, "constant-sample maximum is incorrect");
}

bool expectEmptySummary()
{
    const BenchmarkHarness::TimingDiagnostics diagnostics = BenchmarkHarness::computeTimingDiagnostics({});
    return expect(diagnostics.allStepsCount == 0, "empty-sample count is not zero") &&
           expect(diagnostics.allStepsMeanUs == 0.0, "empty-sample mean is not zero") &&
           expect(diagnostics.allStepsPopulationSdUs == 0.0, "empty-sample standard deviation is not zero") &&
           expect(diagnostics.allStepsMinUs == 0, "empty-sample minimum is not zero") &&
           expect(diagnostics.allStepsMaxUs == 0, "empty-sample maximum is not zero");
}

bool expectLargeValueSummary()
{
    const uint64_t maximum = std::numeric_limits<uint64_t>::max();
    const std::vector<uint64_t> samples{ 0, maximum };
    const BenchmarkHarness::TimingDiagnostics diagnostics = BenchmarkHarness::computeTimingDiagnostics(samples);
    const double expectedHalfRange = static_cast<double>(static_cast<long double>(maximum) / 2.0L);

    bool ok = true;
    ok = expect(diagnostics.allStepsCount == 2, "large-sample count is incorrect") && ok;
    ok = expect(std::isfinite(diagnostics.allStepsMeanUs), "large-sample mean overflowed") && ok;
    ok = expect(std::isfinite(diagnostics.allStepsPopulationSdUs), "large-sample standard deviation overflowed") && ok;
    ok = expectNear(diagnostics.allStepsMeanUs, expectedHalfRange, 0.0, "large-sample mean is incorrect") && ok;
    ok = expectNear(diagnostics.allStepsPopulationSdUs, expectedHalfRange, 0.0,
                    "large-sample standard deviation is incorrect") &&
         ok;
    ok = expect(diagnostics.allStepsMinUs == 0, "large-sample minimum is incorrect") && ok;
    ok = expect(diagnostics.allStepsMaxUs == maximum, "large-sample maximum is incorrect") && ok;
    return ok;
}

bool expectJsonLine(const BenchmarkHarness::TimingDiagnostics& diagnostics, std::string& expected)
{
    std::ostringstream stream;
    const bool wrote = BenchmarkHarness::writeTimingDiagnosticsJsonLine(stream, "Step.\"quoted\"\\path\n", diagnostics);
    expected =
        "{\"schema_version\":1,\"cmd\":\"Step.\\\"quoted\\\"\\\\path\\n\",\"all_steps_count\":2,"
        "\"all_steps_mean_us\":1.5,\"all_steps_sd_us\":0.5,\"all_steps_min_us\":1,"
        "\"all_steps_max_us\":2}\n";
    return expect(wrote, "JSONL serialization failed") &&
           expect(stream.str() == expected, "JSONL object or command escaping is incorrect");
}

bool expectWriter(const char* path, const BenchmarkHarness::TimingDiagnostics& diagnostics, const std::string& expected)
{
    std::remove(path);
    BenchmarkHarness::TimingDiagnosticsWriter writer(path);
    bool ok = expect(writer.isEnabled(), "timing diagnostics writer was not enabled") &&
              expect(writer.isReady(), "timing diagnostics output file did not open");
    ok = expect(writer.append("Step.\"quoted\"\\path\n", diagnostics), "timing diagnostics append failed") && ok;
    ok = expect(writer.append("Step.\"quoted\"\\path\n", diagnostics), "second timing diagnostics append failed") && ok;
    ok = expect(writer.finish(), "timing diagnostics close failed") && ok;

    std::ifstream stream(path, std::ios::in | std::ios::binary);
    const std::string contents((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
    ok = expect(stream.good() || stream.eof(), "timing diagnostics output file could not be read") && ok;
    ok = expect(contents == expected + expected,
                "timing diagnostics writer did not append one JSONL object per call") &&
         ok;
    stream.close();
    std::remove(path);
    return ok;
}

bool expectOpenFailure()
{
    BenchmarkHarness::TimingDiagnosticsWriter writer("");
    return expect(writer.isEnabled(), "empty timing diagnostics path did not enable the writer") &&
           expect(!writer.isReady(), "empty timing diagnostics path did not report an open failure") &&
           expect(!writer.getError().empty(), "timing diagnostics open failure has no error text");
}

bool expectWriteFailure(const BenchmarkHarness::TimingDiagnostics& diagnostics)
{
#if defined(_WIN32)
    (void)diagnostics;
    return true;
#else
    std::ifstream fullDevice("/dev/full", std::ios::in | std::ios::binary);
    if (!fullDevice.good())
    {
        return true;
    }
    fullDevice.close();

    BenchmarkHarness::TimingDiagnosticsWriter writer("/dev/full");
    return expect(writer.isReady(), "the write-failure test could not open /dev/full") &&
           expect(!writer.append("Step.write_failure", diagnostics),
                  "timing diagnostics write failure was not reported") &&
           expect(!writer.getError().empty(), "timing diagnostics write failure has no error text");
#endif
}

bool expectDisabledWriter(const BenchmarkHarness::TimingDiagnostics& diagnostics)
{
    BenchmarkHarness::TimingDiagnosticsWriter writer(nullptr);
    return expect(!writer.isEnabled(), "null timing diagnostics path enabled the writer") &&
           expect(writer.isReady(), "disabled timing diagnostics writer is not ready") &&
           expect(writer.append("Step.disabled", diagnostics), "disabled timing diagnostics append failed") &&
           expect(writer.finish(), "disabled timing diagnostics finish failed") &&
           expect(writer.getError().empty(), "disabled timing diagnostics writer recorded an error");
}

} // namespace


int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::fprintf(stderr, "expected one temporary output path\n");
        return 1;
    }

    const std::vector<uint64_t> samples{ 1, 2 };
    const BenchmarkHarness::TimingDiagnostics diagnostics = BenchmarkHarness::computeTimingDiagnostics(samples);
    std::string expected;

    bool ok = expectOneThroughOneHundredSummary();
    ok = expectConstantSummary() && ok;
    ok = expectEmptySummary() && ok;
    ok = expectLargeValueSummary() && ok;
    ok = expectJsonLine(diagnostics, expected) && ok;
    ok = expectWriter(argv[1], diagnostics, expected) && ok;
    ok = expectOpenFailure() && ok;
    ok = expectWriteFailure(diagnostics) && ok;
    ok = expectDisabledWriter(diagnostics) && ok;
    return ok ? 0 : 1;
}
