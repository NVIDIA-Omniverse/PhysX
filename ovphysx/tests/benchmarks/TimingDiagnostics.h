// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-BENCHMARK-002
 * @covers AC-1 AC-2 AC-3
 */

#ifndef TIMING_DIAGNOSTICS_H
#define TIMING_DIAGNOSTICS_H

#include <cstdint>
#include <fstream>
#include <ostream>
#include <string>
#include <vector>

namespace BenchmarkHarness
{

struct TimingDiagnostics
{
    uint64_t allStepsCount = 0;
    double allStepsMeanUs = 0.0;
    double allStepsPopulationSdUs = 0.0;
    uint64_t allStepsMinUs = 0;
    uint64_t allStepsMaxUs = 0;
};

TimingDiagnostics computeTimingDiagnostics(const std::vector<uint64_t>& samples);

bool writeTimingDiagnosticsJsonLine(std::ostream& stream, const char* command, const TimingDiagnostics& diagnostics);

class TimingDiagnosticsWriter
{
public:
    explicit TimingDiagnosticsWriter(const char* path);

    bool isEnabled() const;
    bool isReady() const;
    bool append(const char* command, const TimingDiagnostics& diagnostics);
    bool finish();
    const std::string& getError() const;

private:
    void setError(const std::string& error);

    bool mEnabled;
    bool mFinished;
    std::ofstream mStream;
    std::string mError;
};

} // namespace BenchmarkHarness

#endif
