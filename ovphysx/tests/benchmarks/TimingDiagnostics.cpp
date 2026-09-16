// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-BENCHMARK-002
 * @covers AC-1 AC-2 AC-3
 */

#include "TimingDiagnostics.h"

#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>

namespace BenchmarkHarness
{
namespace
{

void writeJsonString(std::ostream& stream, const char* value)
{
    static const char hexDigits[] = "0123456789abcdef";

    stream.put('"');
    const unsigned char* current = reinterpret_cast<const unsigned char*>(value ? value : "");
    while (*current != 0)
    {
        const unsigned char character = *current++;
        switch (character)
        {
        case '"':
            stream << "\\\"";
            break;
        case '\\':
            stream << "\\\\";
            break;
        case '\b':
            stream << "\\b";
            break;
        case '\f':
            stream << "\\f";
            break;
        case '\n':
            stream << "\\n";
            break;
        case '\r':
            stream << "\\r";
            break;
        case '\t':
            stream << "\\t";
            break;
        default:
            if (character < 0x20)
            {
                stream << "\\u00" << hexDigits[character >> 4] << hexDigits[character & 0x0f];
            }
            else
            {
                stream.put(static_cast<char>(character));
            }
            break;
        }
    }
    stream.put('"');
}

} // namespace


TimingDiagnostics computeTimingDiagnostics(const std::vector<uint64_t>& samples)
{
    TimingDiagnostics diagnostics;
    if (samples.empty())
    {
        return diagnostics;
    }

    diagnostics.allStepsCount = static_cast<uint64_t>(samples.size());
    diagnostics.allStepsMinUs = samples[0];
    diagnostics.allStepsMaxUs = samples[0];

    long double sum = 0.0L;
    for (size_t i = 0; i < samples.size(); ++i)
    {
        const uint64_t sample = samples[i];
        sum += static_cast<long double>(sample);
        if (sample < diagnostics.allStepsMinUs)
        {
            diagnostics.allStepsMinUs = sample;
        }
        if (sample > diagnostics.allStepsMaxUs)
        {
            diagnostics.allStepsMaxUs = sample;
        }
    }

    const long double mean = sum / static_cast<long double>(samples.size());
    long double squaredDeviations = 0.0L;
    for (size_t i = 0; i < samples.size(); ++i)
    {
        const long double difference = static_cast<long double>(samples[i]) - mean;
        squaredDeviations += difference * difference;
    }

    diagnostics.allStepsMeanUs = static_cast<double>(mean);
    diagnostics.allStepsPopulationSdUs =
        static_cast<double>(std::sqrt(squaredDeviations / static_cast<long double>(samples.size())));
    return diagnostics;
}


bool writeTimingDiagnosticsJsonLine(std::ostream& stream, const char* command, const TimingDiagnostics& diagnostics)
{
    stream.imbue(std::locale::classic());
    stream << "{\"schema_version\":1,\"cmd\":";
    writeJsonString(stream, command);
    stream << ",\"all_steps_count\":" << diagnostics.allStepsCount
           << ",\"all_steps_mean_us\":" << std::setprecision(std::numeric_limits<double>::max_digits10)
           << diagnostics.allStepsMeanUs << ",\"all_steps_sd_us\":" << diagnostics.allStepsPopulationSdUs
           << ",\"all_steps_min_us\":" << diagnostics.allStepsMinUs
           << ",\"all_steps_max_us\":" << diagnostics.allStepsMaxUs << "}\n";
    return stream.good();
}


TimingDiagnosticsWriter::TimingDiagnosticsWriter(const char* path) : mEnabled(path != nullptr), mFinished(false)
{
    if (!mEnabled)
    {
        return;
    }

    mStream.imbue(std::locale::classic());
    mStream.open(path, std::ios::out | std::ios::binary | std::ios::trunc);
    if (!mStream.is_open())
    {
        setError(std::string("could not open timing diagnostics file '") + path + "'");
    }
}


bool TimingDiagnosticsWriter::isEnabled() const
{
    return mEnabled;
}


bool TimingDiagnosticsWriter::isReady() const
{
    return !mEnabled || (mStream.is_open() && mError.empty());
}


bool TimingDiagnosticsWriter::append(const char* command, const TimingDiagnostics& diagnostics)
{
    if (!mEnabled)
    {
        return true;
    }
    if (mFinished)
    {
        setError("timing diagnostics file is already closed");
        return false;
    }
    if (!mError.empty())
    {
        return false;
    }
    if (diagnostics.allStepsCount == 0)
    {
        setError("cannot write timing diagnostics without an actual timedStep sample");
        return false;
    }

    std::ostringstream line;
    if (!writeTimingDiagnosticsJsonLine(line, command, diagnostics))
    {
        setError("could not serialize timing diagnostics");
        return false;
    }

    const std::string serialized = line.str();
    mStream.write(serialized.data(), static_cast<std::streamsize>(serialized.size()));
    mStream.flush();
    if (!mStream.good())
    {
        setError("could not write timing diagnostics file");
        return false;
    }
    return true;
}


bool TimingDiagnosticsWriter::finish()
{
    if (!mEnabled || mFinished)
    {
        return mError.empty();
    }

    mStream.flush();
    mStream.close();
    mFinished = true;
    if (mStream.fail())
    {
        setError("could not close timing diagnostics file");
    }
    return mError.empty();
}


const std::string& TimingDiagnosticsWriter::getError() const
{
    return mError;
}


void TimingDiagnosticsWriter::setError(const std::string& error)
{
    if (mError.empty())
    {
        mError = error;
    }
}

} // namespace BenchmarkHarness
