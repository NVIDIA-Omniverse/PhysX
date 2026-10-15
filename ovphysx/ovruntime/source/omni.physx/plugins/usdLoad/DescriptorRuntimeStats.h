// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-PARSE-PARALLEL-001
 * @covers AC-4
 */
#pragma once
#include <omni/physics/parse/DescriptorPipelineStats.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
namespace omni::physx::usdparser
{
// Opt-in coordinator timing; this helper never enters a descriptor worker.
class DescriptorRuntimeTimer
{
public:
    explicit DescriptorRuntimeTimer(const char* phase) : mPhase(phase), mEnabled(enabled()), mTimer(mEnabled ? &mSeconds : nullptr) {}
    ~DescriptorRuntimeTimer()
    {
        mTimer.finish();
        if (mEnabled) std::fprintf(stderr, "Descriptor runtime: %s=%.6f\n", mPhase, mSeconds);
    }
private:
    static bool enabled()
    {
        const char* value = std::getenv("OVRUNTIME_DESCRIPTOR_STATS");
        return value && std::strcmp(value, "1") == 0;
    }
    const char* mPhase;
    bool mEnabled;
    double mSeconds = 0.0;
    omni::physics::parse::DescriptorPhaseTimer mTimer;
};
}
