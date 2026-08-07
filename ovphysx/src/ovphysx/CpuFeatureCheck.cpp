// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "internal/CpuFeatureCheck.h"

#if defined(__x86_64__) || defined(_M_X64) || defined(__amd64__)

#if defined(_MSC_VER)
#include <intrin.h>
#else
#include <cpuid.h>
#endif

namespace ovphysx {
namespace internal {
namespace {

bool queryCpuid(unsigned int leaf, unsigned int subleaf, unsigned int outRegs[4])
{
#if defined(_MSC_VER)
    int regs[4] = { 0, 0, 0, 0 };
    __cpuidex(regs, static_cast<int>(leaf), static_cast<int>(subleaf));
    outRegs[0] = static_cast<unsigned int>(regs[0]);
    outRegs[1] = static_cast<unsigned int>(regs[1]);
    outRegs[2] = static_cast<unsigned int>(regs[2]);
    outRegs[3] = static_cast<unsigned int>(regs[3]);
    return true;
#else
    return __get_cpuid_count(leaf, subleaf, &outRegs[0], &outRegs[1], &outRegs[2], &outRegs[3]) != 0;
#endif
}

unsigned long long readXcr0()
{
#if defined(_MSC_VER)
    return static_cast<unsigned long long>(_xgetbv(0));
#else
    unsigned int eax = 0;
    unsigned int edx = 0;
    __asm__ volatile("xgetbv" : "=a"(eax), "=d"(edx) : "c"(0));
    return (static_cast<unsigned long long>(edx) << 32) | eax;
#endif
}

} // namespace

bool cpuSupportsAvx()
{
    unsigned int regs[4] = { 0, 0, 0, 0 };
    if (!queryCpuid(0, 0, regs))
    {
        return false;
    }

    const unsigned int maxLeaf = regs[0];
    if (maxLeaf < 1)
    {
        return false;
    }

    if (!queryCpuid(1, 0, regs))
    {
        return false;
    }

    const unsigned int ecx = regs[2];
    const bool hwAvx = (ecx & (1u << 28)) != 0;
    const bool osxsave = (ecx & (1u << 27)) != 0;
    if (!hwAvx || !osxsave)
    {
        return false;
    }

    const unsigned long long xcr0 = readXcr0();
    return (xcr0 & 0x6ULL) == 0x6ULL;
}

} // namespace internal
} // namespace ovphysx

#else

namespace ovphysx {
namespace internal {

bool cpuSupportsAvx()
{
    return true;
}

} // namespace internal
} // namespace ovphysx

#endif
