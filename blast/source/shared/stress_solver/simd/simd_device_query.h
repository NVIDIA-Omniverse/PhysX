// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2022-2024 NVIDIA Corporation. All rights reserved.

#pragma once

#include <utility>
#include <stdio.h>


inline static constexpr uint32_t
instSetCode(uint8_t fn, uint8_t bitset, uint8_t bit)
{
    return (uint32_t)fn << 16 | (uint32_t)bitset << 8 | (uint32_t)bit;
}

inline static void
extractInstSetBitsetAndBit(int& fn, int& bitset, int& bit, uint32_t code)
{
    fn = (int)(code >> 16);
    bitset = (int)(code >> 8)&0xff;
    bit = (int)(code & 0xff);
}

struct InstructionSet
{
    enum Enum
    {
        MMX = instSetCode(1, 3, 23),
        SSE = instSetCode(1, 3, 25),
        SSE2 = instSetCode(1, 3, 26),
        SSE3 = instSetCode(1, 2, 0),
        SSSE3 = instSetCode(1, 2, 9),
        SSE4_1 = instSetCode(1, 2, 19),
        SSE4_2 = instSetCode(1, 2, 20),
        OSXSAVE = instSetCode(1, 2, 27),
        AVX = instSetCode(1, 2, 28),
        AVX2 = instSetCode(7, 1, 5),
        FMA3 = instSetCode(1, 2, 12),
        AVX512F = instSetCode(7, 1, 16),
        AVX512PF = instSetCode(7, 1, 26),
        AVX512ER = instSetCode(7, 1, 27),
        AVX512CD = instSetCode(7, 1, 28)
    };
};

#define InstructionSetEntry(_name) { #_name, InstructionSet::_name }
constexpr std::pair<const char*, uint32_t> sInstructionSetLookup[] =
{
    InstructionSetEntry(MMX),
    InstructionSetEntry(SSE),
    InstructionSetEntry(SSE2),
    InstructionSetEntry(SSE3),
    InstructionSetEntry(SSSE3),
    InstructionSetEntry(SSE4_1),
    InstructionSetEntry(SSE4_2),
    InstructionSetEntry(OSXSAVE),
    InstructionSetEntry(AVX),
    InstructionSetEntry(AVX2),
    InstructionSetEntry(FMA3),
    InstructionSetEntry(AVX512F),
    InstructionSetEntry(AVX512PF),
    InstructionSetEntry(AVX512ER),
    InstructionSetEntry(AVX512CD),
};


#if NV_WINDOWS_FAMILY
#include <intrin.h> // for __cpuidex
inline void cpuid(int cpui[4], int fn) { __cpuidex(cpui, fn, 0); }
inline bool os_supports_avx_restore() { return ((uint32_t)_xgetbv(0) & 6) == 6; }
#else
#include <cpuid.h> // for __cpuid_count
inline void cpuid(int cpui[4], int fn) { __cpuid_count(fn, 0, cpui[0], cpui[1], cpui[2], cpui[3]); }
inline bool os_supports_avx_restore()
{
    uint32_t xcr0;
    __asm__("xgetbv" : "=a" (xcr0) : "c" (0) : "%edx");
    return (xcr0 & 6) == 6;
}
#endif

static bool
device_supports_instruction_set(uint32_t inst_set)
{
    int fn, bitset, bit;
    extractInstSetBitsetAndBit(fn, bitset, bit, inst_set);

    int cpui[4];
    cpuid(cpui, 0);

    if (cpui[0] < fn) return false;

    cpuid(cpui, fn);

    return !!((cpui[bitset] >> bit) & 1);
}

static void
print_supported_instruction_sets()
{
    printf("Supported instruction sets:\n");
    for (std::pair<const char*, uint32_t> entry : sInstructionSetLookup)
    {
        printf("%s: %s\n", entry.first, device_supports_instruction_set(entry.second) ? "yes" : "no");
    }
}
