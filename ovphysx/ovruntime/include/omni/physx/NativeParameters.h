// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CUSTOM-PARAMETERS-001
// @covers AC-1 AC-2 AC-3
#pragma once
#include <cstdint>

namespace omni::physx::native
{
enum class ParameterKind : uint32_t { Float = 1, Double = 2, Int = 3, Bool = 4, Float3 = 5 };

// Fixed-layout values: only the field selected by kind is meaningful. Values
// contain no source pointers, USD objects, token handles or borrowed strings.
struct ParameterValue
{
    ParameterKind kind;
    uint32_t reserved = 0;
    union
    {
        float scalar;
        double real;
        int64_t integer;
        uint32_t boolean;
        float vector[3];
    };
};
struct ParameterSpec
{
    const char* name;
    ParameterKind kind;
    // Missing attributes fail when required. Otherwise use this explicit value;
    // authored/schema-provided values always take precedence over fallback.
    uint32_t required;
    ParameterValue fallback;
};

// For custom create/explicit parameter-refresh callbacks executing synchronously under the selected owner's
// admission. Other callers fail; this does not acquire a second
// execution lease. Source values are copied from that owner's current sealed
// input, never from a USD stage. Exact kinds are required (no numeric coercion).
// All outputs remain unchanged on failure except invalidParameter: index of a
// bad spec/missing/mistyped/nonfinite value, or UINT32_MAX for owner/admission.
// The caller owns output[parameterCount]. Specs/names remain valid through return.
bool copyParameters(uint64_t attachment, uint64_t objectKey, const ParameterSpec* specs,
    uint32_t parameterCount, ParameterValue* output, uint32_t& invalidParameter) noexcept;
}
