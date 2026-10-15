// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-OVD-DECODE-001
// @covers AC-1 AC-2 AC-3
#pragma once
#include <cstdint>
#include <string>
#include <vector>
namespace omni::physx::diagnostics
{
enum class DecodeResult : uint32_t { Success, Invalid, Unsupported, Malformed, Limit, Error };
struct DecodeLimits
{
    uint64_t inputBytes = 64*1024*1024;
    uint32_t records = 1000000, payloadBytes = 16*1024*1024, stringBytes = 2048;
};
struct DecodeDiagnostic { uint64_t offset = 0; char message[160]{}; };
struct OvdRecord
{
    uint64_t offset = 0, context = 0, object = 0, timestamp = 0;
    uint32_t command = 0, classHandle = 0, baseClass = 0, attribute = 0;
    uint32_t dataType = 0, elements = 0, attributeClass = 0, enumValue = 0;
    uint32_t line = 0, messageType = 0;
    std::string name, file;
    std::vector<uint32_t> attributePath;
    std::vector<uint8_t> payload;
};
struct OvdDocument
{
    uint32_t major=0, minor=0, patch=0;
    std::vector<OvdRecord> records;
};
// Stopped bounded utility. Little-endian format 0.1.42 through 0.4.0 only.
// SDK eREGISTER_ENUM is reserved and unsupported; enum values use attribute
// registration, as in the SDK writer. Attribute paths are retained in full.
// Command/schema payload interpretation belongs to the consumer. No semantic
// object lookup, physical-time inference, USD or viewer creation happens here.
DecodeResult decodeOvd(const void* data, uint64_t size, const DecodeLimits& limits,
                      OvdDocument& output, DecodeDiagnostic& diagnostic) noexcept;
}
