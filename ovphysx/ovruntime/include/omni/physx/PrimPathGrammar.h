// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-BUILD-UNIBUILD-001
 * @covers AC-3
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

// pxr-free stand-in for SdfPath::IsValidPathString + IsAbsoluteRootOrPrimPath, shared by every
// runtime gate that must reject a malformed path string before it reaches a source lookup
// (getObjectType) or the replicator (clone).
//
// The grammar is deliberately a SUPERSET of what USD accepts: a wrong rejection is not a missing
// warning but a silently wrong answer (getObjectType reports eInvalid without a lookup) or a hard
// clone failure. USD prim names are UTF-8 XID identifiers, not C identifiers, so every well-formed
// non-ASCII UTF-8 sequence is accepted wherever an identifier character may appear; the exact XID
// classification stays USD's job. Only structurally malformed UTF-8 (which no SdfPath can hold)
// and the ASCII syntax USD itself rejects for a prim path are refused.
namespace omni
{
namespace physx
{

// Structural UTF-8 check (RFC 3629 table): correct lead/continuation lengths, no overlongs
// (C0/C1, E0 80-9F, F0 80-8F), no surrogates (ED A0-BF), nothing above U+10FFFF (F4 90+, F5+).
inline bool isWellFormedUtf8(std::string_view s)
{
    const size_t n = s.size();
    size_t i = 0;
    while (i < n)
    {
        const uint8_t b0 = static_cast<uint8_t>(s[i]);
        if (b0 < 0x80)
        {
            ++i;
            continue;
        }
        size_t len = 0;
        uint8_t lo = 0x80, hi = 0xBF; // bounds for the first continuation byte
        if (b0 >= 0xC2 && b0 <= 0xDF)
        {
            len = 2;
        }
        else if (b0 >= 0xE0 && b0 <= 0xEF)
        {
            len = 3;
            if (b0 == 0xE0)
                lo = 0xA0; // overlong
            else if (b0 == 0xED)
                hi = 0x9F; // surrogates
        }
        else if (b0 >= 0xF0 && b0 <= 0xF4)
        {
            len = 4;
            if (b0 == 0xF0)
                lo = 0x90; // overlong
            else if (b0 == 0xF4)
                hi = 0x8F; // > U+10FFFF
        }
        else
        {
            return false; // stray continuation, C0/C1, F5+
        }
        if (n - i < len)
            return false; // truncated
        const uint8_t b1 = static_cast<uint8_t>(s[i + 1]);
        if (b1 < lo || b1 > hi)
            return false;
        for (size_t k = 2; k < len; ++k)
        {
            const uint8_t bk = static_cast<uint8_t>(s[i + k]);
            if (bk < 0x80 || bk > 0xBF)
                return false;
        }
        i += len;
    }
    return true;
}

// Absolute prim path shape: leading '/', no empty segment (so no "//" and no trailing '/'), each
// segment identifier-shaped -- ASCII [A-Za-z_][A-Za-z0-9_]* with any non-ASCII UTF-8 byte accepted
// at any position -- and the whole string well-formed UTF-8. '.', ':', '[' and ']' (property and
// relational syntax) are never identifier characters. allowRoot decides whether "/" alone passes.
inline bool looksLikeAbsolutePrimPath(std::string_view path, bool allowRoot)
{
    if (path.empty() || path.front() != '/')
        return false;
    if (path.size() == 1)
        return allowRoot;
    if (!isWellFormedUtf8(path))
        return false;
    auto isIdentifierChar = [](char c, bool first)
    {
        if (static_cast<unsigned char>(c) >= 0x80)
            return true; // UTF-8 lead or continuation byte (well-formedness checked above)
        return c == '_' || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (!first && c >= '0' && c <= '9');
    };
    size_t segStart = 1;
    for (size_t i = 1; i <= path.size(); ++i)
    {
        if (i == path.size() || path[i] == '/')
        {
            if (i == segStart || !isIdentifierChar(path[segStart], true))
                return false;
            for (size_t j = segStart + 1; j < i; ++j)
            {
                if (!isIdentifierChar(path[j], false))
                    return false;
            }
            segStart = i + 1;
        }
    }
    return true;
}

} // namespace physx
} // namespace omni
