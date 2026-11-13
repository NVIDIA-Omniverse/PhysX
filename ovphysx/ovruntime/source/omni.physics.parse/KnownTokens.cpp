// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-NEWTON-JOINT-001
 * @covers AC-1
 *
 * @implements REQ-PARSE-CORE-002
 * @covers AC-2
 *
 * @implements REQ-SPLINE-CURVE-001
 * @covers AC-6
 *
 * @implements REQ-PARSE-JOINT-002
 * @covers AC-2
 *
 * @implements REQ-PARSE-JOINT-005
 * @covers AC-5
 *
 * @implements REQ-SIM-SCENEQUERY-001
 * @covers AC-3
 *
 * @implements REQ-POPULATION-CONTRACT-001
 * @covers AC-11
 *
 * @implements REQ-LOAD-TOKENS-001
 * @covers AC-5
 */

#include <omni/physics/parse/KnownTokens.h>
#include <omni/physics/parse/IPhysicsSource.h>

#include <atomic>

namespace omni::physics::parse
{

namespace
{
std::atomic<uint64_t> gInternCallCount{ 0 };
std::atomic<uint64_t> gVocabularyInternCount{ 0 };
}

uint64_t internCallCount()
{
    return gInternCallCount.load(std::memory_order_relaxed);
}

uint64_t vocabularyInternCount()
{
    return gVocabularyInternCount.load(std::memory_order_relaxed);
}

void KnownTokens::intern(const IPhysicsSource& source)
{
    if (const KnownTokens* cached = source.knownTokens(); cached && cached != this)
    {
        gInternCallCount.fetch_add(1, std::memory_order_relaxed);
        *this = *cached;
    }
    else
        internVocabulary(source);
}

void KnownTokens::internVocabulary(const IPhysicsSource& source)
{
    gInternCallCount.fetch_add(1, std::memory_order_relaxed);
    gVocabularyInternCount.fetch_add(1, std::memory_order_relaxed);
#define PHYSICS_KNOWN_TOKEN(member, name) member = source.internToken(name);
#include <omni/physics/parse/KnownTokenNames.inl>
#include <omni/physics/parse/KnownTokenOtherNames.inl>
#undef PHYSICS_KNOWN_TOKEN
}

} // namespace omni::physics::parse
