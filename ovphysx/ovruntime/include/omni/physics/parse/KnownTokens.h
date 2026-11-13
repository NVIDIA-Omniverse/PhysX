// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-NEWTON-JOINT-001
 * @covers AC-1
 *
 * @implements REQ-PARSE-CORE-001
 * @covers AC-1
 *
 * @implements REQ-SPLINE-CURVE-001
 * @covers AC-6
 *
 * @implements REQ-PARSE-CORE-002
 * @covers AC-2
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
 */

#pragma once

#include "Handles.h"

namespace omni::physics::parse
{

class IPhysicsSource;

// ---------------------------------------------------------------------------
// KnownTokens - well-known schema attribute names, populated as one batch and
// reused by its owner. AttachedStage owners populate the batch at source attach;
// ParseContext owners populate it lazily on first use. Parsers then do
// integer-compare lookups with zero string operations on hot paths.
//
// Populated by KnownTokens::intern() from the source's token table, or copied
// from the batch the source caches (IPhysicsSource::knownTokens()).
//
// The members are generated. KnownTokenNames.inl holds every name the physics
// population contract declares (physics_population_domain/, gen_contract.py):
// prim types, API schemas, instances and columns. A member is its name in camel
// case across ':' (physxScene:bounceThreshold -> physxSceneBounceThreshold), as
// usdGenSchema spells schema tokens; a prim type adds Type (Points ->
// pointsType) and a multi-apply template name reads
// <prefix>_MultipleApplyTemplate_<Rest>. KnownTokenOtherNames.inl is the
// hand-written rest: token values, metadata keys and the other names that are
// not contract data. To compare a new column, type or schema by token, add it
// to the contract and regenerate.
// ---------------------------------------------------------------------------

struct KnownTokens
{
#define PHYSICS_KNOWN_TOKEN(member, name) TokenId member;
#include "KnownTokenNames.inl"
#include "KnownTokenOtherNames.inl"
#undef PHYSICS_KNOWN_TOKEN

    // Fill this batch for source: a copy of the batch source caches when it has
    // one, since its TokenIds never change, else internVocabulary(source).
    void intern(const IPhysicsSource& source);
    // Intern every name against source, ignoring any batch it caches.
    void internVocabulary(const IPhysicsSource& source);
};

// Test-only instrumentation: process-wide count of batches filled by
// KnownTokens::intern() or internVocabulary(). Cheap relaxed atomic increment
// (negligible next to filling a batch) -- not gated behind a build macro. Regression tests
// snapshot this before/after a hot-path call to assert it re-interns at most
// once per query/update instead of once per candidate (REQ-SIM-SCENEQUERY-001).
uint64_t internCallCount();
// Test-only instrumentation: the subset of those batches that internVocabulary() filled
// name by name, rather than copied from a source's cached batch.
uint64_t vocabularyInternCount();

} // namespace omni::physics::parse
