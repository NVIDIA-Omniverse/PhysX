// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-32 AC-33 AC-34 AC-40 AC-49
 */

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/tensors/ISimulationView.h>

#include <regex>
#include <string>
#include <string_view>
#include <vector>

namespace omni
{
namespace physx
{
namespace tensors
{

// Sentinel token used internally to mark a '**' recursive-descent segment.
// Chosen so it cannot collide with any legal USD prim-name glob.
constexpr const char* kRecursiveDescentToken = "**";

std::vector<std::string> splitPatternRespectingGroups(const std::string& pattern);

// pxr-free equivalent of PXR_NS::TfStringTrim(s, "/"): trims leading and trailing '/'
// characters. Shared by this file's source-routed matcher and
// BaseSimulationView.cpp's internal-DB matcher (findMatchingKeysInternalDb), so both
// tokenize identically before splitPatternRespectingGroups; needs no pxr type at all.
std::string trimSlashes(const std::string& s);

// pxr-free approximation of PXR_NS::SdfPath::IsValidPathString for the literal-path
// fast-path gate used by findMatchingObjectKeys (and mirrored by
// BaseSimulationView.cpp's getObjectType / findMatchingKeysInternalDb): true when `s`
// looks path-shaped rather than glob-laden. See PathPatternMatcher.cpp for the full
// rationale. Used unconditionally (both build variants) -- the difference from real
// SdfPath::IsValidPathString only costs one extra findByPath()/glob round trip, never
// a wrong result.
bool looksLikePathString(std::string_view s);

// Compile-once, match-many glob matcher (ADR-0019 increment 8): reproduces
// TfPatternMatcher's historical matching semantics (its own glob pre-transform --
// '.' -> '\.', '*' -> '.*', '?' -> '.', in that exact order -- followed by a POSIX
// EXTENDED regular expression compile, explicitly not std::regex's ECMAScript
// default) without depending on pxr/tf, so both the source-routed matcher below and
// BaseSimulationView.cpp's internal-DB matcher can share one matching engine.
// Construct with one raw pattern token; the constructor groups and anchors it
// ("^(token)$", so '|' alternation stays anchored) before compiling. A token that
// fails to compile as a POSIX ERE degrades to "never matches" rather than throwing,
// mirroring TfPatternMatcher::Match's failure mode on an invalid pattern.
//
// A token longer than omni::physics::tensors::kMaxPathPatternComponentLength is not
// compiled at all and never matches either: libstdc++'s std::regex compiler recurses
// once per pattern term (_Compiler::_M_alternative) and once more per nested group, so
// compiling an N-character token costs O(N) stack -- at the roughly 144 bytes per term
// measured on a GCC 13 release build, a literal token of about 58K characters exhausts
// the 8 MB default thread stack and kills the process before any regex_error can be
// caught. At 4096 the costliest shape, a token that is nothing but nested parentheses,
// compiles within 2 MB of stack (literals within 1 MB), well under the 8 MB default
// the tensor entry points run on, while the bound stays far beyond any real prim name
// or alternation list.
class GlobRegex
{
public:
    explicit GlobRegex(const std::string& token);
    bool match(const std::string& name) const;

private:
    std::regex mRegex;
    bool mValid = false;
};

// Glob-pattern match against `source`'s object hierarchy (ADR-0019 increment 8): the
// IPhysicsSource-routed replacement for the old UsdStage + TfPatternMatcher walk. Works
// uniformly for a USD-backed or ovstage-backed source, with or without a resident USD
// stage -- the traversal goes through IPhysicsSource::getRootKey/forEachChild/
// forEachDescendant/findByPath rather than any backend-specific API.
void findMatchingObjectKeys(const omni::physics::parse::IPhysicsSource& source,
                            const std::string& pattern,
                            bool recursiveLeafPatternMatch,
                            std::vector<omni::physics::parse::ObjectKey>& keysRet);

// Batched form of findMatchingObjectKeys: matches every pattern in `patterns`
// against `source` in one call, `keysRet[i]` holding `patterns[i]`'s matches
// (same order/size as `patterns`). Semantically identical to calling
// findMatchingObjectKeys once per pattern -- the only difference is that the
// literal-path fast path's existence checks are batched into one
// IPhysicsSource::existsBatch call across the WHOLE list instead of one
// exists() call per pattern, which matters when `patterns` is large (e.g. a
// per-index filter-path list).
void findMatchingObjectKeysBatch(const omni::physics::parse::IPhysicsSource& source,
                                 const std::vector<std::string>& patterns,
                                 bool recursiveLeafPatternMatch,
                                 std::vector<std::vector<omni::physics::parse::ObjectKey>>& keysRet);

} // namespace tensors
} // namespace physx
} // namespace omni
