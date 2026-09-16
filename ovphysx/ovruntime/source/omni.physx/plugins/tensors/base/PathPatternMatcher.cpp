// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-32 AC-33 AC-34 AC-40 AC-49
 */

#include "tensors/base/PathPatternMatcher.h"

#include <carb/logging/Log.h>

#include <cctype>
#include <set>
#include <utility>

namespace omni
{
namespace physx
{
namespace tensors
{

using omni::physics::parse::IPhysicsSource;
using omni::physics::parse::ObjectKey;

namespace
{

// Applies TfPatternMatcher's own glob pre-transform: three independent whole-string
// passes, in this exact order and no other -- '.' -> '\.', then '*' -> '.*', then
// '?' -> '.'. Each pass sees only the input state left by the previous one, so a '.'
// introduced by the '*' or '?' passes is never re-escaped by the (already-completed)
// first pass -- matching TfPatternMatcher::SetIsGlobPattern's documented behavior.
std::string applyGlobTransform(const std::string& pattern)
{
    auto replaceAll = [](const std::string& s, char from, const char* to) -> std::string
    {
        std::string out;
        out.reserve(s.size());
        for (char c : s)
        {
            if (c == from)
                out += to;
            else
                out += c;
        }
        return out;
    };
    std::string result = replaceAll(pattern, '.', "\\.");
    result = replaceAll(result, '*', ".*");
    result = replaceAll(result, '?', ".");
    return result;
}

std::string makeAnchoredTokenPattern(const std::string& token)
{
    // Group the token before anchoring: '|' stays ERE alternation even in glob
    // mode, so "^a|b$" parses as (^a)|(b$) and would match prefixes/suffixes
    // (base_link_extra, prefix_link_0). "^(a|b)$" anchors each alternative.
    return "^(" + token + ")$";
}

// Derives a path segment's name from the source's display string for `key`, since
// IPhysicsSource has no getName -- the trailing component after the last slash.
std::string lastPathComponent(const IPhysicsSource& source, ObjectKey key)
{
    const std::string_view full = source.sourceKeyToString(key);
    const size_t pos = full.rfind('/');
    return std::string(pos == std::string_view::npos ? full : full.substr(pos + 1));
}

// Collects every descendant of `key` (not `key` itself) whose name matches `matcher`,
// at ANY depth beneath it -- leaf-recursive matching. Suppresses a match whose name
// already matched an ancestor on the current path (a spurious self-nested overlap);
// distinct alternatives at different depths are kept. `matchedOnPath` carries the
// matched names on the root-to-node path. Structurally identical to the pre-ADR-0019
// UsdPrim-walking version, just routed through IPhysicsSource::forEachChild.
void collectMatchingDescendants(const IPhysicsSource& source, ObjectKey key, const GlobRegex& matcher,
                                std::set<std::string>& matchedOnPath, std::vector<ObjectKey>& out)
{
    source.forEachChild(key,
        [&](ObjectKey child)
        {
            const std::string name = lastPathComponent(source, child);
            if (matcher.match(name) && matchedOnPath.insert(name).second)
            {
                out.push_back(child);
                collectMatchingDescendants(source, child, matcher, matchedOnPath, out);
                matchedOnPath.erase(name);
            }
            else
            {
                collectMatchingDescendants(source, child, matcher, matchedOnPath, out);
            }
        });
}

} // namespace

GlobRegex::GlobRegex(const std::string& token)
{
    // The compile itself can overflow the stack (see the class comment), which no catch
    // below can intercept; an over-long token is refused up front and never matches.
    // Callers are expected to reject such input earlier (ovphysx does at its public
    // boundary), so getting here is reported at error level.
    if (token.size() > omni::physics::tensors::kMaxPathPatternComponentLength)
    {
        CARB_LOG_ERROR("Path pattern component of %zu characters exceeds the %zu-character limit and cannot match "
                       "any object",
                       token.size(), omni::physics::tensors::kMaxPathPatternComponentLength);
        return;
    }
    try
    {
        mRegex = std::regex(applyGlobTransform(makeAnchoredTokenPattern(token)), std::regex::extended);
        mValid = true;
    }
    catch (const std::regex_error&)
    {
        mValid = false;
    }
}

bool GlobRegex::match(const std::string& name) const
{
    return mValid && std::regex_match(name, mRegex);
}

std::string trimSlashes(const std::string& s)
{
    const size_t begin = s.find_first_not_of('/');
    if (begin == std::string::npos)
    {
        return std::string();
    }
    const size_t end = s.find_last_not_of('/');
    return s.substr(begin, end - begin + 1);
}

bool looksLikePathString(std::string_view s)
{
    // Rejects the glob metacharacters this matcher's own token grammar uses ('*', '?',
    // '(', ')', '|', '[', ']', '{', '}') and whitespace -- the same "is this even
    // path-shaped" gate SdfPath's own parser would balk at a glob-laden string on.
    // Deliberately permissive rather than a full SdfPath grammar re-implementation (see
    // PhysX.cpp's isValidClonePath for a stricter absolute-prim-path variant used
    // elsewhere): a false positive here just costs one findByPath() miss before falling
    // through to the glob path below, and a false negative costs one avoidable glob
    // traversal for an already-literal path -- neither changes the result, only which
    // path gets there.
    if (s.empty())
    {
        return false;
    }
    for (const char c : s)
    {
        if (c == '*' || c == '?' || c == '(' || c == ')' || c == '|' || c == '[' || c == ']' || c == '{' ||
            c == '}' || std::isspace(static_cast<unsigned char>(c)))
        {
            return false;
        }
    }
    return true;
}

// Splits a pattern on '/' but ignores separators that appear inside a
// balanced '(...)' group. This preserves regex alternation groups like
// "(foo|bar)" intact across splits.
std::vector<std::string> splitPatternRespectingGroups(const std::string& pattern)
{
    std::vector<std::string> tokens;
    std::string current;
    int depth = 0;
    for (char c : pattern)
    {
        if (c == '(')
        {
            ++depth;
            current += c;
        }
        else if (c == ')')
        {
            if (depth > 0)
            {
                --depth;
            }
            current += c;
        }
        else if (c == '/' && depth == 0)
        {
            if (!current.empty())
            {
                tokens.push_back(std::move(current));
                current.clear();
            }
        }
        else
        {
            current += c;
        }
    }
    if (!current.empty())
    {
        tokens.push_back(std::move(current));
    }
    return tokens;
}

namespace
{

// The glob/wildcard-matching body of findMatchingObjectKeys, factored out so
// findMatchingObjectKeysBatch can skip repeating the literal-path fast-path check
// (which it resolves once for the WHOLE pattern list via a single existsBatch call)
// while sharing the traversal/matching logic byte-for-byte with the single-pattern
// entry point below.
void matchPatternKeys(const IPhysicsSource& source,
                      const std::string& pattern_,
                      bool recursiveLeafPatternMatch,
                      std::vector<ObjectKey>& keysRet)
{
    const std::string pattern = trimSlashes(pattern_);
    const std::vector<std::string> tokens = splitPatternRespectingGroups(pattern);
    if (tokens.empty())
    {
        return;
    }

    std::vector<ObjectKey> roots;
    std::vector<ObjectKey> matches;
    roots.push_back(source.getRootKey());

    const int numTokens = int(tokens.size());

    // An explicit '**' already expands the roots to every descendant, so an
    // additional recursive-leaf descent would re-scan overlapping roots and emit
    // the same path once per ancestor (duplicates + O(N^2)). When the pattern
    // carries a '**', the leaf falls back to strict direct-child matching.
    bool patternHasRecursiveDescent = false;
    for (const std::string& t : tokens)
    {
        if (t == kRecursiveDescentToken)
        {
            patternHasRecursiveDescent = true;
            break;
        }
    }

    for (int i = 0; i < numTokens; i++)
    {
        matches.clear();

        const bool isLeaf = (i == numTokens - 1);
        const bool isRecursiveDescent = (tokens[i] == kRecursiveDescentToken);
        // A bare `*` leaf stays strict (direct children only); only a *named*
        // or glob leaf is searched at any depth. Callers who want a bare
        // recursive descent must use `**`.
        const bool isBareWildcard = (tokens[i] == "*");

        if (isRecursiveDescent)
        {
            for (ObjectKey root : roots)
            {
                source.forEachDescendant(root, [&](ObjectKey key) { matches.push_back(key); });
            }
        }
        else if (isLeaf && recursiveLeafPatternMatch && !isBareWildcard && !patternHasRecursiveDescent)
        {
            // Leaf-recursive: the final named/glob token is searched at any depth
            // beneath the strictly-matched ancestor chain (with same-name
            // suppression). Pre-leaf tokens above were matched strictly, so
            // intermediate structure is respected.
            const GlobRegex matcher(tokens[i]);
            for (ObjectKey root : roots)
            {
                std::set<std::string> matchedOnPath;
                collectMatchingDescendants(source, root, matcher, matchedOnPath, matches);
            }
        }
        else
        {
            const GlobRegex matcher(tokens[i]);
            for (ObjectKey root : roots)
            {
                source.forEachChild(root,
                    [&](ObjectKey child)
                    {
                        if (matcher.match(lastPathComponent(source, child)))
                        {
                            matches.push_back(child);
                        }
                    });
            }
        }

        if (i < numTokens - 1)
        {
            std::swap(roots, matches);
        }
    }

    for (ObjectKey key : matches)
    {
        keysRet.push_back(key);
    }
}

} // namespace

void findMatchingObjectKeys(const IPhysicsSource& source,
                            const std::string& pattern_,
                            bool recursiveLeafPatternMatch,
                            std::vector<ObjectKey>& keysRet)
{
    // First try if the pattern_ is an actual object path itself; if so, do not try to
    // pattern match for performance reasons. Guard with IsValidPathString first (as
    // before) so a glob-laden pattern never reaches findByPath's path parser -- USD's
    // parser emits a diagnostic for syntactically invalid path characters like '*'.
    // Gate the resolved key on exists(): unlike UsdSource, OvstageSource::findByPath
    // interns any syntactically valid path string without checking whether it names a
    // live object (a pre-existing, documented backend gap -- ADR-0019 increment 1's
    // resolveObjectKey finding), so exists() is required here to preserve the
    // "literal path must actually resolve" contract on both backends.
    if (looksLikePathString(pattern_))
    {
        const ObjectKey direct = source.findByPath(pattern_);
        if (direct.valid() && source.exists(direct))
        {
            keysRet.push_back(direct);
            return;
        }
    }

    matchPatternKeys(source, pattern_, recursiveLeafPatternMatch, keysRet);
}

void findMatchingObjectKeysBatch(const IPhysicsSource& source,
                                 const std::vector<std::string>& patterns,
                                 bool recursiveLeafPatternMatch,
                                 std::vector<std::vector<ObjectKey>>& keysRet)
{
    keysRet.clear();
    keysRet.resize(patterns.size());

    // Same literal-path fast path as findMatchingObjectKeys (see its comment), but
    // resolved for the WHOLE pattern list in one existsBatch call instead of one
    // exists() call per pattern -- the fix for the O(N) live round trips a caller
    // holding a large literal-path candidate list (e.g. thousands of per-index
    // sensor/filter paths) would otherwise issue one at a time.
    std::vector<bool> isLiteralCandidate(patterns.size(), false);
    std::vector<size_t> literalPatternIndex;
    std::vector<ObjectKey> literalKeys;
    literalPatternIndex.reserve(patterns.size());
    literalKeys.reserve(patterns.size());
    for (size_t i = 0; i < patterns.size(); ++i)
    {
        if (looksLikePathString(patterns[i]))
        {
            const ObjectKey direct = source.findByPath(patterns[i]);
            if (direct.valid())
            {
                isLiteralCandidate[i] = true;
                literalPatternIndex.push_back(i);
                literalKeys.push_back(direct);
            }
        }
    }

    std::vector<bool> literalExists;
    if (!literalKeys.empty())
    {
        source.existsBatch(literalKeys, literalExists);
    }

    for (size_t li = 0; li < literalPatternIndex.size(); ++li)
    {
        const size_t patternIdx = literalPatternIndex[li];
        if (literalExists[li])
        {
            keysRet[patternIdx].push_back(literalKeys[li]);
        }
        else
        {
            // Same fallback findMatchingObjectKeys takes when a literal-shaped
            // path does not resolve to a live object: fall through to glob
            // matching on that same pattern string.
            matchPatternKeys(source, patterns[patternIdx], recursiveLeafPatternMatch, keysRet[patternIdx]);
        }
    }

    // Every pattern that was never a syntactically valid literal path skips the
    // fast path entirely, exactly like findMatchingObjectKeys does.
    for (size_t i = 0; i < patterns.size(); ++i)
    {
        if (!isLiteralCandidate[i])
        {
            matchPatternKeys(source, patterns[i], recursiveLeafPatternMatch, keysRet[i]);
        }
    }
}

} // namespace tensors
} // namespace physx
} // namespace omni
