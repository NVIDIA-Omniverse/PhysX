// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include <UsdPCH.h>
#include <pxr/base/tf/patternMatcher.h>
#include <pxr/base/tf/stringUtils.h>
// clang-format on

#include "tensors/base/PathPatternMatcher.h"

#include <set>
#include <utility>

using namespace PXR_NS;

namespace omni
{
namespace physx
{
namespace tensors
{

namespace
{

void findMatchingChildren(UsdPrim root, const std::string& pattern, std::vector<UsdPrim>& primsRet)
{
    if (!root)
    {
        return;
    }

    TfPatternMatcher matcher(pattern, true, true);
    UsdPrimSiblingRange range = root.GetAllChildren();
    for (auto child : range)
    {
        if (matcher.Match(child.GetName()))
        {
            primsRet.push_back(child);
        }
    }
}

void collectSelfAndDescendants(UsdPrim prim, std::vector<UsdPrim>& out)
{
    if (!prim)
    {
        return;
    }
    std::vector<UsdPrim> stack;
    stack.push_back(prim);
    while (!stack.empty())
    {
        UsdPrim current = stack.back();
        stack.pop_back();
        out.push_back(current);
        // Push children in reverse so pop order matches declaration order.
        auto children = current.GetAllChildren();
        std::vector<UsdPrim> childVec(children.begin(), children.end());
        for (auto it = childVec.rbegin(); it != childVec.rend(); ++it)
        {
            stack.push_back(*it);
        }
    }
}

// Collects every descendant of `prim` (not prim itself) whose name matches
// `matcher`, at ANY depth -- leaf-recursive matching, so a named leaf token
// reaches bodies nested below the pattern level (e.g. Isaac Sim URDF converter
// output: /Robot/link_0 finds link_0 wherever the converter nested it). The
// pre-leaf tokens are matched strictly by the caller, so intermediate structure
// is respected -- a buried pivot (Group/Robot) needs an explicit '**'.
// Suppresses a match whose name already matched an ancestor on the current path:
// the same name nested inside its own match (robot/Disc001/robot/Disc001, or
// base_link under base_link) is a spurious overlap. Distinct alternatives at
// different depths are kept (base_link|link_0). `matchedOnPath` carries the
// matched names on the root-to-node path.
void collectMatchingDescendants(UsdPrim prim, TfPatternMatcher& matcher,
                                std::set<std::string>& matchedOnPath, std::vector<UsdPrim>& out)
{
    if (!prim)
    {
        return;
    }
    for (auto child : prim.GetAllChildren())
    {
        const std::string name = child.GetName().GetString();
        if (matcher.Match(name) && matchedOnPath.insert(name).second)
        {
            out.push_back(child);
            collectMatchingDescendants(child, matcher, matchedOnPath, out);
            matchedOnPath.erase(name);
        }
        else
        {
            collectMatchingDescendants(child, matcher, matchedOnPath, out);
        }
    }
}

} // namespace

std::string makeAnchoredTokenPattern(const std::string& token)
{
    // Group the token before anchoring: '|' stays ERE alternation even in glob
    // mode, so "^a|b$" parses as (^a)|(b$) and would match prefixes/suffixes
    // (base_link_extra, prefix_link_0). "^(a|b)$" anchors each alternative.
    return "^(" + token + ")$";
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

void findMatchingUsdPaths(UsdStageWeakPtr stage,
                          const std::string& pattern_,
                          bool recursiveLeafPatternMatch,
                          std::vector<SdfPath>& pathsRet)
{
    if (!stage)
    {
        return;
    }

    // First try if the pattern_ is an actual prim path itself; if so, do not
    // try to pattern match for performance reasons.
    if (SdfPath::IsValidPathString(pattern_))
    {
        if (stage->GetPrimAtPath(SdfPath(pattern_)))
        {
            pathsRet.push_back(SdfPath(pattern_));
            return;
        }
    }

    std::string pattern = TfStringTrim(pattern_, "/");
    std::vector<std::string> tokens = splitPatternRespectingGroups(pattern);

    std::vector<UsdPrim> roots;
    std::vector<UsdPrim> matches;

    roots.push_back(stage->GetPseudoRoot());

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
            for (auto& prim : roots)
            {
                collectSelfAndDescendants(prim, matches);
            }
        }
        else if (isLeaf && recursiveLeafPatternMatch && !isBareWildcard && !patternHasRecursiveDescent)
        {
            // Leaf-recursive: the final named/glob token is searched at any depth
            // beneath the strictly-matched ancestor chain (with same-name
            // suppression). Pre-leaf tokens above were matched strictly, so
            // intermediate structure is respected.
            TfPatternMatcher matcher(makeAnchoredTokenPattern(tokens[i]), true, true);
            for (auto& prim : roots)
            {
                std::set<std::string> matchedOnPath;
                collectMatchingDescendants(prim, matcher, matchedOnPath, matches);
            }
        }
        else
        {
            const std::string tokPattern = makeAnchoredTokenPattern(tokens[i]);
            for (auto& prim : roots)
            {
                findMatchingChildren(prim, tokPattern, matches);
            }
        }

        if (i < numTokens - 1)
        {
            std::swap(roots, matches);
        }
    }

    for (auto& prim : matches)
    {
        pathsRet.push_back(prim.GetPath());
    }
}

} // namespace tensors
} // namespace physx
} // namespace omni
