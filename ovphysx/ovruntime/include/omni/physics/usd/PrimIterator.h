// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * Polymorphic prim iterators that abstract over whole-stage USD
 * traversal, single-prim-range traversal, and prim-map-driven
 * traversal.
 *
 * Namespace `omni::physics::schema` is retained so the dozens of
 * in-tree call sites continue to compile unchanged; the type is the
 * public input to
 * `omni::physics::usd::scanStage(stage, primIterator)`.
 */

#pragma once

#include <carb/Assert.h>

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>

#include <map>
#include <set>
#include <unordered_set>

namespace omni
{
namespace physics
{
namespace schema
{

using UsdPrimMap = std::map<const PXR_NS::SdfPath, PXR_NS::UsdPrim>;

class PrimIteratorBase
{
public:
    virtual ~PrimIteratorBase() = default;
    virtual void reset() = 0;
    virtual bool atEnd() const = 0;
    virtual PXR_NS::UsdPrimRange::const_iterator getCurrent() = 0;
    virtual void next() = 0;
    virtual void pruneChildren() = 0;
};

class PrimIteratorRange : public PrimIteratorBase
{
public:
    PrimIteratorRange(PXR_NS::UsdPrimRange& range) : mRange(range)
    {
        reset();
    }

    virtual void reset() override
    {
        mIter = mRange.begin();
    }

    virtual void pruneChildren() override
    {
        CARB_ASSERT(!atEnd());
        mIter.PruneChildren();
    }

    virtual bool atEnd() const override
    {
        return mIter == mRange.end();
    }

    virtual PXR_NS::UsdPrimRange::const_iterator getCurrent() override
    {
        return mIter;
    }

    virtual void next() override
    {
        if (mIter != mRange.end())
        {
            mIter++;
        }
    }

private:
    PXR_NS::UsdPrimRange mRange;
    PXR_NS::UsdPrimRange::const_iterator mIter;
};

class PrimIteratorMapRange : public PrimIteratorBase
{
public:
    PrimIteratorMapRange(const UsdPrimMap& primMap) : mPrimMap(primMap)
    {
        reset();
    }

    // Path-driven overload: consumers that hold only the set of subtree-root
    // paths (USD-free state) pass the paths + stage; the prims are resolved here,
    // in the USD layer, into an owned map that the iteration then walks.
    PrimIteratorMapRange(const std::set<PXR_NS::SdfPath>& paths, const PXR_NS::UsdStageWeakPtr& stage)
        : mPrimMap(mOwnedPrimMap)
    {
        if (stage)
        {
            for (const PXR_NS::SdfPath& path : paths)
            {
                PXR_NS::UsdPrim prim = stage->GetPrimAtPath(path);
                if (prim)
                    mOwnedPrimMap.emplace(path, prim);
            }
        }
        reset();
    }

    virtual void reset() override
    {
        mAtEnd = true;

        mPrimMapIter = mPrimMap.begin();

        if (mPrimMapIter != mPrimMap.end())
        {
            mRange = makeRange(mPrimMapIter->second);
            if (mRange.begin() != mRange.end())
            {
                mIter = mRange.begin();
                mAtEnd = false;
            }
        }
    }

    virtual bool atEnd() const override
    {
        return mAtEnd;
    }

    virtual void pruneChildren() override
    {
        CARB_ASSERT(!atEnd());
        mIter.PruneChildren();
    }

    virtual PXR_NS::UsdPrimRange::const_iterator getCurrent() override
    {
        return mIter;
    }

    virtual void next() override
    {
        CARB_ASSERT(mIter != mRange.end());
        mIter++;

        if (mIter == mRange.end())
        {
            mPrimMapIter++;

            if (mPrimMapIter == mPrimMap.end())
            {
                mAtEnd = true;
            }
            else
            {
                mRange = makeRange(mPrimMapIter->second);
                mIter = mRange.begin();
            }
        }
    }

private:
    // Single source of truth for the per-root traversal options so reset() and
    // next() iterate every mapped root identically (instance proxies included).
    static PXR_NS::UsdPrimRange makeRange(const PXR_NS::UsdPrim& root)
    {
        return PXR_NS::UsdPrimRange(root, PXR_NS::UsdTraverseInstanceProxies());
    }

    bool mAtEnd;

    // Backing storage for the path-driven ctor; empty (unused) for the
    // external-map ctor. Declared before mPrimMap so it is constructed first.
    UsdPrimMap mOwnedPrimMap;
    const UsdPrimMap& mPrimMap;
    UsdPrimMap::const_iterator mPrimMapIter;

    PXR_NS::UsdPrimRange mRange;
    PXR_NS::UsdPrimRange::const_iterator mIter;
};

// Exclude nested subtrees while preserving the legacy root-prim exception.
// @implements REQ-PARSE-SCAN-001
// @covers AC-5 AC-15
class PrimIteratorExcludeRange : public PrimIteratorBase
{
public:
    PrimIteratorExcludeRange(PrimIteratorBase& input,
        const std::unordered_set<PXR_NS::SdfPath, PXR_NS::SdfPath::Hash>& excludePaths)
        : mInput(input), mExclude(excludePaths)
    {
        reset();
    }

    void reset() override { mInput.reset(); skipExcluded(); }
    bool atEnd() const override { return mInput.atEnd(); }
    PXR_NS::UsdPrimRange::const_iterator getCurrent() override { return mInput.getCurrent(); }
    void pruneChildren() override { mInput.pruneChildren(); }
    void next() override
    {
        if (!mInput.atEnd())
            mInput.next();
        skipExcluded();
    }

private:
    void skipExcluded()
    {
        while (!mInput.atEnd())
        {
            const PXR_NS::SdfPath path = mInput.getCurrent()->GetPath();
            bool excluded = false;
            for (const PXR_NS::SdfPath& root : mExclude)
            {
                if (!root.IsRootPrimPath() && path.HasPrefix(root))
                {
                    excluded = true;
                    break;
                }
            }
            if (!excluded)
                return;
            mInput.pruneChildren();
            mInput.next();
        }
    }

    PrimIteratorBase& mInput;
    std::unordered_set<PXR_NS::SdfPath, PXR_NS::SdfPath::Hash> mExclude;
};

} // namespace schema
} // namespace physics
} // namespace omni
