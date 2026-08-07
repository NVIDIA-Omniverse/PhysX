// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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

// Whole-subtree iteration that skips any prim whose path is in `excludePaths`,
// together with its entire subtree (matches the legacy
// `ReplicatorPrimIteratorRange` selective-load semantics). Constructs its own
// range from `stage`/`root`/`predicate` so the root/exclude `scanStage` overload
// can keep all USD traversal inside the backend (consumers pass only paths).
class PrimIteratorExcludeRange : public PrimIteratorBase
{
public:
    PrimIteratorExcludeRange(PXR_NS::UsdPrimRange range,
                             const std::unordered_set<PXR_NS::SdfPath, PXR_NS::SdfPath::Hash>& excludePaths)
        : mRange(std::move(range)), mExclude(excludePaths)
    {
        reset();
    }

    virtual void reset() override
    {
        // Match ReplicatorPrimIteratorRange: the root is yielded as-is; excluded
        // subtrees are skipped on advance (next()).
        mIter = mRange.begin();
    }

    virtual bool atEnd() const override
    {
        return mIter == mRange.end();
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
        if (mIter != mRange.end())
        {
            bool validPrim = false;
            while (!validPrim)
            {
                mIter++;
                if (mIter != mRange.end())
                {
                    const PXR_NS::UsdPrim& prim = *mIter;
                    // A root-prim exclude path is never pruned: the legacy
                    // ReplicatorPrimIteratorRange walked stage->Traverse(), whose
                    // first element is the stage's root prim, and reset() yielded
                    // that first element unchecked.  So excluding a root prim
                    // (e.g. clone()'s derived env root "/World" for a body parented
                    // directly under it) was effectively a no-op and the subtree
                    // was still parsed.  Preserve that: only prune non-root-prim
                    // exclude paths.
                    if (prim && !prim.GetPrimPath().IsRootPrimPath() &&
                        mExclude.find(prim.GetPrimPath()) != mExclude.end())
                    {
                        mIter.PruneChildren();
                    }
                    else
                    {
                        validPrim = true;
                    }
                }
                else
                {
                    validPrim = true;
                }
            }
        }
    }

private:
    PXR_NS::UsdPrimRange mRange;
    // Stored by value: the exclude set is tiny (replicator selective load) and a
    // public-header iterator must not depend on the caller's set outliving it.
    std::unordered_set<PXR_NS::SdfPath, PXR_NS::SdfPath::Hash> mExclude;
    PXR_NS::UsdPrimRange::const_iterator mIter;
};

} // namespace schema
} // namespace physics
} // namespace omni
