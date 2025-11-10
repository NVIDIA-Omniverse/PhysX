// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physics/schema/IUsdPhysics.h>


namespace omni
{
namespace physx
{
namespace usdparser
{

class ReplicatorPrimIteratorRange : public omni::physics::schema::PrimIteratorBase
{
public:
    ReplicatorPrimIteratorRange(pxr::UsdPrimRange& range, const PathSet& pathSet) : mRange(range), mPathSet(pathSet)
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

    virtual pxr::UsdPrimRange::const_iterator getCurrent() override
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
                    const pxr::UsdPrim& prim = *mIter;
                    if (prim)
                    {
                        PathSet::const_iterator fit = mPathSet.find(prim.GetPrimPath());
                        if (fit != mPathSet.end())
                        {
                            mIter.PruneChildren();
                        }
                        else
                        {
                            validPrim = true;
                        }
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
    pxr::UsdPrimRange mRange;
    pxr::UsdPrimRange::const_iterator mIter;
    const PathSet& mPathSet;
};

} // namespace usdparser
} // namespace physx
} // namespace omni
