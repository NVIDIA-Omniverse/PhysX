// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Defines.h>
#include <private/omni/physics/schema/IUsdPhysicsListener.h>

namespace omni
{
namespace physics
{
namespace schema
{

using UsdPrimMap = std::map<const pxr::SdfPath, pxr::UsdPrim>;

class PrimIteratorBase
{
public:
    virtual ~PrimIteratorBase() = default;
    virtual void reset() = 0;
    virtual bool atEnd() const = 0;
    virtual pxr::UsdPrimRange::const_iterator getCurrent() = 0;
    virtual void next() = 0;
    virtual void pruneChildren() = 0;
};

class PrimIteratorRange : public PrimIteratorBase
{
public:
    PrimIteratorRange(pxr::UsdPrimRange& range) : mRange(range)
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
            mIter++;
        }
    }

private:
    pxr::UsdPrimRange mRange;
    pxr::UsdPrimRange::const_iterator mIter;
};

class PrimIteratorMapRange : public PrimIteratorBase
{
public:
    PrimIteratorMapRange(const UsdPrimMap& primMap) : mPrimMap(primMap)
    {
        reset();
    }

    virtual void reset() override
    {
        mAtEnd = true;

        mPrimMapIter = mPrimMap.begin();

        if (mPrimMapIter != mPrimMap.end())
        {
            mRange = pxr::UsdPrimRange(mPrimMapIter->second, pxr::UsdTraverseInstanceProxies());
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

    virtual pxr::UsdPrimRange::const_iterator getCurrent() override
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
                mRange = pxr::UsdPrimRange(mPrimMapIter->second);
                mIter = mRange.begin();
            }
        }
    }

private:
    bool mAtEnd;

    const UsdPrimMap& mPrimMap;
    UsdPrimMap::const_iterator mPrimMapIter;

    pxr::UsdPrimRange mRange;
    pxr::UsdPrimRange::const_iterator mIter;
};

struct IUsdPhysics
{
    // IPhysicsSchema interface to load physics data from USD
    CARB_PLUGIN_INTERFACE("omni::physics::schema::IUsdPhysics", 1, 1)

    /// Load physics from a given range
    ///
    /// \param[in] stage      Stage to parse
    /// \param[in] range      USDRange to parse
    /// \return True if load was successful
    bool(CARB_ABI* loadFromRange)(const pxr::UsdStageWeakPtr stage,
                                  pxr::UsdGeomXformCache& xfCache,
                                  PrimIteratorBase& range);

    /// Register physics listener that will receive parsed physics objects
    ///
    /// \param[in] listener      Physics listener
    void(CARB_ABI* registerPhysicsListener)(IUsdPhysicsListener* listener);

    /// Unregister physics listener that will receive parsed physics objects
    ///
    /// \param[in] listener      Physics listener
    void(CARB_ABI* unregisterPhysicsListener)(IUsdPhysicsListener* listener);

    /// Add custom shape token
    ///
    /// \param[in] token      Custom shape token
    void(CARB_ABI* addCustomShapeToken)(const pxr::TfToken& shapeToken);

    /// Add custom joint token
    ///
    /// \param[in] token      Custom joint token
    void(CARB_ABI* addCustomJointToken)(const pxr::TfToken& jointToken);

    /// Remove custom joint token
    ///
    /// \param[in] token      Custom joint token
    void(CARB_ABI* removeCustomJointToken)(const pxr::TfToken& jointToken);

    /// Remove custom shape token
    ///
    /// \param[in] shapeToken      Custom shape token
    void(CARB_ABI* removeCustomShapeToken)(const pxr::TfToken& shapeToken);

    /// Add custom physics instancer token
    ///
    /// \param[in] token      Custom physics instancer token
    void(CARB_ABI* addCustomPhysicsInstancerToken)(const pxr::TfToken& instancerToken);

    /// Remove custom physics instancer token
    ///
    /// \param[in] token      Custom physics instancer token
    void(CARB_ABI* removeCustomPhysicsInstancerToken)(const pxr::TfToken& insatancerToken);
};

} // namespace schema
} // namespace physics
} // namespace omni
