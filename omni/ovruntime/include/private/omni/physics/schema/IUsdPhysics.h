// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Defines.h>
#include <carb/Assert.h>
#include <private/omni/physics/schema/IUsdPhysicsListener.h>

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

    virtual void reset() override
    {
        mAtEnd = true;

        mPrimMapIter = mPrimMap.begin();

        if (mPrimMapIter != mPrimMap.end())
        {
            mRange = PXR_NS::UsdPrimRange(mPrimMapIter->second, PXR_NS::UsdTraverseInstanceProxies());
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
                mRange = PXR_NS::UsdPrimRange(mPrimMapIter->second);
                mIter = mRange.begin();
            }
        }
    }

private:
    bool mAtEnd;

    const UsdPrimMap& mPrimMap;
    UsdPrimMap::const_iterator mPrimMapIter;

    PXR_NS::UsdPrimRange mRange;
    PXR_NS::UsdPrimRange::const_iterator mIter;
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
    bool(CARB_ABI* loadFromRange)(const PXR_NS::UsdStageWeakPtr stage,
                                  PXR_NS::UsdGeomXformCache& xfCache,
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
    void(CARB_ABI* addCustomShapeToken)(const PXR_NS::TfToken& shapeToken);

    /// Add custom joint token
    ///
    /// \param[in] token      Custom joint token
    void(CARB_ABI* addCustomJointToken)(const PXR_NS::TfToken& jointToken);

    /// Remove custom joint token
    ///
    /// \param[in] token      Custom joint token
    void(CARB_ABI* removeCustomJointToken)(const PXR_NS::TfToken& jointToken);

    /// Remove custom shape token
    ///
    /// \param[in] shapeToken      Custom shape token
    void(CARB_ABI* removeCustomShapeToken)(const PXR_NS::TfToken& shapeToken);

    /// Add custom physics instancer token
    ///
    /// \param[in] token      Custom physics instancer token
    void(CARB_ABI* addCustomPhysicsInstancerToken)(const PXR_NS::TfToken& instancerToken);

    /// Remove custom physics instancer token
    ///
    /// \param[in] token      Custom physics instancer token
    void(CARB_ABI* removeCustomPhysicsInstancerToken)(const PXR_NS::TfToken& insatancerToken);
};

} // namespace schema
} // namespace physics
} // namespace omni
