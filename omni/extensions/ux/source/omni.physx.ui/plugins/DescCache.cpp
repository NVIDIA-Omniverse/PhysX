// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "DescCache.h"
#include <private/omni/physx/IPhysxUsdLoad.h>

using namespace pxr;
using namespace omni::physx::usdparser;

extern omni::physx::usdparser::IPhysxUsdLoad* gUsdLoad;

namespace omni
{
namespace physx
{
namespace ui
{

void DescCache::release()
{
    ObjectDescMap::iterator it = mObjectDescMap.begin();
    while (it != mObjectDescMap.end())
    {
        gUsdLoad->releaseDesc(it->second.second);
        it++;
    }

    mObjectDescMap.clear();
}

void DescCache::addDesc(const pxr::SdfPath& path, ObjectCategory cat, PhysxObjectDesc* desc)
{
    mObjectDescMap.insert(std::make_pair(path, std::make_pair(cat, desc)));
}

void DescCache::releaseDesc(const pxr::SdfPath& path, PhysxObjectDesc* desc)
{
    ObjectDescMap::const_iterator it = mObjectDescMap.find(path);
    while (it != mObjectDescMap.end() && it->first == path)
    {
        if (it->second.second == desc)
        {
            mObjectDescMap.erase(it);
            return;
        }
        it++;
    }

    gUsdLoad->releaseDesc(desc);
}

PhysxObjectDesc* DescCache::getDesc(const pxr::SdfPath& path, ObjectCategory cat) const
{
    ObjectDescMap::const_iterator it = mObjectDescMap.find(path);
    while (it != mObjectDescMap.end() && it->first == path)
    {
        if (it->second.first == cat)
            return it->second.second;
        it++;
    }

    return nullptr;
}

void DescCache::releasePath(const pxr::SdfPath& path)
{
    ObjectDescMap::iterator it = mObjectDescMap.find(path);
    while (it != mObjectDescMap.end() && it->first == path)
    {
        gUsdLoad->releaseDesc(it->second.second);
        it = mObjectDescMap.erase(it);
    }
}

} // namespace ui
} // namespace physx
} // namespace omni
