// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "UsdPCH.h"

#include <private/omni/physx/PhysxUsd.h>

namespace omni
{
namespace physx
{
namespace ui
{
typedef std::pair<usdparser::ObjectCategory, usdparser::PhysxObjectDesc*> ObjectRecord;
typedef std::multimap<pxr::SdfPath, ObjectRecord> ObjectDescMap;

class DescCache
{
public:
    DescCache() = default;

    ~DescCache()
    {
        release();
    }

    bool empty() const
    {
        return mObjectDescMap.empty();
    }

    void release();

    void addDesc(const pxr::SdfPath& path, usdparser::ObjectCategory cat, usdparser::PhysxObjectDesc* desc);

    void releaseDesc(const pxr::SdfPath& path, usdparser::PhysxObjectDesc* desc);

    void releasePath(const pxr::SdfPath& path);

    usdparser::PhysxObjectDesc* getDesc(const pxr::SdfPath& path, usdparser::ObjectCategory cat) const;

private:
    ObjectDescMap mObjectDescMap;
};

} // namespace ui
} // namespace physx
} // namespace omni
