// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physics/schema/IUsdPhysics.h>
#include <private/omni/physx/IPhysxSupportUi.h>
#include "PrimDef.h"

namespace omni
{
namespace physx
{

class PrimUpdateMap final
{
public:
    typedef std::unordered_map<const pxr::SdfPath, PrimDef, pxr::SdfPath::Hash> TPrimAddMap;

public:
    PrimUpdateMap()
    {
    }

    void addPrim(const PrimDef& primDef);

    void removePrim(const pxr::SdfPath&);

    bool exists(const pxr::UsdPrim& prim) const;

    void clearMap()
    {
        mPrimAddMap.clear();
    }

    void checkMap(const pxr::UsdStageWeakPtr stage);

    const TPrimAddMap& getMap() const
    {
        return mPrimAddMap;
    }

private:
    TPrimAddMap mPrimAddMap;
};

} // namespace physx
} // namespace omni
