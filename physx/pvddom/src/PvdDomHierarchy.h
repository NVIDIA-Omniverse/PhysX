// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include "PvdDom.h"

namespace PvdDomInspect
{
namespace detail
{
inline bool canParent(OmniPvdObject* parent, OmniPvdObject* child, size_t limit,
                      OmniPvdObject* movedChild = nullptr, OmniPvdObject* movedParent = nullptr)
{
    size_t visited = 0;
    for (OmniPvdObject* node = parent; node; node = node == movedChild ? movedParent : node->mAncestor)
        if (node == child || ++visited > limit) return false;
    return true;
}
inline bool canParentArticulation(OmniPvdObject* parent, OmniPvdObject* child, OmniPvdObject* joint, size_t limit)
{
    return canParent(parent, child, limit) && canParent(child, joint, limit, parent ? child : nullptr, parent);
}
}
}
