// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

namespace omni
{
namespace physx
{
namespace ui
{

class DeformableMeshVisualizer
{
public:
    virtual ~DeformableMeshVisualizer()
    {
    }

    virtual void setPoints(const pxr::VtArray<pxr::GfVec3f>& originalPoints) = 0;
    virtual void setIndices(const pxr::VtArray<int>& indices) = 0;
    virtual void setGapValue(const float gap) = 0;
    virtual void setColors(const pxr::VtArray<pxr::GfVec3f>& colors) = 0;
    virtual void updateTopology() = 0;
    virtual void updatePoints() = 0;
};

} // namespace ui
} // namespace physx
} // namespace omni
