// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

namespace omni
{
namespace physx
{
namespace ui
{
    struct SortedTriangle
    {
        uint32_t A;
        uint32_t B;
        uint32_t C;
        int32_t TetIndex;
        int32_t TriIndex;
        bool Flipped;

        inline SortedTriangle(uint32_t a = -1, uint32_t b = -1, uint32_t c = -1, int32_t tetIndex = -1, int32_t triIndex = -1)
        {
            A = a;
            B = b;
            C = c;
            TetIndex = tetIndex;
            TriIndex = triIndex;
            Flipped = false;
            if (A > B)
            {
                std::swap(A, B);
                Flipped = !Flipped;
            }
            if (B > C)
            {
                std::swap(B, C);
                Flipped = !Flipped;
            }
            if (A > B)
            {
                std::swap(A, B);
                Flipped = !Flipped;
            }
        }
    };

    class SortedTriangleEqualFunction
    {
    public:
        inline bool operator()(const SortedTriangle& first, const SortedTriangle& second) const
        {
            return first.A == second.A && first.B == second.B && first.C == second.C;
        }
    };

    struct TriangleHash
    {
        inline std::size_t operator()(const SortedTriangle& k) const
        {
            return k.A ^ k.B ^ k.C;
        }
    };

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
