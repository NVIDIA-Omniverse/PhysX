// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "DebugVisualizationHelpers.h"

namespace omni
{
namespace physx
{
namespace ui
{

const DebugLine* getUnitAABBDebugDraw(uint32_t color, uint32_t& numLines)
{
    const Float3 minimum{ 0.0f, 0.0f, 0.0f };
    const Float3 maximum{ 1.0f, 1.0f, 1.0f };

    const Float3 vertices[] = {
        Float3{ minimum.x, minimum.y, minimum.z },
        Float3{ maximum.x, minimum.y, minimum.z },
        Float3{ maximum.x, maximum.y, minimum.z },
        Float3{ minimum.x, maximum.y, minimum.z },
        Float3{ minimum.x, minimum.y, maximum.z },
        Float3{ maximum.x, minimum.y, maximum.z },
        Float3{ maximum.x, maximum.y, maximum.z },
        Float3{ minimum.x, maximum.y, maximum.z },
    };

    numLines = 12;
    DebugLine* lines = new DebugLine[numLines];
    int idx = 0;

    auto addSimplexLine = [&](const Float3 p0, const Float3 p1)
    {
        DebugLine& l = lines[idx++];
        l.mPos0 = p0;
        l.mColor0 = color;
        l.mPos1 = p1;
        l.mColor1 = color;
    };

    #define L(a, b) addSimplexLine(vertices[a], vertices[b]);

    L(0, 1)
    L(1, 2)
    L(2, 3)
    L(3, 0)

    L(4, 5)
    L(5, 6)
    L(6, 7)
    L(7, 4)

    L(0, 4)
    L(1, 5)
    L(2, 6)
    L(3, 7)

    #undef L

    return lines;
}

const DebugLine* getUnitCornerAABBDebugDraw(uint32_t color, uint32_t& numLines)
{
    const Float3 minimum{ 0.0f, 0.0f, 0.0f };
    const Float3 maximum{ 1.0f, 1.0f, 1.0f };

    const float edgeLength{ 0.15f }; // 0.0f..1.0f
    const Float3 minimum2{ edgeLength, edgeLength, edgeLength };
    const Float3 maximum2{ 1.0f - edgeLength, 1.0f - edgeLength, 1.0f - edgeLength };

    const Float3 vertices[] = {
        Float3{ minimum.x, minimum.y, minimum.z },
            Float3{ minimum2.x, minimum.y, minimum.z },
            Float3{ minimum.x, minimum2.y, minimum.z },
            Float3{ minimum.x, minimum.y, minimum2.z },
        Float3{ maximum.x, minimum.y, minimum.z },
            Float3{ maximum2.x, minimum.y, minimum.z },
            Float3{ maximum.x, minimum2.y, minimum.z },
            Float3{ maximum.x, minimum.y, minimum2.z },
        Float3{ maximum.x, maximum.y, minimum.z },
            Float3{ maximum2.x, maximum.y, minimum.z },
            Float3{ maximum.x, maximum2.y, minimum.z },
            Float3{ maximum.x, maximum.y, minimum2.z },
        Float3{ minimum.x, maximum.y, minimum.z },
            Float3{ minimum2.x, maximum.y, minimum.z },
            Float3{ minimum.x, maximum2.y, minimum.z },
            Float3{ minimum.x, maximum.y, minimum2.z },
        Float3{ minimum.x, minimum.y, maximum.z },
            Float3{ minimum2.x, minimum.y, maximum.z },
            Float3{ minimum.x, minimum2.y, maximum.z },
            Float3{ minimum.x, minimum.y, maximum2.z },
        Float3{ maximum.x, minimum.y, maximum.z },
            Float3{ maximum2.x, minimum.y, maximum.z },
            Float3{ maximum.x, minimum2.y, maximum.z },
            Float3{ maximum.x, minimum.y, maximum2.z },
        Float3{ maximum.x, maximum.y, maximum.z },
            Float3{ maximum2.x, maximum.y, maximum.z },
            Float3{ maximum.x, maximum2.y, maximum.z },
            Float3{ maximum.x, maximum.y, maximum2.z },
        Float3{ minimum.x, maximum.y, maximum.z },
            Float3{ minimum2.x, maximum.y, maximum.z },
            Float3{ minimum.x, maximum2.y, maximum.z },
            Float3{ minimum.x, maximum.y, maximum2.z },
    };

    numLines = 24;
    DebugLine* lines = new DebugLine[numLines];
    int idx = 0;

    auto addSimplexLine = [&](const Float3 p0, const Float3 p1)
    {
        DebugLine& l = lines[idx++];
        l.mPos0 = p0;
        l.mColor0 = color;
        l.mPos1 = p1;
        l.mColor1 = color;
    };

    #define L(a, b) addSimplexLine(vertices[a], vertices[b]);

    // Connected edges
    L(0, 1)
    L(0, 2)
    L(0, 3)

    L(4, 5)
    L(4, 6)
    L(4, 7)

    L(8, 9)
    L(8, 10)
    L(8, 11)

    L(12, 13)
    L(12, 14)
    L(12, 15)

    L(16, 17)
    L(16, 18)
    L(16, 19)

    L(20, 21)
    L(20, 22)
    L(20, 23)

    L(24, 25)
    L(24, 26)
    L(24, 27)

    L(28, 29)
    L(28, 30)
    L(28, 31)

    #undef L

    return lines;
}

} // namespace ui
} // namespace physx
} // namespace omni
