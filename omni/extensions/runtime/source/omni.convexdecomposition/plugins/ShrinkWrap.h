// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

// This is a code snippet which 'shrinkwraps' a convex hull
// to a source mesh.
//
// It is a somewhat complicated algorithm. It works as follows:
//
// * Step #1 : Compute the mean unit normal vector for each vertex in the convex hull
// * Step #2 : For each vertex in the conex hull we project is slightly outwards along the mean normal vector
// * Step #3 : We then raycast from this slightly extruded point back into the opposite direction of the mean normal
// vector
//             resulting in a raycast from slightly beyond the vertex in the hull into the source mesh we are trying
//             to 'shrink wrap' against
// * Step #4 : If the raycast fails we leave the original vertex alone
// * Step #5 : If the raycast hits a backface we leave the original vertex alone
// * Step #6 : If the raycast hits too far away (no more than a certain threshold distance) we live it alone
// * Step #7 : If the point we hit on the source mesh is not still within the convex hull, we reject it.
// * Step #8 : If all of the previous conditions are met, then we take the raycast hit location as the 'new position'
// * Step #9 : Once all points have been projected, if possible, we need to recompute the convex hull again based on
// these shrinkwrapped points
// * Step #10 : In theory that should work.. let's see...
#include <stdint.h>

namespace vcd
{

class SimpleMesh;
class RaycastMesh;


class ShrinkWrap
{
public:
    static ShrinkWrap* create(void);

    virtual void shrinkWrap(SimpleMesh& sourceConvexHull,
                            RaycastMesh& raycastMesh,
                            uint32_t maxHullVertexCount,
                            double distanceThreshold,
                            bool doShrinkWrap) = 0;

    virtual void release(void) = 0;
};

} // namespace vcd
