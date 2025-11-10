// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <stdint.h>

namespace vcd
{

class AABBTree
{
public:
    static AABBTree* create(const double* vertices, uint32_t numVerts, const uint32_t* indices, uint32_t numFaces);

    virtual bool raycast(const double* start,
                         const double* dir,
                         double& outT,
                         double& u,
                         double& v,
                         double& w,
                         double& faceSign,
                         uint32_t& faceIndex) const = 0;


    virtual bool getClosestPointWithinDistance(const double* point, double maxDistance, double* closestPoint) = 0;

    virtual void release(void) = 0;

protected:
    virtual ~AABBTree(void)
    {
    }
};

} // namespace vcd
