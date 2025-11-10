// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "PhysXPointFinder.h"

#include <PhysXTools.h>
#include <common/utilities/MemoryMacros.h>

using namespace physx;

namespace omni
{
    namespace pointfinder
    {
        struct PointFinder : Allocateable
        {
            struct hash
            {
                size_t operator()(const carb::Float3& vec) const
                {
                    return carb::fnv1aHash<12>(reinterpret_cast<const char(&)[12]>(vec));
                }
            };

            struct equal_to
            {
                constexpr bool operator()(const carb::Float3& a, const carb::Float3& b) const
                {
                    return a.x == b.x && a.y == b.y && a.z == b.z;
                }
            };

            using Vec3ToIndexMap = std::unordered_map<carb::Float3, uint32_t, hash, equal_to>;

            PointFinder(const carb::Float3* points, const uint32_t pointsSize)
            {
                for (uint32_t p = 0; p < pointsSize; ++p)
                {
                    mMap[points[p]] = p;
                }
            }

            void pointToIndices(int32_t* pointIndices, const carb::Float3* points, const uint32_t pointsSize)
            {
                for (uint32_t p = 0; p < pointsSize; ++p)
                {
                    Vec3ToIndexMap::const_iterator it = mMap.find(points[p]);
                    if (it != mMap.cend())
                    {
                        pointIndices[p] = it->second;
                    }
                    else
                    {
                        pointIndices[p] = -1;
                    }
                }
            }

            Vec3ToIndexMap mMap;
        };

        // interface functions

        uint64_t createPointFinder(const carb::Float3* points, const uint32_t pointsSize)
        {
            PointFinder* pointFinder = ICE_NEW(PointFinder)(points, pointsSize);
            return uint64_t(pointFinder);
        }

        void releasePointFinder(const uint64_t pointFinder)
        {
            PointFinder* pf = reinterpret_cast<PointFinder*>(pointFinder);
            SAFE_DELETE_SINGLE(pf);
        }

        void pointsToIndices(int32_t* pointIndices, const uint64_t pointFinder, const carb::Float3* points, const uint32_t pointsSize)
        {
            PointFinder* pf = reinterpret_cast<PointFinder*>(pointFinder);
            pf->pointToIndices(pointIndices, points, pointsSize);
        }
    }
}
