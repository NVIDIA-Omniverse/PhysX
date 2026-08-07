// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <cstdint>

namespace ovphysx
{
namespace internal
{

// Convert wrench data from AoS [count,9] to SoA contiguous layout:
//   forces[count,3] | torques[count,3] | positions[count,3]
//
// Input row layout: [fx,fy,fz, tx,ty,tz, px,py,pz]
inline void convertWrenchAosToSoaCpu(const float* srcAos, int64_t count, float* dstSoa)
{
    float* forces = dstSoa;
    float* torques = dstSoa + count * 3;
    float* positions = dstSoa + count * 6;

    for (int64_t i = 0; i < count; ++i)
    {
        const float* row = srcAos + i * 9;
        forces[i * 3 + 0] = row[0];
        forces[i * 3 + 1] = row[1];
        forces[i * 3 + 2] = row[2];
        torques[i * 3 + 0] = row[3];
        torques[i * 3 + 1] = row[4];
        torques[i * 3 + 2] = row[5];
        positions[i * 3 + 0] = row[6];
        positions[i * 3 + 1] = row[7];
        positions[i * 3 + 2] = row[8];
    }
}

} // namespace internal
} // namespace ovphysx
