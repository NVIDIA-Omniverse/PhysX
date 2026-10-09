// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-OVSTAGE-OUTPUT-001
 * @covers AC-2 AC-6
 */

#pragma once

#include <cfloat>
#include <cmath>

#if defined(__CUDACC__)
#    define OVPHYSX_OUTPUT_HOST_DEVICE __host__ __device__
#else
#    define OVPHYSX_OUTPUT_HOST_DEVICE
#endif

namespace ovphysx::utils::detail
{

OVPHYSX_OUTPUT_HOST_DEVICE inline bool finiteOutputValue(double value)
{
    return value >= -DBL_MAX && value <= DBL_MAX;
}

OVPHYSX_OUTPUT_HOST_DEVICE inline bool composeWorldMatrix(const float* position,
                                                          const float* orientation,
                                                          const double* scale,
                                                          double* matrix)
{
    for (int i = 0; i < 3; ++i)
    {
        if (!finiteOutputValue(position[i]) || !finiteOutputValue(scale[i]))
            return false;
    }
    double lengthSquared = 0.0;
    for (int i = 0; i < 4; ++i)
    {
        if (!finiteOutputValue(orientation[i]))
            return false;
        const double component = orientation[i];
        lengthSquared += component * component;
    }
    if (lengthSquared == 0.0)
        return false;

    const double inverseLength = 1.0 / ::sqrt(lengthSquared);
    const double x = orientation[0] * inverseLength;
    const double y = orientation[1] * inverseLength;
    const double z = orientation[2] * inverseLength;
    const double w = orientation[3] * inverseLength;

    // Row-vector rotation with the preserved signed scale applied to each row.
    matrix[0] = scale[0] * (1.0 - 2.0 * (y * y + z * z));
    matrix[1] = scale[0] * (2.0 * (x * y + w * z));
    matrix[2] = scale[0] * (2.0 * (x * z - w * y));
    matrix[3] = 0.0;
    matrix[4] = scale[1] * (2.0 * (x * y - w * z));
    matrix[5] = scale[1] * (1.0 - 2.0 * (x * x + z * z));
    matrix[6] = scale[1] * (2.0 * (y * z + w * x));
    matrix[7] = 0.0;
    matrix[8] = scale[2] * (2.0 * (x * z + w * y));
    matrix[9] = scale[2] * (2.0 * (y * z - w * x));
    matrix[10] = scale[2] * (1.0 - 2.0 * (x * x + y * y));
    matrix[11] = 0.0;
    matrix[12] = position[0];
    matrix[13] = position[1];
    matrix[14] = position[2];
    matrix[15] = 1.0;
    for (int i = 0; i < 16; ++i)
    {
        if (!finiteOutputValue(matrix[i]))
            return false;
    }
    return true;
}

} // namespace ovphysx::utils::detail

#undef OVPHYSX_OUTPUT_HOST_DEVICE
