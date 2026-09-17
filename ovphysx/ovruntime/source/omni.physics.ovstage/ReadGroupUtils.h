// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <dlpack/dlpack.h>
#include <cstdint>
#include <limits>

namespace omni::physics::ovstage::detail
{
// OVStage publishes explicit compact strides; older producers may use nullptr.
// Strides count complete dtype elements, including all lanes of a tuple.
inline bool isCompactReadTensor(const DLTensor& tensor)
{
    if (tensor.ndim < 0 || (tensor.ndim > 0 && !tensor.shape) || tensor.dtype.lanes == 0)
        return false;
    int64_t elements = 1;
    for (int i = 0; i < tensor.ndim; ++i)
    {
        const int64_t extent = tensor.shape[i];
        if (extent < 0 || (extent > 0 && elements > std::numeric_limits<int64_t>::max() / extent))
            return false;
        elements *= extent;
    }
    // Empty arrays address no elements, so their strides do not constrain a copy.
    if (elements == 0)
        return true;
    int64_t stride = 1;
    for (int i = tensor.ndim - 1; i >= 0; --i)
    {
        const int64_t extent = tensor.shape[i];
        if (extent <= 0 || (tensor.strides && extent > 1 && tensor.strides[i] != stride) ||
            stride > std::numeric_limits<int64_t>::max() / extent)
            return false;
        stride *= extent;
    }
    return stride <= std::numeric_limits<int64_t>::max() / tensor.dtype.lanes;
}
} // namespace omni::physics::ovstage::detail
