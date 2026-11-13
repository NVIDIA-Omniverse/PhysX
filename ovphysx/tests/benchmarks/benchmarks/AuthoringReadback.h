// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-BENCHMARK-008
 * @covers AC-4
 *
 * @implements REQ-CAPI-BENCHMARK-009
 * @covers AC-4
 */

#pragma once

#include <ovstage/ovstage.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace authoringbm
{

// OVStage can return vector components in dtype.lanes or as a trailing scalar
// dimension. DLPack strides count dtype elements, so only packed row strides
// need multiplication by the component count. The caller resolves group maps.
template <typename T>
bool readSpawnTensor(const DLTensor& tensor, uint32_t dataRow, uint8_t code,
                     uint16_t components, bool isArray, size_t scalarCount,
                     std::array<T, 4>& actual)
{
    if (!tensor.data || tensor.device.device_type != kDLCPU || !tensor.shape ||
        tensor.dtype.code != code || tensor.dtype.bits != sizeof(T) * 8 ||
        components == 0 || scalarCount == 0 || scalarCount > actual.size() || scalarCount % components != 0)
        return false;

    const bool packed = tensor.ndim == 1 && tensor.dtype.lanes == components;
    const bool expanded = tensor.ndim == 2 && tensor.dtype.lanes == 1 && tensor.shape[1] == components;
    if ((!packed && !expanded) || tensor.shape[0] <= 0 ||
        (isArray ? tensor.shape[0] != static_cast<int64_t>(scalarCount / components) :
                   tensor.shape[0] <= static_cast<int64_t>(dataRow) || scalarCount != components))
        return false;

    const int64_t rowStride = tensor.strides ? tensor.strides[0] : (expanded ? components : 1);
    const int64_t componentStride = expanded && tensor.strides ? tensor.strides[1] : 1;
    if (rowStride <= 0 || componentStride <= 0)
        return false;

    const char* const bytes = static_cast<const char*>(tensor.data) + tensor.byte_offset;
    for (size_t i = 0; i < scalarCount; ++i)
    {
        const size_t row = isArray ? i / components : dataRow;
        const size_t scalar = row * static_cast<size_t>(rowStride) * tensor.dtype.lanes +
                              (i % components) * static_cast<size_t>(componentStride);
        std::memcpy(&actual[i], bytes + scalar * sizeof(T), sizeof(T));
    }
    return true;
}

} // namespace authoringbm
