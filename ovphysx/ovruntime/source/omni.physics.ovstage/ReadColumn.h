// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-FEED-005
 * @covers AC-8
 */

#pragma once

#include <ovstage/ovstage.h>

#include <cstddef>
#include <cstdint>
#include <limits>

namespace omni::physics::ovstage
{

// A borrowed fixed-size host column. Prim-list indexes identify keys; data indexes
// independently select rows in the full tensor (including reorder and broadcast).
struct ReadColumn
{
    const DLTensor* tensor = nullptr;
    const uint32_t* indexes = nullptr;
    const uint64_t* mask = nullptr;
    int64_t components = 0;
    size_t rowBytes = 0;

    bool present(uint32_t logicalRow) const
    {
        return !mask || (mask[logicalRow / 64] & (uint64_t(1) << (logicalRow % 64))) != 0;
    }

    uint32_t row(uint32_t logicalRow) const
    {
        return indexes ? indexes[logicalRow] : logicalRow;
    }

    const uint8_t* data(uint32_t logicalRow) const
    {
        return static_cast<const uint8_t*>(tensor->data) + tensor->byte_offset + row(logicalRow) * rowBytes;
    }
};

inline bool readColumn(const ovstage_read_group_t& group, ReadColumn& out)
{
    if (group.is_delete || group.is_array || group.prims.count == 0 ||
        group.data.tensor_count != 1 || !group.data.tensors ||
        (group.data.index_map && group.data.mask) ||
        ((group.data.index_map || group.data.mask) && group.data.count < group.prims.count))
        return false;

    const DLTensor& t = group.data.tensors[0];
    if (!t.data || t.device.device_type != kDLCPU || t.ndim < 1 || !t.shape ||
        t.dtype.lanes == 0 || t.dtype.bits == 0 || t.dtype.bits % 8 != 0)
        return false;

    int64_t elements = 1;
    for (int i = t.ndim; i-- > 0;)
    {
        if (t.shape[i] <= 0 || (t.strides && t.shape[i] > 1 && t.strides[i] != elements) ||
            t.shape[i] > std::numeric_limits<int64_t>::max() / elements)
            return false;
        elements *= t.shape[i];
    }
    if (elements > std::numeric_limits<int64_t>::max() / t.dtype.lanes)
        return false;
    elements *= t.dtype.lanes;

    // With a gather the tensor may contain many more rows than the changed prim
    // group. Dividing by prims.count (or max(index_map)+1) gives the wrong stride.
    const int64_t storedRows = (group.data.index_map || group.data.mask) ? t.shape[0] : group.prims.count;
    if (elements % storedRows != 0)
        return false;
    const int64_t components = elements / storedRows;
    const size_t scalarBytes = t.dtype.bits / 8;
    if (t.byte_offset > std::numeric_limits<size_t>::max() ||
        static_cast<uint64_t>(elements) > (std::numeric_limits<size_t>::max() - t.byte_offset) / scalarBytes)
        return false;
    ReadColumn column{ &t, group.data.index_map, group.data.mask, components,
                       static_cast<size_t>(components) * scalarBytes };
    for (uint32_t i = 0; i < group.prims.count; ++i)
        if (column.present(i) && column.row(i) >= static_cast<uint64_t>(storedRows))
            return false;
    out = column;
    return true;
}

} // namespace omni::physics::ovstage
