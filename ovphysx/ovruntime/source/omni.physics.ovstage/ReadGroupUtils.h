// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-FEED-005
 * @covers AC-1, AC-9
 */

#pragma once

#include <dlpack/dlpack.h>
#include <ovstage/ovstage.h>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace omni::physics::ovstage
{
// One attribute column of a fetched read group, with the group's prim rows. OVStage
// puts the columns that share prim rows in one group; a column with those rows reads
// as a single-attribute group, the shape the readers here consume.
struct ReadGroupColumn
{
    ovx_token_t attribute = OVX_INVALID_TOKEN;
    ovstage_ordinal_t ordinal = 0;
    bool is_delete = false;
    bool is_array = false;
    ovstage_attribute_semantic_t semantic{};
    ovstage_prim_group_t prims{};
    ovstage_data_t data{};
    ovstage_attribute_meta_t meta{};
};

// Fetches a read's groups one at a time and yields their columns in order. A group is
// released after its last column, unless keep() handed it to an owner that releases it
// later with ovstage_release_groups.
class ReadColumnCursor
{
public:
    ReadColumnCursor(ovstage_instance_t* instance, ovstage_read_handle_t read) : mInstance(instance), mRead(read) {}
    ReadColumnCursor(const ReadColumnCursor&) = delete;
    ReadColumnCursor& operator=(const ReadColumnCursor&) = delete;
    ~ReadColumnCursor() { releaseGroup(); }

    // OVSTAGE_OK with the next column in `out`, else the status that ended the read
    // (OVSTAGE_ERROR_END_OF_ITERATION once every group was delivered).
    ovstage_api_status_t next(ReadGroupColumn& out)
    {
        while (!mHeld || mColumn == mGroup.column_count)
        {
            releaseGroup();
            size_t count = 0;
            const ovstage_api_status_t status =
                ovstage_fetch_read_next(mInstance, mRead, OVSTAGE_TIMEOUT_INFINITE, &mGroup, 1, &count);
            if (status != OVSTAGE_OK)
                return status;
            mHeld = true;
            mKept = false;
            mColumn = 0;
        }
        const ovstage_read_column_t& column = mGroup.columns[mColumn++];
        out = { column.attribute, column.ordinal, column.is_delete, column.is_array, column.semantic,
                mGroup.prims,     column.data,    column.meta };
        return OVSTAGE_OK;
    }

    // Hands the group of the column last returned to `groups`, once per group. Its
    // columns stay valid until that owner releases it.
    template <class Groups>
    void keep(Groups& groups)
    {
        if (mHeld && !mKept)
        {
            groups.push_back(mGroup);
            mKept = true;
        }
    }

private:
    void releaseGroup()
    {
        if (mHeld && !mKept)
            ovstage_release_groups(mInstance, &mGroup, 1);
        mHeld = false;
    }

    ovstage_instance_t* mInstance;
    ovstage_read_handle_t mRead;
    ovstage_read_group_t mGroup{};
    size_t mColumn = 0;
    bool mHeld = false;
    bool mKept = false;
};
} // namespace omni::physics::ovstage

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
