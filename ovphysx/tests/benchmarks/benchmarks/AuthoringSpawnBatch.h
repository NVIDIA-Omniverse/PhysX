// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-BENCHMARK-009
 * @covers AC-2 AC-3
 */

#pragma once

#include <ovstage/ovstage.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>

namespace authoringbm
{

// Borrowed tensors need independent data, shape and descriptor storage until
// the batch operation completes. Never reset this owner while a write is live.
// Attribute names passed here must also outlive that operation.
class SpawnBatchPayload
{
public:
    static constexpr size_t kCapacity = 8;
    static constexpr uint16_t kFloatCapacity = 4;
    static constexpr size_t kTokenCapacity = 2;

    SpawnBatchPayload() = default;
    SpawnBatchPayload(const SpawnBatchPayload&) = delete;
    SpawnBatchPayload& operator=(const SpawnBatchPayload&) = delete;
    SpawnBatchPayload(SpawnBatchPayload&&) = delete;
    SpawnBatchPayload& operator=(SpawnBatchPayload&&) = delete;

    void reset() { mCount = 0; }
    size_t size() const { return mCount; }
    const ovstage_attribute_write_t* writes() const { return mWrites.data(); }

    bool addFloat(std::string_view name, const float* values, uint16_t lanes,
                  ovstage_attribute_semantic_t semantic)
    {
        if (!name.data() || !values || lanes == 0 || lanes > kFloatCapacity || mCount == kCapacity)
            return false;
        Column& column = append(name, kDLFloat, 32, lanes, 1, false, semantic);
        std::memcpy(column.f32.data(), values, sizeof(float) * lanes);
        column.tensor.data = column.f32.data();
        return true;
    }

    bool addDouble(std::string_view name, double value)
    {
        if (!name.data() || mCount == kCapacity)
            return false;
        Column& column = append(name, kDLFloat, 64, 1, 1, false, OVSTAGE_SEMANTIC_NONE);
        column.f64 = value;
        column.tensor.data = &column.f64;
        return true;
    }

    bool addTokens(std::string_view name, const uint64_t* values, size_t count, bool isArray,
                   ovstage_attribute_semantic_t semantic)
    {
        if (!name.data() || !values || count == 0 || count > kTokenCapacity ||
            (!isArray && count != 1) || mCount == kCapacity)
            return false;
        Column& column = append(name, kDLUInt, 64, 1, static_cast<int64_t>(count), isArray, semantic);
        std::memcpy(column.tokens.data(), values, sizeof(uint64_t) * count);
        column.tensor.data = column.tokens.data();
        return true;
    }

private:
    struct Column
    {
        std::array<float, kFloatCapacity> f32{};
        double f64 = 0.0;
        std::array<uint64_t, kTokenCapacity> tokens{};
        int64_t shape = 1;
        DLTensor tensor{};
    };

    Column& append(std::string_view name, uint8_t code, uint8_t bits, uint16_t lanes,
                   int64_t shape, bool isArray, ovstage_attribute_semantic_t semantic)
    {
        Column& column = mColumns[mCount];
        column = {};
        column.shape = shape;
        column.tensor.device.device_type = kDLCPU;
        column.tensor.ndim = 1;
        column.tensor.shape = &column.shape;
        column.tensor.dtype.code = code;
        column.tensor.dtype.bits = bits;
        column.tensor.dtype.lanes = lanes;
        ovstage_attribute_write_t& write = mWrites[mCount++];
        write = {};
        write.attribute.string.ptr = name.data();
        write.attribute.string.length = name.size();
        write.data.tensors = &column.tensor;
        write.data.tensor_count = 1;
        write.data.is_array = isArray;
        write.data.semantic = semantic;
        return column;
    }

    std::array<Column, kCapacity> mColumns{};
    std::array<ovstage_attribute_write_t, kCapacity> mWrites{};
    size_t mCount = 0;
};

} // namespace authoringbm
