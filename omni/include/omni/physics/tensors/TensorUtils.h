// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "TensorDesc.h"

#include <carb/logging/Log.h>

#include <array>
#include <initializer_list>
#include <string>

namespace omni
{
namespace physics
{
namespace tensors
{

class TensorShape
{
public:
    TensorShape(std::initializer_list<size_t> dims)
    {
        size_t count = 0;
        for (auto it = dims.begin(); it != dims.end(); ++it)
        {
            // Defensive: TensorDesc supports up to kMaxDimensions. If a caller passes
            // more dims (not expected in normal use), ignore the extras.
            if (count >= size_t(kMaxDimensions))
            {
                break;
            }
            mDims[count++] = *it;
        }
        mNumDims = count;
    }

    size_t numDims() const
    {
        return mNumDims;
    }

    size_t operator[](size_t i) const
    {
        return mDims[i];
    }

    size_t getTotalSize() const
    {
        if (mNumDims == 0)
        {
            return 0;
        }
        size_t totalSize = size_t(mDims[0]);
        for (size_t i = 1; i < mNumDims; i++)
        {
            totalSize *= size_t(mDims[i]);
        }
        return totalSize;
    }

    std::string getString() const
    {
        // this is a sub-par implementation, but a workaround for a bug with std stream output
        if (mNumDims == 0)
        {
            return "(0)";
        }
        else if (mNumDims == 1)
        {
            return "(" + std::to_string(mDims[0]) + ")";
        }
        else
        {
            std::string out = "(";
            size_t i;
            for (i = 0; i < mNumDims - 1; i++)
            {
                out += std::to_string(mDims[i]);
                out += ", ";
            }
            out += std::to_string(mDims[i]);
            out += ")";
            return out;
        }
    }

private:
    size_t mDims[kMaxDimensions];
    size_t mNumDims;
};

inline size_t getTensorTotalSize(const TensorDesc& tensor)
{
    if (tensor.numDims <= 0)
    {
        return 0;
    }
    size_t totalSize = size_t(tensor.dims[0]);
    for (int i = 1; i < tensor.numDims; i++)
    {
        totalSize *= size_t(tensor.dims[i]);
    }
    return totalSize;
}

inline std::string getTensorShapeString(const TensorDesc& tensor)
{
    // this is a sub-par implementation, but a workaround for a bug with std stream output
    if (tensor.numDims < 0 || tensor.numDims > kMaxDimensions)
    {
        return "(invalid dimensions)";
    }
    else if (tensor.numDims == 0)
    {
        return "(0)";
    }
    else if (tensor.numDims == 1)
    {
        return "(" + std::to_string(tensor.dims[0]) + ")";
    }
    else
    {
        std::string out = "(";
        int i = 0;
        for (i = 0; i < tensor.numDims - 1; i++)
        {
            out += std::to_string(tensor.dims[i]);
            out += ", ";
        }
        out += std::to_string(tensor.dims[i]);
        out += ")";
        return out;
    }
}

inline const char* getTensorDtypeCstr(const TensorDesc& tensor)
{
    switch (tensor.dtype)
    {
    case TensorDataType::eFloat32:
        return "32-bit floating point";
    case TensorDataType::eFloat64:
        return "64-bit floating point";
    case TensorDataType::eInt8:
        return "8-bit integer";
    case TensorDataType::eInt16:
        return "16-bit integer";
    case TensorDataType::eInt32:
        return "32-bit integer";
    case TensorDataType::eInt64:
        return "64-bit integer";
    case TensorDataType::eUint8:
        return "8-bit unsigned integer";
    case TensorDataType::eUint16:
        return "16-bit unsigned integer";
    case TensorDataType::eUint32:
        return "32-bit unsigned integer";
    case TensorDataType::eUint64:
        return "64-bit unsigned integer";

    case TensorDataType::eNone:
    default:
        return "unsupported data type";
    }
}

inline bool checkTensorDevice(const TensorDesc& tensor, int expectedDevice, const char* tensorName, const char* funcName)
{
    if (tensor.device != expectedDevice)
    {
        CARB_LOG_ERROR("Incompatible device of %s tensor in function %s: expected device %d, received device %d",
                       tensorName, funcName, expectedDevice, tensor.device);
        return false;
    }
    return true;
}

inline bool checkTensorFloat32(const TensorDesc& tensor, const char* tensorName, const char* funcName)
{
    if (tensor.dtype != TensorDataType::eFloat32)
    {
        CARB_LOG_ERROR("Incompatible data type of %s tensor in function %s: expected 32-bit floating point, received %s",
                       tensorName, funcName, getTensorDtypeCstr(tensor));
        return false;
    }
    return true;
}

inline bool checkTensorInt32(const TensorDesc& tensor, const char* tensorName, const char* funcName)
{
    if (tensor.dtype != TensorDataType::eInt32 && tensor.dtype != TensorDataType::eUint32)
    {
        CARB_LOG_ERROR("Incompatible data type of %s tensor in function %s: expected 32-bit integer, received %s",
                       tensorName, funcName, getTensorDtypeCstr(tensor));
        return false;
    }
    return true;
}

inline bool checkTensorInt8(const TensorDesc& tensor, const char* tensorName, const char* funcName)
{
    if (tensor.dtype != TensorDataType::eInt8 && tensor.dtype != TensorDataType::eUint8)
    {
        CARB_LOG_ERROR("Incompatible data type of %s tensor in function %s: expected 8-bit integer, received %s",
                       tensorName, funcName, getTensorDtypeCstr(tensor));
        return false;
    }
    return true;
}

inline bool checkTensorInt64(const TensorDesc& tensor, const char* tensorName, const char* funcName)
{
    if (tensor.dtype != TensorDataType::eInt64 && tensor.dtype != TensorDataType::eUint64)
    {
        CARB_LOG_ERROR("Incompatible data type of %s tensor in function %s: expected 64-bit integer, received %s",
                       tensorName, funcName, getTensorDtypeCstr(tensor));
        return false;
    }
    return true;
}

inline bool checkTensorSizeExact(const TensorDesc& tensor, size_t expectedSize, const char* tensorName, const char* funcName)
{
    size_t totalSize = getTensorTotalSize(tensor);
    if (totalSize != expectedSize)
    {
        CARB_LOG_ERROR(
            "Incompatible size of %s tensor in function %s: expected total size %llu, received total size %llu with shape %s",
            tensorName, funcName, (unsigned long long)expectedSize, (unsigned long long)totalSize,
            getTensorShapeString(tensor).c_str());
        return false;
    }
    return true;
}

inline bool checkTensorSizeMinimum(const TensorDesc& tensor, size_t expectedSize, const char* tensorName, const char* funcName)
{
    size_t totalSize = getTensorTotalSize(tensor);
    if (totalSize < expectedSize)
    {
        CARB_LOG_ERROR(
            "Incompatible size of %s tensor in function %s: expected total size %llu, received total size %llu with shape %s",
            tensorName, funcName, (unsigned long long)expectedSize, (unsigned long long)totalSize,
            getTensorShapeString(tensor).c_str());
        return false;
    }
    return true;
}

inline bool checkTensorSizeExact(const TensorDesc& tensor,
                                 const TensorShape& expectedShape,
                                 const char* tensorName,
                                 const char* funcName)
{
    size_t totalSize = getTensorTotalSize(tensor);
    size_t expectedSize = expectedShape.getTotalSize();
    if (totalSize != expectedSize)
    {
        CARB_LOG_ERROR("Incompatible shape of %s tensor in function %s: expected %s, received %s", tensorName, funcName,
                       expectedShape.getString().c_str(), getTensorShapeString(tensor).c_str());
        return false;
    }
    return true;
}

inline bool checkTensorSizeMinimum(const TensorDesc& tensor,
                                   const TensorShape& expectedShape,
                                   const char* tensorName,
                                   const char* funcName)
{
    size_t totalSize = getTensorTotalSize(tensor);
    size_t expectedSize = expectedShape.getTotalSize();
    if (totalSize < expectedSize)
    {
        CARB_LOG_ERROR("Incompatible shape of %s tensor in function %s: expected %s, received %s", tensorName, funcName,
                       expectedShape.getString().c_str(), getTensorShapeString(tensor).c_str());
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Shared CPU-side mask-to-indices resolution.
//
// Validates a uint8 mask tensor (device must match expectedDevice) and populates
// outIndices with the positions of non-zero elements. Used by all CPU-side
// masked setters and by base material views.
//
// Returns:
//   MaskResult::Error   - validation failed (error already logged)
//   MaskResult::Empty   - mask is all zeros, nothing to write
//   MaskResult::All     - mask is all ones, caller should use the unindexed path
//   MaskResult::Partial - outIndices contains the selected indices
// ---------------------------------------------------------------------------
enum class MaskResult { Error, Empty, All, Partial };

inline MaskResult resolveMaskToIndices(const TensorDesc* maskTensor,
                                       uint32_t viewCount,
                                       int expectedDevice,
                                       std::vector<uint32_t>& outIndices,
                                       const char* funcName)
{
    if (!maskTensor || !maskTensor->data)
    {
        CARB_LOG_ERROR("mask tensor is null or has no data in %s", funcName);
        return MaskResult::Error;
    }

    if (!checkTensorDevice(*maskTensor, expectedDevice, "mask", funcName))
        return MaskResult::Error;

    if (maskTensor->dtype != TensorDataType::eUint8)
    {
        CARB_LOG_ERROR("mask tensor must be uint8 in %s", funcName);
        return MaskResult::Error;
    }

    if (getTensorTotalSize(*maskTensor) != viewCount)
    {
        CARB_LOG_ERROR("mask tensor size (%llu) must equal view count (%u) in %s",
                       (unsigned long long)getTensorTotalSize(*maskTensor), viewCount, funcName);
        return MaskResult::Error;
    }

    const uint8_t* mask = static_cast<const uint8_t*>(maskTensor->data);
    outIndices.clear();
    outIndices.reserve(viewCount);
    for (uint32_t i = 0; i < viewCount; ++i)
    {
        if (mask[i])
            outIndices.push_back(i);
    }

    if (outIndices.empty())
        return MaskResult::Empty;
    if (outIndices.size() == viewCount)
        return MaskResult::All;
    return MaskResult::Partial;
}

// Build a 1D uint32 TensorDesc pointing at an index vector. The caller must keep
// the vector alive for the lifetime of the returned TensorDesc.
inline TensorDesc makeIndexTensorDesc(std::vector<uint32_t>& indices, int device)
{
    TensorDesc idx{};
    idx.device = device;
    idx.dtype = TensorDataType::eUint32;
    idx.numDims = 1;
    idx.dims[0] = static_cast<int>(indices.size());
    idx.data = indices.data();
    return idx;
}

} // namespace tensors
} // namespace physics
} // namespace omni
