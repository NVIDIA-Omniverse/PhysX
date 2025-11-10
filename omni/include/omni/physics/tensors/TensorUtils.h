// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
            // eek?
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

} // namespace tensors
} // namespace physics
} // namespace omni
