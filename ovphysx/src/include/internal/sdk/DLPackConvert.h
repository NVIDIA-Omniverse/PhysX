// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "ovphysx/ovphysx.h"
#include <omni/physics/tensors/TensorDesc.h>
#include <omni/physics/tensors/TensorApi.h>
#include <omni/physx/PhysXRuntime.h>

#include <cstdint>
#include <limits>

namespace ovphysx
{
namespace internal
{

enum class DLConvertError
{
    Success = 0,
    NullInput,
    UnsupportedDevice,
    InvalidDims,
    NonContiguous,
    UnsupportedDtype,
    ByteOffsetOverflow
};

// Convert DLTensor to TensorDesc with detailed error reporting.
// TensorAPI requires contiguous C-order layout.
inline DLConvertError dlToTensorDesc(const DLTensor* dl, omni::physics::tensors::TensorDesc& out)
{
    if (!dl || !dl->data)
        return DLConvertError::NullInput;

    if (dl->ndim <= 0 || dl->ndim > omni::physics::tensors::kMaxDimensions)
        return DLConvertError::InvalidDims;

    if (!dl->shape)
        return DLConvertError::NullInput;

    switch (dl->device.device_type)
    {
        case kDLCPU:
        case kDLCUDAHost:
            out.device = -1;
            break;
        case kDLCUDA:
        case kDLCUDAManaged:
            if (dl->device.device_id < 0)
                return DLConvertError::UnsupportedDevice;
            out.device = dl->device.device_id;
            break;
        default:
            return DLConvertError::UnsupportedDevice;
    }

    uintptr_t base = reinterpret_cast<uintptr_t>(dl->data);
    uintptr_t offset = static_cast<uintptr_t>(dl->byte_offset);

    if (offset > 0 && offset > (std::numeric_limits<uintptr_t>::max() - base))
        return DLConvertError::ByteOffsetOverflow;

    out.data = reinterpret_cast<void*>(base + offset);
    out.ownData = false;

    out.numDims = static_cast<int>(dl->ndim);

    for (int i = 0; i < out.numDims; i++)
    {
        if (dl->shape[i] < 0 || dl->shape[i] > INT32_MAX)
            return DLConvertError::InvalidDims;
        out.dims[i] = static_cast<int>(dl->shape[i]);
    }

    if (dl->strides != nullptr)
    {
        // Any zero-sized dim makes the tensor empty; contiguity is trivially
        // satisfied (no data to lay out). NumPy reports stride 0 for the dims
        // outside a zero-sized one, which would otherwise fail the per-dim
        // stride check below. Mirror the Python-side guard in _dlpack_utils.py.
        bool hasZeroDim = false;
        for (int i = 0; i < out.numDims; ++i)
        {
            if (dl->shape[i] == 0)
            {
                hasZeroDim = true;
                break;
            }
        }
        if (!hasZeroDim)
        {
            int64_t expected_stride = 1;
            for (int i = out.numDims - 1; i >= 0; --i)
            {
                const int64_t dim = dl->shape[i];

                // Dim of size 1 has irrelevant stride and contributes a
                // factor of 1 to expected_stride, so skip.
                if (dim == 1)
                    continue;

                if (dl->strides[i] != expected_stride)
                    return DLConvertError::NonContiguous;

                if (expected_stride > (std::numeric_limits<int64_t>::max() / dim))
                    return DLConvertError::InvalidDims;

                expected_stride *= dim;
            }
        }
    }

    if (dl->dtype.lanes != 1)
        return DLConvertError::UnsupportedDtype;

    if (dl->dtype.code == kDLFloat && dl->dtype.bits == 32)
        out.dtype = omni::physics::tensors::TensorDataType::eFloat32;
    else if (dl->dtype.code == kDLInt && dl->dtype.bits == 32)
        out.dtype = omni::physics::tensors::TensorDataType::eInt32;
    else if (dl->dtype.code == kDLUInt && dl->dtype.bits == 32)
        out.dtype = omni::physics::tensors::TensorDataType::eUint32;
    else if (dl->dtype.code == kDLInt && dl->dtype.bits == 64)
        out.dtype = omni::physics::tensors::TensorDataType::eInt64;
    else if (dl->dtype.code == kDLUInt && dl->dtype.bits == 64)
        out.dtype = omni::physics::tensors::TensorDataType::eUint64;
    else if (dl->dtype.code == kDLUInt && dl->dtype.bits == 8)
        out.dtype = omni::physics::tensors::TensorDataType::eUint8;
    else if (dl->dtype.code == kDLBool && dl->dtype.bits == 8)
        out.dtype = omni::physics::tensors::TensorDataType::eUint8;
    else
        return DLConvertError::UnsupportedDtype;

    return DLConvertError::Success;
}

inline const char* dlConvertErrorMessage(DLConvertError err)
{
    switch (err)
    {
        case DLConvertError::Success: return nullptr;
        case DLConvertError::NullInput: return "tensor has null data or shape";
        case DLConvertError::UnsupportedDevice: return "unsupported device type (only CPU/CUDA supported)";
        case DLConvertError::InvalidDims: return "invalid tensor dimensions";
        case DLConvertError::NonContiguous: return "non-contiguous tensor not supported (contiguous strides are OK; make the tensor C-order contiguous)";
        case DLConvertError::UnsupportedDtype: return "unsupported dtype (only float32/int32/uint32/int64/uint64/uint8/bool supported)";
        case DLConvertError::ByteOffsetOverflow: return "byte_offset causes pointer overflow";
        default: return "unknown tensor conversion error";
    }
}

inline omni::physics::tensors::TensorApi* getTensorApi()
{
    return omni::physx::runtime::tryGetTensorApiInterface();
}

} // namespace internal
} // namespace ovphysx
