// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

namespace omni
{
namespace physics
{
namespace tensors
{

constexpr int kMaxDimensions = 8;

/// Data types the tensors can be
enum class TensorDataType
{
    eNone,
    eFloat32,
    eFloat64,
    eInt8,
    eInt16,
    eInt32,
    eInt64,
    eUint8,
    eUint16,
    eUint32,
    eUint64,
};

/// Generic tensor descriptor for CPU or GPU tensors used for interop with other frameworks
/**
 */
struct TensorDesc
{
    int device = -1; //!< -1 for CPU, 0 or more for GPU device ordinal
    TensorDataType dtype = TensorDataType::eNone; //!< Type of data in the tensor.
    int numDims = 0; //!< Rank of the tensor. The maximum rank of a tensor is OMNI_PHYSX_DOI_MAX_TENSOR_DIMENSIONS.
    int dims[kMaxDimensions] = { 0 }; //!< Size of each dimension of the tensor.
    void* data = nullptr; //!< Pointer to the data.
    bool ownData = false; //!< Whether this tensor owns the data allocation
};

} // namespace tensors
} // namespace physics
} // namespace omni
