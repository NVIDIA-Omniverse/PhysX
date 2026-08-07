// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-CORE-001
 * @covers AC-1
 */

#pragma once

namespace omni::physics::parse
{

/// @brief 3x3 row-major double matrix. Defaults to identity.
/// Vectors transform as row-vectors: `v_out = v_in * R`.
struct Matrix3d
{
    double data[9] = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };
};

/// @brief 4x4 row-major double matrix. Defaults to identity.
/// Used by IPhysicsSource transform accessors and by descriptors that
/// carry world-space transforms.
struct Matrix4d
{
    double data[16] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };
};

} // namespace omni::physics::parse
