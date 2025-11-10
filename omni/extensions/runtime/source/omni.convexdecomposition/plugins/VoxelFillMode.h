// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

namespace vcd
{

enum class VoxelFillMode
{
    eFloodFill, // This is the default behavior, after the voxelization step it uses a flood fill to determine 'inside'
                // from 'outside'. However, meshes with holes can fail and create hollow results.
    eSurfaceOnly, // Only consider the 'surface', will create 'skins' with hollow centers.
    eRaycastFill // Uses raycasting to determine inside from outside.
};

}
