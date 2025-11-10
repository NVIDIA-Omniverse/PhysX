// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

static pxr::TfToken gInfiniteVoxelMapAPI("InfiniteVoxelMapAPI");

bool setVoxelRange(long int stageId,
                   const pxr::SdfPath& path,
                   const int sx,
                   const int sy,
                   const int sz,
                   const int ex,
                   const int ey,
                   const int ez,
                   const int type,
                   const int subType,
                   const int update);
