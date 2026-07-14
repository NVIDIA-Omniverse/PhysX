// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

static PXR_NS::TfToken gInfiniteVoxelMapAPI("InfiniteVoxelMapAPI");

bool setVoxelRange(long int stageId,
                   const PXR_NS::SdfPath& path,
                   const int sx,
                   const int sy,
                   const int sz,
                   const int ex,
                   const int ey,
                   const int ez,
                   const int type,
                   const int subType,
                   const int update);
