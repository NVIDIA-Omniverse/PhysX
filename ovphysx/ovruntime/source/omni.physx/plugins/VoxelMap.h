// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-8
 */

#pragma once

#include <omni/physics/parse/Handles.h> // ObjectKey

bool setVoxelRange(long int stageId,
                   omni::physics::parse::ObjectKey key,
                   const int sx,
                   const int sy,
                   const int sz,
                   const int ex,
                   const int ey,
                   const int ez,
                   const int type,
                   const int subType,
                   const int update);
