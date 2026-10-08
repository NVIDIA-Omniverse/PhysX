// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

// PT: TODO: refactor
#ifdef __linux__
    #define __forceinline __attribute__((always_inline))
#endif

#define OV_FL __FILE__, __LINE__
