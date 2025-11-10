// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>

#define MKS 0
#define CGS 1

#define UNITS MKS // Set to MKS or CGS
#define ENABLE_STABILIZATION_AND_CCD 0 // More expensive but more stable
#define USE_PHYSX_GPU 1 // GPU Rigid Bodies

#define ENABLE_FABRIC_FOR_PARTICLE_SETS 1

#define SQRT_FLT_MAX 1e16f

const float kAlmostZero = 1e-5f;

const uint64_t kPhysicsProfilerMask = 1 << 8;
