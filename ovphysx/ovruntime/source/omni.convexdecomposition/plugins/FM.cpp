// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


// This include must come first
// clang-format off
// clang-format on

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include "FM.h"
#include <vector>

#define REAL float

#include "FM.inl"

#undef REAL
#define REAL double

#include "FM.inl"
