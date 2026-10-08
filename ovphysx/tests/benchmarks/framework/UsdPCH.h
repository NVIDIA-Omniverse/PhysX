// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Minimal shim. omni.physx ships a real precompiled header named "UsdPCH.h"
// that pulls in USD, std and other dependencies. The framework files copied
// unmodified from omni.physx (BmTime.cpp, BmOutput.cpp, BmGlobals.cpp)
// include it as the first line but only depend on it transitively bringing
// in std-library types like uint32_t/uint64_t. Only those are re-exposed
// here so the originals compile in ovphysx without per-file edits.

#pragma once

#include <carb/logging/Log.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
