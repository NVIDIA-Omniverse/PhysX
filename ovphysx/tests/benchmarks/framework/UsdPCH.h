// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// Minimal shim. omni.physx ships a real precompiled header named "UsdPCH.h"
// that pulls in USD, std and other dependencies. The framework files we
// copied unmodified (BmTime.cpp, BmOutput.cpp, BmGlobals.cpp) include it as
// the first line but only depend on it transitively bringing in std-library
// types like uint32_t/uint64_t. We re-expose just those here so the
// originals compile in ovphysx without any per-file edits.

#pragma once

#include <carb/logging/Log.h>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
