// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

// Common header for platform-specific includes. Include this instead of the
// platform headers directly so the configuration stays consistent.

#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN  // Exclude rarely-used Windows headers
    #define NOMINMAX             // Prevent min/max macros that conflict with std::min/max
    #include <windows.h>
#else
    // Linux/Unix includes for dynamic loading
    #include <dlfcn.h>
#endif
