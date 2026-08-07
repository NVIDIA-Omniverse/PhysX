// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

// Common header for platform-specific includes
// Use this instead of directly including platform headers to ensure consistent configuration

#ifdef _WIN32
    // Configure Windows.h to be minimal and avoid conflicts
    #define WIN32_LEAN_AND_MEAN  // Exclude rarely-used Windows headers
    #define NOMINMAX             // Prevent min/max macros that conflict with std::min/max
    #include <windows.h>
#else
    // Linux/Unix includes for dynamic loading
    #include <dlfcn.h>
#endif
