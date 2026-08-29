// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

// Export macro for ovphysx internal sidecar
// Follows same pattern as main library but with separate define
//
// When building the internal sidecar DLL:
//   - OVPHYSX_INTERNAL_API_EXPORTS is defined by CMake
//   - OVPHYSX_INTERNAL_API expands to __declspec(dllexport) on Windows
//
// When using the internal sidecar (e.g., main library loading it):
//   - OVPHYSX_INTERNAL_API_EXPORTS is NOT defined
//   - OVPHYSX_INTERNAL_API expands to __declspec(dllimport) on Windows

#if defined(_WIN32) || defined(_WIN64)
    #ifdef OVPHYSX_INTERNAL_API_EXPORTS
        #define OVPHYSX_INTERNAL_API __declspec(dllexport)
    #else
        #define OVPHYSX_INTERNAL_API __declspec(dllimport)
    #endif
#elif defined(__GNUC__) && __GNUC__ >= 4
    #define OVPHYSX_INTERNAL_API __attribute__((visibility("default")))
#else
    #define OVPHYSX_INTERNAL_API
#endif
