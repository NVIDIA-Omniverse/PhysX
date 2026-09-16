// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

// Export macro for the ovphysx internal sidecar. Same pattern as the main library,
// with a separate define. CMake defines OVPHYSX_INTERNAL_API_EXPORTS only when
// building the sidecar DLL, so consumers such as the main library see dllimport.

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
