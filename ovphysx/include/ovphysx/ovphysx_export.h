// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


// Shared DLL export/import macro for ovphysx.
// Included by all public API headers so that symbol visibility is consistent.

#ifndef OVPHYSX_EXPORT_H
#define OVPHYSX_EXPORT_H

// Export/import macros for Windows DLL and GCC visibility.
#if defined(_WIN32) || defined(_WIN64)
    #ifdef OVPHYSX_EXPORTS
        #define OVPHYSX_API __declspec(dllexport)
    #else
        #define OVPHYSX_API __declspec(dllimport)
    #endif
#elif defined(__GNUC__) && __GNUC__ >= 4
    #define OVPHYSX_API __attribute__((visibility("default")))
#else
    #define OVPHYSX_API
#endif

// Cross-platform deprecation macros for public API declarations.
// Usage:
//   OVPHYSX_API OVPHYSX_DEPRECATED return_t func(...);
//   OVPHYSX_API OVPHYSX_DEPRECATED_MSG("use new_func()") return_t old_func(...);
//
// GCC/Clang deliberately use the GNU __attribute__ form, NOT C++ [[deprecated]]:
// OVPHYSX_API expands to __attribute__((visibility("default"))), and a standard
// [[deprecated]] following it in the decl-specifier-seq appertains to the *return
// type*, which Clang rejects ("'deprecated' attribute cannot be applied to types")
// while GCC only tolerates it. The GNU attribute is position-tolerant and binds to
// the function on both. Keep _MSC_VER and __clang__/__GNUC__ ahead of the standard
// [[deprecated]] fallback. Do not move the C++14 branch to the top.
#if defined(_MSC_VER)
    #define OVPHYSX_DEPRECATED __declspec(deprecated)
    #define OVPHYSX_DEPRECATED_MSG(msg) __declspec(deprecated(msg))
#elif defined(__clang__) || defined(__GNUC__)
    #define OVPHYSX_DEPRECATED __attribute__((deprecated))
    #define OVPHYSX_DEPRECATED_MSG(msg) __attribute__((deprecated(msg)))
#elif defined(__cplusplus) && (__cplusplus >= 201402L)
    // Any other conforming C++14 compiler: OVPHYSX_API is empty there, so the
    // standard attribute leads the declaration and correctly binds to the function.
    #define OVPHYSX_DEPRECATED [[deprecated]]
    #define OVPHYSX_DEPRECATED_MSG(msg) [[deprecated(msg)]]
#else
    #define OVPHYSX_DEPRECATED
    #define OVPHYSX_DEPRECATED_MSG(msg)
#endif

#endif // OVPHYSX_EXPORT_H
