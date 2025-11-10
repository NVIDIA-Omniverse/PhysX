// SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//


#ifndef QH_PREPROCESSOR_H
#define QH_PREPROCESSOR_H

#include <stddef.h>
#if !defined(QH_GENERATE_META_DATA)
#include <ciso646>  
#endif
/** \addtogroup foundation
  @{
*/

#define QH_STRINGIZE_HELPER(X) #X
#define QH_STRINGIZE(X) QH_STRINGIZE_HELPER(X)

#define QH_CONCAT_HELPER(X, Y) X##Y
#define QH_CONCAT(X, Y) QH_CONCAT_HELPER(X, Y)

/*
The following preprocessor identifiers specify compiler, OS, and architecture.
All definitions have a value of 1 or 0, use '#if' instead of '#ifdef'.
*/

/**
Compiler defines, see http://sourceforge.net/p/predef/wiki/Compilers/
*/
#if defined(_MSC_VER)
#if _MSC_VER >= 1910
#define QH_VC 15
#elif _MSC_VER >= 1900
#define QH_VC 14
#elif _MSC_VER >= 1800
#define QH_VC 12
#elif _MSC_VER >= 1700
#define QH_VC 11
#elif _MSC_VER >= 1600
#define QH_VC 10
#elif _MSC_VER >= 1500
#define QH_VC 9
#else
#error "Unknown VC version"
#endif
#elif defined(__clang__)
#define QH_CLANG 1
	#if defined (__clang_major__) 
		#define QH_CLANG_MAJOR __clang_major__
	#elif defined (_clang_major)
		#define QH_CLANG_MAJOR _clang_major
	#else
		#define QH_CLANG_MAJOR 0
	#endif	
#elif defined(__GNUC__) // note: __clang__ implies __GNUC__
#define QH_GCC 1
#else
#error "Unknown compiler"
#endif

/**
Operating system defines, see http://sourceforge.net/p/predef/wiki/OperatingSystems/
*/
#if defined(_XBOX_ONE)
#define QH_XBOXONE 1
#elif defined(_GAMING_XBOX) || defined (_GAMING_XBOX_SCARLETT)
#define QH_XBOX_SERIES_X 1
#elif defined(WINAPI_FAMILY) && WINAPI_FAMILY == WINAPI_FAMILY_APP
#define QH_UWP 1
#elif defined(_WIN64) // note: _XBOX_ONE implies _WIN64
#define QH_WIN64 1
#elif defined(_WIN32) // note: _M_PPC implies _WIN32
#define QH_WIN32 1
#elif defined(__ANDROID__)
#define QH_ANDROID 1
#elif defined(__linux__) || defined (__EMSCRIPTEN__) // note: __ANDROID__ implies __linux__
#define QH_LINUX 1
#elif defined(__APPLE__) && (defined(__arm__) || defined(__arm64__))
#define QH_IOS 1
#elif defined(__APPLE__)
#define QH_OSX 1
#elif defined(__ORBIS__)
#define QH_PS4 1
#elif defined(__NX__)
#define QH_SWITCH 1
#else
#error "Unknown operating system"
#endif

/**
Architecture defines, see http://sourceforge.net/p/predef/wiki/Architectures/
*/
#if defined(__x86_64__) || defined(_M_X64) // ps4 compiler defines _M_X64 without value
#define QH_X64 1
#elif defined(__i386__) || defined(_M_IX86) || defined (__EMSCRIPTEN__)
#define QH_X86 1
#elif defined(__arm64__) || defined(__aarch64__) || defined(_M_ARM64)
#define QH_A64 1
#elif defined(__arm__) || defined(_M_ARM)
#define QH_ARM 1
#elif defined(__ppc__) || defined(_M_PPC) || defined(__CELLOS_LV2__)
#define QH_PPC 1
#else
#error "Unknown architecture"
#endif

/**
SIMD defines
*/
#if !defined(QH_SIMD_DISABLED)
#if defined(__i386__) || defined(_M_IX86) || defined(__x86_64__) || defined(_M_X64) || (defined (__EMSCRIPTEN__) && defined(__SSE2__))
#define QH_SSE2 1
#endif
#if defined(_M_ARM) || defined(__ARM_NEON__) || defined(__ARM_NEON)
#define QH_NEON 1
#endif
#if defined(_M_PPC) || defined(__CELLOS_LV2__)
#define QH_VMX 1
#endif
#endif

/**
define anything not defined on this platform to 0
*/
#ifndef QH_VC
#define QH_VC 0
#endif
#ifndef QH_CLANG
#define QH_CLANG 0
#endif
#ifndef QH_GCC
#define QH_GCC 0
#endif
#ifndef QH_XBOXONE
#define QH_XBOXONE 0
#endif
#ifndef QH_XBOX_SERIES_X
#define QH_XBOX_SERIES_X 0
#endif
#ifndef QH_WIN64
#define QH_WIN64 0
#endif
#ifndef QH_WIN32
#define QH_WIN32 0
#endif
#ifndef QH_ANDROID
#define QH_ANDROID 0
#endif
#ifndef QH_LINUX
#define QH_LINUX 0
#endif
#ifndef QH_IOS
#define QH_IOS 0
#endif
#ifndef QH_OSX
#define QH_OSX 0
#endif
#ifndef QH_PS4
#define QH_PS4 0
#endif
#ifndef QH_SWITCH
#define QH_SWITCH 0
#endif
#ifndef QH_UWP
#define QH_UWP 0
#endif
#ifndef QH_X64
#define QH_X64 0
#endif
#ifndef QH_X86
#define QH_X86 0
#endif
#ifndef QH_A64
#define QH_A64 0
#endif
#ifndef QH_ARM
#define QH_ARM 0
#endif
#ifndef QH_PPC
#define QH_PPC 0
#endif
#ifndef QH_SSE2
#define QH_SSE2 0
#endif
#ifndef QH_NEON
#define QH_NEON 0
#endif
#ifndef QH_VMX
#define QH_VMX 0
#endif

/*
define anything not defined through the command line to 0
*/
#ifndef QH_DEBUG
#define QH_DEBUG 0
#endif
#ifndef QH_CHECKED
#define QH_CHECKED 0
#endif
#ifndef QH_PROFILE
#define QH_PROFILE 0
#endif
#ifndef QH_DEBUG_CRT
#define QH_DEBUG_CRT 0
#endif
#ifndef QH_NVTX
#define QH_NVTX 0
#endif
#ifndef QH_DOXYGEN
#define QH_DOXYGEN 0
#endif

/**
family shortcuts
*/
// compiler
#define QH_GCC_FAMILY (QH_CLANG || QH_GCC)
// os
#define QH_WINDOWS_FAMILY (QH_WIN32 || QH_WIN64 || QH_UWP)
#define QH_MICROSOFT_FAMILY (QH_XBOXONE || QH_WINDOWS_FAMILY || QH_XBOX_SERIES_X)
#define QH_LINUX_FAMILY (QH_LINUX || QH_ANDROID)
#define QH_APPLE_FAMILY (QH_IOS || QH_OSX)                  // equivalent to #if __APPLE__
#define QH_UNIX_FAMILY (QH_LINUX_FAMILY || QH_APPLE_FAMILY) // shortcut for unix/posix platforms
#if defined(__EMSCRIPTEN__)
#define QH_EMSCRIPTEN 1
#else
#define QH_EMSCRIPTEN 0
#endif
// architecture
#define QH_INTEL_FAMILY (QH_X64 || QH_X86)
#define QH_ARM_FAMILY (QH_ARM || QH_A64)
#define QH_P64_FAMILY (QH_X64 || QH_A64) // shortcut for 64-bit architectures

/**
C++ standard library defines
*/
#if defined(_LIBCPP_VERSION) || QH_WIN64 || QH_WIN32 || QH_PS4 || QH_XBOXONE || QH_UWP || QH_EMSCRIPTEN || QH_XBOX_SERIES_X
#define QH_LIBCPP 1
#else
#define QH_LIBCPP 0
#endif

// legacy define for PhysX
#define QH_WINDOWS (QH_WINDOWS_FAMILY && !QH_ARM_FAMILY)

/**
Assert macro
*/
#ifndef QH_ENABLE_ASSERTS
#if QH_DEBUG && !defined(__CUDACC__)
#define QH_ENABLE_ASSERTS 1
#else
#define QH_ENABLE_ASSERTS 0
#endif
#endif

/**
DLL export macros
*/
#ifndef QH_C_EXPORT
#if QH_WINDOWS_FAMILY || QH_LINUX
#define QH_C_EXPORT extern "C"
#else
#define QH_C_EXPORT
#endif
#endif

#if QH_UNIX_FAMILY&& __GNUC__ >= 4
#define QH_UNIX_EXPORT __attribute__((visibility("default")))
#else
#define QH_UNIX_EXPORT
#endif

#if (QH_WINDOWS_FAMILY || QH_XBOXONE || QH_PS4 || QH_XBOX_SERIES_X)
#define QH_DLL_EXPORT __declspec(dllexport)
#define QH_DLL_IMPORT __declspec(dllimport)
#else
#define QH_DLL_EXPORT QH_UNIX_EXPORT
#define QH_DLL_IMPORT
#endif

/**
Calling convention
*/
#ifndef QH_CALL_CONV
#if QH_MICROSOFT_FAMILY
#define QH_CALL_CONV __cdecl
#else
#define QH_CALL_CONV
#endif
#endif

/**
Pack macros - disabled on SPU because they are not supported
*/
#if QH_VC
#define QH_PUSH_PACK_DEFAULT __pragma(pack(push, 8))
#define QH_POP_PACK __pragma(pack(pop))
#elif QH_GCC_FAMILY
#define QH_PUSH_PACK_DEFAULT _Pragma("pack(push, 8)")
#define QH_POP_PACK _Pragma("pack(pop)")
#else
#define QH_PUSH_PACK_DEFAULT
#define QH_POP_PACK
#endif

/**
Inline macro
*/
#define QH_INLINE inline
#if QH_MICROSOFT_FAMILY
#pragma inline_depth(255)
#endif

/**
Force inline macro
*/
#if QH_VC
#define QH_FORCE_INLINE __forceinline
#elif QH_LINUX // Workaround; Fedora Core 3 do not agree with force inline and QhcPool
#define QH_FORCE_INLINE inline
#elif QH_GCC_FAMILY
#define QH_FORCE_INLINE inline __attribute__((always_inline))
#else
#define QH_FORCE_INLINE inline
#endif

/**
Noinline macro
*/
#if QH_MICROSOFT_FAMILY
#define QH_NOINLINE __declspec(noinline)
#elif QH_GCC_FAMILY
#define QH_NOINLINE __attribute__((noinline))
#else
#define QH_NOINLINE
#endif

/**
Restrict macro
*/
#if defined(__CUDACC__)
#define QH_RESTRICT __restrict__
#else
#define QH_RESTRICT __restrict
#endif

/**
Noalias macro
*/
#if QH_MICROSOFT_FAMILY
#define QH_NOALIAS __declspec(noalias)
#else
#define QH_NOALIAS
#endif

/**
Alignment macros

QH_ALIGN_PREFIX and QH_ALIGN_SUFFIX can be used for type alignment instead of aligning individual variables as follows:
QH_ALIGN_PREFIX(16)
struct A {
...
} QH_ALIGN_SUFFIX(16);
This declaration style is parsed correctly by Visual Assist.

*/
#ifndef QH_ALIGN
#if QH_MICROSOFT_FAMILY
#define QH_ALIGN(alignment, decl) __declspec(align(alignment)) decl
#define QH_ALIGN_PREFIX(alignment) __declspec(align(alignment))
#define QH_ALIGN_SUFFIX(alignment)
#elif QH_GCC_FAMILY
#define QH_ALIGN(alignment, decl) decl __attribute__((aligned(alignment)))
#define QH_ALIGN_PREFIX(alignment)
#define QH_ALIGN_SUFFIX(alignment) __attribute__((aligned(alignment)))
#elif defined __CUDACC__
#define QH_ALIGN(alignment, decl) __align__(alignment) decl
#define QH_ALIGN_PREFIX(alignment)
#define QH_ALIGN_SUFFIX(alignment) __align__(alignment))
#else
#define QH_ALIGN(alignment, decl)
#define QH_ALIGN_PREFIX(alignment)
#define QH_ALIGN_SUFFIX(alignment)
#endif
#endif

/**
Deprecated macro
- To deprecate a function: Place QH_DEPRECATED at the start of the function header (leftmost word).
- To deprecate a 'typedef', a 'struct' or a 'class': Place QH_DEPRECATED directly after the keywords ('typdef',
'struct', 'class').

Use these macro definitions to create warnings for deprecated functions
\#define QH_DEPRECATED __declspec(deprecated) // Microsoft
\#define QH_DEPRECATED __attribute__((deprecated())) // GCC
*/
#define QH_DEPRECATED

/**
General defines
*/

// static assert
#if(defined(__GNUC__) && (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 7))) || (QH_PS4) || (QH_APPLE_FAMILY) || (QH_SWITCH) || (QH_CLANG && QH_ARM) || (QH_CLANG && QH_A64)
#define QH_COMPILE_TIME_ASSERT(exp) typedef char QH_CONCAT(QhCompileTimeAssert_Dummy, __COUNTER__)[(exp) ? 1 : -1] __attribute__((unused))
#else
#define QH_COMPILE_TIME_ASSERT(exp) typedef char QhCompileTimeAssert_Dummy[(exp) ? 1 : -1]
#endif

#if QH_GCC_FAMILY
#define QH_OFFSET_OF(X, Y) __builtin_offsetof(X, Y)
#else
#define QH_OFFSET_OF(X, Y) offsetof(X, Y)
#endif

#define QH_OFFSETOF_BASE 0x100 // casting the null ptr takes a special-case code path, which we don't want
#define QH_OFFSET_OF_RT(Class, Member)                                                                                 \
	(reinterpret_cast<size_t>(&reinterpret_cast<Class*>(QH_OFFSETOF_BASE)->Member) - size_t(QH_OFFSETOF_BASE))

// check that exactly one of NDEBUG and _DEBUG is defined
#if !defined(NDEBUG) ^ defined(_DEBUG)
#error Exactly one of NDEBUG and _DEBUG needs to be defined!
#endif

// make sure QH_CHECKED is defined in all _DEBUG configurations as well
#if !QH_CHECKED && QH_DEBUG
#error QH_CHECKED must be defined when QH_DEBUG is defined
#endif

// avoid unreferenced parameter warning
// preferred solution: omit the parameter's name from the declaration
template <class T>
 QH_INLINE void QH_UNUSED(T const&)
{
}

// Ensure that the application hasn't tweaked the pack value to less than 8, which would break
// matching between the API headers and the binaries
// This assert works on win32/win64, but may need further specialization on other platforms.
// Some GCC compilers need the compiler flag -malign-double to be set.
// Apparently the apple-clang-llvm compiler doesn't support malign-double.
#if QH_PS4 || QH_APPLE_FAMILY || (QH_CLANG && !QH_ARM)
struct QhPackValidation
{
	char _;
	long a;
};
#elif QH_ANDROID || (QH_CLANG && QH_ARM)
struct QhPackValidation
{
	char _;
	double a;
};
#else
struct QhPackValidation
{
	char _;
	long long a;
};
#endif
// clang (as of version 3.9) cannot align doubles on 8 byte boundary  when compiling for Intel 32 bit target
#if !QH_APPLE_FAMILY && !QH_EMSCRIPTEN && !(QH_CLANG && QH_X86)
QH_COMPILE_TIME_ASSERT(QH_OFFSET_OF(QhPackValidation, a) == 8);
#endif

// use in a cpp file to suppress LNK4221
#if QH_VC
#define QH_DUMMY_SYMBOL                                                                                                \
	namespace                                                                                                          \
	{                                                                                                                  \
	char QhDummySymbol;                                                                                                \
	}
#else
#define QH_DUMMY_SYMBOL
#endif

#if QH_GCC_FAMILY
#define QH_WEAK_SYMBOL __attribute__((weak)) // this is to support SIMD constant merging in template specialization
#else
#define QH_WEAK_SYMBOL
#endif

// Macro for avoiding default assignment and copy, because doing this by inheritance can increase class size on some
// platforms.
#define QH_NOCOPY(Class)                                                                                               \
	\
protected:                                                                                                             \
	Class(const Class&);                                                                                               \
	Class& operator=(const Class&);

#ifndef DISABLE_CUDA_PHYSX
//CUDA is currently supported only on windows 
#define QH_SUPPORT_GPU_PHYSX ((QH_WINDOWS_FAMILY) || (QH_LINUX && QH_X64))
#else
#define QH_SUPPORT_GPU_PHYSX 0
#endif

#define QH_SUPPORT_COMPUTE_PHYSX 0

#ifndef QH_SUPPORT_EXTERN_TEMPLATE
#define QH_SUPPORT_EXTERN_TEMPLATE ((!QH_ANDROID) && (QH_VC != 11))
#else
#define QH_SUPPORT_EXTERN_TEMPLATE 0
#endif

/** @} */
#endif
