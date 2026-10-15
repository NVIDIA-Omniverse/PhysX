// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-NATIVE-ABI-001
// @covers AC-1 AC-5
#pragma once
// Optional C++17 build contract. Include only for native PhysX SDK access.
#include <ovphysx_companion/native_build.h>
#include <PxPhysicsAPI.h>
#include <omni/physx/NativeParameters.h>
#include <cstdint>
#include <cstring>
#define OVPHYSX_COMPANION_STRINGIFY_(x) #x
#define OVPHYSX_COMPANION_STRINGIFY(x) OVPHYSX_COMPANION_STRINGIFY_(x)
namespace ovphysx::companion::native
{
using omni::physx::native::ParameterValue;
struct Compatibility
{
    uint32_t size = sizeof(Compatibility);
    uint32_t version = 1;
    char physxBuild[65] = OVPHYSX_COMPANION_PHYSX_BUILD;
#if defined(_MSC_VER)
    char compiler[128] = "msvc:" OVPHYSX_COMPANION_STRINGIFY(_MSC_VER) ":iter:"
        OVPHYSX_COMPANION_STRINGIFY(_ITERATOR_DEBUG_LEVEL) ":dll:" OVPHYSX_COMPANION_STRINGIFY(_DLL);
#elif defined(__clang__)
    char compiler[128] = "clang:" __clang_version__ ":abi:" OVPHYSX_COMPANION_STRINGIFY(__GXX_ABI_VERSION)
        ":stdlib:" OVPHYSX_COMPANION_STRINGIFY(_GLIBCXX_USE_CXX11_ABI);
#else
    char compiler[128] = "gcc:" __VERSION__ ":abi:" OVPHYSX_COMPANION_STRINGIFY(__GXX_ABI_VERSION)
        ":stdlib:" OVPHYSX_COMPANION_STRINGIFY(_GLIBCXX_USE_CXX11_ABI);
#endif
    uint32_t sdkVersion = PX_PHYSICS_VERSION;
    uint32_t pointerSize = sizeof(void*);
    uint32_t transformSize = sizeof(physx::PxTransform);
    uint32_t constraintRowSize = sizeof(physx::Px1DConstraint);
    uint32_t parameterSize = sizeof(ParameterValue);
#if defined(_DEBUG)
    uint32_t debug = 1;
#else
    uint32_t debug = 0;
#endif
};

inline bool compatibleBuild(const Compatibility& c) noexcept
{
    const Compatibility expected;
    return c.size==sizeof(c) && c.version==1 &&
        !std::memcmp(c.physxBuild,expected.physxBuild,sizeof(c.physxBuild)) &&
        !std::memcmp(c.compiler,expected.compiler,sizeof(c.compiler)) &&
        c.sdkVersion==expected.sdkVersion && c.pointerSize==expected.pointerSize &&
        c.transformSize==expected.transformSize && c.constraintRowSize==expected.constraintRowSize &&
        c.parameterSize==expected.parameterSize && c.debug==expected.debug;
}

}
#undef OVPHYSX_COMPANION_STRINGIFY
#undef OVPHYSX_COMPANION_STRINGIFY_
