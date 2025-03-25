## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions
## are met:
##  * Redistributions of source code must retain the above copyright
##    notice, this list of conditions and the following disclaimer.
##  * Redistributions in binary form must reproduce the above copyright
##    notice, this list of conditions and the following disclaimer in the
##    documentation and/or other materials provided with the distribution.
##  * Neither the name of NVIDIA CORPORATION nor the names of its
##    contributors may be used to endorse or promote products derived
##    from this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
## EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
## PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
## CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
## EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
## PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
## PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
## OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
## (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
## OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##
## Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.

IF(PX_OUTPUT_ARCH STREQUAL "arm")
    SET(OSX_BITNESS "-arch arm64")
    SET(CMAKE_OSX_ARCHITECTURES "arm64")
    SET(ARCH_SIMD_FLAG "") # No flags needed to enable NEON
ELSE()
    SET(OSX_BITNESS "-arch x86_64")
    SET(CMAKE_OSX_ARCHITECTURES "x86_64")
    SET(ARCH_SIMD_FLAG "-msse2")
ENDIF()

SET(PHYSX_CXX_FLAGS "${OSX_BITNESS} ${ARCH_SIMD_FLAG} -std=c++11 -fno-rtti -fno-exceptions -ffunction-sections -fdata-sections \
    -ferror-limit=0 -Wall -Wextra -Werror -fstrict-aliasing -Wstrict-aliasing=2 -Weverything\
    -Wno-alloca \
    -Wno-atomic-implicit-seq-cst \
    -Wno-c++98-compat-pedantic \
    -Wno-cast-align \
    -Wno-conversion \
    -Wno-covered-switch-default \
    -Wno-deprecated \
    -Wno-documentation-deprecated-sync \
    -Wno-documentation-unknown-command \
    -Wno-exit-time-destructors \
    -Wno-float-equal \
    -Wno-global-constructors \
    -Wno-cast-qual \
    -Wno-unknown-warning-option \
    -Wno-missing-noreturn \
    -Wno-missing-prototypes \
    -Wno-missing-variable-declarations \
    -Wno-newline-eof \
    -Wno-non-virtual-dtor \
    -Wno-old-style-cast \
    -Wno-padded \
    -Wno-suggest-destructor-override \
    -Wno-suggest-override \
    -Wno-switch-enum \
    -Wno-undef \
    -Wno-undefined-reinterpret-cast \
    -Wno-unreachable-code \
    -Wno-shift-sign-overflow \
    -Wno-unused-macros \
    -Wno-unused-member-function \
    -Wno-unused-private-field \
    -Wno-used-but-marked-unused \
    -Wno-weak-template-vtables \
    -Wno-weak-vtables \
    -Wno-invalid-noreturn \
    -Wno-return-type-c-linkage \
    -Wno-reserved-id-macro \
    -Wno-unused-local-typedef \
    -Wno-invalid-offsetof \
    -Wno-zero-as-null-pointer-constant \
    -Wno-unused-but-set-variable \
    -Wno-shadow-field \
    -Wno-unused-variable \
    -Wno-double-promotion \
    -Wno-extra-semi \
    -Wno-conditional-uninitialized \
    -Wno-unused-parameter \
    -Wno-bitwise-instead-of-logical \
    -Wno-undefined-func-template \
    -Wno-inconsistent-missing-override \
    -Wno-inconsistent-missing-destructor-override \
    -Wno-unused-function \
" CACHE INTERNAL "PhysX CXX")

SET(CMAKE_SHARED_LINKER_FLAGS_CHECKED "")
SET(CMAKE_SHARED_LINKER_FLAGS_PROFILE "")
SET(CMAKE_SHARED_LINKER_FLAGS "")

# Build debug info for all configurations
SET(PHYSX_CXX_FLAGS_DEBUG "-O0 -g3 -gdwarf-2" CACHE INTERNAL "PhysX Debug CXX Flags")
SET(PHYSX_CXX_FLAGS_CHECKED "-O3 -g3 -gdwarf-2" CACHE INTERNAL "PhysX Checked CXX Flags")
SET(PHYSX_CXX_FLAGS_PROFILE "-O3" CACHE INTERNAL "PhysX Profile CXX Flags")
SET(PHYSX_CXX_FLAGS_RELEASE "-O3" CACHE INTERNAL "PhysX Release CXX Flags")

# These flags are local to the directory the CMakeLists.txt is in, so don't get carried over to OTHER CMakeLists.txt (thus the CACHE variables above)
SET(CMAKE_CXX_FLAGS ${PHYSX_CXX_FLAGS})

SET(CMAKE_CXX_FLAGS_DEBUG   ${PHYSX_CXX_FLAGS_DEBUG})
SET(CMAKE_CXX_FLAGS_CHECKED ${PHYSX_CXX_FLAGS_CHECKED})
SET(CMAKE_CXX_FLAGS_PROFILE ${PHYSX_CXX_FLAGS_PROFILE})
SET(CMAKE_CXX_FLAGS_RELEASE ${PHYSX_CXX_FLAGS_RELEASE})

IF(PUBLIC_RELEASE)
	SET(PUBLIC_RELEASE_FLAG "PX_PUBLIC_RELEASE=1")
ELSE()
	SET(PUBLIC_RELEASE_FLAG "PX_PUBLIC_RELEASE=0")
ENDIF()

# Controls PX_NVTX for all projects
SET(NVTX_FLAG "PX_NVTX=0")

# Disable OmniPVD for all projects on Mac
SET(PX_SUPPORT_OMNI_PVD_FLAG "PX_SUPPORT_OMNI_PVD=0")

# Disable cuda for all projects
SET(PHYSX_MAC_COMPILE_DEFS "DISABLE_CUDA_PHYSX;${PHYSX_AUTOBUILD};${PUBLIC_RELEASE_FLAG};${FEATURES_UNDER_CONSTRUCTION_FLAG}" CACHE INTERNAL "Base PhysX preprocessor definitions")
SET(PHYSX_MAC_DEBUG_COMPILE_DEFS "_DEBUG;PX_DEBUG=1;PX_CHECKED=1;${NVTX_FLAG};PX_SUPPORT_PVD=1;${PX_SUPPORT_OMNI_PVD_FLAG}" CACHE INTERNAL "Debug PhysX preprocessor definitions")
SET(PHYSX_MAC_CHECKED_COMPILE_DEFS "NDEBUG;PX_CHECKED=1;${NVTX_FLAG};PX_SUPPORT_PVD=1;${PX_SUPPORT_OMNI_PVD_FLAG}" CACHE INTERNAL "Checked PhysX preprocessor definitions")
SET(PHYSX_MAC_PROFILE_COMPILE_DEFS "NDEBUG;PX_PROFILE=1;${NVTX_FLAG};PX_SUPPORT_PVD=1;${PX_SUPPORT_OMNI_PVD_FLAG}" CACHE INTERNAL "Profile PhysX preprocessor definitions")
SET(PHYSX_MAC_RELEASE_COMPILE_DEFS "NDEBUG;PX_SUPPORT_PVD=0;PX_SUPPORT_OMNI_PVD=0" CACHE INTERNAL "Release PhysX preprocessor definitions")


# Include all of the projects
INCLUDE(PhysXFoundation.cmake)
INCLUDE(LowLevel.cmake)
INCLUDE(LowLevelAABB.cmake)
INCLUDE(LowLevelDynamics.cmake)
INCLUDE(PhysX.cmake)
INCLUDE(PhysXCharacterKinematic.cmake)
INCLUDE(PhysXCommon.cmake)
INCLUDE(PhysXCooking.cmake)
INCLUDE(PhysXExtensions.cmake)
INCLUDE(PhysXVehicle.cmake)
INCLUDE(SceneQuery.cmake)
INCLUDE(SimulationController.cmake)
INCLUDE(PhysXPvdSDK.cmake)
INCLUDE(PhysXTask.cmake)
INCLUDE(jni-mac/PhysXJniBindings.cmake)

# Set folder PhysX SDK to all common SDK source projects
SET_PROPERTY(TARGET PhysX PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXCharacterKinematic PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXCommon PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXCooking PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXExtensions PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXVehicle2 PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET LowLevel PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET LowLevelAABB PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET LowLevelDynamics PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET SceneQuery PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET SimulationController PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXPvdSDK PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXTask PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXFoundation PROPERTY FOLDER "PhysX SDK")
