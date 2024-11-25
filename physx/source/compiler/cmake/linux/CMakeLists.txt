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
## Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.

STRING(TOLOWER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_LOWERCASE)

#TODO: Fix warnings
SET(CLANG_WARNINGS "-ferror-limit=0 -Wall -Wextra -Werror -Weverything\
    -Wno-unused-but-set-variable \
    -Wno-switch-default \
    -Wno-cast-qual \
    -Wno-invalid-offsetof \
    -Wno-unsafe-buffer-usage \
    -Wno-alloca \
    -Wno-atomic-implicit-seq-cst \
    -Wno-c++98-compat-pedantic \
    -Wno-c++98-compat \
    -Wno-cast-align \
    -Wno-conversion \
    -Wno-covered-switch-default \
    -Wno-deprecated \
    -Wno-documentation-deprecated-sync \
    -Wno-documentation-unknown-command \
    -Wno-exit-time-destructors \
    -Wno-extra-semi-stmt \
    -Wno-float-equal \
    -Wno-format-nonliteral \
    -Wno-global-constructors \
    -Wno-implicit-fallthrough \
    -Wno-inconsistent-missing-destructor-override \
    -Wno-inconsistent-missing-override \
    -Wno-missing-noreturn \
    -Wno-missing-prototypes \
    -Wno-missing-variable-declarations \
    -Wno-newline-eof \
    -Wno-non-virtual-dtor \
    -Wno-old-style-cast \
    -Wno-padded \
    -Wno-reserved-id-macro \
    -Wno-suggest-destructor-override \
    -Wno-suggest-override \
    -Wno-switch-enum \
    -Wno-undef \
    -Wno-undefined-reinterpret-cast \
    -Wno-unknown-warning-option \
    -Wno-unreachable-code \
    -Wno-unused-function \
    -Wno-unused-macros \
    -Wno-unused-member-function \
    -Wno-unused-private-field \
    -Wno-used-but-marked-unused \
    -Wno-weak-template-vtables \
    -Wno-weak-vtables \
    -Wno-zero-as-null-pointer-constant \
    -Wno-reserved-identifier \
    -Wno-undefined-func-template \
")

SET(GCC_WARNINGS "-Wall -Werror \
	-Wno-address \
	-Wno-aligned-new \
	-Wno-array-bounds \
	-Wno-class-memaccess \
	-Wno-conversion-null \
	-Wno-format \
	-Wno-format-overflow \
	-Wno-invalid-offsetof \
	-Wno-misleading-indentation \
	-Wno-mismatched-new-delete \
	-Wno-nonnull \
	-Wno-nonnull-compare \
	-Wno-pragmas \
	-Wno-restrict \
	-Wno-stringop-overflow \
	-Wno-stringop-overread \
	-Wno-subobject-linkage \
	-Wno-template-id-cdtor \
	-Wno-uninitialized \
	-Wno-unused-but-set-variable \
	-Wno-unused-function \
	-Wno-unused-result \
	-Wno-unknown-pragmas \
	-Wno-use-after-free \
")

IF(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
	# gw: these optimizations are disabled for now. fp-contract causes floating point inaccuracy and deviation in behavior,
	# tree-vrp and delete-null-pointer-checks cause known bugs on some versions of gcc
	IF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
		SET(AARCH64_FLAGS "-ffp-contract=off -fno-delete-null-pointer-checks -faligned-new")
	ELSE()
		SET(AARCH64_FLAGS "-ffp-contract=off")
	ENDIF()
ELSE()
	SET(AARCH64_FLAGS "")
ENDIF()

SET(COMMON_CXX_FLAGS "-std=c++11 -fno-rtti -fno-exceptions -ffunction-sections -fdata-sections -fvisibility=hidden ${AARCH64_FLAGS}")

IF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	IF ("${CMAKE_CXX_COMPILER_VERSION}" VERSION_LESS "10.0.0")
		SET(PHYSX_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_CXX_FLAGS} -fstrict-aliasing ${CLANG_WARNINGS}" CACHE INTERNAL "PhysX CXX")
	ELSE()
		SET(PHYSX_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_CXX_FLAGS} -fstrict-aliasing -ffp-exception-behavior=maytrap ${CLANG_WARNINGS}" CACHE INTERNAL "PhysX CXX")
	ENDIF()
ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	SET(PHYSX_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_CXX_FLAGS} -fno-strict-aliasing ${GCC_WARNINGS}" CACHE INTERNAL "PhysX CXX")
ENDIF()

IF (PX_USE_MOLD_LINKER)
	SET (PHYSX_CXX_FLAGS "${PHYSX_CXX_FLAGS} -Wno-unused-command-line-argument -fuse-ld=mold " CACHE INTERNAL "PhysX CXX")
ENDIF()

# Build debug info for all configurations
SET(PHYSX_CXX_FLAGS_DEBUG   "-O0 -g3 -gdwarf-2" CACHE INTERNAL "PhysX Debug CXX Flags")
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
IF(${CMAKE_CROSSCOMPILING} AND CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
	SET(PX_SUPPORT_OMNI_PVD_FLAG "PX_SUPPORT_OMNI_PVD=0")
ELSE()
	SET(PX_SUPPORT_OMNI_PVD_FLAG "PX_SUPPORT_OMNI_PVD=1")
ENDIF()
SET(PHYSX_LINUX_COMPILE_DEFS "${PHYSX_AUTOBUILD};${PUBLIC_RELEASE_FLAG}" CACHE INTERNAL "Base PhysX preprocessor definitions")
SET(PHYSX_LINUX_DEBUG_COMPILE_DEFS   "PX_DEBUG=1;PX_CHECKED=1;${NVTX_FLAG};PX_SUPPORT_PVD=1;${PX_SUPPORT_OMNI_PVD_FLAG}"  CACHE INTERNAL "Debug PhysX preprocessor definitions")
SET(PHYSX_LINUX_CHECKED_COMPILE_DEFS "NDEBUG;PX_CHECKED=1;${NVTX_FLAG};PX_SUPPORT_PVD=1;${PX_SUPPORT_OMNI_PVD_FLAG}" CACHE INTERNAL "Checked PhysX preprocessor definitions")
SET(PHYSX_LINUX_PROFILE_COMPILE_DEFS "NDEBUG;PX_PROFILE=1;${NVTX_FLAG};PX_SUPPORT_PVD=1;${PX_SUPPORT_OMNI_PVD_FLAG}"  CACHE INTERNAL "Profile PhysX preprocessor definitions")
SET(PHYSX_LINUX_RELEASE_COMPILE_DEFS "NDEBUG;PX_SUPPORT_PVD=0;PX_SUPPORT_OMNI_PVD=0" CACHE INTERNAL "Release PhysX preprocessor definitions")

IF(PUBLIC_RELEASE)
	IF(NOT PHYSX_PHYSXGPU_PATH)
		SET(PHYSX_PHYSXGPU_PATH "$ENV{PM_PhysXGpu_PATH}/bin" CACHE INTERNAL "PhysXGpu copy path")
	ENDIF()
	IF(NOT GPU_LIB_COPIED)
		IF(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
			SET(GPU_LIB_COPIED 1 CACHE INTERNAL "PhysX GPU so files copied")
			FILE(COPY ${PHYSX_PHYSXGPU_PATH}/linux.aarch64/checked/libPhysXGpu_64.so DESTINATION ${PX_EXE_OUTPUT_DIRECTORY_DEBUG})
			FILE(COPY ${PHYSX_PHYSXGPU_PATH}/linux.aarch64/checked/libPhysXGpu_64.so DESTINATION ${PX_EXE_OUTPUT_DIRECTORY_CHECKED})
			FILE(COPY ${PHYSX_PHYSXGPU_PATH}/linux.aarch64/profile/libPhysXGpu_64.so DESTINATION ${PX_EXE_OUTPUT_DIRECTORY_PROFILE})
			FILE(COPY ${PHYSX_PHYSXGPU_PATH}/linux.aarch64/release/libPhysXGpu_64.so DESTINATION ${PX_EXE_OUTPUT_DIRECTORY_RELEASE})
			INSTALL(PROGRAMS ${PHYSX_PHYSXGPU_PATH}/linux.aarch64/$<$<CONFIG:debug>:checked>$<$<CONFIG:release>:release>$<$<CONFIG:checked>:checked>$<$<CONFIG:profile>:profile>/libPhysXGpu_64.so
				DESTINATION $<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile> OPTIONAL)
		ELSEIF(UNIX)
			SET(GPU_LIB_COPIED 1 CACHE INTERNAL "PhysX GPU so files copied")
			FILE(COPY ${PHYSX_PHYSXGPU_PATH}/linux.x86_64/checked/libPhysXGpu_64.so DESTINATION ${PX_EXE_OUTPUT_DIRECTORY_DEBUG})
			FILE(COPY ${PHYSX_PHYSXGPU_PATH}/linux.x86_64/checked/libPhysXGpu_64.so DESTINATION ${PX_EXE_OUTPUT_DIRECTORY_CHECKED})
			FILE(COPY ${PHYSX_PHYSXGPU_PATH}/linux.x86_64/profile/libPhysXGpu_64.so DESTINATION ${PX_EXE_OUTPUT_DIRECTORY_PROFILE})
			FILE(COPY ${PHYSX_PHYSXGPU_PATH}/linux.x86_64/release/libPhysXGpu_64.so DESTINATION ${PX_EXE_OUTPUT_DIRECTORY_RELEASE})
			INSTALL(PROGRAMS ${PHYSX_PHYSXGPU_PATH}/linux.x86_64/$<$<CONFIG:debug>:checked>$<$<CONFIG:release>:release>$<$<CONFIG:checked>:checked>$<$<CONFIG:profile>:profile>/libPhysXGpu_64.so
				DESTINATION $<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile> OPTIONAL)
		ENDIF()
	ENDIF()
ENDIF()

# Include all of the projects
IF(PX_GENERATE_GPU_PROJECTS_ONLY)
	INCLUDE(PhysXCommon.cmake)
	INCLUDE(PhysXFoundation.cmake)
	INCLUDE(LowLevelAABB.cmake)

	SET_PROPERTY(TARGET PhysXCommon PROPERTY FOLDER "PhysX SDK")
	SET_PROPERTY(TARGET PhysXFoundation PROPERTY FOLDER "PhysX SDK")
ELSE()
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
	INCLUDE(PhysXVehicle2.cmake)
	INCLUDE(SceneQuery.cmake)
	INCLUDE(SimulationController.cmake)
	INCLUDE(PhysXPvdSDK.cmake)
	INCLUDE(PhysXTask.cmake)

	# Set folder PhysX SDK to all common SDK source projects
	SET_PROPERTY(TARGET PhysX PROPERTY FOLDER "PhysX SDK")
	SET_PROPERTY(TARGET PhysXCharacterKinematic PROPERTY FOLDER "PhysX SDK")
	SET_PROPERTY(TARGET PhysXCommon PROPERTY FOLDER "PhysX SDK")
	SET_PROPERTY(TARGET PhysXCooking PROPERTY FOLDER "PhysX SDK")
	SET_PROPERTY(TARGET PhysXExtensions PROPERTY FOLDER "PhysX SDK")
	SET_PROPERTY(TARGET PhysXVehicle PROPERTY FOLDER "PhysX SDK")
	SET_PROPERTY(TARGET PhysXVehicle2 PROPERTY FOLDER "PhysX SDK")
	SET_PROPERTY(TARGET LowLevel PROPERTY FOLDER "PhysX SDK")
	SET_PROPERTY(TARGET LowLevelAABB PROPERTY FOLDER "PhysX SDK")
	SET_PROPERTY(TARGET LowLevelDynamics PROPERTY FOLDER "PhysX SDK")
	SET_PROPERTY(TARGET SceneQuery PROPERTY FOLDER "PhysX SDK")
	SET_PROPERTY(TARGET SimulationController PROPERTY FOLDER "PhysX SDK")
	SET_PROPERTY(TARGET PhysXPvdSDK PROPERTY FOLDER "PhysX SDK")
	SET_PROPERTY(TARGET PhysXTask PROPERTY FOLDER "PhysX SDK")
	SET_PROPERTY(TARGET PhysXFoundation PROPERTY FOLDER "PhysX SDK")


	SET(PHYSXDISTRO_LIBS PhysXFoundation PhysX PhysXCharacterKinematic PhysXPvdSDK PhysXCommon PhysXCooking PhysXExtensions PhysXVehicle PhysXVehicle2)


	INSTALL(
		TARGETS ${PHYSXDISTRO_LIBS}
		EXPORT PhysXSDK
		DESTINATION $<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile>
		)

ENDIF()
