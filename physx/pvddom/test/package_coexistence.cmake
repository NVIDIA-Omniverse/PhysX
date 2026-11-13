## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.21)
function(checked)
    execute_process(COMMAND ${ARGN} RESULT_VARIABLE code OUTPUT_VARIABLE output ERROR_VARIABLE diagnostic)
    if(NOT "${code}" STREQUAL "0")
        message(FATAL_ERROR "${ARGN}\n${output}\n${diagnostic}")
    endif()
endfunction()
set(standalone "${BUILD}/standalone-package-${CONFIG}")
set(runtime "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
if(STATIC_CRT)
    set(runtime "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()
set(runtimeRoot "${SOURCE}/../pvdruntime")
file(STRINGS "${SOURCE}/../include/foundation/PxPhysicsVersion.h" versionLines REGEX "#define PX_PHYSICS_VERSION_(MAJOR|MINOR|BUGFIX)")
foreach(line IN LISTS versionLines)
    if(line MATCHES "#define PX_PHYSICS_VERSION_(MAJOR|MINOR|BUGFIX)[ \t]+([0-9]+)")
        set(version_${CMAKE_MATCH_1} "${CMAKE_MATCH_2}")
    endif()
endforeach()
set(sdkVersion "${version_MAJOR}.${version_MINOR}.${version_BUGFIX}")
if(WIN32)
    file(TO_NATIVE_PATH "${runtimeRoot}" runtimeRoot)
endif()
checked("${CMAKE_COMMAND}" -S "${SOURCE}" -B "${standalone}" -G "${GENERATOR}"
    "-DPVDRUNTIME_ROOT=${runtimeRoot}" -DPVDDOM_BUILD_INSPECTION=ON -DPVDDOM_BUILD_TESTS=OFF -DPVDDOM_BUILD_TOOLS=OFF
    -DCMAKE_BUILD_TYPE=${CONFIG} -DCMAKE_CONFIGURATION_TYPES=${CONFIG} -DNV_USE_STATIC_WINCRT=${STATIC_CRT}
    "-DCMAKE_CXX_FLAGS=${CXX_FLAGS}" "-DCMAKE_EXE_LINKER_FLAGS=${LINKER_FLAGS}")
checked("${CMAKE_COMMAND}" --build "${standalone}" --config "${CONFIG}" --parallel 4)
checked("${CMAKE_COMMAND}" --install "${standalone}" --config "${CONFIG}" --prefix "${standalone}/installed")
foreach(installedFile pvdruntime/include/OmniPvdLibraryFunctions.h lib/cmake/PvdDom/PvdDomConfig.cmake lib/cmake/PvdDom/PvdDomConfigVersion.cmake share/physx/skills)
    if(NOT EXISTS "${standalone}/installed/${installedFile}")
        message(FATAL_ERROR "Standalone package missing ${installedFile}")
    endif()
endforeach()
foreach(order sdk-first dom-first)
    set(consumer "${BUILD}/consumer-${order}-${CONFIG}")
    checked("${CMAKE_COMMAND}" -S "${SOURCE}/test/consumer" -B "${consumer}" -G "${GENERATOR}"
        -DCMAKE_BUILD_TYPE=${CONFIG} -DCMAKE_CONFIGURATION_TYPES=${CONFIG} -DCMAKE_PREFIX_PATH=${standalone}/installed
        -DPhysX_DIR=${BUILD}/sdk-package -DPACKAGE_ORDER=${order}
        -DREQUESTED_DOM_VERSION=${sdkVersion}
        "-DCMAKE_MSVC_RUNTIME_LIBRARY=${runtime}"
        "-DCMAKE_CXX_FLAGS=${CXX_FLAGS}" "-DCMAKE_EXE_LINKER_FLAGS=${LINKER_FLAGS}")
    checked("${CMAKE_COMMAND}" --build "${consumer}" --config "${CONFIG}")
    checked("${CMAKE_CTEST_COMMAND}" --test-dir "${consumer}" -C "${CONFIG}" --output-on-failure)
endforeach()
message(STATUS "Installed PvdDom and actual SDK exports link and run in both package orders")
