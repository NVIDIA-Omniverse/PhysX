## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.21)
execute_process(COMMAND "${CMAKE_COMMAND}" --install "${BUILD}" --config "${CONFIG}" --prefix "${BUILD}/installed"
    RESULT_VARIABLE code OUTPUT_VARIABLE log ERROR_VARIABLE diagnostic)
if(NOT code EQUAL 0)
    message(FATAL_ERROR "Install failed: ${log}\n${diagnostic}")
endif()
set(CLI "${BUILD}/installed/bin/ovd-inspect")
if(WIN32)
    string(APPEND CLI ".exe")
endif()
set(WORK "${BUILD}/outside checkout")
include("${SCRIPT}")
execute_process(COMMAND "${CMAKE_COMMAND}" -S "${CONSUMER}" -B "${BUILD}/consumer"
    -G "${GENERATOR}" -DCMAKE_PREFIX_PATH=${BUILD}/installed -DCMAKE_BUILD_TYPE=${CONFIG}
    -DREQUESTED_DOM_VERSION=${REQUESTED_DOM_VERSION}
    "-DCMAKE_CXX_FLAGS=${CXX_FLAGS}" "-DCMAKE_EXE_LINKER_FLAGS=${LINKER_FLAGS}"
    RESULT_VARIABLE code OUTPUT_VARIABLE log ERROR_VARIABLE diagnostic)
if(NOT code EQUAL 0)
    message(FATAL_ERROR "Installed consumer configure failed: ${log}\n${diagnostic}")
endif()
foreach(case newer-major older-major unknown-component)
    set(extra)
    if(case STREQUAL "newer-major" OR case STREQUAL "older-major")
        string(REGEX MATCH "^[0-9]+" major "${REQUESTED_DOM_VERSION}")
        if(case STREQUAL "newer-major")
            math(EXPR wrongMajor "${major} + 1")
        else()
            math(EXPR wrongMajor "${major} - 1")
        endif()
        set(extra -DREQUESTED_DOM_VERSION=${wrongMajor}.0.0)
    else()
        set(extra -DREQUESTED_DOM_VERSION=${REQUESTED_DOM_VERSION} -DREQUESTED_DOM_COMPONENT=Imaginary)
    endif()
    execute_process(COMMAND "${CMAKE_COMMAND}" -S "${CONSUMER}" -B "${BUILD}/consumer-${case}"
        -G "${GENERATOR}" -DCMAKE_PREFIX_PATH=${BUILD}/installed ${extra}
        RESULT_VARIABLE rejected OUTPUT_VARIABLE rejectLog ERROR_VARIABLE rejectDiagnostic)
    if("${rejected}" STREQUAL "0")
        message(FATAL_ERROR "Installed package accepted ${case}")
    endif()
    if((case STREQUAL "newer-major" OR case STREQUAL "older-major") AND NOT rejectDiagnostic MATCHES "requested version")
        message(FATAL_ERROR "Wrong-version check failed for another reason: ${rejectLog}\n${rejectDiagnostic}")
    elseif(case STREQUAL "unknown-component" AND NOT rejectDiagnostic MATCHES "PvdDom_FOUND to FALSE")
        message(FATAL_ERROR "Unknown-component check failed for another reason: ${rejectLog}\n${rejectDiagnostic}")
    endif()
endforeach()
execute_process(COMMAND "${CMAKE_COMMAND}" -S "${CONSUMER}" -B "${BUILD}/consumer-older-minor"
    -G "${GENERATOR}" -DCMAKE_PREFIX_PATH=${BUILD}/installed -DREQUESTED_DOM_VERSION=5.0
    RESULT_VARIABLE olderMinorCode OUTPUT_VARIABLE olderMinorLog ERROR_VARIABLE olderMinorDiagnostic)
if(NOT olderMinorCode EQUAL 0)
    message(FATAL_ERROR "Installed package rejected same-major older minor: ${olderMinorLog}\n${olderMinorDiagnostic}")
endif()
execute_process(COMMAND "${CMAKE_COMMAND}" --build "${BUILD}/consumer" --config "${CONFIG}"
    RESULT_VARIABLE code OUTPUT_VARIABLE log ERROR_VARIABLE diagnostic)
if(NOT code EQUAL 0)
    message(FATAL_ERROR "Installed consumer build failed: ${log}\n${diagnostic}")
endif()
execute_process(COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${BUILD}/consumer" -C "${CONFIG}" --output-on-failure
    RESULT_VARIABLE code OUTPUT_VARIABLE log ERROR_VARIABLE diagnostic)
if(NOT code EQUAL 0)
    message(FATAL_ERROR "Installed consumer failed: ${log}\n${diagnostic}")
endif()
