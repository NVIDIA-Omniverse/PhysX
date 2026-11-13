## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.21)
project(InstalledPvdDomConsumer LANGUAGES CXX)
if(PACKAGE_ORDER STREQUAL "sdk-first")
    find_package(PhysX CONFIG REQUIRED)
endif()
if(DEFINED REQUESTED_DOM_COMPONENT)
    find_package(PvdDom ${REQUESTED_DOM_VERSION} CONFIG REQUIRED COMPONENTS ${REQUESTED_DOM_COMPONENT})
else()
    find_package(PvdDom ${REQUESTED_DOM_VERSION} CONFIG REQUIRED)
endif()
if(PACKAGE_ORDER STREQUAL "dom-first")
    find_package(PhysX CONFIG REQUIRED)
endif()
add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE PhysX::PvdDom)
if(PACKAGE_ORDER)
    target_link_libraries(consumer PRIVATE PhysX::PhysX PhysX::PhysXFoundation PhysX::PhysXExtensions)
    target_compile_definitions(consumer PRIVATE TEST_PHYSX_PACKAGE
        $<$<NOT:$<CONFIG:Debug>>:NDEBUG>)
    get_target_property(sdkType PhysX::PhysX TYPE)
    if(sdkType STREQUAL "STATIC_LIBRARY")
        target_compile_definitions(consumer PRIVATE PX_PHYSX_STATIC_LIB)
    elseif(WIN32)
        add_custom_command(TARGET consumer POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_RUNTIME_DLLS:consumer> $<TARGET_FILE_DIR:consumer>
            COMMAND_EXPAND_LISTS VERBATIM)
    endif()
endif()
enable_testing()
set(closure "${CMAKE_CURRENT_BINARY_DIR}/runtime-closure/$<CONFIG>/consumer.cmake")
set(requireRuntimeDlls OFF)
set(requiredDlls "")
if(WIN32 AND PACKAGE_ORDER AND NOT sdkType STREQUAL "STATIC_LIBRARY")
    set(requireRuntimeDlls ON)
    foreach(sdkTarget PhysX::PhysX PhysX::PhysXCommon PhysX::PhysXFoundation)
        get_target_property(sdkTargetType ${sdkTarget} TYPE)
        if(sdkTargetType STREQUAL "SHARED_LIBRARY")
            list(APPEND requiredDlls "$<TARGET_FILE:${sdkTarget}>")
        endif()
    endforeach()
endif()
file(GENERATE OUTPUT "${closure}"
    CONTENT "set(EXECUTABLE \"$<TARGET_FILE:consumer>\")\nset(RUNTIME_DLLS \"$<TARGET_RUNTIME_DLLS:consumer>\")\nset(REQUIRED_DLLS \"${requiredDlls}\")\n")
if(PACKAGE_ORDER)
    add_test(NAME installed_consumer COMMAND ${CMAKE_COMMAND}
        -DCLOSURE=${closure} -DREQUIRE_RUNTIME_DLLS=${requireRuntimeDlls}
        -DARGS=${CMAKE_CURRENT_SOURCE_DIR}/../data/test_scene.ovd
        -P ${CMAKE_CURRENT_SOURCE_DIR}/../run_with_runtime_closure.cmake)
else()
    add_test(NAME installed_consumer COMMAND ${CMAKE_COMMAND}
        -DCLOSURE=${closure} -DREQUIRE_RUNTIME_DLLS=${requireRuntimeDlls}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/../run_with_runtime_closure.cmake)
endif()
