## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.21)
project(OvdSdkWorkflow LANGUAGES C CXX)
get_filename_component(PHYSX_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(PX_BUILDPVDRUNTIME ON CACHE BOOL "" FORCE)
option(PX_GENERATE_GPU_PROJECTS "Build GPU support for the optional capture case" OFF)
set(PX_GENERATE_STATIC_LIBRARIES ON CACHE BOOL "Build static SDK archives")
set(PX_BUILDSNIPPETS OFF CACHE BOOL "" FORCE)
set(PVDDOM_BUILD_TOOLS ON CACHE BOOL "" FORCE)
set(PVDDOM_BUILD_INSPECTION ON CACHE BOOL "" FORCE)
set(PVDDOM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory("${PHYSX_ROOT}" sdk)
add_subdirectory("${PHYSX_ROOT}/pvddom" pvddom)
add_executable(ovd_record ovd_record.cpp)
set_target_properties(ovd_record PROPERTIES OUTPUT_NAME ovd-record)
target_compile_features(ovd_record PRIVATE cxx_std_14)
target_compile_definitions(ovd_record PRIVATE PVDDOM_BUILD_INFO="${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION}-$<CONFIG>")
target_include_directories(ovd_record PRIVATE ../src ../tools)
target_link_libraries(ovd_record PRIVATE physx_lib PvdDom)
if(PX_GENERATE_STATIC_LIBRARIES)
    target_compile_definitions(ovd_record PRIVATE PX_PHYSX_STATIC_LIB)
endif()
install(TARGETS ovd_record RUNTIME DESTINATION bin)
if(UNIX AND NOT APPLE AND NOT PX_GENERATE_STATIC_LIBRARIES)
    if(IS_ABSOLUTE "${CMAKE_INSTALL_LIBDIR}")
        set_target_properties(ovd_record PROPERTIES INSTALL_RPATH "${CMAKE_INSTALL_LIBDIR}")
    else()
        set_target_properties(ovd_record PROPERTIES INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
    endif()
endif()
if(WIN32 AND (NOT PX_GENERATE_STATIC_LIBRARIES OR (PX_GENERATE_GPU_PROJECTS AND NOT PX_GENERATE_GPU_STATIC_LIBRARIES)))
    add_custom_command(TARGET ovd_record POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:ovd_record> $<TARGET_FILE_DIR:ovd_record>
        COMMAND_EXPAND_LISTS VERBATIM)
    install(FILES "$<TARGET_RUNTIME_DLLS:ovd_record>" DESTINATION bin)
endif()
install(DIRECTORY "${PHYSX_ROOT}/skills/" DESTINATION share/physx/skills)
install(FILES "${PHYSX_ROOT}/SKILLS.md" DESTINATION share/physx)
enable_testing()
add_executable(ovd_record_errors ../test/record_errors.cpp)
target_include_directories(ovd_record_errors PRIVATE ../src ../tools)
target_link_libraries(ovd_record_errors PRIVATE physx_lib PvdDom)
target_compile_definitions(ovd_record_errors PRIVATE PVDDOM_BUILD_INFO="test")
if(PX_GENERATE_STATIC_LIBRARIES)
    target_compile_definitions(ovd_record_errors PRIVATE PX_PHYSX_STATIC_LIB)
endif()
if(WIN32 AND (NOT PX_GENERATE_STATIC_LIBRARIES OR (PX_GENERATE_GPU_PROJECTS AND NOT PX_GENERATE_GPU_STATIC_LIBRARIES)))
    add_custom_command(TARGET ovd_record_errors POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:ovd_record_errors> $<TARGET_FILE_DIR:ovd_record_errors>
        COMMAND_EXPAND_LISTS VERBATIM)
endif()
add_dependencies(ovd_record ovd_record_errors)
foreach(runtimeTarget ovd_record ovd_record_errors)
    target_compile_definitions(${runtimeTarget} PRIVATE $<$<OR:$<CONFIG:checked>,$<CONFIG:profile>>:NDEBUG>)
    file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/runtime-closure/$<CONFIG>/${runtimeTarget}.cmake"
        CONTENT "set(EXECUTABLE \"$<TARGET_FILE:${runtimeTarget}>\")\nset(RUNTIME_DLLS \"$<TARGET_RUNTIME_DLLS:${runtimeTarget}>\")\n")
endforeach()
set(recordClosure "${CMAKE_CURRENT_BINARY_DIR}/runtime-closure/$<CONFIG>/ovd_record.cmake")
set(errorsClosure "${CMAKE_CURRENT_BINARY_DIR}/runtime-closure/$<CONFIG>/ovd_record_errors.cmake")
set(requireRuntimeDlls OFF)
if(WIN32 AND (NOT PX_GENERATE_STATIC_LIBRARIES OR (PX_GENERATE_GPU_PROJECTS AND NOT PX_GENERATE_GPU_STATIC_LIBRARIES)))
    set(requireRuntimeDlls ON)
endif()
add_test(NAME ovd_record_error_diagnostic COMMAND ${CMAKE_COMMAND}
    -DCLOSURE=${errorsClosure} -DREQUIRE_RUNTIME_DLLS=${requireRuntimeDlls}
    -DCAPTURE_DIR=${CMAKE_CURRENT_BINARY_DIR}/captures
    -P ${CMAKE_CURRENT_SOURCE_DIR}/../test/run_with_runtime_closure.cmake)

# Exercise the SDK's real target set and config with the installed standalone package.
get_directory_property(sdkExportTargets DIRECTORY "${PHYSX_ROOT}" DEFINITION PHYSX_INSTALL_LIBS)
export(TARGETS ${sdkExportTargets} physx_lib NAMESPACE PhysX:: FILE "${CMAKE_BINARY_DIR}/sdk-package/PhysXTargets.cmake")
include(CMakePackageConfigHelpers)
configure_package_config_file("${PHYSX_ROOT}/source/compiler/cmake/PhysXConfig.cmake.in"
    "${CMAKE_BINARY_DIR}/sdk-package/PhysXConfig.cmake" INSTALL_DESTINATION sdk-package)
add_test(NAME ovd_sdk_package_coexistence COMMAND ${CMAKE_COMMAND}
    -DSOURCE=${PHYSX_ROOT}/pvddom -DBUILD=${CMAKE_BINARY_DIR} -DCONFIG=$<CONFIG>
    -DGENERATOR=${CMAKE_GENERATOR} -DSTATIC_CRT=${NV_USE_STATIC_WINCRT}
    "-DCXX_FLAGS=${CMAKE_CXX_FLAGS}" "-DLINKER_FLAGS=${CMAKE_EXE_LINKER_FLAGS}"
    -P ${CMAKE_CURRENT_SOURCE_DIR}/../test/package_coexistence.cmake)
add_test(NAME ovd_record_options COMMAND ${CMAKE_COMMAND}
    -DRECORDER=$<TARGET_FILE:ovd_record> -DCLI=$<TARGET_FILE:ovd_inspect>
    -DWORK=${CMAKE_CURRENT_BINARY_DIR}/captures
    -DCLOSURE=${recordClosure} -DREQUIRE_RUNTIME_DLLS=${requireRuntimeDlls}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/../test/record_cli.cmake)
add_test(NAME ovd_sdk_capture_inspect_compare COMMAND ${CMAKE_COMMAND}
    -DBUILD=${CMAKE_BINARY_DIR} -DCONFIG=$<CONFIG>
    -DRECORDER=$<TARGET_FILE:ovd_record> -DCLI=$<TARGET_FILE:ovd_inspect>
    -DINSTALL_LIBDIR=${CMAKE_INSTALL_LIBDIR}
    -DCLOSURE=${recordClosure} -DREQUIRE_RUNTIME_DLLS=${requireRuntimeDlls}
    -DWORK=${CMAKE_CURRENT_BINARY_DIR}/captures -P ${CMAKE_CURRENT_SOURCE_DIR}/../test/workflow.cmake)
if(PX_GENERATE_GPU_PROJECTS)
    add_test(NAME ovd_sdk_gpu_capture COMMAND ${CMAKE_COMMAND}
        -DGPU=ON -DRECORDER=$<TARGET_FILE:ovd_record> -DCLI=$<TARGET_FILE:ovd_inspect>
        -DINSTALL_LIBDIR=${CMAKE_INSTALL_LIBDIR}
        -DCLOSURE=${recordClosure} -DREQUIRE_RUNTIME_DLLS=${requireRuntimeDlls}
        -DWORK=${CMAKE_CURRENT_BINARY_DIR}/gpu-captures -P ${CMAKE_CURRENT_SOURCE_DIR}/../test/workflow.cmake)
    set_tests_properties(ovd_sdk_gpu_capture PROPERTIES SKIP_REGULAR_EXPRESSION "SKIP: GPU unavailable")
endif()
