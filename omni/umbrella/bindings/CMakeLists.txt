# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary

if(NOT UMBRELLA_ENABLE_PYTHON_BINDINGS)
    return()
endif()

# Only build if Carbonite SDK is available (required for these bindings)
if(NOT DEFINED CARB_SDK_DIR)
    message(STATUS "Carbonite SDK not available - skipping Python bindings")
    return()
endif()

# Check if pybind11 is available from kit-sdk dependencies
if(NOT DEFINED PYBIND11_DIR)
    set(PYBIND11_DIR "${UMBRELLA_TARGET_DEPS}/pybind11")
endif()

if(NOT EXISTS "${PYBIND11_DIR}/include/pybind11/pybind11.h")
    message(STATUS "pybind11 not found - Python bindings will not be built")
    return()
endif()

message(STATUS "Using pybind11 from packman: ${PYBIND11_DIR}")

# Set Python from kit-sdk dependencies
set(Python_ROOT_DIR "${UMBRELLA_TARGET_DEPS}/python")

if(WIN32)
    set(Python_LIBRARY "${Python_ROOT_DIR}/libs/python312.lib")
    set(Python_EXECUTABLE "${Python_ROOT_DIR}/python.exe")
    set(Python_INCLUDE_DIR "${Python_ROOT_DIR}/include")
else()
    set(Python_LIBRARY "${Python_ROOT_DIR}/lib/libpython3.12.so")
    set(Python_EXECUTABLE "${Python_ROOT_DIR}/bin/python3")
    set(Python_INCLUDE_DIR "${Python_ROOT_DIR}/include/python3.12")
endif()

# Legacy variables for older CMake FindPython
set(PYTHON_EXECUTABLE ${Python_EXECUTABLE})
set(PYTHON_LIBRARY ${Python_LIBRARY})
set(PYTHON_INCLUDE_DIR ${Python_INCLUDE_DIR})

# Make sure pybind11 can find the Python library directory
link_directories("${Python_ROOT_DIR}/libs")

# Add pybind11 subdirectory
add_subdirectory(${PYBIND11_DIR} ${CMAKE_CURRENT_BINARY_DIR}/_pybind11)
set(pybind11_FOUND TRUE)

message(STATUS "Building Python bindings with pybind11")

# Create Python bindings module
# Note: BindingsPhysicsPython.cpp #includes the other Bindings*.cpp files (unity build),
# so only the main file is listed here to avoid double-compilation and unused-function warnings.
pybind11_add_module(PhysicsCarbonite
    BindingsPhysicsPython.cpp
)

# Bindings use pybind11 and Carbonite headers that require C++ exception support.
# Override the global no-exceptions setting from omni/cmake/CompilerDefaults.cmake.
if(MSVC)
    target_compile_options(PhysicsCarbonite PRIVATE /EHsc)
    target_compile_definitions(PhysicsCarbonite PRIVATE _HAS_EXCEPTIONS=1)
endif()

# Include directories
target_include_directories(PhysicsCarbonite
    PRIVATE ${UMBRELLA_ROOT_DIR}/include
    PRIVATE ${UMBRELLA_ROOT_DIR}/include/private
    PRIVATE ${UMBRELLA_ROOT_DIR}/source
)
target_include_directories(PhysicsCarbonite SYSTEM
    PRIVATE ${CARB_SDK_DIR}/include
    PRIVATE ${Python3_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(PhysicsCarbonite
    PRIVATE PhysicsUmbrella
    PRIVATE carb
)

# Link directories - Carbonite SDK layout: _build/<platform>/<config>/
target_link_directories(PhysicsCarbonite
    PRIVATE ${CARB_SDK_DIR}/_build/${CARB_PLATFORM_DIR}/$<LOWER_CASE:$<CONFIG>>
)

# Set output name
set_target_properties(PhysicsCarbonite PROPERTIES
    OUTPUT_NAME "_physics"
    DEBUG_POSTFIX ""
    FOLDER "Bindings"
)

# Stage build artifacts into the umbrella output tree for easy testing (standalone only).
# When built as a subproject of ovexts, the omni.physics extension handles deployment.
if(_UMBRELLA_STANDALONE)
    set(_STAGE_DIR "${PX_OUTPUT_BIN_DIR}/bindings")
    add_custom_command(TARGET PhysicsCarbonite POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "${_STAGE_DIR}"
        COMMAND ${CMAKE_COMMAND} -E copy
            $<TARGET_FILE:PhysicsCarbonite>
            "${_STAGE_DIR}/$<TARGET_FILE_NAME:PhysicsCarbonite>"
        COMMAND ${CMAKE_COMMAND} -E copy
            $<TARGET_FILE:PhysicsUmbrella>
            "${_STAGE_DIR}/$<TARGET_FILE_NAME:PhysicsUmbrella>"
        COMMAND ${CMAKE_COMMAND} -E copy
            "${CARB_SDK_DIR}/_build/${CARB_PLATFORM_DIR}/$<LOWER_CASE:$<CONFIG>>/${CMAKE_SHARED_LIBRARY_PREFIX}carb${CMAKE_SHARED_LIBRARY_SUFFIX}"
            "${_STAGE_DIR}/${CMAKE_SHARED_LIBRARY_PREFIX}carb${CMAKE_SHARED_LIBRARY_SUFFIX}"
        COMMENT "Staging Python bindings, PhysicsUmbrella, and carb to ${_STAGE_DIR}"
    )
endif()

# Install bindings into a flat python/ directory (self-contained for deployment)
install(TARGETS PhysicsCarbonite
    LIBRARY DESTINATION python
    RUNTIME DESTINATION python
)

# Install Python package files
install(FILES __init__.py
    DESTINATION python
)

# Co-locate PhysicsUmbrella and carb so DLL resolution works at install time
install(FILES $<TARGET_FILE:PhysicsUmbrella>
    DESTINATION python
)
install(FILES "${CARB_SDK_DIR}/_build/${CARB_PLATFORM_DIR}/$<LOWER_CASE:$<CONFIG>>/${CMAKE_SHARED_LIBRARY_PREFIX}carb${CMAKE_SHARED_LIBRARY_SUFFIX}"
    DESTINATION python
)
