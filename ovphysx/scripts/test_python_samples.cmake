# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# ovphysx Python Sample Tests
# Runs all Python sample applications against the installed wheel.
# Usage: cmake -P scripts/test_python_samples.cmake
#
# For wheel smoke tests (python -m ovphysx), see test_python_wheel.cmake.

# Set up project paths
get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
get_filename_component(PROJECT_ROOT "${SCRIPT_DIR}/.." ABSOLUTE)
include("${SCRIPT_DIR}/build_common.cmake")

message(STATUS "")
message(STATUS "=== Python Sample Applications ===")

# Verify uv is available (required for venv creation and dependency management)
execute_process(
    COMMAND "${OVPHYSX_UV_COMMAND}" --version
    OUTPUT_VARIABLE UV_VERSION
    ERROR_QUIET
    RESULT_VARIABLE UV_CHECK
)
if(NOT UV_CHECK EQUAL 0)
    message(FATAL_ERROR "uv not found. Install it via: https://docs.astral.sh/uv/getting-started/installation/")
endif()
string(STRIP "${UV_VERSION}" UV_VERSION)
message(STATUS "Found uv: ${UV_VERSION}")

set(UV_CACHE_DIR_PATH "${PROJECT_ROOT}/_build/.uv_cache")
file(MAKE_DIRECTORY "${UV_CACHE_DIR_PATH}")
set(OVSTAGE_WHEEL_DIR "${PROJECT_ROOT}/_build/target-deps/ovstage_wheel")
file(GLOB OVSTAGE_WHEELS "${OVSTAGE_WHEEL_DIR}/ovstage-*.whl")
if(NOT OVSTAGE_WHEELS)
    message(FATAL_ERROR "ovstage wheel not found in ${OVSTAGE_WHEEL_DIR}. Run the ovphysx build first.")
endif()
set(UV_ENV
    "UV_CACHE_DIR=${UV_CACHE_DIR_PATH}"
    "UV_NO_CONFIG=1"
    "UV_FIND_LINKS=${OVSTAGE_WHEEL_DIR}"
    "UV_HTTP_TIMEOUT=300"
    "UV_SKIP_WHEEL_FILENAME_CHECK=1"
)

# Clean environment - samples should be self-contained with wheel
unset(ENV{LD_LIBRARY_PATH})
unset(ENV{VIRTUAL_ENV}) # on windows CI, sometimes a system-wide venv is active

# Check if SDK wheel exists
file(GLOB WHEEL_FILES "${PROJECT_ROOT}/_dist/ovphysx-*.whl")
if(NOT WHEEL_FILES)
    message(FATAL_ERROR "SDK wheel not found in ${PROJECT_ROOT}/_dist/. Run build_wheel.cmake first.")
endif()
list(LENGTH WHEEL_FILES _WHEEL_COUNT)
if(_WHEEL_COUNT GREATER 1)
    message(FATAL_ERROR
        "Multiple wheels found in ${PROJECT_ROOT}/_dist/ (${_WHEEL_COUNT}):\n"
        "  ${WHEEL_FILES}\n"
        "Clean _dist/ and rebuild to ensure samples test the correct artifact.")
endif()
list(GET WHEEL_FILES 0 SDK_WHEEL_PATH)

set(PYTHON_VERSIONS
    "3.10"
    "3.11"
    "3.12"
    "3.13"
    # Python 3.14 is intentionally not in this matrix. It's a bleeding-edge alpha and
    # third-party wheels (notably NumPy) may not be ABI-compatible yet, which would
    # prevent meaningful end-to-end validation of the tensor samples on CI.
)

# ==============================================================================
# Common helpers
# ==============================================================================

# setup_sample_venv(<samples_dir> <py_ver>)
# Creates a fresh venv, syncs pyproject.toml deps, installs the local wheel.
# Requires UV_PYTHON_PATH to be set by the caller (via ensure_uv_managed_python).
macro(setup_sample_venv SAMPLES_DIR PY_VER)
    set(_VENV_DIR "${${SAMPLES_DIR}}/.venv")
    message(STATUS "Creating fresh venv at ${_VENV_DIR} (Python ${${PY_VER}}: ${UV_PYTHON_PATH})...")
    file(REMOVE_RECURSE "${_VENV_DIR}")
    execute_process(
        COMMAND "${OVPHYSX_UV_COMMAND}" venv --python "${UV_PYTHON_PATH}" "${_VENV_DIR}"
        WORKING_DIRECTORY "${${SAMPLES_DIR}}"
        RESULT_VARIABLE _VENV_RESULT
        ERROR_VARIABLE _VENV_ERROR
    )
    if(NOT _VENV_RESULT STREQUAL "0")
        message(FATAL_ERROR "Failed to create venv for Python ${${PY_VER}} (exit code: ${_VENV_RESULT})\n"
                            "Python path: ${UV_PYTHON_PATH}\n"
                            "Venv path: ${_VENV_DIR}\n"
                            "Error: ${_VENV_ERROR}")
    endif()

    message(STATUS "Syncing sample dependencies (pyproject.toml)")
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E env ${UV_ENV} "${OVPHYSX_UV_COMMAND}" sync --python ${_VENV_DIR} --no-install-project --no-install-package ovphysx
        WORKING_DIRECTORY "${${SAMPLES_DIR}}"
        RESULT_VARIABLE _SYNC_RESULT
    )
    if(NOT _SYNC_RESULT STREQUAL "0")
        message(FATAL_ERROR "Failed to sync sample dependencies")
    endif()

    message(STATUS "Installing locally built ovphysx wheel: ${SDK_WHEEL_PATH}")
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E env ${UV_ENV} "${OVPHYSX_UV_COMMAND}" pip install --reinstall --python ${_VENV_DIR} ${SDK_WHEEL_PATH}
        WORKING_DIRECTORY "${${SAMPLES_DIR}}"
        RESULT_VARIABLE _INSTALL_RESULT
    )
    if(NOT _INSTALL_RESULT STREQUAL "0")
        message(FATAL_ERROR "Failed to install SDK wheel")
    endif()
endmacro()

# run_python_samples(<samples_dir> <sample_list> <label>)
# Runs each sample in the list.
macro(run_python_samples SAMPLES_DIR SAMPLE_LIST LABEL)
    set(_VENV_DIR "${${SAMPLES_DIR}}/.venv")
    foreach(_SAMPLE_NAME ${${SAMPLE_LIST}})
        set(_SAMPLE_PATH "${${SAMPLES_DIR}}/${_SAMPLE_NAME}")
        message(STATUS "  Running ${${LABEL}}${_SAMPLE_NAME}...")

        execute_process(
            COMMAND ${CMAKE_COMMAND} -E env ${UV_ENV} PYTHONUNBUFFERED=1 "${OVPHYSX_UV_COMMAND}" run --python ${_VENV_DIR} --no-sync -- python ${_SAMPLE_PATH}
            WORKING_DIRECTORY "${${SAMPLES_DIR}}"
            RESULT_VARIABLE _SAMPLE_RESULT
            OUTPUT_VARIABLE _SAMPLE_OUTPUT
            ERROR_VARIABLE _SAMPLE_OUTPUT
            ECHO_OUTPUT_VARIABLE
            ECHO_ERROR_VARIABLE
            TIMEOUT 120
        )

        if(NOT _SAMPLE_RESULT STREQUAL "0")
            message(STATUS "Sample output: ${_SAMPLE_OUTPUT}")
            message(FATAL_ERROR "Sample ${${LABEL}}${_SAMPLE_NAME} failed (exit code: ${_SAMPLE_RESULT})")
        else()
            message(STATUS "  [PASS] ${${LABEL}}${_SAMPLE_NAME}")
        endif()
    endforeach()
endmacro()

# ==============================================================================
# External Python Samples (clean, user-facing examples)
# ==============================================================================
set(PYTHON_SAMPLES_DIR "${PROJECT_ROOT}/tests/python_samples")

if(NOT EXISTS "${PYTHON_SAMPLES_DIR}")
    message(FATAL_ERROR "Python samples directory not found: ${PYTHON_SAMPLES_DIR}")
endif()

message(STATUS "")
message(STATUS "--- Running External Python Samples ---")

set(BASE_SAMPLES
    "hello_world.py"
    "contact_binding.py"
    "tensor_bindings.py"
    "clone.py"
    "omnipvd_recording.py"
    "tensor_bindings_views.py"  # TensorBindingsAPI-only (pure ctypes), cross-minor
    "output_read.py"            # closed-loop ovstage control-in / output-read (ADR-0007)
)

file(GLOB EXTERNAL_SAMPLE_FILES RELATIVE "${PYTHON_SAMPLES_DIR}" "${PYTHON_SAMPLES_DIR}/*.py")
list(SORT EXTERNAL_SAMPLE_FILES)
set(EXPECTED_EXTERNAL_SAMPLE_FILES ${BASE_SAMPLES})
list(SORT EXPECTED_EXTERNAL_SAMPLE_FILES)
if(NOT EXTERNAL_SAMPLE_FILES STREQUAL EXPECTED_EXTERNAL_SAMPLE_FILES)
    message(FATAL_ERROR
        "External Python sample list is out of sync with tests/python_samples/*.py.\n"
        "Discovered: ${EXTERNAL_SAMPLE_FILES}\n"
        "Configured: ${EXPECTED_EXTERNAL_SAMPLE_FILES}")
endif()

set(EXTERNAL_LABEL "")

foreach(PY_VER ${PYTHON_VERSIONS})
    message(STATUS "")
    message(STATUS "--- Running Python Samples (Python ${PY_VER}) ---")

    # Always use uv-managed Python for repeatable tests
    ensure_uv_managed_python(${PY_VER} UV_PYTHON_PATH)

    # Resolve Python lib directory for libpython (needed by tensor bindings)
    execute_process(
        COMMAND ${UV_PYTHON_PATH} -c "import sysconfig; print(sysconfig.get_config_var('LIBDIR') or '')"
        OUTPUT_VARIABLE PY_LIBDIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    execute_process(
        COMMAND ${UV_PYTHON_PATH} -c "import sysconfig; print(sysconfig.get_config_var('LIBPL') or '')"
        OUTPUT_VARIABLE PY_LIBPL
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )

    setup_sample_venv(PYTHON_SAMPLES_DIR PY_VER)

    run_python_samples(PYTHON_SAMPLES_DIR BASE_SAMPLES EXTERNAL_LABEL)
endforeach()

# ==============================================================================
# Internal Python Samples (not included in open-source distribution)
# ==============================================================================
set(INTERNAL_SAMPLES_DIR "${PROJECT_ROOT}/tests/python_samples_internal")

if(EXISTS "${INTERNAL_SAMPLES_DIR}")
    message(STATUS "")
    message(STATUS "--- Running Internal Python Samples ---")

    set(INTERNAL_LABEL "internal/")

    foreach(PY_VER ${PYTHON_VERSIONS})
        message(STATUS "")
        message(STATUS "--- Running Internal Samples (Python ${PY_VER}) ---")

        ensure_uv_managed_python(${PY_VER} UV_PYTHON_PATH)
        setup_sample_venv(INTERNAL_SAMPLES_DIR PY_VER)

        set(INTERNAL_TO_RUN
            "self_contained.py"
            "rigid_body_falling_tensors.py"
        )

        run_python_samples(INTERNAL_SAMPLES_DIR INTERNAL_TO_RUN INTERNAL_LABEL)
    endforeach()
endif()

# ==============================================================================
# Extra Python Samples (additional dependencies beyond ovphysx+numpy)
# ==============================================================================
set(EXTRA_SAMPLES_DIR "${PROJECT_ROOT}/tests/python_samples_extra")

if(EXISTS "${EXTRA_SAMPLES_DIR}")
    message(STATUS "")
    message(STATUS "--- Running Extra Python Samples (Python 3.12 only) ---")

    # Extra samples have heavier dependencies (e.g. rerun-sdk) and run on a
    # single Python version to keep CI fast.
    set(EXTRA_PY_VER "3.12")

    file(GLOB _EXTRA_SUBDIRS RELATIVE "${EXTRA_SAMPLES_DIR}" "${EXTRA_SAMPLES_DIR}/*")
    foreach(_SUBDIR ${_EXTRA_SUBDIRS})
        set(_SUBDIR_PATH "${EXTRA_SAMPLES_DIR}/${_SUBDIR}")
        if(NOT IS_DIRECTORY "${_SUBDIR_PATH}")
            continue()
        endif()
        if(NOT EXISTS "${_SUBDIR_PATH}/pyproject.toml")
            continue()
        endif()

        message(STATUS "")
        message(STATUS "--- Extra sample: ${_SUBDIR} (Python ${EXTRA_PY_VER}) ---")

        ensure_uv_managed_python(${EXTRA_PY_VER} UV_PYTHON_PATH)
        setup_sample_venv(_SUBDIR_PATH EXTRA_PY_VER)

        # Discover and run all .py files
        file(GLOB _EXTRA_SCRIPTS "${_SUBDIR_PATH}/*.py")
        set(_EXTRA_VENV "${_SUBDIR_PATH}/.venv")
        foreach(_SCRIPT ${_EXTRA_SCRIPTS})
            get_filename_component(_SCRIPT_NAME "${_SCRIPT}" NAME)
            message(STATUS "  Running extra/${_SUBDIR}/${_SCRIPT_NAME}...")

            execute_process(
                COMMAND ${CMAKE_COMMAND} -E env ${UV_ENV} PYTHONUNBUFFERED=1
                    "${OVPHYSX_UV_COMMAND}" run --python ${_EXTRA_VENV} --no-sync -- python ${_SCRIPT}
                WORKING_DIRECTORY "${_SUBDIR_PATH}"
                RESULT_VARIABLE _EXTRA_RESULT
                OUTPUT_VARIABLE _EXTRA_OUTPUT
                ERROR_VARIABLE _EXTRA_OUTPUT
                ECHO_OUTPUT_VARIABLE
                ECHO_ERROR_VARIABLE
                TIMEOUT 120
            )

            if(NOT _EXTRA_RESULT STREQUAL "0")
                message(STATUS "Sample output: ${_EXTRA_OUTPUT}")
                message(FATAL_ERROR "Sample extra/${_SUBDIR}/${_SCRIPT_NAME} failed (exit code: ${_EXTRA_RESULT})")
            else()
                message(STATUS "  [PASS] extra/${_SUBDIR}/${_SCRIPT_NAME}")
            endif()
        endforeach()
    endforeach()
endif()

message(STATUS "")
message(STATUS "Python sample tests: PASSED")
