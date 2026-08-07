# Fetch IsaacLab ANYmal-C asset for the Lab.anymal_* benchmarks.
# Wrapper around scripts/fetch_anymal_asset.py — invokes the platform-
# agnostic Python helper via packman's bundled Python.
#
# Usage:
#     cmake -P scripts/fetch_anymal_asset.cmake
#
# Idempotent: each file is checked for expected size and skipped if matched.

cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
get_filename_component(OVPHYSX_ROOT "${SCRIPT_DIR}/.." ABSOLUTE)

if(WIN32)
    set(_TARGET_PYTHON "${OVPHYSX_ROOT}/_build/target-deps/python/python.exe")
else()
    set(_TARGET_PYTHON "${OVPHYSX_ROOT}/_build/target-deps/python/bin/python3")
endif()

# Fall back to system python3 if target-deps isn't fetched yet — the script
# uses only stdlib so any Python 3.8+ works.
if(NOT EXISTS "${_TARGET_PYTHON}")
    find_program(_TARGET_PYTHON NAMES python3 python)
    if(NOT _TARGET_PYTHON)
        message(FATAL_ERROR "No Python interpreter found (looked at packman + PATH)")
    endif()
endif()

set(_FETCH_SCRIPT "${SCRIPT_DIR}/fetch_anymal_asset.py")
if(NOT EXISTS "${_FETCH_SCRIPT}")
    message(FATAL_ERROR "Fetch script not found: ${_FETCH_SCRIPT}")
endif()

message(STATUS "Running ${_FETCH_SCRIPT}")
execute_process(
    COMMAND "${_TARGET_PYTHON}" "${_FETCH_SCRIPT}"
    RESULT_VARIABLE _RC
)
if(NOT _RC EQUAL 0)
    message(FATAL_ERROR "ANYmal asset fetch failed (exit ${_RC})")
endif()
