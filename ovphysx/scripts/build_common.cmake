# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# Common build configuration

# Get script directory and project root
get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
get_filename_component(PROJECT_ROOT "${SCRIPT_DIR}/.." ABSOLUTE)
include("${SCRIPT_DIR}/crossplatform_helpers.cmake")
include("${SCRIPT_DIR}/host_toolchain.cmake")

if(NOT DEFINED BUILD_TYPE)
    set(BUILD_TYPE "Release")
endif()
string(TOLOWER "${BUILD_TYPE}" BUILD_TYPE_LOWER)

if(NOT DEFINED OVPHYSX_USE_RELEASE_RUNTIME_DEPS)
    if(DEFINED ENV{OVPHYSX_USE_RELEASE_RUNTIME_DEPS} AND NOT "$ENV{OVPHYSX_USE_RELEASE_RUNTIME_DEPS}" STREQUAL "")
        set(OVPHYSX_USE_RELEASE_RUNTIME_DEPS "$ENV{OVPHYSX_USE_RELEASE_RUNTIME_DEPS}")
    else()
        set(OVPHYSX_USE_RELEASE_RUNTIME_DEPS ON)
    endif()
endif()
string(TOUPPER "${OVPHYSX_USE_RELEASE_RUNTIME_DEPS}" _OVPHYSX_RELEASE_RUNTIME_DEPS_VALUE)
if(_OVPHYSX_RELEASE_RUNTIME_DEPS_VALUE MATCHES "^(ON|TRUE|YES|1)$")
    set(OVPHYSX_USE_RELEASE_RUNTIME_DEPS ON)
    set(OVPHYSX_RUNTIME_DEPS_CONFIG "release")
elseif(_OVPHYSX_RELEASE_RUNTIME_DEPS_VALUE MATCHES "^(OFF|FALSE|NO|0)$")
    set(OVPHYSX_USE_RELEASE_RUNTIME_DEPS OFF)
    if(BUILD_TYPE_LOWER STREQUAL "debug")
        set(OVPHYSX_RUNTIME_DEPS_CONFIG "debug")
    else()
        set(OVPHYSX_RUNTIME_DEPS_CONFIG "release")
    endif()
else()
    message(FATAL_ERROR
        "OVPHYSX_USE_RELEASE_RUNTIME_DEPS must be ON or OFF; got '${OVPHYSX_USE_RELEASE_RUNTIME_DEPS}'")
endif()
if(BUILD_TYPE_LOWER STREQUAL "debug" AND NOT OVPHYSX_USE_RELEASE_RUNTIME_DEPS)
    message(FATAL_ERROR
        "True-Debug runtime dependencies are unsupported because the published "
        "OVStage package supplies a Release-only resolver/client runtime. "
        "Use -DOVPHYSX_USE_RELEASE_RUNTIME_DEPS=ON for Debug compilation.")
endif()
unset(_OVPHYSX_RELEASE_RUNTIME_DEPS_VALUE)

set(BUILD_PATH "${PROJECT_ROOT}/_build")

# Compute a safe parallel-build job count, shared by every script that invokes
# a compile (main build + the sample-test source builds). Building the
# ovruntime/PhysX stack is RAM-bound, not core-bound: USD/PhysX/boost template
# TUs hold ~1-2 GiB resident per cc1plus, so an unbounded `make -j` (e.g.
# `cmake --build --parallel` with no count) spawns enough workers to exhaust
# RAM and thrash the machine into swap. Bound by BOTH cores and RAM.
#
# -DJOBS=N overrides. CI/containers run under a cgroup CPU quota -- trust it
# (capped at 16 for linker OOM). Bare-metal/dev: half the cores AND ~4 GiB of
# RAM budget per job, whichever is smaller.
function(ovphysx_compute_build_jobs _out_var)
    if(DEFINED JOBS AND NOT "${JOBS}" STREQUAL "")
        set(${_out_var} "${JOBS}" PARENT_SCOPE)
        return()
    endif()

    set(_jobs 0)
    set(_cgroup FALSE)
    if(EXISTS "/sys/fs/cgroup/cpu.max")
        file(READ "/sys/fs/cgroup/cpu.max" _cpu_max)
        string(STRIP "${_cpu_max}" _cpu_max)
        string(REPLACE " " ";" _parts "${_cpu_max}")
        list(GET _parts 0 _quota)
        list(GET _parts 1 _period)
        if(NOT "${_quota}" STREQUAL "max")
            math(EXPR _jobs "(${_quota} + ${_period} - 1) / ${_period}")
            if(_jobs GREATER 0)
                set(_cgroup TRUE)
            endif()
        endif()
    endif()
    if(NOT _cgroup
       AND EXISTS "/sys/fs/cgroup/cpu/cpu.cfs_quota_us"
       AND EXISTS "/sys/fs/cgroup/cpu/cpu.cfs_period_us")
        file(READ "/sys/fs/cgroup/cpu/cpu.cfs_quota_us" _quota)
        file(READ "/sys/fs/cgroup/cpu/cpu.cfs_period_us" _period)
        string(STRIP "${_quota}" _quota)
        string(STRIP "${_period}" _period)
        if(_quota GREATER 0 AND _period GREATER 0)
            math(EXPR _jobs "(${_quota} + ${_period} - 1) / ${_period}")
            if(_jobs GREATER 0)
                set(_cgroup TRUE)
            endif()
        endif()
    endif()

    if(_cgroup)
        # Also clamp by the cgroup memory limit (4 GiB/job): a high CPU quota on
        # a memory-constrained runner would otherwise pick too many jobs and OOM.
        set(_cgroup_ram_jobs 0)
        if(EXISTS "/sys/fs/cgroup/memory.max")
            file(READ "/sys/fs/cgroup/memory.max" _mem_max)
            string(STRIP "${_mem_max}" _mem_max)
            if(NOT "${_mem_max}" STREQUAL "max")
                math(EXPR _cgroup_ram_jobs "${_mem_max} / 4294967296")
            endif()
        elseif(EXISTS "/sys/fs/cgroup/memory/memory.limit_in_bytes")
            file(READ "/sys/fs/cgroup/memory/memory.limit_in_bytes" _mem_max)
            string(STRIP "${_mem_max}" _mem_max)
            if(_mem_max GREATER 0)
                math(EXPR _cgroup_ram_jobs "${_mem_max} / 4294967296")
            endif()
        endif()
        if(_cgroup_ram_jobs GREATER 0 AND _cgroup_ram_jobs LESS _jobs)
            set(_jobs ${_cgroup_ram_jobs})
        endif()
        if(_jobs GREATER 16)
            set(_jobs 16)
        endif()
        if(_jobs LESS 1)
            set(_jobs 1)
        endif()
        set(${_out_var} ${_jobs} PARENT_SCOPE)
        return()
    endif()

    # Bare-metal / dev: min(cores/2, RAM_GiB/4).
    include(ProcessorCount)
    ProcessorCount(_ncores)
    if(_ncores LESS 1)
        set(_ncores 1)
    endif()
    math(EXPR _by_core "${_ncores} / 2")

    set(_by_ram 0)
    if(EXISTS "/proc/meminfo")
        file(STRINGS "/proc/meminfo" _memtotal_line REGEX "^MemTotal:")
        string(REGEX MATCH "[0-9]+" _mem_kb "${_memtotal_line}")
        if(_mem_kb)
            # KiB / (4 GiB in KiB) == RAM_GiB / 4
            math(EXPR _by_ram "${_mem_kb} / 4194304")
        endif()
    endif()

    set(_result ${_by_core})
    if(_by_ram GREATER 0 AND _by_ram LESS _result)
        set(_result ${_by_ram})
    endif()
    if(_result LESS 1)
        set(_result 1)
    endif()
    set(${_out_var} ${_result} PARENT_SCOPE)
endfunction()

# Resolve uv executable once for all scripts.
# MSBuild/CTest environments on Windows may not inherit the same PATH as an
# interactive shell, so we include common installation locations as hints.
if(NOT DEFINED OVPHYSX_UV_COMMAND OR OVPHYSX_UV_COMMAND STREQUAL "")
    set(_UV_HINTS
        "$ENV{ProgramData}/chocolatey/bin"
        "$ENV{USERPROFILE}/.local/bin"
        "$ENV{LOCALAPPDATA}/uv/bin"
        "$ENV{USERPROFILE}/.cargo/bin"
    )
    file(GLOB _PY_SCRIPT_DIRS "$ENV{LOCALAPPDATA}/Programs/Python/*/Scripts")
    list(APPEND _UV_HINTS ${_PY_SCRIPT_DIRS})
    # `pip install --user uv` on Windows installs into %APPDATA%\Python\PythonXY\Scripts,
    # which is not on PATH by default and is not one of the locations above.
    file(GLOB _PY_USER_SCRIPT_DIRS "$ENV{APPDATA}/Python/Python*/Scripts")
    list(APPEND _UV_HINTS ${_PY_USER_SCRIPT_DIRS})
    find_program(OVPHYSX_UV_COMMAND
        NAMES uv uv.exe
        HINTS ${_UV_HINTS}
    )
    if(NOT OVPHYSX_UV_COMMAND)
        message(WARNING "uv not found; Python test scripts will fail. Install: https://docs.astral.sh/uv/")
    endif()
endif()

# Standardized output directory: _build/<platform>/<config>/
set(STANDARD_OUTPUT_DIR "${BUILD_PATH}/${PLATFORM_NAME}/${BUILD_TYPE_LOWER}")

# Resolve the external ovstage package used by SDK tests.
#
# Sets in the caller's scope:
#   OVPHYSX_OVSTAGE_ROOT           - the configured ovstage package root
#   OVPHYSX_OVSTAGE_RUNTIME_DIR    - directory holding libovstage.so / ovstage.dll
#   OVPHYSX_OVSTAGE_PYTHON_DIR     - parent of the importable ovstage python package,
#                                    or "" when the layout carries no python package
function(ovphysx_resolve_ovstage_paths)
    if(NOT OVSTAGE_DIR)
        set(_cache "${BUILD_PATH}/CMakeCache.txt")
        # file(STRINGS) is fatal on a missing file, which would pre-empt the
        # message below in artifact-consumer jobs that have no local _build.
        set(_line "")
        if(EXISTS "${_cache}")
            file(STRINGS "${_cache}" _line REGEX "^_OVPHYSX_OVSTAGE_DIR:INTERNAL=")
        endif()
        if(NOT _line)
            message(FATAL_ERROR "Pass OVSTAGE_DIR or configure ovphysx first.")
        endif()
        list(GET _line 0 _line_first)
        string(REGEX REPLACE "^[^=]+=" "" OVSTAGE_DIR "${_line_first}")
    endif()
    include("${PROJECT_ROOT}/ovruntime/cmake/OvstageDependency.cmake")
    set(OVPHYSX_OVSTAGE_ROOT "${OVSTAGE_DIR}" PARENT_SCOPE)
    set(OVPHYSX_OVSTAGE_RUNTIME_DIR "${OVSTAGE_LIBRARY_DIR}" PARENT_SCOPE)

    # The importable package sits directly under the root in the wheel layout and
    # under python/ in the packaged layout.
    set(_python_dir "")
    foreach(_candidate "${OVSTAGE_DIR}" "${OVSTAGE_DIR}/python")
        if(EXISTS "${_candidate}/ovstage/__init__.py")
            set(_python_dir "${_candidate}")
            break()
        endif()
    endforeach()
    set(OVPHYSX_OVSTAGE_PYTHON_DIR "${_python_dir}" PARENT_SCOPE)
endfunction()

# Generate Python _version.py from VERSION file into a specified directory.
# Only used by the wheel staging step; editable installs resolve version from
# the repo VERSION file via __init__.py fallback chain.
# Uses PEP440 format for Python compatibility (X.Y.Z-suffix -> X.Y.Z.suffix).
function(generate_python_version_file OUTPUT_DIR)
    set(_VERSION_PY_PATH "${OUTPUT_DIR}/_version.py")
    file(READ "${PROJECT_ROOT}/VERSION" _VERSION_CONTENT)
    string(STRIP "${_VERSION_CONTENT}" _VERSION_CONTENT)
    # Convert to PEP 440 format for Python compatibility
    semver_to_pep440("${_VERSION_CONTENT}")
    append_branch_local_version("${PEP440_VERSION}" "${PROJECT_ROOT}" PEP440_WITH_LOCAL)
    file(WRITE "${_VERSION_PY_PATH}" "# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.\n# SPDX-License-Identifier: BSD-3-Clause\n#\n# Auto-generated by cmake from VERSION file\n__version__ = \"${PEP440_WITH_LOCAL}\"\n")
    message(STATUS "Generated ${_VERSION_PY_PATH} with version ${PEP440_WITH_LOCAL}")
endfunction()

# Ensure uv-managed Python is available and return its path.
# Usage: ensure_uv_managed_python(<version> <out_var>)
function(ensure_uv_managed_python PY_VER OUT_VAR)
    execute_process(
        COMMAND "${OVPHYSX_UV_COMMAND}" python find --managed-python ${PY_VER}
        OUTPUT_VARIABLE _UV_PYTHON_PATH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_VARIABLE _UV_FIND_STDERR
        RESULT_VARIABLE _PYTHON_FIND_RESULT
    )

    set(_UV_MANAGED_OK FALSE)
    if(_PYTHON_FIND_RESULT EQUAL 0 AND _UV_PYTHON_PATH)
        if(_UV_PYTHON_PATH MATCHES "/uv/python/" OR
           _UV_PYTHON_PATH MATCHES "/\\.local/share/uv/" OR
           _UV_PYTHON_PATH MATCHES "/\\.cache/uv/" OR
           _UV_PYTHON_PATH MATCHES "/Library/Application Support/uv/" OR
           _UV_PYTHON_PATH MATCHES "\\\\uv\\\\python\\\\")
            set(_UV_MANAGED_OK TRUE)
        endif()
    endif()

    if(NOT _UV_MANAGED_OK)
        message(STATUS "Installing Python ${PY_VER} via uv...")
        execute_process(
            COMMAND "${OVPHYSX_UV_COMMAND}" python install --managed-python ${PY_VER}
            OUTPUT_QUIET
            ERROR_VARIABLE _UV_INSTALL_STDERR
            RESULT_VARIABLE _UV_INSTALL_RESULT
        )
        if(NOT _UV_INSTALL_RESULT EQUAL 0)
            message(FATAL_ERROR "Failed to install Python ${PY_VER} via uv (exit code: ${_UV_INSTALL_RESULT})\n"
                                "Error: ${_UV_INSTALL_STDERR}")
        endif()
        execute_process(
            COMMAND "${OVPHYSX_UV_COMMAND}" python find --managed-python ${PY_VER}
            OUTPUT_VARIABLE _UV_PYTHON_PATH
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_VARIABLE _UV_FIND_STDERR
            RESULT_VARIABLE _PYTHON_FIND_RESULT
        )
        if(NOT _PYTHON_FIND_RESULT EQUAL 0 OR NOT _UV_PYTHON_PATH)
            message(FATAL_ERROR "Failed to locate Python ${PY_VER} after install (exit code: ${_PYTHON_FIND_RESULT})\n"
                                "Error: ${_UV_FIND_STDERR}")
        endif()
        if(NOT (_UV_PYTHON_PATH MATCHES "/uv/python/" OR
                _UV_PYTHON_PATH MATCHES "/\\.local/share/uv/" OR
                _UV_PYTHON_PATH MATCHES "/\\.cache/uv/" OR
                _UV_PYTHON_PATH MATCHES "/Library/Application Support/uv/" OR
                _UV_PYTHON_PATH MATCHES "\\\\uv\\\\python\\\\"))
            message(FATAL_ERROR "uv python find ${PY_VER} resolved to system Python (${_UV_PYTHON_PATH}) after install.")
        endif()
    endif()

    message(STATUS "Python ${PY_VER}: ${_UV_PYTHON_PATH}")
    set(${OUT_VAR} "${_UV_PYTHON_PATH}" PARENT_SCOPE)
endfunction()

function(copy_filtered_extensions SOURCE_EXTS_DIR TARGET_EXT_CACHE DEPS_MANIFEST_PATH TARGET_PYTHON PACKAGE_DEPS_SCRIPT LABEL)
    if(NOT EXISTS "${SOURCE_EXTS_DIR}")
        message(FATAL_ERROR "Could not locate ${SOURCE_EXTS_DIR}")
    endif()
    if(NOT EXISTS "${DEPS_MANIFEST_PATH}")
        message(FATAL_ERROR "deps_manifest.toml not found at ${DEPS_MANIFEST_PATH}")
    endif()

    execute_process(
        COMMAND "${TARGET_PYTHON}" "${PACKAGE_DEPS_SCRIPT}" list-extensions
                --manifest ${DEPS_MANIFEST_PATH}
                --exts-dir ${SOURCE_EXTS_DIR}
                --format=cmake
        OUTPUT_VARIABLE FILTERED_EXTS_RAW
        ERROR_VARIABLE FILTERED_EXTS_ERR
        RESULT_VARIABLE FILTERED_EXTS_RESULT
    )
    if(NOT FILTERED_EXTS_RESULT STREQUAL "0")
        message(STATUS "FILTERED_EXTS_RAW: ${FILTERED_EXTS_RAW}")
        message(STATUS "FILTERED_EXTS_ERR: ${FILTERED_EXTS_ERR}")
        message(FATAL_ERROR "Failed to filter ${LABEL} via deps_manifest.toml (exit code: ${FILTERED_EXTS_RESULT})\n${FILTERED_EXTS_ERR}")
    endif()

    string(STRIP "${FILTERED_EXTS_RAW}" FILTERED_EXTS_RAW)
    set(FILTERED_EXTS "${FILTERED_EXTS_RAW}")
    if(NOT FILTERED_EXTS)
        message(STATUS "  No extensions matched deps_manifest.toml in ${SOURCE_EXTS_DIR}")
        return()
    endif()
    message(STATUS "  Extensions to copy from ${LABEL}: ${FILTERED_EXTS}")

    foreach(EXT_NAME ${FILTERED_EXTS})
        set(EXT_ITEM "${SOURCE_EXTS_DIR}/${EXT_NAME}")
        if(NOT IS_DIRECTORY "${EXT_ITEM}")
            message(FATAL_ERROR
                "Extension not found (expected directory): ${EXT_ITEM} "
                "(EXT_NAME='${EXT_NAME}', SOURCE_EXTS_DIR='${SOURCE_EXTS_DIR}')"
            )
        endif()

        if(WIN32)
            set(EXT_DST "${TARGET_EXT_CACHE}/${EXT_NAME}")
            execute_process(
                COMMAND robocopy "${EXT_ITEM}" "${EXT_DST}" /E /NFL /NDL /NJH /nc /ns /np
                RESULT_VARIABLE ROBOCOPY_RESULT
            )
            if(ROBOCOPY_RESULT GREATER 7)
                message(FATAL_ERROR "Failed to copy ${EXT_NAME} from ${LABEL} (robocopy exit code: ${ROBOCOPY_RESULT})")
            endif()
        else()
            file(COPY "${EXT_ITEM}" DESTINATION "${TARGET_EXT_CACHE}")
        endif()
        message(STATUS "  [OK] ${EXT_NAME} copied successfully to ${TARGET_EXT_CACHE}/${EXT_NAME}")
    endforeach()
endfunction()

# Strip unstripped ELF binaries under a root directory.
# Usage:
#   strip_unstripped_elf_binaries(<root_dir> <label>)
# - root_dir: directory tree to scan for *.so* files
# - label: short text used in status/error messages
function(strip_unstripped_elf_binaries ROOT_DIR LABEL)
    if(WIN32)
        message(STATUS "Skipping ELF stripping for ${LABEL} on Windows")
        return()
    endif()

    execute_process(
        COMMAND file --version
        OUTPUT_QUIET
        ERROR_QUIET
        RESULT_VARIABLE FILE_TOOL_RESULT
    )
    if(NOT FILE_TOOL_RESULT EQUAL 0)
        message(FATAL_ERROR
            "'file' not found -- required to detect unstripped ELF binaries in ${LABEL}.\n"
            "It is the OS file-type utility (libmagic), not CMake's file() command, "
            "and is absent from minimal container images.\n"
            "Install it: apt-get install file  (RPM distros: dnf install file)")
    endif()

    # Probed here rather than at first use: a missing strip only surfaces as an
    # empty per-file error further down, which says nothing about binutils.
    execute_process(
        COMMAND strip --version
        OUTPUT_QUIET
        ERROR_QUIET
        RESULT_VARIABLE STRIP_TOOL_RESULT
    )
    if(NOT STRIP_TOOL_RESULT EQUAL 0)
        message(FATAL_ERROR
            "strip not found -- required to strip ELF binaries in ${LABEL}.\n"
            "Install binutils: apt-get install binutils")
    endif()

    message(STATUS "Stripping symbols from unstripped libraries in ${LABEL}...")
    file(GLOB_RECURSE ELF_CANDIDATES "${ROOT_DIR}/*.so*")
    set(FILES_TO_STRIP "")
    set(FILE_INSPECTION_FAILURES "")
    set(ELF_FILES_COUNT 0)

    foreach(FILE_PATH ${ELF_CANDIDATES})
        if(IS_DIRECTORY "${FILE_PATH}")
            continue()
        endif()

        execute_process(
            COMMAND file -b "${FILE_PATH}"
            OUTPUT_VARIABLE FILE_DESC
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_VARIABLE FILE_DESC_ERROR
            RESULT_VARIABLE FILE_DESC_RESULT
        )
        if(NOT FILE_DESC_RESULT EQUAL 0)
            string(STRIP "${FILE_DESC_ERROR}" FILE_DESC_ERROR_STRIPPED)
            if(FILE_DESC_ERROR_STRIPPED STREQUAL "")
                set(FILE_DESC_ERROR_STRIPPED "<no stderr output>")
            endif()
            list(APPEND FILE_INSPECTION_FAILURES "${FILE_PATH}: ${FILE_DESC_ERROR_STRIPPED}")
            continue()
        endif()

        if(FILE_DESC MATCHES "ELF")
            math(EXPR ELF_FILES_COUNT "${ELF_FILES_COUNT} + 1")
            if(FILE_DESC MATCHES "not stripped")
                list(APPEND FILES_TO_STRIP "${FILE_PATH}")
            endif()
        endif()
    endforeach()

    list(LENGTH FILE_INSPECTION_FAILURES FILE_INSPECTION_FAILURE_COUNT)
    if(FILE_INSPECTION_FAILURE_COUNT GREATER 0)
        message(STATUS "Failed to inspect ${FILE_INSPECTION_FAILURE_COUNT} files with 'file -b' in ${LABEL}:")
        foreach(FAILED_INSPECTION ${FILE_INSPECTION_FAILURES})
            message(STATUS "  - ${FAILED_INSPECTION}")
        endforeach()
        message(FATAL_ERROR "Cannot continue stripping in ${LABEL}: file inspection failed")
    endif()

    list(LENGTH FILES_TO_STRIP STRIP_COUNT)
    if(STRIP_COUNT GREATER 0)
        message(STATUS "Stripping ${STRIP_COUNT} libraries (out of ${ELF_FILES_COUNT} ELF libraries) in ${LABEL}")
        set(STRIP_FAILURES "")
        foreach(SO_FILE ${FILES_TO_STRIP})
            execute_process(
                COMMAND strip --strip-unneeded "${SO_FILE}"
                OUTPUT_QUIET
                ERROR_VARIABLE STRIP_ERROR
                RESULT_VARIABLE STRIP_RESULT
            )
            if(NOT STRIP_RESULT EQUAL 0)
                string(STRIP "${STRIP_ERROR}" STRIP_ERROR_STRIPPED)
                if(STRIP_ERROR_STRIPPED STREQUAL "")
                    set(STRIP_ERROR_STRIPPED "<no stderr output>")
                endif()
                list(APPEND STRIP_FAILURES "${SO_FILE}: ${STRIP_ERROR_STRIPPED}")
            endif()
        endforeach()

        list(LENGTH STRIP_FAILURES STRIP_FAILURE_COUNT)
        if(STRIP_FAILURE_COUNT GREATER 0)
            message(STATUS "Failed to strip ${STRIP_FAILURE_COUNT} libraries in ${LABEL}:")
            foreach(FAILED_STRIP ${STRIP_FAILURES})
                message(STATUS "  - ${FAILED_STRIP}")
            endforeach()
            message(FATAL_ERROR "Cannot continue: strip failed in ${LABEL}")
        endif()
    else()
        message(STATUS "All ELF libraries are already stripped (${ELF_FILES_COUNT} checked) in ${LABEL}")
    endif()
endfunction()

# ============================================================================
# ABI baseline enforcement for shipped binaries (Linux only).
# ============================================================================
# We target Ubuntu 22.04 LTS (glibc 2.35, GCC 11 / GLIBCXX_3.4.30) as the
# minimum supported platform.  This is the oldest Ubuntu LTS still in standard
# support, and matches the CI Docker images used by both ovphysx and PhysX SDK.
# Pinning to this baseline means the SDK tarball and wheel work out-of-the-box
# on Ubuntu 22.04+ and any distro with glibc >= 2.35 (RHEL 9, Debian 12, etc.)
# without requiring users to upgrade system libraries.
# All .so files in release artifacts must not require newer versions.
set(GLIBC_BASELINE   "2.35")
set(GLIBCXX_BASELINE "3.4.30")

# Verify that no ELF binary under ROOT_DIR requires a GLIBC or GLIBCXX
# version above the configured baseline.
#
# Uses readelf --version-info to inspect the .gnu.version_r (required)
# section.  Ignores the .gnu.version_d (defined/provided) section and
# GLIBC_PRIVATE entries.
#
# Usage:
#   verify_glibc_baseline(<root_dir> <label>)
#
# Controlled by:
#   SKIP_GLIBC_CHECK  -- set to ON to skip (useful for local dev on newer OS).
#   May be passed as a -D variable or as an environment variable; the env-var
#   form is what lets validate_all.cmake forward the flag through its
#   subprocess chain (cmake -P build.cmake -> cmake --build --target
#   validate_all -> install_sdk -> cmake -P install.cmake), since the inner
#   custom-target invocations of install.cmake do not see -D flags from the
#   top.
function(verify_glibc_baseline ROOT_DIR LABEL)
    if(WIN32)
        message(STATUS "Skipping glibc baseline check for ${LABEL} on Windows")
        return()
    endif()
    if(SKIP_GLIBC_CHECK)
        message(STATUS "Skipping glibc baseline check for ${LABEL} (SKIP_GLIBC_CHECK set)")
        return()
    elseif(DEFINED ENV{SKIP_GLIBC_CHECK} AND "$ENV{SKIP_GLIBC_CHECK}")
        # Distinct message so a stale shell env (e.g. left over from a previous
        # debug session) doesn't silently skip the check on a direct cmake -P run.
        message(STATUS "Skipping glibc baseline check for ${LABEL} (SKIP_GLIBC_CHECK from environment)")
        return()
    endif()

    # Verify readelf is available (binutils)
    execute_process(
        COMMAND readelf --version
        OUTPUT_QUIET
        ERROR_QUIET
        RESULT_VARIABLE _READELF_RESULT
    )
    if(NOT _READELF_RESULT EQUAL 0)
        message(FATAL_ERROR
            "readelf not found -- required for glibc baseline verification.\n"
            "Install binutils: apt-get install binutils")
    endif()

    message(STATUS "Verifying ABI baseline for ${LABEL} (GLIBC <= ${GLIBC_BASELINE}, GLIBCXX <= ${GLIBCXX_BASELINE})...")
    file(GLOB_RECURSE _ELF_CANDIDATES "${ROOT_DIR}/*.so*")
    set(_VIOLATIONS "")
    set(_CHECKED 0)

    foreach(_FILE ${_ELF_CANDIDATES})
        if(IS_DIRECTORY "${_FILE}")
            continue()
        endif()

        # Use readelf -h as a quick ELF check: non-ELF files (e.g. text stubs)
        # cause readelf to exit non-zero, so we skip them.  This also avoids a
        # dependency on the 'file' command for the ELF-detection step.
        execute_process(
            COMMAND readelf -h "${_FILE}"
            OUTPUT_QUIET
            ERROR_QUIET
            RESULT_VARIABLE _HDR_RESULT
        )
        if(NOT _HDR_RESULT EQUAL 0)
            continue()
        endif()

        math(EXPR _CHECKED "${_CHECKED} + 1")

        # Extract required version tags from .gnu.version_r section.
        # readelf --version-info prints both .gnu.version_d (defined) and
        # .gnu.version_r (required).  We only want the required section.
        # Force LC_ALL=C so section headers are always in English.
        set(ENV{LC_ALL} "C")
        execute_process(
            COMMAND readelf --version-info "${_FILE}"
            OUTPUT_VARIABLE _VI_OUTPUT
            ERROR_QUIET
            RESULT_VARIABLE _VI_RESULT
        )
        if(NOT _VI_RESULT EQUAL 0)
            continue()
        endif()

        # Parse only the "Version needs" (required) section, not
        # "Version definition" (provided) or "Version symbols".
        set(_IN_NEEDS FALSE)
        string(REPLACE "\n" ";" _VI_LINES "${_VI_OUTPUT}")
        foreach(_LINE ${_VI_LINES})
            if(_LINE MATCHES "Version needs section")
                set(_IN_NEEDS TRUE)
            elseif(_LINE MATCHES "Version definition section" OR
                   _LINE MATCHES "Version symbols section")
                set(_IN_NEEDS FALSE)
            endif()
            if(NOT _IN_NEEDS)
                continue()
            endif()

            # Match "Name: GLIBC_X.Y.Z" or "Name: GLIBCXX_X.Y.Z".
            # The numeric-only regex naturally excludes GLIBC_PRIVATE.
            if(_LINE MATCHES "Name: GLIBC_([0-9]+\\.[0-9]+(\\.[0-9]+)?)")
                set(_VER "${CMAKE_MATCH_1}")
                if("${_VER}" VERSION_GREATER "${GLIBC_BASELINE}")
                    file(RELATIVE_PATH _REL "${ROOT_DIR}" "${_FILE}")
                    list(APPEND _VIOLATIONS "${_REL}: requires GLIBC_${_VER} (baseline: ${GLIBC_BASELINE})")
                endif()
            elseif(_LINE MATCHES "Name: GLIBCXX_([0-9]+\\.[0-9]+(\\.[0-9]+)?)")
                set(_VER "${CMAKE_MATCH_1}")
                if("${_VER}" VERSION_GREATER "${GLIBCXX_BASELINE}")
                    file(RELATIVE_PATH _REL "${ROOT_DIR}" "${_FILE}")
                    list(APPEND _VIOLATIONS "${_REL}: requires GLIBCXX_${_VER} (baseline: ${GLIBCXX_BASELINE})")
                endif()
            endif()
        endforeach()
    endforeach()

    list(LENGTH _VIOLATIONS _VIOLATION_COUNT)
    if(_VIOLATION_COUNT GREATER 0)
        list(REMOVE_DUPLICATES _VIOLATIONS)
        list(LENGTH _VIOLATIONS _VIOLATION_COUNT)
        message(STATUS "")
        message(STATUS "ABI BASELINE VIOLATIONS in ${LABEL} (${_VIOLATION_COUNT} issues):")
        foreach(_V ${_VIOLATIONS})
            message(STATUS "  - ${_V}")
        endforeach()
        message(STATUS "")
        message(FATAL_ERROR
            "ABI baseline check failed for ${LABEL}.\n"
            "  ${_VIOLATION_COUNT} binary(ies) require a newer glibc/libstdc++ than the target baseline.\n"
            "  Baseline: GLIBC_${GLIBC_BASELINE}, GLIBCXX_${GLIBCXX_BASELINE} (Ubuntu 22.04)\n"
            "  Build inside a container with glibc ${GLIBC_BASELINE} to fix.\n"
            "  To skip this check for local development: cmake -DSKIP_GLIBC_CHECK=ON -P ...")
    else()
        message(STATUS "ABI baseline OK: ${_CHECKED} ELF binaries checked, all within GLIBC_${GLIBC_BASELINE} / GLIBCXX_${GLIBCXX_BASELINE}")
    endif()
endfunction()
