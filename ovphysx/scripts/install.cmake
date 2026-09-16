# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-DOCS-001
# @covers AC-1
# @implements REQ-PACKAGING-USDFREE-001
# @covers AC-7

# ovphysx SDK installation script (cross-platform).
# Usage: cmake [options] -P scripts/install.cmake
#
# Prerequisites: Run scripts/build.cmake first to build the SDK
#
# Options (passed via -D flags):
#   -DBUILD_TYPE=Debug|Release    Build type (default: Release)
#
# Examples:
#   cmake -P scripts/install.cmake
#   cmake -DBUILD_TYPE=Debug -P scripts/install.cmake
#   cmake -DSKIP_GLIBC_CHECK=ON -P scripts/install.cmake
#
# What this does:
#   - Performs the CMake install (copies libraries, headers, config files to _install/)
#   - Packages filtered native dependencies using deps_manifest.toml
#   - Strips debug symbols on Linux (only files that are still unstripped)
#   - Verifies the glibc/libstdc++ baseline
#   - Required for: C++ sample tests (validates find_package() integration)
#   - Typical use: CI, pre-release validation, or when testing the CMake package config

cmake_minimum_required(VERSION 3.16)

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
get_filename_component(PROJECT_ROOT "${SCRIPT_DIR}/.." ABSOLUTE)

set(_OVPHYSX_INSTALL_RUNTIME_DEPS_EXPLICIT FALSE)
if(DEFINED OVPHYSX_USE_RELEASE_RUNTIME_DEPS
   OR (DEFINED ENV{OVPHYSX_USE_RELEASE_RUNTIME_DEPS} AND NOT "$ENV{OVPHYSX_USE_RELEASE_RUNTIME_DEPS}" STREQUAL ""))
    set(_OVPHYSX_INSTALL_RUNTIME_DEPS_EXPLICIT TRUE)
endif()

include("${SCRIPT_DIR}/build_common.cmake")
include("${SCRIPT_DIR}/crossplatform_helpers.cmake")
# The target-deps Python (3.12) provides tomllib for the packaging scripts.
if(WIN32)
    set(TARGET_PYTHON "${PROJECT_ROOT}/_build/target-deps/python/python.exe")
    if(DEFINED ENV{SystemRoot})
        set(_OVPHYSX_POWERSHELL_DIR "$ENV{SystemRoot}/System32/WindowsPowerShell/v1.0")
    else()
        set(_OVPHYSX_POWERSHELL_DIR "C:/Windows/System32/WindowsPowerShell/v1.0")
    endif()
    if(EXISTS "${_OVPHYSX_POWERSHELL_DIR}/powershell.exe")
        file(TO_CMAKE_PATH "$ENV{PATH}" _OVPHYSX_CURRENT_PATH)
        if(NOT _OVPHYSX_CURRENT_PATH MATCHES "(^|;)${_OVPHYSX_POWERSHELL_DIR}($|;)")
            set(ENV{PATH} "${_OVPHYSX_POWERSHELL_DIR};$ENV{PATH}")
        endif()
    endif()
else()
    set(TARGET_PYTHON "${PROJECT_ROOT}/_build/target-deps/python/bin/python3")
endif()

if(NOT EXISTS "${TARGET_PYTHON}")
    message(FATAL_ERROR "Target Python not found at: ${TARGET_PYTHON}\n"
                        "Run scripts/fetch_deps.bat (Windows) or scripts/fetch_deps.sh (Linux) first.")
endif()

if(NOT DEFINED BUILD_DIR)
    set(BUILD_DIR "_build")
endif()

set(BUILD_PATH "${PROJECT_ROOT}/${BUILD_DIR}")

if(NOT _OVPHYSX_INSTALL_RUNTIME_DEPS_EXPLICIT AND EXISTS "${BUILD_PATH}/CMakeCache.txt")
    file(STRINGS "${BUILD_PATH}/CMakeCache.txt" _OVPHYSX_CACHED_RELEASE_RUNTIME_DEPS
        REGEX "^OVPHYSX_USE_RELEASE_RUNTIME_DEPS:(BOOL|UNINITIALIZED)=")
    if(_OVPHYSX_CACHED_RELEASE_RUNTIME_DEPS)
        string(REGEX REPLACE "^[^=]*=" "" OVPHYSX_USE_RELEASE_RUNTIME_DEPS
            "${_OVPHYSX_CACHED_RELEASE_RUNTIME_DEPS}")
        string(TOUPPER "${OVPHYSX_USE_RELEASE_RUNTIME_DEPS}" _OVPHYSX_RELEASE_RUNTIME_DEPS_VALUE)
        if(_OVPHYSX_RELEASE_RUNTIME_DEPS_VALUE MATCHES "^(ON|TRUE|YES|1)$")
            set(OVPHYSX_RUNTIME_DEPS_CONFIG "release")
        elseif(_OVPHYSX_RELEASE_RUNTIME_DEPS_VALUE MATCHES "^(OFF|FALSE|NO|0)$")
            if(BUILD_TYPE_LOWER STREQUAL "debug")
                set(OVPHYSX_RUNTIME_DEPS_CONFIG "debug")
            else()
                set(OVPHYSX_RUNTIME_DEPS_CONFIG "release")
            endif()
        else()
            message(FATAL_ERROR
                "OVPHYSX_USE_RELEASE_RUNTIME_DEPS must be ON or OFF; got '${OVPHYSX_USE_RELEASE_RUNTIME_DEPS}'")
        endif()
        unset(_OVPHYSX_RELEASE_RUNTIME_DEPS_VALUE)
    endif()
    unset(_OVPHYSX_CACHED_RELEASE_RUNTIME_DEPS)
endif()
unset(_OVPHYSX_INSTALL_RUNTIME_DEPS_EXPLICIT)

# build_common.cmake validates the command-line/default selection before the
# configured build cache is imported above. Validate again after that import so
# a cached Debug/OFF build cannot silently select a true-Debug dependency tree.
if(BUILD_TYPE_LOWER STREQUAL "debug" AND NOT OVPHYSX_USE_RELEASE_RUNTIME_DEPS)
    message(FATAL_ERROR
        "True-Debug runtime dependencies are unsupported because the published "
        "OVStage package supplies a Release-only resolver/client runtime. "
        "Use -DOVPHYSX_USE_RELEASE_RUNTIME_DEPS=ON for Debug compilation.")
endif()

# Preserve the schema source selected by build.cmake when install.cmake is run
# later as a separate step. A stamp means package the local schemas/physx build.
# No stamp means package the prebuilt namespaced usd_ext_physics package.
if(NOT DEFINED OVPHYSX_DEV_SCHEMA)
    set(_DEV_SCHEMA_STAMP "${BUILD_PATH}/ovphysx_dev_schema.stamp")
    if(EXISTS "${_DEV_SCHEMA_STAMP}")
        set(OVPHYSX_DEV_SCHEMA ON)
    else()
        set(OVPHYSX_DEV_SCHEMA OFF)
    endif()
endif()
if(NOT "${OVPHYSX_DEV_SCHEMA}" MATCHES "^(ON|OFF|TRUE|FALSE|YES|NO|1|0)$")
    message(FATAL_ERROR
        "Invalid OVPHYSX_DEV_SCHEMA value: '${OVPHYSX_DEV_SCHEMA}'. "
        "Expected one of ON, OFF, TRUE, FALSE, YES, NO, 1, 0."
    )
endif()

if(NOT EXISTS "${BUILD_PATH}")
    message(FATAL_ERROR "Build directory not found: ${BUILD_PATH}\nPlease run: cmake -P scripts/build.cmake")
endif()

message(STATUS "ovphysx SDK Installation")
message(STATUS "==========================")
message(STATUS "Project root: ${PROJECT_ROOT}")
message(STATUS "Build type: ${BUILD_TYPE}")
message(STATUS "Build directory: ${BUILD_DIR}")
message(STATUS "Platform: ${CMAKE_HOST_SYSTEM_NAME}")
message(STATUS "USD: none shipped (ovstage brings its own runtime; codeless schemas staged as data)")
message(STATUS "Static Carbonite: ON")
message(STATUS "")

# repo_licensing is internal-only (no public package exists) and is removed
# from the public source drop's repo-deps manifest. Open-source builds skip
# license gathering and produce no ovphysx-LICENSES.zip. Official artifacts
# with bundled third-party licenses come from internal builds.
file(READ "${PROJECT_ROOT}/deps/repo-deps.packman.xml" _OVPHYSX_REPO_DEPS_CONTENT)
if(_OVPHYSX_REPO_DEPS_CONTENT MATCHES "repo_licensing")
    set(OVPHYSX_LICENSING_AVAILABLE TRUE)
else()
    set(OVPHYSX_LICENSING_AVAILABLE FALSE)
endif()
unset(_OVPHYSX_REPO_DEPS_CONTENT)

if(OVPHYSX_LICENSING_AVAILABLE)

message(STATUS "License gathering")
set(OVPHYSX_OVRUNTIME_LICENSE_IMPORT "deps/ovruntime-deps-import.packman.xml")
set(OVPHYSX_CARB_LICENSE_IMPORT "deps/carb-sdk-deps-import.packman.xml")
set(OVRUNTIME_DEPS_LICENSE_IMPORT "ovruntime/deps/ovruntime-deps.packman.xml")
set(OVRUNTIME_LICENSE_IMPORT "ovruntime/deps/ovruntime-deps-import.packman.xml")
set(OVRUNTIME_USD_LICENSE_IMPORT "ovruntime/deps/usd-deps.packman.xml")
set(OVRUNTIME_CARB_LICENSE_IMPORT "ovruntime/deps/carb-sdk-deps-import.packman.xml")
set(OVRUNTIME_SCHEMA_LICENSE_IMPORT "ovruntime/deps/schema-deps.packman.xml")
execute_process(
    COMMAND "${PROJECT_ROOT}/repo${SCRIPT_SUFFIX}"
            --set-token abi:2_35
            # usd_ver is a placeholder. It only builds the version string of a
            # filtered-out ovruntime_deps entry that is never resolved, and does not track USD.
            --set-token usd_ver:unused
            licensing gather -d . --fail
            --platform ${PLATFORM_NAME}
            --config ${OVPHYSX_RUNTIME_DEPS_CONFIG}
            -p ${OVPHYSX_OVRUNTIME_LICENSE_IMPORT}
               ${OVPHYSX_CARB_LICENSE_IMPORT}
               deps/carb-sdk-static.packman.xml
               deps/repo-deps.packman.xml
               ovruntime/deps/repo-deps.packman.xml
               ovruntime/deps/target-deps.packman.xml
               ${OVRUNTIME_DEPS_LICENSE_IMPORT}
               ${OVRUNTIME_LICENSE_IMPORT}
               ${OVRUNTIME_USD_LICENSE_IMPORT}
               ${OVRUNTIME_CARB_LICENSE_IMPORT}
               ${OVRUNTIME_SCHEMA_LICENSE_IMPORT}
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    RESULT_VARIABLE LICENSE_GATHER_RESULT
)
if(NOT LICENSE_GATHER_RESULT STREQUAL "0")
    message(FATAL_ERROR "Failed to gather licenses (exit code: ${LICENSE_GATHER_RESULT})")
endif()
set(LICENSE_OUTPUT "${PROJECT_ROOT}/_build/PACKAGE-LICENSES/ovphysx-LICENSES.zip")
if(NOT EXISTS "${LICENSE_OUTPUT}")
    message(FATAL_ERROR "License gathering did not produce expected output: ${LICENSE_OUTPUT}")
endif()
message(STATUS "  [OK] Licenses gathered successfully")

else()
    message(STATUS "License gathering skipped: repo_licensing is not in deps/repo-deps.packman.xml (public source drop)")
endif()

# ============================================================================
# Installation Logic
# ============================================================================

message(STATUS "Performing full installation with CMake packaging")

message(STATUS "Placing a local installation of the package in ${PROJECT_ROOT}/_install")
# CMake install is additive. Start from a clean generated tree so files from an
# older SDK that bundled OVStage cannot survive into this package.
file(REMOVE_RECURSE "${PROJECT_ROOT}/_install")
set(_OVSTAGE_NOTICES "${PROJECT_ROOT}/_install/ovstage-THIRD-PARTY-NOTICES.txt")
file(REMOVE "${_OVSTAGE_NOTICES}")
execute_process(
    COMMAND ${CMAKE_COMMAND} --install . --config ${BUILD_TYPE} --prefix ${PROJECT_ROOT}/_install
    WORKING_DIRECTORY "${BUILD_PATH}"
    RESULT_VARIABLE INSTALL_RESULT
)
if(NOT INSTALL_RESULT EQUAL 0)
    message(FATAL_ERROR "Installation failed (exit code: ${INSTALL_RESULT})")
endif()
if(NOT EXISTS "${_OVSTAGE_NOTICES}")
    message(FATAL_ERROR "Required ovstage third-party notices were not installed: ${_OVSTAGE_NOTICES}")
endif()


# Preprocess and copy public portable docs, excluding internal docs.
# Docs land at _install/docs/ (same relative structure as source) so that
# SDK_README.md links (docs/...) resolve correctly from _install/.
include("${SCRIPT_DIR}/preprocess_docs.cmake")
preprocess_public_docs(
    "${TARGET_PYTHON}" "${PROJECT_ROOT}"
    "${PROJECT_ROOT}/docs" "${PROJECT_ROOT}/_install/docs"
)

# SDK_README.md becomes _install/README.md, the consumer-facing entry point.
# Its links use the docs/ prefix, matching the installed layout.
execute_process(
    COMMAND "${TARGET_PYTHON}" "${SCRIPT_DIR}/preprocess_markdown.py"
        "${PROJECT_ROOT}/SDK_README.md" "${PROJECT_ROOT}/_install/README.md"
        --project-root "${PROJECT_ROOT}"
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    RESULT_VARIABLE _SDK_README_RESULT
)
if(NOT _SDK_README_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to preprocess SDK_README.md")
endif()

# The skills index and skills ship with the SDK. AGENTS.md is a monorepo
# developer doc and is intentionally not shipped.
if(EXISTS "${PROJECT_ROOT}/SKILLS.md")
    copy_file_if_different("${PROJECT_ROOT}/SKILLS.md" "${PROJECT_ROOT}/_install/SKILLS.md")
    message(STATUS "  Copied SKILLS.md to _install/")
else()
    message(FATAL_ERROR "SKILLS.md not found at ${PROJECT_ROOT}/SKILLS.md")
endif()
if(EXISTS "${PROJECT_ROOT}/skills")
    copy_tree_if_different("${PROJECT_ROOT}/skills" "${PROJECT_ROOT}/_install/skills")
    message(STATUS "  Copied skills/ to _install/")
else()
    message(FATAL_ERROR "skills/ directory not found at ${PROJECT_ROOT}/skills")
endif()

# Public sample source code and data ship with the SDK. This avoids duplicating
# sample code under skills/ and ships what CI already tests.
set(SAMPLES_DST "${PROJECT_ROOT}/_install/samples")
file(MAKE_DIRECTORY "${SAMPLES_DST}")
# Python samples do not ship in the C/C++ SDK artifact. A stale directory from
# a previous install would confuse SDK consumers.
if(EXISTS "${SAMPLES_DST}/python_samples")
    file(REMOVE_RECURSE "${SAMPLES_DST}/python_samples")
endif()

if(EXISTS "${PROJECT_ROOT}/tests/c_samples")
    copy_tree_if_different("${PROJECT_ROOT}/tests/c_samples" "${SAMPLES_DST}/c_samples" EXCLUDE_DIRS _build)
    message(STATUS "  Copied tests/c_samples -> _install/samples/c_samples/")
else()
    message(FATAL_ERROR "tests/c_samples not found at ${PROJECT_ROOT}/tests/c_samples")
endif()

if(EXISTS "${PROJECT_ROOT}/tests/data")
    copy_tree_if_different("${PROJECT_ROOT}/tests/data" "${SAMPLES_DST}/data")
    message(STATUS "  Copied tests/data -> _install/samples/data/")
else()
    message(FATAL_ERROR "tests/data not found at ${PROJECT_ROOT}/tests/data")
endif()

# Package dependencies into flat _install/plugins/ structure.
# Sources: ovruntime install, ovphysx target-deps, ovruntime_deps via omni_physics.
message(STATUS "Packaging dependencies...")

# ovruntime output follows the NvidiaBuildOptions convention PX_OUTPUT_LIB_DIR/<config_lower>.
set(OVRUNTIME_INSTALL_DIR "${BUILD_PATH}/${BUILD_TYPE_LOWER}")
if(NOT EXISTS "${OVRUNTIME_INSTALL_DIR}" AND BUILD_TYPE_LOWER STREQUAL "release")
    # devphysx remaps Release to checked.
    set(OVRUNTIME_INSTALL_DIR "${BUILD_PATH}/checked")
endif()
if(NOT EXISTS "${OVRUNTIME_INSTALL_DIR}")
    message(FATAL_ERROR "ovruntime output not found in ${BUILD_PATH}/${BUILD_TYPE_LOWER}.\n"
                        "Run ./build.sh first.")
endif()

# package_deps must use the exact OVStage root selected by the configured build.
# A sibling target-deps path is never guessed here, because an explicit OVSTAGE_DIR
# override must remain the provider for every packaged runtime file and notice.
file(STRINGS "${BUILD_PATH}/CMakeCache.txt" _OVPHYSX_OVSTAGE_DIR_CACHE_LINE
    REGEX "^_OVPHYSX_OVSTAGE_DIR:INTERNAL=")
if(NOT _OVPHYSX_OVSTAGE_DIR_CACHE_LINE)
    message(FATAL_ERROR
        "Configured OVStage root is missing from ${BUILD_PATH}/CMakeCache.txt. "
        "Reconfigure ovphysx before installing.")
endif()
list(GET _OVPHYSX_OVSTAGE_DIR_CACHE_LINE 0 _OVPHYSX_OVSTAGE_DIR_CACHE_LINE_FIRST)
string(REGEX REPLACE "^[^=]+=" "" _OVPHYSX_RESOLVED_OVSTAGE_DIR
    "${_OVPHYSX_OVSTAGE_DIR_CACHE_LINE_FIRST}")
if(NOT IS_DIRECTORY "${_OVPHYSX_RESOLVED_OVSTAGE_DIR}")
    message(FATAL_ERROR
        "Configured OVStage root no longer exists: ${_OVPHYSX_RESOLVED_OVSTAGE_DIR}")
endif()
file(STRINGS "${BUILD_PATH}/CMakeCache.txt" _OVPHYSX_OVSTAGE_RUNTIME_DIR_CACHE_LINE
    REGEX "^_OVPHYSX_OVSTAGE_RUNTIME_DIR:INTERNAL=")
if(NOT _OVPHYSX_OVSTAGE_RUNTIME_DIR_CACHE_LINE)
    message(FATAL_ERROR
        "Configured OVStage runtime directory is missing from ${BUILD_PATH}/CMakeCache.txt. "
        "Reconfigure ovphysx before installing.")
endif()
list(GET _OVPHYSX_OVSTAGE_RUNTIME_DIR_CACHE_LINE 0
    _OVPHYSX_OVSTAGE_RUNTIME_DIR_CACHE_LINE_FIRST)
string(REGEX REPLACE "^[^=]+=" "" _OVPHYSX_RESOLVED_OVSTAGE_RUNTIME_DIR
    "${_OVPHYSX_OVSTAGE_RUNTIME_DIR_CACHE_LINE_FIRST}")
if(NOT IS_DIRECTORY "${_OVPHYSX_RESOLVED_OVSTAGE_RUNTIME_DIR}/plugins")
    message(FATAL_ERROR
        "Configured OVStage runtime plugin directory no longer exists: "
        "${_OVPHYSX_RESOLVED_OVSTAGE_RUNTIME_DIR}/plugins")
endif()
message(STATUS
    "Packaging runtime files from configured OVStage root: ${_OVPHYSX_RESOLVED_OVSTAGE_DIR}")
message(STATUS
    "Packaging runtime files from configured OVStage runtime: ${_OVPHYSX_RESOLVED_OVSTAGE_RUNTIME_DIR}")

set(_PACKAGE_DEPS_ARGS
    "${TARGET_PYTHON}" "${SCRIPT_DIR}/package_deps.py"
    --build-dir=${BUILD_PATH}
    --install-dir=${PROJECT_ROOT}/_install
    --ovruntime-install-dir=${OVRUNTIME_INSTALL_DIR}
    --ovstage-dir=${_OVPHYSX_RESOLVED_OVSTAGE_DIR}
    --ovstage-runtime-dir=${_OVPHYSX_RESOLVED_OVSTAGE_RUNTIME_DIR}
    --config=${OVPHYSX_RUNTIME_DEPS_CONFIG}
)
if(OVPHYSX_DEV_SCHEMA)
    list(APPEND _PACKAGE_DEPS_ARGS --devschema)
endif()
if(NOT OVPHYSX_LICENSING_AVAILABLE)
    # In the public source drop license gathering was skipped above, so there is
    # no ovphysx-LICENSES.zip for package_deps to merge OVStage notices into.
    list(APPEND _PACKAGE_DEPS_ARGS --no-license-archive)
endif()

execute_process(
    COMMAND ${_PACKAGE_DEPS_ARGS}
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    RESULT_VARIABLE PACKAGE_DEPS_RESULT
)

if(NOT PACKAGE_DEPS_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to package dependencies (exit code: ${PACKAGE_DEPS_RESULT})")
endif()

# The codeless PhysX USD schemas (schemas/physx/, OMPE-86833) are staged by
# package_deps.py from the selected schema package (packman usd_ext_physics by
# default, or a local build under --devschema). They are data only. The
# application registers them with the USD runtime it owns. See
# ovphysx_get_codeless_schema_root() and ovphysx.codeless_schema_root().

# Copy PDB debug symbol files for Debug builds on Windows.
# CMake install already handles ovphysx.pdb and ovphysx_internal.pdb (via CMakeLists.txt).
# The PDBs from the ovruntime output are copied as well so the dependency
# libraries are debuggable too.
if(WIN32 AND BUILD_TYPE STREQUAL "Debug")
    message(STATUS "Copying debug symbol files (PDBs) for Debug build...")
    file(GLOB _OVRUNTIME_PDBS "${OVRUNTIME_INSTALL_DIR}/*.pdb")
    foreach(_pdb IN LISTS _OVRUNTIME_PDBS)
        get_filename_component(_pdb_name "${_pdb}" NAME)
        file(COPY "${_pdb}" DESTINATION "${PROJECT_ROOT}/_install/plugins")
        message(STATUS "  Copied ${_pdb_name} -> _install/plugins/")
    endforeach()
endif()

# Strip any ELF binaries that are still unstripped. Debug builds keep their symbols.
if(NOT WIN32)
    if(BUILD_TYPE STREQUAL "Debug")
        message(STATUS "Skipping stripping for Debug build (preserving debug symbols)")
    else()
        strip_unstripped_elf_binaries("${PROJECT_ROOT}/_install" "_install tree")
    endif()
endif()

# Verify that all shipped binaries respect the glibc/libstdc++ baseline.
verify_glibc_baseline("${PROJECT_ROOT}/_install" "_install tree")

# Some shipped plugin libraries can carry stale build-tree RPATH/RUNPATH entries
# from copied prebuilt plugins (for example
# $ORIGIN:/.../ovphysx/ovruntime/_build/target-deps/usd/release/lib). The packaged
# SDK is flat under _install/plugins, so target-deps entries are removed before
# the final verifier runs.
#
# The exact OLD_RPATH is read from `readelf -d` first, and only libraries that
# contain target-deps are rewritten. This works for explicit --devschema builds,
# the prebuilt packman schema package, and the prebuilt ovruntime_deps plugin
# payload.
if(NOT WIN32)
    # RPATH_CHANGE needs the literal "$ORIGIN" token, not CMake variable expansion.
    set(_PLUGIN_ORIGIN_RPATH "\$ORIGIN")
    file(GLOB _PYLESS_SANITIZE_CANDIDATES
        "${PROJECT_ROOT}/_install/plugins/*.so"
        "${PROJECT_ROOT}/_install/plugins/*.so.*"
    )
    foreach(_plugin_lib IN LISTS _PYLESS_SANITIZE_CANDIDATES)
        if(EXISTS "${_plugin_lib}" AND NOT IS_SYMLINK "${_plugin_lib}")
            execute_process(
                COMMAND readelf -d "${_plugin_lib}"
                OUTPUT_VARIABLE _PLUGIN_READELF_OUTPUT
                ERROR_VARIABLE _PLUGIN_READELF_ERROR
                RESULT_VARIABLE _PLUGIN_READELF_RESULT
            )
            if(NOT _PLUGIN_READELF_RESULT EQUAL 0)
                message(FATAL_ERROR
                    "Failed to inspect plugin library RPATHs with readelf: ${_plugin_lib}\n"
                    "stderr: ${_PLUGIN_READELF_ERROR}"
                )
            endif()

            set(_PLUGIN_OLD_RPATH "")
            string(REPLACE "\n" ";" _PLUGIN_READELF_LINES "${_PLUGIN_READELF_OUTPUT}")
            foreach(_PLUGIN_READELF_LINE IN LISTS _PLUGIN_READELF_LINES)
                if(_PLUGIN_READELF_LINE MATCHES "\\((RPATH|RUNPATH)\\).*[[]([^]]+)[]]")
                    set(_PLUGIN_OLD_RPATH "${CMAKE_MATCH_2}")
                    break()
                endif()
            endforeach()

            if(_PLUGIN_OLD_RPATH MATCHES "(^|:).*/target-deps(/|:|$)" OR
               _PLUGIN_OLD_RPATH MATCHES "(^:|::|:$)")
                string(REPLACE ":" ";" _PLUGIN_OLD_RPATH_ENTRIES "${_PLUGIN_OLD_RPATH}")
                set(_PLUGIN_NEW_RPATH_ENTRIES "")
                foreach(_PLUGIN_RPATH_ENTRY IN LISTS _PLUGIN_OLD_RPATH_ENTRIES)
                    if(_PLUGIN_RPATH_ENTRY AND NOT _PLUGIN_RPATH_ENTRY MATCHES "/target-deps(/|$)")
                        list(APPEND _PLUGIN_NEW_RPATH_ENTRIES "${_PLUGIN_RPATH_ENTRY}")
                    endif()
                endforeach()
                if(NOT _PLUGIN_NEW_RPATH_ENTRIES)
                    list(APPEND _PLUGIN_NEW_RPATH_ENTRIES "${_PLUGIN_ORIGIN_RPATH}")
                endif()
                list(JOIN _PLUGIN_NEW_RPATH_ENTRIES ":" _PLUGIN_NEW_RPATH)

                message(STATUS "Sanitizing plugin RPATH: ${_plugin_lib}")
                file(RPATH_CHANGE
                    FILE "${_plugin_lib}"
                    OLD_RPATH "${_PLUGIN_OLD_RPATH}"
                    NEW_RPATH "${_PLUGIN_NEW_RPATH}"
                )

                execute_process(
                    COMMAND readelf -d "${_plugin_lib}"
                    OUTPUT_VARIABLE _PLUGIN_POST_READELF_OUTPUT
                    ERROR_VARIABLE _PLUGIN_POST_READELF_ERROR
                    RESULT_VARIABLE _PLUGIN_POST_READELF_RESULT
                )
                if(NOT _PLUGIN_POST_READELF_RESULT EQUAL 0)
                    message(FATAL_ERROR
                        "Failed to re-inspect sanitized plugin RPATHs with readelf: ${_plugin_lib}\n"
                        "stderr: ${_PLUGIN_POST_READELF_ERROR}"
                    )
                endif()
                if(_PLUGIN_POST_READELF_OUTPUT MATCHES "target-deps")
                    message(FATAL_ERROR
                        "RPATH sanitization did not remove the stale target-deps path from ${_plugin_lib}.\n"
                        "Old RPATH: ${_PLUGIN_OLD_RPATH}\n"
                        "New RPATH: ${_PLUGIN_NEW_RPATH}"
                    )
                endif()
            endif()
        endif()
    endforeach()
endif()

# Generate the SDK packaging lock files for CI artifacts.
# generate_packaging_lock.py is an internal CI-only artifact-drift tracker and
# is excluded from the open-source copy along with packaging_lock/. When it is
# absent, the entire lock mechanism is skipped. The OSS build does not need it.
set(SDK_LOCK_SENTINEL "${PROJECT_ROOT}/_build/.sdk_lock_failed")
if(EXISTS "${SDK_LOCK_SENTINEL}")
    file(REMOVE "${SDK_LOCK_SENTINEL}")
endif()

if(EXISTS "${SCRIPT_DIR}/generate_packaging_lock.py")
    message(STATUS "Generating SDK packaging lock files...")
    execute_process(
        COMMAND "${TARGET_PYTHON}" "${SCRIPT_DIR}/generate_packaging_lock.py"
                "--install-dir=${PROJECT_ROOT}/_install"
                "--output-dir=${PROJECT_ROOT}/packaging_lock"
                --artifact=sdk
        WORKING_DIRECTORY "${PROJECT_ROOT}"
        RESULT_VARIABLE LOCK_GEN_RESULT
    )
    if(NOT LOCK_GEN_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to generate SDK packaging lock files (exit code: ${LOCK_GEN_RESULT})")
    endif()

    # Honor SKIP_LOCK_CHECK from either the -D form or the environment. The env
    # form lets validate_all.cmake forward the flag through its subprocess chain.
    # See verify_glibc_baseline() in build_common.cmake for the same pattern.
    set(_OVPHYSX_SKIP_LOCK_CHECK FALSE)
    if(SKIP_LOCK_CHECK)
        set(_OVPHYSX_SKIP_LOCK_CHECK TRUE)
    elseif(DEFINED ENV{SKIP_LOCK_CHECK} AND "$ENV{SKIP_LOCK_CHECK}")
        set(_OVPHYSX_SKIP_LOCK_CHECK TRUE)
        # Distinct message so a stale shell env does not silently skip the check
        # on a direct cmake -P invocation.
        message(STATUS "SKIP_LOCK_CHECK picked up from environment (not -D)")
    endif()

    # Verify the SDK against the committed lock files. A mismatch writes a sentinel
    # and continues (WARNING, not FATAL_ERROR) so that build_wheel.cmake can still
    # run and generate the wheel _new.json. build_wheel.cmake emits the deferred
    # FATAL_ERROR after both _new.json files exist.
    if(NOT _OVPHYSX_SKIP_LOCK_CHECK AND NOT BUILD_TYPE STREQUAL "Debug")
        message(STATUS "Checking SDK packaging lock...")
        # devphysx/from-source builds produce checked, locally built libraries that are
        # legitimately larger than the prebuilt-release size baseline, so that mode skips
        # ONLY the size-delta check. The library set/layout is still enforced, and the
        # normal prebuilt-release path keeps the size check strict. BUILD_DEVPHYSX is the
        # CI variable that drives ./build.sh --devphysx. It is empty on prebuilt-release
        # builds and reaches this cmake -P step as an environment variable (like SKIP_LOCK_CHECK).
        set(_OVPHYSX_LOCK_SIZE_ARGS)
        if(DEFINED ENV{BUILD_DEVPHYSX} AND NOT "$ENV{BUILD_DEVPHYSX}" STREQUAL "")
            list(APPEND _OVPHYSX_LOCK_SIZE_ARGS "--ignore-size-deltas")
            message(STATUS "  devphysx build: relaxing the SDK lock size-delta check (set/layout still enforced)")
        endif()
        execute_process(
            COMMAND "${TARGET_PYTHON}" "${SCRIPT_DIR}/generate_packaging_lock.py"
                    "--install-dir=${PROJECT_ROOT}/_install"
                    "--output-dir=${PROJECT_ROOT}/packaging_lock"
                    --check
                    --artifact=sdk
                    ${_OVPHYSX_LOCK_SIZE_ARGS}
            WORKING_DIRECTORY "${PROJECT_ROOT}"
            RESULT_VARIABLE LOCK_CHECK_RESULT
        )
        if(NOT LOCK_CHECK_RESULT EQUAL 0)
            file(WRITE "${SDK_LOCK_SENTINEL}" "SDK packaging lock mismatch detected during install.cmake")
            message(WARNING
                "SDK packaging lock mismatch! The set of shipped libraries has changed.\n"
                "Continuing so that build_wheel.cmake can generate the wheel lock snapshot.\n"
                "The build will fail at the end of build_wheel.cmake with fix instructions.\n"
                "See the generate_packaging_lock.py output above for details."
            )
        endif()
    else()
        message(WARNING "SDK packaging lock check skipped (SKIP_LOCK_CHECK set or Debug build)")
    endif()
else()
    message(STATUS "SDK packaging lock skipped (generate_packaging_lock.py not present)")
endif()

# Verify the shipped package is py-less and USD-free: no bundled libpython, no
# OpenUSD library or USD dependency closure, no USD plugin registry, no Python
# schema bindings, no build-tree Python RPATHs, and a complete codeless schema
# tree under schemas/physx.
message(STATUS "Verifying py-less, USD-free package contents...")
execute_process(
    COMMAND "${TARGET_PYTHON}" "${SCRIPT_DIR}/verify_pyless_closure.py"
            "--dir=${PROJECT_ROOT}/_install" --require-schemas
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    RESULT_VARIABLE PYLESS_CHECK_RESULT
)
if(NOT PYLESS_CHECK_RESULT EQUAL 0)
    message(FATAL_ERROR
        "Package content verification failed for _install/.\n"
        "The native package must not contain libpython, any OpenUSD library or its\n"
        "dependency closure, a USD plugin registry, Python schema bindings, or\n"
        "build-tree Python RPATH entries, and must ship schemas/physx.\n"
        "See verify_pyless_closure.py output above for details."
    )
endif()

# The binary license is the Omniverse License for binary distributions.
set(BINARY_LICENSE "${PROJECT_ROOT}/licenses/LICENSE-binary.txt")
set(LICENSES_ZIP "${BUILD_PATH}/PACKAGE-LICENSES/ovphysx-LICENSES.zip")
if(NOT EXISTS "${BINARY_LICENSE}")
    message(FATAL_ERROR "Binary license not found: ${BINARY_LICENSE}")
endif()
copy_file_if_different("${BINARY_LICENSE}" "${PROJECT_ROOT}/_install/LICENSE.txt")
if(EXISTS "${LICENSES_ZIP}")
    copy_file_if_different("${LICENSES_ZIP}" "${PROJECT_ROOT}/_install/ovphysx-LICENSES.zip")
elseif(OVPHYSX_LICENSING_AVAILABLE)
    message(FATAL_ERROR "Third-party licenses archive not found: ${LICENSES_ZIP}")
else()
    message(STATUS "Third-party licenses archive not installed (license gathering skipped in the public source drop)")
endif()

message(STATUS "SDK installation complete: ${PROJECT_ROOT}/_install")
