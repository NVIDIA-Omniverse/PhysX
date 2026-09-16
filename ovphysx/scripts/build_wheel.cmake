# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PACKAGING-DOCS-001
# @covers AC-1
# @implements REQ-PACKAGING-USDFREE-001
# @covers AC-4 AC-5 AC-7

# ovphysx wheel build script.
# Stages the wheel contents in _build/python_wheel_staging/ and builds the
# Python wheel from that staging root, writing the output to _dist/.
# Prerequisites: Run scripts/install.cmake first (which creates _install/plugins/)
# Usage: cmake [options] -P scripts/build_wheel.cmake
# Options:
#   -DFORCE_REBUILD=ON : Force rebuild even if wheel is up-to-date
#
# Note: When using options, -D flags must come BEFORE -P:
#   cmake -DFORCE_REBUILD=ON -P scripts/build_wheel.cmake

cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED FORCE_REBUILD)
    set(FORCE_REBUILD OFF)
endif()

get_filename_component(SCRIPT_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)
get_filename_component(PROJECT_ROOT "${SCRIPT_DIR}/.." ABSOLUTE)

include("${SCRIPT_DIR}/build_common.cmake")
include("${SCRIPT_DIR}/crossplatform_helpers.cmake")
include("${SCRIPT_DIR}/preprocess_docs.cmake")
# The target-deps Python (3.12) provides tomllib for the packaging scripts.
if(WIN32)
    set(TARGET_PYTHON "${PROJECT_ROOT}/_build/target-deps/python/python.exe")
else()
    set(TARGET_PYTHON "${PROJECT_ROOT}/_build/target-deps/python/bin/python3")
endif()

message(STATUS "")
message(STATUS "========================================")
message(STATUS "ovphysx Wheel Build")
message(STATUS "========================================")
message(STATUS "Project root: ${PROJECT_ROOT}")
message(STATUS "Python command: ${TARGET_PYTHON}")

# The source Python package is read-only during the wheel build.
set(PYTHON_SRC_DIR "${PROJECT_ROOT}/python")
# The staging root is a complete PEP 517 packaging tree, regenerated on demand.
set(STAGING_ROOT "${PROJECT_ROOT}/_build/python_wheel_staging")
set(STAGING_WHEEL_PKG_DIR "${STAGING_ROOT}/ovphysx")
set(DIST_DIR "${PROJECT_ROOT}/_dist")
set(WHEEL_STAMP "${PROJECT_ROOT}/_build/.wheel_stamp")
set(TEMP_VENV "${PROJECT_ROOT}/_build/.venv_wheel_build")

if(NOT EXISTS "${PROJECT_ROOT}/_install")
    message(FATAL_ERROR "Install directory not found. Run 'cmake -P scripts/install.cmake' first.")
endif()

function(record_stamp OUTPUT_VAR PUBLIC_DOC_PATH_SET_SHA256)
    set(LOCAL_CONTENT "")
    foreach(LIB_PATH IN LISTS STAMPED_LIBS)
        if(EXISTS "${LIB_PATH}")
            file(TIMESTAMP "${LIB_PATH}" THIS_TIME "%s")
            string(APPEND LOCAL_CONTENT "${LIB_PATH}=${THIS_TIME}\n")
        else()
            string(APPEND LOCAL_CONTENT "${LIB_PATH}=MISSING\n")
        endif()
    endforeach()
    string(APPEND LOCAL_CONTENT
        "PUBLIC_DOC_PATH_SET_SHA256=${PUBLIC_DOC_PATH_SET_SHA256}\n")
    set(${OUTPUT_VAR} "${LOCAL_CONTENT}" PARENT_SCOPE)
endfunction()

function(get_latest_file OUTPUT_VAR GLOB_PATTERN)
    file(GLOB LOCAL_FILES "${GLOB_PATTERN}")
    if(NOT LOCAL_FILES)
        set(${OUTPUT_VAR} "" PARENT_SCOPE)
        return()
    endif()
    list(GET LOCAL_FILES 0 LATEST_FILE)
    file(TIMESTAMP "${LATEST_FILE}" LATEST_TS "%s")
    foreach(CANDIDATE IN LISTS LOCAL_FILES)
        file(TIMESTAMP "${CANDIDATE}" CANDIDATE_TS "%s")
        if(CANDIDATE_TS GREATER LATEST_TS)
            set(LATEST_FILE "${CANDIDATE}")
            set(LATEST_TS "${CANDIDATE_TS}")
        endif()
    endforeach()
    set(${OUTPUT_VAR} "${LATEST_FILE}" PARENT_SCOPE)
endfunction()

# Discover ovphysx package sources under python/ovphysx/ (.py, .pyi, py.typed).
# Returns absolute paths for stamp tracking and paths relative to PKG_DIR for staging copy.
function(ovphysx_collect_python_package_sources PKG_DIR OUT_ABS OUT_REL OUT_PY_TYPED)
    set(_source_globs "${PKG_DIR}/*.py" "${PKG_DIR}/*.pyi")
    file(GLOB_RECURSE _abs ${_source_globs})
    file(GLOB_RECURSE _rel RELATIVE "${PKG_DIR}" ${_source_globs})
    set(${OUT_ABS} "${_abs}" PARENT_SCOPE)
    set(${OUT_REL} "${_rel}" PARENT_SCOPE)
    set(${OUT_PY_TYPED} "${PKG_DIR}/py.typed" PARENT_SCOPE)
endfunction()

set(REBUILD_REASON "")
set(LIB_NAME "${SHARED_LIB_PREFIX}ovphysx${SHARED_LIB_SUFFIX}")
set(INTERNAL_LIB_NAME "${SHARED_LIB_PREFIX}ovphysx_internal${SHARED_LIB_SUFFIX}")
set(INSTALL_LIB_DIR "${PROJECT_ROOT}/_install/${INSTALL_RUNTIME_SUBDIR}")
set(_OVPHYSX_PKG_DIR "${PYTHON_SRC_DIR}/ovphysx")
ovphysx_collect_python_package_sources(
    "${_OVPHYSX_PKG_DIR}"
    _OVPHYSX_PKG_SOURCE_PATHS_ABS
    _OVPHYSX_PKG_SOURCE_PATHS_REL
    _OVPHYSX_PY_TYPED_PATH)
ovphysx_collect_public_doc_paths(
    "${PROJECT_ROOT}/docs"
    _OVPHYSX_PUBLIC_DOCS_MD_REL
    _OVPHYSX_PUBLIC_DOCS_RAW_REL)
set(_OVPHYSX_PUBLIC_DOC_PATHS_REL
    ${_OVPHYSX_PUBLIC_DOCS_MD_REL} ${_OVPHYSX_PUBLIC_DOCS_RAW_REL})
list(SORT _OVPHYSX_PUBLIC_DOC_PATHS_REL)
string(JOIN "\n" _OVPHYSX_PUBLIC_DOC_PATH_SET
    ${_OVPHYSX_PUBLIC_DOC_PATHS_REL})
string(SHA256 _OVPHYSX_PUBLIC_DOC_PATH_SET_SHA256
    "${_OVPHYSX_PUBLIC_DOC_PATH_SET}")
set(_OVPHYSX_PUBLIC_DOC_PATHS_ABS "")
foreach(_REL_DOC IN LISTS _OVPHYSX_PUBLIC_DOC_PATHS_REL)
    list(APPEND _OVPHYSX_PUBLIC_DOC_PATHS_ABS "${PROJECT_ROOT}/docs/${_REL_DOC}")
endforeach()
set(_OVPHYSX_PUBLIC_DOC_LITERALINCLUDES_ABS "")
foreach(_REL_MD IN LISTS _OVPHYSX_PUBLIC_DOCS_MD_REL)
    set(_SOURCE_MD "${PROJECT_ROOT}/docs/${_REL_MD}")
    get_filename_component(_SOURCE_MD_DIR "${_SOURCE_MD}" DIRECTORY)
    file(STRINGS "${_SOURCE_MD}" _LITERALINCLUDE_LINES
        REGEX "^`+\\{literalinclude\\}[ \t]+")
    foreach(_DIRECTIVE_LINE IN LISTS _LITERALINCLUDE_LINES)
        string(REGEX REPLACE "^`+\\{literalinclude\\}[ \t]+" ""
            _RELATIVE_INCLUDE "${_DIRECTIVE_LINE}")
        string(STRIP "${_RELATIVE_INCLUDE}" _RELATIVE_INCLUDE)
        get_filename_component(_ABSOLUTE_INCLUDE
            "${_SOURCE_MD_DIR}/${_RELATIVE_INCLUDE}" ABSOLUTE)
        list(APPEND _OVPHYSX_PUBLIC_DOC_LITERALINCLUDES_ABS "${_ABSOLUTE_INCLUDE}")
    endforeach()
endforeach()
list(REMOVE_DUPLICATES _OVPHYSX_PUBLIC_DOC_LITERALINCLUDES_ABS)
list(SORT _OVPHYSX_PUBLIC_DOC_LITERALINCLUDES_ABS)

if(NOT FORCE_REBUILD)
    message(STATUS "")
    message(STATUS "=== Checking if wheel rebuild is needed ===")

    set(INSTALL_MAIN "${INSTALL_LIB_DIR}/${LIB_NAME}")
    set(INSTALL_INTERNAL "${INSTALL_LIB_DIR}/${INTERNAL_LIB_NAME}")

    set(STAMPED_LIBS
        "${INSTALL_MAIN}"
        "${INSTALL_INTERNAL}"
        "${PYTHON_SRC_DIR}/pyproject.toml"
        "${SCRIPT_DIR}/preprocess_docs.cmake"
        "${SCRIPT_DIR}/preprocess_markdown.py"
    )
    list(APPEND STAMPED_LIBS ${_OVPHYSX_PKG_SOURCE_PATHS_ABS})
    list(APPEND STAMPED_LIBS ${_OVPHYSX_PUBLIC_DOC_PATHS_ABS})
    list(APPEND STAMPED_LIBS ${_OVPHYSX_PUBLIC_DOC_LITERALINCLUDES_ABS})
    if(EXISTS "${_OVPHYSX_PY_TYPED_PATH}")
        list(APPEND STAMPED_LIBS "${_OVPHYSX_PY_TYPED_PATH}")
    endif()
    file(GLOB_RECURSE OVSTAGE_PACKAGE_FILES "${PROJECT_ROOT}/_install/python/ovstage/*")
    list(APPEND STAMPED_LIBS ${OVSTAGE_PACKAGE_FILES})

    # The codeless USD schema artifacts (staged into _install/schemas by install.cmake)
    # are copied into the wheel later in this script. Tracking them makes a schema-only
    # change (regenerated/added schema, libs/py unchanged) invalidate the stamp instead
    # of shipping stale codeless schema data.
    file(GLOB_RECURSE INSTALL_SCHEMA_FILES "${PROJECT_ROOT}/_install/schemas/*")
    list(APPEND STAMPED_LIBS ${INSTALL_SCHEMA_FILES})
    file(GLOB_RECURSE INSTALL_PLUGIN_FILES "${PROJECT_ROOT}/_install/plugins/*")
    list(APPEND STAMPED_LIBS ${INSTALL_PLUGIN_FILES})

    if(EXISTS "${WHEEL_STAMP}")
        file(READ "${WHEEL_STAMP}" RECORDED_STAMP)
        string(REGEX MATCH
            "PUBLIC_DOC_PATH_SET_SHA256=([0-9a-f]+)"
            _PUBLIC_DOC_PATH_SET_MATCH "${RECORDED_STAMP}")
        if(NOT _PUBLIC_DOC_PATH_SET_MATCH OR
                NOT CMAKE_MATCH_1 STREQUAL _OVPHYSX_PUBLIC_DOC_PATH_SET_SHA256)
            set(REBUILD_REASON "Public documentation path set changed")
        else()
            foreach(LIB_PATH IN LISTS STAMPED_LIBS)
                if(NOT EXISTS "${LIB_PATH}")
                    set(REBUILD_REASON "Tracked file missing: ${LIB_PATH}")
                    break()
                endif()
                file(TIMESTAMP "${LIB_PATH}" CURRENT_TS "%s")
                string(REGEX MATCH "${LIB_PATH}=([^\n]+)" MATCH_LINE "${RECORDED_STAMP}")
                if(NOT MATCH_LINE)
                    set(REBUILD_REASON "No stamp entry for ${LIB_PATH}")
                    break()
                endif()
                string(REGEX REPLACE ".*=" "" RECORDED_TS "${MATCH_LINE}")
                if(NOT CURRENT_TS STREQUAL RECORDED_TS)
                    set(REBUILD_REASON "File changed: ${LIB_PATH}")
                    break()
                endif()
            endforeach()
        endif()
    else()
        set(REBUILD_REASON "Stamp file missing (first build or cleaned deps)")
    endif()

    if(REBUILD_REASON STREQUAL "")
        message(STATUS "Wheel is up-to-date, skipping rebuild")
        message(STATUS "  Stamp: ${WHEEL_STAMP}")
        message(STATUS "")
        message(STATUS "To force rebuild: cmake -DFORCE_REBUILD=ON -P scripts/build_wheel.cmake")
        message(STATUS "")
        message(STATUS "========================================")
        message(STATUS "Wheel is current (no rebuild needed)")
        message(STATUS "========================================")
        message(STATUS "")
        return()
    else()
        message(STATUS "Wheel rebuild required: ${REBUILD_REASON}")
    endif()
else()
    set(REBUILD_REASON "FORCE_REBUILD=ON")
endif()

# ============================================================================
# Stage wheel contents in _build/python_wheel_staging/
# ============================================================================

message(STATUS "")
message(STATUS "=== Staging Wheel Contents ===")
message(STATUS "Staging root: ${STAGING_ROOT}")

if(EXISTS "${STAGING_ROOT}")
    file(REMOVE_RECURSE "${STAGING_ROOT}")
endif()
file(MAKE_DIRECTORY "${STAGING_ROOT}")
file(MAKE_DIRECTORY "${STAGING_WHEEL_PKG_DIR}")

file(COPY "${PYTHON_SRC_DIR}/pyproject.toml" DESTINATION "${STAGING_ROOT}")
if(EXISTS "${PYTHON_SRC_DIR}/setup.py")
    file(COPY "${PYTHON_SRC_DIR}/setup.py" DESTINATION "${STAGING_ROOT}")
endif()
if(EXISTS "${PYTHON_SRC_DIR}/README.md")
    file(COPY "${PYTHON_SRC_DIR}/README.md" DESTINATION "${STAGING_ROOT}")
endif()
message(STATUS "  Copied packaging metadata (pyproject.toml, setup.py, README.md)")

# Copy the Python source and stub files into the staging package dir. The old
# ovphysx.tensors Python compatibility package must not silently reappear in
# the wheel.
set(_LEGACY_TENSOR_PY_SOURCES ${_OVPHYSX_PKG_SOURCE_PATHS_REL})
list(FILTER _LEGACY_TENSOR_PY_SOURCES INCLUDE REGEX "^tensors/")
if(_LEGACY_TENSOR_PY_SOURCES)
    string(REPLACE ";" "\n  " _LEGACY_TENSOR_PY_LIST "${_LEGACY_TENSOR_PY_SOURCES}")
    message(FATAL_ERROR
        "Something is wrong: legacy ovphysx.tensors Python files still exist, "
        "but the old Python TensorAPI must not be shipped in the ovphysx wheel. "
        "Remove these files instead of silently excluding them. This does not "
        "apply to the native tensor backend, now folded into the static PhysX runtime "
        "(OMPE-96492), which is still required by TensorBindingsAPI:\n"
        "  ${_LEGACY_TENSOR_PY_LIST}"
    )
endif()
foreach(_PY_FILE IN LISTS _OVPHYSX_PKG_SOURCE_PATHS_REL)
    get_filename_component(_PY_DIR "${_PY_FILE}" DIRECTORY)
    if(_PY_DIR)
        file(MAKE_DIRECTORY "${STAGING_WHEEL_PKG_DIR}/${_PY_DIR}")
    endif()
    file(COPY "${_OVPHYSX_PKG_DIR}/${_PY_FILE}" DESTINATION "${STAGING_WHEEL_PKG_DIR}/${_PY_DIR}")
endforeach()
list(LENGTH _OVPHYSX_PKG_SOURCE_PATHS_REL _PY_COUNT)
message(STATUS "  Copied ${_PY_COUNT} Python source and stub files (.py/.pyi)")

if(EXISTS "${_OVPHYSX_PY_TYPED_PATH}")
    file(COPY "${_OVPHYSX_PY_TYPED_PATH}" DESTINATION "${STAGING_WHEEL_PKG_DIR}")
    message(STATUS "  Copied py.typed (PEP 561 typed package marker)")
endif()

# Two-wheel split: the ovphysx wheel ships ONLY ovphysx (no ovstage package, no USD
# closure). ovstage and USD come from the separate ovstage wheel, which a consumer
# installs alongside this one. libovphysx resolves libovstage and the USD monolith
# from there at load. Bundling ovstage here would duplicate ovstage/USD and produce
# two ovstage and two USD instances.
message(STATUS "  Skipped ovstage Python package (shipped as the separate ovstage wheel)")

# The wheel metadata needs _version.py in the staging tree.
message(STATUS "")
message(STATUS "=== Generate Version File ===")
generate_python_version_file("${STAGING_WHEEL_PKG_DIR}")

# The wheel structure MUST match _install/ because CarboniteLoader computes
# pluginsDir = <libDir>/../plugins, so the wheel needs ovphysx/lib/libovphysx.so
# and ovphysx/plugins/*.so.
message(STATUS "")
message(STATUS "=== Copying from Install Tree (matching _install/ structure) ===")

set(PKG_LIB_DIR "${STAGING_WHEEL_PKG_DIR}/lib")
set(PKG_PLUGINS_DIR "${STAGING_WHEEL_PKG_DIR}/plugins")
set(INSTALL_PLUGINS_DIR "${PROJECT_ROOT}/_install/plugins")

if(NOT EXISTS "${INSTALL_PLUGINS_DIR}")
    message(FATAL_ERROR "Plugins not found at: ${INSTALL_PLUGINS_DIR}\nRun 'cmake -P scripts/install.cmake' first.")
endif()

# lib/ holds the main library and the internal sidecar.
file(MAKE_DIRECTORY "${PKG_LIB_DIR}")

set(SRC_LIB "${INSTALL_LIB_DIR}/${LIB_NAME}")
set(SRC_INTERNAL "${INSTALL_LIB_DIR}/${INTERNAL_LIB_NAME}")

# Symlinks are resolved so the real library file is copied into the wheel.
if(EXISTS "${SRC_LIB}")
    file(REAL_PATH "${SRC_LIB}" SRC_LIB_REAL)
else()
    set(SRC_LIB_REAL "")
endif()
if(EXISTS "${SRC_INTERNAL}")
    file(REAL_PATH "${SRC_INTERNAL}" SRC_INTERNAL_REAL)
else()
    set(SRC_INTERNAL_REAL "")
endif()

if(NOT EXISTS "${SRC_LIB}" AND NOT EXISTS "${SRC_LIB_REAL}")
    message(FATAL_ERROR "Library not found: ${SRC_LIB}")
endif()
if(NOT EXISTS "${SRC_INTERNAL}" AND NOT EXISTS "${SRC_INTERNAL_REAL}")
    message(FATAL_ERROR "Internal sidecar not found: ${SRC_INTERNAL}")
endif()

function(_copy_lib_with_links SRC_PATH SRC_REAL LIB_NAME SONAME_PATH)
    if(SRC_REAL)
        set(SRC_VERSIONED "${SRC_REAL}")
        get_filename_component(SRC_VERSIONED_NAME "${SRC_REAL}" NAME)
    else()
        set(SRC_VERSIONED "${SRC_PATH}")
        get_filename_component(SRC_VERSIONED_NAME "${SRC_PATH}" NAME)
    endif()
    file(COPY "${SRC_VERSIONED}" DESTINATION "${PKG_LIB_DIR}")
    if(NOT "${LIB_NAME}" STREQUAL "${SRC_VERSIONED_NAME}")
        file(RENAME "${PKG_LIB_DIR}/${SRC_VERSIONED_NAME}" "${PKG_LIB_DIR}/${LIB_NAME}")
    endif()
endfunction()

_copy_lib_with_links("${SRC_LIB}" "${SRC_LIB_REAL}" "${LIB_NAME}" "${SRC_LIB_SONAME}")
_copy_lib_with_links("${SRC_INTERNAL}" "${SRC_INTERNAL_REAL}" "${INTERNAL_LIB_NAME}" "${SRC_INTERNAL_SONAME}")
message(STATUS "  Copied to lib/: ${LIB_NAME}, ${INTERNAL_LIB_NAME}")

# Copy PDB debug symbol files alongside DLLs for Debug builds on Windows.
if(WIN32 AND BUILD_TYPE STREQUAL "Debug")
    file(GLOB _INSTALL_PDBS "${INSTALL_LIB_DIR}/*.pdb")
    foreach(_pdb IN LISTS _INSTALL_PDBS)
        file(COPY "${_pdb}" DESTINATION "${PKG_LIB_DIR}")
        get_filename_component(_pdb_name "${_pdb}" NAME)
        message(STATUS "  Copied PDB to lib/: ${_pdb_name}")
    endforeach()
endif()

# Copy the flat plugins/ directory with all carbonite/physx plugins.
# The install tree carries no USD libraries or USD plugin registry. The ovstage
# wheel brings its own namespaced USD runtime, and verify_pyless_closure.py below
# fails the build if any USD payload reappears here.
file(COPY "${INSTALL_PLUGINS_DIR}/" DESTINATION "${PKG_PLUGINS_DIR}")
message(STATUS "  Copied plugins/ directory")

file(GLOB ALL_PLUGINS "${PKG_PLUGINS_DIR}/*${SHARED_LIB_SUFFIX}*")
list(LENGTH ALL_PLUGINS PLUGIN_COUNT)
message(STATUS "  Total plugins: ${PLUGIN_COUNT}")

# The skills index and skills ship in the wheel package. AGENTS.md is a monorepo
# developer doc and is intentionally not shipped.
if(EXISTS "${PROJECT_ROOT}/SKILLS.md")
    copy_file_if_different("${PROJECT_ROOT}/SKILLS.md" "${STAGING_WHEEL_PKG_DIR}/SKILLS.md")
    message(STATUS "  Copied SKILLS.md into wheel package")
else()
    message(FATAL_ERROR "SKILLS.md not found at ${PROJECT_ROOT}/SKILLS.md")
endif()
if(EXISTS "${PROJECT_ROOT}/skills")
    copy_tree_if_different("${PROJECT_ROOT}/skills" "${STAGING_WHEEL_PKG_DIR}/skills")
    message(STATUS "  Copied skills/ into wheel package")
else()
    message(FATAL_ERROR "skills/ directory not found at ${PROJECT_ROOT}/skills")
endif()


# Preprocess and copy the public portable docs, excluding internal docs, so LLMs can
# read headings/structure directly. Markdown has its MyST-only constructs resolved.
# RST and image assets retain their source paths.
preprocess_public_docs(
    "${TARGET_PYTHON}" "${PROJECT_ROOT}"
    "${PROJECT_ROOT}/docs" "${STAGING_WHEEL_PKG_DIR}/docs"
)

# Public sample source code and data ship in the wheel package under ovphysx/samples/.
set(SAMPLES_DST "${STAGING_WHEEL_PKG_DIR}/samples")
file(MAKE_DIRECTORY "${SAMPLES_DST}")

if(EXISTS "${PROJECT_ROOT}/tests/python_samples")
    # Generated local state is build-environment-specific and must not ship in
    # the wheel. The helper excludes it from the copy itself.
    stage_python_samples_tree(
        "${PROJECT_ROOT}/tests/python_samples" "${SAMPLES_DST}/python_samples")
    message(STATUS "  Copied tests/python_samples -> wheel: ovphysx/samples/python_samples/")
else()
    message(FATAL_ERROR "tests/python_samples not found at ${PROJECT_ROOT}/tests/python_samples")
endif()

if(EXISTS "${PROJECT_ROOT}/tests/data")
    copy_tree_if_different("${PROJECT_ROOT}/tests/data" "${SAMPLES_DST}/data")
    message(STATUS "  Copied tests/data -> wheel: ovphysx/samples/data/")
else()
    message(FATAL_ERROR "tests/data not found at ${PROJECT_ROOT}/tests/data")
endif()

# The codeless USD schemas ship in the wheel package under ovphysx/schemas/.
# install.cmake stages them into _install/schemas. They are exposed at a stable
# path for external authoring/validation tooling. See ovphysx.codeless_schema_paths().
set(INSTALL_SCHEMAS_DIR "${PROJECT_ROOT}/_install/schemas")
if(EXISTS "${INSTALL_SCHEMAS_DIR}")
    copy_tree_if_different("${INSTALL_SCHEMAS_DIR}" "${STAGING_WHEEL_PKG_DIR}/schemas")
    message(STATUS "  Copied codeless USD schemas -> wheel: ovphysx/schemas/")
else()
    message(FATAL_ERROR
        "Codeless USD schemas not found at ${INSTALL_SCHEMAS_DIR}. "
        "Run 'cmake -P scripts/install.cmake' first.")
endif()

if(NOT WIN32)
    if(BUILD_TYPE STREQUAL "Debug")
        message(STATUS "Skipping stripping for Debug build (preserving debug symbols)")
    else()
        strip_unstripped_elf_binaries("${STAGING_WHEEL_PKG_DIR}" "wheel package tree")
    endif()
endif()

# Verify that all shipped binaries respect the glibc/libstdc++ baseline.
verify_glibc_baseline("${STAGING_WHEEL_PKG_DIR}" "wheel package tree")

# The wheel is namespaced and py-less on every platform, so the staging tree
# must not contain interpreter runtimes, Python-flavored USD, or classic modular
# USD.
message(STATUS "Verifying py-less, USD-free package contents for wheel staging tree...")
execute_process(
    COMMAND "${TARGET_PYTHON}" "${SCRIPT_DIR}/verify_pyless_closure.py"
            "--dir=${STAGING_WHEEL_PKG_DIR}" --require-schemas
    WORKING_DIRECTORY "${PROJECT_ROOT}"
    RESULT_VARIABLE PYLESS_WHEEL_CHECK_RESULT
)
if(NOT PYLESS_WHEEL_CHECK_RESULT EQUAL 0)
    message(FATAL_ERROR
        "Package content verification failed for the wheel staging tree.\n"
        "The wheel must not ship libpython, any OpenUSD library or its dependency\n"
        "closure, a USD plugin registry, Python schema bindings, or build-tree Python\n"
        "RPATH entries, and must ship schemas/physx.\n"
        "See verify_pyless_closure.py output above for details."
    )
endif()

# Generate the wheel packaging lock files for CI artifacts.
# generate_packaging_lock.py is excluded from the open-source copy along with
# packaging_lock/. When absent, the entire lock mechanism is skipped.
if(EXISTS "${SCRIPT_DIR}/generate_packaging_lock.py")
    message(STATUS "Generating wheel packaging lock files...")
    execute_process(
        COMMAND "${TARGET_PYTHON}" "${SCRIPT_DIR}/generate_packaging_lock.py"
                "--install-dir=${PROJECT_ROOT}/_install"
                "--wheel-dir=${STAGING_WHEEL_PKG_DIR}"
                "--output-dir=${PROJECT_ROOT}/packaging_lock"
                --artifact=wheel
        WORKING_DIRECTORY "${PROJECT_ROOT}"
        RESULT_VARIABLE WHEEL_LOCK_GEN_RESULT
    )
    if(NOT WHEEL_LOCK_GEN_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to generate wheel packaging lock files (exit code: ${WHEEL_LOCK_GEN_RESULT})")
    endif()

    # Verify the wheel against the committed lock files and pick up a deferred SDK
    # lock failure. install.cmake writes a sentinel instead of FATAL_ERROR so both
    # _new.json files exist before the build fails. With both snapshots available as
    # CI artifacts, a single combined error is emitted if either lock check failed.
    set(SDK_LOCK_SENTINEL "${PROJECT_ROOT}/_build/.sdk_lock_failed")
    set(_SDK_LOCK_FAILED FALSE)
    set(_WHEEL_LOCK_FAILED FALSE)

    if(EXISTS "${SDK_LOCK_SENTINEL}")
        set(_SDK_LOCK_FAILED TRUE)
    endif()

    # Honor SKIP_LOCK_CHECK from either the -D form or the environment. See
    # install.cmake / verify_glibc_baseline() for why the env-var form matters.
    # It lets validate_all.cmake forward the flag through nested subprocess
    # invocations of cmake --build.
    set(_OVPHYSX_SKIP_LOCK_CHECK FALSE)
    if(SKIP_LOCK_CHECK)
        set(_OVPHYSX_SKIP_LOCK_CHECK TRUE)
    elseif(DEFINED ENV{SKIP_LOCK_CHECK} AND "$ENV{SKIP_LOCK_CHECK}")
        set(_OVPHYSX_SKIP_LOCK_CHECK TRUE)
        # Distinct message so a stale shell env does not silently skip the check
        # on a direct cmake -P invocation.
        message(STATUS "SKIP_LOCK_CHECK picked up from environment (not -D)")
    endif()
    if(NOT _OVPHYSX_SKIP_LOCK_CHECK AND NOT BUILD_TYPE STREQUAL "Debug")
        message(STATUS "Checking wheel packaging lock...")
        # devphysx/from-source builds produce checked, locally built libraries that are
        # legitimately larger than the prebuilt-release size baseline, so that mode skips
        # ONLY the size-delta check. The library set/layout is still enforced, and the
        # normal prebuilt-release path keeps the size check strict. BUILD_DEVPHYSX is the
        # CI variable that drives ./build.sh --devphysx. It is empty on prebuilt-release
        # builds and reaches this cmake -P step as an environment variable (like SKIP_LOCK_CHECK).
        set(_OVPHYSX_LOCK_SIZE_ARGS)
        if(DEFINED ENV{BUILD_DEVPHYSX} AND NOT "$ENV{BUILD_DEVPHYSX}" STREQUAL "")
            list(APPEND _OVPHYSX_LOCK_SIZE_ARGS "--ignore-size-deltas")
            message(STATUS "  devphysx build: relaxing the wheel lock size-delta check (set/layout still enforced)")
        endif()
        execute_process(
            COMMAND "${TARGET_PYTHON}" "${SCRIPT_DIR}/generate_packaging_lock.py"
                    "--install-dir=${PROJECT_ROOT}/_install"
                    "--wheel-dir=${STAGING_WHEEL_PKG_DIR}"
                    "--output-dir=${PROJECT_ROOT}/packaging_lock"
                    --check
                    --artifact=wheel
                    ${_OVPHYSX_LOCK_SIZE_ARGS}
            WORKING_DIRECTORY "${PROJECT_ROOT}"
            RESULT_VARIABLE WHEEL_LOCK_CHECK_RESULT
        )
        if(NOT WHEEL_LOCK_CHECK_RESULT EQUAL 0)
            set(_WHEEL_LOCK_FAILED TRUE)
        endif()
    else()
        message(WARNING "Wheel packaging lock check skipped (SKIP_LOCK_CHECK set or Debug build)")
    endif()

    # Deferred combined error gate. Both _new.json files have been generated by this
    # point, so CI can always download them as artifacts even when the build fails.
    if(_SDK_LOCK_FAILED OR _WHEEL_LOCK_FAILED)
        set(_LOCK_MSG "Packaging lock mismatch detected!\n\n")
        if(_SDK_LOCK_FAILED)
            string(APPEND _LOCK_MSG "  - SDK lock FAILED (see install.cmake output above)\n")
        endif()
        if(_WHEEL_LOCK_FAILED)
            string(APPEND _LOCK_MSG "  - Wheel lock FAILED (see generate_packaging_lock.py output above)\n")
        endif()
        string(APPEND _LOCK_MSG "\n"
            "Both SDK and wheel _new.json snapshots have been generated in packaging_lock/.\n"
            "See the generate_packaging_lock.py output above for detailed fix instructions.\n"
            "To skip lock checks for local development:\n"
            "  cmake -DSKIP_LOCK_CHECK=ON -P scripts/install.cmake\n"
            "  cmake -DSKIP_LOCK_CHECK=ON -P scripts/build_wheel.cmake"
        )
        if(EXISTS "${SDK_LOCK_SENTINEL}")
            file(REMOVE "${SDK_LOCK_SENTINEL}")
        endif()
        message(FATAL_ERROR "${_LOCK_MSG}")
    endif()
else()
    message(STATUS "Wheel packaging lock skipped (generate_packaging_lock.py not present)")
endif()

# ============================================================================
# Build wheel from staging root
# ============================================================================

message(STATUS "")
message(STATUS "=== Building Python Wheel ===")

file(MAKE_DIRECTORY "${DIST_DIR}")

# Remove stale dist/build artifacts from the staging root.
set(STAGING_DIST "${STAGING_ROOT}/dist")
if(EXISTS "${STAGING_DIST}")
    file(REMOVE_RECURSE "${STAGING_DIST}")
endif()
set(STAGING_BUILD "${STAGING_ROOT}/build")
if(EXISTS "${STAGING_BUILD}")
    file(REMOVE_RECURSE "${STAGING_BUILD}")
endif()

# Copy the binary license and LICENSES archive to the staging root for wheel packaging.
# The LICENSES archive only exists when repo_licensing ran during install. The
# public source drop does not ship repo_licensing (no public package exists),
# so open-source wheels are built without the bundled third-party archive.
file(READ "${PROJECT_ROOT}/deps/repo-deps.packman.xml" _OVPHYSX_REPO_DEPS_CONTENT)
set(BINARY_LICENSE "${PROJECT_ROOT}/licenses/LICENSE-binary.txt")
set(LICENSES_ZIP "${PROJECT_ROOT}/_install/ovphysx-LICENSES.zip")
if(NOT EXISTS "${BINARY_LICENSE}")
    message(FATAL_ERROR "Binary license not found: ${BINARY_LICENSE}")
endif()
if(NOT EXISTS "${LICENSES_ZIP}" AND _OVPHYSX_REPO_DEPS_CONTENT MATCHES "repo_licensing")
    message(FATAL_ERROR "Third-party licenses archive not found: ${LICENSES_ZIP}\nRun 'cmake -P scripts/install.cmake' first.")
endif()
unset(_OVPHYSX_REPO_DEPS_CONTENT)
file(COPY "${BINARY_LICENSE}" DESTINATION "${STAGING_ROOT}")
file(RENAME "${STAGING_ROOT}/LICENSE-binary.txt" "${STAGING_ROOT}/LICENSE.txt")
if(EXISTS "${LICENSES_ZIP}")
    file(COPY "${LICENSES_ZIP}" DESTINATION "${STAGING_ROOT}")
else()
    message(STATUS "Wheel staged without ovphysx-LICENSES.zip (license gathering skipped in the public source drop)")
endif()

# The wheel is built in a temporary venv created from the target-deps Python.
message(STATUS "Building wheel...")

if(EXISTS "${TEMP_VENV}")
    file(REMOVE_RECURSE "${TEMP_VENV}")
endif()

message(STATUS "Creating temporary build environment...")
execute_process(
    COMMAND "${TARGET_PYTHON}" -m venv ${TEMP_VENV}
    WORKING_DIRECTORY "${STAGING_ROOT}"
    RESULT_VARIABLE VENV_RESULT
    ERROR_VARIABLE VENV_ERROR
)

if(NOT VENV_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to create temporary venv with ${TARGET_PYTHON}\n${VENV_ERROR}")
endif()

if(WIN32)
    set(VENV_PYTHON "${TEMP_VENV}/Scripts/python.exe")
    set(VENV_PIP "${TEMP_VENV}/Scripts/pip.exe")
else()
    set(VENV_PYTHON "${TEMP_VENV}/bin/python")
    set(VENV_PIP "${TEMP_VENV}/bin/pip")
endif()

message(STATUS "Installing setuptools and wheel modules...")
execute_process(
    COMMAND "${VENV_PIP}" install --quiet setuptools wheel
    WORKING_DIRECTORY "${STAGING_ROOT}"
    RESULT_VARIABLE INSTALL_RESULT
    OUTPUT_QUIET
    ERROR_QUIET
)

if(NOT INSTALL_RESULT STREQUAL "0")
    file(REMOVE_RECURSE "${TEMP_VENV}")
    message(FATAL_ERROR "Failed to install setuptools and wheel in venv")
endif()

get_wheel_platform_tag()

message(STATUS "Building platform-specific wheel for ${WHEEL_PLAT_NAME}...")
execute_process(
    COMMAND ${VENV_PYTHON} setup.py bdist_wheel --plat-name ${WHEEL_PLAT_NAME}
    WORKING_DIRECTORY "${STAGING_ROOT}"
    RESULT_VARIABLE BUILD_RESULT
)

file(REMOVE_RECURSE "${TEMP_VENV}")

if(NOT BUILD_RESULT STREQUAL "0")
    message(FATAL_ERROR "Failed to build wheel (exit code: ${BUILD_RESULT})")
endif()

message(STATUS "Wheel built successfully")

get_latest_file(WHEEL_FILE "${STAGING_DIST}/*.whl")
if(NOT WHEEL_FILE)
    message(FATAL_ERROR "Wheel file not found in ${STAGING_DIST}")
endif()

get_filename_component(WHEEL_NAME "${WHEEL_FILE}" NAME)
set(FINAL_WHEEL "${DIST_DIR}/${WHEEL_NAME}")
file(RENAME "${WHEEL_FILE}" "${FINAL_WHEEL}")

# Wheels from previous builds or branches would otherwise accumulate in _dist/.
file(GLOB _EXISTING_WHEELS "${DIST_DIR}/ovphysx-*.whl")
foreach(_OLD_WHEEL IN LISTS _EXISTING_WHEELS)
    if(NOT _OLD_WHEEL STREQUAL FINAL_WHEEL)
        get_filename_component(_OLD_NAME "${_OLD_WHEEL}" NAME)
        message(STATUS "Removing stale wheel: ${_OLD_NAME}")
        file(REMOVE "${_OLD_WHEEL}")
    endif()
endforeach()

message(STATUS "")
message(STATUS "=== Verifying Wheel Artifact ===")
file(SIZE "${FINAL_WHEEL}" WHEEL_SIZE)
math(EXPR WHEEL_SIZE_MB "${WHEEL_SIZE} / (1024 * 1024)")

message(STATUS "Wheel artifact verified:")
message(STATUS "  File: ${WHEEL_NAME}")
message(STATUS "  Path: ${FINAL_WHEEL}")
message(STATUS "  Size: ${WHEEL_SIZE_MB} MB")

# The stamp lets future runs skip the rebuild when nothing changed.
list(APPEND STAMPED_LIBS "${FINAL_WHEEL}")
record_stamp(CURRENT_STAMP "${_OVPHYSX_PUBLIC_DOC_PATH_SET_SHA256}")
file(WRITE "${WHEEL_STAMP}" "${CURRENT_STAMP}")
message(STATUS "Updated wheel stamp: ${WHEEL_STAMP}")

if(IS_DIRECTORY "${TEMP_VENV}")
    file(REMOVE_RECURSE "${TEMP_VENV}")
endif()

message(STATUS "")
message(STATUS "========================================")
message(STATUS "Wheel build completed successfully!")
message(STATUS "========================================")
message(STATUS "")
