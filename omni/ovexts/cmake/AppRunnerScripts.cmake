# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# App launcher script generation for ovexts.
# Replaces define_app() calls from premake5.lua.

# ---------------------------------------------------------------------------
# ovexts_define_app(<app_name>
#     [EXTRA_ARGS <args>]       # extra kit arguments (e.g., "--portable --/renderer/debug/validation/enabled=false")
# )
#
# Generates:
#   1. A Kit app launcher script (<app_name>.bat / .sh)
#   2. A Visual Studio utility project (Windows only) with debugger settings
#      so the app can be launched/debugged directly from Visual Studio.
#
# The app_name should be a .kit file name (e.g., "omni.bloky.kit").
# The launcher invokes kit/kit with the .kit file from apps/ directory.
# ---------------------------------------------------------------------------
function(ovexts_define_app APP_NAME)
    cmake_parse_arguments(ARG "" "EXTRA_ARGS" "" ${ARGN})

    if(NOT ARG_EXTRA_ARGS)
        set(ARG_EXTRA_ARGS "")
    endif()

    # Pin the portable root to the app's own build output dir so Kit's
    # data/cache/logs (incl. the extension registry cache) live under
    # release/ instead of defaulting into the shared packman kit-kernel
    # package dir (${kit}/data/...).  This keeps the runtime cache project-local
    # and wipeable by `build.bat -x` / `--clear-cache`.
    if(WIN32)
        set(_CONTENT "@echo off\r\nsetlocal\r\n")
        string(APPEND _CONTENT "\"%~dp0kit\\kit.exe\" \"%~dp0apps\\${APP_NAME}\" --portable-root \"%~dp0.\" ${ARG_EXTRA_ARGS} %*\r\n")

        file(GENERATE OUTPUT "${OVEXTS_BIN_DIR}/${APP_NAME}.bat"
            CONTENT "${_CONTENT}")
    else()
        set(_CONTENT "#!/bin/bash\nset -e\nSCRIPT_DIR=$(dirname \${BASH_SOURCE})\n")
        string(APPEND _CONTENT "${OVEXTS_LINUX_ASAN_PREAMBLE}\n")
        string(APPEND _CONTENT "\${EXEC:-exec} \"\$SCRIPT_DIR/kit/kit\" \"\$SCRIPT_DIR/apps/${APP_NAME}\" --portable-root \"\$SCRIPT_DIR\" ${ARG_EXTRA_ARGS} \"\$@\"\n")

        file(GENERATE OUTPUT "${OVEXTS_BIN_DIR}/${APP_NAME}.sh"
            CONTENT "${_CONTENT}"
            FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                             GROUP_READ GROUP_EXECUTE
                             WORLD_READ WORLD_EXECUTE)
    endif()

    # Visual Studio app project — creates a utility target that can be set as
    # startup project for launching/debugging the Kit app directly from VS.
    if(CMAKE_GENERATOR MATCHES "Visual Studio")
        # Create a target name safe for CMake (replace dots with underscores
        # for the target, but keep the original name for display).
        string(REPLACE "." "_" _TARGET_NAME "${APP_NAME}_app")
        add_custom_target(${_TARGET_NAME})
        set_target_properties(${_TARGET_NAME} PROPERTIES
            PROJECT_LABEL "${APP_NAME}"
            FOLDER "Apps"
            VS_DEBUGGER_COMMAND "${OVEXTS_BIN_DIR}/kit/kit.exe"
            VS_DEBUGGER_COMMAND_ARGUMENTS "\"${OVEXTS_BIN_DIR}/apps/${APP_NAME}\" --portable-root \"${OVEXTS_BIN_DIR}\" ${ARG_EXTRA_ARGS}"
            VS_DEBUGGER_WORKING_DIRECTORY "${OVEXTS_BIN_DIR}"
        )
    endif()

    message(STATUS "  App script: ${APP_NAME}")
endfunction()
