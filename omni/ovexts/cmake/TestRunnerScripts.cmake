# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: LicenseRef-NvidiaProprietary
#
# Test runner script generation for ovexts.
# Replaces define_physics_test_experience() and define_etm_test_experience() from premake5-public.lua.

# Common ASAN preamble for Linux test/app scripts.
# Used by both test runner and app launcher scripts.
set(OVEXTS_LINUX_ASAN_PREAMBLE [=[
if [ ! -z "$OMNI_KIT_LD_PRELOAD" ]; then
    export LD_PRELOAD="$OMNI_KIT_LD_PRELOAD:$LD_PRELOAD"
fi
if [ "$_MR_BUILD_WITH_ASAN" == "true" ]; then
    export LIB_ASAN_PATH=$SCRIPT_DIR/libasan.so.6
    export ASAN_OPTIONS="protect_shadow_gap=0:halt_on_error=0:detect_leaks=1"
    export LSAN_OPTIONS="exitcode=0"
    if [ "$LD_PRELOAD" != *"libasan.so"* ]; then
        export LD_PRELOAD="${LIB_ASAN_PATH}${LD_PRELOAD:+:$LD_PRELOAD}"
    fi
    echo "ASAN_OPTIONS=${ASAN_OPTIONS}"
    echo "LSAN_OPTIONS=${LSAN_OPTIONS}"
    echo "LIB_ASAN_PATH=${LIB_ASAN_PATH}"
    echo "LD_PRELOAD=${LD_PRELOAD}"
fi]=])

# Common ASAN preamble for Windows test/app scripts.
set(OVEXTS_WINDOWS_ASAN_PREAMBLE "")

# ---------------------------------------------------------------------------
# ovexts_define_physics_test_experience(<ext_name>
#     [SUITE <suite>]           # default: "python"
#     [DIR <dir>]               # default: "extsPhysics"
#     [SUFFIX <suffix>]         # default: ""
#     [TEST_ARGS <arg1> ...]    # extra kit arguments
# )
#
# Generates a Kit-based test runner script:
#   tests-<suite>-<ext_name><suffix>.sh (Linux)
#   tests-<suite>-<ext_name><suffix>.bat (Windows)
# ---------------------------------------------------------------------------
function(ovexts_define_physics_test_experience EXT_NAME)
    cmake_parse_arguments(ARG "" "SUITE;DIR;SUFFIX" "TEST_ARGS" ${ARGN})

    if(NOT ARG_SUITE)
        set(ARG_SUITE "python")
    endif()
    if(NOT ARG_DIR)
        set(ARG_DIR "extsPhysics")
    endif()
    if(NOT ARG_SUFFIX)
        set(ARG_SUFFIX "")
    endif()

    set(_SCRIPT_NAME "tests-${ARG_SUITE}-${EXT_NAME}${ARG_SUFFIX}")

    # Build the extra test args string
    set(_EXTRA_ARGS "")
    if(ARG_TEST_ARGS)
        foreach(_arg IN LISTS ARG_TEST_ARGS)
            string(APPEND _EXTRA_ARGS " ${_arg}")
        endforeach()
    endif()

    if(WIN32)
        # Windows .bat script
        set(_EXT_ARGS "%~dp0apps\\omni.bloky.test_ext.kit")
        string(APPEND _EXT_ARGS " --/app/exts/folders/0='$\{kit}/../extsPhysics'")
        string(APPEND _EXT_ARGS " --/app/exts/folders/1='$\{kit}/../extsPhysicsRepo'")
        string(APPEND _EXT_ARGS " --/exts/omni.kit.test/testExtApp=%~dp0apps\\omni.bloky.test_ext.kit")
        string(APPEND _EXT_ARGS " --empty --portable --enable omni.kit.test")
        string(APPEND _EXT_ARGS " --/app/enableStdoutOutput=0")
        string(APPEND _EXT_ARGS " --/exts/omni.kit.test/testExts/0='${EXT_NAME}'")
        string(APPEND _EXT_ARGS " --ext-folder \"%~dp0/apps\"")
        if(ARG_TEST_ARGS)
            string(APPEND _EXT_ARGS " %*")
            string(APPEND _EXT_ARGS "${_EXTRA_ARGS}")
        endif()

        set(_CONTENT "@echo off\r\nsetlocal\r\n")
        string(APPEND _CONTENT "\"%~dp0kit\\kit.exe\" ${_EXT_ARGS}")
        if(NOT ARG_TEST_ARGS)
            string(APPEND _CONTENT " %*")
        endif()
        string(APPEND _CONTENT "\r\n")

        file(GENERATE OUTPUT "${OVEXTS_BIN_DIR}/${_SCRIPT_NAME}.bat"
            CONTENT "${_CONTENT}")
    else()
        # Linux .sh script
        set(_EXT_ARGS "\$SCRIPT_DIR/apps/omni.bloky.test_ext.kit")
        string(APPEND _EXT_ARGS " --/app/exts/folders/0='$\{kit}/../extsPhysics'")
        string(APPEND _EXT_ARGS " --/app/exts/folders/1='$\{kit}/../extsPhysicsRepo'")
        string(APPEND _EXT_ARGS " --/app/exts/folders/2='$\{kit}/../apps'")
        string(APPEND _EXT_ARGS " --/exts/omni.kit.test/testExtApp=\$SCRIPT_DIR'/apps/omni.bloky.test_ext.kit'")
        string(APPEND _EXT_ARGS " --empty --portable --enable omni.kit.test")
        string(APPEND _EXT_ARGS " --/app/enableStdoutOutput=0")
        string(APPEND _EXT_ARGS " --/exts/omni.kit.test/testExts/0='${EXT_NAME}'")
        if(ARG_TEST_ARGS)
            string(APPEND _EXT_ARGS " \$@")
            string(APPEND _EXT_ARGS "${_EXTRA_ARGS}")
        endif()

        set(_CONTENT "#!/bin/bash\nset -e\nSCRIPT_DIR=$(dirname \${BASH_SOURCE})\n")
        string(APPEND _CONTENT "${OVEXTS_LINUX_ASAN_PREAMBLE}\n")
        string(APPEND _CONTENT "\${EXEC:-exec} \"\$SCRIPT_DIR/kit/kit\"  ${_EXT_ARGS}")
        if(NOT ARG_TEST_ARGS)
            string(APPEND _CONTENT " \"\$@\"")
        else()
            string(APPEND _CONTENT " \"\$@\"")
        endif()
        string(APPEND _CONTENT "\n")

        file(GENERATE OUTPUT "${OVEXTS_BIN_DIR}/${_SCRIPT_NAME}.sh"
            CONTENT "${_CONTENT}"
            FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                             GROUP_READ GROUP_EXECUTE
                             WORLD_READ WORLD_EXECUTE)
    endif()

    message(STATUS "  Test script: ${_SCRIPT_NAME}")
endfunction()

# ---------------------------------------------------------------------------
# ovexts_define_etm_test_experience(<ext_name>)
#
# Generates an ETM test runner script that delegates to repo.sh/repo.bat:
#   tests-etm-<ext_name>.sh (Linux)
#   tests-etm-<ext_name>.bat (Windows)
# ---------------------------------------------------------------------------
function(ovexts_define_etm_test_experience EXT_NAME)
    set(_SCRIPT_NAME "tests-etm-${EXT_NAME}")

    if(WIN32)
        set(_CONTENT "@echo off\r\nsetlocal\r\n")
        string(APPEND _CONTENT "call \"%~dp0..\\..\\..\\repo.bat\" etm -e ${EXT_NAME} %*\r\n")

        file(GENERATE OUTPUT "${OVEXTS_BIN_DIR}/${_SCRIPT_NAME}.bat"
            CONTENT "${_CONTENT}")
    else()
        set(_CONTENT "#!/bin/bash\nset -e\nSCRIPT_DIR=$(dirname \${BASH_SOURCE})\n")
        string(APPEND _CONTENT "\${EXEC:-exec} \"\$SCRIPT_DIR/../../../repo.sh\" etm -e ${EXT_NAME} \"\$@\"\n")

        file(GENERATE OUTPUT "${OVEXTS_BIN_DIR}/${_SCRIPT_NAME}.sh"
            CONTENT "${_CONTENT}"
            FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                             GROUP_READ GROUP_EXECUTE
                             WORLD_READ WORLD_EXECUTE)
    endif()

    message(STATUS "  ETM test script: ${_SCRIPT_NAME}")
endfunction()
