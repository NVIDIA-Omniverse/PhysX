cmake_minimum_required(VERSION 3.16)

function(_assert_runtime_deps_config build_type use_release_deps expected_config)
    set(BUILD_TYPE "${build_type}")
    unset(ENV{OVPHYSX_USE_RELEASE_RUNTIME_DEPS})
    if("${use_release_deps}" STREQUAL "UNSET")
        unset(OVPHYSX_USE_RELEASE_RUNTIME_DEPS)
    else()
        set(OVPHYSX_USE_RELEASE_RUNTIME_DEPS "${use_release_deps}")
    endif()

    include("${CMAKE_CURRENT_LIST_DIR}/build_common.cmake")

    if(NOT OVPHYSX_RUNTIME_DEPS_CONFIG STREQUAL "${expected_config}")
        message(FATAL_ERROR
            "Expected BUILD_TYPE=${build_type}, OVPHYSX_USE_RELEASE_RUNTIME_DEPS=${use_release_deps} "
            "to select runtime deps config '${expected_config}', got '${OVPHYSX_RUNTIME_DEPS_CONFIG}'")
    endif()
endfunction()

_assert_runtime_deps_config(Debug UNSET release)
_assert_runtime_deps_config(Debug ON release)
_assert_runtime_deps_config(Release ON release)
_assert_runtime_deps_config(RelWithDebInfo UNSET release)

execute_process(
    COMMAND "${CMAKE_COMMAND}"
        -DBUILD_TYPE=Debug
        -DOVPHYSX_USE_RELEASE_RUNTIME_DEPS=OFF
        -P "${CMAKE_CURRENT_LIST_DIR}/build_common.cmake"
    RESULT_VARIABLE _debug_runtime_result
    OUTPUT_VARIABLE _debug_runtime_output
    ERROR_VARIABLE _debug_runtime_error
)
if(_debug_runtime_result EQUAL 0)
    message(FATAL_ERROR "True-Debug runtime dependency selection unexpectedly succeeded")
endif()
string(CONCAT _debug_runtime_log "${_debug_runtime_output}" "${_debug_runtime_error}")
if(NOT _debug_runtime_log MATCHES "True-Debug runtime dependencies are unsupported")
    message(FATAL_ERROR
        "True-Debug runtime dependency selection failed for the wrong reason:\n${_debug_runtime_log}")
endif()

# install.cmake imports this setting from the configured build after including
# build_common.cmake. Exercise that later path so a cached Debug/OFF selection
# cannot bypass the Release-runtime-only policy.
set(_install_cache_build_dir "_build_runtime_deps_config_test")
set(_install_cache_build_path "${CMAKE_CURRENT_LIST_DIR}/../${_install_cache_build_dir}")
file(REMOVE_RECURSE "${_install_cache_build_path}")
file(MAKE_DIRECTORY "${_install_cache_build_path}")
file(WRITE "${_install_cache_build_path}/CMakeCache.txt"
    "OVPHYSX_USE_RELEASE_RUNTIME_DEPS:BOOL=OFF\n")
execute_process(
    COMMAND "${CMAKE_COMMAND}"
        -DBUILD_TYPE=Debug
        -DBUILD_DIR=${_install_cache_build_dir}
        -P "${CMAKE_CURRENT_LIST_DIR}/install.cmake"
    RESULT_VARIABLE _debug_install_result
    OUTPUT_VARIABLE _debug_install_output
    ERROR_VARIABLE _debug_install_error
)
file(REMOVE_RECURSE "${_install_cache_build_path}")
if(_debug_install_result EQUAL 0)
    message(FATAL_ERROR "Debug install with cached true-Debug runtime selection unexpectedly succeeded")
endif()
string(CONCAT _debug_install_log "${_debug_install_output}" "${_debug_install_error}")
if(NOT _debug_install_log MATCHES "True-Debug runtime dependencies are unsupported")
    message(FATAL_ERROR
        "Debug install with cached true-Debug runtime selection failed for the wrong reason:\n"
        "${_debug_install_log}")
endif()
