# hello_world_source_link — demonstrates building against the ovphysx source tree
# using add_subdirectory() instead of find_package().
#
# Usage (from the repo root):
#   cmake -S omni/ovphysx/tests/c_samples/hello_world_source_link -B /tmp/sl_test
#   cmake --build /tmp/sl_test
#   /tmp/sl_test/hello_world_source_link
#
# Or with a custom source checkout:
#   cmake -S . -B _build -DOVPHYSX_SOURCE_DIR=/path/to/ovphysx
cmake_minimum_required(VERSION 3.16)
project(HelloWorldSourceLink C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Point at the ovphysx source tree. Default assumes monorepo layout.
if(NOT DEFINED OVPHYSX_SOURCE_DIR)
    get_filename_component(OVPHYSX_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
endif()

if(NOT EXISTS "${OVPHYSX_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
        "ovphysx source not found at ${OVPHYSX_SOURCE_DIR}.\n"
        "Pass -DOVPHYSX_SOURCE_DIR=<path> to point at your ovphysx checkout.")
endif()

# Include ovphysx as a subdirectory. Dependencies are auto-fetched by default
# (OVPHYSX_FETCH_DEPS=ON). Pass -DOVPHYSX_FETCH_DEPS=OFF to skip.
add_subdirectory("${OVPHYSX_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/ovphysx")

add_executable(hello_world_source_link main.c)

# Strict C compilation flags (same as hello_world_c)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(hello_world_source_link PRIVATE
        -Werror=implicit-function-declaration
        -pedantic
    )
endif()
if(MSVC)
    target_compile_options(hello_world_source_link PRIVATE /TC)
endif()

target_link_libraries(hello_world_source_link PRIVATE ovphysx::ovphysx ovphysx::ovstage)
target_include_directories(hello_world_source_link PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../common")

get_filename_component(OVPHYSX_TEST_DATA_DIR
    "${OVPHYSX_SOURCE_DIR}/tests/data" ABSOLUTE)
target_compile_definitions(hello_world_source_link PRIVATE
    OVPHYSX_TEST_DATA="${OVPHYSX_TEST_DATA_DIR}"
)

# Set up runtime library paths so the executable finds ovphysx and its deps.
ovphysx_setup_source_link_runtime(hello_world_source_link)
