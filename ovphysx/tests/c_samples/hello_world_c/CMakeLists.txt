cmake_minimum_required(VERSION 3.16)
project(HelloWorldC C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(ovphysx REQUIRED)

add_executable(hello_world_c main.c)

# Explicitly set the language to C (not C++)
set_source_files_properties(main.c PROPERTIES LANGUAGE C)
set_target_properties(hello_world_c PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)


# Add compiler-specific flags to enforce strict C compilation
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    # GCC/Clang: Add -Werror=implicit-function-declaration to catch C++ leakage
    target_compile_options(hello_world_c PRIVATE 
        -Werror=implicit-function-declaration
        -pedantic
    )
endif()

if(MSVC)
    # MSVC: Compile as C code explicitly
    target_compile_options(hello_world_c PRIVATE /TC)
endif()

target_link_libraries(hello_world_c PRIVATE ovphysx::ovphysx)
get_filename_component(OVPHYSX_TEST_DATA_DIR "${CMAKE_CURRENT_LIST_DIR}/../../data" ABSOLUTE)
target_compile_definitions(hello_world_c PRIVATE
    OVPHYSX_TEST_DATA="${OVPHYSX_TEST_DATA_DIR}"
)
if(WIN32)
    ovphysx_copy_runtime_dlls(hello_world_c)
endif()

