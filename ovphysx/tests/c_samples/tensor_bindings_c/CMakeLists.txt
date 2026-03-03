cmake_minimum_required(VERSION 3.16)
project(TensorBindingsC C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(ovphysx REQUIRED)

add_executable(tensor_bindings_c main.c)

# Explicitly set the language to C (not C++)
set_source_files_properties(main.c PROPERTIES LANGUAGE C)
set_target_properties(tensor_bindings_c PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)

# Add compiler-specific flags to enforce strict C compilation
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(tensor_bindings_c PRIVATE
        -Werror=implicit-function-declaration
        -pedantic
    )
endif()

if(MSVC)
    target_compile_options(tensor_bindings_c PRIVATE /TC)
endif()

target_link_libraries(tensor_bindings_c PRIVATE ovphysx::ovphysx)
get_filename_component(OVPHYSX_TEST_DATA_DIR "${CMAKE_CURRENT_LIST_DIR}/../../data" ABSOLUTE)
target_compile_definitions(tensor_bindings_c PRIVATE
    OVPHYSX_TEST_DATA="${OVPHYSX_TEST_DATA_DIR}"
)
if(WIN32)
    ovphysx_copy_runtime_dlls(tensor_bindings_c)
endif()
