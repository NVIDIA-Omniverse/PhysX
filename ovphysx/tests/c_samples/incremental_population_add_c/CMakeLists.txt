cmake_minimum_required(VERSION 3.16)
project(IncrementalPopulationAddC C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(ovphysx REQUIRED)

add_executable(incremental_population_add_c main.c)

# Explicitly set the language to C (not C++)
set_source_files_properties(main.c PROPERTIES LANGUAGE C)
set_target_properties(incremental_population_add_c PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(incremental_population_add_c PRIVATE
        -Werror=implicit-function-declaration
    )
endif()

if(MSVC)
    # MSVC: Compile as C code explicitly
    target_compile_options(incremental_population_add_c PRIVATE /TC)
endif()

target_link_libraries(incremental_population_add_c PRIVATE ovphysx::ovphysx ovphysx::ovstage)
target_include_directories(incremental_population_add_c PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../common")
get_filename_component(OVPHYSX_TEST_DATA_DIR "${CMAKE_CURRENT_LIST_DIR}/../../data" ABSOLUTE)
target_compile_definitions(incremental_population_add_c PRIVATE
    OVPHYSX_TEST_DATA="${OVPHYSX_TEST_DATA_DIR}"
)
if(WIN32)
    ovphysx_copy_runtime_dlls(incremental_population_add_c)
endif()
