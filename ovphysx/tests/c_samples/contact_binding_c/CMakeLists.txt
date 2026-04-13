cmake_minimum_required(VERSION 3.16)
project(ContactBindingC C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(ovphysx REQUIRED)

add_executable(contact_binding_c main.c)

set_source_files_properties(main.c PROPERTIES LANGUAGE C)
set_target_properties(contact_binding_c PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED ON
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(contact_binding_c PRIVATE
        -Werror=implicit-function-declaration
        -pedantic
    )
endif()

if(MSVC)
    target_compile_options(contact_binding_c PRIVATE /TC)
endif()

target_link_libraries(contact_binding_c PRIVATE ovphysx::ovphysx)
get_filename_component(OVPHYSX_TEST_DATA_DIR "${CMAKE_CURRENT_LIST_DIR}/../../data" ABSOLUTE)
target_compile_definitions(contact_binding_c PRIVATE
    OVPHYSX_TEST_DATA="${OVPHYSX_TEST_DATA_DIR}"
)
if(WIN32)
    ovphysx_copy_runtime_dlls(contact_binding_c)
endif()
