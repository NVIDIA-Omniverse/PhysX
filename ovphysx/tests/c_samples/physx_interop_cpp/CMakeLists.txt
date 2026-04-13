cmake_minimum_required(VERSION 3.16)
project(PhysxInteropCpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ovphysx REQUIRED)

add_executable(physx_interop_cpp main.cpp)

target_link_libraries(physx_interop_cpp PRIVATE ovphysx::ovphysx)

# PhysX SDK headers shipped with the ovphysx SDK (headers only, no library linking needed)
target_include_directories(physx_interop_cpp PRIVATE ${ovphysx_PHYSX_INCLUDE_DIR})

get_filename_component(OVPHYSX_TEST_DATA_DIR "${CMAKE_CURRENT_LIST_DIR}/../../data" ABSOLUTE)
target_compile_definitions(physx_interop_cpp PRIVATE
    OVPHYSX_TEST_DATA="${OVPHYSX_TEST_DATA_DIR}"
)
if(WIN32)
    ovphysx_copy_runtime_dlls(physx_interop_cpp)
endif()
