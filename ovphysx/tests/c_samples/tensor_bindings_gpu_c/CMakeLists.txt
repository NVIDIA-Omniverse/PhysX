cmake_minimum_required(VERSION 3.16)
project(TensorBindingsGpuC C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(ovphysx REQUIRED)
find_package(CUDAToolkit QUIET)

add_executable(tensor_bindings_gpu_c main.c)
target_link_libraries(tensor_bindings_gpu_c PRIVATE ovphysx::ovphysx)
get_filename_component(OVPHYSX_TEST_DATA_DIR "${CMAKE_CURRENT_LIST_DIR}/../../data" ABSOLUTE)
target_compile_definitions(tensor_bindings_gpu_c PRIVATE
    OVPHYSX_TEST_DATA="${OVPHYSX_TEST_DATA_DIR}"
)
if(WIN32)
    ovphysx_copy_runtime_dlls(tensor_bindings_gpu_c)
endif()

if(CUDAToolkit_FOUND)
    target_compile_definitions(tensor_bindings_gpu_c PRIVATE OVPHYSX_HAS_CUDA=1)
    target_link_libraries(tensor_bindings_gpu_c PRIVATE CUDA::cudart)
endif()
