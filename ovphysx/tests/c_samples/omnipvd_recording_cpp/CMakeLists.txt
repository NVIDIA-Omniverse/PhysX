cmake_minimum_required(VERSION 3.16)
project(OmniPvdRecordingCpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ovphysx REQUIRED)

add_executable(omnipvd_recording_cpp main.cpp)

target_link_libraries(omnipvd_recording_cpp PRIVATE ovphysx::ovphysx)
get_filename_component(OVPHYSX_TEST_DATA_DIR "${CMAKE_CURRENT_LIST_DIR}/../../data" ABSOLUTE)
target_compile_definitions(omnipvd_recording_cpp PRIVATE
    OVPHYSX_TEST_DATA="${OVPHYSX_TEST_DATA_DIR}"
)
if(WIN32)
    ovphysx_copy_runtime_dlls(omnipvd_recording_cpp)
endif()
