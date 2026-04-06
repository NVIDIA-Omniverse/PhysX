cmake_minimum_required(VERSION 3.16)
project(blast_stress_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(blast_stress_demo main.cpp)

# Blast include path
target_include_directories(blast_stress_demo PRIVATE
    ${CMAKE_SOURCE_DIR}/blast/include
)

# Link against Blast libraries
target_link_libraries(blast_stress_demo PRIVATE
    NvBlast
    NvBlastGlobals
    NvBlastTk
    NvBlastExtAuthoring
    NvBlastExtStress
)

# PhysX include paths and libraries (assuming PhysX compiled elsewhere in the workspace)
target_include_directories(blast_stress_demo PRIVATE
    ${CMAKE_SOURCE_DIR}/physx/include
)

target_link_libraries(blast_stress_demo PRIVATE
    PhysX
    PhysXExtensions
    PhysXFoundation
    PhysXCommon
)
