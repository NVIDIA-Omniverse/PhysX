set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(TARGET_ABI "linux-gnu")
set(CMAKE_LIBRARY_ARCHITECTURE aarch64-linux-gnu)

set(CMAKE_C_COMPILER aarch64-${TARGET_ABI}-gcc)
set(CMAKE_CXX_COMPILER aarch64-${TARGET_ABI}-g++)

set(CMAKE_FIND_ROOT_PATH "/usr/aarch64-${TARGET_ABI}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

find_program(GCC_LOCATION ${CMAKE_C_COMPILER})
if(NOT GCC_LOCATION)
    message(FATAL_ERROR "Failed to find ${CMAKE_C_COMPILER}")
endif()
