## Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
## Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0



include(FetchContent)
FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(pvddom_tests test_pvddom.cpp)
target_link_libraries(pvddom_tests PRIVATE PvdDom gtest_main)

add_test(NAME pvddom_tests COMMAND pvddom_tests)

# OVD file generator tool
add_executable(generate_test_ovd generate_test_ovd.cpp)
target_link_libraries(generate_test_ovd PRIVATE PvdDom)

# Standalone OVD command-stream decoder (diagnostic tool)
add_executable(dump_ovd dump_ovd.cpp)
target_link_libraries(dump_ovd PRIVATE PvdDom)
