## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

include("${CMAKE_CURRENT_LIST_DIR}/runtime_closure.cmake")
if(DEFINED CAPTURE_DIR)
    file(MAKE_DIRECTORY "${CAPTURE_DIR}")
    file(GLOB beforeCaptures "${CAPTURE_DIR}/record-errors-*.ovd")
    set(workArgs WORKING_DIRECTORY "${CAPTURE_DIR}")
endif()
execute_process(COMMAND "${EXECUTABLE}" ${ARGS}
    ${workArgs}
    RESULT_VARIABLE code OUTPUT_VARIABLE output ERROR_VARIABLE diagnostic)
if(NOT "${code}" STREQUAL "0")
    message(FATAL_ERROR "${EXECUTABLE} failed: ${code}\n${output}\n${diagnostic}")
endif()
if(DEFINED CAPTURE_DIR)
    file(GLOB afterCaptures "${CAPTURE_DIR}/record-errors-*.ovd")
    if(NOT beforeCaptures STREQUAL afterCaptures)
        message(FATAL_ERROR "Successful recorder oracle left a new capture: ${afterCaptures}")
    endif()
endif()
