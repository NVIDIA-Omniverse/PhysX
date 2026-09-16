## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

# A macro that will return the nvcc arguments for code generation depending on the architecture
# Example: GENERATE_ARCH_CODE_LIST(SASS "89" PTX "90") will return:
# --generate-code=arch=compute_89,code=[compute_89,sm_89];--generate-code=arch=compute_90,code=[compute_90,compute_90]
macro(GENERATE_ARCH_CODE_LIST)
    cmake_parse_arguments(GENERATE_ARCH_CODE_LIST "" "SASS;PTX" "" ${ARGN})

    set(ARCH_CODE_LIST "")

    if (GENERATE_ARCH_CODE_LIST_SASS)
        string(REPLACE "," ";" sass_archs "${GENERATE_ARCH_CODE_LIST_SASS}")
        foreach (arch IN LISTS sass_archs)
			set(ARCH_CODE_LIST "${ARCH_CODE_LIST}--generate-code=arch=compute_${arch},code=[compute_${arch},sm_${arch}];")
        endforeach ()
    endif ()

    if (GENERATE_ARCH_CODE_LIST_PTX)
        foreach (arch IN LISTS GENERATE_ARCH_CODE_LIST_PTX)
			set(ARCH_CODE_LIST "${ARCH_CODE_LIST}--generate-code=arch=compute_${arch},code=[compute_${arch},compute_${arch}];")
        endforeach ()
    endif ()

    set(ARCH_CODE_LIST ${ARCH_CODE_LIST} CACHE INTERNAL "")
endmacro()