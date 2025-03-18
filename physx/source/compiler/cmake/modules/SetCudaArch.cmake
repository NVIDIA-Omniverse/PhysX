## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions
## are met:
##  * Redistributions of source code must retain the above copyright
##    notice, this list of conditions and the following disclaimer.
##  * Redistributions in binary form must reproduce the above copyright
##    notice, this list of conditions and the following disclaimer in the
##    documentation and/or other materials provided with the distribution.
##  * Neither the name of NVIDIA CORPORATION nor the names of its
##    contributors may be used to endorse or promote products derived
##    from this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
## EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
## PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
## CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
## EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
## PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
## PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
## OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
## (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
## OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##
## Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.

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