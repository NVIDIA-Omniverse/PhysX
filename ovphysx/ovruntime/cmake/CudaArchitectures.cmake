# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# Run before this project's CUDA language discovery. Otherwise CMake's compiler
# default can look like a caller override when CudaUtils is included later.
# Preserve every nonempty caller value and the established distribution default.
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR "${CMAKE_CUDA_ARCHITECTURES}" STREQUAL "")
    set(CMAKE_CUDA_ARCHITECTURES 70 75 80 86 89 90 100 120)
endif()
set(OVRUNTIME_CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}")
