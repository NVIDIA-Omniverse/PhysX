// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
#include "OvStageOutputCuda.h"
#include "OvStageOutputMath.h"
#include <cstdio>
#include <string>

#define CHECK(value) do { if (!(value)) return __LINE__; } while (false)

int main()
{
    using namespace ovphysx::utils::detail;
    static_assert(sizeof(void*) == 8, "This test validates the memory64 target");
    std::string error;
    OutputCudaContextScope cpu;
    CHECK(cpu.active());
    CHECK(cpu.cuda() == nullptr);
    CHECK(cpu.restore(error));
    CHECK(error.empty());

    // Exercise the original CPU composition used by OvStageOutput.cpp.
    const float position[3] = { 2.0f, -3.0f, 4.0f };
    const float quaternion[4] = { 0.0f, 0.0f, 0.0f, 2.0f };
    const float scale[3] = { -2.0f, 3.0f, 4.0f };
    double matrix[16]{};
    CHECK(composeWorldMatrix(position, quaternion, scale, matrix));
    CHECK(matrix[0] == -2.0 && matrix[5] == 3.0 && matrix[10] == 4.0);
    CHECK(matrix[12] == 2.0 && matrix[13] == -3.0 && matrix[14] == 4.0 && matrix[15] == 1.0);
    const float zeroQuaternion[4]{};
    CHECK(!composeWorldMatrix(position, zeroQuaternion, scale, matrix));

    OutputCudaBuffers gpu;
    CHECK(!gpu.initialize(0, 1, error));
    CHECK(error.find("no CUDA device") != std::string::npos);
    CHECK(gpu.matrices() == nullptr);
    ovstage_read_column_t column{};
    error.clear();
    CHECK(!gpu.compose(column, column, column, error));
    CHECK(error.find("no CUDA device") != std::string::npos);
    CHECK(gpu.matrices() == nullptr);

    OutputCudaContextScope gpuScope(&gpu);
    CHECK(!gpuScope.active());
    CHECK(gpuScope.cuda() == nullptr);
    error.clear();
    CHECK(!gpuScope.restore(error));
    CHECK(error.find("no CUDA device") != std::string::npos);

    OutputCudaContextScope cpuAfterFailure(nullptr);
    CHECK(cpuAfterFailure.active());
    CHECK(cpuAfterFailure.restore(error));
    std::puts("PASS original CPU matrix composition; CUDA output fails without allocating or publishing data");
    return 0;
}
