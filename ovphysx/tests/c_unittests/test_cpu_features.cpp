// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include "ovphysx/ovphysx.h"

#include <string>

#if defined(__x86_64__) || defined(_M_X64) || defined(__amd64__)

TEST(CpuFeatures, GlobalInitializeSucceededOnAvxHost)
{
    // PhysXShutdownEnvironment already called ovphysx_initialize(). If AVX were
    // missing, the process would have exited in that global setup. Do not call
    // ovphysx_shutdown() here; that would break later tests in the full suite.
    EXPECT_EQ(ovphysx_initialize().status, OVPHYSX_API_ERROR);
    ovphysx_string_t err = ovphysx_get_last_error();
    ASSERT_NE(err.ptr, nullptr);
    EXPECT_NE(std::string(err.ptr, err.length).find("already initialized"), std::string::npos);
}

#endif
