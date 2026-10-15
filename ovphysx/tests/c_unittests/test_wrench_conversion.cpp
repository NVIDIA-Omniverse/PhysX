// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Unit test for the AoS->SoA wrench conversion used by the session rigid-body wrench write. Independent of
// any data-movement API; lived in test_tensor_binding.cpp only incidentally and moved here so it survives
// that file's removal with the tensor binding.

#include <gtest/gtest.h>
#include <cstdint>

#include "WrenchConversion.h"

TEST(WrenchConversion, AosToSoaCpu)
{
    // AoS row layout: [fx,fy,fz, tx,ty,tz, px,py,pz]
    const int64_t N = 2;
    const float aos[N * 9] = {
        1.f, 2.f, 3.f,   4.f, 5.f, 6.f,   7.f, 8.f, 9.f,
        10.f, 11.f, 12.f, 13.f, 14.f, 15.f, 16.f, 17.f, 18.f,
    };

    float soa[N * 9] = {};
    ovphysx::internal::convertWrenchAosToSoaCpu(aos, N, soa);

    // forces
    EXPECT_FLOAT_EQ(soa[0], 1.f);
    EXPECT_FLOAT_EQ(soa[1], 2.f);
    EXPECT_FLOAT_EQ(soa[2], 3.f);
    EXPECT_FLOAT_EQ(soa[3], 10.f);
    EXPECT_FLOAT_EQ(soa[4], 11.f);
    EXPECT_FLOAT_EQ(soa[5], 12.f);

    // torques
    EXPECT_FLOAT_EQ(soa[6], 4.f);
    EXPECT_FLOAT_EQ(soa[7], 5.f);
    EXPECT_FLOAT_EQ(soa[8], 6.f);
    EXPECT_FLOAT_EQ(soa[9], 13.f);
    EXPECT_FLOAT_EQ(soa[10], 14.f);
    EXPECT_FLOAT_EQ(soa[11], 15.f);

    // positions
    EXPECT_FLOAT_EQ(soa[12], 7.f);
    EXPECT_FLOAT_EQ(soa[13], 8.f);
    EXPECT_FLOAT_EQ(soa[14], 9.f);
    EXPECT_FLOAT_EQ(soa[15], 16.f);
    EXPECT_FLOAT_EQ(soa[16], 17.f);
    EXPECT_FLOAT_EQ(soa[17], 18.f);
}
