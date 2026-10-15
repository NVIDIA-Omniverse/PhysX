// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-LEGACY-001
 * @covers AC-2 AC-3 AC-5
 */

#include <fstream>
#include <iterator>
#include <string>

#include <gtest/gtest.h>

#include "ovphysx/ovphysx.h"

static_assert(sizeof(ovphysx_scene_query_hit_t) == 64,
              "ovphysx_scene_query_hit_t layout is part of the public ABI");

TEST(LegacyHeader, UmbrellaExposesMovedSymbol)
{
    EXPECT_NE(reinterpret_cast<void*>(&ovphysx_create_tensor_binding), nullptr);
    EXPECT_NE(reinterpret_cast<void*>(&ovphysx_create_sdf_view), nullptr);
}

TEST(LegacyHeader, NoIncludeCyclesOrTypeLeak)
{
    auto read = [](const char* rel) {
        std::ifstream in(std::string(OVPHYSX_SOURCE_DIR) + rel);
        EXPECT_TRUE(in) << rel;
        return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    };

    const std::string primary = read("/include/ovphysx/ovphysx.h");
    const std::string legacy = read("/include/ovphysx/ovphysx_legacy.h");
    const std::string types = read("/include/ovphysx/ovphysx_types.h");

    EXPECT_NE(primary.find("#include \"ovphysx/ovphysx_legacy.h\""), std::string::npos);
    EXPECT_EQ(legacy.find("\n#include \"ovphysx/ovphysx.h\""), std::string::npos);
    EXPECT_EQ(legacy.find("\n#include <ovphysx/ovphysx.h>"), std::string::npos);
    EXPECT_EQ(types.find("ovphysx_legacy_types.h"), std::string::npos);
    EXPECT_NE(primary.find("ovphysx_result_t ovphysx_warmup("), std::string::npos);
    EXPECT_EQ(legacy.find("ovphysx_result_t ovphysx_warmup("), std::string::npos);
}
