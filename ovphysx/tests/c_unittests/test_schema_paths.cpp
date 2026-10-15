// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-OVSTAGE-SCHEMA-001
 * @covers AC-1 AC-2 AC-3 AC-4
 */
#include <gtest/gtest.h>
#include "../c_samples/common/schema_support.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace {
class ExternalSchemaProvider : public testing::Test
{
protected:
    std::filesystem::path root = std::filesystem::temp_directory_path() /
        ("ovphysx_external_schemas_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    physx_sample_schema_paths_t paths{};
    char error[PHYSX_SAMPLE_SCHEMA_PATH_SIZE]{};

    void SetUp() override
    {
        for (const char* module : {"PhysxSchema", "OmniUsdPhysicsDeformableSchema"})
        {
            const std::filesystem::path resources = root / "physx_usd_schemas" / module / "resources";
            std::filesystem::create_directories(resources);
            std::ofstream(resources / "plugInfo.json") << "{}";
            std::ofstream(resources / "generatedSchema.usda") << "#usda 1.0";
        }
    }
    void TearDown() override
    {
        std::error_code ignored;
        std::filesystem::remove_all(root, ignored);
    }
    bool discover()
    {
        return physx_sample_schema_paths(root.string().c_str(), &paths, error, sizeof(error)) != 0;
    }
};
} // namespace

TEST_F(ExternalSchemaProvider, FreshAndRelocatedInstallation)
{
    ASSERT_TRUE(discover()) << error;
    EXPECT_EQ(std::filesystem::path(paths.resources[0]), root / "physx_usd_schemas/PhysxSchema/resources");
    EXPECT_EQ(paths.lengths[0], std::string(paths.resources[0]).size());
    EXPECT_EQ(paths.lengths[1], std::string(paths.resources[1]).size());
    const std::string moved = root.string() + "_moved";
    std::filesystem::rename(root, moved);
    root = moved;
    ASSERT_TRUE(discover()) << error;
    EXPECT_EQ(std::filesystem::path(paths.resources[1]),
              root / "physx_usd_schemas/OmniUsdPhysicsDeformableSchema/resources");
}

TEST_F(ExternalSchemaProvider, MissingProviderFailsBeforePopulation)
{
    EXPECT_FALSE(physx_sample_schema_paths(nullptr, &paths, error, sizeof(error)));
    EXPECT_NE(std::string(error).find("PHYSX_USD_SCHEMAS_ROOT"), std::string::npos);
    const std::string missing = (root / "missing").string();
    EXPECT_FALSE(physx_sample_schema_paths(missing.c_str(), &paths, error, sizeof(error)));
    EXPECT_NE(std::string(error).find("reinstall physx-usd-schemas"), std::string::npos);
    EXPECT_TRUE(discover()) << error;
}

TEST_F(ExternalSchemaProvider, IncompleteProviderNamesMissingFile)
{
    std::filesystem::remove(root / "physx_usd_schemas/PhysxSchema/resources/generatedSchema.usda");
    EXPECT_FALSE(discover());
    EXPECT_NE(std::string(error).find("generatedSchema.usda"), std::string::npos);
    EXPECT_NE(std::string(error).find("reinstall"), std::string::npos);
}

TEST(SchemaPaths, SeparatelyInstalledProviderContainsBothModules)
{
    physx_sample_schema_paths_t paths{};
    char error[PHYSX_SAMPLE_SCHEMA_PATH_SIZE]{};
    ASSERT_TRUE(physx_sample_schema_paths(std::getenv("PHYSX_USD_SCHEMAS_ROOT"), &paths, error, sizeof(error))) << error;
    EXPECT_TRUE(std::filesystem::is_regular_file(std::filesystem::path(paths.resources[0]) / "generatedSchema.usda"));
    EXPECT_TRUE(std::filesystem::is_regular_file(std::filesystem::path(paths.resources[1]) / "generatedSchema.usda"));
}
