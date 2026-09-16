// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-OVSTAGE-SCHEMA-001
 * @covers AC-1 AC-2 AC-3
 */

#include <gtest/gtest.h>

#include "ovphysx/ovphysx.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr const char* kNamespacedPluginPathEnv = "OV_PXR_PLUGINPATH_2511";
constexpr const char* kClassicPluginPathEnv = "PXR_PLUGINPATH_NAME";

void setEnv(const char* name, const std::string& value)
{
#ifdef _WIN32
    _putenv_s(name, value.c_str());
#else
    setenv(name, value.c_str(), 1);
#endif
}

void unsetEnv(const char* name)
{
#ifdef _WIN32
    _putenv_s(name, "");
#else
    unsetenv(name);
#endif
}

std::string getEnvString(const char* name)
{
    const char* value = std::getenv(name);
    return value ? value : "";
}

std::string toString(const ovphysx_string_t& value)
{
    return std::string(value.ptr ? value.ptr : "", value.ptr ? value.length : 0);
}

void writeEmptyFile(const std::filesystem::path& path)
{
    std::filesystem::create_directories(path.parent_path());
    std::ofstream(path.string()).put('\0');
}

// <root>/lib/libovphysx.so next to <root>/schemas/physx/plugInfo.json: the SDK
// and wheel layout.
std::filesystem::path makeFakeSdk(const std::filesystem::path& base)
{
    const std::filesystem::path root = base / "sdk";
#ifdef _WIN32
    const std::filesystem::path libPath = root / "lib" / "ovphysx.dll";
#else
    const std::filesystem::path libPath = root / "lib" / "libovphysx.so";
#endif
    writeEmptyFile(libPath);
    writeEmptyFile(root / "schemas" / "physx" / "plugInfo.json");
    return libPath;
}

// <app>/libovphysx.so next to <app>/schemas/physx/plugInfo.json: a runtime copied
// beside an application. An unrelated but complete schemas/physx registry one
// level above the application must not win.
std::filesystem::path makeFakeCopiedRuntime(const std::filesystem::path& base)
{
    const std::filesystem::path root = base / "app";
    writeEmptyFile(base / "schemas" / "physx" / "plugInfo.json");
#ifdef _WIN32
    const std::filesystem::path libPath = root / "ovphysx.dll";
#else
    const std::filesystem::path libPath = root / "libovphysx.so";
#endif
    writeEmptyFile(libPath);
    writeEmptyFile(root / "schemas" / "physx" / "plugInfo.json");
    return libPath;
}

// Restores OVPHYSX_LIB and the USD plugin-path variables after each test, so a
// fake layout never leaks into the tests that run against the installed SDK.
class ScopedSchemaPathEnv
{
public:
    ScopedSchemaPathEnv()
        : m_ovphysxLib(getEnvString("OVPHYSX_LIB"))
        , m_namespacedPluginPath(getEnvString(kNamespacedPluginPathEnv))
        , m_classicPluginPath(getEnvString(kClassicPluginPathEnv))
        , m_hadOvphysxLib(std::getenv("OVPHYSX_LIB") != nullptr)
        , m_hadNamespacedPluginPath(std::getenv(kNamespacedPluginPathEnv) != nullptr)
        , m_hadClassicPluginPath(std::getenv(kClassicPluginPathEnv) != nullptr)
    {
    }

    ~ScopedSchemaPathEnv()
    {
        restore("OVPHYSX_LIB", m_hadOvphysxLib, m_ovphysxLib);
        restore(kNamespacedPluginPathEnv, m_hadNamespacedPluginPath, m_namespacedPluginPath);
        restore(kClassicPluginPathEnv, m_hadClassicPluginPath, m_classicPluginPath);
    }

private:
    static void restore(const char* name, bool hadValue, const std::string& value)
    {
        if (hadValue)
        {
            setEnv(name, value);
        }
        else
        {
            unsetEnv(name);
        }
    }

    std::string m_ovphysxLib;
    std::string m_namespacedPluginPath;
    std::string m_classicPluginPath;
    bool m_hadOvphysxLib;
    bool m_hadNamespacedPluginPath;
    bool m_hadClassicPluginPath;
};

} // namespace

TEST(SchemaPaths, InstalledSdkShipsCodelessSchemaRoot)
{
    // Against the real installed SDK the tests run from: the root exists, holds
    // the include registry, and holds the shipped PhysX schema modules.
    ovphysx_string_t root{};
    const ovphysx_result_t result = ovphysx_get_codeless_schema_root(&root);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << toString(ovphysx_get_last_error());
    ASSERT_NE(root.ptr, nullptr);
    ASSERT_GT(root.length, 0u);
    EXPECT_EQ(root.ptr[root.length], '\0');

    const std::filesystem::path rootPath(toString(root));
    EXPECT_TRUE(std::filesystem::is_regular_file(rootPath / "plugInfo.json"));
    EXPECT_TRUE(std::filesystem::is_regular_file(rootPath / "PhysxSchema" / "resources" / "plugInfo.json"));
    EXPECT_TRUE(std::filesystem::is_regular_file(rootPath / "PhysxSchema" / "resources" / "generatedSchema.usda"));
    EXPECT_TRUE(std::filesystem::is_regular_file(
        rootPath / "OmniUsdPhysicsDeformableSchema" / "resources" / "plugInfo.json"));
}

TEST(SchemaPaths, QueryDoesNotTouchUsdEnvironment)
{
    // ovphysx tells the application where the schemas are and never registers
    // them, so neither USD plugin-path variable may change.
    ScopedSchemaPathEnv env;
    setEnv(kNamespacedPluginPathEnv, "sentinel-namespaced");
    setEnv(kClassicPluginPathEnv, "sentinel-classic");

    ovphysx_string_t root{};
    ASSERT_EQ(ovphysx_get_codeless_schema_root(&root).status, OVPHYSX_API_SUCCESS)
        << toString(ovphysx_get_last_error());

    EXPECT_EQ(getEnvString(kNamespacedPluginPathEnv), "sentinel-namespaced");
    EXPECT_EQ(getEnvString(kClassicPluginPathEnv), "sentinel-classic");
}

TEST(SchemaPaths, ResolvesSdkLayoutFromOvphysxLib)
{
    ScopedSchemaPathEnv env;
    const std::filesystem::path libPath =
        makeFakeSdk(std::filesystem::temp_directory_path() / "ovphysx_schema_paths_sdk");
    const std::filesystem::path expected =
        std::filesystem::weakly_canonical(libPath.parent_path().parent_path() / "schemas" / "physx");

    setEnv("OVPHYSX_LIB", libPath.string());

    ovphysx_string_t root{};
    ASSERT_EQ(ovphysx_get_codeless_schema_root(&root).status, OVPHYSX_API_SUCCESS)
        << toString(ovphysx_get_last_error());
    EXPECT_EQ(toString(root), expected.string());
    EXPECT_EQ(ovphysx_get_last_error().length, 0u);
}

TEST(SchemaPaths, ResolvesCopiedRuntimeLayout)
{
    ScopedSchemaPathEnv env;
    const std::filesystem::path libPath =
        makeFakeCopiedRuntime(std::filesystem::temp_directory_path() / "ovphysx_schema_paths_copied");
    const std::filesystem::path expected =
        std::filesystem::weakly_canonical(libPath.parent_path() / "schemas" / "physx");

    setEnv("OVPHYSX_LIB", libPath.string());

    ovphysx_string_t root{};
    ASSERT_EQ(ovphysx_get_codeless_schema_root(&root).status, OVPHYSX_API_SUCCESS)
        << toString(ovphysx_get_last_error());
    EXPECT_EQ(toString(root), expected.string());
}

TEST(SchemaPaths, MissingSchemasFailWithActionableErrorAndRetry)
{
    ScopedSchemaPathEnv env;
    const std::filesystem::path base = std::filesystem::temp_directory_path() / "ovphysx_schema_paths_missing";
#ifdef _WIN32
    const std::filesystem::path badLib = base / "bad" / "lib" / "ovphysx.dll";
#else
    const std::filesystem::path badLib = base / "bad" / "lib" / "libovphysx.so";
#endif
    writeEmptyFile(badLib);
    setEnv("OVPHYSX_LIB", badLib.string());

    ovphysx_string_t root{ "stale", 5 };
    const ovphysx_result_t failed = ovphysx_get_codeless_schema_root(&root);
    ASSERT_EQ(failed.status, OVPHYSX_API_ERROR);
    ASSERT_NE(root.ptr, nullptr);
    EXPECT_EQ(root.length, 0u);
    const std::string error = toString(ovphysx_get_last_error());
    EXPECT_NE(error.find("schemas/physx/plugInfo.json"), std::string::npos) << error;
    EXPECT_NE(error.find("OVPHYSX_LIB"), std::string::npos) << error;

    // Nothing is memoized: pointing at a complete layout succeeds on the next call.
    const std::filesystem::path goodLib = makeFakeSdk(base / "good");
    setEnv("OVPHYSX_LIB", goodLib.string());
    ASSERT_EQ(ovphysx_get_codeless_schema_root(&root).status, OVPHYSX_API_SUCCESS)
        << toString(ovphysx_get_last_error());
    EXPECT_GT(root.length, 0u);
    EXPECT_EQ(ovphysx_get_last_error().length, 0u);
}

TEST(SchemaPaths, NullOutputIsRejected)
{
    const ovphysx_result_t result = ovphysx_get_codeless_schema_root(nullptr);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_GT(ovphysx_get_last_error().length, 0u);
}

TEST(SchemaPaths, ConcurrentQueriesReturnTheSameRoot)
{
    ScopedSchemaPathEnv env;
    const std::filesystem::path libPath =
        makeFakeSdk(std::filesystem::temp_directory_path() / "ovphysx_schema_paths_concurrent");
    const std::filesystem::path expected =
        std::filesystem::weakly_canonical(libPath.parent_path().parent_path() / "schemas" / "physx");
    setEnv("OVPHYSX_LIB", libPath.string());

    std::vector<std::string> roots(8);
    std::vector<ovphysx_api_status_t> statuses(8, OVPHYSX_API_ERROR);
    std::vector<std::thread> threads;
    for (size_t i = 0; i < statuses.size(); ++i)
    {
        threads.emplace_back([&roots, &statuses, i]() {
            ovphysx_string_t root{};
            statuses[i] = ovphysx_get_codeless_schema_root(&root).status;
            roots[i] = toString(root);
        });
    }
    for (std::thread& thread : threads)
    {
        thread.join();
    }

    for (size_t i = 0; i < statuses.size(); ++i)
    {
        EXPECT_EQ(statuses[i], OVPHYSX_API_SUCCESS);
        EXPECT_EQ(roots[i], expected.string());
    }
}
