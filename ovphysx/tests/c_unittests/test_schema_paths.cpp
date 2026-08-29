// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include "ovphysx/ovphysx.h"
#include "ovphysxTestHelpers.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr const char* kPluginPathEnv = "OV_PXR_PLUGINPATH_2511";
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

std::vector<std::string> splitPaths(const std::string& value)
{
    std::vector<std::string> paths;
    size_t start = 0;
    while (start <= value.size())
    {
        size_t end = value.find(
#ifdef _WIN32
            ';',
#else
            ':',
#endif
            start);
        if (end == std::string::npos)
        {
            end = value.size();
        }
        if (end > start)
        {
            paths.push_back(value.substr(start, end - start));
        }
        if (end == value.size())
        {
            break;
        }
        start = end + 1;
    }
    return paths;
}

std::filesystem::path makeFakeSdk(const std::filesystem::path& base)
{
    const std::filesystem::path root = base / "sdk";
    std::filesystem::create_directories(root / "lib");
    std::filesystem::create_directories(root / "plugins" / "usd");
#ifdef _WIN32
    const std::filesystem::path libPath = root / "lib" / "ovphysx.dll";
#else
    const std::filesystem::path libPath = root / "lib" / "libovphysx.so";
#endif
    std::ofstream(libPath.string()).put('\0');
    return libPath;
}

std::filesystem::path makeFakeCopiedRuntime(const std::filesystem::path& base)
{
    const std::filesystem::path root = base / "app";
    std::filesystem::create_directories(base / "plugins");
    std::filesystem::create_directories(root / "plugins" / "usd");
#ifdef _WIN32
    const std::filesystem::path libPath = root / "ovphysx.dll";
#else
    const std::filesystem::path libPath = root / "libovphysx.so";
#endif
    std::ofstream(libPath.string()).put('\0');
    return libPath;
}

class ScopedSchemaPathEnv
{
public:
    ScopedSchemaPathEnv()
        : m_ovphysxLib(getEnvString("OVPHYSX_LIB"))
        , m_pluginPath(getEnvString(kPluginPathEnv))
        , m_classicPluginPath(getEnvString(kClassicPluginPathEnv))
        , m_hadOvphysxLib(std::getenv("OVPHYSX_LIB") != nullptr)
        , m_hadPluginPath(std::getenv(kPluginPathEnv) != nullptr)
        , m_hadClassicPluginPath(std::getenv(kClassicPluginPathEnv) != nullptr)
    {
        ovphysx_reset_schema_path_registration_internal();
    }

    ~ScopedSchemaPathEnv()
    {
        restore("OVPHYSX_LIB", m_hadOvphysxLib, m_ovphysxLib);
        restore(kPluginPathEnv, m_hadPluginPath, m_pluginPath);
        restore(kClassicPluginPathEnv, m_hadClassicPluginPath, m_classicPluginPath);
        ovphysx_reset_schema_path_registration_internal();
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
    std::string m_pluginPath;
    std::string m_classicPluginPath;
    bool m_hadOvphysxLib;
    bool m_hadPluginPath;
    bool m_hadClassicPluginPath;
};

} // namespace

TEST(SchemaPaths, RegisterSchemaPathsAppendsOvphysxRoot)
{
    ScopedSchemaPathEnv env;
    const std::filesystem::path libPath = makeFakeSdk(std::filesystem::temp_directory_path() / "ovphysx_schema_paths_append");
    const std::filesystem::path usdPluginPath = libPath.parent_path().parent_path() / "plugins" / "usd";

    setEnv("OVPHYSX_LIB", libPath.string());
    unsetEnv(kPluginPathEnv);
    unsetEnv(kClassicPluginPathEnv);

    ovphysx_result_t result = ovphysx_register_schema_paths();

    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(getEnvString(kClassicPluginPathEnv), "");
    EXPECT_EQ(splitPaths(getEnvString(kPluginPathEnv)), std::vector<std::string>{ std::filesystem::weakly_canonical(usdPluginPath).string() });
}

TEST(SchemaPaths, RegisterSchemaPathsPreservesExistingPathsAndDedupes)
{
    ScopedSchemaPathEnv env;
    const std::filesystem::path base = std::filesystem::temp_directory_path() / "ovphysx_schema_paths_dedupe";
    const std::filesystem::path libPath = makeFakeSdk(base);
    const std::filesystem::path usdPluginPath = std::filesystem::weakly_canonical(libPath.parent_path().parent_path() / "plugins" / "usd");
    const std::filesystem::path sentinel = base / "ovrtx_usd_plugins";
    std::filesystem::create_directories(sentinel);

    setEnv("OVPHYSX_LIB", libPath.string());
    setEnv(kPluginPathEnv, sentinel.string());

    ASSERT_EQ(ovphysx_register_schema_paths().status, OVPHYSX_API_SUCCESS);
    ovphysx_reset_schema_path_registration_internal();
    ASSERT_EQ(ovphysx_register_schema_paths().status, OVPHYSX_API_SUCCESS);

    std::vector<std::string> paths = splitPaths(getEnvString(kPluginPathEnv));
    ASSERT_EQ(paths.size(), 2u);
    EXPECT_EQ(paths[0], sentinel.string());
    EXPECT_EQ(paths[1], usdPluginPath.string());
}

TEST(SchemaPaths, RegisterSchemaPathsSupportsCopiedRuntimeLayout)
{
    ScopedSchemaPathEnv env;
    const std::filesystem::path libPath =
        makeFakeCopiedRuntime(std::filesystem::temp_directory_path() / "ovphysx_schema_paths_copied_runtime");
    const std::filesystem::path usdPluginPath =
        std::filesystem::weakly_canonical(libPath.parent_path() / "plugins" / "usd");

    setEnv("OVPHYSX_LIB", libPath.string());
    unsetEnv(kPluginPathEnv);

    ovphysx_result_t result = ovphysx_register_schema_paths();

    ovphysx_string_t error = ovphysx_get_last_error();
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS)
        << std::string(error.ptr ? error.ptr : "", error.ptr ? error.length : 0);
    EXPECT_EQ(splitPaths(getEnvString(kPluginPathEnv)), std::vector<std::string>{ usdPluginPath.string() });
}

TEST(SchemaPaths, RegisterSchemaPathsFailureDoesNotMutateEnvAndCanRetry)
{
    ScopedSchemaPathEnv env;
    const std::filesystem::path base = std::filesystem::temp_directory_path() / "ovphysx_schema_paths_retry";
    const std::filesystem::path badLib = base / "bad" / "lib" / "libovphysx.so";
    std::filesystem::create_directories(badLib.parent_path());
    std::ofstream(badLib.string()).put('\0');

    setEnv("OVPHYSX_LIB", badLib.string());
    setEnv(kPluginPathEnv, "sentinel");

    ovphysx_result_t failed = ovphysx_register_schema_paths();
    ASSERT_EQ(failed.status, OVPHYSX_API_ERROR);
    EXPECT_EQ(getEnvString(kPluginPathEnv), "sentinel");
    ovphysx_string_t error = ovphysx_get_last_error();
    EXPECT_GT(error.length, 0u);

    const std::filesystem::path goodLib = makeFakeSdk(base / "good");
    setEnv("OVPHYSX_LIB", goodLib.string());

    ovphysx_result_t retried = ovphysx_register_schema_paths();
    ASSERT_EQ(retried.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_get_last_error().length, 0u);
    EXPECT_EQ(splitPaths(getEnvString(kPluginPathEnv)).size(), 2u);
}

TEST(SchemaPaths, RegisterSchemaPathsConcurrentCallsDeduped)
{
    ScopedSchemaPathEnv env;
    const std::filesystem::path libPath = makeFakeSdk(std::filesystem::temp_directory_path() / "ovphysx_schema_paths_concurrent");

    setEnv("OVPHYSX_LIB", libPath.string());
    unsetEnv(kPluginPathEnv);

    std::vector<ovphysx_api_status_t> statuses(8, OVPHYSX_API_ERROR);
    std::vector<std::thread> threads;
    for (size_t i = 0; i < statuses.size(); ++i)
    {
        threads.emplace_back([&statuses, i]() {
            statuses[i] = ovphysx_register_schema_paths().status;
        });
    }
    for (std::thread& thread : threads)
    {
        thread.join();
    }

    for (ovphysx_api_status_t status : statuses)
    {
        EXPECT_EQ(status, OVPHYSX_API_SUCCESS);
    }
    EXPECT_EQ(splitPaths(getEnvString(kPluginPathEnv)).size(), 1u);
}
