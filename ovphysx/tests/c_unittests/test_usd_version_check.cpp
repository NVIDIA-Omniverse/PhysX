// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

/**
 * @file test_usd_version_check.cpp
 * @brief Unit tests for USD version detection and compatibility checking
 */

#include <gtest/gtest.h>
#include "global_test_environment.h"
#include "UsdVersionCheck/UsdVersionCheck.h"
#include <string>
#include <fstream>
#include <algorithm>
#include <cctype>
#include <filesystem>
#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

using namespace omni::sdk::usd_version;

namespace {

bool isInstalledUsdMonolith(const std::filesystem::path& path)
{
    std::string name = path.filename().string();
    std::transform(name.begin(), name.end(), name.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
#ifdef _WIN32
    return name.rfind("ov_", 0) == 0 &&
           name.find("usd_ms") != std::string::npos &&
           name.size() >= 4 &&
           name.substr(name.size() - 4) == ".dll";
#else
    return name.rfind("libov_", 0) == 0 &&
           name.find("usd_ms") != std::string::npos &&
           name.find(".so") != std::string::npos;
#endif
}

std::filesystem::path findInstalledUsdMonolith()
{
    std::error_code ec;
    const std::filesystem::path pluginsDir = std::filesystem::path(OVPHYSX_INSTALL_DIR) / "plugins";
    if (!std::filesystem::is_directory(pluginsDir, ec))
        return {};

    for (const auto& entry : std::filesystem::directory_iterator(pluginsDir, ec))
    {
        if (!ec && entry.is_regular_file(ec) && isInstalledUsdMonolith(entry.path()))
            return entry.path();
    }
    return {};
}

bool loadSharedLibrary(const std::filesystem::path& path, std::string& error)
{
#ifdef _WIN32
    HMODULE module = LoadLibraryA(path.string().c_str());
    if (module != nullptr)
        return true;
    error = "LoadLibraryA failed with error " + std::to_string(GetLastError());
    return false;
#else
    void* handle = dlopen(path.string().c_str(), RTLD_NOW | RTLD_LOCAL);
    if (handle != nullptr)
        return true;
    const char* dlerrorText = dlerror();
    error = dlerrorText ? dlerrorText : "dlopen failed";
    return false;
#endif
}

::testing::AssertionResult requireCarboniteRuntime()
{
    if (ensureSharedCpuInstance())
        return ::testing::AssertionSuccess();

    ovphysx_string_t error = ovphysx_get_last_error();
    if (error.ptr && error.length > 0)
    {
        return ::testing::AssertionFailure()
            << "Carbonite runtime must be available for UsdVersionCheck config tests: "
            << std::string(error.ptr, error.length);
    }
    return ::testing::AssertionFailure()
        << "Carbonite runtime must be available for UsdVersionCheck config tests";
}

} // namespace

TEST(UsdVersionCheck, ExtractUsdVersionFromNamespacedMonolithPaths) {
    EXPECT_EQ(extractUsdVersion("/opt/ov/libov_25.11usd_ms.so"), "25.11");
    EXPECT_EQ(extractUsdVersion("C:\\ov\\ov_25.11usd_ms.dll"), "25.11");
    EXPECT_EQ(extractUsdVersion("/opt/pixar/usd-23.11/lib/libusd_tf.so"), "unknown");
}

TEST(UsdVersionCheck, DetectUsdFindsAlreadyLoadedNamespacedMonolith) {
    const std::filesystem::path usdMonolith = findInstalledUsdMonolith();
    ASSERT_FALSE(usdMonolith.empty()) << "Installed namespaced USD monolith not found under _install/plugins";

    std::string loadError;
    ASSERT_TRUE(loadSharedLibrary(std::filesystem::absolute(usdMonolith), loadError)) << loadError;
    const std::string expectedVersion = extractUsdVersion(usdMonolith.string());
    ASSERT_FALSE(expectedVersion.empty());
    ASSERT_NE(expectedVersion, "unknown");

    UsdDetectionResult result = detectUsdInProcess();
    ASSERT_TRUE(result.is_loaded);
    EXPECT_EQ(result.version, expectedVersion);
    EXPECT_NE(result.library_path.find("_install"), std::string::npos);
}

TEST(UsdVersionCheck, ExtractPackmanPackageId_Match) {
    std::string path = "/home/u/.cache/packman/chk/usd.py312.manylinux_2_35_x86_64.stock.release/0.25.11.kit.2-gl.19811/lib/libusd_tf.so";
    EXPECT_EQ(extractPackmanPackageId(path), "0.25.11.kit.2-gl.19811");
}

TEST(UsdVersionCheck, ExtractPackmanPackageId_DirectoryPath) {
    // The call site in checkUsdCompatibility passes detection.library_path, which
    // is the *directory* containing the library (getDirectoryFromPath strips the
    // filename). Verify the walk succeeds without the trailing filename component.
    std::string dir = "/home/u/.cache/packman/chk/usd.py312.manylinux_2_35_x86_64.stock.release/0.25.11.kit.2-gl.19811/lib";
    EXPECT_EQ(extractPackmanPackageId(dir), "0.25.11.kit.2-gl.19811");
}

TEST(UsdVersionCheck, ExtractPackmanPackageId_NonPackmanPath) {
    // Regression: in 0.4.0 a missing fixpoint-break in the parent walk caused
    // this input to spin forever. Must return "" quickly.
    EXPECT_EQ(extractPackmanPackageId("/opt/pixar/usd-25.11/lib/libusd_tf.so"), "");
    EXPECT_EQ(extractPackmanPackageId(""), "");
    // ovrtx-style flat bin/plugins/ layout: no `chk` ancestor anywhere.
    EXPECT_EQ(extractPackmanPackageId("/home/u/.venv/lib/python3.12/site-packages/ovrtx/bin/plugins/libusd_tf.so"), "");
    // Short path near root shouldn't hang either.
    EXPECT_EQ(extractPackmanPackageId("/libusd_tf.so"), "");
    EXPECT_EQ(extractPackmanPackageId("libusd_tf.so"), "");
}

TEST(UsdVersionCheck, CheckCompatibility_SimpleRange) {
    EXPECT_TRUE(checkCompatibility("25.11", "==25.11"));
    EXPECT_FALSE(checkCompatibility("25.10", "==25.11"));
    EXPECT_FALSE(checkCompatibility("25.12", "==25.11"));
}

TEST(UsdVersionCheck, CheckCompatibility_WithExclusion) {
    EXPECT_TRUE(checkCompatibility("25.11", "==25.11,!=25.10"));
    EXPECT_FALSE(checkCompatibility("25.10", "==25.11,!=25.10"));
}

TEST(UsdVersionCheck, CheckCompatibility_ExactMatch) {
    EXPECT_TRUE(checkCompatibility("25.11", "==25.11"));
    EXPECT_FALSE(checkCompatibility("25.10", "==25.11"));
}

TEST(UsdVersionCheck, CheckCompatibility_UnknownVersion) {
    // Unknown version should be allowed (conservative approach with warning)
    EXPECT_TRUE(checkCompatibility("unknown", "==25.11"));
}

TEST(UsdVersionCheck, CheckCompatibility_PatchVersions) {
    EXPECT_TRUE(checkCompatibility("25.11.5", "~=25.11.0"));
    EXPECT_FALSE(checkCompatibility("25.12.0", "~=25.11.0"));
}

TEST(UsdVersionCheck, CheckCompatibility_GreaterThan) {
    EXPECT_TRUE(checkCompatibility("25.11", ">=25.11"));
    EXPECT_FALSE(checkCompatibility("25.10", ">=25.11"));
}

TEST(UsdVersionCheck, CheckCompatibility_LessThanOrEqual) {
    EXPECT_TRUE(checkCompatibility("25.11", "<=25.11"));
    EXPECT_TRUE(checkCompatibility("25.10", "<=25.11"));
    EXPECT_FALSE(checkCompatibility("25.12", "<=25.11"));
}

// Test compatible release operator (~=) per PEP 440
TEST(UsdVersionCheck, CheckCompatibility_CompatibleRelease_MajorMinor) {
    // ~=25.11 means >=25.11, <26.0
    EXPECT_TRUE(checkCompatibility("25.11", "~=25.11"));   // Exact match
    EXPECT_TRUE(checkCompatibility("25.12", "~=25.11"));   // Within same major
    EXPECT_TRUE(checkCompatibility("25.99", "~=25.11"));   // Within same major
    EXPECT_FALSE(checkCompatibility("25.10", "~=25.11"));  // Below minimum
    EXPECT_FALSE(checkCompatibility("26.0", "~=25.11"));   // Next major version
    EXPECT_FALSE(checkCompatibility("24.99", "~=25.11"));  // Previous major
}

TEST(UsdVersionCheck, CheckCompatibility_CompatibleRelease_WithPatch) {
    // ~=25.11.5 means >=25.11.5, <25.12.0
    EXPECT_TRUE(checkCompatibility("25.11.5", "~=25.11.5"));   // Exact match
    EXPECT_TRUE(checkCompatibility("25.11.6", "~=25.11.5"));   // Higher patch
    EXPECT_TRUE(checkCompatibility("25.11.99", "~=25.11.5"));  // Much higher patch
    EXPECT_FALSE(checkCompatibility("25.11.4", "~=25.11.5"));  // Lower patch
    EXPECT_FALSE(checkCompatibility("25.12.0", "~=25.11.5"));  // Next minor version
    EXPECT_FALSE(checkCompatibility("25.10.9", "~=25.11.5"));  // Previous minor
    EXPECT_FALSE(checkCompatibility("26.0.0", "~=25.11.5"));   // Next major version
}

TEST(UsdVersionCheck, CheckCompatibility_CompatibleRelease_EdgeCases) {
    // Test version boundaries
    EXPECT_TRUE(checkCompatibility("25.11", "~=25.11"));      // Exact at boundary
    EXPECT_TRUE(checkCompatibility("25.12", "~=25.11"));      // Just above
    EXPECT_FALSE(checkCompatibility("25.10", "~=25.11"));     // Just below
    EXPECT_FALSE(checkCompatibility("26.0", "~=25.11"));      // At upper bound
    
    // Test with zero patch versions
    EXPECT_TRUE(checkCompatibility("25.11.0", "~=25.11.0"));
    EXPECT_TRUE(checkCompatibility("25.11.1", "~=25.11.0"));
    EXPECT_FALSE(checkCompatibility("25.12.0", "~=25.11.0"));
}

// Note: These tests require Carbonite framework to be initialized
TEST(UsdVersionCheck, LoadConfig_ValidFile) {
    ASSERT_TRUE(requireCarboniteRuntime());

    std::string config_path = "tests/configs/valid_config.toml";
    
    std::ifstream test(config_path);
    ASSERT_TRUE(test.good()) << "Config file not found: " << config_path;
    
    // Reset cached config to allow testing in isolation
    shutdownUsdVersionCheck();
    
    LibraryConfig config;
    bool success = loadConfig(config_path, config);
    
    ASSERT_TRUE(success) << "Config loading failed";
    EXPECT_EQ(config.name, "test.library");
    EXPECT_EQ(config.version, "1.0.0");
    EXPECT_EQ(config.description, "Test library for unit testing");
    EXPECT_TRUE(config.usd.required);
    EXPECT_EQ(config.usd.version_spec, "==25.11");
}

TEST(UsdVersionCheck, LoadConfig_MinimalFile) {
    ASSERT_TRUE(requireCarboniteRuntime());

    std::string config_path = "tests/configs/minimal_config.toml";
    
    std::ifstream test(config_path);
    ASSERT_TRUE(test.good()) << "Config file not found: " << config_path;
    
    // Reset cached config to allow testing in isolation
    shutdownUsdVersionCheck();
    
    LibraryConfig config;
    bool success = loadConfig(config_path, config);
    
    ASSERT_TRUE(success) << "Config loading failed";
    EXPECT_EQ(config.name, "minimal.library");
    EXPECT_FALSE(config.usd.required);
}

TEST(UsdVersionCheck, LoadConfig_NonExistentFile) {
    ASSERT_TRUE(requireCarboniteRuntime());

    // Reset cached config to test actual error handling
    shutdownUsdVersionCheck();
    
    LibraryConfig config;
    bool success = loadConfig("/nonexistent/config.toml", config);
    EXPECT_FALSE(success);
}

TEST(UsdVersionCheck, LoadConfig_InvalidSyntax) {
    ASSERT_TRUE(requireCarboniteRuntime());

    std::string config_path = "tests/configs/invalid_syntax.toml";
    
    std::ifstream test(config_path);
    ASSERT_TRUE(test.good()) << "Config file not found: " << config_path;
    
    // Reset cached config to test actual syntax error handling
    shutdownUsdVersionCheck();
    
    LibraryConfig config;
    bool success = loadConfig(config_path, config);
    EXPECT_FALSE(success);
}

// Test USD detection (will depend on whether USD is actually loaded)
TEST(UsdVersionCheck, DetectUsd_ReturnsValidResult) {
    UsdDetectionResult result = detectUsdInProcess();
    
    if (result.is_loaded) {
        EXPECT_FALSE(result.version.empty());
    } else {
        EXPECT_TRUE(result.version.empty() || result.version == "");
    }
}

TEST(UsdVersionCheck, FormatCompatibilityError_ContainsRelevantInfo) {
    LibraryConfig config;
    config.name = "ovphysx";
    config.version = "0.1.0";
    config.usd.version_spec = "==25.11";
    
    std::string error = formatCompatibilityError("21.8", "/opt/usd-21.8", config);
    
    EXPECT_NE(error.find("ovphysx"), std::string::npos);
    EXPECT_NE(error.find("21.8"), std::string::npos);
    EXPECT_NE(error.find("INCOMPATIBLE"), std::string::npos);
    EXPECT_NE(error.find("==25.11"), std::string::npos);
    EXPECT_NE(error.find("/opt/usd-21.8"), std::string::npos);
}

TEST(UsdVersionCheck, GetLibraryDirectory_ReturnsValidPath) {
    std::string lib_dir = getLibraryDirectory();
    
    EXPECT_FALSE(lib_dir.empty());
    
    // Unix: starts with '/', Windows: starts with drive letter and ':'
    EXPECT_TRUE(lib_dir[0] == '/' || (lib_dir.size() > 1 && lib_dir[1] == ':'));
}

TEST(UsdVersionCheck, FindConfigFile_FoundInRepoRoot) {
    ASSERT_TRUE(requireCarboniteRuntime());

    // Assumes we're running from the build directory, where config.toml
    // exists at the repo root and findConfigFile locates it relative to
    // the library location (parent directory in a dev build).
    std::string config_path;
    
    bool found = findConfigFile(config_path);
    
    ASSERT_TRUE(found) << "Config file not found - requires Carbonite filesystem";
    EXPECT_FALSE(config_path.empty());
    EXPECT_NE(config_path.find("config.toml"), std::string::npos);
}

TEST(UsdVersionCheck, InitializeShutdown_NoThrow) {
    ASSERT_TRUE(requireCarboniteRuntime());
    shutdownUsdVersionCheck();

    ovphysx_result_t result = initializeUsdVersionCheck();
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);
    shutdownUsdVersionCheck();
}

TEST(UsdVersionCheck, ShutdownWithoutInit_NoThrow) {
    shutdownUsdVersionCheck();
    // Should not crash
    EXPECT_TRUE(true);
}
