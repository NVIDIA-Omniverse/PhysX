// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/ClientUtils.h>
#include <carb/filesystem/IFileSystem.h>
#include <carb/logging/Logger.h>
#include <carb/windowing/IWindowing.h>
#include <omni/str/Wildcard.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace carb
{

constexpr char kAppDirectoryToBuildDirectory[] = "/../..";

// Error and fatal, plus leaks that are reported as warning.
static const char* kMatchIncludeFailurePatterns[] = {
    "*[error]*", "*[fatal]*", "*[warning]*Leaking graphics objects. Live device refcount*",
};

constexpr char kFabricSceneDelegate[] = "FSD";
constexpr char kUsdImagingDelegate[] = "usdImaging";
constexpr char kPreferFabricSceneDelegate[] = "preferFSD";
constexpr char kGoldenServiceLogExt[] = ".golden-service.log";

/**
 * A structure to specify wildcard search patterns to include or exclude.
 */
struct WildcardPattern
{
    const char** includePatterns; ///< An array of wildcard patterns to match. cannot be nullptr. For a default pattern
                                  ///< use kMatchIncludeFailurePatterns and CARB_COUNTOF(kMatchIncludeFailurePatterns)
                                  ///< for includePatternCount.
    uint64_t includePatternCount; ///< The size of includePatterns array.

    const char** excludePatterns; ///< An array of wildcard patterns to exclude, or nullptr.
    uint64_t excludePatternCount; ///< The size of excludePatterns array.

    /**
     * A helper to return default values to initialize WildcardPattern.
     */
    static WildcardPattern getDefaults()
    {
        WildcardPattern patterns = {};
        patterns.includePatterns = kMatchIncludeFailurePatterns;
        patterns.includePatternCount = (uint64_t)CARB_COUNTOF(kMatchIncludeFailurePatterns);
        return patterns;
    }
};

/**
 * Test Session global settings. Set with test.unit CLI, use "-h" for more info.
 */
struct TestGlobalSettings
{
    bool carbLogEnabled = true; ///< Determines if logging is enabled for unit tests. Enabled by default, to capture any
                                ///< error as a failure in FrameworkScoped.
    int32_t carbLogLevel = logging::kLevelWarn; ///< ILogging level, if enabled.
    bool carbLogAll = false; ///< Determines if extra logging settings should be applied.
    bool generateGoldenImages = false; ///< Generate golden images for all the visual tests.
                                       ///< it does not control dumping for [executable] tests.
    bool generateGoldenFailureOnly = false; ///< Generate golden image only for tests that fail comparison.
    bool kitWaitForDebugger = false; ///< Run kit process (if test runs any) with a flag to wait for debugger attach.
    bool skipVulkan = false; ///< Skips running Vulkan tests, if enabled.
    bool skipDirect3D12 = false; ///< Skips running Direct3D 12 tests, if enabled.
    std::string subTestNames = ""; ///< sub tests to run in a test group
    std::string subTestNamesExclude = ""; ///< sub tests to exclude running in a test group
    std::string extraAppSettings; ///< Passes a series of settings to apps like UsdTestViewer launched in RTX tests.
    std::string outputRoot; ///< root folder for all output files

    bool noWindow = false; ///< Run usdTestViewer in offline mode
    bool useKit = false; ///< Use the kit binary rather than usdTestViewer executable
    std::string sceneDelegate = kPreferFabricSceneDelegate; ///< Which scene delegate to use to run the unit tests. PreferFSD
    /// means use FSD where it is known to be working otherwise fall back to usdImaging.

    /// Returns the singleton instance.
    /// Getter MUST remain constant, no test outside main.cpp should modify the values.
    ///
    /// NOTE: This requires an explicit const_cast<> to make sure only option parsing step at the start up of test.unit
    /// is allowed to modify these values and no test will modify global setting for any reason at runtime.
    static const TestGlobalSettings& get()
    {
        static TestGlobalSettings instance;
        return instance;
    }
};

enum class TestAssetType
{
    eNone, ///< Invalid type, such as a temporary folder somewhere outside assets folder.
    eShader, ///< Shader folder inside assets.
    eTexture, ///< texture folder inside assets.
    eImageComparison, ///< image comparison folder inside assets.
    eUsd ///< USD files inside assets.
};

enum class TestAssetDirectoryType
{
    ePackageRoot, ///< The main root folder that contains _build, data, etc. as sub-folders.
    eBuildRoot, ///< Build folder, which is "_build".
    eDataRoot, ///< "data" folder.
    eImageComparison, ///< Golden images for comparisons inside assets folder.
    eImageComparisonOutput, ///< Temporary "outputs" folder that images will be dumped into.
    eFullBuildTarget, ///< Full path to build target, which is either "debug" or "release" folder.
};

std::string getAssetUriInDataSource(TestAssetType assetType, const char* filename = nullptr);
std::string getAssetDirectory(std::string appDirectoryPath, TestAssetDirectoryType assetType);
std::string getRelativeUrlPath(std::string baseDirPath, std::string pathUrl);

/**
 * Converts a string to acceptable ##teamcity format rules. (e.g. "[" to "|[", etc.)
 */
std::string convertToTeamCityFormat(const std::string& strInput);

/*
 * Appends cmdLine with what is passed to the unit test via command line arguments.
 *
 * NOTE: No more changes to cmdLine should be made after this call.
 * extraAppSettings must be at the end of cmdLine to override previous settings, if any.
 */
void addExtraAppSettingsTestArgs(std::string& cmdLine);

/**
 * A class to help reading log files and to find matching wildcard patterns such as "*[error]*" and "*[fatal]*".
 */
class MatchLogPattern
{
public:
    /**
     * Deep copy of pattern strings are performed and strings are converted to lower cases.
     *
     * @wildcardPattern  The pattern to include or exclude.
     */
    MatchLogPattern(const WildcardPattern& wildcardPattern);

    /**
     * Extracts log file path by searching for /log/file setting in cmdLine, without any double quotations in the path.
     * cmdLine must contain a settings to log file location to be successful.
     *
     * @retruns true if it was successful.
     */
    static bool getLogFileFromCommandLine(const std::string& cmdLine, std::string& outLogPath);

    /**
     * Find matching wildcardPattern in logLine string. logLine can be any log message or a line of kit's .log file that
     * will be lower cased. If isMatched is true, no includePatterns matching is performed on the logLine (assumes the
     * logLine is already verified as a match) and instead only the excludePatterns are examined to reject the match.
     */
    bool matchPatterns(const char* logLine, bool isMatched = false);

    /**
     * Find matching pattern by log level first, then match by wildcardPattern.
     * Used when logLine does not contain [level] or any tag/info other than error message.
     */
    bool matchPatterns(const char* logLine, int32_t logLevel);

    /**
     * Find matching pattern by log level only.
     */
    bool matchLogLevel(int32_t logLevel);

    /**
     * Find matching wildcardPattern by loading the log file located at logPath and returning the matches cases in
     * outMatchedFailures (separated by \n) and the count of them in outMatchedFailureCount. log per line will be lower
     * cased.
     */
    bool matchPatternsInLogFile(const std::string& logPath, std::string& outMatched, uint64_t& outMatchedCount);

    /**
     * Extracts the specified /log/file path from cmdLine and then performs matchPatternsInLogFile() on it.
     */
    bool matchPatternsInCmdLineLogFile(const std::string& cmdLine, std::string& outMatched, uint64_t& outMatchedCount);

private:
    void deepCopyLowerCasePatterns(const char** patterns,
                                   uint64_t patternCount,
                                   std::vector<const char*>& outPatterns,
                                   std::vector<std::string>& outPatternStr);

    // Members
    std::vector<const char*> m_includePatterns; ///< deep copied flattened patterns to include in lower case,
                                                ///< pointers in m_includePatternStr.
    std::vector<const char*> m_excludePatterns; ///< deep copied flattened patterns to exclude in lower case, pointers
                                                ///< in m_includePatternStr.
    std::vector<std::string> m_includePatternStr; ///< Deep copy of m_includePatterns strings.
    std::vector<std::string> m_excludePatternStr; ///< Deep copy of m_excludePatterns strings.
    std::vector<int32_t> m_includeLogLevels; ///< Converts log [level] patterns of  m_includePatterns to logLevel
                                             ///< integers. Used in general unit test, where we only have access to
                                             ///< message and logLevel and not the log file.
};

} // namespace carb
