// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "carb/ClientUtils.h"
#include "common/TestHelpers.h"
#include "physics/PhysicsTools.h"

#if CARB_COMPILER_MSC
#    pragma warning(push)
#    pragma warning(disable : 4267) // disable conversion loss of data warning
#endif
#include <cxxopts/include/cxxopts.hpp>
#if CARB_COMPILER_MSC
#    pragma warning(pop)
#endif

#define DOCTEST_CONFIG_IMPLEMENT // we will be supplying main()
#include <doctest/doctest.h>
#include <omni/core/Omni.h>

OMNI_APP_GLOBALS("test.unit", "Omniverse physics unit tests.")

namespace
{

// carb::logging::stringToLevel does not handle errors..
struct
{
    int32_t level;
    const char* const name;
} carbLogLevels[] = {
    { carb::logging::kLevelVerbose, "verbose" }, { carb::logging::kLevelInfo, "info" },
    { carb::logging::kLevelWarn, "warn" },       { carb::logging::kLevelError, "error" },
    { carb::logging::kLevelFatal, "fatal" },
};

bool parseCarbLogLevel(const std::string& str, int32_t& output)
{
    for (auto& level : carbLogLevels)
    {
        if (str == std::to_string(level.level) || str == level.name)
        {
            output = level.level;
            return true;
        }
    }
    return false;
}

std::string carbLogLevelToString(int32_t value)
{
    for (auto& level : carbLogLevels)
    {
        if (value == level.level)
            return level.name;
    }
    CARB_ASSERT(false);
    return "warn";
}

}

int main(int argc, char** argv)
{
    cxxopts::Options options(argv[0], "\nUnit test");
    auto addOption = options.allow_unrecognised_options().add_options();

#if defined(_WIN32) && defined(_DEBUG)
    bool enableDebugHeap(false);
    addOption("debug-heap", "Use Debug Heap as provided by VC C runtime", cxxopts::value<bool>(enableDebugHeap));
#endif
    bool flaky = false;
    bool enablePVD = false;
    bool disableAssertDialog = false;

    // It's TestGlobalSettings::get documentation that is stating we must const_cast.
    auto testGlobals = const_cast<carb::TestGlobalSettings&>(carb::TestGlobalSettings::get());
    testGlobals.carbLogEnabled = true;

    int repeatCount = 0;

    addOption("g,carb-log", "Enable Carbonite Framework logging.",
              cxxopts::value<bool>(testGlobals.carbLogEnabled));
    addOption("w,wait-kit-debugger", "Run kit process (if test runs any) with a flag to wait for debugger attach.",
              cxxopts::value<bool>(testGlobals.kitWaitForDebugger));
    addOption("flaky", "Tests marked as [flaky] are disabled by default. This option enables them.",
              cxxopts::value<bool>(flaky));
    addOption("pvd", "Enable PVD support.",
              cxxopts::value<bool>(enablePVD));
    addOption("no-assert-dialog", "Disable assert window.",
              cxxopts::value<bool>(disableAssertDialog));
    std::string carbLogLevel = carbLogLevelToString(testGlobals.carbLogLevel);
    std::string carbLogLevelList;
    for (auto& level : carbLogLevels)
    {
        if (!carbLogLevelList.empty())
            carbLogLevelList += "|";
        carbLogLevelList += level.name;
    }
    addOption("carb-log-level",
              "Set Carbonite Framework logging log level. Default is " + carbLogLevel +
                  ". Use with --carb-log. Valid values: " + carbLogLevelList,
              cxxopts::value<std::string>(carbLogLevel));
    addOption("carb-golden", "Generate golden comparison images.",
              cxxopts::value<bool>(testGlobals.generateGoldenImages));
    addOption("carb-golden-failure", "Generate golden comparison image only for tests that fail comparison.",
              cxxopts::value<bool>(testGlobals.generateGoldenFailureOnly));
    addOption("carb-no-vulkan", "Skips running Vulkan tests.",
              cxxopts::value<bool>(testGlobals.skipVulkan));
    addOption("carb-no-d3d12", "Skips running Direct3D 12 tests.",
              cxxopts::value<bool>(testGlobals.skipDirect3D12));
    addOption("repeat", "Sets the number of times to repeat tests. Set to negative to repeat infinitely.",
              cxxopts::value<int>(repeatCount));

    bool showHelpFlag(false);
    addOption("h,help", "Shows help", cxxopts::value<bool>(showHelpFlag));

    // Parse the options
    try
    {
        // Make a copy of argc/argv since cxxopts eats the ones it understands (like help)
        int vargc = argc;
        std::unique_ptr<char*[]> pargv(new char*[argc]);
        memcpy(pargv.get(), argv, sizeof(argv[0]) * argc);
        char** vargv = pargv.get();
        options.parse(vargc, vargv);
    }
    catch (...)
    {
    }

    if (!parseCarbLogLevel(carbLogLevel, testGlobals.carbLogLevel))
        std::cout << "Unrecognized option for --carb-log-level: " << carbLogLevel << std::endl;

#if defined(_WIN32) && defined(_DEBUG)
    if (enableDebugHeap)
    {
        // See https://docs.microsoft.com/en-us/visualstudio/debugger/crt-debug-heap-details?view=vs-2017
        int tmpFlag = _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG);
        tmpFlag |= _CRTDBG_CHECK_ALWAYS_DF | _CRTDBG_CHECK_CRT_DF;
        _CrtSetDbgFlag(tmpFlag);
    }
#endif

    PhysicsTest* physicsTestGlobal = PhysicsTest::getPhysicsTests();
    physicsTestGlobal->enablePVD(enablePVD);

    if (disableAssertDialog)
    {
        carb::assert::disableDialog(true);
    }

    ++repeatCount;
    int result = EXIT_SUCCESS;
    bool const forever = repeatCount <= 0;

    for (int i = 0; i < repeatCount || forever; ++i)
    {
        doctest::Context context;
        context.applyCommandLine(argc, argv);

        if (!flaky)
            context.addFilter("test-case-exclude", "*[flaky]*");

        result = context.run();
        if (result != EXIT_SUCCESS)
            break;
    }

    if (showHelpFlag)
    {
        std::cout << "Additional options: " << std::endl;
        std::cout << options.help();
        std::cout << std::endl
                  << "NOTE: wildcards are needed when specifying a test case filter. Example:" << std::endl
                  << argv[0] << " -tc=\"*[rtx]*\"" << std::endl;
    }

    physicsTestGlobal->release();

    return result;
}
