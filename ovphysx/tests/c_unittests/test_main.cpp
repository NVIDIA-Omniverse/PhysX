// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "global_test_environment.h"
#include <filesystem>
#include <iostream>
#include <cstdlib>
#include <string>

namespace fs = std::filesystem;

// Self-bootstrap the test environment so the binary can run standalone from
// PROJECT_ROOT (ovphysx/) after build+install.
// OVPHYSX_INSTALL_DIR is set via a CMake compile definition as a relative path,
// so the binary must be launched with cwd = PROJECT_ROOT.
static void bootstrap_test_environment()
{
    const fs::path installDir(OVPHYSX_INSTALL_DIR);

    // Verify install layout exists — relative paths require cwd = ovphysx/
    if (!fs::is_directory(installDir))
    {
        std::cerr << "ERROR: Install directory not found: " << fs::absolute(installDir) << "\n"
                  << "Make sure you run from ovphysx/ (PROJECT_ROOT) and that\n"
                  << "the SDK is installed: cmake -P scripts/install.cmake\n";
        std::exit(1);
    }
    const fs::path pluginsDir = installDir / "plugins";
    if (!fs::is_directory(pluginsDir))
    {
        std::cerr << "ERROR: Plugins directory not found: " << fs::absolute(pluginsDir) << "\n"
                  << "Run: cmake -P scripts/install.cmake\n";
        std::exit(1);
    }

#ifdef _WIN32
    // On Windows, GetModuleHandleExA may resolve to the test executable instead
    // of ovphysx.dll, so set OVPHYSX_LIB to guide plugin/config discovery to
    // the installed SDK layout. Linux uses dladdr which resolves correctly.
    {
        const fs::path installLib = installDir / "bin" / "ovphysx.dll";
        if (std::getenv("OVPHYSX_LIB") == nullptr && fs::exists(installLib))
        {
            _putenv_s("OVPHYSX_LIB", fs::absolute(installLib).string().c_str());
        }
    }
#endif

    // Windows note: link-time DLLs (ovphysx.dll and dependencies) must be on PATH before
    // process launch — the OS loader resolves them before main(). test_cpp.cmake handles
    // this. This bootstrap only covers env vars needed after the process is already running.
    // For standalone Windows execution, the user must ensure DLLs are on PATH themselves.
    //
    // Linux: no LD_LIBRARY_PATH needed — BUILD_RPATH in CMakeLists.txt already covers
    // _install/lib, _install/plugins, _install/plugins/bin/deps, and Python lib dir.

    // Clear PYTHONPATH to avoid conflicts with installed Python packages.
    // Note: _putenv_s with "" removes the variable on MSVC (equivalent to unsetenv).
#ifdef _WIN32
    _putenv_s("PYTHONPATH", "");
#else
    setenv("PYTHONPATH", "", 1);
#endif
}

class PhysXShutdownEnvironment final : public ::testing::Environment
{
public:
    void SetUp() override
    {
        const std::string filter = GTEST_FLAG_GET(filter);
        if (filter == "GlobalLifecycle.*")
        {
            return;
        }

        ovphysx_result_t r = ovphysx_initialize();
        if (r.status != OVPHYSX_API_SUCCESS)
        {
            std::cerr << "ERROR: ovphysx_initialize failed in test setup\n";
            std::exit(1);
        }
        m_initialized = true;
    }

    void TearDown() override
    {
        destroySharedCpuInstance();
        if (m_initialized)
        {
            // Balance ovphysx_initialize() and clear the process-lifecycle
            // token. The static runtime stays resident until process exit.
            const ovphysx_result_t shutdownResult = ovphysx_shutdown();
            if (shutdownResult.status != OVPHYSX_API_SUCCESS)
            {
                ovphysx_string_t err = ovphysx_get_last_error();
                ADD_FAILURE() << "ovphysx_shutdown() failed; leaked instance(s) may still be alive: "
                              << std::string(err.ptr ? err.ptr : "", err.ptr ? err.length : 0);
            }
        }
    }

private:
    bool m_initialized = false;
};

int main(int argc, char **argv) {
    bootstrap_test_environment();
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new PhysXShutdownEnvironment());
    return RUN_ALL_TESTS();
}
