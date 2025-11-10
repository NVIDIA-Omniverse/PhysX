// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/ClientUtils.h>
#include <carb/extras/EnvironmentVariable.h>
#include <carb/extras/Path.h>
#include <carb/filesystem/IFileSystem.h>
#include <carb/logging/Logger.h>
#include <carb/windowing/IWindowing.h>

#include "common/test/TestCommonUtils.h"

#include <carb/settings/ISettings.h>

#include <omni/kit/IApp.h>
#include <omni/core/Omni.h>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

// This is the main include for doctest
// Note that global config defines are specified in premake5.lua
#include <doctest/doctest.h>

// Compatibility shim allowing a Catch2-style interface with doctest
// See here: https://github.com/onqtam/doctest/blob/master/doc/markdown/faq.md#how-is-doctest-different-from-catch
#define SECTION(name) DOCTEST_SUBCASE(std::string(name).c_str())
#undef TEST_CASE
#define TEST_CASE(name, tags) DOCTEST_TEST_CASE(tags " " name)
#undef SCENARIO
#define SCENARIO(name, tags) DOCTEST_SCENARIO(tags " " name)
using doctest::Approx;

namespace carb
{
    constexpr const char* const kGraphenePluginsSearchPaths[] =
    {
        "../exts",
        "../kit",
        "../kit/plugins/carb_gfx",
        "../kit/plugins",
        "../kit/kernel/plugins",        
        "../kit/extscore/omni.assets.plugins/bin"
    };

inline extras::Path getCarbAppPath(filesystem::IFileSystem* fs)
{
    extras::Path carbAppPath = fs->getExecutableDirectoryPath();
    // Temp hack to remove "../plugins":
    if (carbAppPath.getStem() == "plugins")
    {
        carbAppPath = carbAppPath.getParent();
        carbAppPath /= "kit";
    }

    return carbAppPath;
}

class FrameworkScoped
{
public:
    FrameworkScoped()
    {
        m_framework = acquireFrameworkAndRegisterBuiltins();
        // Remove default logger:
        logging::ILogging* ls = logging::getLogging();

        ls->setLevelThreshold(carb::logging::kLevelWarn);

        // Remove default logger (to avoid TC/console spam) unless logging was not explicitly enabled:
        if (!TestGlobalSettings::get().carbLogEnabled)
            ls->removeLogger(ls->getDefaultLogger()->getLogger());
    }

    ~FrameworkScoped()
    {
        releaseFrameworkAndDeregisterBuiltins();
    }

    operator Framework*()
    {
        return m_framework;
    }

    Framework* getFramework()
    {
        return m_framework;
    }

    Framework* operator->()
    {
        return m_framework;
    }

    void loadPlugins(const std::vector<const char*>& loadedFileWildcards)
    {
        PluginLoadingDesc desc = PluginLoadingDesc::getDefault();
        desc.loadedFileWildcards = loadedFileWildcards.data();
        desc.loadedFileWildcardCount = loadedFileWildcards.size();
        desc.searchPaths = kGraphenePluginsSearchPaths;
        desc.searchPathCount = CARB_COUNTOF(kGraphenePluginsSearchPaths);
        m_framework->loadPlugins(desc);
    }

private:
    Framework* m_framework;
};

class AppScoped : public FrameworkScoped
{
public:
    AppScoped()
    {
        m_fileSystem = getFramework()->acquireInterface<filesystem::IFileSystem>();
    }

    ~AppScoped()
    {
        m_app->shutdown();
    }

    std::vector<std::string> getEmptyExperienceArgv()
    {
        extras::Path carbAppPath = getCarbAppPath(m_fileSystem);

        std::vector<std::string> argv;
        argv.reserve(3);
        argv.push_back("[executable name]");
        argv.push_back("--empty");

        return argv;
    }

    void startupEmpty()
    {
        std::vector<std::string> argv = getEmptyExperienceArgv();
        startup(argv);
    }

    void startup(const std::vector<std::string>& argv)
    {
        extras::Path carbAppPath = getCarbAppPath(m_fileSystem);
        this->loadPlugins({ "omni.kit.app.plugin" });

        int argc = (int)argv.size();

        std::vector<const char*> argvFlattened;
        argvFlattened.reserve(argc);
        for (int i = 0; i < argc; i++)
            argvFlattened.push_back(argv[i].c_str());

        m_app = getFramework()->acquireInterface<omni::kit::IApp>();
        omni::kit::AppDesc desc;
        desc.carbAppName = "test.unit";
        desc.carbAppPath = carbAppPath.getStringBuffer();
        desc.argc = argc;
        // Dangerous cast, but `kit::AppDesc` for some reason wants non-const chars
        desc.argv = const_cast<char**>(argvFlattened.data());
        m_app->startup(desc);

        // App changes app dir, revert (temp):
        m_fileSystem->setAppDirectoryPath(m_fileSystem->getExecutableDirectoryPath());

        carb::settings::ISettings* settings = getFramework()->acquireInterface<carb::settings::ISettings>();
        settings->setBool("/app/fastShutdown", false);
    }

    omni::kit::IApp* getIApp() const
    {
        return m_app;
    }

    omni::kit::IApp* operator->()
    {
        return getIApp();
    }

    filesystem::IFileSystem* getFileSystem() const
    {
        return m_fileSystem;
    }

private:
    filesystem::IFileSystem* m_fileSystem = nullptr;
    std::string m_experienceConfigPath;
    omni::kit::IApp* m_app;
};

/**
 * Custom Logger which saves all messages to be used to test that logging happened
 */
struct TestLogger : public logging::Logger
{
    TestLogger()
    {
        handleMessage = msg;
    }

    static void msg(Logger* logger,
                    const char* source,
                    int32_t level,
                    const char* filename,
                    const char* functionName,
                    int lineNumber,
                    const char* message)
    {
        TestLogger* thisLogger = static_cast<TestLogger*>(logger);
        std::lock_guard<Lock> g(thisLogger->m_mutex);
        thisLogger->m_msgs[level].push_back(message);
    }

    bool hasMessage(int32_t level, const std::string& subStr)
    {
        std::lock_guard<Lock> g(m_mutex);
        for (const auto& m : m_msgs[level])
            if (m.find(subStr) != std::string::npos)
                return true;
        return false;
    }

    size_t getMessageCount(int32_t level)
    {
        std::lock_guard<Lock> g(m_mutex);
        return m_msgs[level].size();
    }

    size_t getMessageCount(int32_t level, const std::string& subStr)
    {
        std::lock_guard<Lock> g(m_mutex);
        size_t count = 0;
        for (const auto& m : m_msgs[level])
            if (m.find(subStr) != std::string::npos)
                count++;
        return count;
    }

    /**
     * Returns the number of errors and fatal messages.
     */
    size_t getErrorMessageCount()
    {
        std::lock_guard<Lock> g(m_mutex);
        size_t count = 0;
        for (int32_t i = logging::kLevelError; i < logging::kLevelFatal; i++)
        {
            auto value = m_msgs.find(i);
            if (value != m_msgs.end())
            {
                count += value->second.size();
            }
        }

        return count;
    }

    /**
     * Returns the number of debug layer errors that go into "Info" level in carb.graphics.
     * tagged as "D3D12 ERROR:" for DX12, "ERROR:" for Vulkan.
     */
    size_t getValidationLayerErrorMessageCount()
    {
        return getMessageCount(logging::kLevelInfo, "ERROR:");
    }

    size_t getTotalMessageCount() const
    {
        std::lock_guard<Lock> g(m_mutex);
        size_t t = 0;
        for (auto& kv : m_msgs)
            t += kv.second.size();
        return t;
    }

    void clear()
    {
        std::lock_guard<Lock> g(m_mutex);
        m_msgs.clear();
    }

private:
    using Lock = std::mutex;
    mutable Lock m_mutex;
    std::unordered_map<int32_t, std::vector<std::string>> m_msgs;
};

class TestLoggerScoped
{
public:
    TestLoggerScoped()
    {
        logging::getLogging()->addLogger(&m_logger);
    }

    ~TestLoggerScoped()
    {
        logging::getLogging()->removeLogger(&m_logger);
    }

    TestLogger* getLogger()
    {
        return &m_logger;
    }

    TestLogger* operator->()
    {
        return getLogger();
    }

private:
    TestLogger m_logger;
};

/**
 * Custom scoped logger which saves all messages at Info level,
 * including optional debug layer errors in some plugins.
 */
class TestErrorLoggerScoped
{
public:
    TestErrorLoggerScoped()
    {
        logging::ILogging* logSystem = logging::getLogging();
        // At least, Info level is required to capture "ERROR:" or validation layer errors in some plugins
        m_prevLogLevel = logSystem->getLevelThreshold();
        logSystem->setLevelThreshold(logging::kLevelInfo);
        logSystem->addLogger(&m_logger);
    }

    ~TestErrorLoggerScoped()
    {
        logging::ILogging* logSystem = logging::getLogging();
        logSystem->setLevelThreshold(m_prevLogLevel);
        logSystem->removeLogger(&m_logger);
    }

    TestLogger* operator->()
    {
        return &m_logger;
    }

private:
    TestLogger m_logger;
    int32_t m_prevLogLevel;
};

class AutoTempDir
{
    char m_path[1024];
    filesystem::IFileSystem* m_fs;

public:
    AutoTempDir()
    {
        m_fs = getFramework()->acquireInterface<filesystem::IFileSystem>();
        m_fs->makeTempDirectory(m_path, sizeof(m_path));
    }

    ~AutoTempDir()
    {
        m_fs->removeDirectory(m_path);
    }

    const char* getPath() const
    {
        return m_path;
    }

    std::string createFile(const char* filename, const char* stringData)
    {
        std::string filePath = m_path;
        filePath = filePath + "/" + std::string(filename);
        filesystem::File* file = m_fs->openFileToWrite(filePath.c_str());
        m_fs->writeFileLine(file, stringData);
        m_fs->closeFile(file);
        return filePath;
    }
};
}
