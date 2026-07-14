// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//
#pragma once

#include <carb/ClientUtils.h>
#include <carb/extras/EnvironmentVariable.h>
#include <carb/extras/Path.h>
#include <carb/filesystem/IFileSystem.h>
#include <carb/logging/Logger.h>
#include <carb/windowing/IWindowing.h>

#include <carb/settings/ISettings.h>

#include <omni/core/Omni.h>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace carb
{
    // Plugin search paths relative to the test executable directory.
    // The built ovruntime plugins (omni.physx.plugin.so etc.) are in the same directory as the executable.
    // The Carbonite SDK plugins (carb.settings.plugin.so etc.) are resolved via CARB_SDK_SEARCH_PATH.
    // The RTX plugins (carb.stats.plugin.so etc.) are resolved via RTX_PLUGINS_SEARCH_PATH.
    // Both are set by CMake as compile definitions.
    constexpr const char* const kGraphenePluginsSearchPaths[] =
    {
        ".",               // built ovruntime plugins (co-located with executable)
#if defined(CARB_SDK_SEARCH_PATH)
        CARB_SDK_SEARCH_PATH,
#endif
#if defined(RTX_PLUGINS_SEARCH_PATH)
        RTX_PLUGINS_SEARCH_PATH,
#endif
#if defined(RTX_FABRIC_SEARCH_PATH)
        RTX_FABRIC_SEARCH_PATH,
#endif
#if defined(RTX_CUBRIC_SEARCH_PATH)
        RTX_CUBRIC_SEARCH_PATH,
#endif
#if defined(RTX_USD_SEARCH_PATH)
        RTX_USD_SEARCH_PATH,
#endif
#if defined(RTX_SCENEGRAPH_SEARCH_PATH)
        RTX_SCENEGRAPH_SEARCH_PATH,
#endif
#if defined(RTX_USDRT_SEARCH_PATH)
        RTX_USDRT_SEARCH_PATH,
#endif
#if defined(RTX_UJITSO_AGENT_SEARCH_PATH)
        RTX_UJITSO_AGENT_SEARCH_PATH,
#endif
#if defined(RTX_UJITSO_DEFAULT_SEARCH_PATH)
        RTX_UJITSO_DEFAULT_SEARCH_PATH,
#endif
#if defined(RTX_BLOBKEY_SEARCH_PATH)
        RTX_BLOBKEY_SEARCH_PATH,
#endif
#if defined(RTX_DATASTORE_SEARCH_PATH)
        RTX_DATASTORE_SEARCH_PATH,
#endif
#if defined(RTX_GPUCOMPUTE_SEARCH_PATH)
        RTX_GPUCOMPUTE_SEARCH_PATH,
#endif
#if defined(CARB_SDK_SCRIPTING_SEARCH_PATH)
        CARB_SDK_SCRIPTING_SEARCH_PATH,
#endif
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
        eBuildRoot, ///< Build folder, which is "_build".
        eDataRoot, ///< "data" folder.
        eImageComparison, ///< Golden images for comparisons inside assets folder.
        eImageComparisonOutput, ///< Temporary "outputs" folder that images will be dumped into.
        eFullBuildTarget, ///< Full path to build target, which is either "debug" or "release" folder.
        eAssetRoot ///< assets folder immediately underneath "_build"
    };

    std::string getAssetUriInDataSource(TestAssetType assetType, const char* filename = nullptr);
    std::string getAssetDirectory(TestAssetDirectoryType assetType);

    class FrameworkScoped
    {
    public:
        FrameworkScoped()
        {
            m_framework = acquireFrameworkAndRegisterBuiltins();
            // Remove default logger:
            logging::ILogging* ls = logging::getLogging();

            ls->setLevelThreshold(carb::logging::kLevelError);
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
        }

        void startupEmpty()
        {
            // Load core Carbonite plugins needed for settings, events, etc.
            // omni.kit.app.plugin is loaded to provide carb::stats::IStats (required by omni.physx.plugin)
            // but Kit IApp is NOT started — we only need the interface registration.
            this->loadPlugins({
                "omni.kit.app.plugin",
                "carb.dictionary.plugin",
                "carb.settings.plugin",
                "carb.tokens.plugin",
                "carb.dictionary.serializer-json.plugin",
                "carb.dictionary.serializer-toml.plugin",
                "carb.events.plugin",
            });

            m_fileSystem->setAppDirectoryPath(m_fileSystem->getExecutableDirectoryPath());
        }

        filesystem::IFileSystem* getFileSystem() const
        {
            return m_fileSystem;
        }

    private:
        filesystem::IFileSystem* m_fileSystem = nullptr;
    };
}
