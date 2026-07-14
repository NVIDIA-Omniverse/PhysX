// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#include "UsdPCH.h"

#include "TestHelpers.h"

#include "../framework/BmGlobals.h"

#include <carb/Framework.h>
#include <carb/filesystem/IFileSystem.h>

namespace carb
{

std::string getAssetUriInDataSource(TestAssetType assetType, const char* filename)
{
    std::string outputPath;
    switch (assetType)
    {
    case TestAssetType::eShader:
        outputPath = "shaders/tests";
        break;
    case TestAssetType::eTexture:
        outputPath = "textures/tests";
        break;
    case TestAssetType::eImageComparison:
        outputPath = "textures/tests/compare";
        break;
    case TestAssetType::eUsd:
        outputPath = "usd/tests";
        break;
    case TestAssetType::eNone:
        CARB_ASSERT(false);
        break;
    }

    if (filename)
    {
        outputPath.append("/");
        outputPath.append(filename);
    }

    return outputPath;
}

std::string getAssetDirectory(TestAssetDirectoryType assetType)
{    
    std::string outputPath = BmGlobals::getInstance().getDataFolder();    

    switch (assetType)
    {
    case TestAssetDirectoryType::eBuildRoot:
        break;
    case TestAssetDirectoryType::eDataRoot:
        outputPath += "/data";
        break;
    case TestAssetDirectoryType::eImageComparison:
        outputPath += "/data/textures/tests/compare";
        break;
    case TestAssetDirectoryType::eImageComparisonOutput:
        outputPath += "/outputs";
        break;
    case TestAssetDirectoryType::eFullBuildTarget:
        break;
    default:
        break;
    }

    return outputPath;
}
} // namespace carb
