// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// from omni/usd/LayerUtils.h, cannot include directly because of an imgui include in KitUtils.h
namespace omni
{
namespace usd
{

static constexpr size_t kLayerIndexNone = SIZE_MAX;

namespace LayerUtils
{

static std::string normalizeUrl(const std::string& url)
{
    std::string result;
    auto omniclient = carb::getCachedInterface<carb::omniclient::IOmniClient>();
    if (omniclient)
    {
        size_t bufferSize = 0;
        omniclient->normalizeUrl(url.c_str(), nullptr, &bufferSize);
        if (bufferSize != 0)
        {
            auto stringBufferHeap = std::unique_ptr<char[]>(new char[bufferSize]);
            const char* normalizedUrl = omniclient->normalizeUrl(url.c_str(), stringBufferHeap.get(), &bufferSize);
            if (!normalizedUrl)
            {
                result = url;
            }
            else
            {
                result = normalizedUrl;
            }
        }
        else
        {
            result = url;
        }
    }
    else
    {
        result = url;
    }

    return result;
}

static std::string normalizePath(const std::string& path)
{
    // OMPE-43063: The result of makeRelativeUrl may include %xx escape characters, so here we
    // need to go through an unquote process to decode the escapte characters.
    // Because the input will not always be % escaped, we only convert valid % escaped characters.
    std::string finalPath;
    finalPath.reserve(path.length());

    auto hexCharToInt = [](char c) -> int
    {
        if (c >= '0' && c <= '9') return c - '0';
        c = std::toupper(c);
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    };

    for (size_t i = 0; i < path.length(); ++i)
    {
        if (path[i] == '%')
        {
            if (i + 2 < path.length())
            {
                int high = hexCharToInt(path[i + 1]);
                int low = hexCharToInt(path[i + 2]);
                if (high != -1 && low != -1)
                {
                    finalPath += static_cast<char>((high << 4) | low);
                    i += 2;
                }
                else
                {

                    finalPath += '%';
                }
            }
            else
            {

                finalPath += '%';
            }
        }

        else if (path[i] == '+')
        {
            finalPath += ' ';
        }
        else
        {
            finalPath += path[i];
        }
    }

    std::replace(finalPath.begin(), finalPath.end(), '\\', '/');

    return finalPath;
}

static std::string computeAbsolutePath(const PXR_NS::SdfLayerRefPtr& rootLayer, const std::string& path)
{
    if (PXR_NS::SdfLayer::IsAnonymousLayerIdentifier(path) || rootLayer->IsAnonymous())
    {
        return path;
    }
    else
    {
        // Compute the path through the resolver
        const std::string& absolutePath = rootLayer->ComputeAbsolutePath(path);
        return normalizePath(absolutePath);
    }
}

static size_t getSublayerPositionInHost(const PXR_NS::SdfLayerRefPtr& hostLayer, const std::string& layerIdentifier)
{
    const auto& sublayerPaths = hostLayer->GetSubLayerPaths();
    for (size_t i = 0; i < sublayerPaths.size(); i++)
    {
        const auto& absolutePath = computeAbsolutePath(hostLayer, sublayerPaths[i]);
        if (normalizeUrl(absolutePath) == normalizeUrl(layerIdentifier))
        {
            return i;
        }
    }

    return kLayerIndexNone;
}

}}}
