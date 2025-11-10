// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "IOmniClient.h"

#include <memory>
#include <utility>

namespace carb
{
namespace omniclient
{

class OmniClientUrlPtr
{
public:
    OmniClientUrlPtr() = default;
    OmniClientUrlPtr(IOmniClient* omniclient, const char* url)
    {
        this->omniclient = omniclient;
        this->url = omniclient->breakUrl(url);
    }
    OmniClientUrlPtr(IOmniClient* omniclient, OmniClientUrl* url)
    {
        this->omniclient = omniclient;
        this->url = url;
    }
    OmniClientUrlPtr(OmniClientUrlPtr&& obj)
        : omniclient(obj.omniclient),
          url(obj.url)
    {
        this->omniclient = nullptr;
        this->url = nullptr;
    }
    ~OmniClientUrlPtr()
    {
        if (omniclient)
        {
            omniclient->freeUrl(url);
        }
    }
    OmniClientUrlPtr& operator=(OmniClientUrlPtr&& that)
    {
        this->omniclient = std::exchange(that.omniclient, nullptr);
        this->url = std::exchange(that.url, nullptr);
        return *this;
    }
    const OmniClientUrl* operator->() const
    {
        return url;
    }
    operator const OmniClientUrl*() const {
        return url;
    }
    OmniClientUrl operator*() const
    {
        return *url;
    }

private:
    IOmniClient* omniclient{};
    OmniClientUrl* url{};
};

static constexpr size_t kDefaultUrlSize = 100;

inline std::string makeUrl(IOmniClient* omniclient, OmniClientUrl const* url)
{
    size_t urlSize = kDefaultUrlSize;
    std::string buffer(urlSize, 0);
    char* urlString = nullptr;
    while (urlString == nullptr)
    {
        urlString = omniclient->makeUrl(url, &buffer[0], &urlSize);
        buffer.resize(urlSize - 1);
    };
    return buffer;
}

inline std::string makeFileUrl(IOmniClient* omniclient, char const* path)
{
    size_t urlSize = kDefaultUrlSize;
    std::string buffer(urlSize, 0);
    char* fileUrlConverted = nullptr;
    while (fileUrlConverted == nullptr)
    {
        fileUrlConverted = omniclient->makeFileUrl(path, &buffer[0], &urlSize);
        buffer.resize(urlSize - 1);
    };
    return buffer;
}

inline std::string combineUrls(IOmniClient* omniclient, char const* baseUrl, char const* otherUrl)
{
    size_t urlSize = kDefaultUrlSize;
    std::string buffer(urlSize, 0);
    char* urlCombined = nullptr;
    while (urlCombined == nullptr)
    {
        urlCombined = omniclient->combineUrls(baseUrl, otherUrl, &buffer[0], &urlSize);
        buffer.resize(urlSize - 1);
    };
    return buffer;
}

inline std::string normalizeUrl(IOmniClient* omniclient, char const* url)
{
    size_t urlSize = kDefaultUrlSize;
    std::string buffer(urlSize, 0);
    char* urlNormalized = nullptr;
    while (urlNormalized == nullptr)
    {
        urlNormalized = omniclient->normalizeUrl(url, &buffer[0], &urlSize);
        buffer.resize(urlSize - 1);
    };
    return buffer;
}

inline std::string makeRelativeUrl(IOmniClient* omniclient, char const* baseUrl, char const* otherUrl)
{
    size_t urlSize = kDefaultUrlSize;
    std::string buffer(urlSize, 0);
    char* relativeUrl = nullptr;
    while (relativeUrl == nullptr)
    {
        relativeUrl = omniclient->makeRelativeUrl(baseUrl, otherUrl, &buffer[0], &urlSize);
        buffer.resize(urlSize - 1);
    };
    return buffer;
}

inline bool getBranchAndCheckpointFromQuery(IOmniClient* omniclient, const char* query, std::string& branch, uint64_t &checkpoint)
{
    auto branchAndCheckpoint = omniclient->getBranchAndCheckpointFromQuery(query);
    if (branchAndCheckpoint)
    {
        branch = branchAndCheckpoint->branch;
        checkpoint = branchAndCheckpoint->checkpoint;
        omniclient->freeBranchAndCheckpoint(branchAndCheckpoint);
        return true;
    }
    return false;
}

} // namespace omniclient
} // namespace carb
