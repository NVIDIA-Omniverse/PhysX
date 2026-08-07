// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <ovx/path_dictionary/path_dictionary.h>

#include <string>
#include <vector>

using ovx_path_dictionary_t = path_dictionary_instance_t;

#ifndef OVX_OK
#define OVX_OK OVX_API_SUCCESS
#endif

namespace ovstage_compat
{
inline ovx_api_status_t status(ovx_path_dictionary_t* dict, ovx_api_result_t result)
{
    const ovx_api_status_t out = result.status;
    if (out != OVX_API_SUCCESS && result.error.ptr && dict)
        path_dictionary_release_error(dict, result.error);
    return out;
}
} // namespace ovstage_compat

static inline ovx_api_status_t ovx_path_dictionary_intern_token(
    ovx_path_dictionary_t* dict,
    ovx_string_t str,
    ovx_token_t* outToken)
{
    if (!dict || !outToken)
        return OVX_API_ERROR;
    return ovstage_compat::status(dict, path_dictionary_create_tokens_from_strings(dict, &str, 1, outToken));
}

static inline ovx_api_status_t ovx_path_dictionary_intern_path(
    ovx_path_dictionary_t* dict,
    ovx_string_t str,
    ovx_primpath_t* outPath)
{
    if (!dict || !outPath)
        return OVX_API_ERROR;
    return ovstage_compat::status(dict, path_dictionary_create_paths_from_strings(dict, &str, 1, outPath));
}

static inline ovx_api_status_t ovx_path_dictionary_create_path_list(
    ovx_path_dictionary_t* dict,
    const ovx_primpath_t* paths,
    size_t numPaths,
    ovx_primpath_list_t* outList)
{
    if (!dict || !outList)
        return OVX_API_ERROR;
    return ovstage_compat::status(
        dict, path_dictionary_create_path_list_from_paths(dict, paths, numPaths, outList));
}

static inline ovx_api_status_t ovx_path_dictionary_create_path_list_from_strings(
    ovx_path_dictionary_t* dict,
    const ovx_string_t* pathStrings,
    size_t numPaths,
    ovx_primpath_list_t* outList)
{
    if (!dict || !outList)
        return OVX_API_ERROR;
    return ovstage_compat::status(
        dict, path_dictionary_create_path_list_from_strings(dict, pathStrings, numPaths, outList));
}

static inline ovx_api_status_t ovx_path_dictionary_destroy_path_list(
    ovx_path_dictionary_t* dict,
    ovx_primpath_list_t list)
{
    if (!dict)
        return OVX_API_ERROR;
    return ovstage_compat::status(dict, path_dictionary_release_path_list_reference(dict, list));
}

static inline ovx_api_status_t ovx_path_dictionary_token_to_string(
    ovx_path_dictionary_t* dict,
    ovx_token_t token,
    ovx_string_t* outString)
{
    if (!dict || !outString)
        return OVX_API_ERROR;
    return ovstage_compat::status(dict, path_dictionary_get_strings_from_tokens(dict, &token, 1, outString));
}

static inline ovx_api_status_t ovx_path_dictionary_get_paths(
    ovx_path_dictionary_t* dict,
    ovx_primpath_list_t list,
    const ovx_primpath_t** outPaths,
    size_t* outCount)
{
    if (!dict || !outPaths || !outCount)
        return OVX_API_ERROR;

    *outPaths = nullptr;
    *outCount = 0;

    size_t count = 0;
    ovx_api_result_t result = path_dictionary_get_num_paths_from_path_list(dict, list, &count);
    if (result.status != OVX_API_SUCCESS)
        return ovstage_compat::status(dict, result);

    thread_local std::vector<ovx_primpath_t> paths;
    paths.assign(count, OVX_INVALID_PRIMPATH);
    if (count > 0)
    {
        size_t fetched = 0;
        result = path_dictionary_get_paths_from_path_list(dict, list, 0, count, paths.data(), &fetched);
        if (result.status != OVX_API_SUCCESS)
            return ovstage_compat::status(dict, result);
        paths.resize(fetched);
    }

    *outPaths = paths.empty() ? nullptr : paths.data();
    *outCount = paths.size();
    return OVX_API_SUCCESS;
}

static inline ovx_api_status_t ovx_path_dictionary_path_to_string(
    ovx_path_dictionary_t* dict,
    ovx_primpath_t path,
    ovx_string_t* outString)
{
    if (!dict || !outString)
        return OVX_API_ERROR;

    *outString = {};

    thread_local std::vector<ovx_token_t> tokenBuffer;
    thread_local std::vector<ovx_string_t> segments;
    thread_local std::string pathStorage;

    if (tokenBuffer.empty())
        tokenBuffer.resize(64);

    ovx_token_t* tokens = nullptr;
    size_t tokenCount = 0;
    size_t processed = 0;
    ovx_api_result_t result{};
    for (int attempt = 0; attempt < 4; ++attempt)
    {
        result = path_dictionary_get_tokens_from_paths(
            dict, &path, 1, tokenBuffer.data(), tokenBuffer.size(), &tokens, &tokenCount, &processed);
        if (result.status != OVX_API_SUCCESS)
            return ovstage_compat::status(dict, result);
        if (processed != 0)
            break;
        tokenBuffer.resize(tokenBuffer.size() * 2);
    }

    if (processed == 0)
        return OVX_API_ERROR;

    if (tokenCount == 0)
    {
        pathStorage = "/";
        outString->ptr = pathStorage.data();
        outString->length = pathStorage.size();
        return OVX_API_SUCCESS;
    }

    if (!tokens)
        return OVX_API_ERROR;

    segments.assign(tokenCount, ovx_string_t{});
    if (tokenCount > 0)
    {
        result = path_dictionary_get_strings_from_tokens(dict, tokens, tokenCount, segments.data());
        if (result.status != OVX_API_SUCCESS)
            return ovstage_compat::status(dict, result);
    }

    pathStorage.clear();
    for (const ovx_string_t& segment : segments)
    {
        pathStorage.push_back('/');
        if (segment.ptr && segment.length)
            pathStorage.append(segment.ptr, segment.length);
    }

    outString->ptr = pathStorage.data();
    outString->length = pathStorage.size();
    return OVX_API_SUCCESS;
}
