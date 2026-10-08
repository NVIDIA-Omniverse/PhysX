// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OvstageOutput.h"

#include "OvstageSource.h"

#include <cstdint>
#include <vector>

namespace omni::physics::ovstage
{

ovx_primpath_list_t buildPathList(const omni::physics::parse::IPhysicsSource& source,
                                  const omni::physics::parse::ObjectKey* keys,
                                  size_t count,
                                  ovx_path_dictionary_t** outDict)
{
    if (outDict)
        *outDict = nullptr;

    // ovstage-only: any other backend can't produce interned ovstage handles.
    const auto* ovSource = dynamic_cast<const OvstageSource*>(&source);
    if (!ovSource)
        return OVX_INVALID_PRIMPATH_LIST;

    ovx_path_dictionary_t* dict = ovSource->dictionary();
    if (!dict)
        return OVX_INVALID_PRIMPATH_LIST;
    if (outDict)
        *outDict = dict;

    if (count == 0 || !keys)
        return OVX_INVALID_PRIMPATH_LIST;

    std::vector<ovx_primpath_t> handles;
    if (!canonicalisePathHandles(source, keys, count, handles, nullptr))
        return OVX_INVALID_PRIMPATH_LIST;
    return buildPathListFromHandles(source, handles.data(), handles.size(), nullptr);
}

bool canonicalisePathHandles(const IPhysicsSource& source,
                             const ObjectKey* keys,
                             size_t count,
                             std::vector<ovx_primpath_t>& outHandles,
                             ovx_path_dictionary_t** outDict)
{
    if (outDict)
        *outDict = nullptr;

    const auto* ovSource = dynamic_cast<const OvstageSource*>(&source);
    if (!ovSource)
        return false;

    ovx_path_dictionary_t* dict = ovSource->dictionary();
    if (!dict)
        return false;
    if (outDict)
        *outDict = dict;

    if (count == 0)
        return true; // a valid dictionary with nothing to canonicalise

    // count > 0 with no keys is a caller error, not an empty request: returning true here left
    // outHandles empty while promising it was filled, and the caller then builds a prim list of the
    // wrong length.
    if (!keys)
        return false;

    // create_path_list requires dictionary-interned prim paths, not the walker/enumerate handle.
    outHandles.reserve(outHandles.size() + count);
    for (size_t i = 0; i < count; ++i)
        outHandles.push_back(ovSource->canonicalPath(keys[i]));
    return true;
}

ovx_primpath_list_t buildPathListFromHandles(const IPhysicsSource& source,
                                             const ovx_primpath_t* handles,
                                             size_t count,
                                             ovx_path_dictionary_t** outDict)
{
    if (outDict)
        *outDict = nullptr;

    const auto* ovSource = dynamic_cast<const OvstageSource*>(&source);
    if (!ovSource)
        return OVX_INVALID_PRIMPATH_LIST;

    ovx_path_dictionary_t* dict = ovSource->dictionary();
    if (!dict)
        return OVX_INVALID_PRIMPATH_LIST;
    if (outDict)
        *outDict = dict;

    if (count == 0 || !handles)
        return OVX_INVALID_PRIMPATH_LIST;

    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
    if (ovx_path_dictionary_create_path_list(dict, handles, count, &list) != OVX_OK)
        return OVX_INVALID_PRIMPATH_LIST;
    return list;
}

} // namespace omni::physics::ovstage
