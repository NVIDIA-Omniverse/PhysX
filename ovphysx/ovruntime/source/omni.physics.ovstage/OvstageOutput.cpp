// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include <omni/physics/ovstage/OvstageOutput.h>

#include "OvstageSource.h"

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

    // Canonicalise each key's handle so the list is built from dictionary-interned
    // prim paths (the create_path_list precondition), never the walker/enumerate
    // handle — no string conversion on the caller side.
    std::vector<ovx_primpath_t> handles;
    handles.reserve(count);
    for (size_t i = 0; i < count; ++i)
        handles.push_back(ovSource->canonicalPath(keys[i]));

    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
    if (ovx_path_dictionary_create_path_list(dict, handles.data(), handles.size(), &list) != OVX_OK)
        return OVX_INVALID_PRIMPATH_LIST;
    return list;
}

} // namespace omni::physics::ovstage
