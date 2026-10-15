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

bool internHandles(const IPhysicsSource& source,
                   const ovx_primpath_t* handles,
                   size_t count,
                   std::vector<ObjectKey>& outKeys)
{
    const auto* ovSource = dynamic_cast<const OvstageSource*>(&source);
    if (!ovSource)
        return false;

    if (count == 0)
        return true; // an ovstage source with nothing to intern
    if (!handles)
        return false;

    outKeys.reserve(outKeys.size() + count);
    for (size_t i = 0; i < count; ++i)
    {
        // internKey packs the raw dictionary handle into this source's generation-tagged ObjectKey, then
        // canonicalKey normalises it to the interned-by-path handle. This normalisation is load-bearing:
        // intern_path and enumerate hand back DISTINCT handles for the same prim (canonicalKey's own doc),
        // so a caller's interned handle would not compare equal to the enumerated key a read gates on. The
        // glob path resolves literals through findByPath, i.e. the same canonical form, so both selection
        // routes land on the identical key. A raw handle carries no generation, so internKey tags whatever it
        // is given with this source's: the generation check that guards keys this source mints cannot apply.
        // A handle from the current dictionary names its path; a number the dictionary does not know stays
        // non-canonical and matches nothing (a selection is a superset filter); a zero handle interns to the
        // invalid sentinel and is dropped. A handle kept from a dictionary that has since been destroyed (every
        // ovstage instance released) is invalid but undetectable -- the dictionary has no liveness query -- and
        // may name a different prim. The public contract requires handles from the attached Stage's dictionary.
        const ObjectKey key = ovSource->canonicalKey(ovSource->internKey(static_cast<uint64_t>(handles[i])));
        if (key.valid())
            outKeys.push_back(key);
    }
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
