// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <omni/physics/parse/Handles.h> // parse::ObjectKey

#include <ovstage/ovx_path_dictionary.h> // ovx_primpath_list_t / ovx_path_dictionary_t

#include <cstddef>
#include <vector>

namespace omni::physics::parse
{
class IPhysicsSource;
}

namespace omni::physics::ovstage
{

// Build an immutable ovx_primpath_list_t over `keys` using the active source's shared path
// dictionary, interning each key's prim-path handle canonically (no string round-trip). Returns
// OVX_INVALID_PRIMPATH_LIST unless `source` is the ovstage-backed `OvstageSource`. The caller owns
// the list and must release it with `ovx_path_dictionary_destroy_path_list` on `outDict`.
ovx_primpath_list_t buildPathList(const omni::physics::parse::IPhysicsSource& source,
                                  const omni::physics::parse::ObjectKey* keys,
                                  size_t count,
                                  ovx_path_dictionary_t** outDict);

// Canonicalise `keys` into dictionary-interned prim-path handles, the expensive half of
// buildPathList (a lock plus a cache lookup per key). A caller emitting several groups over the
// same prim set canonicalises once here and then calls buildPathListFromHandles per group.
// Appends to `outHandles`; returns false for a non-ovstage source, leaving `outHandles` untouched.
bool canonicalisePathHandles(const omni::physics::parse::IPhysicsSource& source,
                             const omni::physics::parse::ObjectKey* keys,
                             size_t count,
                             std::vector<ovx_primpath_t>& outHandles,
                             ovx_path_dictionary_t** outDict);

// buildPathList over handles already produced by canonicalisePathHandles. Each call returns a
// distinct list that the caller owns and must release independently: groups cannot share one,
// because releasing a group destroys its list.
ovx_primpath_list_t buildPathListFromHandles(const omni::physics::parse::IPhysicsSource& source,
                                             const ovx_primpath_t* handles,
                                             size_t count,
                                             ovx_path_dictionary_t** outDict);

// The reverse of canonicalisePathHandles: map dictionary-interned prim-path handles (the native
// `ovx_primpath_t` the caller already holds from a read group's prim list or from ovstage) back to their
// ObjectKeys, without re-running the string pattern matcher. Each handle is canonicalised to the
// interned-by-path key so it compares equal to the enumerated key a read/write gates on (see the impl for
// why this matters). Appends the resolved keys to `outKeys`; a handle that does not resolve to a live key
// on this source is skipped (a prim-set selection is a superset filter, so an unresolvable entry simply
// matches nothing). Returns false for a non-ovstage source, leaving `outKeys` untouched.
bool internHandles(const omni::physics::parse::IPhysicsSource& source,
                   const ovx_primpath_t* handles,
                   size_t count,
                   std::vector<omni::physics::parse::ObjectKey>& outKeys);

} // namespace omni::physics::ovstage
