// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-ATTACH-RESOLVE-001
// @covers AC-1 AC-2 AC-4
// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-3 AC-4
#pragma once

#include "internal/InternalPhysXDatabase.h"
#include "usdLoad/AttachedStage.h"

namespace omni::physx::attachmentLookup
{
// Private helpers for callers already executing against an explicit owner.
// Public interfaces acquire runtime admission before using these functions.
inline omni::physics::parse::ObjectKey resolveKey(const usdparser::AttachedStage* owner, const char* path)
{
    if (!owner || !path)
        return {};
    const omni::physics::parse::IPhysicsSource* source = owner->getSource();
    if (!source)
        return {};
    const omni::physics::parse::ObjectKey key = owner->keyFor(std::string_view(path));
    if (!key.valid())
        return {};
    // Registry first: an ObjectDb hit is a map lookup, while exists() can cost an
    // ovstage whole-stage query for keys outside the source's caches.
    const usdparser::ObjectIdMap* entries = owner->getObjectIds(key);
    if (entries && !entries->empty())
        return key;
    return source->exists(key) ? key : omni::physics::parse::ObjectKey{};
}

inline const char* pathFor(const usdparser::AttachedStage* owner, omni::physics::parse::ObjectKey key)
{
    if (!owner || !owner->getSource() || !owner->isKeyLive(key))
        return "";
    // Every source returns null-terminated string storage for a resolved key.
    const std::string_view path = owner->getSource()->sourceKeyToString(key);
    return path.data() ? path.data() : "";
}

inline omni::physics::parse::ObjectKey keyForObject(const usdparser::AttachedStage* owner, usdparser::ObjectId id)
{
    if (!owner)
        return {};
    const internal::InternalPhysXDatabase& database = owner->getInternalPhysXDatabase();
    if (id < database.getRecords().size())
    {
        const internal::InternalDatabase::Record& record = database.getRecords()[id];
        // Retirement leaves the old key in a slot: never resurrect it.
        if (record.mType != ePTRemoved)
            return record.mKey;
    }
    return {};
}
}
