// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-CORE-003
 * @covers AC-5
 */

#include <omni/physics/parse/ParseContext.h>

namespace omni::physics::parse
{

// ---------------------------------------------------------------------------
// ParseContext
// ---------------------------------------------------------------------------

ParseContext::ParseContext(IPhysicsSource& source, IDescriptorAllocator& allocator)
    : mSource(source)
    , mAllocator(allocator)
    , mUnits(source.getSourceUnits())
{
}

ParseContext::~ParseContext() = default;

uint32_t ParseContext::registerEnvId(TokenId token)
{
    auto it = mEnvIdMap.find(token);
    if (it == mEnvIdMap.end())
    {
        uint32_t id = mEnvIdCounter++;
        mEnvIdMap[token] = id;
        return id;
    }
    return it->second;
}

uint32_t ParseContext::getEnvId(TokenId token) const
{
    auto it = mEnvIdMap.find(token);
    if (it != mEnvIdMap.end())
        return it->second;
    return 0;
}

// ---------------------------------------------------------------------------
// ObjectDatabase
// ---------------------------------------------------------------------------

void ObjectDatabase::findOrCreateEntry(ObjectKey key, ObjectCategory category, ObjectId id)
{
    mKeyMap[key].insert(std::make_pair(category, id));
}

const ObjectIdMap* ObjectDatabase::getEntries(ObjectKey key) const
{
    auto it = mKeyMap.find(key);
    if (it != mKeyMap.end())
        return &it->second;
    return nullptr;
}

ObjectIdMap* ObjectDatabase::getEntries(ObjectKey key)
{
    auto it = mKeyMap.find(key);
    if (it != mKeyMap.end())
        return &it->second;
    return nullptr;
}

ObjectId ObjectDatabase::findEntry(ObjectKey key, ObjectCategory category) const
{
    auto it = mKeyMap.find(key);
    if (it != mKeyMap.end())
    {
        const ObjectIdMap& map = it->second;
        auto mapit = map.find(category);
        if (mapit != map.end())
            return mapit->second;
    }
    return kInvalidObjectId;
}

bool ObjectDatabase::removeEntries(ObjectKey key)
{
    auto it = mKeyMap.find(key);
    if (it != mKeyMap.end())
    {
        it->second.clear();
        mKeyMap.erase(it);
    }
    return true;
}

void ObjectDatabase::removeEntry(ObjectKey key, ObjectCategory category, ObjectId id)
{
    auto it = mKeyMap.find(key);
    if (it != mKeyMap.end())
    {
        ObjectIdMap& entries = it->second;
        auto range = entries.equal_range(category);
        for (auto rit = range.first; rit != range.second; ++rit)
        {
            if (rit->second == id)
            {
                entries.erase(rit);
                break;
            }
        }
        if (entries.empty())
            mKeyMap.erase(it);
    }
}

void ObjectDatabase::addSchemaAPI(ObjectKey key, SchemaAPIFlag::Enum api)
{
    mSchemaAPIMap[key] |= api;
}

void ObjectDatabase::setSchemaAPI(ObjectKey key, uint64_t flags)
{
    mSchemaAPIMap[key] = flags;
}

void ObjectDatabase::removeSchemaAPIs(ObjectKey key)
{
    auto it = mSchemaAPIMap.find(key);
    if (it != mSchemaAPIMap.end())
        mSchemaAPIMap.erase(it);
}

void ObjectDatabase::removeSchemaAPI(ObjectKey key, SchemaAPIFlag::Enum api)
{
    mSchemaAPIMap[key] &= ~api;
}

uint64_t ObjectDatabase::getSchemaAPIs(ObjectKey key) const
{
    auto it = mSchemaAPIMap.find(key);
    if (it != mSchemaAPIMap.end())
        return it->second;
    return 0;
}

} // namespace omni::physics::parse
