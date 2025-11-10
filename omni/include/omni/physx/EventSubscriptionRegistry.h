// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include <algorithm>
#include <map>

namespace omni
{

namespace physx
{

using SubscriptionId = size_t;
const SubscriptionId kInvalidSubscriptionId = SubscriptionId(0xFFffFFffFF);

// Allocating a EventSubscriptionRegistry<ElementType, OrderedRegistryTag> will allow you to
// specify an additional parameter when inserting events: the order.
// The internal container guarantees that when iterating the relative order between
// the elements (according to the 'order' parameter), will be respected.
// E.g.
//  addEvent(some_event_to_add_1, 0)
//  addEvent(some_event_to_add_2, 1)
//  addEvent(some_event_to_add_3, 56)
// it is guaranteed that when iterating the internal container, the ordering will be:
// {some_event_to_add_1, some_event_to_add_2, some_event_to_add_3}
struct UnorderedRegistryTag
{
};
struct OrderedRegistryTag
{
};

template <class TSubscriptionEntry, typename OrderingType = UnorderedRegistryTag>
struct EventSubscriptionRegistry;

template <class TSubscriptionEntry>
struct EventSubscriptionRegistry<TSubscriptionEntry, UnorderedRegistryTag>
{
    using EventMap = std::map<omni::physx::SubscriptionId, TSubscriptionEntry>;

    omni::physx::SubscriptionId addEvent(TSubscriptionEntry entry)
    {
        omni::physx::SubscriptionId id;
        if (freeIds.empty())
        {
            id = map.size();
        }
        else
        {
            id = freeIds.back();
            freeIds.pop_back();
        }
        map[id] = entry;
        return id;
    }

    void removeEvent(omni::physx::SubscriptionId id)
    {
        typename EventMap::iterator it = map.find(id);
        CARB_ASSERT(it != map.end());
        if (it != map.end())
        {
            map.erase(it);
            freeIds.push_back(id);
        }
    }

    void clear()
    {
        map.clear();
        freeIds.clear();
    }

    EventMap map;
    std::vector<omni::physx::SubscriptionId> freeIds;
};

template <class TSubscriptionEntry>
struct EventSubscriptionRegistry<TSubscriptionEntry, OrderedRegistryTag>
{
public:
    using EventMap = std::multimap<int, std::pair<omni::physx::SubscriptionId, TSubscriptionEntry>>;
    // This class also supports an optional external id generator so two subscription registries can share
    // a single generator and have unique ids shared among them (so that an unregister(id) doesn't need to know
    // which map it should go to)
    using IdGenerator = std::function<omni::physx::SubscriptionId()>;

    explicit EventSubscriptionRegistry(IdGenerator idGenerator = nullptr /* optional id generator */)
        : idGenerator(idGenerator)
    {
    }

    void setIdGenerator(IdGenerator idGenerator)
    {
        this->idGenerator = idGenerator;
    }

    omni::physx::SubscriptionId getNextId()
    {
        omni::physx::SubscriptionId id;
        if (idGenerator)
        {
            id = idGenerator();
        }
        else if (freeIds.empty())
        {
            id = eventMap.size();
        }
        else
        {
            id = freeIds.back();
            freeIds.pop_back();
        }
        return id;
    }

    omni::physx::SubscriptionId addEvent(TSubscriptionEntry entry, int order)
    {
        omni::physx::SubscriptionId id;
        if (idGenerator)
        {
            id = idGenerator();
        }
        else if (freeIds.empty())
        {
            id = eventMap.size();
        }
        else
        {
            id = freeIds.back();
            freeIds.pop_back();
        }
        eventMap.insert({ order, { id, entry } });
        return id;
    }

    void removeEvent(omni::physx::SubscriptionId id)
    {
        auto it =
            std::find_if(eventMap.begin(), eventMap.end(), [&](const auto& pair) { return pair.second.first == id; });

        if (it != eventMap.end())
        {
            eventMap.erase(it);
            freeIds.push_back(id);
        }
    }

    void clear()
    {
        eventMap.clear();
        freeIds.clear();
    }

    const EventMap& getMap() const
    {
        return eventMap;
    }

    EventMap& getMap()
    {
        return eventMap;
    }

private:
    EventMap eventMap;
    std::vector<omni::physx::SubscriptionId> freeIds;
    IdGenerator idGenerator;
};

} // namespace physx
} // namespace omni
