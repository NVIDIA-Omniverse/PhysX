// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include <private/omni/physics/schema/IUsdPhysicsListener.h>

namespace omni
{
namespace physics
{
namespace schema
{

using ObjectDescReportFn = std::function<void(const pxr::SdfPath& path, const ObjectDesc& desc, void* userData)>;
using SubscriptionId = size_t;

template <class EventFn>
struct EventSubscriptionRegistry
{
    typedef std::unordered_map<SubscriptionId, std::pair<EventFn, void*>> EventMap;

    SubscriptionId addEvent(std::pair<EventFn, void*> pair)
    {
        SubscriptionId id;
        if (freeIds.empty())
        {
            id = map.size();
        }
        else
        {
            id = freeIds.back();
            freeIds.pop_back();
        }
        map[id] = pair;
        return id;
    }

    void removeEvent(SubscriptionId id)
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
    std::vector<SubscriptionId> freeIds;
};

using ObjectDescReportSubscriptionRegistry = EventSubscriptionRegistry<ObjectDescReportFn>;

struct IUsdPhysicsTests
{
    CARB_PLUGIN_INTERFACE("omni::physics::schema::IUsdPhysicsTests", 1, 0)

    void(CARB_ABI* attachStage)(long int stageId);

    void(CARB_ABI* deattachStage)();

    SubscriptionId(CARB_ABI* subscribeObjectDescEventFn)(ObjectDescReportFn reportFn, void* userData);

    void(CARB_ABI* unsubscribeObjectDescEventFn)(SubscriptionId subscriptionId);
};

} // namespace schema
} // namespace physics
} // namespace omni
