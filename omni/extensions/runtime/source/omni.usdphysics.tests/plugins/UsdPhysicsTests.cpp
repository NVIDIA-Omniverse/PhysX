// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#define CARB_EXPORTS

#include <private/omni/physics/schema/IUsdPhysics.h>
#include <private/omni/physics/schema/IUsdPhysicsListener.h>
#include <private/omni/physics/schematests/IUsdPhysicsTests.h>
#include <carb/Framework.h>


#include <carb/Framework.h>
#include <carb/PluginUtils.h>

using namespace carb;
using namespace omni;
using namespace physics;
using namespace schema;

using namespace pxr;

ObjectDescReportSubscriptionRegistry gObjectDescReportSubscriptions;

const struct carb::PluginImplDesc kPluginImpl = { "omni.physicsschema.tests.plugin", "Physics", "NVIDIA",
                                                  carb::PluginHotReload::eDisabled, "dev" };
CARB_PLUGIN_IMPL(kPluginImpl, omni::physics::schema::IUsdPhysicsTests)
CARB_PLUGIN_IMPL_DEPS(omni::physics::schema::IUsdPhysics)

class TestPhysicsListener: public IUsdPhysicsListener
{
    virtual void parsePrim(const pxr::UsdPrim& prim, ObjectDesc* objectDesc, uint64_t primTypes, const pxr::TfTokenVector& appliedApis)
    {
        if (!gObjectDescReportSubscriptions.map.empty())
        {
            ObjectDescReportSubscriptionRegistry::EventMap::const_iterator it = gObjectDescReportSubscriptions.map.begin();
            ObjectDescReportSubscriptionRegistry::EventMap::const_iterator itEnd = gObjectDescReportSubscriptions.map.end();
            while (it != itEnd)
            {
                it->second.first(prim.GetPrimPath(), *objectDesc, it->second.second);
                it++;
            }
        }
    }

    virtual void reportObjectDesc(const pxr::SdfPath& path, const omni::physics::schema::ObjectDesc* objectDesc)
    {
    }

} gTestListener;

CARB_EXPORT void carbOnPluginStartup()
{
}

CARB_EXPORT void carbOnPluginShutdown()
{
    gObjectDescReportSubscriptions.clear();

}

void attachStage(long int stageId)
{    
    omni::physics::schema::IUsdPhysics* schemaParser = carb::getCachedInterface<omni::physics::schema::IUsdPhysics>();
    schemaParser->registerPhysicsListener(&gTestListener);

    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(stageId));
    if (stage)
    {
        UsdGeomXformCache xfCache;
        pxr::UsdPrimRange range = stage->TraverseAll();
        omni::physics::schema::PrimIteratorRange primIteratorRange(range);
        schemaParser->loadFromRange(stage, xfCache, primIteratorRange);
    }
}

void deattachStage()
{    
    omni::physics::schema::IUsdPhysics* schemaParser = carb::getCachedInterface<omni::physics::schema::IUsdPhysics>();
    schemaParser->unregisterPhysicsListener(&gTestListener);
}

SubscriptionId subscribeObjectDescEventFn(ObjectDescReportFn reportFn, void* userData)
{
    return gObjectDescReportSubscriptions.addEvent(std::make_pair(reportFn, userData));
}

void unsubscribeObjectDescEventFn(SubscriptionId id)
{
    gObjectDescReportSubscriptions.removeEvent(id);
}

void fillInterface(IUsdPhysicsTests& iface)
{
    iface.attachStage = attachStage;
    iface.deattachStage = deattachStage;
    iface.subscribeObjectDescEventFn = subscribeObjectDescEventFn;
    iface.unsubscribeObjectDescEventFn = unsubscribeObjectDescEventFn;
}
