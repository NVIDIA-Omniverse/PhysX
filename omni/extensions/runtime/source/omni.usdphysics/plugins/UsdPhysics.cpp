// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#define CARB_EXPORTS

#include <private/omni/physics/schema/IUsdPhysics.h>
#include <private/omni/physics/IUsdPhysicsParse.h>

#include <carb/Framework.h>
#include <carb/PluginUtils.h>

#include "UsdLoad.h"

using namespace omni;
using namespace physics;
using namespace schema;

using namespace pxr;

static PhysicsSchemaUsdLoad*     gUsdLoad = nullptr;

const struct carb::PluginImplDesc kPluginImpl = { "omni.physicsschema.plugin", "Physics", "NVIDIA",
                                                  carb::PluginHotReload::eDisabled, "dev" };
CARB_PLUGIN_IMPL(kPluginImpl, omni::physics::schema::IUsdPhysics, omni::physics::usdparser::IUsdPhysicsParse)
CARB_PLUGIN_IMPL_NO_DEPS()


CARB_EXPORT void carbOnPluginStartup()
{
    gUsdLoad = new PhysicsSchemaUsdLoad();
}

CARB_EXPORT void carbOnPluginShutdown()
{
    delete gUsdLoad;
    gUsdLoad = nullptr;
}

bool loadFromRange(const pxr::UsdStageWeakPtr stage, UsdGeomXformCache& xfCache, PrimIteratorBase& range)
{
    return gUsdLoad->loadFromRange(stage, xfCache, range);
}

void registerPhysicsListener(IUsdPhysicsListener* listener)
{
    gUsdLoad->registerPhysicsListener(listener);
}

void unregisterPhysicsListener(IUsdPhysicsListener* listener)
{
    gUsdLoad->unregisterPhysicsListener(listener);
}

void addCustomShapeToken(const pxr::TfToken& token)
{
    gUsdLoad->addCustomShapeToken(token);
}

void addCustomJointToken(const pxr::TfToken& token)
{
    gUsdLoad->addCustomJointToken(token);
}

void removeCustomJointToken(const pxr::TfToken& token)
{
    gUsdLoad->removeCustomJointToken(token);
}

void addCustomPhysicsInstancerToken(const pxr::TfToken& token)
{
    gUsdLoad->addCustomPhysicsInstancerToken(token);
}

void removeCustomPhysicsInstancerToken(const pxr::TfToken& token)
{
    gUsdLoad->removeCustomPhysicsInstancerToken(token);
}

void removeCustomShapeToken(const pxr::TfToken& token)
{
    gUsdLoad->removeCustomShapeToken(token);
}

void fillInterface(IUsdPhysics& iface)
{
    iface.loadFromRange = loadFromRange;
    iface.registerPhysicsListener = registerPhysicsListener;
    iface.unregisterPhysicsListener = unregisterPhysicsListener;
    iface.addCustomShapeToken = addCustomShapeToken;
    iface.addCustomJointToken = addCustomJointToken;
    iface.removeCustomJointToken = removeCustomJointToken;
    iface.removeCustomShapeToken = removeCustomShapeToken;
    iface.addCustomPhysicsInstancerToken = addCustomPhysicsInstancerToken;
    iface.removeCustomPhysicsInstancerToken = removeCustomPhysicsInstancerToken;
}

struct UsdParsePhysicsListener: public IUsdPhysicsListener
{
    virtual void parsePrim(const pxr::UsdPrim& prim, ObjectDesc* objectDesc, uint64_t primTypes, const pxr::TfTokenVector& appliedApis)
    {
    }

    virtual void reportObjectDesc(const pxr::SdfPath& path, const omni::physics::schema::ObjectDesc* objectDesc)
    {
        
        switch (objectDesc->type)
        {
            case omni::physics::schema::ObjectType::eJointSpherical:
            {
                const SphericalJointDesc* inDesc = (const SphericalJointDesc*)objectDesc;
                SphericalJointDesc* outDesc = new omni::physics::schema::SphericalJointDesc();
                *outDesc = *inDesc;
                jointDesc = outDesc;
            }
            break;
            case omni::physics::schema::ObjectType::eJointD6:
            {
                const D6JointDesc* inDesc = (const D6JointDesc*)objectDesc;
                D6JointDesc* outDesc = new omni::physics::schema::D6JointDesc();
                *outDesc = *inDesc;
                jointDesc = outDesc;
            }
            break;
            case omni::physics::schema::ObjectType::eJointDistance:
            {
                const DistanceJointDesc* inDesc = (const DistanceJointDesc*)objectDesc;
                DistanceJointDesc* outDesc = new omni::physics::schema::DistanceJointDesc();
                *outDesc = *inDesc;
                jointDesc = outDesc;
            }
            break;
            case omni::physics::schema::ObjectType::eJointFixed:
            {
                const FixedJointDesc* inDesc = (const FixedJointDesc*)objectDesc;
                FixedJointDesc* outDesc = new omni::physics::schema::FixedJointDesc();
                *outDesc = *inDesc;
                jointDesc = outDesc;
            }
            break;
            case omni::physics::schema::ObjectType::eJointPrismatic:
            {
                const PrismaticJointDesc* inDesc = (const PrismaticJointDesc*)objectDesc;
                PrismaticJointDesc* outDesc = new omni::physics::schema::PrismaticJointDesc();
                *outDesc = *inDesc;
                jointDesc = outDesc;
            }
            break;
            case omni::physics::schema::ObjectType::eJointRevolute:
            {
                const RevoluteJointDesc* inDesc = (const RevoluteJointDesc*)objectDesc;
                RevoluteJointDesc* outDesc = new omni::physics::schema::RevoluteJointDesc();
                *outDesc = *inDesc;
                jointDesc = outDesc;
            }
            break;
            case omni::physics::schema::ObjectType::eJointCustom:
            {
                const JointDesc* inDesc = (const JointDesc*)objectDesc;
                JointDesc* outDesc = new omni::physics::schema::JointDesc();
                *outDesc = *inDesc;
                jointDesc = outDesc;
            }
            break;
            case omni::physics::schema::ObjectType::eAttachmentVtxVtx:
            case omni::physics::schema::ObjectType::eAttachmentVtxTri:
            case omni::physics::schema::ObjectType::eAttachmentVtxTet:
            case omni::physics::schema::ObjectType::eAttachmentVtxCrv:
            case omni::physics::schema::ObjectType::eAttachmentVtxXform:
            case omni::physics::schema::ObjectType::eAttachmentTetXform:
            case omni::physics::schema::ObjectType::eAttachmentTriTri:
            {
                const AttachmentDesc* inDesc = (const AttachmentDesc*)objectDesc;
                AttachmentDesc* outDesc = new omni::physics::schema::AttachmentDesc(objectDesc->type);
                *outDesc = *inDesc;
                attachmentDesc = outDesc;
            }
            break;
            case omni::physics::schema::ObjectType::eElementCollisionFilter:
            {
                const ElementCollisionFilterDesc* inDesc = (const ElementCollisionFilterDesc*)objectDesc;
                ElementCollisionFilterDesc* outDesc = new omni::physics::schema::ElementCollisionFilterDesc();
                *outDesc = *inDesc;
                collisionFilterDesc = outDesc;
            }
            break;

            default:
            break;
        }
    }
    omni::physics::schema::JointDesc* jointDesc = nullptr;
    omni::physics::schema::AttachmentDesc* attachmentDesc = nullptr;
    omni::physics::schema::ElementCollisionFilterDesc* collisionFilterDesc = nullptr;
};

omni::physics::schema::JointDesc* parseJoint(uint64_t stageId, const pxr::SdfPath& path)
{
    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt((long)stageId));
    if (!stage)
        return nullptr;

    UsdParsePhysicsListener usdParseListner;
    gUsdLoad->registerPhysicsListener(&usdParseListner);
    UsdGeomXformCache xfCache;
    UsdPrim usdPrim = stage->GetPrimAtPath(path);
    pxr::UsdPrimRange range(usdPrim);
    omni::physics::schema::PrimIteratorRange primIteratorRange(range);
    gUsdLoad->loadFromRange(stage, xfCache, primIteratorRange);
    gUsdLoad->unregisterPhysicsListener(&usdParseListner);
    return usdParseListner.jointDesc;
}

omni::physics::schema::AttachmentDesc* parseAttachment(uint64_t stageId, const pxr::SdfPath& path)
{
    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt((long)stageId));
    if (!stage)
        return nullptr;

    UsdParsePhysicsListener usdParseListner;
    gUsdLoad->registerPhysicsListener(&usdParseListner);
    UsdGeomXformCache xfCache;
    UsdPrim usdPrim = stage->GetPrimAtPath(path);
    pxr::UsdPrimRange range(usdPrim);
    omni::physics::schema::PrimIteratorRange primIteratorRange(range);
    primIteratorRange.pruneChildren();
    gUsdLoad->loadFromRange(stage, xfCache, primIteratorRange, false);
    gUsdLoad->unregisterPhysicsListener(&usdParseListner);
    return usdParseListner.attachmentDesc;
}

omni::physics::schema::ElementCollisionFilterDesc* parseCollisionFilter(uint64_t stageId, const pxr::SdfPath& path)
{
    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt((long)stageId));
    if (!stage)
        return nullptr;

    UsdParsePhysicsListener usdParseListner;
    gUsdLoad->registerPhysicsListener(&usdParseListner);
    UsdGeomXformCache xfCache;
    UsdPrim usdPrim = stage->GetPrimAtPath(path);
    pxr::UsdPrimRange range(usdPrim);
    omni::physics::schema::PrimIteratorRange primIteratorRange(range);
    primIteratorRange.pruneChildren();
    gUsdLoad->loadFromRange(stage, xfCache, primIteratorRange, false);
    gUsdLoad->unregisterPhysicsListener(&usdParseListner);
    return usdParseListner.collisionFilterDesc;
}

void fillInterface(omni::physics::usdparser::IUsdPhysicsParse& iface)
{
    iface.parseJoint = parseJoint;
    iface.parseAttachment = parseAttachment;
    iface.parseCollisionFilter = parseCollisionFilter;
    iface.releaseDesc = [] (omni::physics::schema::ObjectDesc* desc) { delete desc; };
}
