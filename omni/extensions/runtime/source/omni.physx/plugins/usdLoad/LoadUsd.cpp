// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/settings/ISettings.h>
#include <private/omni/physics/schema/IUsdPhysics.h>
#include <omni/physx/IPhysxSettings.h>
#include <omni/physx/PhysxTokens.h>
#include <common/foundation/Allocator.h>
#include <common/utilities/PrimUtilities.h>
#include <common/utilities/OmniPhysXUtilities.h>
#include <carb/profiler/Profile.h>

#include "LoadUsd.h"
#include "LoadTools.h"
#include "PrimUpdate.h"
#include "LoadStage.h"
#include "Mass.h"
#include "Joint.h"
#include "Articulation.h"
#include "Collision.h"
#include "Particles.h"
#include "SoftBodyDeprecated.h"
#include "AttachmentDeprecated.h"
#include "Collision.h"
#include "CollisionGroup.h"
#include "Vehicle.h"
#include "FixedTendon.h"
#include "SpatialTendon.h"
#include "Particles.h"
#include <CookingDataAsync.h>
#include <OmniPhysX.h>
#include "PhysicsBody.h"

#include <common/foundation/Algorithms.h>

#if CARB_COMPILER_MSC
#    pragma warning(disable : 4996)
#endif

omni::fabric::IChangeTrackerConfig* gChangeTrackerConfig = nullptr;

using namespace pxr;
using namespace carb;
using namespace omni::physics::schema;

namespace omni
{
namespace physx
{
namespace usdparser
{

static UsdLoad* gUsdLoad = nullptr;

UsdLoad* UsdLoad::getUsdLoad()
{
    if (!gUsdLoad)
        gUsdLoad = new UsdLoad();

    return gUsdLoad;
}

UsdLoad::UsdLoad()
    : mUsdNoticeListener(nullptr),
    mBlockUsdUpdate({0}), mAsyncUpdate(false)
{
    carb::Framework* framework = carb::getFramework();
    mUsdNoticeListener = new omni::physx::usdparser::UsdNoticeListener();
    mUsdNoticeListenerKey = TfNotice::Register(TfCreateWeakPtr(mUsdNoticeListener), &omni::physx::usdparser::UsdNoticeListener::Handle);

    // Register AttributeValuesChangedNotice type with USD.
    // We have to do it here rather than statically using TF_REGISTRY_FUNCTION
    // because USD calls in static initializers doesn't work with our unit
    // test framework.
    static bool firstTime = true;
    if (firstTime)
    {
        const std::type_info &typeInfo = typeid(omni::fabric::AttributeValuesChangedNotice);
        TfType tt = TfType::FindByTypeid(typeInfo);
        if(tt.IsUnknown())
        {
            TfType::Define<omni::fabric::AttributeValuesChangedNotice, TfType::Bases<TfNotice>>();
        }
        firstTime = false;
    }

    mAttributeValuesChangedListenerKey = TfNotice::Register(TfCreateWeakPtr(mUsdNoticeListener),
        &omni::physx::usdparser::UsdNoticeListener::HandleAttributeValuesChanged);

    // allocate fabric global tokens
    if (carb::getCachedInterface<omni::fabric::IToken>())
    {
        mFabricTokens.worldMatrix = new omni::fabric::Token(gWorldMatrixTokenString);
        mFabricTokens.localMatrix = new omni::fabric::Token(gLocalMatrixTokenString);
        mFabricTokens.worldForce = new omni::fabric::Token(gWorldForceTokenString);
        mFabricTokens.worldTorque = new omni::fabric::Token(gWorldTorqueTokenString);
        mFabricTokens.positionInvMasses = new omni::fabric::Token(gPositionInvMassesTokenString);
        mFabricTokens.velocitiesFloat4 = new omni::fabric::Token(gVelocitiesFloat4TokenString);
    }

    if (!gChangeTrackerConfig)
    {
        gChangeTrackerConfig = carb::getFramework()->tryAcquireInterface<omni::fabric::IChangeTrackerConfig>();
    }
}

UsdLoad::~UsdLoad()
{
    TfNotice::Revoke(mUsdNoticeListenerKey);
    TfNotice::Revoke(mAttributeValuesChangedListenerKey);
    delete mUsdNoticeListener;
    mUsdNoticeListener = nullptr;

    if (mFabricTokens.worldMatrix)
    {
        delete mFabricTokens.worldMatrix;
        mFabricTokens.worldMatrix = nullptr;
    }

    if (mFabricTokens.localMatrix)
    {
        delete mFabricTokens.localMatrix;
        mFabricTokens.localMatrix = nullptr;
    }

    if (mFabricTokens.worldForce)
    {
        delete mFabricTokens.worldForce;
        mFabricTokens.worldForce = nullptr;
    }

    if (mFabricTokens.worldTorque)
    {
        delete mFabricTokens.worldTorque;
        mFabricTokens.worldTorque = nullptr;
    }

    if (mFabricTokens.positionInvMasses)
    {
        delete mFabricTokens.positionInvMasses;
        mFabricTokens.positionInvMasses = nullptr;
    }

    if (mFabricTokens.velocitiesFloat4)
    {
        delete mFabricTokens.velocitiesFloat4;
        mFabricTokens.velocitiesFloat4 = nullptr;
    }

    gChangeTrackerConfig = nullptr;
}

bool UsdLoad::attach(bool loadPhysics, uint64_t stageId, PhysXUsdPhysicsInterface* usdPhysicsInt)
{
    if (mAttachedStages.find(stageId) != mAttachedStages.end())
    {
        CARB_LOG_ERROR("Stage already attached!");
        return false;
    }

    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(long(stageId)));
    if (!stageId || !stage)
    {
        CARB_LOG_ERROR("PhysicsUsdLoad - could not find USD stage");
        return false;
    }

    AttachedStage* attachedStage = new AttachedStage(stage, usdPhysicsInt);
    mAttachedStages[stageId] = attachedStage;

    // check if we should load the scene on attach
    if (loadPhysics)
    {
        attachedStage->getPrimUpdateMap().setEmptyScene(true);

        attachedStage->getPhysXPhysicsInterface()->enableObjectChangeNotifications(false);

        // load the scene by traversing from the root prim
        loadFromStage(*attachedStage);

        attachedStage->getPhysXPhysicsInterface()->enableObjectChangeNotifications(true);
        // now the initial load is done and notifications should be sent

        if (!attachedStage->getObjectDatabase()->empty())
        {
            attachedStage->getPrimUpdateMap().setEmptyScene(false);
        }
    }
    else
        attachedStage->getPrimUpdateMap().setEmptyScene(true);

    if (OmniPhysX::getInstance().getIStageReaderWriter())
    {
        fabricAttach(stageId);
    }
    return true;
}

bool UsdLoad::attachReplicator(uint64_t stageId, PhysXUsdPhysicsInterface* usdPhysicsInt, const PathSet& excludePaths, bool attachStage)
{
    if (attachStage && !attach(false, stageId, usdPhysicsInt))
        return false;

    AttachedStage* attachedStage = getAttachedStage(stageId);
    attachedStage->setReplicatorStage(true);
    {
        attachedStage->getPrimUpdateMap().setEmptyScene(true);
        attachedStage->getPrimUpdateMap().clearMap();
        attachedStage->getPrimChangeMap().clearMap();

        attachedStage->getPhysXPhysicsInterface()->enableObjectChangeNotifications(false);

        // load the scene by traversing from the root prim
        loadFromStage(*attachedStage, &excludePaths);

        attachedStage->getPhysXPhysicsInterface()->enableObjectChangeNotifications(true);
        // now the initial load is done and notifications should be sent

        if (!attachedStage->getObjectDatabase()->empty())
        {
            attachedStage->getPrimUpdateMap().setEmptyScene(false);
        }
    }
    return true;
}

void UsdLoad::fabricAttach(uint64_t usdStageId)
{
    if (!gChangeTrackerConfig)
    {
        gChangeTrackerConfig = carb::getFramework()->tryAcquireInterface<omni::fabric::IChangeTrackerConfig>();
    }
    if (gChangeTrackerConfig)
    {
        AttachedStageMap::iterator fit = mAttachedStages.find(usdStageId);
        AttachedStage* attachedStage = nullptr;
        if (fit != mAttachedStages.end())
        {
            fit->second->fabricAttach();
        }
    }
}

void UsdLoad::releasePhysicsObjects(uint64_t stageId)
{
    notifyStageReset();

    AttachedStageMap::iterator fit = mAttachedStages.find(stageId);
    if (fit != mAttachedStages.end())
    {
        fit->second->releasePhysicsObjects();
    }
}

void UsdLoad::requestRigidBodyMassUpdate(const UsdPrim& prim)
{
    const uint64_t stageId = UsdUtilsStageCache::Get().GetId(prim.GetStage()).ToLongInt();
    AttachedStageMap::const_iterator fit = mAttachedStages.find(stageId);
    if (fit != mAttachedStages.end())
    {
        RequestRigidBodyMassUpdate(*fit->second, prim);
    }
}

void UsdLoad::requestDeformableBodyMassUpdateDeprecated(const UsdPrim& prim)
{
    const uint64_t stageId = UsdUtilsStageCache::Get().GetId(prim.GetStage()).ToLongInt();
    AttachedStageMap::const_iterator fit = mAttachedStages.find(stageId);
    if (fit != mAttachedStages.end())
    {
        RequestDeformableBodyMassUpdateDeprecated(*fit->second, prim);
    }
}

void UsdLoad::requestDeformableSurfaceMassUpdateDeprecated(const UsdPrim& prim)
{
    const uint64_t stageId = UsdUtilsStageCache::Get().GetId(prim.GetStage()).ToLongInt();
    AttachedStageMap::const_iterator fit = mAttachedStages.find(stageId);
    if (fit != mAttachedStages.end())
    {
        RequestDeformableSurfaceMassUpdateDeprecated(*fit->second, prim);
    }
}

void UsdLoad::requestParticleMassUpdate(const UsdPrim& prim)
{
    const uint64_t stageId = UsdUtilsStageCache::Get().GetId(prim.GetStage()).ToLongInt();
    AttachedStageMap::const_iterator fit = mAttachedStages.find(stageId);
    if (fit != mAttachedStages.end())
    {
        RequestParticleMassUpdate(*fit->second, prim);
    }
}

void UsdLoad::detach(uint64_t stageId)
{
    AttachedStageMap::iterator fit = mAttachedStages.find(stageId);
    if (fit != mAttachedStages.end())
    {
        notifyStageReset();
        fit->second->getPhysXPhysicsInterface()->enableObjectChangeNotifications(false);  // do not send these notifications when the simulation is to end
        releasePhysicsObjects(stageId);

        delete fit->second;

        mAttachedStages.erase(fit);
    }
    else
    {
        CARB_LOG_WARN("Detach stage failed!");
    }
}

void UsdLoad::update(uint64_t stageId, float currentTime)
{
    AttachedStageMap::const_iterator fit = mAttachedStages.find(stageId);
    if (fit != mAttachedStages.end())
    {
        processUpdates(*fit->second, currentTime);
    }
}

void UsdLoad::update(float currentTime)
{
    for (AttachedStageMap::reference ref : mAttachedStages)
    {
        processUpdates(*ref.second, currentTime);
    }
}

void UsdLoad::flushChanges()
{
    for (AttachedStageMap::reference ref : mAttachedStages)
    {
        flushBufferedChanges(*ref.second, -1.0f);
    }
}

void UsdLoad::changeDefaultSimulator(const std::string& defaultSim)
{
    const bool defSim = omni::physx::isPhysXDefaultSimulator();
    for (AttachedStageMap::reference ref : mAttachedStages)
    {
        ref.second->setIsPhysXDefaultSimulator(defSim);
    }
}

class LoadUsdPhysicsListener : public omni::physics::schema::IUsdPhysicsListener
{
public:
    void parsePrim(const UsdPrim& prim, omni::physics::schema::ObjectDesc* objectDesc, uint64_t, const TfTokenVector& appliedApis) override
    {
        if (objectDesc)
        {
            switch (objectDesc->type)
            {
            //tendon axes not supported on fixed and spherical joints for now
            //case omni::physics::schema::ObjectType::eJointFixed:
            case omni::physics::schema::ObjectType::eJointPrismatic:
            case omni::physics::schema::ObjectType::eJointRevolute:
            //case omni::physics::schema::ObjectType::eJointSpherical:
            {
                const JointDesc* jointDesc = static_cast<const JointDesc*>(objectDesc);
                if (prim.HasAPI<PhysxSchemaPhysxTendonAxisAPI>())
                {
                    parseTendonAxes(mAttachedStage, prim, jointDesc, mTendonAxisMap, mFixedTendons);
                }
            }
            break;
            default:
                break;
            }
        }
    }

    void reportObjectDesc(const SdfPath& path, const omni::physics::schema::ObjectDesc* objectDesc) override
    {
        switch (objectDesc->type)
        {
            // shapes are created third
        case omni::physics::schema::ObjectType::eSphereShape:
        case omni::physics::schema::ObjectType::eCubeShape:
        case omni::physics::schema::ObjectType::eCapsuleShape:
        case omni::physics::schema::ObjectType::eCylinderShape:
        case omni::physics::schema::ObjectType::eConeShape:
        case omni::physics::schema::ObjectType::eMeshShape:
        case omni::physics::schema::ObjectType::eCustomShape:
        case omni::physics::schema::ObjectType::eSpherePointsShape:
        {
            const ShapeDesc* inDesc = (const ShapeDesc*)objectDesc;
            switch(parseMode)
            {
                case PARSE_RIGID_BODIES_AND_COLLIDERS:
                {
                    CollisionPairVector filteredPairs;
                    UsdGeomXformCache xfCache(UsdTimeCode::EarliestTime());
                    SdfPathVector materials;
                    PhysxShapeDesc* collider = parseCollisionDesc(mAttachedStage, xfCache, mPhysXDescCache, path, *inDesc, filteredPairs, materials, true);
                    if(collider)
                    {
                        rigidBodyShapesMap.insert({path, collider});
                    }
                }
                break;
                default:
                {
                    if ((!inDesc->masterDesc && inDesc->sourceGprim == gPrim) || (inDesc->masterDesc && inDesc->usdPrim == gPrim))
                    {
                        CollisionPairVector filteredPairs;
                        UsdGeomXformCache xfCache(UsdTimeCode::EarliestTime());
                        SdfPathVector materials;
                        shapeDesc = parseCollisionDesc(mAttachedStage, xfCache, mPhysXDescCache, path, *inDesc, filteredPairs, materials, true);
                    }
                }
                break;
            }
        }
        break;
        case omni::physics::schema::ObjectType::eJointSpherical:
        case omni::physics::schema::ObjectType::eJointD6:
        case omni::physics::schema::ObjectType::eJointDistance:
        case omni::physics::schema::ObjectType::eJointFixed:
        case omni::physics::schema::ObjectType::eJointPrismatic:
        case omni::physics::schema::ObjectType::eJointRevolute:
        case omni::physics::schema::ObjectType::eJointCustom:
        {
            const JointDesc* inDesc = (const JointDesc*)objectDesc;
            jointDesc = parseJoint(inDesc->usdPrim.GetStage(), *inDesc, mXfCache);
        }
        break;
        case omni::physics::schema::ObjectType::eRigidBody:
        {
            const RigidBodyDesc* inDesc = (const RigidBodyDesc*)objectDesc;
            switch(parseMode)
            {
                case PARSE_RIGID_BODIES_AND_COLLIDERS:
                {
                    UsdGeomXformCache xfCache(UsdTimeCode::EarliestTime());
                    CollisionPairVector collisionBlocks;
                    PhysxRigidBodyDesc* desc = parseRigidBody(mAttachedStage, xfCache, *inDesc, collisionBlocks, true);
                    bodyAndCollider.desc = desc;
                    bodyAndCollider.collisions = inDesc->collisions;
                }
                break;
                default:
                {
                    UsdPrim prim = mStage->GetPrimAtPath(path);
                    if (prim.HasAPI<PhysxSchemaPhysxTendonAttachmentAPI>())
                    {
                        parseTendonAttachments(mAttachedStage, prim, mXfCache, mTendonAttachmentMap, mSpatialTendons);
                    }
                }
                break;
            }
        }
        break;
        case omni::physics::schema::ObjectType::eArticulation:
        {
            const ArticulationDesc* inDesc = (const ArticulationDesc*)objectDesc;
            CollisionPairVector filteredPairs;
            ArticulationMap articulationMap;
            parseArticulation(inDesc->usdPrim.GetStage(), *inDesc, filteredPairs, articulationMap);
            if(inDesc->usdPrim == gPrim)
            {
                articulationDesc = articulationMap[gPrim.GetPrimPath()];
            }
            else
            {
                for(auto& vec: articulationMap)
                {
                    for(auto& it: vec.second)
                    {
                        ICE_PLACEMENT_DELETE(it, PhysxArticulationDesc);
                    }
                }
            }
        }
        break;
        default:
            break;
        }
    }

    void clear()
    {
        gPrim = UsdPrim();
        shapeDesc = nullptr;
        jointDesc = nullptr;
        bodyAndCollider.desc = nullptr;
        bodyAndCollider.collisions.clear();
        rigidBodyShapesMap.clear();
        for (PathPhysXDescMap::reference ref : mPhysXDescCache)
        {
            PhysxObjectDesc* desc = (PhysxObjectDesc*)ref.second;
            ICE_FREE(desc);
        }
        articulationDesc.clear();
        mPhysXDescCache.clear();
        mStage.Reset();
        mXfCache.Clear(); // is this really needed?
        mSpatialTendons.clear();
        mTendonAttachmentMap.clear();
        mFixedTendons.clear();
        mTendonAxisMap.clear();
    }

    void setup(UsdGeomXformCache& xfCache, UsdStageWeakPtr stage)
    {
        mStage = stage;
        mXfCache = xfCache;
        mAttachedStage.setStage(stage);
    }

    enum ParseMode
    {
        PARSE_DEFAULT,
        PARSE_RIGID_BODIES_AND_COLLIDERS
    };
    ParseMode parseMode = PARSE_DEFAULT;
 	AttachedStage   mAttachedStage;
    UsdGeomXformCache mXfCache;
    PathPhysXDescMap mPhysXDescCache;

    UsdPrim    gPrim;
    PhysxShapeDesc* shapeDesc;
    PhysxJointDesc* jointDesc;
    TfHashMultiMap<SdfPath, PhysxShapeDesc*, SdfPath::Hash> rigidBodyShapesMap;
    BodyDescAndColliders bodyAndCollider;
    std::vector<omni::physx::usdparser::PhysxArticulationDesc*> articulationDesc;

    // tendon data structures
    SpatialTendonVector mSpatialTendons;
    TendonAttachmentMap mTendonAttachmentMap;
    FixedTendonVector mFixedTendons;
    TendonAxisMap mTendonAxisMap;

    private:
        UsdStageWeakPtr mStage;

} gLoadPhysicsListener;

PhysxJointDesc* parseJoint(uint64_t stageId, const SdfPath& jointPath)
{
    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(long(stageId)));
    if (!stage)
        return nullptr;

    IUsdPhysics* usdPhysics = carb::getCachedInterface<IUsdPhysics>();
    gLoadPhysicsListener.clear();
    UsdGeomXformCache xfCache(UsdTimeCode::EarliestTime());
    gLoadPhysicsListener.setup(xfCache, stage);
    usdPhysics->registerPhysicsListener(&gLoadPhysicsListener);
    UsdPrim usdPrim = stage->GetPrimAtPath(jointPath);
    UsdPrimRange range(usdPrim);
    omni::physics::schema::PrimIteratorRange primIteratorRange(range);
    usdPhysics->loadFromRange(stage, xfCache, primIteratorRange);
    usdPhysics->unregisterPhysicsListener(&gLoadPhysicsListener);

    return gLoadPhysicsListener.jointDesc;
}

PhysxRigidBodyDesc* parseRigidBody(uint64_t stageId,
                                   const SdfPath& path,
                                   std::vector<std::pair<SdfPath, PhysxShapeDesc*>>& collision)
{
    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(static_cast<long int>(stageId)));

    if (!stage)
    {
        return nullptr;
    }

    UsdPrim usdPrim = stage->GetPrimAtPath(path);

    if (!usdPrim.IsValid())
    {
        return nullptr;
    }
    IUsdPhysics* usdPhysics = carb::getCachedInterface<IUsdPhysics>();
    gLoadPhysicsListener.clear();
    UsdGeomXformCache xfCache(UsdTimeCode::EarliestTime());
    gLoadPhysicsListener.setup(xfCache, stage);
    usdPhysics->registerPhysicsListener(&gLoadPhysicsListener);
    gLoadPhysicsListener.parseMode = LoadUsdPhysicsListener::PARSE_RIGID_BODIES_AND_COLLIDERS;
    UsdPrimRange range(usdPrim, UsdTraverseInstanceProxies());
    omni::physics::schema::PrimIteratorRange primIteratorRange(range);
    usdPhysics->loadFromRange(stage, xfCache, primIteratorRange);
    usdPhysics->unregisterPhysicsListener(&gLoadPhysicsListener);
    gLoadPhysicsListener.parseMode = LoadUsdPhysicsListener::PARSE_DEFAULT;

    for (const auto& collisionPath : gLoadPhysicsListener.bodyAndCollider.collisions)
    {
        if (collisionPath != SdfPath())
        {
            auto range = gLoadPhysicsListener.rigidBodyShapesMap.equal_range(collisionPath);

            for (auto it = range.first; it != range.second; ++it)
            {
                collision.push_back(*it);
            }
        }
    }
    return gLoadPhysicsListener.bodyAndCollider.desc;
}

PhysxShapeDesc* parseCollision(uint64_t stageId, const SdfPath& colPath, const SdfPath& gprimPath)
{
    CARB_PROFILE_ZONE(0, "physx::usdparser::parseCollision");
    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(long(stageId)));
    if (!stage)
        return nullptr;

    IUsdPhysics* usdPhysics = carb::getCachedInterface<IUsdPhysics>();
    gLoadPhysicsListener.clear();
    UsdGeomXformCache xfCache(UsdTimeCode::EarliestTime());
    gLoadPhysicsListener.setup(xfCache, stage);
    gLoadPhysicsListener.gPrim = stage->GetPrimAtPath(gprimPath);
    usdPhysics->registerPhysicsListener(&gLoadPhysicsListener);
    UsdPrim usdPrim = stage->GetPrimAtPath(colPath);
    UsdPrimRange range(usdPrim, UsdTraverseInstanceProxies());
    omni::physics::schema::PrimIteratorRange primIteratorRange(range);
    usdPhysics->loadFromRange(stage, xfCache, primIteratorRange);
    usdPhysics->unregisterPhysicsListener(&gLoadPhysicsListener);

    return gLoadPhysicsListener.shapeDesc;
}

std::vector<PhysxArticulationDesc*> parseArticulations(uint64_t stageId, const SdfPath& articulationRootPath)
{
    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(long(stageId)));
    if (!stage)
        return std::vector<PhysxArticulationDesc*>();

    IUsdPhysics* usdPhysics = carb::getCachedInterface<IUsdPhysics>();
    gLoadPhysicsListener.clear();
    UsdGeomXformCache xfCache(UsdTimeCode::EarliestTime());
    gLoadPhysicsListener.setup(xfCache, stage);
    gLoadPhysicsListener.gPrim = stage->GetPrimAtPath(articulationRootPath);
    usdPhysics->registerPhysicsListener(&gLoadPhysicsListener);
    // We have to traverse the whole stage here because we need to build joints and bodies map
    // that may be referenced even if they're outside of the range identified by current articulation
    // root path
    UsdPrimRange range = stage->Traverse(UsdTraverseInstanceProxies());
    omni::physics::schema::PrimIteratorRange primIteratorRange(range);
    usdPhysics->loadFromRange(stage, xfCache, primIteratorRange);
    usdPhysics->unregisterPhysicsListener(&gLoadPhysicsListener);
    return gLoadPhysicsListener.articulationDesc;
}

std::vector<PhysxTendonAttachmentHierarchyDesc*> parseSpatialTendons(uint64_t stageId, const SdfPath& parentPath)
{
    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(long(stageId)));
    if (!stage)
        return std::vector<PhysxTendonAttachmentHierarchyDesc*>();

    IUsdPhysics* usdPhysics = carb::getCachedInterface<IUsdPhysics>();
    gLoadPhysicsListener.clear();
    UsdGeomXformCache xfCache(UsdTimeCode::EarliestTime());
    gLoadPhysicsListener.setup(xfCache, stage);
    usdPhysics->registerPhysicsListener(&gLoadPhysicsListener);
    UsdPrim usdPrim = stage->GetPrimAtPath(parentPath);
    UsdPrimRange range(usdPrim);
    omni::physics::schema::PrimIteratorRange primIteratorRange(range);
    usdPhysics->loadFromRange(stage, xfCache, primIteratorRange);
    usdPhysics->unregisterPhysicsListener(&gLoadPhysicsListener);

    return getAttachmentUiInfo(gLoadPhysicsListener.mTendonAttachmentMap, gLoadPhysicsListener.mSpatialTendons);
}

std::vector<PhysxTendonAxisHierarchyDesc*> parseFixedTendons(uint64_t stageId, const SdfPath& parentPath)
{
    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(long(stageId)));
    if (!stage)
        return std::vector<PhysxTendonAxisHierarchyDesc*>();

    IUsdPhysics* usdPhysics = carb::getCachedInterface<IUsdPhysics>();
    gLoadPhysicsListener.clear();
    UsdGeomXformCache xfCache(UsdTimeCode::EarliestTime());
    gLoadPhysicsListener.setup(xfCache, stage);
    usdPhysics->registerPhysicsListener(&gLoadPhysicsListener);
    UsdPrim usdPrim = stage->GetPrimAtPath(parentPath);
    UsdPrimRange range(usdPrim);
    omni::physics::schema::PrimIteratorRange primIteratorRange(range);
    usdPhysics->loadFromRange(stage, xfCache, primIteratorRange);
    usdPhysics->unregisterPhysicsListener(&gLoadPhysicsListener);

    return getAxesUiInfo(gLoadPhysicsListener.mTendonAxisMap, gLoadPhysicsListener.mFixedTendons);
}

void releaseDesc(usdparser::PhysxObjectDesc* desc)
{
    ICE_FREE(desc);
}

void UsdLoad::blockUSDUpdate(bool val)
{
    mBlockUsdUpdate += (val ? 1 : -1);
    CARB_ASSERT(mBlockUsdUpdate >= 0, "Unbalanced calls to release blocked state");
}

bool UsdLoad::usdUpdateIsBlocked()
{
    return (mBlockUsdUpdate > 0);
}

usdparser::SoftBodyDesc* parseDeformableBodyDeprecated(uint64_t stageId, const SdfPath& softbodyPath)
{
    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(long(stageId)));
    if (!stage)
        return nullptr;

    UsdPrim usdPrim = stage->GetPrimAtPath(softbodyPath);
    AttachedStage attachedStage(stage);
    SoftBodyDesc* softBodyDesc = ParseDeformableBodyDeprecated(attachedStage, usdPrim);
    return softBodyDesc;
}

usdparser::FEMClothDesc* parseDeformableSurfaceDeprecated(uint64_t stageId, const SdfPath& femClothPath)
{
    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(long(stageId)));
    if (!stage)
        return nullptr;

    UsdPrim usdPrim = stage->GetPrimAtPath(femClothPath);
    AttachedStage attachedStage(stage);
    FEMClothDesc* femClothDesc = ParseDeformableSurfaceDeprecated(attachedStage, usdPrim);

    return femClothDesc;
}

usdparser::ParticleClothDesc* parseParticleClothDeprecated(uint64_t stageId, const SdfPath& clothPath)
{
    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(long(stageId)));
    if (!stage)
        return nullptr;

    UsdPrim usdPrim = stage->GetPrimAtPath(clothPath);
    AttachedStage attachedStage(stage);
    return ParseParticleClothDeprecated(attachedStage, usdPrim);
}

usdparser::ParticleSetDesc* parseParticleSet(uint64_t stageId, const SdfPath& particleSetPath)
{
    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(long(stageId)));
    if (!stage)
        return nullptr;

    UsdPrim usdPrim = stage->GetPrimAtPath(particleSetPath);
    AttachedStage attachedStage(stage);
    return ParseParticleSet(attachedStage, usdPrim);
}

usdparser::ParticleSamplingDesc* parseParticleSampling(uint64_t stageId, const SdfPath& samplingPath)
{
    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(long(stageId)));
    if (!stage)
        return nullptr;

    UsdPrim usdPrim = stage->GetPrimAtPath(samplingPath);
    AttachedStage attachedStage(stage);
    return ParseParticleSampling(attachedStage, usdPrim);
}

usdparser::PhysxAttachmentDesc* parsePhysxAttachmentDeprecated(uint64_t stageId, const SdfPath& attachmentPath)
{
    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(long(stageId)));
    if (!stage)
        return nullptr;

    UsdPrim usdPrim = stage->GetPrimAtPath(attachmentPath);

    return parsePhysxAttachmentDeprecated(stage, usdPrim);
}

usdparser::VehicleComponentTrackerHandle createVehicleComponentTracker()
{
    VehicleComponentTracker* vehicleComponentTracker = ICE_PLACEMENT_NEW(VehicleComponentTracker)();
    usdparser::VehicleComponentTrackerHandle handle = reinterpret_cast<usdparser::VehicleComponentTrackerHandle>(vehicleComponentTracker);
    return handle;
}

void releaseVehicleComponentTracker(usdparser::VehicleComponentTrackerHandle handle)
{
    VehicleComponentTracker* vehicleComponentTracker = reinterpret_cast<VehicleComponentTracker*>(handle);
    ICE_FREE(vehicleComponentTracker);
}

usdparser::VehicleDesc* parseVehicle(uint64_t stageId, const SdfPath& vehiclePath,
    usdparser::VehicleComponentTrackerHandle handle)
{
    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(long(stageId)));
    if (!stage)
        return nullptr;

    UsdPrim usdPrim = stage->GetPrimAtPath(vehiclePath);

    VehicleComponentTracker* vehicleComponentTracker = reinterpret_cast<VehicleComponentTracker*>(handle);
    ObjectType vehicleControllerType;
    VehicleControllerDesc vehicleControllerDesc;
    VehicleTankControllerDesc vehicleTankControllerDesc;
    VehicleDesc* vehicleDesc = ICE_PLACEMENT_NEW(VehicleDesc)();
    if (vehicleDesc)
    {
        UsdGeomXformCache xfCache(UsdTimeCode::EarliestTime());
        if (parseVehicle(stage, usdPrim, *vehicleDesc,
            vehicleControllerDesc, vehicleTankControllerDesc, vehicleControllerType,
            *vehicleComponentTracker, xfCache))
        {
            return vehicleDesc;
        }
        else
        {
            ICE_FREE(vehicleDesc);
        }
    }
    else
    {
        CARB_LOG_ERROR("parseVehicle: allocation of vehicle descriptor failed\n");
    }

    return nullptr;
}

usdparser::ParticleSystemDesc* parseParticleSystem(uint64_t stageId, const SdfPath& particleSystemPath)
{
    UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(long(stageId)));
    if (!stage)
        return nullptr;

    UsdPrim usdPrim = stage->GetPrimAtPath(particleSystemPath);
    AttachedStage attachedStage(stage);
    return ParseParticleSystem(attachedStage, usdPrim);
}

void UsdLoad::processChanges()
{
    for (AttachedStageMap::reference ref : mAttachedStages)
    {
        processChangeMap(*ref.second);
    }
}

void UsdLoad::updateRigidBodyMass()
{
    for (AttachedStageMap::reference ref : mAttachedStages)
    {
        ref.second->updateRigidBodyMass();
    }
}

} // namespace usdparser
} // namespace physx
} // namespace omni

