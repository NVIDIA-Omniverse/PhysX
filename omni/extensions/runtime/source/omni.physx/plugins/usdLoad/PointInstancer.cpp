// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <private/omni/physics/schema/IUsdPhysics.h>
#include <omni/physx/IPhysxSettings.h>
#include <common/foundation/Allocator.h>

#include "LoadTools.h"
#include "LoadUsd.h"
#include "Particles.h"
#include "Collision.h"
#include "PhysicsBody.h"
#include "Material.h"
#include "Mass.h"
#include "CollisionGroup.h"
#include "PointInstancer.h"

#include <PhysXTools.h>
#include <OmniPhysX.h>

#include <foundation/PxBitMap.h>

using namespace pxr;
using namespace carb;
using namespace omni::physics::schema;

namespace omni
{
namespace physx
{
namespace usdparser
{

bool isOutsideInstancer(const UsdPrim& prim, const UsdPrim& instancerPrim)
{
    UsdPrim parent = prim;
    while (parent && !parent.IsPseudoRoot())
    {
        if (parent == instancerPrim)
            return false;

        parent = parent.GetParent();
    }
    return true;
}

class PhysxInstancerListener : public omni::physics::schema::IUsdPhysicsListener
{
public:

    PhysxInstancerListener(AttachedStage& inAttachedStage, UsdGeomXformCache& inXfCache,
        const UsdPrim& inUsdPrim)
        : attachedStage(inAttachedStage), xfCache(inXfCache), usdPrim(inUsdPrim)        
    {        
    }

    void parsePrim(const pxr::UsdPrim& prim, omni::physics::schema::ObjectDesc* objectDesc, uint64_t, const pxr::TfTokenVector& appliedApis) override
    {
    }

    void reportObjectDesc(const pxr::SdfPath& path, const omni::physics::schema::ObjectDesc* objectDesc) override
    {
        switch (objectDesc->type)
        {
        case omni::physics::schema::ObjectType::eMaterial:
        {
            const MaterialDesc* inDesc = (const MaterialDesc*)objectDesc;
            PhysxMaterialDesc* desc = parseMaterialDesc(attachedStage.getStage(), *inDesc);
            const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, path, *desc);
            if (id != kInvalidObjectId)
            {
                attachedStage.getObjectDatabase()->findOrCreateEntry(path, desc->type, id);
            }
            ICE_FREE(desc);
        }
        default:
            break;
        }
    }

    void parse()
    {
        IUsdPhysics* usdPhysics = carb::getCachedInterface<IUsdPhysics>();
        usdPhysics->registerPhysicsListener(this);        
        for (auto prim : usdPrim.GetChildren())
        {
            pxr::UsdPrimRange range(prim, UsdTraverseInstanceProxies(pxr::UsdPrimIsActive && pxr::UsdPrimIsLoaded && !pxr::UsdPrimIsAbstract));
            UsdGeomXformCache xfCache;
            omni::physics::schema::PrimIteratorRange primIteratorRange(range);
            usdPhysics->loadFromRange(attachedStage.getStage(), xfCache, primIteratorRange);
        }
        usdPhysics->unregisterPhysicsListener(this);
    }

private:
    AttachedStage& attachedStage;
    UsdGeomXformCache& xfCache;
    const UsdPrim& usdPrim;
};


class PhysxPrototypeListener : public omni::physics::schema::IUsdPhysicsListener
{
public:

    PhysxPrototypeListener(AttachedStage& inAttachedStage, UsdGeomXformCache& inXfCache,
        const UsdPrim& inUsdPrim,
        CollisionPairVector& inFilteredPairs,
        const ObjectInstance& inObjectInstance,
        ShapeDescVector* inShapeDesc)
        : protoDesc(nullptr), usdPrim(inUsdPrim), filteredPairs(inFilteredPairs),          
          objectInstance(inObjectInstance),
          xfCache(inXfCache),
          shapeDescs(inShapeDesc),
        attachedStage(inAttachedStage)
    {
        stage = attachedStage.getStage();
    }

    void parsePrim(const pxr::UsdPrim& prim, omni::physics::schema::ObjectDesc* objectDesc, uint64_t, const pxr::TfTokenVector& appliedApis) override
    {
    }

    void reportObjectDesc(const pxr::SdfPath& path, const omni::physics::schema::ObjectDesc* objectDesc) override
    {
        switch (objectDesc->type)
        {
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
            SdfPathVector materials;
            PhysxShapeDesc* desc = parseCollisionDesc(attachedStage, xfCache, physXDescCache, path, *inDesc, filteredPairs, materials);
            if (!desc)
                return;

            finalizeShape(attachedStage, desc, materials.empty() ? inDesc->materials : materials);
            
            bool shouldCreateShape = true;
            if (desc->rigidBody == SdfPath())
            {
                bool bodyInstanced = false;

                // We want to create the shape only if its going to be shared, be a part of instanced body
                // We also support rigidBody applied on the point instancer or above, in that case we need to
                // instance individual shapes and not share them.
                UsdPrim parentPrim = stage->GetPrimAtPath(objectInstance.instancerPath);
                while (parentPrim != stage->GetPseudoRoot())
                {
                    if (parentPrim.HasAPI<UsdPhysicsRigidBodyAPI>())
                    {
                        bodyInstanced = true;
                        desc->rigidBody = parentPrim.GetPrimPath();
                        // get the local transform
                        bool resetXformStack;
                        const GfMatrix4d mat = xfCache.GetLocalTransformation(objectDesc->usdPrim, &resetXformStack);
                        const GfTransform tr(mat);
                        GfVec3ToFloat3(tr.GetTranslation(), desc->localPos);
                        GfQuatToFloat4(tr.GetRotation().GetQuat(), desc->localRot);
                        GfVec3ToFloat3(tr.GetScale(), desc->localScale);
                        break;
                    }
                    parentPrim = parentPrim.GetParent();
                }

                if (bodyInstanced)
                {
                    shouldCreateShape = false;
                }
            }

            if (shouldCreateShape)
            {
                if (shapeDescs)
                {
                    if (desc->rigidBody == SdfPath())
                    {
                        PhysxRigidBodyDesc* bodyDesc = createStaticBody();
                        bodyDesc->position = desc->localPos;
                        bodyDesc->rotation = desc->localRot;
                        bodyDesc->scale = desc->localScale;

                        desc->localPos = { 0.0f, 0.0f , 0.0f };
                        desc->localRot = { 0.0f, 0.0f , 0.0f, 1.0f };
                        desc->localScale = { 1.0f, 1.0f , 1.0f };

                        protoDesc = bodyDesc;
                    }
                    shapeDescs->push_back(std::make_pair(path, desc));
                }
                else
                {
                    if (desc->rigidBody == SdfPath())
                    {
                        if (!protoDesc || protoDesc->type != eStaticBody)
                        {
                            // create a static body based on the prototype root prim to assign the shapes to
                            PhysxRigidBodyDesc* rbDesc = createStaticBody();
                            bool resetXformStack;
                            const GfMatrix4d mat = xfCache.GetLocalTransformation(usdPrim, &resetXformStack);
                            const GfTransform tr(mat);
                            GfVec3ToFloat3(tr.GetTranslation(), rbDesc->position);
                            GfQuatToFloat4(tr.GetRotation().GetQuat(), rbDesc->rotation);
                            GfVec3ToFloat3(tr.GetScale(), rbDesc->scale);
                            rbDesc->sceneIds = desc->sceneIds;

                            protoDesc = rbDesc;
                        }

                        CARB_ASSERT(protoDesc->type == eStaticBody);
                        PhysxRigidBodyDesc* rbDesc = static_cast<PhysxRigidBodyDesc*>(protoDesc);

                        // recompute the local transform to be relative to the static body
                        UsdPrim shapePrim = attachedStage.getStage()->GetPrimAtPath(path);
                        const UsdPrim& bodyPrim = usdPrim;
                        GfVec3f localPos, localScale;
                        GfQuatf localRot;
                        getCollisionShapeLocalTransfrom(xfCache, shapePrim, bodyPrim, localPos, localRot, localScale);
                        GfVec3ToFloat3(localPos, desc->localPos);
                        GfQuatToFloat4(localRot, desc->localRot);
                        GfVec3ToFloat3(localScale, desc->localScale);

                        const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createShape(path, *desc, kInvalidObjectId, &objectInstance);
                        attachedStage.getObjectDatabase()->findOrCreateEntry(path, eShape, id);
                        rbDesc->shapes.push_back(id);
                    }
                    else
                    {
                        ObjectId shapeId = kInvalidObjectId;
                        protoDesc = createShape(attachedStage, path, xfCache, desc, &objectInstance, &shapeId);
                        shapeIds.push_back(shapeId);
                    }
                        
                }
                    
            }
            else
            {
                protoDesc = desc;
                protoPath = path;
            }
        }
        break;
        case omni::physics::schema::ObjectType::eRigidBody:
        {
            const RigidBodyDesc* inDesc = (const RigidBodyDesc*)objectDesc;
            PhysxRigidBodyDesc* rbDesc = parseRigidBody(attachedStage, xfCache, *inDesc, filteredPairs);
            protoDesc = rbDesc;
            protoPath = path;
            if (!shapeDescs && rbDesc)
            {
                rbDesc->shapes.clear();
                rbDesc->shapes = shapeIds;
            }
        }
        break;       
        default:
            break;
        }
    }

    PhysxObjectDesc* getProtoDesc() const
    {
        return protoDesc;
    }

    const SdfPath& getProtoPath() const
    {
        return protoPath;
    }

    void parsePrim()
    {
        IUsdPhysics* usdPhysics = carb::getCachedInterface<IUsdPhysics>();
        usdPhysics->registerPhysicsListener(this);        
        pxr::UsdPrimRange range(usdPrim, UsdTraverseInstanceProxies(pxr::UsdPrimIsActive && pxr::UsdPrimIsLoaded && !pxr::UsdPrimIsAbstract));
        UsdGeomXformCache xfCache;
        omni::physics::schema::PrimIteratorRange primIteratorRange(range);
        usdPhysics->loadFromRange(stage, xfCache, primIteratorRange);
        usdPhysics->unregisterPhysicsListener(this);
    }

    void clear()
    {
        for (PathPhysXDescMap::reference ref : physXDescCache)
        {
            PhysxObjectDesc* desc = (PhysxObjectDesc*)ref.second;
            ICE_FREE(desc);
        }
        physXDescCache.clear();
    }

private:
    PhysxObjectDesc* protoDesc;
    SdfPath         protoPath;
    ShapeDescVector* shapeDescs;
    ObjectIdVector   shapeIds;
    UsdStageWeakPtr stage;
    const UsdPrim& usdPrim;
    CollisionPairVector& filteredPairs;
    const ObjectInstance& objectInstance;
    UsdGeomXformCache& xfCache;
    PathPhysXDescMap physXDescCache;
    AttachedStage& attachedStage;
};

SdfPath parsePrototype(AttachedStage& attachedStage, UsdGeomXformCache& xfCache,
                           const UsdPrim& usdPrim,
                           CollisionPairVector& filteredPairs,
                           const ObjectInstance& objectInstance,
                           ShapeDescVector* inShapeDesc, PhysxObjectDesc** descOut)
{
    PhysxPrototypeListener prototypeListener(attachedStage, xfCache, usdPrim, filteredPairs, objectInstance, inShapeDesc);
    prototypeListener.parsePrim();
    prototypeListener.clear();

    *descOut = prototypeListener.getProtoDesc();
    return prototypeListener.getProtoPath();
}

void parseRigidBodyInstancer(AttachedStage& attachedStage, UsdGeomXformCache& xfCache,
                                                          const UsdPrim& usdPrim,
                                                          CollisionPairVector& filteredPairs)
{

    // parse the objects below point instancer that we need, like materials, those dont belong to a prototype
    PhysxInstancerListener instancerListener(attachedStage, xfCache, usdPrim);
    instancerListener.parse();

    const UsdGeomPointInstancer instancer(usdPrim);
    UsdRelationship prototypes = instancer.GetPrototypesRel();
    
    const GfMatrix4d instancerMatrix = xfCache.GetLocalToWorldTransform(usdPrim);
    const GfMatrix4d instancerMatrixInverse = instancerMatrix.GetInverse();
    const GfVec3d sc = GfTransform(instancerMatrix).GetScale();

    SdfPathVector targets;
    TargetDescVector targetObjects;
    prototypes.GetTargets(&targets);

    UsdTimeCode earliestTime = UsdTimeCode::EarliestTime();

    // these attributes are required and must match in length
    VtArray<int> indices;
    if (!getAttributeArrayTimedFallback(indices, instancer.GetProtoIndicesAttr(), earliestTime))
    {
        CARB_LOG_WARN("Physics:PointInstancer: (%s) indices array not valid\n", instancer.GetPath().GetText());
    }

    if (indices.size() == 0)
    {
        return;
    }

    VtArray<GfVec3f> positions;
    if (!getAttributeArrayTimedFallback(positions, instancer.GetPositionsAttr(), earliestTime) || positions.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:PointInstancer: (%s) positions array not valid\n", instancer.GetPath().GetText());
    }

    // these attributes are optional, but must match in length with 'indices' if they are defined
    VtArray<GfQuath> orientations;
    if (getAttributeArrayTimedFallback(orientations, instancer.GetOrientationsAttr(), earliestTime) && orientations.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:PointInstancer: (%s) orientations array size does not match instance count\n",
                      instancer.GetPath().GetText());
    }

    VtArray<GfVec3f> velocities;
    getAttributeArrayTimedFallback(velocities, instancer.GetVelocitiesAttr(), earliestTime);
    if (velocities.size() > 0 && velocities.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:PointInstancer: (%s) velocities defined but size does not match instance count\n",
                      instancer.GetPath().GetText());
    }

    VtArray<GfVec3f> angularVelocities;
    getAttributeArrayTimedFallback(angularVelocities, instancer.GetAngularVelocitiesAttr(), earliestTime);
    if (angularVelocities.size() > 0  && angularVelocities.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:PointInstancer: (%s) angularVelocities defined but size does not match instance count\n",
                      instancer.GetPath().GetText());
    }

    // attributes that aren't currently supported go here
    VtArray<GfVec3f> scales;
    if (getAttributeArrayTimedFallback(scales, instancer.GetScalesAttr(), earliestTime) && scales.size() != indices.size())
    {
        CARB_LOG_WARN("Physics:PointInstancer: (%s) scales array size does not match instance count\n",
                      instancer.GetPath().GetText());
    }

    // support shared shapes only if we dont have scales provided. For scaled instances we cant share shapes
    const bool sharedShapes = scales.empty();

    SdfInt64ListOp inactiveIdsListOp;
    usdPrim.GetMetadata(UsdGeomTokens->inactiveIds, &inactiveIdsListOp);
    const std::vector<int64_t>& inactiveItemsVector = inactiveIdsListOp.GetExplicitItems();
    ::physx::PxBitMap inactiveIds;
    ::physx::PxBitMap activeTargets;
    if (!inactiveItemsVector.empty())
    {
        // we dont want to parse all targets, only those used.
        activeTargets.resize(::physx::PxU32(targets.size()));
        // get activeIds
        inactiveIds.resize(::physx::PxU32(indices.size()));
        for (size_t i = 0; i < inactiveItemsVector.size(); i++)
        {
            inactiveIds.set(::physx::PxU32(inactiveItemsVector[i]));
        }

        // if target is used set the bit for activeTargets        
        for (size_t index = 0; index < indices.size(); index++)
        {
            if (!inactiveIds.test(::physx::PxU32(index)))
            {
                activeTargets.set(::physx::PxU32(indices[index]));
            }            
        }
    }

    // traverse bitmap only if needed, its slower
    if (!inactiveItemsVector.empty())
    {
        ::physx::PxBitMap::Iterator it(activeTargets);
        for (::physx::PxU32 index = it.getNext(); index != ::physx::PxBitMap::Iterator::DONE; index = it.getNext())
        {
            ObjectInstance objectInstance = { usdPrim.GetPrimPath(), index, SdfPath(), false };
            TargetDesc targetDesc;
            UsdPrim targetPrim = attachedStage.getStage()->GetPrimAtPath(targets[index]);
            if (targetPrim)
            {
                targetDesc.targetPrim = targetPrim;
                PhysxObjectDesc* descOut = nullptr;
                const SdfPath descPath = parsePrototype(attachedStage, xfCache, targetPrim, filteredPairs, objectInstance,
                    sharedShapes ? nullptr : &targetDesc.shapeDescVector, &descOut);
                targetDesc.desc = descOut;
                targetDesc.descPath = descPath;
                targetDesc.outsideInstancer = isOutsideInstancer(targetPrim, usdPrim);
                targetObjects.push_back(targetDesc);
            }
            else
            {
                targetObjects.push_back(TargetDesc());
            }
        }
    }
    else
    {
        for (size_t i = 0; i < targets.size(); i++)
        {
            ObjectInstance objectInstance = { usdPrim.GetPrimPath(), (uint32_t)i, SdfPath(), false };
            const UsdPrim targetPrim = attachedStage.getStage()->GetPrimAtPath(targets[i]);
            if (targetPrim)
            {
                TargetDesc targetDesc;
                targetDesc.targetPrim = targetPrim;
                PhysxObjectDesc* descOut = nullptr;
                const SdfPath descPath = parsePrototype(attachedStage, xfCache, targetPrim, filteredPairs, objectInstance,
                    sharedShapes ? nullptr : &targetDesc.shapeDescVector, &descOut);
                targetDesc.desc = descOut;
                targetDesc.descPath = descPath;
                targetDesc.outsideInstancer = isOutsideInstancer(targetPrim, usdPrim);
                targetObjects.push_back(targetDesc);
            }
            else
            {
                targetObjects.push_back(TargetDesc());
            }
        }
    }

    UsdPrim topBodyPrim = UsdPrim();    
    GfMatrix4d topBodyMatrixInverse;    

    for (size_t i = 0; i < indices.size(); i++)
    {
        if (size_t(indices[i]) >= targets.size())
            continue;

        // skip inactiveItemsVector
        // A.B. we might consider traversing the BitMap here instead, though in most cases it might not be faster
        if (!inactiveItemsVector.empty())
        {
            if (inactiveIds.test(::physx::PxU32(i)))
            {
                continue; // skip inactive instances
            }
        }

        ObjectInstance objectInstance = { usdPrim.GetPrimPath(), (uint32_t)i, targets[indices[i]], false };
        const TargetDesc& targetDesc = targetObjects[indices[i]];
        PhysxObjectDesc* objectDesc = targetDesc.desc;

        if (!objectDesc)
            continue;

        if (objectDesc->type == eStaticBody || objectDesc->type == eDynamicBody)
        {
            PhysxRigidBodyDesc* sourceBodyDesc = static_cast<PhysxRigidBodyDesc*>(objectDesc);

            const GfVec3f instancePos = i < positions.size() ? positions[i] : GfVec3f(0.0f);
            const GfQuatf instanceOrient = i < orientations.size() ? GfQuatf(orientations[i]) : GfQuatf(1.0f);
            const GfVec3f instanceScale = i < scales.size() ? scales[i] : GfVec3f(1.0f);

            const ShapeDescVector& shapesDescs = targetObjects[indices[i]].shapeDescVector;
            // create shapes if scaling is used
            if (!shapesDescs.empty())
            {
                sourceBodyDesc->shapes.clear();
                for (size_t shapeIndex = 0; shapeIndex < shapesDescs.size(); shapeIndex++)
                {
                    const SdfPath& shapePath = shapesDescs[shapeIndex].first;
                    const PhysxShapeDesc& shapeDesc = *shapesDescs[shapeIndex].second;
                    PhysxShapeDesc* scaledShapeDesc = scaleShapeDesc(shapeDesc, instanceScale);
                    if (scaledShapeDesc)
                    {
                        ObjectId shapeId = kInvalidObjectId;
                        objectInstance.isExclusive = true;
                        createShape(attachedStage, shapePath, xfCache, scaledShapeDesc, &objectInstance, &shapeId);
                        if (shapeId != kInvalidObjectId)
                            sourceBodyDesc->shapes.push_back(shapeId);
                    }
                }
            }
            

            GfMatrix4d localBodyMatrix;
            localBodyMatrix.SetTranslate(GfVec3d(sourceBodyDesc->position.x, sourceBodyDesc->position.y, sourceBodyDesc->position.z));
            localBodyMatrix.SetRotateOnly(GfQuatd(sourceBodyDesc->rotation.w, sourceBodyDesc->rotation.x, sourceBodyDesc->rotation.y, sourceBodyDesc->rotation.z));
            localBodyMatrix = localBodyMatrix * instancerMatrixInverse;

            localBodyMatrix.SetTranslateOnly(GfCompMult(localBodyMatrix.ExtractTranslation(), GfVec3d(instanceScale)));

            GfMatrix4d instanceMatrix;
            instanceMatrix.SetTranslate(GfVec3d(instancePos));
            instanceMatrix.SetRotateOnly(instanceOrient);

            const GfMatrix4d bodyMatrix = localBodyMatrix * instanceMatrix * instancerMatrix;
        
            PhysxRigidBodyDesc* bodyDesc = nullptr;
            if (sourceBodyDesc->type == eDynamicBody)
            {
                bodyDesc = ICE_PLACEMENT_NEW(DynamicPhysxRigidBodyDesc)();
                DynamicPhysxRigidBodyDesc* dynamicBody = (DynamicPhysxRigidBodyDesc*)bodyDesc;
                *dynamicBody = *(DynamicPhysxRigidBodyDesc*)sourceBodyDesc;

                GfVec3ToFloat3(i < angularVelocities.size() ? degToRad(angularVelocities[i]) : GfVec3f(0.0f), dynamicBody->angularVelocity);

                GfVec3f transformedVelocity = i < velocities.size() ? velocities[i] : GfVec3f(0.0f);
                if (dynamicBody->localSpaceVelocities)
                {
                    transformedVelocity = instancerMatrix.Transform(transformedVelocity);
                }
                GfVec3ToFloat3(transformedVelocity, dynamicBody->linearVelocity);
            }
            else
            {
                bodyDesc = ICE_PLACEMENT_NEW(StaticPhysxRigidBodyDesc)();
                StaticPhysxRigidBodyDesc* staticBody = (StaticPhysxRigidBodyDesc*)bodyDesc;
                *staticBody = *(StaticPhysxRigidBodyDesc*)sourceBodyDesc;
            }

            GfVec3ToFloat3(bodyMatrix.ExtractTranslation(), bodyDesc->position);
            GfVec3ToFloat3(sc, bodyDesc->scale);
            GfQuatToFloat4(GfQuatf(bodyMatrix.RemoveScaleShear().ExtractRotation().GetQuat()), bodyDesc->rotation);

            const ObjectId pointInstancerBodyId = attachedStage.getObjectDatabase()->findEntry(usdPrim.GetPrimPath(), ePointInstancedBody);
            if (pointInstancerBodyId == kInvalidObjectId)
            {
                PointInstancedBodyDesc piDesc;
                const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, usdPrim.GetPrimPath(), piDesc, nullptr);
                attachedStage.getObjectDatabase()->findOrCreateEntry(usdPrim.GetPrimPath(), ePointInstancedBody, id);
            }

            const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, targets[indices[i]], *bodyDesc, &objectInstance);
            attachedStage.getObjectDatabase()->findOrCreateEntry(targets[indices[i]], eBody, id);
            bodyDesc->shapes.clear();

            ICE_FREE(bodyDesc);
        }
        else if (objectDesc->type > eShape && objectDesc->type < eBody)
        {
            // separate shapes, that do belong to a top level already created body            
            const PhysxShapeDesc* sourceShapeDesc = static_cast<PhysxShapeDesc*>(objectDesc);

            if (topBodyPrim == UsdPrim())
            {
                topBodyPrim = attachedStage.getStage()->GetPrimAtPath(sourceShapeDesc->rigidBody);
                CARB_ASSERT(topBodyPrim);
                
                topBodyMatrixInverse = xfCache.GetLocalToWorldTransform(topBodyPrim).GetInverse();
            }

            const GfVec3f instancePos = i < positions.size() ? positions[i] : GfVec3f(0.0f);
            const GfQuatf instanceOrient = i < orientations.size() ? GfQuatf(orientations[i]) : GfQuatf(1.0f);
            const GfVec3f instanceScale = i < scales.size() ? scales[i] : GfVec3f(1.0f);

            UsdPrim descPrim = attachedStage.getStage()->GetPrimAtPath(targetObjects[indices[i]].descPath);
            GfMatrix4d localShapeMatrix = xfCache.GetLocalToWorldTransform(descPrim);

            // Do this only if the object is below the instancer
            if (!targetObjects[indices[i]].outsideInstancer)
                localShapeMatrix = localShapeMatrix * instancerMatrixInverse;

            GfMatrix4d instanceMatrix;
            instanceMatrix.SetTranslate(GfVec3d(instancePos));
            instanceMatrix.SetRotateOnly(instanceOrient);

            const GfMatrix4d shapeWorldMatrix = localShapeMatrix * instanceMatrix * instancerMatrix;
            const GfMatrix4d newLocalShapeMatrix = shapeWorldMatrix * topBodyMatrixInverse;

            PhysxShapeDesc* scaledShapeDesc = scaleShapeDesc(*sourceShapeDesc, instanceScale);

            GfVec3ToFloat3(newLocalShapeMatrix.ExtractTranslation(), scaledShapeDesc->localPos);
            GfQuatToFloat4(newLocalShapeMatrix.ExtractRotationQuat(), scaledShapeDesc->localRot);

            const ObjectId pointInstancerBodyId = attachedStage.getObjectDatabase()->findEntry(usdPrim.GetPrimPath(), ePointInstancedBody);
            if (pointInstancerBodyId == kInvalidObjectId)
            {
                PointInstancedBodyDesc piDesc;
                const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, usdPrim.GetPrimPath(), piDesc, nullptr);
                attachedStage.getObjectDatabase()->findOrCreateEntry(usdPrim.GetPrimPath(), ePointInstancedBody, id);
            }

            objectInstance.isExclusive = true;
            createShape(attachedStage, descPrim.GetPrimPath(), xfCache, scaledShapeDesc, &objectInstance);
        }
    }

    for (size_t i = 0; i < targetObjects.size(); i++)
    {
        for (size_t shapeIndex = 0; shapeIndex < targetObjects[i].shapeDescVector.size(); shapeIndex++)
        {
            ICE_FREE(targetObjects[i].shapeDescVector[shapeIndex].second);
        }
    }

    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    for (size_t i = 0; i < targets.size(); i++)
    {
        attachedStage.bufferRequestRigidBodyMassUpdate(attachedStage.getStage()->GetPrimAtPath(targets[i]));
    }

    if (topBodyPrim != UsdPrim())
    {
        attachedStage.bufferRequestRigidBodyMassUpdate(topBodyPrim);
    }
}
} // namespace usdparser
} // namespace physx
} // namespace omni
