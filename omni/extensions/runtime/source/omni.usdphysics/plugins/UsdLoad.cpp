// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <private/omni/physics/schema/IUsdPhysics.h>
#include <common/utilities/UsdMaterialParsing.h>

#include <carb/logging/Log.h>

#include "UsdLoad.h"

#include "Scene.h"
#include "Articulation.h"
#include "Joint.h"
#include "Collision.h"
#include "RigidBody.h"
#include "DeformableBody.h"
#include "CollisionGroup.h"
#include "Attachment.h"

using namespace omni::physics;
using namespace schema;
using namespace carb;

using namespace pxr;


PhysicsSchemaUsdLoad::PhysicsSchemaUsdLoad()
{
}

PhysicsSchemaUsdLoad::~PhysicsSchemaUsdLoad()
{
}

void PhysicsSchemaUsdLoad::reportPrimDesc(const pxr::UsdPrim& prim, ObjectDesc* desc, uint64_t typeFlags, const TfTokenVector& appliedApis)
{
    for (size_t i = 0; i < mListeners.size(); i++)
    {
        mListeners[i]->parsePrim(prim, desc, typeFlags, appliedApis);
    }
}

void PhysicsSchemaUsdLoad::reportObjectDesc(const pxr::SdfPath& path, const ObjectDesc* desc)
{
    for (size_t i = 0; i < mListeners.size(); i++)
    {
        mListeners[i]->reportObjectDesc(path, desc);
    }
}

uint64_t fillDescCachePrimType(const pxr::TfType& primType, DescCache& descCache)
{
    static const TfType attachmentType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->Attachment);
    static const TfType vtxVtxType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxVtxAttachment);
    static const TfType vtxTriType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxTriAttachment);
    static const TfType vtxTetType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxTetAttachment);
    static const TfType vtxCrvType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxCrvAttachment);
    static const TfType vtxXformType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->VtxXformAttachment);
    static const TfType tetXformType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->TetXformAttachment);
    static const TfType triTriType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->TriTriAttachment);
    static const TfType elemFilterType = UsdSchemaRegistry::GetTypeFromSchemaTypeName(OmniPhysicsDeformableTypeTokens->ElementCollisionFilter);

    uint64_t types = 0;
    if (primType.IsA<UsdGeomImageable>())
    {
        types |= PrimType::eUsdGeomImageable;

        if (primType.IsA<UsdGeomXformable>())
        {
            types |= PrimType::eUsdGeomXformable;

            if (primType.IsA<UsdGeomGprim>())
            {
                types |= PrimType::eUsdGeomGprim;

                if (primType.IsA<UsdGeomMesh>())
                {
                    types |= PrimType::eUsdGeomMesh;
                }
                else if (primType.IsA<UsdGeomTetMesh>())
                {
                    types |= PrimType::eUsdGeomTetMesh;
                }
                else if (primType.IsA<UsdGeomBasisCurves>())
                {
                    types |= PrimType::eUsdGeomBasisCurves;
                }
            }
            else if (primType.IsA<UsdGeomPointInstancer>())
            {
                types |= PrimType::eUsdGeomPointInstancer;
            }
        }
        else if (primType.IsA<UsdPhysicsJoint>())
        {
            types |= PrimType::eUsdPhysicsJoint;
            if (primType.IsA<UsdPhysicsRevoluteJoint>())
            {
                types |= PrimType::eUsdPhysicsRevoluteJoint;
            }
            else if (primType.IsA<UsdPhysicsFixedJoint>())
            {
                types |= PrimType::eUsdPhysicsFixedJoint;
            }
            else if (primType.IsA<UsdPhysicsSphericalJoint>())
            {
                types |= PrimType::eUsdPhysicsSphericalJoint;
            }
            else if (primType.IsA<UsdPhysicsDistanceJoint>())
            {
                types |= PrimType::eUsdPhysicsDistanceJoint;
            }
            else if (primType.IsA<UsdPhysicsPrismaticJoint>())
            {
                types |= PrimType::eUsdPhysicsPrismaticJoint;
            }
        }
        else if (primType.IsA(attachmentType))
        {
            types |= PrimType::eUsdPhysicsAttachment;
            if (primType.IsA(vtxVtxType))
            {
                types |= PrimType::eUsdPhysicsVtxVtxAttachment;
            }
            else if (primType.IsA(vtxTriType))
            {
                types |= PrimType::eUsdPhysicsVtxTriAttachment;
            }
            else if (primType.IsA(vtxTetType))
            {
                types |= PrimType::eUsdPhysicsVtxTetAttachment;
            }
            else if (primType.IsA(vtxCrvType))
            {
                types |= PrimType::eUsdPhysicsVtxCrvAttachment;
            }
            else if (primType.IsA(vtxXformType))
            {
                types |= PrimType::eUsdPhysicsVtxXformAttachment;
            }
            else if (primType.IsA(tetXformType))
            {
                types |= PrimType::eUsdPhysicsTetXformAttachment;
            }
            else if (primType.IsA(triTriType))
            {
                types |= PrimType::eUsdPhysicsTriTriAttachment;
            }
        }
        else if (primType.IsA(elemFilterType))
        {
            types |= PrimType::eUsdPhysicsElementCollisionFilter;
        }
    }
    else if (primType.IsA<UsdPhysicsScene>())
    {
        types |= PrimType::eUsdPhysicsScene;
    }
    else if (primType.IsA<UsdPhysicsCollisionGroup>())
    {
        types |= PrimType::eUsdPhysicsCollisionGroup;
    }

    descCache.addPrimTypes(primType, types);

    return types;
}

struct SchemaAPIFlag
{
    enum Enum
    {
        eArticulationRootAPI            = 1 << 0,
        eCollisionAPI                   = 1 << 1,
        eRigidBodyAPI                   = 1 << 2,
        eDeformableBodyAPI              = 1 << 3,
        eMaterialAPI                    = 1 << 4,
        eDeformableMaterialAPI          = 1 << 5,
        eSurfaceDeformableMaterialAPI   = 1 << 6,
        eCurvesDeformableMaterialAPI    = 1 << 7
    };
};

bool PhysicsSchemaUsdLoad::loadFromRange(const pxr::UsdStageWeakPtr stage, UsdGeomXformCache& xfCache, PrimIteratorBase& primIterator, bool reset)
{
    bool retVal = true;

    if (!stage)
        return false;

    SceneMap sceneMap;
    SceneDesc* defaultScene = nullptr;
    SdfPath defaultScenePath;
    BodyMap bodyMap;
    JointMap jointMap;
    std::vector<std::pair<pxr::SdfPath, JointDesc*>> jointVector;
    CollisionGroupMap collisionGroupsMap;
    MaterialMap materialMap;
    ShapeMap shapeMap;
    ArticulationMap articulationMap;
    AttachmentMap attachmentMap;
    CollisionFilterMap collisionFilterMap;

    // store the descs ptrs here, we release them at the end of parsing
    std::vector<ObjectDesc*> descVector;
    descVector.reserve(2048);
    mDescCache.clear();

    // parse for scene first, get the descriptors, report all prims
    // the descriptors are not complete yet
    if (reset)
        primIterator.reset();

    static const TfToken gRigidBodyAPIToken("PhysicsRigidBodyAPI");
    static const TfToken gDeformableBodyAPIToken("OmniPhysicsDeformableBodyAPI");
    static const TfToken gCollisionAPIToken("PhysicsCollisionAPI");
    static const TfToken gArticulationRootAPIToken("PhysicsArticulationRootAPI");
    static const TfToken gMaterialAPIToken("PhysicsMaterialAPI");
    static const TfToken gDeformableMaterialAPIToken("OmniPhysicsDeformableMaterialAPI");
    static const TfToken gSurfaceDeformableMaterialAPIToken("OmniPhysicsSurfaceDeformableMaterialAPI");
    static const TfToken gCurveDeformableMaterialAPIToken("OmniPhysicsCurveDeformableMaterialAPI");

    {
        while (!primIterator.atEnd())
        {
            const pxr::UsdPrim& prim = *primIterator.getCurrent();
            if (!prim)
            {
                primIterator.pruneChildren();
                primIterator.next();
                continue;
            }

            const SdfPath primPath = prim.GetPrimPath();
            const UsdPrimTypeInfo& typeInfo = prim.GetPrimTypeInfo();
            ObjectDesc* reportDesc = nullptr;

            uint64_t typeFlags = 0;
            const bool knownType = mDescCache.getPrimTypes(typeInfo.GetSchemaType(), typeFlags);
            if (!knownType)
            {
                typeFlags = fillDescCachePrimType(typeInfo.GetSchemaType(), mDescCache);
            }

            uint64_t apiFlags = 0;
            const TfTokenVector& apis = prim.GetPrimTypeInfo().GetAppliedAPISchemas();
            for (const TfToken& token : apis)
            {
                if (token == gArticulationRootAPIToken)
                {
                    apiFlags |= SchemaAPIFlag::eArticulationRootAPI;
                }
                else if (token == gCollisionAPIToken)
                {
                    apiFlags |= SchemaAPIFlag::eCollisionAPI;
                }
                else if (token == gRigidBodyAPIToken)
                {
                    apiFlags |= SchemaAPIFlag::eRigidBodyAPI;
                }
                else if (token == gDeformableBodyAPIToken)
                {
                    apiFlags |= SchemaAPIFlag::eDeformableBodyAPI;
                }
                else if (token == gMaterialAPIToken)
                {
                    apiFlags |= SchemaAPIFlag::eMaterialAPI;
                }
                else if (token == gDeformableMaterialAPIToken)
                {
                    apiFlags |= SchemaAPIFlag::eDeformableMaterialAPI;
                }
                else if (token == gSurfaceDeformableMaterialAPIToken)
                {
                    apiFlags |= SchemaAPIFlag::eSurfaceDeformableMaterialAPI;
                }
                else if (token == gCurveDeformableMaterialAPIToken)
                {
                    apiFlags |= SchemaAPIFlag::eCurvesDeformableMaterialAPI;
                }
            }

            // ignore material APIs if other APIs are present (invalid config)
            {
                static const uint64_t materialAPIs = SchemaAPIFlag::eMaterialAPI | SchemaAPIFlag::eDeformableMaterialAPI |
                    SchemaAPIFlag::eSurfaceDeformableMaterialAPI | SchemaAPIFlag::eCurvesDeformableMaterialAPI;

                uint64_t matFlags = apiFlags & materialAPIs;
                uint64_t otherFlags = apiFlags & ~materialAPIs;
                if (matFlags > 0 && otherFlags > 0)
                {
                    apiFlags &= otherFlags;
                }
            }

            if (typeFlags & PrimType::eUsdGeomPointInstancer)
            {
                primIterator.pruneChildren(); // Skip the subtree rooted at this prim
            }
            else if (!mCustomPhysicsInstancerTokens.empty())
            {
                for (const TfToken& instToken : mCustomPhysicsInstancerTokens)
                {
                    if (instToken == typeInfo.GetTypeName())
                    {
                        primIterator.pruneChildren(); // Skip the subtree rooted at this prim
                        break;
                    }
                }
            }

            if (typeFlags & PrimType::eUsdPhysicsScene)
            {
                SceneDesc* desc = parseSceneDesc(stage, prim);
                if (desc)
                {
                    if (!defaultScene)
                    {
                        defaultScene = desc;
                        defaultScenePath = primPath;
                    }
                    desc->usdPrim = prim;
                    sceneMap[primPath] = desc;
                    descVector.push_back(desc);
                    reportDesc = desc;
                }
            }
            else if (typeFlags & PrimType::eUsdPhysicsCollisionGroup)
            {
                CollisionGroupDesc* desc = parseCollisionGroup(stage, prim);
                if (desc)
                {
                    desc->usdPrim = prim;
                    collisionGroupsMap[primPath] = desc;
                    descVector.push_back(desc);
                    reportDesc = desc;
                }
            }
            else if (apiFlags & SchemaAPIFlag::eMaterialAPI)
            {
                MaterialDesc* desc = new MaterialDesc();
                usdmaterialutils::parseMaterial(stage, prim, *desc);
                if (desc)
                {
                    desc->usdPrim = prim;
                    materialMap[primPath] = desc;
                    descVector.push_back(desc);
                    reportDesc = desc;
                }
            }
            else if (apiFlags & SchemaAPIFlag::eDeformableMaterialAPI)
            {
                DeformableMaterialDesc* desc = nullptr;
                if (apiFlags & SchemaAPIFlag::eSurfaceDeformableMaterialAPI)
                {
                    desc = new SurfaceDeformableMaterialDesc();
                    usdmaterialutils::parseSurfaceDeformableMaterial(stage, prim,
                        *static_cast<SurfaceDeformableMaterialDesc*>(desc));
                }
                else if (apiFlags & SchemaAPIFlag::eCurvesDeformableMaterialAPI)
                {
                    desc = new CurvesDeformableMaterialDesc();
                    usdmaterialutils::parseCurvesDeformableMaterial(stage, prim,
                        *static_cast<CurvesDeformableMaterialDesc*>(desc));
                }
                else
                {
                    desc = new DeformableMaterialDesc();
                    usdmaterialutils::parseDeformableMaterial(stage, prim, *desc);
                }
                if (desc)
                {
                    desc->usdPrim = prim;
                    materialMap[primPath] = desc;
                    descVector.push_back(desc);
                    reportDesc = desc;
                }
            }
            else if (typeFlags & PrimType::eUsdPhysicsJoint)
            {
                // joint parse
                {
                    JointDesc* desc = parseJoint(stage, prim, mCustomJointTokens, typeFlags);
                    if (desc)
                    {
                        desc->usdPrim = prim;
                        jointMap[primPath] = desc;
                        jointVector.push_back(std::make_pair(primPath, desc));
                        descVector.push_back(desc);
                        reportDesc = desc;
                    }
                }
                // can be articulation definition
                if (apiFlags & SchemaAPIFlag::eArticulationRootAPI)
                {
                    ArticulationDesc* desc = parseArticulation(stage, prim, articulationMap);
                    if (desc)
                    {
                        desc->usdPrim = prim;
                        articulationMap[primPath] = desc;
                        descVector.push_back(desc);
                        reportDesc = desc;
                    }
                }
            }
            else if (typeFlags & PrimType::eUsdPhysicsAttachment)
            {
                AttachmentDesc* desc = parseAttachment(stage, prim, typeFlags);
                if (desc)
                {
                    desc->usdPrim = prim;
                    attachmentMap[primPath] = desc;
                    descVector.push_back(desc);
                    reportDesc = desc;
                }
            }
            else if (typeFlags & PrimType::eUsdPhysicsElementCollisionFilter)
            {
                ElementCollisionFilterDesc* desc = parseCollisionFilter(stage, prim);
                if (desc)
                {
                    desc->usdPrim = prim;
                    collisionFilterMap[primPath] = desc;
                    descVector.push_back(desc);
                    reportDesc = desc;
                }
            }
            else
            {
                if ((apiFlags & SchemaAPIFlag::eCollisionAPI) && !(apiFlags & SchemaAPIFlag::eDeformableBodyAPI))
                {
                    bool isDeformableCollider = false;
                    {
                        UsdPrim deformableBodyPrim;
                        bool hasVolumeParent = hasEnabledBodyParent(stage, prim, bodyMap, deformableBodyPrim, ObjectType::eVolumeDeformableBody);
                        bool hasSurfaceParent = hasEnabledBodyParent(stage, prim, bodyMap, deformableBodyPrim, ObjectType::eSurfaceDeformableBody);
                        bool hasCurvesParent = hasEnabledBodyParent(stage, prim, bodyMap, deformableBodyPrim, ObjectType::eCurvesDeformableBody);
                        if (hasVolumeParent || hasSurfaceParent || hasCurvesParent)
                        {
                            //TODO consider xfCache.GetResetXformStack(bodyPrim) inside hasEnabledBodyParent?
                            isDeformableCollider = true;
                        }
                    }

                    if (!isDeformableCollider)
                    {
                        std::vector<ShapeDesc*> shapes;
                        bool instance = false;
                        if (prim.IsInstanceProxy())
                        {
                            // compute the local to world pose to cache the value
                            xfCache.GetLocalToWorldTransform(prim);
                            const UsdPrim shapePrim = prim.GetPrimInPrototype();
                            const SdfPath shapePrimPath = shapePrim.GetPrimPath();
                            const ShapeDesc* masterDesc = mDescCache.getDesc(shapePrimPath);
                            if (!masterDesc)
                            {
                                parseCollision(stage, xfCache, shapePrim, mCustomShapeTokens, apis, shapes);
                                CARB_ASSERT(shapes.size() == 1);
                                if (!shapes.empty())
                                {
                                    shapes[0]->masterDesc = true;
                                    mDescCache.addDesc(shapePrimPath, shapes[0]);
                                }
                            }
                            else
                            {
                                shapes.push_back((ShapeDesc*)masterDesc);
                            }
                            instance = true;
                        }
                        else
                        {
                            parseCollision(stage, xfCache, prim, mCustomShapeTokens, apis, shapes);
                        }
                        for (size_t i = 0; i < shapes.size(); i++)
                        {
                            ShapeDesc* desc = shapes[i];
                            desc->usdPrim = prim;
                            shapeMap.push_back(std::make_pair(primPath, desc));
                            if (!instance)
                                descVector.push_back(desc);
                            reportDesc = desc;
                        }
                    }
                }
                if (apiFlags & SchemaAPIFlag::eRigidBodyAPI)
                {
                    bool reportInstanceError = false;
                    if (prim.IsInstanceProxy())
                    {
                        reportInstanceError = true;

                        const pxr::UsdPhysicsRigidBodyAPI rbAPI(prim);
                        bool kinematic = false;
                        rbAPI.GetKinematicEnabledAttr().Get(&kinematic);
                        if (kinematic)
                            reportInstanceError = false;

                        bool enabled = false;
                        rbAPI.GetRigidBodyEnabledAttr().Get(&enabled);
                        if (!enabled)
                            reportInstanceError = false;

                        if (reportInstanceError)
                        {
                            CARB_LOG_WARN("RigidBodyAPI on an instance proxy not supported, unless set to kinematic or not enabled. %s",
                                prim.GetPrimPath().GetText());
                        }
                    }

                    if (!reportInstanceError)
                    {
                        RigidBodyDesc* desc = parseRigidBody(stage, xfCache, prim, bodyMap, typeFlags);
                        if (desc)
                        {
                            desc->usdPrim = prim;
                            bodyMap[primPath] = desc;
                            descVector.push_back(desc);
                            reportDesc = desc;
                        }
                    }
                }
                if (apiFlags & SchemaAPIFlag::eDeformableBodyAPI)
                {
                    bool reportInstanceError = false;
                    if (prim.IsInstanceProxy())
                    {
                        reportInstanceError = true;

                        bool enabled = false;
                        prim.GetAttribute(OmniPhysicsDeformableAttrTokens->deformableBodyEnabled).Get(&enabled);
                        if (!enabled)
                            reportInstanceError = false;

                        if (reportInstanceError)
                        {
                            CARB_LOG_WARN(
                                "DeformableBodyAPI on an instance proxy not supported, unless not enabled. %s",
                                prim.GetPrimPath().GetText());
                        }
                    }

                    if (!reportInstanceError)
                    {
                        DeformableBodyDesc* desc = parseDeformableBody(stage, xfCache, prim, bodyMap, typeFlags);

                        if (desc)
                        {
                            desc->usdPrim = prim;
                            bodyMap[primPath] = desc;
                            descVector.push_back(desc);
                            reportDesc = desc;
                        }
                    }
                }
                if (apiFlags & SchemaAPIFlag::eArticulationRootAPI)
                {
                    ArticulationDesc* desc = parseArticulation(stage, prim, articulationMap);
                    if (desc)
                    {
                        desc->usdPrim = prim;
                        articulationMap[primPath] = desc;
                        descVector.push_back(desc);
                        reportDesc = desc;
                    }
                }
            }

            reportPrimDesc(prim, reportDesc, typeFlags, apis);

            primIterator.next();
        }
    }

    // get the descriptors, finalize them and send them out in an order
    // 1. send out the scene, first the defaultScene
    if (defaultScene)
    {
        reportObjectDesc(defaultScenePath, defaultScene);

        for (const auto& it : sceneMap)
        {
            if (it.first != defaultScenePath)
                reportObjectDesc(it.first, it.second);
        }
    }

    // 2. send out the CollisionGroups
    // This is required as collisions by default dont have collisionGroup assigned
    // this has to be done in the implementation part, the collisionGroups need
    // to be stored in a global list so that sub range parse knows about all groups.
    for (const auto& it : collisionGroupsMap)
    {
        reportObjectDesc(it.first, it.second);
    }

    // 3. send out the materials
    for (const auto& it : materialMap)
    {
        reportObjectDesc(it.first, it.second);
    }

    // 4. send out shapes
    {
        for (const auto& it : shapeMap)
        {
            // get the body
            SdfPath bodyPath = getRigidBody(stage, stage->GetPrimAtPath(it.first), bodyMap);
            // body was found, add collision to the body
            RigidBodyDesc* bodyDesc = nullptr;
            if (bodyPath != SdfPath())
            {
                BodyMap::iterator bodyIt = bodyMap.find(bodyPath);
                bodyDesc = static_cast<RigidBodyDesc*>(bodyIt->second);
                bodyDesc->collisions.insert(it.first);
            }

            // finalize the collision, fill up the local transform etc
            finalizeCollision(stage, bodyDesc, xfCache, it.second);
            reportObjectDesc(it.first, it.second);
        }
    }

    // 5. finalize joints
    {
        for (const auto& it : jointMap)
        {
            finalizeJoint(stage, it.second, xfCache);
        }
    }

    // 6. send out articulations
    {
        finalizeArticulations(stage, articulationMap, bodyMap, jointMap);
        for (const auto& it : articulationMap)
        {
            if (!it.second->rootPrims.empty())
                reportObjectDesc(it.first, it.second);
        }
    }

    // 7. send out bodies
    {
        for (const auto& it : bodyMap)
        {
            reportObjectDesc(it.first, it.second);
        }
    }

    // 8. send out joints
    for (const auto& it : jointVector)
    {
        reportObjectDesc(it.first, it.second);
    }

    // 9. send out attachments
    {
        for (const auto& it : attachmentMap)
        {
            reportObjectDesc(it.first, it.second);
        }
    }

    // 10. send out collision filters
    {
        for (const auto& it : collisionFilterMap)
        {
            reportObjectDesc(it.first, it.second);
        }
    }

    // release descs memory
    for (size_t i = 0; i < descVector.size(); i++)
    {
        delete descVector[i];
    }
    descVector.clear();

    return retVal;
}

void PhysicsSchemaUsdLoad::registerPhysicsListener(IUsdPhysicsListener* listener)
{
    CARB_ASSERT(listener);
    mListeners.push_back(listener);
}

void PhysicsSchemaUsdLoad::unregisterPhysicsListener(IUsdPhysicsListener* listener)
{
    CARB_ASSERT(listener);
    for (size_t i = 0; i < mListeners.size(); i++)
    {
        if (mListeners[i] == listener)
        {
            mListeners[i] = mListeners.back();
            mListeners.pop_back();
            return;
        }
    }

    CARB_LOG_WARN("PhysicsSchemaUsdLoad: Listener not found for unregister.");
}
