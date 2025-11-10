// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/Defines.h>
#include <carb/profiler/Profile.h>
#include <carb/logging/Log.h>

#include <omni/core/ITypeFactory.h>

#include "FabricSync.h"
#include "LoadTools.h"
#include "LoadUsd.h"
#include "PrimUpdate.h"
#include "LoadStage.h"
#include "Collision.h"
#include <OmniPhysX.h>
#include <PhysXTools.h>

#include <omni/physx/PhysxTokens.h>

#include "particles/FabricParticles.h"

using namespace pxr;
using namespace carb;
using namespace omni::physics::schema;

const omni::fabric::IToken* omni::fabric::Token::iToken = nullptr;
const omni::fabric::IPath* omni::fabric::Path::iPath = nullptr;

static TfToken fabricWorldMatrix(omni::physx::gWorldMatrixTokenString);
static TfToken fabricLocalMatrix(omni::physx::gLocalMatrixTokenString);
static TfToken fabricPositionInvMasses(omni::physx::gPositionInvMassesTokenString);
static TfToken fabricVelocitiesFloat4(omni::physx::gVelocitiesFloat4TokenString);

namespace omni
{
namespace physx
{
namespace usdparser
{


template <typename T>
bool checkSchemaAttribute(const pxr::TfToken& attributeName)
{
    const TfTokenVector& apiAttributes = T::GetSchemaAttributeNames();
    for (size_t i = apiAttributes.size(); i--;)
    {
        if (apiAttributes[i] == attributeName)
            return true;
    }

    return false;
}

bool isMovableBody(const AttachedStage& attachedStage, const pxr::SdfPath& primPath)
{
    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(primPath);
    if (entries && !entries->empty())
    {
        auto it = entries->begin();
        while (it != entries->end())
        {
            if (it->first == eBody || it->first == ePhysxForce || it->first == eArticulationLink ||
                it->first == eParticleSet || it->first == eParticleCloth ||
                it->first == eSoftBody || it->first == eFEMCloth || it->first == eXformActor ||
                it->first == eVolumeDeformableBody || it->first == eSurfaceDeformableBody)
            {
                return true;
            }
            it++;
        }
    }

    return false;
}

bool isNonMovable(const AttachedStage& attachedStage, const pxr::UsdPrim& prim, pxr::UsdPrim& resyncPrim)
{
    const uint64_t primAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(prim.GetPrimPath());
    if (primAPIs & SchemaAPIFlag::eDeformableBodyAPI)
    {
        resyncPrim = prim;
        return true;
    }

    // if we changed the transform of a shape or a transform between the
    // body and the shape we cannot move that, we reconstruct
    UsdPrim parent = prim.GetParent();
    while (parent)
    {
        const uint64_t parentAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(parent.GetPrimPath());
        if (parentAPIs & SchemaAPIFlag::eRigidBodyAPI)
        {
            if (primAPIs & SchemaAPIFlag::eCollisionAPI)
            {
                resyncPrim = prim;
                return true;
            }
            UsdPrimRange range = UsdPrimRange::AllPrims(prim);
            for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
            {
                const pxr::UsdPrim& primC = *iter;
                if (!primC)
                    continue;
                const uint64_t primCAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primC.GetPrimPath());
                if (primCAPIs & SchemaAPIFlag::eCollisionAPI)
                {
                    resyncPrim = prim;
                    return true;
                }
            }
            return false;
        }
        else if (parentAPIs & SchemaAPIFlag::eDeformableBodyAPI)
        {
            resyncPrim = parent;
            return true;
        }

        parent = parent.GetParent();
    }
    return false;
}

void handleRemovedPrim(AttachedStage& attachedStage, const SdfPath& primPath)
{
    attachedStage.getPrimChangeMap().removePrim(primPath);

    if (!attachedStage.getStage())
        return;

    const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(primPath);
    if (entries && !entries->empty())
    {
        auto it = entries->begin();
        while (it != entries->end())
        {
            attachedStage.getPhysXPhysicsInterface()->releaseObject(attachedStage, primPath, it->second);
            it++;
        }

        attachedStage.getObjectDatabase()->removeEntries(primPath);
        attachedStage.getObjectDatabase()->removeSchemaAPIs(primPath);
    }
}

SdfPath findDeformableBodyAncestor(AttachedStage& attachedStage, const SdfPath primPath)
{
    TfType dbType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
    pxr::UsdPrim prim = attachedStage.getStage()->GetPrimAtPath(primPath);
    pxr::UsdPrim deformableBodyPrim = pxr::UsdPrim();
    while (prim && !deformableBodyPrim)
    {
        if (prim.HasAPI(dbType))
        {
            deformableBodyPrim = prim;
        }
        else if (prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>())
        {
            prim = pxr::UsdPrim();
        }
        else
        {
            pxr::UsdGeomXformable xformable(prim);
            bool resetXformStack = xformable && xformable.GetResetXformStack();
            prim = resetXformStack ? pxr::UsdPrim() : prim.GetParent();
        }
    }
    return deformableBodyPrim.GetPath();
}

PrimChangeMap::PrimChangeMap()
{
}

PrimChangeMap::~PrimChangeMap()
{
    m_propertyChanges.clear();
}

void PrimChangeMap::registerPrimChange(const ChangeParams& changeParam)
{
    CARB_ASSERT(!changeParam.changeAttribute.empty());
    PropertyChange pc;
    pc.onPrimCheck = changeParam.onPrimCheck;
    pc.onPrimCheckExt = changeParam.onPrimCheckExt;
    pc.onUpdate = changeParam.onUpdate;
    m_propertyChanges.insert(std::pair<pxr::TfToken, PropertyChange>(TfToken(changeParam.changeAttribute.c_str()), pc));
}

void PrimChangeMap::clearRegisteredChanges()
{
    m_propertyChanges.clear();
}

void PrimChangeMap::registerStageSpecificChange(const ChangeParams& changeParam)
{
    PropertyChange pc;
    pc.onPrimCheck = changeParam.onPrimCheck;
    pc.onPrimCheckExt = changeParam.onPrimCheckExt;
    pc.onUpdate = changeParam.onUpdate;
    m_stageSpecificChanges.insert(std::pair<pxr::TfToken, PropertyChange>(TfToken(changeParam.changeAttribute.c_str()), pc));
}

void PrimChangeMap::clearStageSpecificChanges()
{
    m_stageSpecificChanges.clear();
}

void moveBody(AttachedStage& attachedStage, const pxr::SdfPath& primPath, const pxr::UsdPrim* primIn, UsdGeomXformCache* xfCache, bool fastCache)
{
    UsdLoad* usdLoad = UsdLoad::getUsdLoad();

    // read the xform directly from body prim not from fastcache
    // the notification came from USD, so lets use USD data
    PhysXUsdPhysicsInterface::Transform fcTransform;

    bool scaleProvided = true;
    if (!fastCache)
    {
        const UsdPrim prim = primIn ? *primIn : attachedStage.getStage()->GetPrimAtPath(primPath);

        GfMatrix4d mat = xfCache->GetLocalToWorldTransform(prim);
        const GfTransform tr(mat);
        const GfVec3d pos = tr.GetTranslation();
        const GfQuatd rot = tr.GetRotation().GetQuat();
        const GfVec3d sc = tr.GetScale();

        fcTransform.position = { float(pos[0]), float(pos[1]), float(pos[2]) };
        fcTransform.orientation = { float(rot.GetImaginary()[0]), float(rot.GetImaginary()[1]),
                                    float(rot.GetImaginary()[2]), float(rot.GetReal()) };
        fcTransform.scale = { float(sc[0]), float(sc[1]), float(sc[2]) };
    }
    else
    {
        omni::fabric::IStageReaderWriter* iStageReaderWriter = OmniPhysX::getInstance().getIStageReaderWriter();
        if (iStageReaderWriter)
        {
            const omni::fabric::UsdStageId stageId = { uint64_t(attachedStage.getStageId()) };
            auto stageInProgress = iStageReaderWriter->get(stageId);

            if (stageInProgress.id)
            {
                if (!omni::fabric::Token::iToken)
                    omni::fabric::Token::iToken = carb::getCachedInterface<omni::fabric::IToken>();

                // Grab a pointer to in-memory representation for the attribute value, in this
                // case a pointer to a T. Will be NULL if attribute doesn't exist in fabric
                {
                    auto valueSpan =
                        iStageReaderWriter->getAttributeRd(stageInProgress, omni::fabric::asInt(primPath),
                            *usdLoad->getFabricTokens().worldMatrix);

                    const pxr::GfMatrix4d* valuePtr = (const pxr::GfMatrix4d*)valueSpan.ptr;

                    if (valuePtr)
                    {
                        // We have a value stored for this attribute in fabric, return it
                        const GfTransform tr(*valuePtr);
                        const GfVec3d pos = tr.GetTranslation();
                        const GfQuatd rot = tr.GetRotation().GetQuat();
                        const GfVec3d sc = tr.GetScale();

                        fcTransform.position = { float(pos[0]), float(pos[1]), float(pos[2]) };
                        fcTransform.orientation = { float(rot.GetImaginary()[0]), float(rot.GetImaginary()[1]),
                                                    float(rot.GetImaginary()[2]), float(rot.GetReal()) };
                        fcTransform.scale = { float(sc[0]), float(sc[1]), float(sc[2]) };

                        scaleProvided = true;
                    }
                }
            }
        }
        else
        {
            return;
        }
    }

    bool structChange = false;
    bool articulationLinkChange = false;

    const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(primPath);
    if (entries && !entries->empty())
    {
        auto it = entries->begin();
        while (it != entries->end())
        {
            if (!attachedStage.getPhysXPhysicsInterface()->updateTransform(attachedStage,
                primPath, it->second, fcTransform, true, scaleProvided))
            {
                structChange = true;
                if (it->first == eArticulationLink)
                {
                    CARB_LOG_ERROR("Articulation scale changed for %s in runtime, this does trigger full stage reparse.", primPath.GetText());
                    articulationLinkChange = true;
                }
            }
            it++;
        }
    }

    if (structChange)
    {
        if (articulationLinkChange)
        {
            // A.B. we could do eventually better, but we might not want to reconstruct the hierarchy here atm
            UsdLoad::getUsdLoad()->releasePhysicsObjects(attachedStage.getStageId());
        }
        else
        {
            const UsdPrim prim = primIn ? *primIn : attachedStage.getStage()->GetPrimAtPath(primPath);

            PrimHierarchyStorage& primStorage = attachedStage.getObjectDatabase()->getPrimHierarchyStorage();
            PrimHierarchyStorage::Iterator iterator(primStorage, primPath);
            for (size_t i = iterator.getDescendentsPaths().size(); i--;)
            {
                const pxr::SdfPath& primCPath = iterator.getDescendentsPaths()[i];
                handleRemovedPrim(attachedStage, primCPath);
            }
            primStorage.removeIteration(iterator);
            attachedStage.getPrimUpdateMap().addPrim(attachedStage, prim);
        }
    }
}

void movePointInstancer(AttachedStage& attachedStage, const pxr::UsdPrim& prim)
{
    const UsdGeomPointInstancer& instancer = (const UsdGeomPointInstancer&)prim;
    UsdRelationship prototypes = instancer.GetPrototypesRel();
    UsdAttributeVector attributes = prim.GetAttributes();

    SdfPathVector targets;
    prototypes.GetTargets(&targets);
    std::vector<ObjectIdMap::const_iterator> targetEntries;

    VtArray<int> indices;
    VtArray<GfVec3f> positions;
    VtArray<GfQuath> orientations;

    static const TfToken posToken("positions");
    static const TfToken rotToken("orientations");
    static const TfToken indicesToken("protoIndices");
    for (UsdAttribute attr : attributes)
    {
        const TfToken usdPropName = attr.GetName();

        if (usdPropName == posToken)
        {
            getAttributeArray(positions, attr);
        }
        else if (usdPropName == rotToken)
        {
            getAttributeArray(orientations, attr);
        }
        else if (usdPropName == indicesToken)
        {
            getAttributeArray(indices, attr);
        }
    }

    PhysXUsdPhysicsInterface::Transform fcTransform;

    UsdGeomXformCache xfCache;

    const GfMatrix4d instancerMatrix = xfCache.GetLocalToWorldTransform(prim);
    const GfQuatf instancerRotation = GfQuatf(instancerMatrix.ExtractRotation().GetQuat());
    const GfVec3d sc = GfTransform(instancerMatrix).GetScale();
    fcTransform.scale = { float(sc[0]), float(sc[1]), float(sc[2]) };

    bool structChange = false;

    targetEntries.clear();

    UsdLoad* usdLoad = UsdLoad::getUsdLoad();

    for (size_t iProt = 0; iProt < targets.size(); iProt++)
    {
        const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(targets[iProt]);
        if (!entries)
        {
            structChange = true;
            break;
        }
        targetEntries.push_back(entries->begin());
    }

    if (!structChange)
    {
        for (size_t i = 0; i < indices.size(); i++)
        {
            if (size_t(indices[i]) < targets.size())
            {
                const GfVec3f instancePos = i < positions.size() ? positions[i] : GfVec3f(0.0f);
                const GfVec3f transfPos = instancerMatrix.Transform(instancePos);

                const GfQuatf instanceOrient = i < orientations.size() ? GfQuatf(orientations[i]) : GfQuatf(1.0f);
                const GfQuatf transfOrient = instanceOrient * instancerRotation;

                fcTransform.position = { float(transfPos[0]), float(transfPos[1]), float(transfPos[2]) };
                fcTransform.orientation = { float(transfOrient.GetImaginary()[0]), float(transfOrient.GetImaginary()[1]),
                                            float(transfOrient.GetImaginary()[2]), float(transfOrient.GetReal()) };

                ObjectIdMap::const_iterator& it = targetEntries[indices[i]];
                if (!attachedStage.getPhysXPhysicsInterface()->updateTransform(attachedStage,
                    targets[indices[i]], it->second, fcTransform))
                {
                    structChange = true;
                    break;
                }
                it++;

                if (structChange)
                    break;
            }
        }
    }


    if (structChange)
    {
        attachedStage.getPrimUpdateMap().removePrim(attachedStage, prim.GetPath());
        attachedStage.getPrimUpdateMap().addPrim(attachedStage, prim);
    }
}

void PrimChangeMap::handleTransformChange(AttachedStage& attachedStage, const pxr::SdfPath& primPath, const pxr::UsdPrim* primIn, pxr::UsdGeomXformCache* xfCache, bool fastCache)
{
    TRACE_FUNCTION();
    if (attachedStage.getObjectDatabase()->empty())
        return;


    if (isMovableBody(attachedStage, primPath))
    {
        moveBody(attachedStage, primPath, primIn, xfCache, fastCache);
    }
    else
    {
        const UsdPrim prim = primIn ? *primIn : attachedStage.getStage()->GetPrimAtPath(primPath);

        if (prim.IsA<UsdGeomPointInstancer>())
        {
            movePointInstancer(attachedStage, prim);
        }
        else
        {
            // check the parents
            UsdPrim resyncPrim;
            if (isNonMovable(attachedStage, prim, resyncPrim))
            {
                PrimHierarchyStorage& primStorage = attachedStage.getObjectDatabase()->getPrimHierarchyStorage();
                PrimHierarchyStorage::Iterator iterator(primStorage, resyncPrim.GetPrimPath());
                for (size_t i = iterator.getDescendentsPaths().size(); i--;)
                {
                    const pxr::SdfPath& primCPath = iterator.getDescendentsPaths()[i];
                    handleRemovedPrim(attachedStage, primCPath);
                }
                primStorage.removeIteration(iterator);
                attachedStage.getPrimUpdateMap().addPrim(attachedStage, resyncPrim);
            }
            else
            {
                // we could have changed a parent node for more prims, lets update
                UsdPrimRange range(prim);
                for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
                {
                    const pxr::UsdPrim& primC = *iter;
                    if (!primC)
                        continue;
                    if (isMovableBody(attachedStage, primC.GetPrimPath()))
                    {
                        iter.PruneChildren();
                        moveBody(attachedStage, primC.GetPrimPath(), &primC, xfCache, fastCache);
                    }
                }
            }
        }
    }
}

static const TfToken gMimicJointRotXAPIToken("PhysxMimicJointAPI:rotX");
static const TfToken gMimicJointRotYAPIToken("PhysxMimicJointAPI:rotY");
static const TfToken gMimicJointRotZAPIToken("PhysxMimicJointAPI:rotZ");
static const TfToken gMimicJointAPITokenList[] = {
    gMimicJointRotXAPIToken, gMimicJointRotYAPIToken, gMimicJointRotZAPIToken
};
static const uint32_t gMimicJointAPITokenCount = sizeof(gMimicJointAPITokenList) / sizeof(gMimicJointAPITokenList[0]);
static const SchemaAPIFlag::Enum gMimicJointSchemaAPIFlagList[gMimicJointAPITokenCount] = {
    SchemaAPIFlag::eMimicJointRotXAPI, SchemaAPIFlag::eMimicJointRotYAPI, SchemaAPIFlag::eMimicJointRotZAPI
};

static const pxr::TfToken gDrivePerformanceEnvelopeAPINameTokensInstanced[5] =
{
    pxr::TfToken("PhysxDrivePerformanceEnvelopeAPI:angular"),
    pxr::TfToken("PhysxDrivePerformanceEnvelopeAPI:linear"),
    pxr::TfToken("PhysxDrivePerformanceEnvelopeAPI:rotX"),
    pxr::TfToken("PhysxDrivePerformanceEnvelopeAPI:rotY"),
    pxr::TfToken("PhysxDrivePerformanceEnvelopeAPI:rotZ")
};
static const SchemaAPIFlag::Enum  gDrivePerformanceEnvelopAPIFlagList[] =
{
 SchemaAPIFlag::eDrivePerformanceEnvelopeAngularAPI,
 SchemaAPIFlag::eDrivePerformanceEnvelopeLinearAPI,
 SchemaAPIFlag::eDrivePerformanceEnvelopeRotXAPI,
 SchemaAPIFlag::eDrivePerformanceEnvelopeRotYAPI,
 SchemaAPIFlag::eDrivePerformanceEnvelopeRotZAPI
};

static const pxr::TfToken gJointAxisAPINameTokensInstanced[5] =
{
    pxr::TfToken("PhysxJointAxisAPI:angular"),
    pxr::TfToken("PhysxJointAxisAPI:linear"),
    pxr::TfToken("PhysxJointAxisAPI:rotX"),
    pxr::TfToken("PhysxJointAxisAPI:rotY"),
    pxr::TfToken("PhysxJointAxisAPI:rotZ")
};
static const SchemaAPIFlag::Enum  gJointAxisAPIFlagList[] =
{
 SchemaAPIFlag::eJointAxisAngularAPI,
 SchemaAPIFlag::eJointAxisLinearAPI,
 SchemaAPIFlag::eJointAxisRotXAPI,
 SchemaAPIFlag::eJointAxisRotYAPI,
 SchemaAPIFlag::eJointAxisRotZAPI
};

bool checkForStructuralSchemaAPIChanges(AttachedStage& attachedStage, const pxr::SdfPath& path, const TfTokenVector& apiSchemas,
    uint32_t categories, SdfPath& resyncPath)
{
    // known physics schemas
    static const TfToken gPhysxForceAPIToken("PhysxForceAPI");
    static const TfToken gPhysicsAPIToken("PhysicsRigidBodyAPI");
    static const TfToken gCollisionAPIToken("PhysicsCollisionAPI");
    static const TfToken gParticleSetAPIToken("PhysxParticleSetAPI");
    static const TfToken gParticleClothAPITokenDeprecated("PhysxParticleClothAPI");
    static const TfToken gDeformableBodyAPITokenDeprecated("PhysxDeformableBodyAPI");
    static const TfToken gDeformableSurfaceAPITokenDeprecated("PhysxDeformableSurfaceAPI");
    static const TfToken gDeformableBodyAPIToken("OmniPhysicsDeformableBodyAPI");
    static const TfToken gSurfaceDeformableSimAPIToken("OmniPhysicsSurfaceDeformableSimAPI");
    static const TfToken gVolumeDeformableSimAPIToken("OmniPhysicsVolumeDeformableSimAPI");

    std::vector<ObjectCategory> appliedCategories;
    for (const TfToken& apiSchema : apiSchemas)
    {
        if (apiSchema == gPhysicsAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eRigidBodyAPI))
            {
                // physicsAPI found but not body created, recreate
                return true;
            }
            appliedCategories.push_back(eBody);
        }
        else if (apiSchema == gCollisionAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eCollisionAPI))
            {
                // collisionAPI found but not collision created, recreate
                return true;
            }
            appliedCategories.push_back(eShape);
        }
        if (apiSchema == gPhysxForceAPIToken)
        {
            if (!(categories & SchemaAPIFlag::ePhysxForceAPI))
            {
                // physicsAPI found but not body created, recreate
                return true;
            }
            appliedCategories.push_back(ePhysxForce);
        }
        // DEPRECATED
        else if (apiSchema == gParticleClothAPITokenDeprecated)
        {
            if (!(categories & SchemaAPIFlag::eParticleClothAPIdeprecated))
            {
                return true;
            }
            appliedCategories.push_back(eParticleCloth);
        }
        else if (apiSchema == gParticleSetAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eParticleSetAPI))
            {
                return true;
            }
            appliedCategories.push_back(eParticleSet);
        }
        // DEPRECATED
        else if (apiSchema == gDeformableBodyAPITokenDeprecated)
        {
            if (!(categories & SchemaAPIFlag::eDeformableBodyAPIdeprecated))
            {
                return true;
            }
            appliedCategories.push_back(eSoftBody);
        }
        // DEPRECATED
        else if (apiSchema == gDeformableSurfaceAPITokenDeprecated)
        {
            if (!(categories & SchemaAPIFlag::eDeformableSurfaceAPIdeprecated))
            {
                return true;
            }
            appliedCategories.push_back(eFEMCloth);
        }
        else if (apiSchema == gDeformableBodyAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eDeformableBodyAPI))
            {
                return true;
            }
            appliedCategories.push_back(eDeformableBody);
        }
        else if (apiSchema == gSurfaceDeformableSimAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eSurfaceDeformableSimAPI))
            {
                return true;
            }
            appliedCategories.push_back(eSurfaceDeformableBody);
        }
        else if (apiSchema == gVolumeDeformableSimAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eVolumeDeformableSimAPI))
            {
                return true;
            }
            appliedCategories.push_back(eVolumeDeformableBody);
        }
        else
        {
            // adding the mimic joint schema API should be treated as structural change

            for (uint32_t i = 0; i < gMimicJointAPITokenCount; i++)
            {
                const TfToken token = gMimicJointAPITokenList[i];

                if (apiSchema == token)
                {
                    const SchemaAPIFlag::Enum schemaAPIFlag = gMimicJointSchemaAPIFlagList[i];

                    if (!(categories & schemaAPIFlag))
                    {
                        return true;
                    }
                    // appliedCategories.push_back();
                    // Removing the mimic joint API is not treated as structural change for now, thus appliedCategories
                    // is not used here.
                }
            }
        }
    }

    if (categories & SchemaAPIFlag::eRigidBodyAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), eBody) == appliedCategories.end())
            return true;
    }

    if (categories & SchemaAPIFlag::eCollisionAPI)
    {
        // collisionAPI not found but collision created, recreate
        if (std::find(appliedCategories.begin(), appliedCategories.end(), eShape) == appliedCategories.end())
            return true;
    }

    if (categories & SchemaAPIFlag::ePhysxForceAPI)
    {
        // forceAPI not found but collision created, recreate
        if (std::find(appliedCategories.begin(), appliedCategories.end(), ePhysxForce) == appliedCategories.end())
            return true;
    }

    // DEPRECATED
    if (categories & SchemaAPIFlag::eDeformableBodyAPIdeprecated)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), eSoftBody) == appliedCategories.end())
            return true;
    }

    // DEPRECATED
    if (categories & SchemaAPIFlag::eDeformableSurfaceAPIdeprecated)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), eFEMCloth) == appliedCategories.end())
            return true;
    }

    if (categories & SchemaAPIFlag::eDeformableBodyAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), eVolumeDeformableBody) == appliedCategories.end())
            return true;
    }

    if (categories & SchemaAPIFlag::eSurfaceDeformableSimAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), eSurfaceDeformableBody) == appliedCategories.end())
        {
            resyncPath = path.GetParentPath();
            return true;
        }
    }

    if (categories & SchemaAPIFlag::eVolumeDeformableSimAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), eVolumeDeformableBody) == appliedCategories.end())
        {
            resyncPath = path.GetParentPath();
            return true;
        }
    }

    if (categories & SchemaAPIFlag::eParticleSetAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), eParticleSet) == appliedCategories.end())
            return true;
    }

    // DEPRECATED
    if (categories & SchemaAPIFlag::eParticleClothAPIdeprecated)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), eParticleCloth) == appliedCategories.end())
            return true;
    }

    return false;
}

void processNonstructuralSchemaAPIChanges(AttachedStage& attachedStage, const pxr::SdfPath& path, const TfTokenVector& apiSchemas, uint32_t categories)
{
    static const TfToken gParticleSmoothingAPIToken("PhysxParticleSmoothingAPI");
    static const TfToken gParticleAnisotropyAPIToken("PhysxParticleAnisotropyAPI");
    static const TfToken gParticleIsosurfaceAPIToken("PhysxParticleIsosurfaceAPI");
    static const TfToken gDiffuseParticlesAPIToken("PhysxDiffuseParticlesAPI");
    static const TfToken gFilteredPairsAPIToken("PhysicsFilteredPairsAPI");
    static const TfToken gPhysxContactReportAPIToken("PhysxContactReportAPI");

    std::vector<SchemaAPIFlag::Enum> appliedCategories;

    // iterate over added APIs & create if not existing
    for (const TfToken& apiSchema : apiSchemas)
    {
        if (apiSchema == gParticleIsosurfaceAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eParticleIsosurfaceAPI))
            {
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eParticleIsosurfaceAPI, false);
            }
            appliedCategories.push_back(SchemaAPIFlag::eParticleIsosurfaceAPI);
        }
        else if (apiSchema == gParticleAnisotropyAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eParticleAnisotropyAPI))
            {
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eParticleAnisotropyAPI, false);
            }
            appliedCategories.push_back(SchemaAPIFlag::eParticleAnisotropyAPI);
        }
        else if (apiSchema == gParticleSmoothingAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eParticleSmoothingAPI))
            {
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eParticleSmoothingAPI, false);
            }
            appliedCategories.push_back(SchemaAPIFlag::eParticleSmoothingAPI);
        }
        else if (apiSchema == gDiffuseParticlesAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eDiffuseParticlesAPI))
            {
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eDiffuseParticlesAPI, false);
            }
            appliedCategories.push_back(SchemaAPIFlag::eDiffuseParticlesAPI);
        }
        else if (apiSchema == gFilteredPairsAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eFilteredPairsAPI))
            {
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eFilteredPairsAPI, false);
            }
            appliedCategories.push_back(SchemaAPIFlag::eFilteredPairsAPI);
        }
        else if (apiSchema == gPhysxContactReportAPIToken)
        {
            if (!(categories & SchemaAPIFlag::eContactReportAPI))
            {
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eContactReportAPI, false);
            }
            appliedCategories.push_back(SchemaAPIFlag::eContactReportAPI);
        }
        else
        {
            for (uint32_t i = 0; i < gMimicJointAPITokenCount; i++)
            {
                const TfToken token = gMimicJointAPITokenList[i];

                if (apiSchema == token)
                {
                    const SchemaAPIFlag::Enum schemaAPIFlag = gMimicJointSchemaAPIFlagList[i];

                    // note: adding is treated as structural change for now, thus no call to
                    // attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, schemaAPIFlag, false);

                    appliedCategories.push_back(schemaAPIFlag);
                }
            }

            const uint32_t nbPerfEnvTokens = sizeof(gDrivePerformanceEnvelopeAPINameTokensInstanced)/sizeof(gDrivePerformanceEnvelopeAPINameTokensInstanced[0]);
            for (uint32_t i = 0; i < nbPerfEnvTokens; i++)
            {
                const TfToken token = gDrivePerformanceEnvelopeAPINameTokensInstanced[i];

                if (apiSchema == token)
                {
                    const SchemaAPIFlag::Enum schemaAPIFlag = gDrivePerformanceEnvelopAPIFlagList[i];

                    if (!(categories & schemaAPIFlag))
                    {
                        attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, schemaAPIFlag, false);
                    }
                    appliedCategories.push_back(schemaAPIFlag);
                }
            }

            const uint32_t nbJointAxisTokens = sizeof(gJointAxisAPINameTokensInstanced)/sizeof(gJointAxisAPINameTokensInstanced[0]);
            for (uint32_t i = 0; i < nbJointAxisTokens; i++)
            {
                const TfToken token = gJointAxisAPINameTokensInstanced[i];

                if (apiSchema == token)
                {
                    const SchemaAPIFlag::Enum schemaAPIFlag = gJointAxisAPIFlagList[i];

                    if (!(categories & schemaAPIFlag))
                    {
                        attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, schemaAPIFlag, false);
                    }
                    appliedCategories.push_back(schemaAPIFlag);
                }
            }
        }
    }

    // check for removed APIs
    if (categories & SchemaAPIFlag::eParticleIsosurfaceAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), SchemaAPIFlag::eParticleIsosurfaceAPI) == appliedCategories.end())
            attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eParticleIsosurfaceAPI, true);
    }

    if (categories & SchemaAPIFlag::eParticleAnisotropyAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), SchemaAPIFlag::eParticleAnisotropyAPI) == appliedCategories.end())
            attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eParticleAnisotropyAPI, true);
    }

    if (categories & SchemaAPIFlag::eParticleSmoothingAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), SchemaAPIFlag::eParticleSmoothingAPI) == appliedCategories.end())
            attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eParticleSmoothingAPI, true);
    }

    if (categories & SchemaAPIFlag::eDiffuseParticlesAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), SchemaAPIFlag::eDiffuseParticlesAPI) == appliedCategories.end())
            attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eDiffuseParticlesAPI, true);
    }

    if (categories & SchemaAPIFlag::eFilteredPairsAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), SchemaAPIFlag::eFilteredPairsAPI) == appliedCategories.end())
            attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eFilteredPairsAPI, true);
    }

    if (categories & SchemaAPIFlag::eContactReportAPI)
    {
        if (std::find(appliedCategories.begin(), appliedCategories.end(), SchemaAPIFlag::eContactReportAPI) == appliedCategories.end())
            attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, SchemaAPIFlag::eContactReportAPI, true);
    }

    for (uint32_t i = 0; i < gMimicJointAPITokenCount; i++)
    {
        const SchemaAPIFlag::Enum schemaAPIFlag = gMimicJointSchemaAPIFlagList[i];

        if (categories & schemaAPIFlag)
        {
            if (std::find(appliedCategories.begin(), appliedCategories.end(), schemaAPIFlag) == appliedCategories.end())
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, schemaAPIFlag, true);
        }
    }

    const uint32_t nbPerfEnvTokens = sizeof(gDrivePerformanceEnvelopeAPINameTokensInstanced)/sizeof(gDrivePerformanceEnvelopeAPINameTokensInstanced[0]);
    for (uint32_t i = 0; i < nbPerfEnvTokens; i++)
    {
        const SchemaAPIFlag::Enum schemaAPIFlag = gDrivePerformanceEnvelopAPIFlagList[i];

        if (categories & schemaAPIFlag)
        {
            if (std::find(appliedCategories.begin(), appliedCategories.end(), schemaAPIFlag) == appliedCategories.end())
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, schemaAPIFlag, true);
        }
    }


    const uint32_t nbJointAxisTokens = sizeof(gJointAxisAPINameTokensInstanced)/sizeof(gJointAxisAPINameTokensInstanced[0]);
    for (uint32_t i = 0; i < nbJointAxisTokens; i++)
    {
        const SchemaAPIFlag::Enum schemaAPIFlag = gJointAxisAPIFlagList[i];

        if (categories & schemaAPIFlag)
        {
            if (std::find(appliedCategories.begin(), appliedCategories.end(), schemaAPIFlag) == appliedCategories.end())
                attachedStage.getPhysXPhysicsInterface()->changeSchemaAPI(attachedStage, path, schemaAPIFlag, true);
        }
    }

}

bool PrimChangeMap::getPropertyChange(const pxr::TfToken& token, PropertyChangeMap::const_iterator& iterator, PropertyChangeMap::const_iterator& outItEnd) const
{
    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    PropertyChangeMap::const_iterator itChange = m_propertyChanges.find(token);
    PropertyChangeMap::const_iterator itEnd = m_propertyChanges.end();
    if (itChange != itEnd)
    {
        iterator = itChange;
        outItEnd = itEnd;
        return true;
    }
    else
    {
        // look into stage-specific changemap if nothing found on persistent
        itChange = m_stageSpecificChanges.find(token);
        itEnd = m_stageSpecificChanges.end();
        if (itChange != itEnd)
        {
            iterator = itChange;
            outItEnd = itEnd;
            return true;
        }
    }

    return false;
}

void PrimChangeMap::checkPrimChange(AttachedStage& attachedStage, const pxr::SdfPath& primPath, const pxr::TfToken& propertyName, const pxr::UsdPrim* primIn)
{
    bool structuralChange = false;
    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    PropertyChangeMap::const_iterator itChange;
    PropertyChangeMap::const_iterator itChangeEnd;
    bool propertyChangeFound = getPropertyChange(propertyName, itChange, itChangeEnd);
    pxr::SdfPath resyncPrimPath;
    while (propertyChangeFound && itChange != itChangeEnd && itChange->first == propertyName)
    {
        const PropertyChange& change = itChange->second;
        if (change.onPrimCheck(attachedStage, primPath, propertyName, primIn) &&
            (change.onPrimCheckExt == nullptr || change.onPrimCheckExt(attachedStage, primPath, propertyName, primIn, resyncPrimPath)))
        {
            // check for structural change, onUpdate is not defined and resyncPrimPath not set 
            // we will recreate the objects
            if (!change.onUpdate || !resyncPrimPath.IsEmpty())
            {
                structuralChange = true;
                break;
            }

            if (!usdLoad->getAsyncUSDUpdate())
            {
                const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(primPath);
                if (entries && !entries->empty())
                {
                    ObjectIdMap::const_iterator it = entries->begin();
                    while (it != entries->end())
                    {
                        attachedStage.getPhysXPhysicsInterface()->updateObject(attachedStage, primPath, it->second, change.onUpdate, propertyName, pxr::UsdTimeCode::Default());
                        it++;
                    }
                }
                return;
            }
            else
            {
                bool attributeSet = false;
                ChangeMap::iterator it = m_changeMap.find(primPath);
                while (it != m_changeMap.end() && it->first == primPath)
                {
                    ChangeData& changeData = it->second;
                    if (changeData.second == propertyName)
                    {
                        attributeSet = true;
                        break;
                    }
                    it++;
                }

                if (!attributeSet)
                {
                    attributeSet = true;
                    ChangeMap::iterator it =
                        m_changeMap.insert(std::pair<const pxr::SdfPath, ChangeData>(primPath, { change.onUpdate, propertyName }));
                }
                return;
            }
        }

        itChange++;
    }

    UsdPrim prim;

    if (!structuralChange)
    {
        // handle transform change
        if (UsdGeomXformable::IsTransformationAffectedByAttrNamed(propertyName))
        {
            if (!usdLoad->getAsyncUSDUpdate())
            {
                addTransformChange(primPath, primIn, false);
            }
            else
                m_transformUpdates.push_back(primPath);
            return;
        }

        // If an API schema was added/remove, check whether this triggers a structural change
        if (propertyName == UsdTokens->apiSchemas)
        {
            if (prim == UsdPrim())
            {
                prim = primIn ? *primIn : attachedStage.getStage()->GetPrimAtPath(primPath);
            }
            const TfTokenVector apiSchemas = prim.GetAppliedSchemas();
            const uint64_t storedAPIs = attachedStage.getObjectDatabase()->getSchemaAPIs(primPath);
            structuralChange = checkForStructuralSchemaAPIChanges(attachedStage, primPath, apiSchemas, storedAPIs, resyncPrimPath);
            if (!structuralChange) // handle API changes that don't trigger structural changes here
                processNonstructuralSchemaAPIChanges(attachedStage, primPath, apiSchemas, storedAPIs);
        }
        else if (propertyName == UsdGeomTokens->inactiveIds)
        {
            structuralChange = true;
        }
    }

    // check if we need to reconstruct the prim
    if (structuralChange)
    {
        UsdPrim resyncPrim;
        if (resyncPrimPath.IsEmpty())
        {
            resyncPrim = primIn ? *primIn : attachedStage.getStage()->GetPrimAtPath(primPath);
        }
        else
        {
            resyncPrim = attachedStage.getStage()->GetPrimAtPath(resyncPrimPath);
        }
        PrimHierarchyStorage& primStorage = attachedStage.getObjectDatabase()->getPrimHierarchyStorage();
        PrimHierarchyStorage::Iterator iterator(primStorage, resyncPrim.GetPath());
        for (size_t i = iterator.getDescendentsPaths().size(); i--;)
        {
            const pxr::SdfPath& primCPath = iterator.getDescendentsPaths()[i];
            handleRemovedPrim(attachedStage, primCPath);
        }
        primStorage.removeIteration(iterator);

        attachedStage.getPrimUpdateMap().addPrim(attachedStage, resyncPrim);
    }
}

void PrimChangeMap::clearMap()
{
    m_changeMap.clear();
}

void PrimChangeMap::removePrim(const pxr::SdfPath& primPath)
{
    ChangeMap::iterator itCh = m_changeMap.find(primPath);
    if (itCh != m_changeMap.end())
        m_changeMap.erase(itCh);
}

void PrimChangeMap::processTransformUpdates(AttachedStage& attachedStage)
{
    if (!m_transformUpdates.empty())
    {
        pxr::UsdGeomXformCache xfCache;
        for (size_t i = 0; i < m_transformUpdates.size(); i++)
        {
            const UsdPrim prim = attachedStage.getStage()->GetPrimAtPath(m_transformUpdates[i]);
            if (prim)
                handleTransformChange(attachedStage, prim.GetPrimPath(), &prim, &xfCache);
        }

        m_transformUpdates.clear();
    }
}

void PrimChangeMap::processTransformChanges(AttachedStage& attachedStage, bool fabric)
{
    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    if (fabric)
    {
        for (PrimSet::const_reference ref : m_fabricTransformChangesSet)
        {
            handleTransformChange(attachedStage, ref.first, ref.second, nullptr, fabric);
        }
        m_fabricTransformChangesSet.clear();
    }
    else
    {
        pxr::UsdGeomXformCache xfCache;
        for (PrimSet::const_reference ref : m_usdTransformChangesSet)
        {
            handleTransformChange(attachedStage, ref.first, ref.second, &xfCache, fabric);
        }
        m_usdTransformChangesSet.clear();
    }
}

bool PrimUpdateMap::needsSceneReset(const pxr::UsdPrim& usdPrim)
{
    if (usdPrim.IsA<UsdPhysicsScene>())
        return true;

    return false;
}

void PrimUpdateMap::addPrim(const AttachedStage& attachedStage, const pxr::UsdPrim& prim)
{
    if (m_isNewScene)
        return;

    // if a parent is in the map, we dont need to add the prim
    if (!isInPrimAddMap(prim))
    {
        // before we add we need to check if some child is not already in the map
        UsdPrimRange range(prim);
        for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
        {
            const pxr::UsdPrim& childPrim = *iter;
            if (!childPrim)
                continue;
            UsdPrimMap::iterator fit = m_primAddMap.find(childPrim.GetPrimPath());
            if (fit != m_primAddMap.end())
            {
                m_primAddMap.erase(fit);
            }
        }
        m_primAddMap[prim.GetPrimPath()] = prim;
        if (needsSceneReset(prim))
            UsdLoad::getUsdLoad()->releasePhysicsObjects(attachedStage.getStageId());
    }
}

void PrimUpdateMap::removePrim(AttachedStage& attachedStage, const pxr::SdfPath& primPath)
{
    m_primAddMap.erase(primPath);

    PrimHierarchyStorage& primStorage = attachedStage.getObjectDatabase()->getPrimHierarchyStorage();
    PrimHierarchyStorage::Iterator iterator(primStorage, primPath);
    for (size_t i = iterator.getDescendentsPaths().size(); i--;)
    {
        const pxr::SdfPath& primCPath = iterator.getDescendentsPaths()[i];
        handleRemovedPrim(attachedStage, primCPath);
    }
    primStorage.removeIteration(iterator);
}

bool PrimUpdateMap::isInPrimAddMap(const pxr::UsdPrim& prim) const
{
    if (m_isNewScene)
        return true;

    if (m_primAddMap.empty())
        return false;

    UsdPrim parent = prim;
    const UsdPrim root = prim.GetStage()->GetPseudoRoot();
    while (parent != root)
    {
        UsdPrimMap::const_iterator it = m_primAddMap.find(parent.GetPath());
        if (it != m_primAddMap.end())
        {
            return true;
        }

        parent = parent.GetParent();
    }

    return false;
}

// remove invalid prims
void PrimUpdateMap::checkMap(const pxr::UsdStageWeakPtr stage)
{
    UsdPrimMap::iterator it = m_primAddMap.begin();
    const UsdPrimMap::iterator itEnd = m_primAddMap.end();
    while (it != itEnd)
    {
        if (!it->second || !stage->GetPrimAtPath(it->first))
        {
            it = m_primAddMap.erase(it);
        }
        else
            it++;
    }
}

void processChangeMap(AttachedStage& attachedStage)
{
    auto changeSourceBlock = attachedStage.getChangeSourceBlock(ChangeSource::eUsd); // ChangeMap is populated by USD change notice
    ChangeMap::const_iterator itCh = attachedStage.getPrimChangeMap().getMap().begin();
    ChangeMap::const_iterator itChEnd = attachedStage.getPrimChangeMap().getMap().end();
    while (itCh != itChEnd)
    {
        const SdfPath& primPath = itCh->first;
        const ChangeData& changeData = itCh->second;

        const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(primPath);
        if (entries && !entries->empty())
        {
            auto it = entries->begin();
            while (it != entries->end())
            {
                attachedStage.getPhysXPhysicsInterface()->updateObject(attachedStage, primPath, it->second, changeData.first, changeData.second, pxr::UsdTimeCode::Default());
                it++;
            }
        }
        itCh++;
    }

    attachedStage.getPrimChangeMap().processTransformUpdates(attachedStage);
    attachedStage.getPrimChangeMap().clearMap();
}

void checkChange(AttachedStage& attachedStage, const PropertyChange& change, const SdfPath& primPath, const TfToken& attributeName)
{
    if (change.onPrimCheck(attachedStage, primPath, attributeName, nullptr))
    {
        // check for structural change, onUpdate is not defined
        // we will recreate the objects
        if (change.onUpdate)
        {
            const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(primPath);
            if (entries && !entries->empty())
            {
                ObjectIdMap::const_iterator it = entries->begin();
                while (it != entries->end())
                {
                    attachedStage.getPhysXPhysicsInterface()->updateObject(attachedStage, primPath, it->second, change.onUpdate, attributeName, pxr::UsdTimeCode::Default());
                    it++;
                }
            }
        }
    }
}

void transformationChange(AttachedStage& attachedStage, const gsl::span<const pxr::GfMatrix4d>& matrices, size_t index,  const pxr::SdfPath& primPath)
{
    PhysXUsdPhysicsInterface::Transform fcTransform;
    bool scaleProvided = false;
    if (index < matrices.size())
    {
        const GfTransform tr(matrices[index]);
        const GfVec3d pos = tr.GetTranslation();
        const GfQuatd rot = tr.GetRotation().GetQuat();
        const GfVec3d sc = tr.GetScale();

        fcTransform.position = { float(pos[0]), float(pos[1]), float(pos[2]) };
        fcTransform.orientation = { float(rot.GetImaginary()[0]), float(rot.GetImaginary()[1]),
                                    float(rot.GetImaginary()[2]), float(rot.GetReal()) };
        fcTransform.scale = { float(sc[0]), float(sc[1]), float(sc[2]) };
    }
    else
    {
        fcTransform.position = { 0.0f, 0.0f, 0.0f };
        CARB_LOG_ERROR("Physics update: Transformation change without position array - %s", primPath.GetText());
    }

    const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(primPath);
    if (entries && !entries->empty())
    {
        auto it = entries->begin();
        while (it != entries->end())
        {
            attachedStage.getPhysXPhysicsInterface()->updateTransform(attachedStage,
                primPath, it->second, fcTransform, true, scaleProvided);
            it++;
        }
    }
}

void fabricChangeTracking(AttachedStage& attachedStage, float currentTime)
{
    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();

    if (!omni::fabric::Token::iToken)
        omni::fabric::Token::iToken = carb::getCachedInterface<omni::fabric::IToken>();
    if (!omni::fabric::Path::iPath)
        omni::fabric::Path::iPath = carb::getCachedInterface<omni::fabric::IPath>();

    const omni::fabric::UsdStageId stageId = { uint64_t(attachedStage.getStageId()) };
    omni::fabric::StageReaderWriter stageInProgress = OmniPhysX::getInstance().getIStageReaderWriter()->get(stageId);
    if (stageInProgress.getId().id != 0 && attachedStage.getFabricListenerId().id != kInvalidFabricListenerId)
    {
        if (currentTime == -1.0f)
        {
            // with -1.0 we are just flushing changes no time code change should be requested
            flushUsdToFabric(stageId, false, 0.0);
        }
        else
        {
            flushUsdToFabric(stageId, true, currentTime * attachedStage.getStage()->GetTimeCodesPerSecond());
        }

        omni::fabric::ChangedPrimBucketList changesByType = stageInProgress.getChanges(attachedStage.getFabricListenerId());
        auto changeSourceBlock = attachedStage.getChangeSourceBlock(ChangeSource::eFabric);

        for (size_t i = 0; i != changesByType.size(); i++)
        {
            bool transformationUpdated = false;
            bool particlePositionsUpdated = false;
            bool particleVelocitiesUpdated = false;
            const omni::fabric::BucketChanges changes = changesByType.getChanges(i);
            for (size_t j = 0; j < changes.attrChangedIndices.size(); j++)
            {
                const omni::fabric::AttrAndChangedIndices changeType = changes.attrChangedIndices[j];
                const gsl::span<const omni::fabric::Path> paths = changes.pathArray;
                const TfToken attributeName = omni::fabric::toTfToken(changeType.attr.name);

                FabricBatchData fabricData(stageInProgress, changesByType, i, changeType.attr.name);
                omniPhysX.setFabricBatchData(&fabricData);

                PropertyChangeMap::const_iterator itChange;
                PropertyChangeMap::const_iterator itChangeEnd;
                const bool changeFound = attachedStage.getPrimChangeMap().getPropertyChange(attributeName, itChange, itChangeEnd);

                while (changeFound && itChange != itChangeEnd && itChange->first == attributeName)
                {
                    const PropertyChange& change = itChange->second;

                    if (changeType.allIndicesChanged)
                    {
                        uint32_t index = 0;
                        for (const omni::fabric::Path& path : paths)
                        {
                            fabricData.setCurrentIndex(index);
                            index++;
                            const pxr::SdfPath primPath = omni::fabric::toSdfPath(path);

                            checkChange(attachedStage, change, primPath, attributeName);
                        }
                    }
                    else
                    {
                        for (const size_t index : changeType.changedIndices)
                        {
                            fabricData.setCurrentIndex((uint32_t)index);
                            const pxr::SdfPath primPath = omni::fabric::toSdfPath(paths[index]);

                            checkChange(attachedStage, change, primPath, attributeName);
                        }
                    }
                    itChange++;
                }

                if (!changeFound)
                {
                    if ((attributeName == fabricLocalMatrix ) && !transformationUpdated)
                    {
                        // update transformation
                        transformationUpdated = true;

                        gsl::span<const pxr::GfMatrix4d> matrices = stageInProgress.getAttributeArrayRd<pxr::GfMatrix4d>(changesByType, i, *usdLoad->getFabricTokens().worldMatrix);

                        if (changeType.allIndicesChanged)
                        {
                            size_t index = 0;
                            for (const omni::fabric::Path& path : paths)
                            {
                                const pxr::SdfPath primPath = omni::fabric::toSdfPath(path);
                                transformationChange(attachedStage, matrices, index, primPath);
                                index++;
                            }
                        }
                        else
                        {
                            for (const size_t index : changeType.changedIndices)
                            {
                                const pxr::SdfPath primPath = omni::fabric::toSdfPath(paths[index]);
                                transformationChange(attachedStage, matrices, index, primPath);
                            }
                        }
                    }

                    if (attributeName == fabricPositionInvMasses && !particlePositionsUpdated)
                    {
                        particlePositionsUpdated = true;
                        omniPhysX.getFabricParticles()->updateParticles(changesByType, i, changeType.attr.name);
                    }

                    if (attributeName == fabricVelocitiesFloat4 && !particleVelocitiesUpdated)
                    {
                        particleVelocitiesUpdated = true;
                        omniPhysX.getFabricParticles()->updateParticles(changesByType, i, changeType.attr.name);
                    }
                }

                omniPhysX.setFabricBatchData(nullptr);
            }
        }

        stageInProgress.popChanges(attachedStage.getFabricListenerId());
    }
}

void flushBufferedChanges(AttachedStage& attachedStage, float currentTime)
{
    UsdStageWeakPtr stage = attachedStage.getStage();
    if (stage == nullptr)
        return;

    UsdLoad* usdLoad = UsdLoad::getUsdLoad();

    if (attachedStage.getPrimUpdateMap().isEmptyScene())
    {
        attachedStage.getPrimUpdateMap().clearMap();
        attachedStage.getPrimChangeMap().clearMap();

        if (attachedStage.isReplicatorStage())
        {
            OmniPhysX::getInstance()
                .getReplicator(attachedStage.getStageId())
                ->attach(attachedStage.getStageId(), attachedStage.getPhysXPhysicsInterface(), false);
        }
        else
        {
            loadFromStage(attachedStage);
        }
        attachedStage.getPrimUpdateMap().setEmptyScene(false);
    }

    {
        CARB_PROFILE_ZONE(0, "Fabric updates");

        if (OmniPhysX::getInstance().getIStageReaderWriter())
        {
            fabricChangeTracking(attachedStage, currentTime);
        }
    }

    // don't allow updates while processing the current batch
    // that would modify g_PrimUpdateMap while we are iterating over the contents
    if (!attachedStage.getPrimUpdateMap().getMap().empty())
    {
        UsdLoad::getUsdLoad()->blockUSDUpdate(true);

        attachedStage.getPrimUpdateMap().checkMap(stage);
        PrimIteratorMapRange primUpdateIterator(attachedStage.getPrimUpdateMap().getMap());
        loadPhysicsFromPrimitive(attachedStage, primUpdateIterator);

        // it is safe to allow updates again
        UsdLoad::getUsdLoad()->blockUSDUpdate(false);

        attachedStage.getPrimUpdateMap().clearMap();
    }

    processChangeMap(attachedStage);
}

void processUpdates(AttachedStage& attachedStage, float currentTime)
{
    UsdStageWeakPtr stage = attachedStage.getStage();
    if (stage == nullptr)
        return;

    TRACE_FUNCTION();
    if (!attachedStage.getAnimatedKinematicDeformableBodies().empty() || !attachedStage.getTimeSampleMap().empty())
    {
        pxr::UsdTimeCode timeCode(currentTime * stage->GetTimeCodesPerSecond());

        static UsdGeomXformCache xfCache;
        xfCache.SetTime(timeCode);

        // DEPRECATED
        {
            CARB_PROFILE_ZONE(0, "KinematicSoftBodiesUpdate");
            for (PathPrimMap::const_reference& prims : attachedStage.getAnimatedKinematicDeformableBodies())
            {
                if (!prims.second)
                    continue;

                const UsdPrim usdPrim = prims.second;
                if (usdPrim.IsA<UsdGeomMesh>())
                {
                    const UsdGeomMesh usdMesh(usdPrim);
                    const ObjectId sceneId = attachedStage.getObjectDatabase()->findEntry(usdPrim.GetPrimPath(), eScene);
                    PhysXScene* defaultScene = OmniPhysX::getInstance().getPhysXSetup().getPhysXScene(sceneId);

                    pxr::VtArray<pxr::GfVec3f> pointsValue;
                    usdMesh.GetPointsAttr().Get(&pointsValue, timeCode);

                    const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(usdPrim.GetPrimPath());
                    if (entries && !entries->empty())
                    {
                        auto it = entries->begin();
                        while (it != entries->end())
                        {
                            pxr::GfMatrix4d localToWorld = xfCache.GetLocalToWorldTransform(usdPrim);

                            attachedStage.getPhysXPhysicsInterface()->updateKinematicVertexTargetsFromSkinDeprecated(
                                usdPrim.GetPrimPath(), it->second, reinterpret_cast<const carb::Float3*>(pointsValue.cdata()),
                                pointsValue.size(), localToWorld);

                            it++;
                        }
                    }
                }
            }
        }
    }

    if (!attachedStage.getAnimatedKinematicBodies().empty() || !attachedStage.getTimeSampleMap().empty())
    {
        pxr::UsdTimeCode timeCode(currentTime * stage->GetTimeCodesPerSecond());

        static UsdGeomXformCache xfCache;
        xfCache.SetTime(timeCode);

        {
            CARB_PROFILE_ZONE(0, "KinematicBodiesUpdate");
            for (PathPrimMap::const_reference& prims : attachedStage.getAnimatedKinematicBodies())
            {
                if (!prims.second)
                    continue;

                const UsdPrim prim = prims.second;

                pxr::GfMatrix4d localToWorld = xfCache.GetLocalToWorldTransform(prim);

                const GfTransform tr(localToWorld);
                const GfVec3d pos = tr.GetTranslation();
                const GfQuatd rot = tr.GetRotation().GetQuat();
                const GfVec3d sc = tr.GetScale();

                PhysXUsdPhysicsInterface::Transform fcTransform;

                fcTransform.position = { float(pos[0]), float(pos[1]), float(pos[2]) };
                fcTransform.orientation = { float(rot.GetImaginary()[0]), float(rot.GetImaginary()[1]),
                                            float(rot.GetImaginary()[2]), float(rot.GetReal()) };
                fcTransform.scale = { float(sc[0]), float(sc[1]), float(sc[2]) };


                const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(prim.GetPrimPath());
                if (entries && !entries->empty())
                {
                    auto it = entries->begin();
                    while (it != entries->end())
                    {
                        attachedStage.getPhysXPhysicsInterface()->updateTransform(attachedStage,
                            prim.GetPrimPath(), it->second, fcTransform);
                        it++;
                    }
                }
            }
        }

        {
            CARB_PROFILE_ZONE(0, "KinematicAttributesUpdate");
            for (auto& iterator : attachedStage.getTimeSampleMap())
            {
                const SdfPath primPath = iterator.first.GetPrimPath();
                const TfToken& attrToken = iterator.first.GetNameToken();
                const UsdPrim prim = stage->GetPrimAtPath(primPath);

                const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(primPath);
                if (entries && !entries->empty())
                {
                    auto it = entries->begin();
                    while (it != entries->end())
                    {
                        attachedStage.getPhysXPhysicsInterface()->updateObject(attachedStage, primPath, it->second, iterator.second, attrToken, timeCode);
                        it++;
                    }
                }
            }
        }

        xfCache.Clear();
    }

    flushBufferedChanges(attachedStage, currentTime);
}

void UsdNoticeListener::Handle(const class pxr::UsdNotice::ObjectsChanged& objectsChanged)
{
    TRACE_FUNCTION();

    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    if (usdLoad->usdUpdateIsBlocked())
    {
        return;
    }

    // This is an old callback, ignore it
    const uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(objectsChanged.GetStage()).ToLongInt();
    AttachedStage* attachedStage = usdLoad->getAttachedStage(stageId);
    if (!attachedStage)
    {
        return;
    }
    CARB_ASSERT(attachedStage->getStage() == objectsChanged.GetStage());

    // Early exit if scene is empty we will parse the whole scene
    if (attachedStage->getPrimUpdateMap().isEmptyScene())
    {
        return;
    }

    CARB_PROFILE_ZONE(0, "PhysicsUsdNoticleListener");
    TRACE_FUNCTION();

    UsdStageWeakPtr stage = attachedStage->getStage();
    auto changeSourceBlock = attachedStage->getChangeSourceBlock(ChangeSource::eUsd);

    for (const SdfPath& path : objectsChanged.GetResyncedPaths())
    {
        if (path.IsAbsoluteRootOrPrimPath())
        {
            const SdfPath primPath = stage->GetPseudoRoot().GetPath() == path ?
                stage->GetPseudoRoot().GetPath() :
                                         path.GetPrimPath();

            // If prim is removed, remove it and its descendants from selection.
            UsdPrim prim = stage->GetPrimAtPath(primPath);

            if (prim.IsValid() == false || !prim.IsActive()) // remove prim
            {
                attachedStage->getPrimUpdateMap().removePrim(*attachedStage, primPath);
            }
            else // resync prim
            {
                static const TfToken gkindToken("kind");
                const auto &changedFields = objectsChanged.GetChangedFields(primPath);
                const bool typeNameChange = std::find(changedFields.begin(), changedFields.end(), SdfFieldKeys->TypeName) != changedFields.end();
                if (!typeNameChange && std::find(changedFields.begin(), changedFields.end(), PXR_NS::UsdTokens->apiSchemas) != changedFields.end())
                {
                    if (!attachedStage->getPrimUpdateMap().isInPrimAddMap(prim))
                    {
                        attachedStage->getPrimChangeMap().checkPrimChange(*attachedStage, primPath, PXR_NS::UsdTokens->apiSchemas, &prim);
                    }
                }
                else if (!typeNameChange && std::find(changedFields.begin(), changedFields.end(), gkindToken) != changedFields.end())
                {
                    // We ignore 'kind' changes
                    continue;
                }
                else
                {
                    if (path == stage->GetPseudoRoot().GetPath())
                    {
                        usdLoad->releasePhysicsObjects(stageId);
                    }
                    else
                    {
                        if (!prim.IsPrototype())
                        {
                            attachedStage->getPrimUpdateMap().removePrim(*attachedStage, primPath);
                            attachedStage->getPrimUpdateMap().addPrim(*attachedStage, prim);
                        }
                    }
                }
            }
        }
        else if (path.IsPropertyPath())
        {
            const SdfPath primPath = path.GetParentPath();

            const TfToken& attrToken = path.GetNameToken();
            attachedStage->getPrimChangeMap().checkPrimChange(*attachedStage, primPath, attrToken);
        }
    }

    for (const SdfPath& path : objectsChanged.GetChangedInfoOnlyPaths())
    {
        const SdfPath primPath = stage->GetPseudoRoot().GetPath() == path ? path : path.GetPrimPath();

        const bool isAttributePath = path.IsPropertyPath();
        if (objectsChanged.HasChangedFields(primPath))
        {
            const pxr::TfTokenVector changedFieldTokens = objectsChanged.GetChangedFields(primPath);
            for (const TfToken& f : changedFieldTokens)
            {
                attachedStage->getPrimChangeMap().checkPrimChange(*attachedStage, primPath, f);
            }
        }

        if (isAttributePath)
        {
            TfToken const& attrName = path.GetNameToken();
            attachedStage->getPrimChangeMap().checkPrimChange(*attachedStage, primPath, attrName);
        }
    }

    attachedStage->getPrimChangeMap().processTransformChanges(*attachedStage, false);
}


void UsdNoticeListener::HandleAttributeValuesChanged(const class omni::fabric::AttributeValuesChangedNotice& valuesChanged)
{
    TRACE_FUNCTION();

    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    if (usdLoad->usdUpdateIsBlocked())
    {
        return;
    }

    // A.B. Fabric notice handling does not really support multiple stages?
    AttachedStage* attachedStage = usdLoad->getAttachedStage(0);
    if (!attachedStage)
    {
        return;
    }

    // Early exit if scene is empty
    if (attachedStage->getPrimUpdateMap().isEmptyScene())
    {
        return;
    }

    CARB_PROFILE_ZONE(0, "PhysicsOmniAttributeValueCacheNoticeListenerFabric");
    TRACE_FUNCTION();

    if (valuesChanged.GetPrimPaths().size() != valuesChanged.GetAttributeNames().size())
    {
        return; // should warn
    }

    // Make sure we call flush to also update all the transforms.
    const omni::fabric::UsdStageId stageId = { uint64_t(attachedStage->getStageId()) };
    omni::fabric::StageReaderWriter stageInProgress = OmniPhysX::getInstance().getIStageReaderWriter()->get(stageId);
    flushUsdToFabric(stageId, false);

    std::unordered_map<SdfPath, UsdPrim, SdfPath::Hash> transformationUpdatePrims;

    for (size_t i = 0; i < valuesChanged.GetPrimPaths().size(); ++i)
    {
        const pxr::SdfPath& path = valuesChanged.GetPrimPaths()[i];
        const pxr::TfToken& name = valuesChanged.GetAttributeNames()[i];

        if (name == fabricLocalMatrix)
        {
            // buffer up the change so that we dont update each prim several times
            attachedStage->getPrimChangeMap().addTransformChange(path, nullptr, true);
            continue;
        }

        // Tell physics to pull on this attribute next time
        attachedStage->getPrimChangeMap().checkPrimChange(*attachedStage, path, name);
    }

    attachedStage->getPrimChangeMap().processTransformChanges(*attachedStage, true);
}

} // namespace usdparser
} // namespace physx
} // namespace omni
