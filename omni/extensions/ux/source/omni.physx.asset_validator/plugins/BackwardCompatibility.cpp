// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/settings/ISettings.h>
#include <carb/Framework.h>
#include <usdrt/scenegraph/usd/usd/stage.h>
#include <omni/physx/IPhysxSettings.h>

#include <pxr/usd/usd/primCompositionQuery.h>
#include <pxr/usd/usd/editContext.h>
#include <pxr/usd/pcp/layerStack.h>

#include <omni/fabric/usd/PathConversion.h>
#include <private/omni/physx/PhysxUsd.h>

#include "BackwardCompatibility.h"

using namespace pxr;
using namespace carb;

namespace omni
{
namespace physx
{

static const char* gFrictionType = "frictionType";
static const char* gPhysxSceneFrictionType = "physxScene:frictionType";
static const TfToken gPhysxSceneFrictionTypeToken(gPhysxSceneFrictionType);

static const char* gEnableProjection = "enableProjection";
static const char* gPhysxJointEnableProjection = "physxJoint:enableProjection";
static const TfToken gPhysxJointEnableProjectionToken(gPhysxJointEnableProjection);

static std::string gCheckLog;

inline float degToRad(const float a)
{
    return 0.01745329251994329547f * a;
}

inline float radToDeg(const float a)
{
    return 57.29577951308232286465f * a;
}

inline pxr::GfVec3f degToRad(const pxr::GfVec3f& a)
{
    return pxr::GfVec3f(0.01745329251994329547f * a);
}

inline pxr::GfVec3f radToDeg(const pxr::GfVec3f& a)
{
    return pxr::GfVec3f(57.29577951308232286465f * a);
}

bool removeAttributeFromReference(pxr::UsdStageWeakPtr stage, const TfToken& atrrToken, UsdPrim& usdPrim)
{
    UsdPrimCompositionQuery query = UsdPrimCompositionQuery::GetDirectReferences(usdPrim);
    UsdPrimCompositionQuery::Filter filter;
    filter.arcTypeFilter = UsdPrimCompositionQuery::ArcTypeFilter::Reference;
    query.SetFilter(filter);
    std::vector<UsdPrimCompositionQueryArc> arcs = query.GetCompositionArcs();
    bool changeWritten = false;

    for (size_t i = 0; i < arcs.size(); i++)
    {
        const UsdPrimCompositionQueryArc& currentArc = arcs[i];
        PcpNodeRef nodeRef = currentArc.GetTargetNode();
        PcpLayerStackRefPtr pcpLayerStack = nodeRef.GetLayerStack();
        SdfLayerRefPtrVector layerStack = pcpLayerStack->GetLayers();
        if (!layerStack.empty())
        {
            for (size_t j = 0; j < layerStack.size(); j++)
            {                
                PXR_NS::UsdEditTarget editTarget(layerStack[j], nodeRef);
                pxr::UsdEditContext editContext(stage, editTarget);
                const bool retValLocal = usdPrim.RemoveProperty(atrrToken);
                if (retValLocal)
                {
                    changeWritten = true;
                    break;
                }
            }
        }

        if (changeWritten)
            break;
    }

    if (!changeWritten)
    {
        CARB_LOG_ERROR("Physics backwardsCompatibility: failed to remove token from local reference stack (%s) on a prim (%s).", atrrToken.GetText(), usdPrim.GetPath().GetText());
    }

    return changeWritten;
}

void setTypeName(pxr::UsdStageWeakPtr stage, const TfToken& newTypeName, UsdPrim& usdPrim)
{
    const bool retVal = usdPrim.SetTypeName(newTypeName);
    if (!retVal)
    {
        UsdPrimCompositionQuery query = UsdPrimCompositionQuery::GetDirectReferences(usdPrim);
        UsdPrimCompositionQuery::Filter filter;
        filter.arcTypeFilter = UsdPrimCompositionQuery::ArcTypeFilter::Reference;
        query.SetFilter(filter);
        std::vector<UsdPrimCompositionQueryArc> arcs = query.GetCompositionArcs();
        bool changeWritten = false;

        for (size_t i = 0; i < arcs.size(); i++)
        {
            const UsdPrimCompositionQueryArc& currentArc = arcs[i];
            PcpNodeRef nodeRef = currentArc.GetTargetNode();
            PcpLayerStackRefPtr pcpLayerStack = nodeRef.GetLayerStack();
            SdfLayerRefPtrVector layerStack = pcpLayerStack->GetLayers();
            if (!layerStack.empty())
            {                             
                PXR_NS::UsdEditTarget editTarget(layerStack[0], nodeRef);
                pxr::UsdEditContext editContext(stage, editTarget);
                const bool retValLocal = usdPrim.SetTypeName(newTypeName);                
                if (retValLocal)
                {
                    changeWritten = true;
                    break;
                }
            }
        }

        if (!changeWritten)
        {
            CARB_LOG_ERROR("Physics backwardsCompatibility: failed to change prim typename token from local reference stack (%s) on a prim (%s).", newTypeName.GetText(), usdPrim.GetPath().GetText());
        }
    }
}

template<typename T>
bool switchDoubleToFloat(pxr::UsdStageWeakPtr stage, const TfToken& attrToken, UsdPrim& usdPrim, T& val)
{
    UsdAttribute attribute = usdPrim.GetAttribute(attrToken);
    if (attribute)
    {
        VtValue value;
        if (attribute.Get(&value))
        {            
            if (value.IsHolding<T>())
            {
                attribute.Get(&val);                
                const bool retVal = usdPrim.RemoveProperty(attrToken);
                if (!retVal)
                {
                    removeAttributeFromReference(stage, attrToken, usdPrim);
                }
                return true;
            }
        }
    }
    return false;
}

template<typename T>
bool switchRadToDegree(pxr::UsdStageWeakPtr stage, const TfToken& attrToken, UsdPrim& usdPrim)
{
    UsdAttribute attribute = usdPrim.GetAttribute(attrToken);
    if (attribute)
    {
        VtValue value;
        if (attribute.Get(&value))
        {
            if (value.IsHolding<T>())
            {
                T newVal = radToDeg(value.UncheckedGet<T>());
                attribute.Set(newVal);
            }            
        }
    }
    return false;
}

template<typename T>
bool switchDegreeToRad(pxr::UsdStageWeakPtr stage, const TfToken& attrToken, UsdPrim& usdPrim)
{
    UsdAttribute attribute = usdPrim.GetAttribute(attrToken);
    if (attribute)
    {
        VtValue value;
        if (attribute.Get(&value))
        {
            if (value.IsHolding<T>())
            {
                T newVal = degToRad(value.UncheckedGet<T>());
                attribute.Set(newVal);
            }
        }
    }
    return false;
}

void modifyFloatAttribute(pxr::UsdStageWeakPtr stage, const TfToken& attrToken, UsdPrim& usdPrim, float modifyVal)
{
    UsdAttribute attribute = usdPrim.GetAttribute(attrToken);
    if (attribute)
    {
        float value;
        if (attribute.HasAuthoredValue() && attribute.Get(&value))
        {
            attribute.Set(value * modifyVal);
        }
    }    
}

bool renameProperty(pxr::UsdStageWeakPtr stage, UsdPrim& prim, const char* oldName, const char* newName)
{    
    TfToken oldNameToken(oldName);
    UsdAttribute attr = prim.GetAttribute(oldNameToken);
    if (attr)
    {
        SdfValueTypeName typeName = attr.GetTypeName();
        VtValue val;
        UsdEditContext editContext(stage);
        attr.Get(&val);
        const bool retVal = prim.RemoveProperty(oldNameToken);
        if (!retVal)
        {
            removeAttributeFromReference(stage, oldNameToken, prim);
        }

        prim.CreateAttribute(TfToken(newName), typeName).Set(val);

        return true;
    }
    else
    {
        UsdRelationship rel = prim.GetRelationship(oldNameToken);
        if (rel)
        {
            SdfPathVector targets;
            rel.GetTargets(&targets);
            UsdEditContext editContext(stage);
            const bool retVal = prim.RemoveProperty(oldNameToken);
            if (!retVal)
            {
                removeAttributeFromReference(stage, oldNameToken, prim);
            }
            prim.CreateRelationship(TfToken(newName)).SetTargets(targets);

            return true;
        }
    }

    return false;
}

void removeProperty(pxr::UsdStageWeakPtr stage, UsdPrim& prim, const char* propName)
{
    TfToken propNameToken(propName);
    UsdProperty attr = prim.GetProperty(propNameToken);
    if (attr)
    {
        const bool retVal = prim.RemoveProperty(propNameToken);
        if (!retVal)
        {
            removeAttributeFromReference(stage, propNameToken, prim);
        }        
    }
}

bool removeAPI(UsdPrim& prim, const TfToken& apiName)
{
    if (!prim) {
        TF_CODING_ERROR("Invalid prim.");
        return false;
    }

    if (prim.IsInstanceProxy() || prim.IsInPrototype()) {
        TF_CODING_ERROR("Prim at <%s> is an instance proxy or is inside an "
            "instance master.", prim.GetPath().GetText());
        return false;
    }

    // Get the listop at the current edit target.
    UsdStagePtr stage = prim.GetStage();
    UsdEditTarget editTarget = stage->GetEditTarget();
    SdfPrimSpecHandle primSpec = editTarget.GetPrimSpecForScenePath(
        prim.GetPath());

    // this should be eventually same as UsdAPISchemaBase::_ApplyAPISchemaImpl
    // cannot for now as its hidden 
    if (!primSpec)
    {
        if (ARCH_UNLIKELY(prim.IsInPrototype())) {
            TF_CODING_ERROR("Cannot %s at path <%s>; "
                "authoring to an instancing master is not allowed.",
                "create prim spec", prim.GetPath().GetText());
            return false;
        }

        if (ARCH_UNLIKELY(prim.IsInstanceProxy())) {
            TF_CODING_ERROR("Cannot %s at path <%s>; "
                "authoring to an instance proxy is not allowed.",
                "create prim spec", prim.GetPath().GetText());
            return false;
        }

        const SdfPath& targetPath = editTarget.MapToSpecPath(prim.GetPath());
        primSpec = targetPath.IsEmpty() ? SdfPrimSpecHandle() :
            SdfCreatePrimInLayer(editTarget.GetLayer(), targetPath);
    }

    SdfTokenListOp listOp =
        primSpec->GetInfo(UsdTokens->apiSchemas).UncheckedGet<SdfTokenListOp>();

    // remove from the list
    TfTokenVector existingApiSchemas = listOp.IsExplicit() ?
        listOp.GetExplicitItems() : listOp.GetPrependedItems();

    TfTokenVector::iterator apiIt = std::find(existingApiSchemas.begin(), existingApiSchemas.end(), apiName);
    if (apiIt == existingApiSchemas.end())
    {
        return false;
    }

    existingApiSchemas.erase(apiIt);
    listOp.SetPrependedItems(existingApiSchemas);

    TfTokenVector deleteApiSchemas;
    deleteApiSchemas.push_back(apiName);

    SdfTokenListOp delListOp;

    if (auto result = listOp.ApplyOperations(delListOp)) {
        // Set the listop on the primSpec at the current edit target and return 
        // the prim
        primSpec->SetInfo(UsdTokens->apiSchemas, VtValue(*result));
        return true;
    }
    else {
        TF_CODING_ERROR("Failed to prepend api name %s to 'apiSchemas' listOp "
            "at path <%s>", apiName.GetText(), prim.GetPath().GetText());
        return false;
    }
}

VtValue checkPhysxAttribute(pxr::UsdStageWeakPtr stage, const TfToken& oldToken, UsdPrim& usdPrim)
{
    VtValue val;
    UsdAttribute attr = usdPrim.GetAttribute(oldToken);
    if (attr)
    {
        UsdEditContext editContext(stage);        
        attr.Get(&val);
        const bool retVal = usdPrim.RemoveProperty(oldToken);
        if (!retVal)
        {
            removeAttributeFromReference(stage, oldToken, usdPrim);
        }        
    }
    return val;
}

VtValue getAttribute(pxr::UsdStageWeakPtr stage, TfToken& oldToken, UsdPrim& usdPrim)
{
    VtValue val;
    UsdAttribute attr = usdPrim.GetAttribute(oldToken);
    if (attr)
    {
        attr.Get(&val);
    }
    return val;
}

bool checkAttributeAuthoredValue(const UsdPrim& usdPrim, const TfToken& attrNameToken)
{
    UsdAttribute attr = usdPrim.GetAttribute(attrNameToken);
    return attr && attr.HasAuthoredValue();
}

bool checkAttributeAuthoredValueWLog(const UsdPrim& usdPrim, const std::string& primPathStr, const TfToken& attrNameToken)
{
    if (checkAttributeAuthoredValue(usdPrim, attrNameToken))
    {
        gCheckLog += primPathStr + ":" + attrNameToken.GetString() + "\n";
        return true;
    }

    return false;
}

using TStrVec = std::vector<std::string>;

#define ATTRIBUTE_CHECK(typeName, ...)                                                                                 \
    {                                                                                                                  \
        static TStrVec oldAttrStr = { __VA_ARGS__ };                                                                   \
        static auto oldAttr = TfTokenVector(oldAttrStr.begin(), oldAttrStr.end());                                     \
        if (checkAttributes(usdPrim, oldAttr, primPathStr, typeName))                                                  \
            ret = true;                                                                                                \
    }

#define TOKEN_CHECK(checkFn, ...)                                                                                     \
    {                                                                                                                  \
        static TStrVec oldAttrStr = { __VA_ARGS__ };                                                                   \
        static auto oldAttr = TfTokenVector(oldAttrStr.begin(), oldAttrStr.end());                                     \
        if (checkFn(usdPrim, oldAttr, primPathStr))                                                                                 \
            ret = true;                                                                                                \
    }

auto checkAttributes = [](const pxr::UsdPrim& usdPrim, const TfTokenVector& tokens, const std::string& primPathStr,
    const std::string& typeName = std::string()) -> bool {
        bool ret = false;
        for (const TfToken& token : tokens)
        {
            if (usdPrim.GetAttribute(token))
            {
                gCheckLog += primPathStr + ":" + typeName + ":" + token.GetString() + "\n";
                ret = true;
            }
        }

        return ret;
};

void checkJoints(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{
    for (auto& usdrtPath : usdrtStage->GetPrimsWithTypeName(usdrt::TfToken("UsdPhysicsJoint")))
    {
        const omni::fabric::PathC pathC(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
        UsdPrim usdPrim = stage->GetPrimAtPath(usdPath);
        const std::string& primPathStr = usdrtPath.GetString();
        static const std::string& typeName = TfType::Find<UsdPhysicsJoint>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "body0", "body1", "localPos0", "localRot0", "localPos1",
            "localRot1", "jointEnabled", "collisionEnabled", "breakForce", "breakTorque")
    }
}

void checkSchemas(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{   
    static TStrVec oldApiSchemas = { "PhysicsAPI", "CollisionAPI", "VelocityAPI", "MassAPI", "FilteringPairsAPI",
                                     "ArticulationAPI", "PhysxArticulationJointAPI", "ArticulationJointAPI",
                                     "PhysxMeshCollisionAPI", "CharacterControllerAPI",
                                     "LimitAPI:rotX", "LimitAPI:rotY", "LimitAPI:rotZ", "LimitAPI:angular", "LimitAPI:transX", "LimitAPI:transY", "LimitAPI:transZ", "LimitAPI:distance", "LimitAPI:linear",
                                     "DriveAPI:rotX", "DriveAPI:rotY", "DriveAPI:rotZ", "DriveAPI:angular", "DriveAPI:transX", "DriveAPI:transY", "DriveAPI:transZ", "DriveAPI:distance", "DriveAPI:linear" };

    // A.B. removed PhysxCookedDataAPI - single vs multiple applied schemas cant be detected
    // This double for loop is not ideal.
    // Ideally we'd expose "any" queries in USDRT, or call Fabric findPrims directly
    for (const std::string& schemaName : oldApiSchemas)
    {
        for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken(schemaName)))
        {
            const std::string& primPathStr = usdrtPath.GetString();
            gCheckLog += primPathStr + ":" + schemaName + "\n";
            ret = true;
        }
    }

}

void checkPhysicsMaterialAPI(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{
    for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysicsMaterialAPI")))
    {
        const omni::fabric::PathC pathC(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
        UsdPrim usdPrim = stage->GetPrimAtPath(usdPath);
        const std::string& primPathStr = usdrtPath.GetString();
        static const std::string typeName("PhysicsMaterialAPI");
        ATTRIBUTE_CHECK(typeName, "density", "dynamicFriction", "staticFriction", "restitution")
    }
}

void checkScenes(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{
    for (auto& usdrtPath : usdrtStage->GetPrimsWithTypeName(usdrt::TfToken("UsdPhysicsScene")))
    {
        const omni::fabric::PathC pathC(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
        UsdPrim usdPrim = stage->GetPrimAtPath(usdPath);
        const std::string& primPathStr = usdrtPath.GetString();
        static const std::string& typeName = TfType::Find<UsdPhysicsScene>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "gravity", "invertCollisionGroupFilter")
    }
}

void checkPhysxCookedDataAPI(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{
    // A.B. temp disable
    //for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxCookedDataAPI")))
    //{
    //    const pxr::SdfPath usdPath = omni::fabric::toSdfPath(omni::fabric::PathC(usdrtPath));
    //    UsdPrim usdPrim = stage->GetPrimAtPath(usdPath);
    //    const std::string& primPathStr = usdrtPath.GetString();
    //    static const std::string& typeName = TfType::Find<PhysxSchemaPhysxCookedDataAPI>().GetTypeName();
    //    ATTRIBUTE_CHECK(typeName, "cookedDataType", "cookedData", "physxCookedData", "physxCookedDataType")
    //}
}

void checkPhysxSchemaPhysxSceneAPI(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{
    for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxSceneAPI")))
    {
        const omni::fabric::PathC pathC(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
        UsdPrim usdPrim = stage->GetPrimAtPath(usdPath);
        const std::string& primPathStr = usdrtPath.GetString();
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxSceneAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "bounceThreshold", "frictionOffsetThreshold", "collisionSystem", "solverType",
            "broadphaseType", gFrictionType, "enableCCD", "enableStabilization", "enableGPUDynamics",
            "enableEnhancedDeterminism", "gpuTempBufferCapacity", "gpuHeapCapacity", "gpuFoundLostPairsCapacity",
            "gpuMaxNumPartitions", "asyncSimRender", "physxScene:maxIterationCount", "physxScene:minIterationCount")

        ret |= checkAttributeAuthoredValueWLog(usdPrim, primPathStr, gPhysxSceneFrictionTypeToken);
    }
}

void checkPhysxSchemaPhysxRigidBodyAPI(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{
    for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxRigidBodyAPI")))
    {
        const omni::fabric::PathC pathC(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
        UsdPrim usdPrim = stage->GetPrimAtPath(usdPath);
        const std::string& primPathStr = usdrtPath.GetString();
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxRigidBodyAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "linearDamping", "angularDamping", "maxLinearVelocity", "maxAngularVelocity",
            "sleepThreshold", "stabilizationThreshold", "maxDepenetrationVelocity",
            "solverPositionIterationCount", "solverVelocityIterationCount", "enableCCD", "enableSpeculativeCCD",
            "lockedPosAxis", "lockedRotAxis")
    }
}

void checkPhysxSchemaPhysxTriangleMeshCollisionAPI(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{
    for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxTriangleMeshCollisionAPI")))
    {
        const omni::fabric::PathC pathC(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
        UsdPrim usdPrim = stage->GetPrimAtPath(usdPath);
        const std::string& primPathStr = usdrtPath.GetString();
        TOKEN_CHECK(checkAttributes, "physxsdfcollision:resolution")

        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxTriangleMeshCollisionAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName,
            "physxTriangleMeshCollision:sdfBitsPerSubgridPixel", "physxTriangleMeshCollision:sdfMargin",
            "physxTriangleMeshCollision:sdfNarrowBandThickness", "physxTriangleMeshCollision:sdfResolution",
            "physxTriangleMeshCollision:sdfSubgridResolution")
    }
}

void checkPhysxSchemaUsdPhysicsCollisionAPI(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{
    for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysicsCollisionAPI")))
    {
        const omni::fabric::PathC pathC(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
        UsdPrim usdPrim = stage->GetPrimAtPath(usdPath);
        const std::string& primPathStr = usdrtPath.GetString();
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxCollisionAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "physxCollisionCustomGeometry")
    }
}

void checkPhysxSchemaPhysxCollisionAPI(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{
    for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxCollisionAPI")))
    {
        const omni::fabric::PathC pathC(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
        UsdPrim usdPrim = stage->GetPrimAtPath(usdPath);
        const std::string& primPathStr = usdrtPath.GetString();
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxCollisionAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "contactOffset", "restOffset", "torsionalPatchRadius", "minTorsionalPatchRadius")
    }
}

void checkPhysxSchemaPhysxMaterialAPI(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{
    for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxMaterialAPI")))
    {
        const omni::fabric::PathC pathC(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
        UsdPrim usdPrim = stage->GetPrimAtPath(usdPath);
        const std::string& primPathStr = usdrtPath.GetString();
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxMaterialAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "frictionCombineMode", "restitutionCombineMode", "dampingCombineMode")
    }
}

void checkPhysxSchemaPhysxJointAPI(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{
    for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxJointAPI")))
    {
        const omni::fabric::PathC pathC(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
        UsdPrim usdPrim = stage->GetPrimAtPath(usdPath);
        const std::string& primPathStr = usdrtPath.GetString();
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxJointAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "enableCollision", "physxJoint:enableCollision", gEnableProjection)

        ret |= checkAttributeAuthoredValueWLog(usdPrim, primPathStr, gPhysxJointEnableProjectionToken);
    }
}

void checkPhysxSchemaPhysxArticulationAPI(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{
    for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxArticulationAPI")))
    {
        const omni::fabric::PathC pathC(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
        UsdPrim usdPrim = stage->GetPrimAtPath(usdPath);
        const std::string& primPathStr = usdrtPath.GetString();
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxArticulationAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "solverPositionIterationCount", "solverVelocityIterationCount", "sleepThreshold",
            "stabilizationThreshold")
    }
}

void checkPhysxSchemaPhysxArticulationForceSensorAPI(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{
    for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxArticulationForceSensorAPI")))
    {
        const omni::fabric::PathC pathC(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
        CARB_LOG_WARN(
            "Physics backwardsCompatibility: PhysxArticulationForceSensorAPI has been removed. Prim (%s).",
            usdPath.GetText());

        const std::string& primPathStr = usdrtPath.GetString();
        gCheckLog += primPathStr + ":PhysxArticulationForceSensorAPI\n";
        ret = true;
    }
}

void checkPhysxArticulationJoint(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{
    for (auto& usdrtPath : usdrtStage->GetPrimsWithTypeName(usdrt::TfToken("UsdPhysicsJoint")))
    {
        const omni::fabric::PathC pathC(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
        UsdPrim usdPrim = stage->GetPrimAtPath(usdPath);
        const std::string& primPathStr = usdrtPath.GetString();
        static const std::string& typeName = TfType::Find<UsdPhysicsJoint>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "frictionCoefficient", "physxArticulationJoint:frictionCoefficient")
    }
}

void checkUsdPhysicsRevoluteJoint(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{
    for (auto& usdrtPath : usdrtStage->GetPrimsWithTypeName(usdrt::TfToken("UsdPhysicsRevoluteJoint")))
    {
        const omni::fabric::PathC pathC(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
        UsdPrim usdPrim = stage->GetPrimAtPath(usdPath);
        const std::string& primPathStr = usdrtPath.GetString();
        static const std::string& typeName = TfType::Find<UsdPhysicsRevoluteJoint>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "physxArticulationJoint:maxJointVelocity", "maxJointVelocity")
    }
}

void checkUsdPhysicsSphericalJoint(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{
    for (auto& usdrtPath : usdrtStage->GetPrimsWithTypeName(usdrt::TfToken("UsdPhysicsSphericalJoint")))
    {
        const omni::fabric::PathC pathC(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
        UsdPrim usdPrim = stage->GetPrimAtPath(usdPath);
        const std::string& primPathStr = usdrtPath.GetString();
        static const std::string& typeName = TfType::Find<UsdPhysicsSphericalJoint>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "physxArticulationJoint:maxJointVelocity", "maxJointVelocity")
    }
}

void checkPhysxSchemaPhysxTriggerAPI(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{
    for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxTriggerAPI")))
    {
        const omni::fabric::PathC pathC(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
        UsdPrim usdPrim = stage->GetPrimAtPath(usdPath);
        const std::string& primPathStr = usdrtPath.GetString();
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxTriggerAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "onEnterScript", "onLeaveScript")
    }
}

void checkPhysxVehicle(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{    
    // vehicle old types
    static TStrVec oldTypeVehicleNames = { "PhysxVehicleGlobalSettings", "PhysxVehicleTire", "PhysxVehicleSuspension",
        "PhysxVehicleWheel", "PhysxVehicleEngine", "PhysxVehicleGears", "PhysxVehicleAutoGearBox",
        "PhysxVehicleClutch", "PhysxVehicleDriveBasic", "PhysxVehicleDriveStandard" };

    for (auto& oldType : oldTypeVehicleNames)
    {
        if (!usdrtStage->GetPrimsWithTypeName(usdrt::TfToken(oldType)).empty())
        {
            ret = true;
            break;
        }
    }
}

void checkPhysxSchemaPhysxVehicleContextAPI(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{
    for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxVehicleContextAPI")))
    {
        const omni::fabric::PathC pathC(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
        UsdPrim usdPrim = stage->GetPrimAtPath(usdPath);
        const std::string& primPathStr = usdrtPath.GetString();
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxVehicleContextAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "physxVehicleContext:sweepRadiusScale",
            "physxVehicleContext:sweepWidthScale")
    }
}

void checkPhysxSchemaPhysxVehicleControllerAPI(bool& ret, pxr::UsdStageWeakPtr stage, usdrt::UsdStageRefPtr usdrtStage)
{
    for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxVehicleControllerAPI")))
    {
        const omni::fabric::PathC pathC(usdrtPath);
        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
        UsdPrim usdPrim = stage->GetPrimAtPath(usdPath);
        const std::string& primPathStr = usdrtPath.GetString();
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxVehicleControllerAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "physxVehicleController:shiftUp", "physxVehicleController:shiftDown",
            "physxVehicleController:automatic")
    }
}


bool checkPrimPassive(pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& usdPrim)
{
    bool ret = false;
    const SdfPath& primPath = usdPrim.GetPrimPath();
    const std::string& primPathStr = primPath.GetString();



    auto checkTypes = [](const pxr::UsdPrim& usdPrim, const TfTokenVector& tokens, const std::string& primPathStr) -> bool {
        bool ret = false;
        for (const TfToken& token : tokens)
        {
            if (usdPrim.GetTypeName() == token)
            {
                gCheckLog += primPathStr + " " + token.GetString() + "\n";
                ret = true;
            }
        }

        return ret;
    };

    auto checkSchemas = [&primPathStr](const TfToken& apiSchema, const TStrVec& strings) -> bool {
        bool ret = false;
        for (const std::string& str : strings)
        {
            if (apiSchema.GetString() == str)
            {
                gCheckLog += primPathStr + ":" + str + "\n";
                ret = true;
            }
        }

        return ret;
    };

    TOKEN_CHECK(checkAttributes, "enableKinematic", "approximationShape", "physxMeshCollision:approximation")

    TOKEN_CHECK(checkTypes, "PhysicsMaterial", "ConvexMesh", "Joint", "RevolutePhysicsJoint",
                "PrismaticPhysicsJoint",
                "DistancePhysicsJoint", "SphericalPhysicsJoint", "FixedPhysicsJoint", "CollisionGroup")

    if (usdPrim.IsA<UsdPhysicsJoint>())
    {
        static const std::string& typeName = TfType::Find<UsdPhysicsJoint>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "body0", "body1", "localPos0", "localRot0", "localPos1",
                    "localRot1", "jointEnabled", "collisionEnabled", "breakForce", "breakTorque")
    }

    static TStrVec oldApiSchemas = { "PhysicsAPI", "CollisionAPI", "VelocityAPI", "MassAPI", "FilteringPairsAPI",
                                     "ArticulationAPI", "PhysxArticulationJointAPI", "ArticulationJointAPI",
                                     "PhysxMeshCollisionAPI", "PhysxCookedDataAPI", "CharacterControllerAPI" };

    static TStrVec oldApiMultiSchemas = []() {
        TStrVec ret;
        const TStrVec limitAxes({ "rotX", "rotY", "rotZ", "angular", "transX", "transY", "transZ", "distance", "linear" });
        const std::string limitString("LimitAPI:"), driveString("DriveAPI:");
        for (const std::string& limitAxis : limitAxes)
        {
            ret.push_back(limitString + limitAxis);
            ret.push_back(driveString + limitAxis);
        }
        return ret;
    }();

    auto appliedSchemas = usdPrim.GetPrimTypeInfo().GetAppliedAPISchemas();
    for (const TfToken& apiSchema : appliedSchemas)
    {
        if (checkSchemas(apiSchema, oldApiSchemas))
            ret = true;

        if (checkSchemas(apiSchema, oldApiMultiSchemas))
            ret = true;

        static const std::string typeName("PhysicsMaterialAPI");
        if (apiSchema.GetString() == typeName)
        {
            ATTRIBUTE_CHECK(typeName, "density", "dynamicFriction", "staticFriction", "restitution")
        }

        // articulation sensor API deprecation warning:
        if (apiSchema.GetString() == "PhysxArticulationForceSensorAPI")
        {
            CARB_LOG_WARN("Physics backwardsCompatibility: PhysxArticulationForceSensorAPI has been removed. Prim (%s).", usdPrim.GetPath().GetText());
        }
    }

    if (usdPrim.IsA<UsdPhysicsScene>())
    {
        static const std::string& typeName = TfType::Find<UsdPhysicsScene>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "gravity", "invertCollisionGroupFilter")
    }

    if (usdPrim.HasAPI<PhysxSchemaPhysxCookedDataAPI>())
    {
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxCookedDataAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "cookedDataType", "cookedData", "physxCookedData", "physxCookedDataType")
    }

    if (PhysxSchemaPhysxSceneAPI::Get(stage, primPath))
    {
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxSceneAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "bounceThreshold", "frictionOffsetThreshold", "collisionSystem", "solverType",
                    "broadphaseType", gFrictionType, "enableCCD", "enableStabilization", "enableGPUDynamics",
                    "enableEnhancedDeterminism", "gpuTempBufferCapacity", "gpuHeapCapacity", "gpuFoundLostPairsCapacity",
                    "gpuMaxNumPartitions", "asyncSimRender", "physxScene:maxIterationCount", "physxScene:minIterationCount")

        ret |= checkAttributeAuthoredValueWLog(usdPrim, primPathStr, gPhysxSceneFrictionTypeToken);
    }

    if (PhysxSchemaPhysxRigidBodyAPI::Get(stage, primPath))
    {
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxRigidBodyAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "linearDamping", "angularDamping", "maxLinearVelocity", "maxAngularVelocity",
                    "sleepThreshold", "stabilizationThreshold", "maxDepenetrationVelocity",
                    "solverPositionIterationCount", "solverVelocityIterationCount", "enableCCD", "enableSpeculativeCCD",
                    "lockedPosAxis", "lockedRotAxis")

    }

    if (PhysxSchemaPhysxTriangleMeshCollisionAPI::Get(stage, primPath))
    {
        TOKEN_CHECK(checkAttributes, "physxsdfcollision:resolution")

        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxTriangleMeshCollisionAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName,
            "physxTriangleMeshCollision:sdfBitsPerSubgridPixel", "physxTriangleMeshCollision:sdfMargin",
            "physxTriangleMeshCollision:sdfNarrowBandThickness", "physxTriangleMeshCollision:sdfResolution",
            "physxTriangleMeshCollision:sdfSubgridResolution")
    }

    if (PhysxSchemaPhysxCollisionAPI::Get(stage, primPath))
    {
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxCollisionAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "contactOffset", "restOffset", "torsionalPatchRadius", "minTorsionalPatchRadius")
    }

    if (PhysxSchemaPhysxMaterialAPI::Get(stage, primPath))
    {
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxMaterialAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "frictionCombineMode", "restitutionCombineMode", "dampingCombineMode")
    }

    if (PhysxSchemaPhysxJointAPI::Get(stage, primPath))
    {
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxJointAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "enableCollision", "physxJoint:enableCollision", gEnableProjection)

        ret |= checkAttributeAuthoredValueWLog(usdPrim, primPathStr, gPhysxJointEnableProjectionToken);
    }


    // physxArticulaton
    if (PhysxSchemaPhysxArticulationAPI::Get(stage, primPath))
    {
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxArticulationAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "solverPositionIterationCount", "solverVelocityIterationCount", "sleepThreshold",
                    "stabilizationThreshold")
    }

    // physxArticulatonJoint
    TOKEN_CHECK(checkAttributes, "frictionCoefficient", "physxArticulationJoint:frictionCoefficient")

    if (usdPrim.IsA<UsdPhysicsRevoluteJoint>())
    {
        static const std::string& typeName = TfType::Find<UsdPhysicsRevoluteJoint>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "physxArticulationJoint:maxJointVelocity", "maxJointVelocity")
    }

    if (usdPrim.IsA<UsdPhysicsSphericalJoint>())
    {
        static const std::string& typeName = TfType::Find<UsdPhysicsSphericalJoint>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "physxArticulationJoint:maxJointVelocity", "maxJointVelocity")
    }

    // physxTrigger
    if (PhysxSchemaPhysxTriggerAPI::Get(stage, primPath))
    {
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxTriggerAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "onEnterScript", "onLeaveScript")
    }

    // vehicle old types
    TOKEN_CHECK(checkTypes, "PhysxVehicleGlobalSettings", "PhysxVehicleTire", "PhysxVehicleSuspension",
                "PhysxVehicleWheel", "PhysxVehicleEngine", "PhysxVehicleGears", "PhysxVehicleAutoGearBox",
                "PhysxVehicleClutch", "PhysxVehicleDriveBasic", "PhysxVehicleDriveStandard")

    // PhysxVehicleContext
    if (usdPrim.HasAPI<PhysxSchemaPhysxVehicleContextAPI>())
    {
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxVehicleContextAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "physxVehicleContext:sweepRadiusScale",
            "physxVehicleContext:sweepWidthScale")
    }

    // PhysxVehicleController
    if (usdPrim.HasAPI<PhysxSchemaPhysxVehicleControllerAPI>())
    {
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxVehicleControllerAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "physxVehicleController:shiftUp", "physxVehicleController:shiftDown",
            "physxVehicleController:automatic")
    }

    // UsdPhysicsCollisionAPI
    if (usdPrim.HasAPI<UsdPhysicsCollisionAPI>())
    {
        static const std::string& typeName = TfType::Find<PhysxSchemaPhysxVehicleControllerAPI>().GetTypeName();
        ATTRIBUTE_CHECK(typeName, "physxCollisionCustomGeometry")
    }

    return ret;
}

struct SecondPassPrimTracker
{
    pxr::UsdPrim scenePrim;
    pxr::UsdPrim oldVehicleGlobalSettingsPrim;
};

void checkPrim(pxr::UsdStageWeakPtr stage, pxr::UsdPrim& usdPrim, SecondPassPrimTracker& secondPassPrimTracker)
{
    const SdfPath primPath = usdPrim.GetPrimPath();

    float metersPerUnit = float(UsdGeomGetStageMetersPerUnit(stage));

    // kinematics check
    static TfToken oldKinematicsToken("enableKinematic");
    if (usdPrim.GetAttribute(oldKinematicsToken))
    {
        UsdEditContext editContext(stage);
        bool isKinematic = false;
        usdPrim.GetAttribute(oldKinematicsToken).Get(&isKinematic);        
        const bool retVal = usdPrim.RemoveProperty(oldKinematicsToken);
        if (!retVal)
        {
            removeAttributeFromReference(stage, oldKinematicsToken, usdPrim);
        }
        UsdPhysicsRigidBodyAPI physicsAPI = UsdPhysicsRigidBodyAPI::Get(stage, primPath);
        physicsAPI.CreateKinematicEnabledAttr().Set(isKinematic);
    }

    // approximationShape check
    {
        static TfToken oldApproximationShapeToken("approximationShape");
        if (usdPrim.GetAttribute(oldApproximationShapeToken))
        {
            UsdEditContext editContext(stage);
            TfToken approx = UsdPhysicsTokens.Get()->none;
            usdPrim.GetAttribute(oldApproximationShapeToken).Get(&approx);        
            const bool retVal = usdPrim.RemoveProperty(oldApproximationShapeToken);
            if (!retVal)
            {
                removeAttributeFromReference(stage, oldApproximationShapeToken, usdPrim);
            }
            UsdPhysicsMeshCollisionAPI physicsMeshAPI = UsdPhysicsMeshCollisionAPI::Get(stage, primPath);
            if (!physicsMeshAPI)
                physicsMeshAPI = UsdPhysicsMeshCollisionAPI::Apply(usdPrim);
            physicsMeshAPI.CreateApproximationAttr().Set(approx);
        }
    }
    {
        static TfToken oldApproximationShapeToken("physxMeshCollision:approximation");
        if (usdPrim.GetAttribute(oldApproximationShapeToken))
        {
            UsdEditContext editContext(stage);
            TfToken approx = UsdPhysicsTokens.Get()->none;
            usdPrim.GetAttribute(oldApproximationShapeToken).Get(&approx);
            const bool retVal = usdPrim.RemoveProperty(oldApproximationShapeToken);
            if (!retVal)
            {
                removeAttributeFromReference(stage, oldApproximationShapeToken, usdPrim);
            }
            UsdPhysicsMeshCollisionAPI physicsMeshAPI = UsdPhysicsMeshCollisionAPI::Get(stage, primPath);
            if (!physicsMeshAPI)
                physicsMeshAPI = UsdPhysicsMeshCollisionAPI::Apply(usdPrim);
            physicsMeshAPI.CreateApproximationAttr().Set(approx);
        }
    }

    // Fixup previous boolean 'asynchronous scene render' attribute to new schema update scene token attribute
    // (either 'Disabled' or 'Synchronous' or 'Asynchronous')
    {
        static TfToken oldAsyncSimulationToken("asyncSimRender");
        if (usdPrim.GetAttribute(oldAsyncSimulationToken))
        {
            UsdEditContext editContext(stage);
            bool isAsyncSimBool = false;
            usdPrim.GetAttribute(oldAsyncSimulationToken).Get(&isAsyncSimBool);
            const bool retVal = usdPrim.RemoveProperty(oldAsyncSimulationToken);
            if (!retVal)
            {
                removeAttributeFromReference(stage, oldAsyncSimulationToken, usdPrim);
            }
            
            PhysxSchemaPhysxSceneAPI physicsSchemaSceneAPI = PhysxSchemaPhysxSceneAPI::Get(stage, primPath);
            if (!physicsSchemaSceneAPI)
                physicsSchemaSceneAPI = PhysxSchemaPhysxSceneAPI::Apply(usdPrim);

            if (isAsyncSimBool)
                physicsSchemaSceneAPI.CreateUpdateTypeAttr().Set(TfToken("Asynchronous"));
            else
                physicsSchemaSceneAPI.CreateUpdateTypeAttr().Set(TfToken("Synchronous"));
        }
    }

    // PhysicsMaterial -> PhysicsMaterialAPI
    static TfToken oldMaterialPrim("PhysicsMaterial");
    if (usdPrim.GetTypeName() == oldMaterialPrim)
    {
        UsdEditContext editContext(stage);
        static TfToken materialPrim("Material");
        setTypeName(stage, materialPrim, usdPrim);
        UsdPhysicsMaterialAPI materialAPI = UsdPhysicsMaterialAPI::Apply(usdPrim);

        double val;
        static TfToken densityToken("density");
        if (switchDoubleToFloat(stage, densityToken, usdPrim, val))
        {
            float modifyVal = metersPerUnit * metersPerUnit * metersPerUnit;
            materialAPI.CreateDensityAttr().Set(float(val) / modifyVal);
        }
    }

    // convexMesh check
    static TfToken oldConvexPrim("ConvexMesh");    
    if (usdPrim.GetTypeName() == oldConvexPrim)
    {
        CARB_LOG_WARN("Physics backwardsCompatibility: ConvexMesh prim not supported anymore, replace it with a mesh please, prim (%s).", usdPrim.GetPath().GetText());
    }

    // joint backwards check
    static TfToken oldJoint("Joint");    
    if (usdPrim.GetTypeName() == oldJoint)
    {
        static TfToken revoluteJointAPI("RevoluteJointAPI");
        static TfToken prismaticJointAPI("PrismaticJointAPI");
        static TfToken distanceJointAPI("DistanceJointAPI");
        static TfToken sphericalJointAPI("SphericalJointAPI");

        usdparser::ObjectType jointType = usdparser::ObjectType::eJointD6;
        TfTokenVector appliedApis = usdPrim.GetPrimTypeInfo().GetAppliedAPISchemas();
        for (size_t i = 0; i < appliedApis.size(); i++)
        {
            if (appliedApis[i] == revoluteJointAPI)
            {
                jointType = usdparser::ObjectType::eJointRevolute;
            }
            else if (appliedApis[i] == prismaticJointAPI)
            {
                jointType = usdparser::ObjectType::eJointPrismatic;
            }
            else if (appliedApis[i] == distanceJointAPI)
            {
                jointType = usdparser::ObjectType::eJointDistance;
            }
            else if (appliedApis[i] == sphericalJointAPI)
            {
                jointType = usdparser::ObjectType::eJointSpherical;
            }
        }

        UsdEditContext editContext(stage);
        if (jointType == usdparser::ObjectType::eJointRevolute)
        {
            static TfToken revolutePhysicsJoint("PhysicsRevoluteJoint");
            removeAPI(usdPrim, revoluteJointAPI);
            setTypeName(stage, revolutePhysicsJoint, usdPrim);

            renameProperty(stage, usdPrim, "axis", "physics:axis");
            switchRadToDegree<float>(stage, TfToken("lowerLimit"), usdPrim);
            switchRadToDegree<float>(stage, TfToken("upperLimit"), usdPrim);
            renameProperty(stage, usdPrim, "lowerLimit", "physics:lowerLimit");
            renameProperty(stage, usdPrim, "upperLimit", "physics:upperLimit");
        }
        else if (jointType == usdparser::ObjectType::eJointPrismatic)
        {
            static TfToken prismaticPhysicsJoint("PhysicsPrismaticJoint");
            removeAPI(usdPrim, prismaticJointAPI);            
            setTypeName(stage, prismaticPhysicsJoint, usdPrim);

            renameProperty(stage, usdPrim, "axis", "physics:axis");
            renameProperty(stage, usdPrim, "lowerLimit", "physics:lowerLimit");
            renameProperty(stage, usdPrim, "upperLimit", "physics:upperLimit");
        }
        else if (jointType == usdparser::ObjectType::eJointDistance)
        {
            static TfToken distancePhysicsJoint("PhysicsDistanceJoint");
            removeAPI(usdPrim, distanceJointAPI);            
            setTypeName(stage, distancePhysicsJoint, usdPrim);

            PhysxSchemaPhysxPhysicsDistanceJointAPI::Apply(usdPrim);
            renameProperty(stage, usdPrim, "springEnabled", "physxPhysicsDistanceJoint:springEnabled");
            renameProperty(stage, usdPrim, "minDistance", "physics:minDistance");
            renameProperty(stage, usdPrim, "maxDistance", "physics:maxDistance");
        }
        else if (jointType == usdparser::ObjectType::eJointSpherical)
        {
            static TfToken sphericalPhysicsJoint("PhysicsSphericalJoint");
            removeAPI(usdPrim, sphericalJointAPI);            
            setTypeName(stage, sphericalPhysicsJoint, usdPrim);

            renameProperty(stage, usdPrim, "axis", "physics:axis");
            switchRadToDegree<float>(stage, TfToken("coneAngle0Limit"), usdPrim);
            switchRadToDegree<float>(stage, TfToken("coneAngle1Limit"), usdPrim);
            renameProperty(stage, usdPrim, "coneAngle0Limit", "physics:coneAngle0Limit");
            renameProperty(stage, usdPrim, "coneAngle1Limit", "physics:coneAngle1Limit");
        }
        else
        {
            static TfToken physicsJoint("PhysicsJoint");
            setTypeName(stage, physicsJoint, usdPrim);
        }
    }

    // joint renames
    static TfToken oldRevoluteJoint("RevolutePhysicsJoint");
    if (usdPrim.GetTypeName() == oldRevoluteJoint)
    {
        UsdEditContext editContext(stage);

        renameProperty(stage, usdPrim, "axis", "physics:axis");
        switchRadToDegree<float>(stage, TfToken("lowerLimit"), usdPrim);
        switchRadToDegree<float>(stage, TfToken("upperLimit"), usdPrim);
        renameProperty(stage, usdPrim, "lowerLimit", "physics:lowerLimit");
        renameProperty(stage, usdPrim, "upperLimit", "physics:upperLimit");
        
        static TfToken revolutePhysicsJoint("PhysicsRevoluteJoint");
        setTypeName(stage, revolutePhysicsJoint, usdPrim);
    }
    static TfToken oldPrismaticJoint("PrismaticPhysicsJoint");
    if (usdPrim.GetTypeName() == oldPrismaticJoint)
    {
        UsdEditContext editContext(stage);

        renameProperty(stage, usdPrim, "axis", "physics:axis");
        renameProperty(stage, usdPrim, "lowerLimit", "physics:lowerLimit");
        renameProperty(stage, usdPrim, "upperLimit", "physics:upperLimit");

        static TfToken prismaticPhysicsJoint("PhysicsPrismaticJoint");
        setTypeName(stage, prismaticPhysicsJoint, usdPrim);
    }
    static TfToken oldDistanceJoint("DistancePhysicsJoint");
    if (usdPrim.GetTypeName() == oldDistanceJoint)
    {
        UsdEditContext editContext(stage);

        PhysxSchemaPhysxPhysicsDistanceJointAPI::Apply(usdPrim);
        renameProperty(stage, usdPrim, "springEnabled", "physxPhysicsDistanceJoint:springEnabled");
        renameProperty(stage, usdPrim, "minDistance", "physics:minDistance");
        renameProperty(stage, usdPrim, "maxDistance", "physics:maxDistance");

        static TfToken distancePhysicsJoint("PhysicsDistanceJoint");
        setTypeName(stage, distancePhysicsJoint, usdPrim);
    }
    static TfToken oldSphericalJoint("SphericalPhysicsJoint");
    if (usdPrim.GetTypeName() == oldSphericalJoint)
    {
        UsdEditContext editContext(stage);

        renameProperty(stage, usdPrim, "axis", "physics:axis");
        switchRadToDegree<float>(stage, TfToken("coneAngle0Limit"), usdPrim);
        switchRadToDegree<float>(stage, TfToken("coneAngle1Limit"), usdPrim);
        renameProperty(stage, usdPrim, "coneAngle0Limit", "physics:coneAngle0Limit");
        renameProperty(stage, usdPrim, "coneAngle1Limit", "physics:coneAngle1Limit");

        static TfToken sphericalPhysicsJoint("PhysicsSphericalJoint");
        setTypeName(stage, sphericalPhysicsJoint, usdPrim);
    }
    static TfToken oldFixedJoint("FixedPhysicsJoint");
    if (usdPrim.GetTypeName() == oldFixedJoint)
    {
        UsdEditContext editContext(stage);
        static TfToken fixedPhysicsJoint("PhysicsFixedJoint");
        setTypeName(stage, fixedPhysicsJoint, usdPrim);
    }
    
    if (usdPrim.GetTypeName() == TfToken("CollisionGroup"))
    {
        UsdEditContext editContext(stage);        
        setTypeName(stage, TfToken("PhysicsCollisionGroup"), usdPrim);
        renameProperty(stage, usdPrim, "filteredGroups", "physics:filteredGroups");

        const static TfToken collisionToken("colliders");
        UsdCollectionAPI::Apply(usdPrim, collisionToken);
    }
    
    // try to rename physicsJoint stuff
    if (usdPrim.IsA<UsdPhysicsJoint>())
    {
        renameProperty(stage, usdPrim, "body0", "physics:body0");
        renameProperty(stage, usdPrim, "body1", "physics:body1");
        renameProperty(stage, usdPrim, "localPos0", "physics:localPos0");
        renameProperty(stage, usdPrim, "localRot0", "physics:localRot0");
        renameProperty(stage, usdPrim, "localPos1", "physics:localPos1");
        renameProperty(stage, usdPrim, "localRot1", "physics:localRot1");
        renameProperty(stage, usdPrim, "jointEnabled", "physics:jointEnabled");
        renameProperty(stage, usdPrim, "collisionEnabled", "physics:collisionEnabled");        
        renameProperty(stage, usdPrim, "breakForce", "physics:breakForce");        
        renameProperty(stage, usdPrim, "breakTorque", "physics:breakTorque");
    }

    // PhysicsAPI's 
    TfTokenVector appliedSchemas = usdPrim.GetPrimTypeInfo().GetAppliedAPISchemas();
    std::vector<std::string> limitAxis({ "rotX", "rotY", "rotZ", "angular", "transX", "transY", "transZ", "distance", "linear" });
    const std::string limitString("LimitAPI:");
    const std::string driveString("DriveAPI:");
    for (size_t i = 0; i < appliedSchemas.size(); i++)
    {
        const TfToken& apiSchema = appliedSchemas[i];

        if(apiSchema.GetString() == "PhysxArticulationForceSensorAPI")
        {
            removeAPI(usdPrim, TfToken("PhysxArticulationForceSensorAPI"));
        }

        // PhysicsBodyAPI
        if (apiSchema.GetString() == "PhysicsAPI")
        {            
            removeAPI(usdPrim, TfToken("PhysicsAPI"));
            UsdPhysicsRigidBodyAPI::Apply(usdPrim);
            renameProperty(stage, usdPrim, "physicsEnabled", "physics:rigidBodyEnabled");
            renameProperty(stage, usdPrim, "kinematicEnabled", "physics:kinematicEnabled");
            removeProperty(stage, usdPrim, "bodyType");
            renameProperty(stage, usdPrim, "simulationOwner", "physics:simulationOwner");
            TfToken oldNameToken("physxRigidBody:wakeOnStart");
            UsdAttribute attr = usdPrim.GetAttribute(oldNameToken);
            if (attr)
            {
                SdfValueTypeName typeName = attr.GetTypeName();
                bool val;
                UsdEditContext editContext(stage);
                attr.Get(&val);
                const bool retVal = usdPrim.RemoveProperty(oldNameToken);
                if (!retVal)
                {
                    removeAttributeFromReference(stage, oldNameToken, usdPrim);
                }

                usdPrim.CreateAttribute(TfToken("physics:startsAsleep"), typeName).Set(!val);
            }
        }

        // PhysicsCollisionAPI
        if (apiSchema.GetString() == "CollisionAPI")
        {            
            removeAPI(usdPrim, TfToken("CollisionAPI"));
            UsdPhysicsCollisionAPI::Apply(usdPrim);
            renameProperty(stage, usdPrim, "collisionEnabled", "physics:collisionEnabled");
            renameProperty(stage, usdPrim, "physicsMaterial", "material:binding:physics");
            UsdRelationship rel = usdPrim.GetRelationship(TfToken("collisionGroup"));
            if (rel)
            {
                SdfPathVector targets;
                rel.GetTargets(&targets);
                for (size_t i = 0; i < targets.size(); i++)
                {
                    const SdfPath& collisionGroupPath = targets[i];
                    UsdPhysicsCollisionGroup colGroup = UsdPhysicsCollisionGroup(stage->GetPrimAtPath(collisionGroupPath));
                    if (colGroup)
                    {
                        const static TfToken collisionToken("colliders");
                        UsdCollectionAPI collectionAPI = UsdCollectionAPI::Get(colGroup.GetPrim(), collisionToken);
                        collectionAPI.GetIncludesRel().AddTarget(usdPrim.GetPrimPath());
                    }
                }
                removeProperty(stage, usdPrim, "collisionGroup");
            }
            renameProperty(stage, usdPrim, "simulationOwner", "physics:simulationOwner");
        }

        // PhysicsVelocityAPI
        if (apiSchema.GetString() == "VelocityAPI")
        {            
            removeAPI(usdPrim, TfToken("VelocityAPI"));
            renameProperty(stage, usdPrim, "velocity", "physics:velocity");
            switchRadToDegree<pxr::GfVec3f>(stage, TfToken("angularVelocity"), usdPrim);
            renameProperty(stage, usdPrim, "angularVelocity", "physics:angularVelocity");
        }

        // PhysicsMassAPI
        if (apiSchema.GetString() == "MassAPI")
        {            
            removeAPI(usdPrim, TfToken("MassAPI"));
            UsdPhysicsMassAPI physicsMassAPI = UsdPhysicsMassAPI::Apply(usdPrim);
            if (physicsMassAPI)
            {
                double val;
                if (switchDoubleToFloat(stage, TfToken("density"), usdPrim, val))
                {
                    const float modifyVal = metersPerUnit * metersPerUnit * metersPerUnit;
                    physicsMassAPI.CreateDensityAttr().Set(float(val) * modifyVal);
                }
                else
                {
                    UsdAttribute attr = usdPrim.GetAttribute(TfToken("density"));
                    if (attr)
                    {
                        float fval;
                        attr.Get(&fval);
                        const float modifyVal = metersPerUnit * metersPerUnit * metersPerUnit;
                        attr.Set(float(fval)* modifyVal);
                    }
                    renameProperty(stage, usdPrim, "density", "physics:density");
                }
                if (switchDoubleToFloat(stage, TfToken("mass"), usdPrim, val))
                {
                    physicsMassAPI.CreateMassAttr().Set(float(val));
                }
                else
                {
                    renameProperty(stage, usdPrim, "mass", "physics:mass");
                }
                pxr::GfVec3d val3;
                if (switchDoubleToFloat(stage, TfToken("centerOfMass"), usdPrim, val3))
                {
                    physicsMassAPI.CreateCenterOfMassAttr().Set(GfVec3f(val3));
                }
                else
                {
                    renameProperty(stage, usdPrim, "centerOfMass", "physics:centerOfMass");
                }
                if (switchDoubleToFloat(stage, TfToken("diagonalInertia"), usdPrim, val3))
                {
                    physicsMassAPI.CreateDiagonalInertiaAttr().Set(GfVec3f(val3));
                }
                else
                {
                    renameProperty(stage, usdPrim, "diagonalInertia", "physics:diagonalInertia");
                }
                GfQuatd val4;
                if (switchDoubleToFloat(stage, TfToken("principalAxes"), usdPrim, val4))
                {
                    physicsMassAPI.CreatePrincipalAxesAttr().Set(GfQuatf(val4));
                }
                else
                {
                    renameProperty(stage, usdPrim, "principalAxes", "physics:principalAxes");
                }
            }
        }

        // PhysicsMaterialAPI
        if (apiSchema.GetString() == "PhysicsMaterialAPI")
        {
            UsdPhysicsMaterialAPI physicsMaterialAPI = UsdPhysicsMaterialAPI::Get(stage, usdPrim.GetPrimPath());
            double val;
            if (switchDoubleToFloat(stage, TfToken("density"), usdPrim, val))
            {
                float modifyVal = metersPerUnit * metersPerUnit * metersPerUnit;
                physicsMaterialAPI.CreateDensityAttr().Set(float(val) * modifyVal);
            }            
            renameProperty(stage, usdPrim, "dynamicFriction", "physics:dynamicFriction");
            renameProperty(stage, usdPrim, "staticFriction", "physics:staticFriction");
            renameProperty(stage, usdPrim, "restitution", "physics:restitution");
        }

        // PhysicsFilteredPairsAPI
        if (apiSchema.GetString() == "FilteringPairsAPI")
        {            
            removeAPI(usdPrim, TfToken("FilteringPairsAPI"));
            UsdPhysicsFilteredPairsAPI::Apply(usdPrim);

            renameProperty(stage, usdPrim, "filteredPairs", "physics:filteredPairs");
        }

        // ArticulationAPI
        if (apiSchema.GetString() == "ArticulationAPI")
        {            
            removeAPI(usdPrim, TfToken("ArticulationAPI"));
            UsdPhysicsArticulationRootAPI::Apply(usdPrim);
            PhysxSchemaPhysxArticulationAPI::Apply(usdPrim);
            removeProperty(stage, usdPrim, "fixBase");
            renameProperty(stage, usdPrim, "enableSelfCollisions", "physxArticulation:enabledSelfCollisions");
        }

        if (apiSchema.GetString() == "PhysxArticulationJointAPI")
        {
            removeAPI(usdPrim, TfToken("PhysxArticulationJointAPI"));
            PhysxSchemaPhysxJointAPI::Apply(usdPrim);
        }

        if (apiSchema.GetString() == "ArticulationJointAPI")
        {
            removeAPI(usdPrim, TfToken("ArticulationJointAPI"));
        }

        for (size_t axis = 0; axis < limitAxis.size(); axis++)
        {            
            if (apiSchema.GetString() == limitString + limitAxis[axis])
            {                
                removeAPI(usdPrim, TfToken(limitString + limitAxis[axis]));
                UsdPhysicsLimitAPI::Apply(usdPrim, TfToken(limitAxis[axis]));
                std::string lowLimitStr = "limit:" + limitAxis[axis] + ":low";
                std::string lowLimitPhyStr = "limit:" + limitAxis[axis] + ":physics:low";
                std::string highLimitStr = "limit:" + limitAxis[axis] + ":high";
                std::string highLimitPhyStr = "limit:" + limitAxis[axis] + ":physics:high";
                if (axis < 4)
                {
                    switchRadToDegree<float>(stage, TfToken(lowLimitStr.c_str()), usdPrim);
                    switchRadToDegree<float>(stage, TfToken(highLimitStr.c_str()), usdPrim);
                }
                renameProperty(stage, usdPrim, lowLimitStr.c_str(), lowLimitPhyStr.c_str());
                renameProperty(stage, usdPrim, highLimitStr.c_str(), highLimitPhyStr.c_str());
            }
            else if (apiSchema.GetString() == driveString + limitAxis[axis])
            {                
                removeAPI(usdPrim, TfToken(driveString + limitAxis[axis]));
                UsdPhysicsDriveAPI::Apply(usdPrim, TfToken(limitAxis[axis]));

                std::string tagetTypeStr = "drive:" + limitAxis[axis] + ":targetType";
                UsdAttribute attr = usdPrim.GetAttribute(TfToken(tagetTypeStr.c_str()));
                if (attr)
                {
                    TfToken ttype;
                    attr.Get(&ttype);
                    std::string targetStr = "drive:" + limitAxis[axis] + ":target";
                    if (ttype == TfToken("velocity"))
                    {
                        std::string targetPhyStr = "drive:" + limitAxis[axis] + ":physics:targetVelocity";
                        if (axis < 4)
                        {
                            switchRadToDegree<float>(stage, TfToken(targetStr.c_str()), usdPrim);                            
                        }
                        renameProperty(stage, usdPrim, targetStr.c_str(), targetPhyStr.c_str());
                    }
                    else
                    {
                        std::string targetPhyStr = "drive:" + limitAxis[axis] + ":physics:targetPosition";
                        if (axis < 4)
                        {
                            switchRadToDegree<float>(stage, TfToken(targetStr.c_str()), usdPrim);
                        }
                        renameProperty(stage, usdPrim, targetStr.c_str(), targetPhyStr.c_str());
                    }
                }
                
                removeProperty(stage, usdPrim, tagetTypeStr.c_str());
                std::string typeStr = "drive:" + limitAxis[axis] + ":type";
                std::string typePhyStr = "drive:" + limitAxis[axis] + ":physics:type";
                renameProperty(stage, usdPrim, typeStr.c_str(), typePhyStr.c_str());
                std::string maxForceStr = "drive:" + limitAxis[axis] + ":maxForce";
                std::string maxForcePhyStr = "drive:" + limitAxis[axis] + ":physics:maxForce";
                renameProperty(stage, usdPrim, maxForceStr.c_str(), maxForcePhyStr.c_str());
                std::string dampingForceStr = "drive:" + limitAxis[axis] + ":damping";
                std::string dampingForcePhyStr = "drive:" + limitAxis[axis] + ":physics:damping";
                if (axis < 4)
                {
                    switchDegreeToRad<float>(stage, TfToken(dampingForceStr.c_str()), usdPrim);
                }
                renameProperty(stage, usdPrim, dampingForceStr.c_str(), dampingForcePhyStr.c_str());
                std::string stiffnessForceStr = "drive:" + limitAxis[axis] + ":stiffness";
                std::string stiffnessForcePhyStr = "drive:" + limitAxis[axis] + ":physics:stiffness";
                if (axis < 4)
                {
                    switchDegreeToRad<float>(stage, TfToken(stiffnessForceStr.c_str()), usdPrim);
                }
                renameProperty(stage, usdPrim, stiffnessForceStr.c_str(), stiffnessForcePhyStr.c_str());
            }
        }

        // PhysxMeshCollisionAPI
        if (apiSchema.GetString() == "PhysxMeshCollisionAPI")
        {
            const UsdPhysicsMeshCollisionAPI approximationAPI = UsdPhysicsMeshCollisionAPI::Get(stage, usdPrim.GetPrimPath());
            if (approximationAPI)
            {
                TfToken approximation;
                approximationAPI.GetApproximationAttr().Get(&approximation);
                if (approximation == UsdPhysicsTokens->convexHull)
                {
                    renameProperty(stage, usdPrim, "physxMeshCollision:hullVertexLimit", PhysxSchemaTokens->physxConvexHullCollisionHullVertexLimit.GetText());
                    renameProperty(stage, usdPrim, "physxMeshCollision:minThickness", PhysxSchemaTokens->physxConvexHullCollisionMinThickness.GetText());
                    PhysxSchemaPhysxConvexHullCollisionAPI::Apply(usdPrim);
                }
                else if (approximation == UsdPhysicsTokens->convexDecomposition)
                {
                    renameProperty(stage, usdPrim, "physxMeshCollision:hullVertexLimit", PhysxSchemaTokens->physxConvexDecompositionCollisionHullVertexLimit.GetText());
                    renameProperty(stage, usdPrim, "physxMeshCollision:minThickness", PhysxSchemaTokens->physxConvexDecompositionCollisionMinThickness.GetText());
                    renameProperty(stage, usdPrim, "physxMeshCollision:errorPercentage", PhysxSchemaTokens->physxConvexDecompositionCollisionErrorPercentage.GetText());
                    renameProperty(stage, usdPrim, "physxMeshCollision:voxelResolution", PhysxSchemaTokens->physxConvexDecompositionCollisionVoxelResolution.GetText());
                    renameProperty(stage, usdPrim, "physxMeshCollision:maxConvexHulls", PhysxSchemaTokens->physxConvexDecompositionCollisionMaxConvexHulls.GetText());
                    PhysxSchemaPhysxConvexDecompositionCollisionAPI::Apply(usdPrim);
                }
                else if (approximation == UsdPhysicsTokens->meshSimplification)
                {
                    renameProperty(stage, usdPrim, "physxMeshCollision:simplificationMetric", PhysxSchemaTokens->physxTriangleMeshSimplificationCollisionMetric.GetText());
                    PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI::Apply(usdPrim);
                }
            }

            removeProperty(stage, usdPrim, "physxMeshCollision:errorPercentage");
            removeProperty(stage, usdPrim, "physxMeshCollision:hullVertexLimit");
            removeProperty(stage, usdPrim, "physxMeshCollision:maxConvexHulls");
            removeProperty(stage, usdPrim, "physxMeshCollision:minThickness");
            removeProperty(stage, usdPrim, "physxMeshCollision:simplificationMaxTriangleCount");
            removeProperty(stage, usdPrim, "physxMeshCollision:simplificationMetric");
            removeProperty(stage, usdPrim, "physxMeshCollision:simplificationMinTriangleCount");
            removeProperty(stage, usdPrim, "physxMeshCollision:voxelResolution");
            removeProperty(stage, usdPrim, "physxMeshCollision:meshSimplificationType ");
            removeAPI(usdPrim, TfToken("PhysxMeshCollisionAPI"));
        }

        if (apiSchema.GetString() == "PhysxCookedDataAPI")
        {
            removeProperty(stage, usdPrim, "physxCookedData:buffer");
            removeProperty(stage, usdPrim, "physxCookedData:type");
            removeAPI(usdPrim, TfToken("PhysxCookedDataAPI"));
        }

        if (apiSchema.GetString() == "CharacterControllerAPI")
        {
            renameProperty(stage, usdPrim, "slopeLimit", PhysxSchemaTokens->physxCharacterControllerSlopeLimit.GetText());
            renameProperty(stage, usdPrim, "moveTarget", PhysxSchemaTokens->physxCharacterControllerMoveTarget.GetText());
            renameProperty(stage, usdPrim, "simulationOwner", PhysxSchemaTokens->physxCharacterControllerSimulationOwner.GetText());
            PhysxSchemaPhysxCharacterControllerAPI::Apply(usdPrim);
            removeAPI(usdPrim, TfToken("CharacterControllerAPI"));
        }
    }

    // physicsScene
    if (usdPrim.IsA<UsdPhysicsScene>())
    {
        secondPassPrimTracker.scenePrim = usdPrim;

        static TfToken gravityToken("gravity");
        if (usdPrim.GetAttribute(gravityToken))
        {
            GfVec3f currentGravity;
            usdPrim.GetAttribute(gravityToken).Get(&currentGravity);
            UsdEditContext editContext(stage);
            const bool retVal = usdPrim.RemoveProperty(gravityToken);
            if (!retVal)
            {
                removeAttributeFromReference(stage, gravityToken, usdPrim);
            }

            float magn = currentGravity.GetLength();
            currentGravity.Normalize();

            UsdPhysicsScene scene(usdPrim);
            scene.GetGravityDirectionAttr().Set(currentGravity);
            scene.GetGravityMagnitudeAttr().Set(magn);
        }
        renameProperty(stage, usdPrim, "invertCollisionGroupFilter", "physxScene:invertCollisionGroupFilter");
    }
    

    if (usdPrim.HasAPI<PhysxSchemaPhysxCookedDataAPI>())
    {
        // old version cookedData to be removed
        static TfToken cookedDataType("cookedDataType");
        if (usdPrim.GetAttribute(cookedDataType))
        {
            UsdEditContext editContext(stage);
            const bool retVal = usdPrim.RemoveProperty(cookedDataType);            
            if (!retVal)
            {
                removeAttributeFromReference(stage, cookedDataType, usdPrim);
            }
        }
        static TfToken cookedData("cookedData");
        if (usdPrim.GetAttribute(cookedData))
        {
            UsdEditContext editContext(stage);
            const bool retVal = usdPrim.RemoveProperty(cookedData);
            if (!retVal)
            {
                removeAttributeFromReference(stage, cookedData, usdPrim);
            }
        }
        static TfToken physxCookedData("physxCookedData");
        if (usdPrim.GetAttribute(physxCookedData))
        {
            UsdEditContext editContext(stage);
            const bool retVal = usdPrim.RemoveProperty(physxCookedData);
            if (!retVal)
            {
                removeAttributeFromReference(stage, physxCookedData, usdPrim);
            }
        }
        static TfToken physxCookedDataType("physxCookedDataType");
        if (usdPrim.GetAttribute(physxCookedDataType))
        {
            UsdEditContext editContext(stage);
            const bool retVal = usdPrim.RemoveProperty(physxCookedDataType);
            if (!retVal)
            {
                removeAttributeFromReference(stage, physxCookedDataType, usdPrim);
            }
        }
    }

    // switch physx attributes to new convention

    // physxScene
    PhysxSchemaPhysxSceneAPI physxSceneAPI = PhysxSchemaPhysxSceneAPI::Get(stage, primPath);
    if (physxSceneAPI)
    {
        {
            static TfToken bounceThreshold("bounceThreshold");
            const VtValue val =  checkPhysxAttribute(stage, bounceThreshold, usdPrim);
            if (!val.IsEmpty())
            {
                physxSceneAPI.CreateBounceThresholdAttr(val);
            }
        }
        {
            static TfToken frictionOffsetThreshold("frictionOffsetThreshold");
            const VtValue val =  checkPhysxAttribute(stage, frictionOffsetThreshold, usdPrim);
            if (!val.IsEmpty())
            {
                physxSceneAPI.CreateFrictionOffsetThresholdAttr(val);
            }
        }

        {
            static TfToken collisionSystem("collisionSystem");
            const VtValue val =  checkPhysxAttribute(stage, collisionSystem, usdPrim);
            if (!val.IsEmpty())
            {
                physxSceneAPI.CreateCollisionSystemAttr(val);
            }
        }

        {
            static TfToken solverType("solverType");
            const VtValue val =  checkPhysxAttribute(stage, solverType, usdPrim);
            if (!val.IsEmpty())
            {
                physxSceneAPI.CreateSolverTypeAttr(val);
            }
        }

        {
            static TfToken broadphaseType("broadphaseType");
            const VtValue val =  checkPhysxAttribute(stage, broadphaseType, usdPrim);
            if (!val.IsEmpty())
            {
                physxSceneAPI.CreateBroadphaseTypeAttr(val);
            }
        }

        {
            static TfToken enableCCD("enableCCD");
            const VtValue val = checkPhysxAttribute(stage, enableCCD, usdPrim);
            if (!val.IsEmpty())
            {
                physxSceneAPI.CreateEnableCCDAttr(val);
            }
        }
        {
            static TfToken enableStabilization("enableStabilization");
            const VtValue val = checkPhysxAttribute(stage, enableStabilization, usdPrim);
            if (!val.IsEmpty())
            {
                physxSceneAPI.CreateEnableStabilizationAttr(val);
            }
        }
        {
            static TfToken enableGPUDynamics("enableGPUDynamics");
            const VtValue val = checkPhysxAttribute(stage, enableGPUDynamics, usdPrim);
            if (!val.IsEmpty())
            {
                physxSceneAPI.CreateEnableGPUDynamicsAttr(val);
            }
        }
        {
            static TfToken enableEnhancedDeterminism("enableEnhancedDeterminism");
            const VtValue val = checkPhysxAttribute(stage, enableEnhancedDeterminism, usdPrim);
            if (!val.IsEmpty())
            {
                physxSceneAPI.CreateEnableEnhancedDeterminismAttr(val);
            }
        }
        {
            static TfToken gpuTempBufferCapacity("gpuTempBufferCapacity");
            const VtValue val = checkPhysxAttribute(stage, gpuTempBufferCapacity, usdPrim);
            if (!val.IsEmpty())
            {
                physxSceneAPI.CreateGpuTempBufferCapacityAttr(val);
            }
        }
        {
            static TfToken gpuHeapCapacity("gpuHeapCapacity");
            const VtValue val = checkPhysxAttribute(stage, gpuHeapCapacity, usdPrim);
            if (!val.IsEmpty())
            {
                physxSceneAPI.CreateGpuHeapCapacityAttr(val);
            }
        }
        {
            static TfToken gpuFoundLostPairsCapacity("gpuFoundLostPairsCapacity");
            const VtValue val = checkPhysxAttribute(stage, gpuFoundLostPairsCapacity, usdPrim);
            if (!val.IsEmpty())
            {
                physxSceneAPI.CreateGpuFoundLostPairsCapacityAttr(val);
            }
        }
        {
            static TfToken gpuMaxNumPartitions("gpuMaxNumPartitions");
            const VtValue val = checkPhysxAttribute(stage, gpuMaxNumPartitions, usdPrim);
            if (!val.IsEmpty())
            {
                physxSceneAPI.CreateGpuMaxNumPartitionsAttr(val);
            }
        }
        {
            renameProperty(stage, usdPrim, "physxScene:maxIterationCount", "physxScene:maxPositionIterationCount");
            renameProperty(stage, usdPrim, "physxScene:minIterationCount", "physxScene:minPositionIterationCount");
        }
        {
            removeProperty(stage, usdPrim, gFrictionType);
            if (checkAttributeAuthoredValue(usdPrim, gPhysxSceneFrictionTypeToken))
            {
                removeProperty(stage, usdPrim, gPhysxSceneFrictionType);
            }
        }
    }

    // physxRigidBody
    PhysxSchemaPhysxRigidBodyAPI physxRigidBodyAPI = PhysxSchemaPhysxRigidBodyAPI::Get(stage, primPath);
    if (physxRigidBodyAPI)
    {
        {
            static TfToken linearDamping("linearDamping");
            const VtValue val = checkPhysxAttribute(stage, linearDamping, usdPrim);
            if (!val.IsEmpty())
            {
                physxRigidBodyAPI.CreateLinearDampingAttr(val);
            }
        }
        {
            static TfToken angularDamping("angularDamping");
            const VtValue val = checkPhysxAttribute(stage, angularDamping, usdPrim);
            if (!val.IsEmpty())
            {
                physxRigidBodyAPI.CreateAngularDampingAttr(val);
            }
        }
        {
            static TfToken maxLinearVelocity("maxLinearVelocity");
            const VtValue val = checkPhysxAttribute(stage, maxLinearVelocity, usdPrim);
            if (!val.IsEmpty())
            {
                physxRigidBodyAPI.CreateMaxLinearVelocityAttr(val);
            }
        }
        {
            static TfToken maxAngularVelocity("maxAngularVelocity");
            const VtValue val = checkPhysxAttribute(stage, maxAngularVelocity, usdPrim);
            if (!val.IsEmpty())
            {
                physxRigidBodyAPI.CreateMaxAngularVelocityAttr(val);
            }
        }
        {
            static TfToken sleepThreshold("sleepThreshold");
            const VtValue val = checkPhysxAttribute(stage, sleepThreshold, usdPrim);
            if (!val.IsEmpty())
            {
                physxRigidBodyAPI.CreateSleepThresholdAttr(val);
            }
        }
        {
            static TfToken stabilizationThreshold("stabilizationThreshold");
            const VtValue val = checkPhysxAttribute(stage, stabilizationThreshold, usdPrim);
            if (!val.IsEmpty())
            {
                physxRigidBodyAPI.CreateStabilizationThresholdAttr(val);
            }
        }
        {
            static TfToken maxDepenetrationVelocity("maxDepenetrationVelocity");
            const VtValue val = checkPhysxAttribute(stage, maxDepenetrationVelocity, usdPrim);
            if (!val.IsEmpty())
            {
                physxRigidBodyAPI.CreateMaxDepenetrationVelocityAttr(val);
            }
        }
        {
            static TfToken solverPositionIterationCount("solverPositionIterationCount");
            const VtValue val = checkPhysxAttribute(stage, solverPositionIterationCount, usdPrim);
            if (!val.IsEmpty())
            {
                physxRigidBodyAPI.CreateSolverPositionIterationCountAttr(val);
            }
        }
        {
            static TfToken solverVelocityIterationCount("solverVelocityIterationCount");
            const VtValue val = checkPhysxAttribute(stage, solverVelocityIterationCount, usdPrim);
            if (!val.IsEmpty())
            {
                physxRigidBodyAPI.CreateSolverVelocityIterationCountAttr(val);
            }
        }
        {
            static TfToken enableCCD("enableCCD");
            const VtValue val = checkPhysxAttribute(stage, enableCCD, usdPrim);
            if (!val.IsEmpty())
            {
                physxRigidBodyAPI.CreateEnableCCDAttr(val);
            }
        }
        {
            static TfToken enableSpeculativeCCD("enableSpeculativeCCD");
            const VtValue val = checkPhysxAttribute(stage, enableSpeculativeCCD, usdPrim);
            if (!val.IsEmpty())
            {
                physxRigidBodyAPI.CreateEnableSpeculativeCCDAttr(val);
            }
        }
        {
            static TfToken lockedPosAxis("lockedPosAxis");
            const VtValue val = checkPhysxAttribute(stage, lockedPosAxis, usdPrim);
            if (!val.IsEmpty())
            {
                physxRigidBodyAPI.CreateLockedPosAxisAttr(val);
            }
        }
        {
            static TfToken lockedRotAxis("lockedRotAxis");
            const VtValue val = checkPhysxAttribute(stage, lockedRotAxis, usdPrim);
            if (!val.IsEmpty())
            {
                physxRigidBodyAPI.CreateLockedRotAxisAttr(val);
            }
        }
    }

    // physxCollision
    PhysxSchemaPhysxCollisionAPI physxCollisionAPI = PhysxSchemaPhysxCollisionAPI::Get(stage, primPath);
    if (physxCollisionAPI)
    {
        {
            static TfToken contactOffset("contactOffset");
            const VtValue val = checkPhysxAttribute(stage, contactOffset, usdPrim);
            if (!val.IsEmpty())
            {
                physxCollisionAPI.CreateContactOffsetAttr(val);
            }
        }
        {
            static TfToken restOffset("restOffset");
            const VtValue val = checkPhysxAttribute(stage, restOffset, usdPrim);
            if (!val.IsEmpty())
            {
                physxCollisionAPI.CreateRestOffsetAttr(val);
            }
        }
        {
            static TfToken torsionalPatchRadius("torsionalPatchRadius");
            const VtValue val = checkPhysxAttribute(stage, torsionalPatchRadius, usdPrim);
            if (!val.IsEmpty())
            {
                physxCollisionAPI.CreateTorsionalPatchRadiusAttr(val);
            }
        }
        {
            static TfToken minTorsionalPatchRadius("minTorsionalPatchRadius");
            const VtValue val = checkPhysxAttribute(stage, minTorsionalPatchRadius, usdPrim);
            if (!val.IsEmpty())
            {
                physxCollisionAPI.CreateMinTorsionalPatchRadiusAttr(val);
            }
        }
    }

    // physxsdfcollision:resolution and trianglemesh:sdf params
    PhysxSchemaPhysxTriangleMeshCollisionAPI physxTriMeshCollisionAPI = PhysxSchemaPhysxTriangleMeshCollisionAPI::Get(stage, primPath);
    if (physxTriMeshCollisionAPI)
    {
        bool applyApiAndClean = false;

        // old custom attribute
        const char* physxsdfcollisionResolutionStr = "physxsdfcollision:resolution";
        static TfToken physxsdfcollisionResolution(physxsdfcollisionResolutionStr);
        const VtValue valRes = getAttribute(stage, physxsdfcollisionResolution, usdPrim);
        if (!valRes.IsEmpty() && valRes.Get<int>() > 0)
        {
            applyApiAndClean = true;
            renameProperty(stage, usdPrim, physxsdfcollisionResolutionStr,
                PhysxSchemaTokens->physxSDFMeshCollisionSdfResolution.GetText());
        }
        else
        {
            removeProperty(stage, usdPrim, physxsdfcollisionResolutionStr);
        }        

        // attributes that used to be part of the TriangleMeshCollisionAPI
        static const char arrLen = 5;
        static std::array<const char*, arrLen> triMeshStrs = {
            "physxTriangleMeshCollision:sdfResolution",
            "physxTriangleMeshCollision:sdfBitsPerSubgridPixel",
            "physxTriangleMeshCollision:sdfMargin",
            "physxTriangleMeshCollision:sdfNarrowBandThickness",
            "physxTriangleMeshCollision:sdfSubgridResolution",
        };

        static std::array<const char*, arrLen> sdfStrs = {
            PhysxSchemaTokens->physxSDFMeshCollisionSdfResolution.GetText(),
            PhysxSchemaTokens->physxSDFMeshCollisionSdfBitsPerSubgridPixel.GetText(),
            PhysxSchemaTokens->physxSDFMeshCollisionSdfMargin.GetText(),
            PhysxSchemaTokens->physxSDFMeshCollisionSdfNarrowBandThickness.GetText(),
            PhysxSchemaTokens->physxSDFMeshCollisionSdfSubgridResolution.GetText(),
        };

        static TfToken physxTriangleMeshCollisionSdfResolution(triMeshStrs[0]);

        const VtValue valTMRes = getAttribute(stage, physxTriangleMeshCollisionSdfResolution, usdPrim);
        if (!valTMRes.IsEmpty() && valTMRes.Get<int>() > 0)
        {
            applyApiAndClean = true;

            for (char i = 0; i < arrLen; ++i)
            {
                renameProperty(stage, usdPrim, triMeshStrs[i], sdfStrs[i]);
            }
            
        }
        else
        {
            for (char i = 0; i < arrLen; ++i)
            {
                removeProperty(stage, usdPrim, triMeshStrs[i]);
            }
        }

        if (applyApiAndClean)
        {
            PhysxSchemaPhysxSDFMeshCollisionAPI::Apply(usdPrim);            
            removeAPI(usdPrim, TfToken("PhysxTriangleMeshCollisionAPI"));
            
            UsdPhysicsMeshCollisionAPI physxMeshCollisionAPI = UsdPhysicsMeshCollisionAPI::Get(stage, primPath);
            if (physxMeshCollisionAPI)
            {
                //The following line sets the collider type to sdf. So enabling sdfs with sdfResolution>0 is not required anymore
                physxMeshCollisionAPI.CreateApproximationAttr().Set(TfToken("sdf"));
            }            
        }
    }

    // physxMaterial
    PhysxSchemaPhysxMaterialAPI physxMaterialAPI = PhysxSchemaPhysxMaterialAPI::Get(stage, primPath);
    if (physxMaterialAPI)
    {
        {
            static TfToken frictionCombineMode("frictionCombineMode");
            const VtValue val = checkPhysxAttribute(stage, frictionCombineMode, usdPrim);
            if (!val.IsEmpty())
            {
                physxMaterialAPI.CreateFrictionCombineModeAttr(val);
            }
        }
        {
            static TfToken restitutionCombineMode("restitutionCombineMode");
            const VtValue val = checkPhysxAttribute(stage, restitutionCombineMode, usdPrim);
            if (!val.IsEmpty())
            {
                physxMaterialAPI.CreateRestitutionCombineModeAttr(val);
            }
        }
        {
            static TfToken dampingCombineMode("dampingCombineMode");
            const VtValue val = checkPhysxAttribute(stage, dampingCombineMode, usdPrim);
            if (!val.IsEmpty())
            {
                physxMaterialAPI.CreateDampingCombineModeAttr(val);
            }
        }
    }

    // physxJoint
    PhysxSchemaPhysxJointAPI physxJointAPI = PhysxSchemaPhysxJointAPI::Get(stage, primPath);
    if (physxJointAPI)
    {
        {
            renameProperty(stage, usdPrim, "enableCollision", "physics:collisionEnabled");
            renameProperty(stage, usdPrim, "physxJoint:enableCollision", "physics:collisionEnabled");
        }
        {
            removeProperty(stage, usdPrim, gEnableProjection);
            if (checkAttributeAuthoredValue(usdPrim, gPhysxJointEnableProjectionToken))
            {
                removeProperty(stage, usdPrim, gPhysxJointEnableProjection);
            }
        }
    }


    // physxArticulaton
    PhysxSchemaPhysxArticulationAPI physxArticulationAPI = PhysxSchemaPhysxArticulationAPI::Get(stage, primPath);
    if (physxArticulationAPI)
    {
        {
            static TfToken solverPositionIterationCount("solverPositionIterationCount");
            const VtValue val = checkPhysxAttribute(stage, solverPositionIterationCount, usdPrim);
            if (!val.IsEmpty())
            {
                physxArticulationAPI.CreateSolverPositionIterationCountAttr(val);
            }
        }
        {
            static TfToken solverVelocityIterationCount("solverVelocityIterationCount");
            const VtValue val = checkPhysxAttribute(stage, solverVelocityIterationCount, usdPrim);
            if (!val.IsEmpty())
            {
                physxArticulationAPI.CreateSolverVelocityIterationCountAttr(val);
            }
        }
        {
            static TfToken sleepThreshold("sleepThreshold");
            const VtValue val = checkPhysxAttribute(stage, sleepThreshold, usdPrim);
            if (!val.IsEmpty())
            {
                physxArticulationAPI.CreateSleepThresholdAttr(val);
            }
        }
        {
            static TfToken stabilizationThreshold("stabilizationThreshold");
            const VtValue val = checkPhysxAttribute(stage, stabilizationThreshold, usdPrim);
            if (!val.IsEmpty())
            {
                physxArticulationAPI.CreateStabilizationThresholdAttr(val);
            }
        }
    }

    // physxArticulationJoint
    {        
        renameProperty(stage, usdPrim, "frictionCoefficient", "physxJoint:jointFriction");
        renameProperty(stage, usdPrim, "physxArticulationJoint:frictionCoefficient", "physxJoint:jointFriction");
        if (usdPrim.IsA<UsdPhysicsRevoluteJoint>() || usdPrim.IsA<UsdPhysicsSphericalJoint>())
        {
            switchRadToDegree<float>(stage, TfToken("physxArticulationJoint:maxJointVelocity"), usdPrim);
            switchRadToDegree<float>(stage, TfToken("maxJointVelocity"), usdPrim);
        }
        renameProperty(stage, usdPrim, "physxArticulationJoint:maxJointVelocity", "physxJoint:maxJointVelocity");
        renameProperty(stage, usdPrim, "maxJointVelocity", "physxJoint:maxJointVelocity");
    }

    // physxTrigger
    PhysxSchemaPhysxTriggerAPI physxTriggerAPI = PhysxSchemaPhysxTriggerAPI::Get(stage, primPath);
    if (physxTriggerAPI)
    {
        {
            static TfToken onEnterScript("onEnterScript");
            const VtValue val = checkPhysxAttribute(stage, onEnterScript, usdPrim);
            if (!val.IsEmpty())
            {
                physxTriggerAPI.CreateOnEnterScriptAttr(val);
            }
        }
        {
            static TfToken onLeaveScript("onLeaveScript");
            const VtValue val = checkPhysxAttribute(stage, onLeaveScript, usdPrim);
            if (!val.IsEmpty())
            {
                physxTriggerAPI.CreateOnLeaveScriptAttr(val);
            }
        }
    }

    // PhysxVehicleGlobalSettings -> PhysxVehicleContextAPI
    static TfToken oldGlobalSettingsPrim("PhysxVehicleGlobalSettings");
    if (usdPrim.GetTypeName() == oldGlobalSettingsPrim)
    {
        secondPassPrimTracker.oldVehicleGlobalSettingsPrim = usdPrim;
    }

    // PhysxVehicleContextAPI
    if (usdPrim.HasAPI<PhysxSchemaPhysxVehicleContextAPI>())
    {
        static const TfToken sweepRadiusScaleParam("physxVehicleContext:sweepRadiusScale");
        static const TfToken sweepWidthScaleParam("physxVehicleContext:sweepWidthScale");
        const VtValue valRadiusScale = checkPhysxAttribute(stage, sweepRadiusScaleParam, usdPrim);
        const VtValue valWidthScale = checkPhysxAttribute(stage, sweepWidthScaleParam, usdPrim);
        if (!(valRadiusScale.IsEmpty() && valWidthScale.IsEmpty()))
        {
            CARB_LOG_WARN("Physics backwardsCompatibility: PhysxVehicleContextAPI does not support the \"sweepRadiusScale\" "
                "and \"sweepWidthScale\" attributes any longer. Removing the attributes from prim %s.\n",
                usdPrim.GetPath().GetText());
        }
    }

    // PhysxVehicleTire -> PhysxVehicleTireAPI
    static TfToken oldTirePrim("PhysxVehicleTire");
    static TfToken tirePrim("");  // default prim has no type name
    if (usdPrim.GetTypeName() == oldTirePrim)
    {
        CARB_LOG_WARN("Physics backwardsCompatibility: %s: PhysxVehicleTire was changed to an API schema PhysxVehicleTireAPI.\n",
            usdPrim.GetPath().GetText());

        UsdEditContext editContext(stage);
        setTypeName(stage, tirePrim, usdPrim);
        PhysxSchemaPhysxVehicleTireAPI::Apply(usdPrim);

        renameProperty(stage, usdPrim, "latStiffX", PhysxSchemaTokens.Get()->physxVehicleTireLatStiffX.GetText());
        renameProperty(stage, usdPrim, "latStiffY", PhysxSchemaTokens.Get()->physxVehicleTireLatStiffY.GetText());
        renameProperty(stage, usdPrim, "longitudinalStiffnessPerUnitGravity", PhysxSchemaTokens.Get()->physxVehicleTireLongitudinalStiffnessPerUnitGravity.GetText());
        renameProperty(stage, usdPrim, "camberStiffnessPerUnitGravity", PhysxSchemaTokens.Get()->physxVehicleTireCamberStiffnessPerUnitGravity.GetText());
        renameProperty(stage, usdPrim, "frictionVsSlipGraph", PhysxSchemaTokens.Get()->physxVehicleTireFrictionVsSlipGraph.GetText());
        renameProperty(stage, usdPrim, "frictionTable", PhysxSchemaTokens.Get()->physxVehicleTireFrictionTable.GetText());
    }

    // PhysxVehicleSuspension -> PhysxVehicleSuspensionAPI
    static TfToken oldSuspensionPrim("PhysxVehicleSuspension");
    static TfToken suspensionPrim("");  // default prim has no type name
    if (usdPrim.GetTypeName() == oldSuspensionPrim)
    {
        CARB_LOG_WARN("Physics backwardsCompatibility: %s: PhysxVehicleSuspension was changed to an API schema PhysxVehicleSuspensionAPI.\n",
            usdPrim.GetPath().GetText());

        UsdEditContext editContext(stage);
        setTypeName(stage, suspensionPrim, usdPrim);
        PhysxSchemaPhysxVehicleSuspensionAPI::Apply(usdPrim);

        renameProperty(stage, usdPrim, "springStrength", PhysxSchemaTokens.Get()->physxVehicleSuspensionSpringStrength.GetText());
        renameProperty(stage, usdPrim, "springDamperRate", PhysxSchemaTokens.Get()->physxVehicleSuspensionSpringDamperRate.GetText());
        renameProperty(stage, usdPrim, "maxCompression", PhysxSchemaTokens.Get()->physxVehicleSuspensionMaxCompression.GetText());
        renameProperty(stage, usdPrim, "maxDroop", PhysxSchemaTokens.Get()->physxVehicleSuspensionMaxDroop.GetText());
        renameProperty(stage, usdPrim, "camberAtRest", PhysxSchemaTokens.Get()->physxVehicleSuspensionCamberAtRest.GetText());
        renameProperty(stage, usdPrim, "camberAtMaxCompression", PhysxSchemaTokens.Get()->physxVehicleSuspensionCamberAtMaxCompression.GetText());
        renameProperty(stage, usdPrim, "camberAtMaxDroop", PhysxSchemaTokens.Get()->physxVehicleSuspensionCamberAtMaxDroop.GetText());
        renameProperty(stage, usdPrim, "sprungMass", PhysxSchemaTokens.Get()->physxVehicleSuspensionSprungMass.GetText());
    }

    // PhysxVehicleWheel -> PhysxVehicleWheelAPI
    static TfToken oldWheelPrim("PhysxVehicleWheel");
    static TfToken wheelPrim("");  // default prim has no type name
    if (usdPrim.GetTypeName() == oldWheelPrim)
    {
        CARB_LOG_WARN("Physics backwardsCompatibility: %s: PhysxVehicleWheel was changed to an API schema PhysxVehicleWheelAPI.\n",
            usdPrim.GetPath().GetText());

        UsdEditContext editContext(stage);
        setTypeName(stage, wheelPrim, usdPrim);
        PhysxSchemaPhysxVehicleWheelAPI::Apply(usdPrim);

        renameProperty(stage, usdPrim, "radius", PhysxSchemaTokens.Get()->physxVehicleWheelRadius.GetText());
        renameProperty(stage, usdPrim, "width", PhysxSchemaTokens.Get()->physxVehicleWheelWidth.GetText());
        renameProperty(stage, usdPrim, "mass", PhysxSchemaTokens.Get()->physxVehicleWheelMass.GetText());
        renameProperty(stage, usdPrim, "moi", PhysxSchemaTokens.Get()->physxVehicleWheelMoi.GetText());
        renameProperty(stage, usdPrim, "dampingRate", PhysxSchemaTokens.Get()->physxVehicleWheelDampingRate.GetText());
        renameProperty(stage, usdPrim, "maxBrakeTorque", PhysxSchemaTokens.Get()->physxVehicleWheelMaxBrakeTorque.GetText());
        renameProperty(stage, usdPrim, "maxHandBrakeTorque", PhysxSchemaTokens.Get()->physxVehicleWheelMaxHandBrakeTorque.GetText());
        renameProperty(stage, usdPrim, "maxSteerAngle", PhysxSchemaTokens.Get()->physxVehicleWheelMaxSteerAngle.GetText());
        renameProperty(stage, usdPrim, "toeAngle", PhysxSchemaTokens.Get()->physxVehicleWheelToeAngle.GetText());
    }

    // PhysxVehicleEngine -> PhysxVehicleEngineAPI
    static TfToken oldEnginePrim("PhysxVehicleEngine");
    static TfToken enginePrim("");  // default prim has no type name
    if (usdPrim.GetTypeName() == oldEnginePrim)
    {
        CARB_LOG_WARN("Physics backwardsCompatibility: %s: PhysxVehicleEngine was changed to an API schema PhysxVehicleEngineAPI.\n",
            usdPrim.GetPath().GetText());

        UsdEditContext editContext(stage);
        setTypeName(stage, enginePrim, usdPrim);
        PhysxSchemaPhysxVehicleEngineAPI::Apply(usdPrim);

        renameProperty(stage, usdPrim, "moi", PhysxSchemaTokens.Get()->physxVehicleEngineMoi.GetText());
        renameProperty(stage, usdPrim, "peakTorque", PhysxSchemaTokens.Get()->physxVehicleEnginePeakTorque.GetText());
        renameProperty(stage, usdPrim, "maxRotationSpeed", PhysxSchemaTokens.Get()->physxVehicleEngineMaxRotationSpeed.GetText());
        renameProperty(stage, usdPrim, "torqueCurve", PhysxSchemaTokens.Get()->physxVehicleEngineTorqueCurve.GetText());
        renameProperty(stage, usdPrim, "dampingRateFullThrottle", PhysxSchemaTokens.Get()->physxVehicleEngineDampingRateFullThrottle.GetText());
        renameProperty(stage, usdPrim, "dampingRateZeroThrottleClutchEngaged", PhysxSchemaTokens.Get()->physxVehicleEngineDampingRateZeroThrottleClutchEngaged.GetText());
        renameProperty(stage, usdPrim, "dampingRateZeroThrottleClutchDisengaged", PhysxSchemaTokens.Get()->physxVehicleEngineDampingRateZeroThrottleClutchDisengaged.GetText());
    }

    // PhysxVehicleGears -> PhysxVehicleGearsAPI
    static TfToken oldGearsPrim("PhysxVehicleGears");
    static TfToken gearsPrim("");  // default prim has no type name
    if (usdPrim.GetTypeName() == oldGearsPrim)
    {
        CARB_LOG_WARN("Physics backwardsCompatibility: %s: PhysxVehicleGears was changed to an API schema PhysxVehicleGearsAPI.\n",
            usdPrim.GetPath().GetText());

        UsdEditContext editContext(stage);
        setTypeName(stage, gearsPrim, usdPrim);
        PhysxSchemaPhysxVehicleGearsAPI::Apply(usdPrim);

        renameProperty(stage, usdPrim, "ratios", PhysxSchemaTokens.Get()->physxVehicleGearsRatios.GetText());
        renameProperty(stage, usdPrim, "ratioScale", PhysxSchemaTokens.Get()->physxVehicleGearsRatioScale.GetText());
        renameProperty(stage, usdPrim, "switchTime", PhysxSchemaTokens.Get()->physxVehicleGearsSwitchTime.GetText());
    }

    // PhysxVehicleAutoGearBox -> PhysxVehicleAutoGearBoxAPI
    static TfToken oldautoGearBoxPrim("PhysxVehicleAutoGearBox");
    static TfToken autoGearBoxPrim("");  // default prim has no type name
    if (usdPrim.GetTypeName() == oldautoGearBoxPrim)
    {
        CARB_LOG_WARN("Physics backwardsCompatibility: %s: PhysxVehicleAutoGearBox was changed to an API schema PhysxVehicleAutoGearBoxAPI.\n",
            usdPrim.GetPath().GetText());

        UsdEditContext editContext(stage);
        setTypeName(stage, autoGearBoxPrim, usdPrim);
        PhysxSchemaPhysxVehicleAutoGearBoxAPI::Apply(usdPrim);

        renameProperty(stage, usdPrim, "upRatios", PhysxSchemaTokens.Get()->physxVehicleAutoGearBoxUpRatios.GetText());
        renameProperty(stage, usdPrim, "downRatios", PhysxSchemaTokens.Get()->physxVehicleAutoGearBoxDownRatios.GetText());
        renameProperty(stage, usdPrim, "latency", PhysxSchemaTokens.Get()->physxVehicleAutoGearBoxLatency.GetText());
    }

    // PhysxVehicleClutch -> PhysxVehicleClutchAPI
    static TfToken oldClutchPrim("PhysxVehicleClutch");
    static TfToken clutchPrim("");  // default prim has no type name
    if (usdPrim.GetTypeName() == oldClutchPrim)
    {
        CARB_LOG_WARN("Physics backwardsCompatibility: %s: PhysxVehicleClutch was changed to an API schema PhysxVehicleClutchAPI.\n",
            usdPrim.GetPath().GetText());

        UsdEditContext editContext(stage);
        setTypeName(stage, clutchPrim, usdPrim);
        PhysxSchemaPhysxVehicleClutchAPI::Apply(usdPrim);

        renameProperty(stage, usdPrim, "strength", PhysxSchemaTokens.Get()->physxVehicleClutchStrength.GetText());
    }

    // PhysxVehicleDriveBasic -> PhysxVehicleDriveBasicAPI
    static TfToken oldDriveBasicPrim("PhysxVehicleDriveBasic");
    static TfToken driveBasicPrim("");  // default prim has no type name
    if (usdPrim.GetTypeName() == oldDriveBasicPrim)
    {
        CARB_LOG_WARN("Physics backwardsCompatibility: %s: PhysxVehicleDriveBasic was changed to an API schema PhysxVehicleDriveBasicAPI.\n",
            usdPrim.GetPath().GetText());

        UsdEditContext editContext(stage);
        setTypeName(stage, driveBasicPrim, usdPrim);
        PhysxSchemaPhysxVehicleDriveBasicAPI::Apply(usdPrim);

        renameProperty(stage, usdPrim, "peakTorque", PhysxSchemaTokens.Get()->physxVehicleDriveBasicPeakTorque.GetText());
    }

    // PhysxVehicleDriveStandard -> PhysxVehicleDriveStandardAPI
    static TfToken oldDriveStandardPrim("PhysxVehicleDriveStandard");
    static TfToken driveStandardPrim("");  // default prim has no type name
    if (usdPrim.GetTypeName() == oldDriveStandardPrim)
    {
        CARB_LOG_WARN("Physics backwardsCompatibility: %s: PhysxVehicleDriveStandard was changed to an API schema PhysxVehicleDriveStandardAPI.\n",
            usdPrim.GetPath().GetText());

        UsdEditContext editContext(stage);
        setTypeName(stage, driveStandardPrim, usdPrim);
        PhysxSchemaPhysxVehicleDriveStandardAPI::Apply(usdPrim);

        renameProperty(stage, usdPrim, "engine", PhysxSchemaTokens.Get()->physxVehicleDriveStandardEngine.GetText());
        renameProperty(stage, usdPrim, "gears", PhysxSchemaTokens.Get()->physxVehicleDriveStandardGears.GetText());
        renameProperty(stage, usdPrim, "autoGearBox", PhysxSchemaTokens.Get()->physxVehicleDriveStandardAutoGearBox.GetText());
        renameProperty(stage, usdPrim, "clutch", PhysxSchemaTokens.Get()->physxVehicleDriveStandardClutch.GetText());
    }

    // PhysxVehicleController
    if (usdPrim.HasAPI<PhysxSchemaPhysxVehicleControllerAPI>())
    {
        static TfToken shiftUpParam("physxVehicleController:shiftUp");
        static TfToken shiftDownParam("physxVehicleController:shiftDown");
        const VtValue valUp = checkPhysxAttribute(stage, shiftUpParam, usdPrim);
        const VtValue valDown = checkPhysxAttribute(stage, shiftDownParam, usdPrim);
        if (!(valUp.IsEmpty() && valDown.IsEmpty()))
        {
            CARB_LOG_WARN("Physics backwardsCompatibility: PhysxVehicleControllerAPI does not support the \"shiftUp\" "
                "and \"shiftDown\" attributes any longer. Please use \"targetGear\" instead. "
                "Removing the attributes from prim %s.\n", usdPrim.GetPath().GetText());
        }

        static TfToken automaticParam("physxVehicleController:automatic");
        const VtValue valAutomatic = checkPhysxAttribute(stage, automaticParam, usdPrim);
        if (!valAutomatic.IsEmpty())
        {
            CARB_LOG_WARN("Physics backwardsCompatibility: PhysxVehicleControllerAPI does not support the \"automatic\" "
                "attribute any longer. Please use \"targetGear\" with special value %d instead. "
                "Removing the attribute from prim %s.\n", usdparser::VehicleControllerDesc::automaticGearValue, usdPrim.GetPath().GetText());
        }
    }

    if (usdPrim.HasAPI<UsdPhysicsCollisionAPI>())
    {
        static TfToken customGeometryAttribute("physxCollisionCustomGeometry");
        const VtValue val = checkPhysxAttribute(stage, customGeometryAttribute, usdPrim);
    }
}

void processSecondPassPrims(pxr::UsdStageWeakPtr stage, SecondPassPrimTracker& secondPassPrimTracker)
{
    CARB_PROFILE_ZONE(0, "USD Load processSecondPassPrims");

    if (secondPassPrimTracker.oldVehicleGlobalSettingsPrim.IsValid())
    {
        pxr::UsdPrim& settingsPrim = secondPassPrimTracker.oldVehicleGlobalSettingsPrim;

        CARB_LOG_WARN("Physics backwardsCompatibility: %s: PhysxVehicleGlobalSettings was changed to an API schema PhysxVehicleContextAPI.\n",
            settingsPrim.GetPath().GetText());

        pxr::UsdPrim& scenePrim = secondPassPrimTracker.scenePrim;
        if (scenePrim.IsValid())
        {
            if (!scenePrim.HasAPI<PhysxSchemaPhysxVehicleContextAPI>())
            {
                UsdEditContext editContext(stage);
                PhysxSchemaPhysxVehicleContextAPI contextAPI = PhysxSchemaPhysxVehicleContextAPI::Apply(scenePrim);

                static TfToken velocityChangeValue("velocityChange");
                static TfToken updateModeParam("updateMode");
                static TfToken upAxisParam("upAxis");
                static TfToken forwardAxisParam("forwardAxis");
                
                pxr::VtValue val;
                settingsPrim.GetAttribute(updateModeParam).Get(&val);
                if (val == velocityChangeValue)
                    contextAPI.CreateUpdateModeAttr().Set(PhysxSchemaTokens.Get()->velocityChange);
                else
                    contextAPI.CreateUpdateModeAttr().Set(PhysxSchemaTokens.Get()->acceleration);

                pxr::GfVec3f valVec3;
                settingsPrim.GetAttribute(upAxisParam).Get(&valVec3);
                contextAPI.CreateUpAxisAttr().Set(valVec3);

                settingsPrim.GetAttribute(forwardAxisParam).Get(&valVec3);
                contextAPI.CreateForwardAxisAttr().Set(valVec3);

                stage->RemovePrim(secondPassPrimTracker.oldVehicleGlobalSettingsPrim.GetPrimPath());

                CARB_LOG_WARN(
                    "Physics backwardsCompatibility: %s: the properties have been moved to PhysxVehicleContextAPI which was "
                    "applied to %s. PhysxVehicleGlobalSettings prim was removed.\n",
                    settingsPrim.GetPath().GetText(), scenePrim.GetPath().GetText());
            }
            else
            {
                CARB_LOG_WARN(
                    "Physics backwardsCompatibility: %s: the properties from a PhysxVehicleGlobalSettings could not be moved to %s, since "
                    "it already has a PhysxVehicleContextAPI applied. Please remove the PhysxVehicleGlobalSettings prim manually or unapply "
                    "PhysxVehicleContextAPI and re-run this check.\n",
                    settingsPrim.GetPath().GetText(), scenePrim.GetPath().GetText());
            }
        }
        else
        {
            CARB_LOG_WARN("Physics backwardsCompatibility: %s: PhysxVehicleContextAPI needs to be applied to a "
                "PhysicsScene prim. There is no such prim. Issue can not be resolved automatically.\n",
                settingsPrim.GetPath().GetText());

            // not much we can do. Just adding a scene seems a bit dangerous in case the USD file is referenced by
            // another one. Just changing the prim to have the API applied instead.

            static TfToken contextPrim("");  // default prim has no type name

            UsdEditContext editContext(stage);
            setTypeName(stage, contextPrim, settingsPrim);
            PhysxSchemaPhysxVehicleContextAPI::Apply(settingsPrim);

            renameProperty(stage, settingsPrim, "updateMode", PhysxSchemaTokens.Get()->physxVehicleContextUpdateMode.GetText());
            renameProperty(stage, settingsPrim, "upAxis", PhysxSchemaTokens.Get()->physxVehicleContextUpAxis.GetText());
            renameProperty(stage, settingsPrim, "forwardAxis", PhysxSchemaTokens.Get()->physxVehicleContextForwardAxis.GetText());
        }
    }
}

static pxr::TfToken customLayerDataToken("customLayerData");
static pxr::TfToken coneToken("physicsSettings:/physics/collisionConeCustomGeometrySetting");
static pxr::TfToken cylinderToken("physicsSettings:/physics/collisionCylinderCustomGeometry");

void checkMetadata(bool& ret, pxr::UsdStageWeakPtr stage)
{
    bool collisionConeCustomGeometry;
    if (stage->GetMetadataByDictKey(customLayerDataToken, coneToken, &collisionConeCustomGeometry))
    {
        gCheckLog += "Found deprecated per-stage setting /physics/collisionConeCustomGeometry\n";
        ret = true;
    }

    bool collisionCylinderCustomGeometry;
    if(stage->GetMetadataByDictKey(customLayerDataToken, cylinderToken, &collisionCylinderCustomGeometry))
    {
        gCheckLog += "Found deprecated per-stage setting /physics/collisionCylinderCustomGeometry\n";
        ret = true;
    }
}

void fixMetadata(pxr::UsdStageWeakPtr stage)
{
    bool collisionConeCustomGeometrySetting = false;
    if (stage->GetMetadataByDictKey(customLayerDataToken, coneToken, &collisionConeCustomGeometrySetting))
    {
        carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();

        // transform the old setting
        settings->set(kSettingCollisionApproximateCones, !collisionConeCustomGeometrySetting);

        stage->ClearMetadataByDictKey(customLayerDataToken, coneToken);
    }

    bool collisionCylinderCustomGeometry = false;
    if (stage->GetMetadataByDictKey(customLayerDataToken, cylinderToken, &collisionCylinderCustomGeometry))
    {
        carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();

        // transform the old setting
        settings->set(kSettingCollisionApproximateCylinders, !collisionCylinderCustomGeometry);

        stage->ClearMetadataByDictKey(customLayerDataToken, cylinderToken);
    }
}

void runBackwardCompatibilityOnStage(pxr::UsdStageWeakPtr stage)
{
    CARB_PROFILE_ZONE(0, "USD Load runBackwardCompatibilityOnStage");

    // UsdLoad::getUsdLoad()->blockUSDUpdate(true);

    fixMetadata(stage);

    pxr::UsdPrimRange range = stage->TraverseAll();
    SecondPassPrimTracker secondPassPrimTracker;
    for (pxr::UsdPrim prim : range)
    {
        if (!prim)
            continue;
        checkPrim(stage, prim, secondPassPrimTracker);
    }

    processSecondPassPrims(stage, secondPassPrimTracker);

    // UsdLoad::getUsdLoad()->blockUSDUpdate(false);
}

bool checkBackwardCompatibilityOnStage(pxr::UsdStageWeakPtr stage)
{
    gCheckLog.clear();
    bool ret = false;

    PXR_NS::UsdStageCache& cache = PXR_NS::UsdUtilsStageCache::Get();
    omni::fabric::UsdStageId stageId = { static_cast<uint64_t>(cache.GetId(stage).ToLongInt()) };
    omni::fabric::IStageReaderWriter* iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(stageId);
    usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(stageId, stageInProgress);

    checkMetadata(ret, stage);
    checkJoints(ret, stage, usdrtStage);
    checkSchemas(ret, stage, usdrtStage);
    checkPhysicsMaterialAPI(ret, stage, usdrtStage);
    checkScenes(ret, stage, usdrtStage);
    checkPhysxCookedDataAPI(ret, stage, usdrtStage);
    checkPhysxSchemaPhysxSceneAPI(ret, stage, usdrtStage);
    checkPhysxSchemaPhysxRigidBodyAPI(ret, stage, usdrtStage);
    checkPhysxSchemaPhysxTriangleMeshCollisionAPI(ret, stage, usdrtStage);
    checkPhysxSchemaPhysxCollisionAPI(ret, stage, usdrtStage);
    checkPhysxSchemaPhysxMaterialAPI(ret, stage, usdrtStage);
    checkPhysxSchemaPhysxJointAPI(ret, stage, usdrtStage);
    checkPhysxSchemaPhysxArticulationAPI(ret, stage, usdrtStage);
    checkPhysxSchemaPhysxArticulationForceSensorAPI(ret, stage, usdrtStage);
    checkPhysxArticulationJoint(ret, stage, usdrtStage);
    checkUsdPhysicsRevoluteJoint(ret, stage, usdrtStage);
    checkUsdPhysicsSphericalJoint(ret, stage, usdrtStage);
    checkPhysxSchemaPhysxTriggerAPI(ret, stage, usdrtStage);
    checkPhysxVehicle(ret, stage, usdrtStage);
    checkPhysxSchemaPhysxVehicleContextAPI(ret, stage, usdrtStage);
    checkPhysxSchemaPhysxVehicleControllerAPI(ret, stage, usdrtStage);
    checkPhysxSchemaUsdPhysicsCollisionAPI(ret, stage, usdrtStage);

    return ret;
}

const char* getBackwardsCompatibilityCheckLog()
{
    return gCheckLog.c_str();
}

} // namespace physx
} // namespace omni
