// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "OmniPvdUsdWriter.h"
#include "PvdDomUtils.h"
#include "OmniPvdUsdAdapter.h"

#include "OmniPvdSdfUtils.h"
#include "OmniPvdUsdUtils.h"

#include <carb/logging/Log.h>

#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <omni/usd/Selection.h>

////////////////////////////////////////////////////////////////////////////////
// The fundamental trick here is not to set the display colour below the PxActor
// so that any objects below the PxActor inherit the color from the PxActor
////////////////////////////////////////////////////////////////////////////////
// If the objects are not having a PxActor in their hierarchy? Tough luck.
// I guess set the /scenes or /scene to a certain colour to override below if
// needed.
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Changes the colouring of the OmniPVD object if it's a PxActor
////////////////////////////////////////////////////////////////////////////////
// if omniPvdObject->mActortype == PxActorType::eRIGID_STATIC
//   pastel gray
//
// if omniPvdObject->mActortype == PxActorType::eRIGID_DYNAMIC (1)
//   if not kinematic
//     non-sleeping > light pastel green
//     sleeping -> dark pastel green
//   else kinematic
//     non-sleeping > light pastel blue
//     sleeping -> dark pastel blue
//
// if omniPvdObject->mOmniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxArticulation
//   sleeping -> dark orange
//   non-sleeping > light orange
//
////////////////////////////////////////////////////////////////////////////////

// Get the next state as long as we can
bool getNextState(OmniPvdAttributeSample*& attribSleep, OmniPvdAttributeSample*& attribKinematic, bool& currentSleeping, bool& currentKinematic, uint64_t& minUpdatedTimeStamp) {
    if ((attribSleep == 0) && (attribKinematic == 0)) {
        return false;
    }
    OmniPvdAttributeSample* nextSleepNode = 0;
    OmniPvdAttributeSample* nextKinematicNode = 0;
    if (attribSleep) {
        nextSleepNode = (OmniPvdAttributeSample*)(attribSleep->mNextAttribute);
    }
    if (attribKinematic) {
        nextKinematicNode = (OmniPvdAttributeSample*)(attribKinematic->mNextAttribute);
    }
    // Only the next nodes with smallest time stamps are set as the next current nodes
    // The exception is if both next nodes have the same timestamp, then both get updated to be the next current nodes
    if (nextSleepNode && nextKinematicNode) {
        if (nextSleepNode->mTimeStamp < nextKinematicNode->mTimeStamp) {
            attribSleep = nextSleepNode;
            minUpdatedTimeStamp = nextSleepNode->mTimeStamp;
        } else if (nextSleepNode->mTimeStamp > nextKinematicNode->mTimeStamp) {
            attribKinematic = nextKinematicNode;
            minUpdatedTimeStamp = nextKinematicNode->mTimeStamp;
        } else {
            // same time stamp so update both
            attribSleep = nextSleepNode;
            attribKinematic = nextKinematicNode;
            minUpdatedTimeStamp = nextSleepNode->mTimeStamp;
        }
    } else if (nextSleepNode || nextKinematicNode) {
        if (attribSleep) {
            attribSleep = nextSleepNode;
            if (nextSleepNode) {
                minUpdatedTimeStamp = nextSleepNode->mTimeStamp;
            }
        }
        if (attribKinematic) {
            attribKinematic = nextKinematicNode;
            if (nextKinematicNode) {
                minUpdatedTimeStamp = nextKinematicNode->mTimeStamp;
            }            
        }        
    } else {
        return false;
    }
    // Now we can extract the values
    if (attribSleep) {
        currentSleeping = *((uint8_t*)attribSleep->mData);
    }
    if (attribKinematic) {
        currentKinematic = (*((uint32_t*)attribKinematic->mData)) & 1; // extract the first bit, the kinematic or not flag
    }
    return true;
}

void setColourOfRigidDynamicBody(
    PXR_NS::UsdAttribute& colAttr,
    double minUpdatedTimeStamp,
    bool kinematic,
    bool sleeping,
    PXR_NS::VtArray<PXR_NS::GfVec3f>& colourArrayVtDarkGreen,
    PXR_NS::VtArray<PXR_NS::GfVec3f>& colourArrayVtLightGreen,
    PXR_NS::VtArray<PXR_NS::GfVec3f>& colourArrayVtDarkBlue,
    PXR_NS::VtArray<PXR_NS::GfVec3f>& colourArrayVtLightBlue
    )
{
    if (!kinematic)
    {
        if (sleeping)
        {
            colAttr.Set(colourArrayVtDarkGreen, (double)minUpdatedTimeStamp);
        }
        else
        {
            colAttr.Set(colourArrayVtLightGreen, (double)minUpdatedTimeStamp);
        }
    }
    else
    {
        if (sleeping)
        {
            colAttr.Set(colourArrayVtDarkBlue, (double)minUpdatedTimeStamp);
        }
        else
        {
            colAttr.Set(colourArrayVtLightBlue, (double)minUpdatedTimeStamp);

        }
    }
}

void processActivityColourSeries(PXR_NS::UsdPrim* prim, OmniPvdObject* omniPvdObject)
{
    if (!omniPvdObject) return;
    if (!omniPvdObject->mOmniPvdClass) return;

    ////////////////////////////////////////////////////////////////////////////////
    // Only process the following classes, return quickly otherwise
    ////////////////////////////////////////////////////////////////////////////////
    // ePxActor (rigid static/dynamic only, actortype < 2)
    // ePxArticulation
    // ePxDeformableVolume
    // ePxDeformableSurface
    ////////////////////////////////////////////////////////////////////////////////
    if (!( ((omniPvdObject->mOmniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxActor) &&  (omniPvdObject->mActortype < 2)) ||
           (omniPvdObject->mOmniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxArticulation) ||
           (omniPvdObject->mOmniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxDeformableVolume) ||
           (omniPvdObject->mOmniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxDeformableSurface)
        ))
    {
        return;
    }

    static int32_t pxActorSleepingAttribIndex = -1;
    static int32_t pxActorSleepingClassIndex = -1;

    static int32_t pxActorRigidBodyFlagsAttribIndex = -1;
    static int32_t pxActorRigidBodyFlagsClassIndex = -1;

    static int32_t pxDeformableSleepingAttribIndex = -1;
    static int32_t pxDeformableSleepingClassIndex = -1;



    ////////////////////////////////////////////////////////////////////////////////
    // Even if the prim for a PxActor doesn't have a display colour we set it
    // so that its children inherit it. Test that this works first. Making sure we
    // don't set the display colour in other cases.
    ////////////////////////////////////////////////////////////////////////////////
    static PXR_NS::TfToken colToken = PXR_NS::TfToken("primvars:displayColor");
    PXR_NS::UsdAttribute colAttr = prim->GetAttribute(colToken);
    if (!colAttr)
    {
        colAttr = prim->CreateAttribute(colToken, PXR_NS::SdfValueTypeNames->Color3fArray);
    }

    static PXR_NS::VtArray<PXR_NS::GfVec3f> colourArrayVtStatic;
    static PXR_NS::VtArray<PXR_NS::GfVec3f> colourArrayVtDynamicSleeping;
    static PXR_NS::VtArray<PXR_NS::GfVec3f> colourArrayVtDynamicActive;
    static PXR_NS::VtArray<PXR_NS::GfVec3f> colourArrayVtKinematicSleeping;
    static PXR_NS::VtArray<PXR_NS::GfVec3f> colourArrayVtKinematicActive;

    static bool initDone = false;
    if (!initDone)
    {
        ////////////////////////////////////////////////////////////////////////////////
        // Get the values from a settings file
        ////////////////////////////////////////////////////////////////////////////////
        colourArrayVtStatic.push_back( PXR_NS::GfVec3f(226.0f / 255.0f, 226.0f / 255.0f, 226.0f / 255.0f) ); // static = light gray

        ////////////////////////////////////////////////////////////////////////////////
        // Combo 1
        ////////////////////////////////////////////////////////////////////////////////
        // dynamic active = light pastel green
        // dynamic sleeping = dark green
        // kinematic active = light pastel blue
        // kinematic sleeping = dark blue
        colourArrayVtDynamicActive.push_back( PXR_NS::GfVec3f(119.0f / 255.0f, 221.0f / 255.0f, 119.0f / 255.0f) );
        colourArrayVtDynamicSleeping.push_back( PXR_NS::GfVec3f(47.0f / 255.0f, 76.0f / 255.0f, 57.0f / 255.0f) );                
        colourArrayVtKinematicActive.push_back(PXR_NS::GfVec3f(171.0f / 255.0f, 215.0f / 255.0f, 244.0f / 255.0f) );
        colourArrayVtKinematicSleeping.push_back( PXR_NS::GfVec3f(61.0f / 255.0f, 66.0f / 255.0f, 107.0f / 255.0f) );

        ////////////////////////////////////////////////////////////////////////////////
        // Combo 2
        ////////////////////////////////////////////////////////////////////////////////
        // dynamic active = intense green
        // dynamic sleeping = light pastel green
        // kinematic active = intense blue
        // kinematic sleeping = light pastel blue
        //colourArrayVtDynamicActive.push_back(PXR_NS::GfVec3f(0.0f / 255.0f, 255.0f / 255.0f, 0.0f / 255.0f));
        //colourArrayVtDynamicSleeping.push_back(PXR_NS::GfVec3f(119.0f / 255.0f, 221.0f / 255.0f, 119.0f / 255.0f));
        //colourArrayVtKinematicActive.push_back(PXR_NS::GfVec3f(0.0f / 255.0f, 0.0f / 255.0f, 255.0f / 255.0f));
        //colourArrayVtKinematicSleeping.push_back( PXR_NS::GfVec3f(171.0f / 255.0f, 215.0f / 255.0f, 244.0f / 255.0f));

        initDone = true;
    }

    if (omniPvdObject->mOmniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxActor)
    {
        if (omniPvdObject->mActortype == 0) { // PxActorType::eRIGID_STATIC
            colAttr.Set(colourArrayVtStatic);
        } else if (omniPvdObject->mActortype == 1) { // PxActorType::eRIGID_DYNAMIC
            OmniPvdAttributeInstList* listSleep = getAttribList(pxActorSleepingAttribIndex, pxActorSleepingClassIndex, "isSleeping", omniPvdObject);
            OmniPvdAttributeInstList* listRigidBodyFlags = getAttribList(pxActorRigidBodyFlagsAttribIndex, pxActorRigidBodyFlagsClassIndex,"rigidBodyFlags", omniPvdObject);
            if (listSleep && listRigidBodyFlags) {
                OmniPvdAttributeSample* attribSleep = (OmniPvdAttributeSample*)listSleep->mFirst;
                OmniPvdAttributeSample* attribRigidBodyFlags = (OmniPvdAttributeSample*)listRigidBodyFlags->mFirst;
                if (attribSleep && attribRigidBodyFlags) {
                    uint64_t minUpdatedTimeStamp = attribSleep->mTimeStamp < attribRigidBodyFlags->mTimeStamp ? attribSleep->mTimeStamp : attribRigidBodyFlags->mTimeStamp;
                    bool sleeping = *((uint8_t*)attribSleep->mData);
                    bool kinematic = (*((uint32_t*)attribRigidBodyFlags->mData)) & 1;
                    setColourOfRigidDynamicBody(colAttr, (double)minUpdatedTimeStamp, kinematic, sleeping, colourArrayVtDynamicSleeping, colourArrayVtDynamicActive, colourArrayVtKinematicSleeping, colourArrayVtKinematicActive);
                    while (getNextState(attribSleep, attribRigidBodyFlags, sleeping, kinematic, minUpdatedTimeStamp)) {
                        setColourOfRigidDynamicBody(colAttr, (double)minUpdatedTimeStamp, kinematic, sleeping, colourArrayVtDynamicSleeping, colourArrayVtDynamicActive, colourArrayVtKinematicSleeping, colourArrayVtKinematicActive);
                    }
                }
            }            
        }
    }
    else if (omniPvdObject->mOmniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxArticulation)
    {
        colAttr.Set(colourArrayVtDynamicActive);
    }
    else if (omniPvdObject->mOmniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxDeformableVolume ||
             omniPvdObject->mOmniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxDeformableSurface)
    {
        // Deformables are always dynamic (never kinematic), but do support sleeping
        // Use separate index cache — deformable isSleeping lives on PxDeformableBody, not PxRigidDynamic
        OmniPvdAttributeInstList* listSleep = getAttribList(pxDeformableSleepingAttribIndex, pxDeformableSleepingClassIndex, "isSleeping", omniPvdObject);
        if (listSleep) {
            OmniPvdAttributeSample* attribSleep = (OmniPvdAttributeSample*)listSleep->mFirst;
            if (attribSleep) {
                bool sleeping = *((uint8_t*)attribSleep->mData);
                bool kinematic = false;
                uint64_t minUpdatedTimeStamp = attribSleep->mTimeStamp;
                setColourOfRigidDynamicBody(colAttr, (double)minUpdatedTimeStamp, kinematic, sleeping, colourArrayVtDynamicSleeping, colourArrayVtDynamicActive, colourArrayVtKinematicSleeping, colourArrayVtKinematicActive);
                OmniPvdAttributeSample* dummyKinematic = 0;
                while (getNextState(attribSleep, dummyKinematic, sleeping, kinematic, minUpdatedTimeStamp)) {
                    setColourOfRigidDynamicBody(colAttr, (double)minUpdatedTimeStamp, kinematic, sleeping, colourArrayVtDynamicSleeping, colourArrayVtDynamicActive, colourArrayVtKinematicSleeping, colourArrayVtKinematicActive);
                }
            } else {
                colAttr.Set(colourArrayVtDynamicActive);
            }
        } else {
            colAttr.Set(colourArrayVtDynamicActive);
        }
    }
}



void processObjectHandle(PXR_NS::UsdPrim* prim, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;
    if (omniPvdObject->mAppearedFirstTime)
    {        
        static const PXR_NS::TfToken classToken("omni:pvdi:class");
        PXR_NS::UsdAttribute classAttr = prim->CreateAttribute(classToken, PXR_NS::SdfValueTypeNames->String, PXR_NS::SdfVariability::SdfVariabilityUniform);
        if (classAttr)
        {
            classAttr.Set(omniPvdObject->mOmniPvdClass->mClassName);
        }

        static const PXR_NS::TfToken handleToken("omni:pvdi:handle");
        PXR_NS::UsdAttribute handleAttr = prim->CreateAttribute(handleToken, PXR_NS::SdfValueTypeNames->UInt64, PXR_NS::SdfVariability::SdfVariabilityUniform);
        if (handleAttr)
        {
            handleAttr.Set(omniPvdObject->mOmniAPIHandle);
        }

        if (omniPvdObject->mOmniObjectName.size()>0)
        {
            static const PXR_NS::TfToken nameToken("omni:pvdi:name");
            PXR_NS::UsdAttribute nameAttr = prim->CreateAttribute(nameToken, PXR_NS::SdfValueTypeNames->String, PXR_NS::SdfVariability::SdfVariabilityUniform);
            if (nameAttr)
            {
                nameAttr.Set(omniPvdObject->mOmniObjectName);
            }
        }
        static const PXR_NS::TfToken idToken("omni:pvdi:uid");
        PXR_NS::UsdAttribute idAttr = prim->CreateAttribute(idToken, PXR_NS::SdfValueTypeNames->UInt64, PXR_NS::SdfVariability::SdfVariabilityUniform);
        if (idAttr)
        {
            idAttr.Set(omniPvdObject->mUID);
        }
    }
}

void processObjectTokens(PXR_NS::UsdPrim* prim, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;
    PXR_NS::TfTokenVector tokens;

    const int nbrInheritedClasses = static_cast<int>(omniPvdObject->mInheritedClassInstances.size());
    for (int c = 0; c < nbrInheritedClasses; c++)
    {
        std::vector<OmniPvdAttributeInstList*>& classAttributeLists = omniPvdObject->mInheritedClassInstances[c].mClassAttributeLists;
        const int nbrAttributes = (int)classAttributeLists.size();
        for (int j = 0; j < nbrAttributes; j++)
        {
            OmniPvdAttributeInstList *attributeInstList = classAttributeLists[j];
            if (attributeInstList && attributeInstList->mAttributeDef && OmniPvdUsd::getToken(attributeInstList->mAttributeDef))
            {
                tokens.push_back(*OmniPvdUsd::getToken(attributeInstList->mAttributeDef));
            }
        }
    }
    prim->SetPropertyOrder(tokens);
}

void setDisplayColour(PXR_NS::UsdPrim* prim, float r, float g, float b)
{
    PXR_NS::UsdGeomGprim *geomPrim = (PXR_NS::UsdGeomGprim*)prim;
    PXR_NS::VtArray<PXR_NS::GfVec3f> colourArrayVt;
    colourArrayVt.push_back(PXR_NS::GfVec3f(r, g, b));
    geomPrim->GetDisplayColorAttr().Set(colourArrayVt);
}

////////////////////////////////////////////////////////////////////////////////
// For all concerned objects (first time seen (on creation) or had an attribute set)
//   For all attributes in the class of the object
//     If the attribute was not created and not set : create it
//     If the attribute was not set but just created : set a default value
//     If the attribute was set : set the value stored in the object
////////////////////////////////////////////////////////////////////////////////

void populateEnumMapFromClass(PXR_NS::TfHashMap<uint32_t, PXR_NS::TfToken>& targetMap, const std::string& enumClassName, const OmniPvdDOMState& domState)
{
    // Clear existing entries
    targetMap.clear();
    
    // Find the enum class in the parsed DOM
    std::unordered_map<OmniPvdClassHandle, OmniPvdClass*>::const_iterator it;
    for (it = domState.mClassHandleToClassMap.begin(); it != domState.mClassHandleToClassMap.end(); ++it) {
        OmniPvdClass* omniClass = it->second;
        if (omniClass->mIsEnumClass && omniClass->mClassName == enumClassName) {
            // Found the enum class - populate map from its attribute definitions
            for (size_t i = 0; i < omniClass->mAttributeDefinitions.size(); ++i) {
                OmniPvdAttributeDef* attrDef = omniClass->mAttributeDefinitions[i];
                if (attrDef) {
                    // In enum classes, mNbrFields stores the enum value
                    uint32_t enumValue = attrDef->mNbrFields;
                    PXR_NS::TfToken enumName(attrDef->mAttributeName);
                    targetMap[enumValue] = enumName;
                    
                    /*
                    CARB_LOG_INFO("Populated enum %s: %u -> %s", 
                                  enumClassName.c_str(), enumValue, enumName.GetText());
                    */
                }
            }
            return;
        }
    }
    
    // Enum class not found - log warning
    CARB_LOG_WARN("Enum class '%s' not found in OVD stream", enumClassName.c_str());
}

void updateMaps(const OmniPvdDOMState& domState)
{
    populateEnumMapFromClass(OmniPvd::articulationJointMotionMap, "PxArticulationMotion", domState);
    populateEnumMapFromClass(OmniPvd::articulationJointDriveTypeMap, "PxArticulationDriveType", domState);        
    populateEnumMapFromClass(OmniPvd::jointD6MotionMap, "PxD6Motion", domState);
}

void createAndSetUSDAttribPass(
    PXR_NS::UsdStageRefPtr* usdStage,
    std::list<OmniPvdObject*> &objectCreations,
    bool isSharedLayer,
    int isUSDA,
    PXR_NS::SdfLayerRefPtr& layer,
    OmniPvdDOMState& domState,
    std::string& sharedlayerIdentifier)
{
    updateMaps(domState);

    std::list<OmniPvdObject*>::iterator it;
    for (it = objectCreations.begin(); it != objectCreations.end(); it++)
    {
        OmniPvdObject* omniPvdObject = *it;
        ////////////////////////////////////////////////////////////////////////////////
        // Only process objects that belong to the layer that we are handling. In this
        // case isSharedLayer could just as well be an integer identifying the layer that
        // the object belongs to. Would be cleaner indeed.
        ////////////////////////////////////////////////////////////////////////////////
        if (isSharedLayer == omniPvdObject->mIsShared)
        {
            if (omniPvdObject->mAppearedFirstTime)
            {
                OmniPvdClass *omniPvdClass = omniPvdObject->mOmniPvdClass;
                PXR_NS::UsdPrim prim = (*usdStage)->GetPrimAtPath(OmniPvdUsd::getPrimPath(omniPvdObject));
                if (prim) // valid USD prim object
                {
                    const OmniPvdUsdClassEnum usdClass = OmniPvdUsd::getClassId(omniPvdClass);
                    switch (usdClass)
                    {
                    case OmniPvdUsdClassEnum::eUSDClassGeomCapsule:
                    {
                        PXR_NS::UsdGeomCapsule geom = (PXR_NS::UsdGeomCapsule)prim;
                        if (geom)
                        {
                            geom.GetAxisAttr().Set(OmniPvd::XToken);
                        }
                    }
                    break;
                    case OmniPvdUsdClassEnum::eUSDClassCylinder:
                    {
                        PXR_NS::UsdGeomCylinder geom = (PXR_NS::UsdGeomCylinder)prim;
                        if (geom)
                        {
                            geom.GetAxisAttr().Set(OmniPvd::XToken);
                        }
                    }
                    break;
                    case OmniPvdUsdClassEnum::eUSDClassCone:
                    {
                        PXR_NS::UsdGeomCone geom = (PXR_NS::UsdGeomCone)prim;
                        if (geom)
                        {
                            geom.GetAxisAttr().Set(OmniPvd::XToken);
                        }
                    }
                    break;
                    case OmniPvdUsdClassEnum::eUSDClassGeomPlane:
                    {
                        processPlane(&prim, 0, omniPvdObject);
                    }
                    break;
                    case OmniPvdUsdClassEnum::eUSDClassGeomMesh:
                    {
                        OmniPvdPhysXClassEnum physXClassId = omniPvdObject->mOmniPvdClass->mPhysXBaseProcessingClassId;
                        if (physXClassId == OmniPvdPhysXClassEnum::ePxTriangleMesh)
                        {
                            // Triangle meshes may have animated positions/velocities for deformable surfaces
                            processTetMesh(&prim, 0, omniPvdObject);
                        }
                        else
                        {
                            processMesh(&prim, 0, omniPvdObject);
                        }
                    }
                    break;
                    case OmniPvdUsdClassEnum::eUSDClassGeomTetMesh:
                    {
                        processTetMesh(&prim, 0, omniPvdObject);
                    }
                    break;
                    case OmniPvdUsdClassEnum::eUSDClassGeomPoints:
                    {
                        processPoints(&prim, 0, omniPvdObject);
                    }
                    break;
                    case OmniPvdUsdClassEnum::eUSDClassOver:
                    {
                        if (omniPvdObject->mReferenceObject)
                        {
                            prim.GetReferences().AddReference(sharedlayerIdentifier, OmniPvdUsd::getPrimPath(omniPvdObject->mReferenceObject),
                                PXR_NS::SdfLayerOffset(0.0),
                                PXR_NS::UsdListPosition::UsdListPositionBackOfPrependList);
                        }
                    }
                    break;
                    default:
                    break;
                    }

                    if ((usdClass != OmniPvdUsdClassEnum::eUSDClassGeomPoints) &&
                        (usdClass != OmniPvdUsdClassEnum::eUSDClassGeomMesh) &&
                        (usdClass != OmniPvdUsdClassEnum::eUSDClassGeomTetMesh) &&
                        (usdClass != OmniPvdUsdClassEnum::eUSDClassOver) )
                    {
                        ////////////////////////////////////////////////////////////////////////////////
                        // For all classes that the OVD object inherits, starting at the most "root" ancestor class
                        ////////////////////////////////////////////////////////////////////////////////
                        const int nbrInheritedClasses = static_cast<int>(omniPvdObject->mInheritedClassInstances.size());
                        for (int c = 0; c < nbrInheritedClasses; c++)
                        {
                            ////////////////////////////////////////////////////////////////////////////////
                            // For all attributes that were set on this OVD object, for a certain inherited class
                            //   Process the OVD attribute
                            //     1) if it's tagged a special USD attribute -> call the appropriate USD schema attribute function
                            //     2) also in any case create the mirror USD schemaless attribute
                            ////////////////////////////////////////////////////////////////////////////////
                            std::vector<OmniPvdAttributeInstList*>& classAttributeLists = omniPvdObject->mInheritedClassInstances[c].mClassAttributeLists;
                            const int nbrAttributes = (int)classAttributeLists.size();
                            for (int j = 0; j < nbrAttributes; j++)
                            {
                                OmniPvdAttributeInstList *attributeInstList = classAttributeLists[j];
                                if (attributeInstList)
                                {
                                    if (!attributeInstList->mAttributeDef->mIsUniqueList)
                                    {
                                        OmniPvdAttributeSample *attrib = (OmniPvdAttributeSample*)attributeInstList->mFirst;
                                        bool processedAsCustom = false;
                                        ////////////////////////////////////////////////////////////////////////////////
                                        // This switch is not necessary and should be a switch done once, and the attrib
                                        // iterator increment should instead be done inside the process functions.
                                        ////////////////////////////////////////////////////////////////////////////////
                                        while (attrib)
                                        {
                                            switch (OmniPvdUsd::getAttributeId(attributeInstList->mAttributeDef))
                                            {

                                            case OmniPvdUsdAttributeEnum::eUSDAttributeTranslateOp:
                                            {
                                                processTranslation(&prim, attrib, omniPvdObject);
                                            }
                                            break;

                                            case OmniPvdUsdAttributeEnum::eUSDAttributeScaleOp:
                                            {
                                                processScale(&prim, attrib, omniPvdObject);
                                            }
                                            break;

                                            case OmniPvdUsdAttributeEnum::eUSDAttributeRotationOp:
                                            {
                                                processRotation(&prim, attrib, omniPvdObject);
                                            }
                                            break;

                                            case OmniPvdUsdAttributeEnum::eUSDAttributeDisplayColor:
                                            {
                                                // Not handled - legacy
                                            }
                                            break;

                                            case OmniPvdUsdAttributeEnum::eUSDAttributeCustom:
                                            {
                                                processCustomAttribute(prim, attrib, attributeInstList->mAttributeDef);
                                                processedAsCustom = true;
                                            }
                                            break;

                                            case OmniPvdUsdAttributeEnum::eUSDAttributeChildNode:
                                            {
                                                // Not handled
                                            }
                                            break;

                                            case OmniPvdUsdAttributeEnum::eUSDAttributeAxis:
                                            {
                                                switch (usdClass)
                                                {
                                                case OmniPvdUsdClassEnum::eUSDClassCone:
                                                    processConeAxis(&prim, attrib, omniPvdObject);
                                                    break;
                                                case OmniPvdUsdClassEnum::eUSDClassCylinder:
                                                    processCylinderAxis(&prim, attrib, omniPvdObject);
                                                    break;
                                                }
                                            }
                                            break;

                                            case OmniPvdUsdAttributeEnum::eUSDAttributeHeight:
                                            {
                                                switch (usdClass)
                                                {
                                                case OmniPvdUsdClassEnum::eUSDClassCone:
                                                    processConeHeight(&prim, attrib, omniPvdObject);
                                                    break;
                                                case OmniPvdUsdClassEnum::eUSDClassGeomCapsule:
                                                    processCapsuleHeight(&prim, attrib, omniPvdObject);
                                                    break;
                                                case OmniPvdUsdClassEnum::eUSDClassCylinder:
                                                    processCylinderHeight(&prim, attrib, omniPvdObject);
                                                    break;
                                                }
                                            }
                                            break;

                                            case OmniPvdUsdAttributeEnum::eUSDAttributeRadius:
                                            {
                                                switch (usdClass)
                                                {
                                                case OmniPvdUsdClassEnum::eUSDClassCone:
                                                    processConeRadius(&prim, attrib, omniPvdObject);
                                                    break;
                                                case OmniPvdUsdClassEnum::eUSDClassGeomCapsule:
                                                    processCapsuleRadius(&prim, attrib, omniPvdObject);
                                                    break;
                                                case OmniPvdUsdClassEnum::eUSDClassCylinder:
                                                    processCylinderRadius(&prim, attrib, omniPvdObject);
                                                    break;
                                                case OmniPvdUsdClassEnum::eUSDClassGeomSphere:
                                                    processSphereRadius(&prim, attrib, omniPvdObject);
                                                    break;
                                                }
                                            }
                                            break;

                                            case OmniPvdUsdAttributeEnum::eUSDAttributeVerts:
                                            {
                                                // Taken care of by processMesh
                                            }
                                            break;

                                            case OmniPvdUsdAttributeEnum::eUSDAttributeTris:
                                            {
                                                // Taken care of by processMesh
                                            }
                                            break;

                                            case OmniPvdUsdAttributeEnum::eUSDAttributePoints:
                                            {
                                                // Taken care of by processPoints
                                            }
                                            break;

                                            case OmniPvdUsdAttributeEnum::eUSDAttributeDeformablePositions:
                                            {
                                                // Taken care of by processTetMesh
                                            }
                                            break;

                                            case OmniPvdUsdAttributeEnum::eUSDAttributeDeformableVelocities:
                                            {
                                                // Taken care of by processTetMesh
                                            }
                                            break;

                                            case OmniPvdUsdAttributeEnum::eUSDAttributeTets:
                                            {
                                                // Taken care of by processTetMesh
                                            }
                                            break;

                                            case OmniPvdUsdAttributeEnum::eUSDAttributeEnum:
                                            {
                                                processEnum(&prim, attrib, omniPvdObject, attributeInstList->mAttributeDef);
                                            }
                                            break;

                                            case OmniPvdUsdAttributeEnum::eUSDAttributeTransformFork:
                                            {
                                                processXFormFork(&prim, attrib, omniPvdObject);
                                            }
                                            break;

                                            case OmniPvdUsdAttributeEnum::eUSDAttributeNone:
                                            {
                                                // Not handled, just for completion of all enums
                                            }
                                            break;

                                            }
                                            // end of switch

                                            ////////////////////////////////////////////////////////////////////////////////
                                            // This makes sure that any attribute that was not yet handled with a processCustomAttribute
                                            // does get a mirror, to makle sure it visible as both a USD schema attribute and an OVD
                                            // schemaless attribute
                                            ////////////////////////////////////////////////////////////////////////////////
                                            if ((!processedAsCustom) && (OmniPvdUsd::getAttributeId(attributeInstList->mAttributeDef)!=OmniPvdUsdAttributeEnum::eUSDAttributeNone))
                                            {
                                                processCustomAttribute(prim, attrib, attributeInstList->mAttributeDef);
                                            }

                                            ////////////////////////////////////////////////////////////////////////////////
                                            // TODO : this should be done in a batched fashion inside the processX functions
                                            // Go to the next attribute time sample
                                            ////////////////////////////////////////////////////////////////////////////////
                                            attrib = (OmniPvdAttributeSample*)(attrib->mNextAttribute);
                                        }
                                    }
                                    else // it's a unique list or set
                                    {
                                        OmniPvdUniqueList *uniqueListAttrib = (OmniPvdUniqueList*)attributeInstList->mFirst;
                                        while (uniqueListAttrib)
                                        {
                                            processCustomAttribute(prim, uniqueListAttrib, attributeInstList->mAttributeDef);
                                            uniqueListAttrib = (OmniPvdUniqueList*)(uniqueListAttrib->mNextAttribute);
                                        }
                                    }
                                }
                            }
                        }
                    }
                    processVisibility(&prim, 0, omniPvdObject, domState);
                    processActivityColourSeries(&prim, omniPvdObject);
                    processObjectHandle(&prim, omniPvdObject);
                    processObjectTokens(&prim, omniPvdObject);
                }

                omniPvdObject->mAppearedFirstTime = 0;
            }
        }
    }
}

void extractRootObjects(std::list<OmniPvdObject*> &rootNodes, std::list<OmniPvdObject*> &objectCreations)
{
    rootNodes.clear();

    std::list<OmniPvdObject*>::iterator it;
    for (it = objectCreations.begin(); it != objectCreations.end(); it++)
    {
        OmniPvdObject* pvdObject = (*it);
        if (pvdObject)
        {
            if (!pvdObject->mAncestor)
            {
                rootNodes.push_back(pvdObject);
            }
        }
    }
}

void propagateObjectSharedState(OmniPvdObject* sharedObject)
{
    OmniPvdObject* child = sharedObject->mFirstChild;
    while (child)
    {
        child->mIsShared = 1;
        propagateObjectSharedState(child);
        child = child->mNextSibling;
    }
}

void propagateRootNodeSharedState(std::list<OmniPvdObject*> &rootNodes)
{
    std::list<OmniPvdObject*>::iterator it;
    for (it = rootNodes.begin(); it != rootNodes.end(); it++)
    {
        OmniPvdObject* pvdObject = (*it);
        if (pvdObject->mIsShared)
        {
            propagateObjectSharedState(pvdObject);
        }
    }
}

// Note : Once the function returns the user must now reprocess the root nodes list as they have potentially changed
void processClassGrouping(OmniPvdDOMState &domState)
{
    std::list<OmniPvdObject*>::iterator it;
    for (it = domState.mObjectCreations.begin(); it != domState.mObjectCreations.end(); it++) {
        OmniPvdObject* pvdObject = (*it);
        if (pvdObject) {
            if (!pvdObject->mIsShared) {
                if (!pvdObject->mAncestor) {
                    if (pvdObject != domState.mSceneRoot) {
                        if (pvdObject->mOmniPvdClass) {
                            // Does the class name start with Px and is not PxPhysics? -> push to shared
                            if ((!pvdObject->mOmniPvdClass->mClassName.rfind("Px", 0)) && (pvdObject->mOmniPvdClass->mClassName.compare("PxPhysics"))) {
                                domState.parentUnderClassGroup(pvdObject, true);
                            } else {
                                domState.parentUnderClassGroup(pvdObject, false);
                            }
                        }                        
                    }
                }
            } else {
                if (!pvdObject->mAncestor) {
                    if (pvdObject != domState.mSharedRoot) {
                        if (pvdObject->mOmniPvdClass) {
                            domState.parentUnderClassGroup(pvdObject, true);
                        }                        
                    }
                }

            }
        }
    }    
}

bool getGravity(OmniPvdDOMState &domState, float* gravity)
{
    OmniPvdObject* sceneRoot = domState.mSceneRoot;
    if (!sceneRoot)
    {
        return false;
    }
    OmniPvdObject* scene = sceneRoot->mLastChild;
    while (scene && scene->mOmniPvdClass->mClassName != "PxScene")
    {
        scene = scene->mPrevSibling;
    }
    if (!scene)
    {
        return false;
    }
    while (scene)
    {
        ////////////////////////////////////////////////////////////////////////////////
        // We have a scene, now get the gravity attribute
        ////////////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////////////
        // Must be set to < 0 otherwise the geAttribData function assumes you know the
        // index of the attribute in the class definition vector.
        ////////////////////////////////////////////////////////////////////////////////
        int32_t attribIndex = -1;
        int32_t classIndex = -1;
        OmniPvdAttributeInstList* gravList = getAttribList(attribIndex, classIndex, "gravity", scene);
        if (!gravList)
        {
            return false;
        }
        ////////////////////////////////////////////////////////////////////////////////
        // To be sure go through all attribute instances of gravity for the scene
        ////////////////////////////////////////////////////////////////////////////////
        OmniPvdAttributeSample* attrib = (OmniPvdAttributeSample*)gravList->mFirst;
        float* gVec = 0;
        while (attrib) {
            gVec = (float*)attrib->mData;
            if ((fabs(gVec[0]) > 0.0f) || (fabs(gVec[1]) > 0.0f) || (fabs(gVec[2]) > 0.0f))
            {
                gravity[0] = gVec[0];
                gravity[1] = gVec[1];
                gravity[2] = gVec[2];
                return true;
            }
            attrib = (OmniPvdAttributeSample*)(attrib->mNextAttribute);
        }
        scene = scene->mNextSibling;
    }
    return false;
}

////////////////////////////////////////////////////////////////////////////////
// Gets the tolerancesScale from the PxScene in the DOM state
// tolerancesScale[0] = length, tolerancesScale[1] = speed
// Returns true if the tolerancesScale was found, false otherwise
////////////////////////////////////////////////////////////////////////////////
bool getTolerancesScale(OmniPvdDOMState &domState, float* tolerancesScale)
{
    OmniPvdObject* sceneRoot = domState.mSceneRoot;
    if (!sceneRoot)
    {
        return false;
    }
    OmniPvdObject* scene = sceneRoot->mLastChild;
    while (scene && scene->mOmniPvdClass->mClassName != "PxScene")
    {
        scene = scene->mPrevSibling;
    }
    if (!scene)
    {
        return false;
    }
    while (scene)
    {
        ////////////////////////////////////////////////////////////////////////////////
        // We have a scene, now get the tolerancesScale attribute
        ////////////////////////////////////////////////////////////////////////////////
        int32_t attribIndex = -1;
        int32_t classIndex = -1;
        OmniPvdAttributeInstList* tolList = getAttribList(attribIndex, classIndex, "tolerancesScale", scene);
        if (!tolList)
        {
            scene = scene->mNextSibling;
            continue;
        }
        ////////////////////////////////////////////////////////////////////////////////
        // Get the first tolerancesScale attribute sample
        ////////////////////////////////////////////////////////////////////////////////
        OmniPvdAttributeSample* attrib = (OmniPvdAttributeSample*)tolList->mFirst;
        if (attrib && attrib->mData)
        {
            float* tolVec = (float*)attrib->mData;
            tolerancesScale[0] = tolVec[0];
            tolerancesScale[1] = tolVec[1];
            return true;
        }
        scene = scene->mNextSibling;
    }
    return false;
}

void writeUSDFile(char *usdStageDir, int upAxis, int isUSDA, OmniPvdDOMState &domState)
{
    if (!usdStageDir) return;
    if (usdStageDir[0] == '\0') return;

    std::string mOutputDir(usdStageDir);

    PXR_NS::UsdStageRefPtr mStage;
    std::string mStageName;

    PXR_NS::SdfLayerRefPtr mSceneSublayer;
    std::string mSceneSublayerName;
    std::string mSceneSublayerNameAbsolute;

    PXR_NS::SdfLayerRefPtr mSharedSublayer;
    std::string mSharedSublayerName;
    std::string mSharedSublayerNameAbsolute;

    ////////////////////////////////////////////////////////////////////////////////
    // Set stage and sub layer names
    ////////////////////////////////////////////////////////////////////////////////
    std::string fileSuffix;
    if (isUSDA)
    {
        fileSuffix = ".usda";
    }
    else
    {
        fileSuffix = ".usdc";
    }

    mStageName = mOutputDir + "stage" + ".usda"; // the stage is always USDA

    mSceneSublayerName = "scene" + fileSuffix;
    mSceneSublayerNameAbsolute = mOutputDir + mSceneSublayerName;

    mSharedSublayerName = "shared" + fileSuffix;
    mSharedSublayerNameAbsolute = mOutputDir + mSharedSublayerName;

    ////////////////////////////////////////////////////////////////////////////////
    // Create the stage
    ////////////////////////////////////////////////////////////////////////////////
    const auto& existing_layer = PXR_NS::SdfLayer::FindOrOpen(mStageName.c_str());
    if (existing_layer)
    {
        mStage = PXR_NS::UsdStage::Open(existing_layer);
    }
    else
    {
        mStage = PXR_NS::UsdStage::CreateNew(mStageName.c_str());
        if (!mStage)
        {
            CARB_LOG_ERROR("OmniPvd UsdStage::Create failed");
            return;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Create the layers
    ////////////////////////////////////////////////////////////////////////////////
    createAndClearLayer(mSceneSublayer, mSceneSublayerNameAbsolute);
    createAndClearLayer(mSharedSublayer, mSharedSublayerNameAbsolute);

    ////////////////////////////////////////////////////////////////////////////////
    // Insert the layers as sub layers into/onto the Stage
    ////////////////////////////////////////////////////////////////////////////////
    insertAsSublayer(mStage, mSharedSublayer, mSharedSublayerName);
    insertAsSublayer(mStage, mSceneSublayer, mSceneSublayerName);

    ////////////////////////////////////////////////////////////////////////////////
    // Create and type specify the prims using SDF
    ////////////////////////////////////////////////////////////////////////////////

    std::list<OmniPvdObject*> rootObjects;
    extractRootObjects(rootObjects, domState.mObjectCreations);

    propagateRootNodeSharedState(rootObjects);
    processClassGrouping(domState); // this invalidates the rootObjects list

    rootObjects.clear();
    extractRootObjects(rootObjects, domState.mObjectCreations);

    {
        PXR_NS::SdfChangeBlock block;
        createSDFPrimSpecPass(mSharedSublayer, rootObjects, 1);
    }
    {
        PXR_NS::SdfChangeBlock block;
        createSDFPrimSpecPass(mSceneSublayer, rootObjects, 0);
    }

    rootObjects.clear();

    ////////////////////////////////////////////////////////////////////////////////
    // Extract the up axis from the DOM structure
    ////////////////////////////////////////////////////////////////////////////////

    float gravityVec[3];
    if (getGravity(domState, gravityVec))
    {
        if (fabs(gravityVec[2]) > 0.0f)
        {
            PXR_NS::UsdGeomSetStageUpAxis(mStage, PXR_NS::UsdGeomTokens->z);
        }
        else if (fabs(gravityVec[1]) > 0.0f)
        {
            PXR_NS::UsdGeomSetStageUpAxis(mStage, PXR_NS::UsdGeomTokens->y);
        }
    }
    else
    {
        printf("did not find gravity\n");
        if (upAxis == 0)
        {
            PXR_NS::UsdGeomSetStageUpAxis(mStage, PXR_NS::UsdGeomTokens->y);
        }
        else
        {
            PXR_NS::UsdGeomSetStageUpAxis(mStage, PXR_NS::UsdGeomTokens->z);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Set metersPerUnit based on the PhysX tolerancesScale::length
    // tolerancesScale[0] is the length scale factor (1.0 for meters, 100.0 for cm)
    // metersPerUnit = 1.0 / tolerancesScale[0]
    ////////////////////////////////////////////////////////////////////////////////
    float tolerancesScale[2];
    if (getTolerancesScale(domState, tolerancesScale))
    {
        if (tolerancesScale[0] > 0.0f)
        {
            double metersPerUnit = 1.0 / (double)tolerancesScale[0];
            PXR_NS::UsdGeomSetStageMetersPerUnit(mStage, metersPerUnit);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Create and set attributes using USD
    ////////////////////////////////////////////////////////////////////////////////
    std::string sharedLayerIdentifier = mSharedSublayer->GetIdentifier();
    {
        PXR_NS::SdfChangeBlock block;
        PXR_NS::UsdEditContext editCtx(mStage, mSharedSublayer);
        createAndSetUSDAttribPass(&mStage, domState.mObjectCreations, 1, isUSDA, mSharedSublayer, domState, sharedLayerIdentifier);
    }
    {
        PXR_NS::SdfChangeBlock block;
        PXR_NS::UsdEditContext editCtx(mStage, mSceneSublayer);
        createAndSetUSDAttribPass(&mStage, domState.mObjectCreations, 0, isUSDA, mSceneSublayer, domState, sharedLayerIdentifier);
    }
    mStage->SetStartTimeCode((double)domState.mMinFrame); // Always starts at startFrame
    mStage->SetEndTimeCode((double)domState.mMaxFrame-1); // Always stops at stopFrame-1 as the last frame is one frame beyond the last simulated frame
    
    ////////////////////////////////////////////////////////////////////////////////
    // Add stage lights for proper visualization
    ////////////////////////////////////////////////////////////////////////////////
    if (!mStage->GetPrimAtPath(PXR_NS::SdfPath("/OvdDomeLight")))
    {
        PXR_NS::UsdLuxDomeLight domeLight = PXR_NS::UsdLuxDomeLight::Define(mStage, PXR_NS::SdfPath("/OvdDomeLight"));
        domeLight.CreateIntensityAttr().Set(100.0f);
        domeLight.CreateColorAttr().Set(PXR_NS::GfVec3f(0.5f, 0.75f, 1.0f));
    }
    if (!mStage->GetPrimAtPath(PXR_NS::SdfPath("/OvdDistantLight")))
    {
        PXR_NS::UsdLuxDistantLight distantLight = PXR_NS::UsdLuxDistantLight::Define(mStage, PXR_NS::SdfPath("/OvdDistantLight"));
        distantLight.CreateIntensityAttr().Set(500.0f);
        PXR_NS::UsdGeomXformable(distantLight).AddRotateXYZOp(PXR_NS::UsdGeomXformOp::PrecisionDouble).Set(PXR_NS::GfVec3d(315.0, 0.0, 0.0));
    }

    mStage->Save();

    CARB_LOG_INFO("OmniPvd USD Stage successfully saved to : %s", mStageName.c_str());
}

long int createUSDFileInMemory(int upAxis, OmniPvdDOMState& domState)
{
    // Create an anonymous layer for the stage
    //PXR_NS::SdfLayerRefPtr rootLayer = PXR_NS::SdfLayer::CreateAnonymous("stage.usda");
    PXR_NS::UsdStageRefPtr stage = PXR_NS::UsdStage::CreateInMemory();
    if (!stage)
    {
        CARB_LOG_ERROR("OmniPvd UsdStage::Open failed");
        return 0;
    }
    std::string sharedLayerName = "shared.usd";
    std::string sceneLayerName = "scene.usd";

    // Create anonymous layers for sublayers
    PXR_NS::SdfLayerRefPtr mSceneSublayer = PXR_NS::SdfLayer::CreateAnonymous(sceneLayerName.c_str());
    PXR_NS::SdfLayerRefPtr mSharedSublayer = PXR_NS::SdfLayer::CreateAnonymous(sharedLayerName.c_str());

    // Insert the layers as sub layers into/onto the Stage
    //insertAsSublayer(stage, mSharedSublayer, sharedLayerName);
    //insertAsSublayer(stage, mSceneSublayer, sceneLayerName);

    PXR_NS::SdfLayerHandle rootLayer = stage->GetRootLayer();
    rootLayer->GetSubLayerPaths().push_back(mSharedSublayer->GetIdentifier());
    rootLayer->GetSubLayerPaths().push_back(mSceneSublayer->GetIdentifier());
    
    // Create and type specify the prims using SDF
    std::list<OmniPvdObject*> rootObjects;
    extractRootObjects(rootObjects, domState.mObjectCreations);

    propagateRootNodeSharedState(rootObjects);
    processClassGrouping(domState); // this invalidates the rootObjects list

    rootObjects.clear();
    extractRootObjects(rootObjects, domState.mObjectCreations);

    {
        PXR_NS::SdfChangeBlock block;
        createSDFPrimSpecPass(mSharedSublayer, rootObjects, 1);
    }
    {
        PXR_NS::SdfChangeBlock block;
        createSDFPrimSpecPass(mSceneSublayer, rootObjects, 0);
    }

    rootObjects.clear();

    // Set up axis
    float gravityVec[3];
    if (getGravity(domState, gravityVec))
    {
        if (fabs(gravityVec[2]) > 0.0f)
        {
            PXR_NS::UsdGeomSetStageUpAxis(stage, PXR_NS::UsdGeomTokens->z);
        }
        else if (fabs(gravityVec[1]) > 0.0f)
        {
            PXR_NS::UsdGeomSetStageUpAxis(stage, PXR_NS::UsdGeomTokens->y);
        }
    }
    else
    {
        if (upAxis == 0)
        {
            PXR_NS::UsdGeomSetStageUpAxis(stage, PXR_NS::UsdGeomTokens->y);
        }
        else
        {
            PXR_NS::UsdGeomSetStageUpAxis(stage, PXR_NS::UsdGeomTokens->z);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Set metersPerUnit based on the PhysX tolerancesScale::length
    // tolerancesScale[0] is the length scale factor (1.0 for meters, 100.0 for cm)
    // metersPerUnit = 1.0 / tolerancesScale[0]
    ////////////////////////////////////////////////////////////////////////////////
    float tolerancesScale[2];
    if (getTolerancesScale(domState, tolerancesScale))
    {
        if (tolerancesScale[0] > 0.0f)
        {
            double metersPerUnit = 1.0 / (double)tolerancesScale[0];
            PXR_NS::UsdGeomSetStageMetersPerUnit(stage, metersPerUnit);
        }
    }

    // Create and set attributes using USD
    std::string sharedLayerIdentifier = mSharedSublayer->GetIdentifier();
    {
        PXR_NS::SdfChangeBlock block;
        PXR_NS::UsdEditContext editCtx(stage, mSharedSublayer);
        createAndSetUSDAttribPass(&stage, domState.mObjectCreations, true, 0, mSharedSublayer, domState, sharedLayerIdentifier);
    }
    {
        PXR_NS::SdfChangeBlock block;
        PXR_NS::UsdEditContext editCtx(stage, mSceneSublayer);
        createAndSetUSDAttribPass(&stage, domState.mObjectCreations, false, 0, mSceneSublayer, domState, sharedLayerIdentifier);
    }

    stage->SetStartTimeCode((double)domState.mMinFrame); // Always starts at startFrame
    stage->SetEndTimeCode((double)domState.mMaxFrame-1); // Always stops at stopFrame-1 as the last frame is one frame beyond the last simulated frame

    // Add stage lights for proper visualization
    if (!stage->GetPrimAtPath(PXR_NS::SdfPath("/OvdDomeLight")))
    {
        PXR_NS::UsdLuxDomeLight domeLight = PXR_NS::UsdLuxDomeLight::Define(stage, PXR_NS::SdfPath("/OvdDomeLight"));
        domeLight.CreateIntensityAttr().Set(100.0f);
        domeLight.CreateColorAttr().Set(PXR_NS::GfVec3f(0.5f, 0.75f, 1.0f));
    }
    if (!stage->GetPrimAtPath(PXR_NS::SdfPath("/OvdDistantLight")))
    {
        PXR_NS::UsdLuxDistantLight distantLight = PXR_NS::UsdLuxDistantLight::Define(stage, PXR_NS::SdfPath("/OvdDistantLight"));
        distantLight.CreateIntensityAttr().Set(500.0f);
        PXR_NS::UsdGeomXformable(distantLight).AddRotateXYZOp(PXR_NS::UsdGeomXformOp::PrecisionDouble).Set(PXR_NS::GfVec3d(315.0, 0.0, 0.0));
    }

    // add the stage to the cache
    PXR_NS::UsdUtilsStageCache::Get().Insert(stage);
    
    long stageId = PXR_NS::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    return stageId;
}
