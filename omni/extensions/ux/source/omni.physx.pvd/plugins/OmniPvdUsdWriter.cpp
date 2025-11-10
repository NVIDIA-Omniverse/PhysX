// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "OmniPvdUsdWriter.h"
#include "OmniPvdDomUtils.h"

#include "OmniPvdSdfUtils.h"
#include "OmniPvdUsdUtils.h"

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
    pxr::UsdAttribute& colAttr,
    double minUpdatedTimeStamp,
    bool kinematic,
    bool sleeping,
    pxr::VtArray<pxr::GfVec3f>& colourArrayVtDarkGreen,
    pxr::VtArray<pxr::GfVec3f>& colourArrayVtLightGreen,
    pxr::VtArray<pxr::GfVec3f>& colourArrayVtDarkBlue,
    pxr::VtArray<pxr::GfVec3f>& colourArrayVtLightBlue
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

void processActivityColourSeries(pxr::UsdPrim* prim, OmniPvdObject* omniPvdObject)
{
    if (!omniPvdObject) return;
    if (!omniPvdObject->mOmniPvdClass) return;

    ////////////////////////////////////////////////////////////////////////////////
    // Only process the following
    ////////////////////////////////////////////////////////////////////////////////
    // omniPvdObject->mOmniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxActor && (PxActorType::eRIGID_STATIC or PxActorType::eRIGID_DYNAMIC)
    // omniPvdObject->mOmniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxArticulation
    // else return quickly
    ////////////////////////////////////////////////////////////////////////////////
    if (!( ((omniPvdObject->mOmniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxActor) &&  (omniPvdObject->mActortype < 2)) ||
           (omniPvdObject->mOmniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxArticulation)
        ))
    {
        return;
    }

    static int32_t pxActorSleepingAttribIndex = -1;
    static int32_t pxActorSleepingClassIndex = -1;

    static int32_t pxActorRigidBodyFlagsAttribIndex = -1;
    static int32_t pxActorRigidBodyFlagsClassIndex = -1;



    ////////////////////////////////////////////////////////////////////////////////
    // Even if the prim for a PxActor doesn't have a display colour we set it
    // so that its children inherit it. Test that this works first. Making sure we
    // don't set the display colour in other cases.
    ////////////////////////////////////////////////////////////////////////////////
    static pxr::TfToken colToken = pxr::TfToken("primvars:displayColor");
    pxr::UsdAttribute colAttr = prim->GetAttribute(colToken);
    if (!colAttr)
    {
        colAttr = prim->CreateAttribute(colToken, pxr::SdfValueTypeNames->Color3fArray);
    }

    static pxr::VtArray<pxr::GfVec3f> colourArrayVtStatic;
    static pxr::VtArray<pxr::GfVec3f> colourArrayVtDynamicSleeping;
    static pxr::VtArray<pxr::GfVec3f> colourArrayVtDynamicActive;
    static pxr::VtArray<pxr::GfVec3f> colourArrayVtKinematicSleeping;
    static pxr::VtArray<pxr::GfVec3f> colourArrayVtKinematicActive;

    static bool initDone = false;
    if (!initDone)
    {
        ////////////////////////////////////////////////////////////////////////////////
        // Get the values from a settings file
        ////////////////////////////////////////////////////////////////////////////////
        colourArrayVtStatic.push_back( pxr::GfVec3f(226.0f / 255.0f, 226.0f / 255.0f, 226.0f / 255.0f) ); // static = light gray

        ////////////////////////////////////////////////////////////////////////////////
        // Combo 1
        ////////////////////////////////////////////////////////////////////////////////
        // dynamic active = light pastel green
        // dynamic sleeping = dark green
        // kinematic active = light pastel blue
        // kinematic sleeping = dark blue
        colourArrayVtDynamicActive.push_back( pxr::GfVec3f(119.0f / 255.0f, 221.0f / 255.0f, 119.0f / 255.0f) );
        colourArrayVtDynamicSleeping.push_back( pxr::GfVec3f(47.0f / 255.0f, 76.0f / 255.0f, 57.0f / 255.0f) );                
        colourArrayVtKinematicActive.push_back(pxr::GfVec3f(171.0f / 255.0f, 215.0f / 255.0f, 244.0f / 255.0f) );
        colourArrayVtKinematicSleeping.push_back( pxr::GfVec3f(61.0f / 255.0f, 66.0f / 255.0f, 107.0f / 255.0f) );

        ////////////////////////////////////////////////////////////////////////////////
        // Combo 2
        ////////////////////////////////////////////////////////////////////////////////
        // dynamic active = intense green
        // dynamic sleeping = light pastel green
        // kinematic active = intense blue
        // kinematic sleeping = light pastel blue
        //colourArrayVtDynamicActive.push_back(pxr::GfVec3f(0.0f / 255.0f, 255.0f / 255.0f, 0.0f / 255.0f));
        //colourArrayVtDynamicSleeping.push_back(pxr::GfVec3f(119.0f / 255.0f, 221.0f / 255.0f, 119.0f / 255.0f));
        //colourArrayVtKinematicActive.push_back(pxr::GfVec3f(0.0f / 255.0f, 0.0f / 255.0f, 255.0f / 255.0f));
        //colourArrayVtKinematicSleeping.push_back( pxr::GfVec3f(171.0f / 255.0f, 215.0f / 255.0f, 244.0f / 255.0f));

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
}



void processObjectHandle(pxr::UsdPrim* prim, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;
    if (omniPvdObject->mAppearedFirstTime)
    {        
        static const pxr::TfToken classToken("omni:pvdi:class");
        pxr::UsdAttribute classAttr = prim->CreateAttribute(classToken, pxr::SdfValueTypeNames->String, pxr::SdfVariability::SdfVariabilityUniform);
        if (classAttr)
        {
            classAttr.Set(omniPvdObject->mOmniPvdClass->mClassName);
        }

        static const pxr::TfToken handleToken("omni:pvdi:handle");
        pxr::UsdAttribute handleAttr = prim->CreateAttribute(handleToken, pxr::SdfValueTypeNames->UInt64, pxr::SdfVariability::SdfVariabilityUniform);
        if (handleAttr)
        {
            handleAttr.Set(omniPvdObject->mOmniAPIHandle);
        }

        if (omniPvdObject->mOmniObjectName.size()>0)
        {
            static const pxr::TfToken nameToken("omni:pvdi:name");
            pxr::UsdAttribute nameAttr = prim->CreateAttribute(nameToken, pxr::SdfValueTypeNames->String, pxr::SdfVariability::SdfVariabilityUniform);
            if (nameAttr)
            {
                nameAttr.Set(omniPvdObject->mOmniObjectName);
            }
        }
        static const pxr::TfToken idToken("omni:pvdi:uid");
        pxr::UsdAttribute idAttr = prim->CreateAttribute(idToken, pxr::SdfValueTypeNames->UInt64, pxr::SdfVariability::SdfVariabilityUniform);
        if (idAttr)
        {
            idAttr.Set(omniPvdObject->mUID);
        }
    }
}

void processObjectTokens(pxr::UsdPrim* prim, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;
    pxr::TfTokenVector tokens;

    const int nbrInheritedClasses = static_cast<int>(omniPvdObject->mInheritedClassInstances.size());
    for (int c = 0; c < nbrInheritedClasses; c++)
    {
        std::vector<OmniPvdAttributeInstList*>& classAttributeLists = omniPvdObject->mInheritedClassInstances[c].mClassAttributeLists;
        const int nbrAttributes = (int)classAttributeLists.size();
        for (int j = 0; j < nbrAttributes; j++)
        {
            OmniPvdAttributeInstList *attributeInstList = classAttributeLists[j];
            if (attributeInstList && attributeInstList->mAttributeDef && attributeInstList->mAttributeDef->mPxrToken)
            {
                tokens.push_back(*attributeInstList->mAttributeDef->mPxrToken);
            }
        }
    }
    prim->SetPropertyOrder(tokens);
}

void setDisplayColour(pxr::UsdPrim* prim, float r, float g, float b)
{
    pxr::UsdGeomGprim *geomPrim = (pxr::UsdGeomGprim*)prim;
    pxr::VtArray<pxr::GfVec3f> colourArrayVt;
    colourArrayVt.push_back(pxr::GfVec3f(r, g, b));
    geomPrim->GetDisplayColorAttr().Set(colourArrayVt);
}

////////////////////////////////////////////////////////////////////////////////
// For all concerned objects (first time seen (on creation) or had an attribute set)
//   For all attributes in the class of the object
//     If the attribute was not created and not set : create it
//     If the attribute was not set but just created : set a default value
//     If the attribute was set : set the value stored in the object
////////////////////////////////////////////////////////////////////////////////

void updateMaps()
{
    OmniPvd::articulationJointMotionMap.insert(std::pair<uint32_t, pxr::TfToken>(uint32_t(0), pxr::TfToken("eLOCKED")));
    OmniPvd::articulationJointMotionMap.insert(std::pair<uint32_t, pxr::TfToken>(uint32_t(1), pxr::TfToken("eLIMITED")));
    OmniPvd::articulationJointMotionMap.insert(std::pair<uint32_t, pxr::TfToken>(uint32_t(2), pxr::TfToken("eFREE")));

    OmniPvd::articulationJointDriveTypeMap.insert(std::pair<uint32_t, pxr::TfToken>(uint32_t(0), pxr::TfToken("eFORCE")));
    OmniPvd::articulationJointDriveTypeMap.insert(std::pair<uint32_t, pxr::TfToken>(uint32_t(1), pxr::TfToken("eACCELERATION")));
    OmniPvd::articulationJointDriveTypeMap.insert(std::pair<uint32_t, pxr::TfToken>(uint32_t(2), pxr::TfToken("eTARGET")));
    OmniPvd::articulationJointDriveTypeMap.insert(std::pair<uint32_t, pxr::TfToken>(uint32_t(3), pxr::TfToken("eVELOCITY")));
    OmniPvd::articulationJointDriveTypeMap.insert(std::pair<uint32_t, pxr::TfToken>(uint32_t(4), pxr::TfToken("eNONE")));
    
    OmniPvd::jointD6MotionMap.insert(std::pair<uint32_t, pxr::TfToken>(uint32_t(0), pxr::TfToken("eLOCKED")));
    OmniPvd::jointD6MotionMap.insert(std::pair<uint32_t, pxr::TfToken>(uint32_t(1), pxr::TfToken("eLIMITED")));
    OmniPvd::jointD6MotionMap.insert(std::pair<uint32_t, pxr::TfToken>(uint32_t(2), pxr::TfToken("eFREE")));
}

void createAndSetUSDAttribPass(
    pxr::UsdStageRefPtr* usdStage,
    std::list<OmniPvdObject*> &objectCreations,
    bool isSharedLayer,
    std::unordered_map<std::string, pxr::TfToken*> &tokenMap,
    int isUSDA,
    pxr::SdfLayerRefPtr& layer,
    OmniPvdDOMState& domState,
    std::string& sharedlayerIdentifier)
{
    updateMaps();

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
                pxr::UsdPrim prim = (*usdStage)->GetPrimAtPath(omniPvdObject->mPrimPath);
                if (prim) // valid USD prim object
                {
                    const OmniPvdUsdClassEnum usdClass = omniPvdClass->mUsdClassId;
                    switch (usdClass)
                    {
                    case OmniPvdUsdClassEnum::eUSDClassGeomCapsule:
                    {
                        pxr::UsdGeomCapsule geom = (pxr::UsdGeomCapsule)prim;
                        if (geom)
                        {
                            geom.GetAxisAttr().Set(OmniPvd::XToken);
                        }
                    }
                    break;
                    case OmniPvdUsdClassEnum::eUSDClassCylinder:
                    {
                        pxr::UsdGeomCylinder geom = (pxr::UsdGeomCylinder)prim;
                        if (geom)
                        {
                            geom.GetAxisAttr().Set(OmniPvd::XToken);
                        }
                    }
                    break;
                    case OmniPvdUsdClassEnum::eUSDClassCone:
                    {
                        pxr::UsdGeomCone geom = (pxr::UsdGeomCone)prim;
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
                        processMesh(&prim, 0, omniPvdObject);
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
                            prim.GetReferences().AddReference(sharedlayerIdentifier, omniPvdObject->mReferenceObject->mPrimPath,
                                pxr::SdfLayerOffset(0.0),
                                pxr::UsdListPosition::UsdListPositionBackOfPrependList);
                        }
                    }
                    break;
                    default:
                    break;
                    }

                    if ((usdClass != OmniPvdUsdClassEnum::eUSDClassGeomPoints) &&
                        (usdClass != OmniPvdUsdClassEnum::eUSDClassGeomMesh) &&
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
                                            switch (attributeInstList->mAttributeDef->mUsdAttributeId)
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
                                            if ((!processedAsCustom) && (attributeInstList->mAttributeDef->mUsdAttributeId!=OmniPvdUsdAttributeEnum::eUSDAttributeNone))
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
                                    /*
                                    else // it's a unique list or set
                                    {
                                        OmniPvdUniqueList *attrib = (OmniPvdUniqueList*)attributeInstList->mFirst;
                                        while (attrib)
                                        {
                                            switch (attributeInstList->mAttributeDef->mUsdAttributeId)
                                            {
                                            case OmniPvdUsdAttributeEnum::eUSDAttributeCustom:
                                                processCustomAttribute(prim, attrib, attributeInstList->mAttributeDef);
                                                break;
                                            }
                                            attrib = (OmniPvdUniqueList*)(attrib->mNextAttribute);
                                        }
                                    }
                                    */
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
                    if (pvdObject != domState.mSceneLayerRoot) {
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
                    if (pvdObject != domState.mSharedLayerRoot) {
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
    OmniPvdObject* sceneRoot = domState.mSceneLayerRoot;
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

void writeUSDFile(char *usdStageDir, int upAxis, int isUSDA, OmniPvdDOMState &domState)
{
    if (!usdStageDir) return;
    if (strlen(usdStageDir) == 0) return;

    std::string mOutputDir(usdStageDir);

    pxr::UsdStageRefPtr mStage;
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
    const auto& existing_layer = pxr::SdfLayer::FindOrOpen(mStageName.c_str());
    if (existing_layer)
    {
        mStage = pxr::UsdStage::Open(existing_layer);
    }
    else
    {
        mStage = pxr::UsdStage::CreateNew(mStageName.c_str());
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
        pxr::SdfChangeBlock block;
        createSDFPrimSpecPass(mSharedSublayer, rootObjects, 1);
    }
    {
        pxr::SdfChangeBlock block;
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
            pxr::UsdGeomSetStageUpAxis(mStage, pxr::UsdGeomTokens->z);
        }
        else if (fabs(gravityVec[1]) > 0.0f)
        {
            pxr::UsdGeomSetStageUpAxis(mStage, pxr::UsdGeomTokens->y);
        }
    }
    else
    {
        printf("did not find gravity\n");
        if (upAxis == 0)
        {
            pxr::UsdGeomSetStageUpAxis(mStage, pxr::UsdGeomTokens->y);
        }
        else
        {
            pxr::UsdGeomSetStageUpAxis(mStage, pxr::UsdGeomTokens->z);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Create and set attributes using USD
    ////////////////////////////////////////////////////////////////////////////////
    std::string sharedLayerIdentifier = mSharedSublayer->GetIdentifier();
    {
        pxr::SdfChangeBlock block;
        PXR_NS::UsdEditContext editCtx(mStage, mSharedSublayer);
        createAndSetUSDAttribPass(&mStage, domState.mObjectCreations, 1, domState.mTokenMap, isUSDA, mSharedSublayer, domState, sharedLayerIdentifier);
    }
    {
        pxr::SdfChangeBlock block;
        PXR_NS::UsdEditContext editCtx(mStage, mSceneSublayer);
        createAndSetUSDAttribPass(&mStage, domState.mObjectCreations, 0, domState.mTokenMap, isUSDA, mSceneSublayer, domState, sharedLayerIdentifier);
    }
    mStage->SetStartTimeCode((double)domState.mMinFrame); // Always starts at startFrame
    mStage->SetEndTimeCode((double)domState.mMaxFrame-1); // Always stops at stopFrame-1 as the last frame is one frame beyond the last simulated frame
    
    mStage->Save();

    CARB_LOG_INFO("OmniPvd USD Stage successfully saved to : %s", mStageName.c_str());
}

long int createUSDFileInMemory(int upAxis, OmniPvdDOMState& domState)
{
    // Create an anonymous layer for the stage
    //pxr::SdfLayerRefPtr rootLayer = pxr::SdfLayer::CreateAnonymous("stage.usda");
    pxr::UsdStageRefPtr stage = pxr::UsdStage::CreateInMemory();
    if (!stage)
    {
        CARB_LOG_ERROR("OmniPvd UsdStage::Open failed");
        return 0;
    }
    std::string sharedLayerName = "shared.usd";
    std::string sceneLayerName = "scene.usd";

    // Create anonymous layers for sublayers
    pxr::SdfLayerRefPtr mSceneSublayer = pxr::SdfLayer::CreateAnonymous(sceneLayerName.c_str());
    pxr::SdfLayerRefPtr mSharedSublayer = pxr::SdfLayer::CreateAnonymous(sharedLayerName.c_str());

    // Insert the layers as sub layers into/onto the Stage
    //insertAsSublayer(stage, mSharedSublayer, sharedLayerName);
    //insertAsSublayer(stage, mSceneSublayer, sceneLayerName);

    pxr::SdfLayerHandle rootLayer = stage->GetRootLayer();
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
        pxr::SdfChangeBlock block;
        createSDFPrimSpecPass(mSharedSublayer, rootObjects, 1);
    }
    {
        pxr::SdfChangeBlock block;
        createSDFPrimSpecPass(mSceneSublayer, rootObjects, 0);
    }

    rootObjects.clear();

    // Set up axis
    float gravityVec[3];
    if (getGravity(domState, gravityVec))
    {
        if (fabs(gravityVec[2]) > 0.0f)
        {
            pxr::UsdGeomSetStageUpAxis(stage, pxr::UsdGeomTokens->z);
        }
        else if (fabs(gravityVec[1]) > 0.0f)
        {
            pxr::UsdGeomSetStageUpAxis(stage, pxr::UsdGeomTokens->y);
        }
    }
    else
    {
        if (upAxis == 0)
        {
            pxr::UsdGeomSetStageUpAxis(stage, pxr::UsdGeomTokens->y);
        }
        else
        {
            pxr::UsdGeomSetStageUpAxis(stage, pxr::UsdGeomTokens->z);
        }
    }

    // Create and set attributes using USD
    std::unordered_map<std::string, pxr::TfToken*> tokenMap;
    std::string sharedLayerIdentifier = mSharedSublayer->GetIdentifier();
    {
        pxr::SdfChangeBlock block;
        PXR_NS::UsdEditContext editCtx(stage, mSharedSublayer);
        createAndSetUSDAttribPass(&stage, domState.mObjectCreations, true, tokenMap, 0, mSharedSublayer, domState, sharedLayerIdentifier);
    }
    {
        pxr::SdfChangeBlock block;
        PXR_NS::UsdEditContext editCtx(stage, mSceneSublayer);
        createAndSetUSDAttribPass(&stage, domState.mObjectCreations, false, tokenMap, 0, mSceneSublayer, domState, sharedLayerIdentifier);
    }

    stage->SetStartTimeCode((double)domState.mMinFrame); // Always starts at startFrame
    stage->SetEndTimeCode((double)domState.mMaxFrame-1); // Always stops at stopFrame-1 as the last frame is one frame beyond the last simulated frame

    // add the stage to the cache
    pxr::UsdUtilsStageCache::Get().Insert(stage);
    
    long stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    return stageId;
}

