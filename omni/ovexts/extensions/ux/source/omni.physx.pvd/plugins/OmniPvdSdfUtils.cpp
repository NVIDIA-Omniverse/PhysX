// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "OmniPvdSdfUtils.h"
#include "PvdDomUtils.h"
#include "OmniPvdUsdAdapter.h"

#include <carb/logging/Log.h>

void addCleanName(std::string& primPath, char* objectName)
{
    if (!objectName) return;
    const int maxBuffLen = 512;
    char tmpBuffer[maxBuffLen];
    int buffStep = 0;
    int nbrValidChars = 0;
    while ((*objectName) && (buffStep < (maxBuffLen - 1)))
    {
        char currentChar = *objectName;
        if ( std::isdigit(static_cast<unsigned char>(currentChar)) || std::isalpha(static_cast<unsigned char>(currentChar)) )
        {
            tmpBuffer[buffStep] = currentChar;
            nbrValidChars++;
        }
        else
        {
            tmpBuffer[buffStep] = '_';
        }
        buffStep++;
        objectName++;
    }
    tmpBuffer[buffStep] = 0;
    if (nbrValidChars > 0)
    {
        if (tmpBuffer[0] != '_')
        {
            primPath.append("_");
        }
        primPath.append(tmpBuffer);
    }
}

const char* boolString(bool isTrue)
{
    return isTrue ? "true" : "false";
}

void createSDFPrimSpec(PXR_NS::SdfLayerRefPtr& layer, OmniPvdObject* omniPvdObject, bool isSharedLayer)
{
    static int32_t pxActorNameAttribIndex = -1;
    static int32_t pxActorNameClassIndex = -1;

    if (omniPvdObject->mAncestor)
    {
        omniPvdObject->mIsShared = omniPvdObject->mAncestor->mIsShared;
    }

    if ((!omniPvdObject->mWasSDFCreated) && (isSharedLayer == omniPvdObject->mIsShared))
    {
        omniPvdObject->mWasSDFCreated = true;
        OmniPvdClass *omniPvdClass = omniPvdObject->mOmniPvdClass;

        std::string primPathRelative;
        if (omniPvdObject->mObjectName.size() > 0)
        {
            primPathRelative = omniPvdObject->mObjectName;
        }
        else
        {
            if ((OmniPvdUsd::getClassId(omniPvdClass) == OmniPvdUsdClassEnum::eUSDClassOver) && omniPvdObject->mReferenceObject)
            {
                if (omniPvdObject->mReferenceObject->mOmniObjectHandle > 0)
                {
                    primPathRelative = omniPvdObject->mReferenceObject->mOmniPvdClass->mClassName + "_" + std::to_string(omniPvdObject->mReferenceObject->mOmniObjectHandle);
                }
                else
                {
                    primPathRelative = omniPvdObject->mReferenceObject->mOmniPvdClass->mClassName + "_ref_" + std::to_string(omniPvdObject->mReferenceObject->mOmniAPIHandle);
                }
            }
            else
            {
                primPathRelative = (omniPvdObject->mOmniObjectHandle > 0) ? (omniPvdClass->mClassName + "_" + std::to_string(omniPvdObject->mOmniObjectHandle)) : (omniPvdClass->mClassName);
            }
        }        

        if (isSameString(omniPvdClass->mClassName.c_str(),"actor") ||
            isSameString(omniPvdClass->mClassName.c_str(), "PxActor") ||
            (omniPvdClass->mPhysXBaseProcessingClassId == OmniPvdPhysXClassEnum::ePxActor))
        {
            char* objectName = (char*)getAttribData(pxActorNameAttribIndex, pxActorNameClassIndex, "name", omniPvdObject);
            addCleanName(primPathRelative, objectName);
        }
        else
        {
        }

        if (omniPvdObject->mAncestor && (omniPvdObject->mAncestor != omniPvdObject))
        {
            PXR_NS::SdfPath combinedPath = OmniPvdUsd::getPrimPath(omniPvdObject->mAncestor);
            OmniPvdUsd::setPrimPath(omniPvdObject, combinedPath.AppendElementString(primPathRelative));
        }
        else {
            OmniPvdUsd::setPrimPath(omniPvdObject, PXR_NS::SdfPath("/" + primPathRelative));
        }
        PXR_NS::SdfPrimSpecHandle primSpec = PXR_NS::SdfCreatePrimInLayer(layer, OmniPvdUsd::getPrimPath(omniPvdObject));
        if (primSpec)
        {
            switch (OmniPvdUsd::getClassId(omniPvdClass))
            {
                case OmniPvdUsdClassEnum::eUSDClassXform:
                {
                    primSpec->SetSpecifier(PXR_NS::SdfSpecifierDef);
                    primSpec->SetTypeName("Xform");
                }
                break;
                case OmniPvdUsdClassEnum::eUSDClassGeomScope:
                {
                    primSpec->SetSpecifier(PXR_NS::SdfSpecifierDef);
                    primSpec->SetTypeName("Scope");
                }
                break;
                case OmniPvdUsdClassEnum::eUSDClassGeomSphere:
                {
                    primSpec->SetSpecifier(PXR_NS::SdfSpecifierDef);
                    primSpec->SetTypeName("Sphere");
                }
                break;
                case OmniPvdUsdClassEnum::eUSDClassGeomCapsule:
                {
                    primSpec->SetSpecifier(PXR_NS::SdfSpecifierDef);
                    primSpec->SetTypeName("Capsule");
                }
                break;
                case OmniPvdUsdClassEnum::eUSDClassGeomCube:
                {
                    primSpec->SetSpecifier(PXR_NS::SdfSpecifierDef);
                    primSpec->SetTypeName("Cube");
                }
                break;
                case OmniPvdUsdClassEnum::eUSDClassGeomPlane:
                {
                    primSpec->SetSpecifier(PXR_NS::SdfSpecifierDef);
                    primSpec->SetTypeName("Mesh");
                }
                break;
                case OmniPvdUsdClassEnum::eUSDClassGeomMesh:
                {
                    primSpec->SetSpecifier(PXR_NS::SdfSpecifierDef);
                    primSpec->SetTypeName("Mesh");
                }
                break;
                case OmniPvdUsdClassEnum::eUSDClassGeomTetMesh:
                {
                    // Prim type is Mesh for rendering; tet data stored as additional attributes
                    primSpec->SetSpecifier(PXR_NS::SdfSpecifierDef);
                    primSpec->SetTypeName("Mesh");
                }
                break;
                case OmniPvdUsdClassEnum::eUSDClassGeomPoints:
                {
                    primSpec->SetSpecifier(PXR_NS::SdfSpecifierDef);
                    primSpec->SetTypeName("Points");
                }
                break;
                case OmniPvdUsdClassEnum::eUSDClassOver:
                {
                    primSpec->SetSpecifier(PXR_NS::SdfSpecifierOver);
                }
                break;
                case OmniPvdUsdClassEnum::eUSDClassDistantLight:
                {
                    primSpec->SetSpecifier(PXR_NS::SdfSpecifierDef);
                    primSpec->SetTypeName("DistantLight");
                }
                break;
                case OmniPvdUsdClassEnum::eUSDClassCylinder:
                {
                    primSpec->SetSpecifier(PXR_NS::SdfSpecifierDef);
                    primSpec->SetTypeName("Cylinder");
                }
                break;
                case OmniPvdUsdClassEnum::eUSDClassCone:
                {
                    primSpec->SetSpecifier(PXR_NS::SdfSpecifierDef);
                    primSpec->SetTypeName("Cone");
                }
                break;
                default:
                    break;
            }
            OmniPvdObject *childObject = omniPvdObject->mFirstChild;
            while (childObject)
            {
                createSDFPrimSpec(layer, childObject, isSharedLayer);
                childObject = childObject->mNextSibling;
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// For all objects that were created and that have a USD equivalent
// 1) Create the SDF primitive path
// 2) Create the SDF primitive specifier and set its type
////////////////////////////////////////////////////////////////////////////////
void createSDFPrimSpecPass(PXR_NS::SdfLayerRefPtr& layer, std::list<OmniPvdObject*> &objectCreations, bool isSharedLayer)
{
    std::list<OmniPvdObject*>::iterator it;
    for (it = objectCreations.begin(); it != objectCreations.end(); it++)
    {
        createSDFPrimSpec(layer, (*it) , isSharedLayer);
    }
}

void createAndClearLayer(PXR_NS::SdfLayerRefPtr& sublayer, std::string& sublayerName)
{
    sublayer = PXR_NS::SdfLayer::FindOrOpen(sublayerName);
    if (sublayer)
    {
        sublayer->Clear();
        sublayer->Save();
    }
    else
    {
        sublayer = PXR_NS::SdfLayer::CreateNew(sublayerName);
        if (!sublayer)
        {
            CARB_LOG_ERROR("Sublayer creation failed: %s)", sublayerName.c_str());
        }
    }
}

void insertAsSublayer(PXR_NS::UsdStageRefPtr& stage, PXR_NS::SdfLayerRefPtr& sublayer, std::string& sublayerName)
{
    PXR_NS::SdfLayerHandle rootLayer = stage->GetRootLayer();
    bool alreadyExists = false;
    auto sublayers = rootLayer->GetSubLayerPaths();
    for (auto it = sublayers.begin(); it != sublayers.end(); ++it)
    {
        const std::string& path = *it;
        if (path == sublayerName)
        {
            alreadyExists = true;
            break;
        }
    }
    if (!alreadyExists)
    {
        rootLayer->InsertSubLayerPath(sublayerName);
    }
}
