// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "PvdDomUtils.h"
#include "OmniPvdUsdAdapter.h"

#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <type_traits>

namespace OmniPvd
{
const PXR_NS::TfToken XToken("X");
const PXR_NS::TfToken YToken("Y");
const PXR_NS::TfToken ZToken("Z");

extern PXR_NS::TfHashMap<uint32_t, PXR_NS::TfToken> articulationJointMotionMap;
extern PXR_NS::TfHashMap<uint32_t, PXR_NS::TfToken> articulationJointDriveTypeMap;
extern PXR_NS::TfHashMap<uint32_t, PXR_NS::TfToken> jointD6MotionMap;
}; // namespace OmniPvd

////////////////////////////////////////////////////////////////////////////////
// For geometry processing
////////////////////////////////////////////////////////////////////////////////
float getRandF();
void setFloatVec3(float* dstVec, const float* srcVec);
void setFloatVec3(float* dstVec, float x, float y, float z);
float minFloat(float a, float b);
float maxFloat(float a, float b);
void minFloatVec3(float* minVec, float x, float y, float z);
void minFloatVec3(float* minVec, float* compare);
void maxFloatVec3(float* maxVec, float x, float y, float z);
void maxFloatVec3(float* maxVec, float* compare);
void addFloatVec3(float* dstVec, float x, float y, float z);
void subFloatVec3(float* r, float* a, float* b);
void crossFloatVec3(float* n, float* t1, float* t2);
void normalizeFloatVec3(float* n, float* a);

// int indices : vector of vertex indices that constitute one triangle = [t0(v0,v1,v2), t1(v0,v1,v2) ...]
void getNormals(float* normals, int nbrNormals, float* vertices, int* indices, int nbrTriangles);

////////////////////////////////////////////////////////////////////////////////
// Cone
////////////////////////////////////////////////////////////////////////////////
void processConeRadius(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);
void processConeHeight(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);
void processConeAxis(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);

////////////////////////////////////////////////////////////////////////////////
// Sphere
////////////////////////////////////////////////////////////////////////////////
void processSphereRadius(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);

////////////////////////////////////////////////////////////////////////////////
// Capsule
////////////////////////////////////////////////////////////////////////////////
void processCapsuleHeight(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);
void processCapsuleRadius(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);
void processDisplayColour(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);


////////////////////////////////////////////////////////////////////////////////
// Cylinder
////////////////////////////////////////////////////////////////////////////////
void processCylinderRadius(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);
void processCylinderHeight(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);
void processCylinderAxis(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);

////////////////////////////////////////////////////////////////////////////////
// Attrib setting
////////////////////////////////////////////////////////////////////////////////
template <typename DstType, typename SrcType>
void setSingleValueAttrib(PXR_NS::UsdAttribute& customAttr, OmniPvdAttributeSample* attrib)
{
    SrcType srcVal = *(reinterpret_cast<SrcType*>(attrib->mData));
    DstType dstVal = DstType(srcVal);
    customAttr.Set(dstVal, (double)attrib->mTimeStamp);
}

template <typename DstType, typename SrcType>
void setMultiValueAttrib(PXR_NS::UsdAttribute& customAttr, OmniPvdAttributeSample* attrib, int nbrValsIncoming)
{
    SrcType* srcVec = reinterpret_cast<SrcType*>(attrib->mData);
    PXR_NS::VtArray<DstType> pxrBuff;
    pxrBuff.resize(nbrValsIncoming);
    for (int i = 0; i < nbrValsIncoming; i++)
    {
        pxrBuff[i] = DstType(srcVec[i]);
    }
    customAttr.Set(pxrBuff, (double)attrib->mTimeStamp);
}

template <typename DstType, typename SrcType>
void setUniqueListAttrib(PXR_NS::UsdAttribute& customAttr, OmniPvdUniqueList* attrib)
{
    PXR_NS::VtArray<DstType> pxrBuff;
    pxrBuff.resize(attrib->mElements.size());
    int idx = 0;
    for (auto it = attrib->mElements.begin(); it != attrib->mElements.end(); ++it, ++idx)
    {
        pxrBuff[idx] = static_cast<DstType>(*reinterpret_cast<SrcType*>((*it)->mData));
    }
    customAttr.Set(pxrBuff, static_cast<double>(attrib->mTimeStamp));
}

// Helper template to set attribute values based on attribute type
template<typename DstType, typename SrcType, typename AttribType>
void setAttribValue(PXR_NS::UsdAttribute& customAttr, AttribType* attrib, OmniPvdAttributeDef* attribDef, int nbrElements)
{
    if constexpr (std::is_same_v<AttribType, OmniPvdUniqueList>)
    {
        setUniqueListAttrib<DstType, SrcType>(customAttr, attrib);
    }
    else
    {
        if (attribDef->mNbrFields == 1)
        {
            setSingleValueAttrib<DstType, SrcType>(customAttr, attrib);
        }
        else
        {
            setMultiValueAttrib<DstType, SrcType>(customAttr, attrib, nbrElements);
        }
    }
}

int getComponentByteSize(const OmniPvdDataType::Enum dataEnumVal);

PXR_NS::UsdAttribute getOrCreateAttribute(PXR_NS::UsdPrim& prim, const PXR_NS::TfToken& token, OmniPvdDataType::Enum dataType, bool isArray);

bool processSpecialEnums(PXR_NS::UsdPrim& prim, OmniPvdAttributeSample* attrib, OmniPvdAttributeDef* attribDef);

template<typename AttribType>
void processCustomAttribute(PXR_NS::UsdPrim& prim, AttribType* attrib, OmniPvdAttributeDef* attribDef)
{
    if (!prim) return;
    if (!OmniPvdUsd::getToken(attribDef)) return;

    // Special enum handling (only for OmniPvdAttributeSample)
    if constexpr (std::is_same_v<AttribType, OmniPvdAttributeSample>)
    {
        if (processSpecialEnums(prim, attrib, attribDef))
        {
            return;
        }
    }

    const OmniPvdDataType::Enum dataEnumVal = OmniPvdDataType::Enum(attribDef->mDataType);
    const int componentByteSize = getComponentByteSize(dataEnumVal);

    // Check if empty and get element count
    int nbrElements = 0;
    if constexpr (std::is_same_v<AttribType, OmniPvdUniqueList>)
    {
        if (attrib->mElements.empty()) return;
        nbrElements = static_cast<int>(attrib->mElements.size());
    }
    else
    {
        nbrElements = attrib->mDataLen / componentByteSize;
        if (nbrElements < 1) return;
    }

    // Determine if this should be a single value or array
    // Unique lists are always arrays, OmniPvdAttributeSample depends on mNbrFields
    bool isArray;
    if constexpr (std::is_same_v<AttribType, OmniPvdUniqueList>)
    {
        isArray = true;
    }
    else
    {
        isArray = attribDef->mNbrFields != 1;
    }

    // Get or create attribute
    PXR_NS::UsdAttribute customAttr = getOrCreateAttribute(prim, *OmniPvdUsd::getToken(attribDef), dataEnumVal, isArray);
    if (!customAttr) return;

    // Set values based on data type
    switch (dataEnumVal)
    {
    case OmniPvdDataType::eINT8:
        setAttribValue<int32_t, int8_t>(customAttr, attrib, attribDef, nbrElements);
        break;
    case OmniPvdDataType::eINT16:
        setAttribValue<int32_t, int16_t>(customAttr, attrib, attribDef, nbrElements);
        break;
    case OmniPvdDataType::eINT32:
        setAttribValue<int32_t, int32_t>(customAttr, attrib, attribDef, nbrElements);
        break;
    case OmniPvdDataType::eINT64:
        setAttribValue<int64_t, int64_t>(customAttr, attrib, attribDef, nbrElements);
        break;
    case OmniPvdDataType::eUINT8:
        setAttribValue<uint32_t, uint8_t>(customAttr, attrib, attribDef, nbrElements);
        break;
    case OmniPvdDataType::eUINT16:
        setAttribValue<uint32_t, uint16_t>(customAttr, attrib, attribDef, nbrElements);
        break;
    case OmniPvdDataType::eUINT32:
        setAttribValue<uint32_t, uint32_t>(customAttr, attrib, attribDef, nbrElements);
        break;
    case OmniPvdDataType::eUINT64:
    case OmniPvdDataType::eOBJECT_HANDLE:
        setAttribValue<uint64_t, uint64_t>(customAttr, attrib, attribDef, nbrElements);
        break;
    case OmniPvdDataType::eFLOAT32:
        // USD schema attributes (radius, height) expect double; OVD stores float.
        // Check the existing attribute type and promote if needed.
        if (customAttr.GetTypeName() == PXR_NS::SdfValueTypeNames->Double ||
            customAttr.GetTypeName() == PXR_NS::SdfValueTypeNames->DoubleArray)
            setAttribValue<double, float>(customAttr, attrib, attribDef, nbrElements);
        else
            setAttribValue<float, float>(customAttr, attrib, attribDef, nbrElements);
        break;
    case OmniPvdDataType::eFLOAT64:
        setAttribValue<double, double>(customAttr, attrib, attribDef, nbrElements);
        break;
    case OmniPvdDataType::eSTRING:
        // String needs special handling
        if constexpr (std::is_same_v<AttribType, OmniPvdUniqueList>)
        {
            PXR_NS::VtArray<std::string> pxrBuff;
            pxrBuff.resize(attrib->mElements.size());
            int idx = 0;
            for (auto it = attrib->mElements.begin(); it != attrib->mElements.end(); ++it, ++idx)
            {
                pxrBuff[idx] = std::string(reinterpret_cast<char*>((*it)->mData));
            }
            customAttr.Set(pxrBuff, static_cast<double>(attrib->mTimeStamp));
        }
        else
        {
            if (attribDef->mNbrFields == 1)
            {
                char* srcVal = reinterpret_cast<char*>(attrib->mData);
                std::string dstVal(srcVal);
                customAttr.Set(dstVal, static_cast<double>(attrib->mTimeStamp));
            }
        }
        break;
    default:
        break;
    }
}

void processEnum(PXR_NS::UsdPrim* prim,
                 OmniPvdAttributeSample* attrib,
                 OmniPvdObject* omniPvdObject,
                 OmniPvdAttributeDef* attribDef);

void processXFormFork(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);


void processTranslation(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);

void processRotation(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);

void processScale(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);

void processVisibility(PXR_NS::UsdPrim* prim,
                       OmniPvdAttributeSample* attrib,
                       OmniPvdObject* omniPvdObject,
                       OmniPvdDOMState& domState);

void processPlane(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);

void processMesh(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);

void processPoints(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);

void processTetMesh(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);
