// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "OmniPvdDomUtils.h"

#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <type_traits>

namespace OmniPvd
{
const pxr::TfToken XToken("X");
const pxr::TfToken YToken("Y");
const pxr::TfToken ZToken("Z");

extern pxr::TfHashMap<uint32_t, pxr::TfToken> articulationJointMotionMap;
extern pxr::TfHashMap<uint32_t, pxr::TfToken> articulationJointDriveTypeMap;
extern pxr::TfHashMap<uint32_t, pxr::TfToken> jointD6MotionMap;
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
void processConeRadius(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);
void processConeHeight(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);
void processConeAxis(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);

////////////////////////////////////////////////////////////////////////////////
// Sphere
////////////////////////////////////////////////////////////////////////////////
void processSphereRadius(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);

////////////////////////////////////////////////////////////////////////////////
// Capsule
////////////////////////////////////////////////////////////////////////////////
void processCapsuleHeight(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);
void processCapsuleRadius(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);
void processDisplayColour(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);


////////////////////////////////////////////////////////////////////////////////
// Cylinder
////////////////////////////////////////////////////////////////////////////////
void processCylinderRadius(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);
void processCylinderHeight(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);
void processCylinderAxis(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);

////////////////////////////////////////////////////////////////////////////////
// Attrib setting
////////////////////////////////////////////////////////////////////////////////
template <typename DstType, typename SrcType>
void setSingleValueAttrib(pxr::UsdAttribute& customAttr, OmniPvdAttributeSample* attrib)
{
    SrcType srcVal = *(reinterpret_cast<SrcType*>(attrib->mData));
    DstType dstVal = DstType(srcVal);
    customAttr.Set(dstVal, (double)attrib->mTimeStamp);
}

template <typename DstType, typename SrcType>
void setMultiValueAttrib(pxr::UsdAttribute& customAttr, OmniPvdAttributeSample* attrib, int nbrValsIncoming)
{
    SrcType* srcVec = reinterpret_cast<SrcType*>(attrib->mData);
    pxr::VtArray<DstType> pxrBuff;
    pxrBuff.resize(nbrValsIncoming);
    for (int i = 0; i < nbrValsIncoming; i++)
    {
        pxrBuff[i] = DstType(srcVec[i]);
    }
    customAttr.Set(pxrBuff, (double)attrib->mTimeStamp);
}

template <typename DstType, typename SrcType>
void setUniqueListAttrib(pxr::UsdAttribute& customAttr, OmniPvdUniqueList* attrib)
{
    pxr::VtArray<DstType> pxrBuff;
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
void setAttribValue(pxr::UsdAttribute& customAttr, AttribType* attrib, OmniPvdAttributeDef* attribDef, int nbrElements)
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

pxr::UsdAttribute getOrCreateAttribute(pxr::UsdPrim& prim, const pxr::TfToken& token, OmniPvdDataType::Enum dataType, bool isArray);

bool processSpecialEnums(pxr::UsdPrim& prim, OmniPvdAttributeSample* attrib, OmniPvdAttributeDef* attribDef);

template<typename AttribType>
void processCustomAttribute(pxr::UsdPrim& prim, AttribType* attrib, OmniPvdAttributeDef* attribDef)
{
    if (!prim) return;
    if (!attribDef->mPxrToken) return;

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
    pxr::UsdAttribute customAttr = getOrCreateAttribute(prim, *(attribDef->mPxrToken), dataEnumVal, isArray);
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
        setAttribValue<float, float>(customAttr, attrib, attribDef, nbrElements);
        break;
    case OmniPvdDataType::eFLOAT64:
        setAttribValue<double, double>(customAttr, attrib, attribDef, nbrElements);
        break;
    case OmniPvdDataType::eSTRING:
        // String needs special handling
        if constexpr (std::is_same_v<AttribType, OmniPvdUniqueList>)
        {
            pxr::VtArray<std::string> pxrBuff;
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

void processEnum(pxr::UsdPrim* prim,
                 OmniPvdAttributeSample* attrib,
                 OmniPvdObject* omniPvdObject,
                 OmniPvdAttributeDef* attribDef);

void processXFormFork(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);


void processTranslation(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);

void processRotation(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);

void processScale(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);

void processVisibility(pxr::UsdPrim* prim,
                       OmniPvdAttributeSample* attrib,
                       OmniPvdObject* omniPvdObject,
                       OmniPvdDOMState& domState);

void processPlane(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);

void processMesh(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);

void processPoints(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject);
