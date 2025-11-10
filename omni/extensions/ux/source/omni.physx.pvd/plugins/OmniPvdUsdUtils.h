// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "OmniPvdDomUtils.h"

#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>

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

int getComponentByteSize(const OmniPvdDataType::Enum dataEnumVal);

bool processSpecialEnums(pxr::UsdPrim& prim, OmniPvdAttributeSample* attrib, OmniPvdAttributeDef* attribDef);

void processCustomAttribute(pxr::UsdPrim& prim, OmniPvdAttributeSample* attrib, OmniPvdAttributeDef* attribDef);

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
