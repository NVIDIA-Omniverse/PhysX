// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "OmniPvdUsdUtils.h"

#include "foundation/PxTransform.h"

namespace OmniPvd
{
pxr::TfHashMap<uint32_t, pxr::TfToken> articulationJointMotionMap;
pxr::TfHashMap<uint32_t, pxr::TfToken> articulationJointDriveTypeMap;
pxr::TfHashMap<uint32_t, pxr::TfToken> jointD6MotionMap;
};

float getRandF() {
    return (float)((double)rand() / (double)(RAND_MAX));
}

void setFloatVec3(float* dstVec, const float *srcVec) {
    memcpy(dstVec, srcVec, sizeof(float) * 3);
}

void setFloatVec3(float* dstVec, float x, float y, float z) {
    dstVec[0] = x;
    dstVec[1] = y;
    dstVec[2] = z;
}

float minFloat(float a, float b) {
    return a < b ? a : b;
}

float maxFloat(float a, float b) {
    return a > b ? a : b;
}

void minFloatVec3(float* minVec, float x, float y, float z) {
    setFloatVec3(minVec, minFloat(minVec[0], x), minFloat(minVec[1], y), minFloat(minVec[2], z));
}

void minFloatVec3(float* minVec, float *compare) {
    setFloatVec3(minVec, minFloat(minVec[0], compare[0]), minFloat(minVec[1], compare[1]), minFloat(minVec[2], compare[2]));
}

void maxFloatVec3(float* maxVec, float x, float y, float z) {
    setFloatVec3(maxVec, maxFloat(maxVec[0], x), maxFloat(maxVec[1], y), maxFloat(maxVec[2], z));
}

void maxFloatVec3(float* maxVec, float *compare) {
    setFloatVec3(maxVec, maxFloat(maxVec[0], compare[0]), maxFloat(maxVec[1], compare[1]), maxFloat(maxVec[2], compare[2]));
}

void addFloatVec3(float* dstVec, float x, float y, float z) {
    dstVec[0] += x;
    dstVec[1] += y;
    dstVec[2] += z;
}

void subFloatVec3(float* r, float* a, float *b) {
    r[0] = a[0] - b[0];
    r[1] = a[1] - b[1];
    r[2] = a[2] - b[2];
}
void crossFloatVec3(float *n, float *t1, float *t2) {
    n[0] = t1[1] * t2[2] - t1[2] * t2[1];
    n[1] = t1[2] * t2[0] - t1[0] * t2[2];
    n[2] = t1[0] * t2[1] - t1[1] * t2[0];
}

void normalizeFloatVec3(float *n, float *a) {
    float distInv = 1.0f / sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    n[0] = a[0] * distInv;
    n[1] = a[1] * distInv;
    n[2] = a[2] * distInv;
}

// int indices : vector of vertex indices that constitute one triangle = [t0(v0,v1,v2), t1(v0,v1,v2) ...]
void getNormals(float *normals, int nbrNormals, float *vertices, int *indices, int nbrTriangles) {
    memset(normals, 0, sizeof(float) * 3 * nbrNormals);
    for (int t = 0; t < nbrTriangles; t++) {
        const int v0 = indices[t * 3 + 0] * 3;
        const int v1 = indices[t * 3 + 1] * 3;
        const int v2 = indices[t * 3 + 2] * 3;
        float *vert0 = &vertices[v0 + 0];
        float *vert1 = &vertices[v1 + 0];
        float *vert2 = &vertices[v2 + 0];

        float vec1[3];
        float vec2[3];
        float norm[3];
        subFloatVec3(vec1, vert1, vert0);
        subFloatVec3(vec2, vert2, vert0);
        crossFloatVec3(norm, vec1, vec2);

        addFloatVec3(&normals[v0], norm[0], norm[1], norm[2]);
        addFloatVec3(&normals[v1], norm[0], norm[1], norm[2]);
        addFloatVec3(&normals[v2], norm[0], norm[1], norm[2]);
    }
    for (int v = 0; v < nbrNormals; v++) {
        float *norm = &normals[v * 3];
        normalizeFloatVec3(norm, norm);
    }
}

////////////////////////////////////////////////////////////////////////////////
// Cone
////////////////////////////////////////////////////////////////////////////////
void processConeRadius(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;

    pxr::UsdGeomCone *geom = (pxr::UsdGeomCone*)prim;
    float radius = *((float*)attrib->mData);
    geom->GetRadiusAttr().Set(double(radius));
}

void processConeHeight(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;

    pxr::UsdGeomCone *geom = (pxr::UsdGeomCone*)prim;
    float height = *((float*)attrib->mData);
    geom->GetHeightAttr().Set(double(height));
}

void processConeAxis(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;

    pxr::UsdGeomCone *geom = (pxr::UsdGeomCone*)prim;
    int axis = *((int*)attrib->mData);
    if (axis ==  0) {
        geom->GetAxisAttr().Set(OmniPvd::XToken);
    } else if (axis ==  1) {
        geom->GetAxisAttr().Set(OmniPvd::YToken);
    } else if (axis ==  2) {
        geom->GetAxisAttr().Set(OmniPvd::ZToken);
    }
}

////////////////////////////////////////////////////////////////////////////////
// Sphere
////////////////////////////////////////////////////////////////////////////////
void processSphereRadius(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;

    pxr::UsdGeomSphere *geom = (pxr::UsdGeomSphere*)prim;
    float radius = *((float*)attrib->mData);
    geom->GetRadiusAttr().Set(double(radius));
}

////////////////////////////////////////////////////////////////////////////////
// Capsule
////////////////////////////////////////////////////////////////////////////////
void processCapsuleHeight(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;

    pxr::UsdGeomCapsule *geom = (pxr::UsdGeomCapsule*)prim;
    float height = *((float*)attrib->mData);
    geom->GetHeightAttr().Set(2.0*double(height));
}

void processCapsuleRadius(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;

    pxr::UsdGeomCapsule *geom = (pxr::UsdGeomCapsule*)prim;
    float radius = *((float*)attrib->mData);
    geom->GetRadiusAttr().Set(double(radius));
}

void processDisplayColour(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;

    pxr::UsdGeomGprim *geomPrim = (pxr::UsdGeomGprim*)prim;
    float *col = (float*)attrib->mData;
    pxr::VtArray<pxr::GfVec3f> colourArrayVt;
    colourArrayVt.push_back(pxr::GfVec3f(col[0], col[1], col[2]));
    geomPrim->GetDisplayColorAttr().Set(colourArrayVt);
}

////////////////////////////////////////////////////////////////////////////////
// Cylinder
////////////////////////////////////////////////////////////////////////////////
void processCylinderRadius(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;

    pxr::UsdGeomCylinder *geom = (pxr::UsdGeomCylinder*)prim;
    float radius = *((float*)attrib->mData);
    geom->GetRadiusAttr().Set(double(radius));
}

void processCylinderHeight(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;

    pxr::UsdGeomCylinder *geom = (pxr::UsdGeomCylinder*)prim;
    float height = *((float*)attrib->mData);
    geom->GetHeightAttr().Set(double(height));
}

void processCylinderAxis(pxr::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;

    pxr::UsdGeomCylinder *geom = (pxr::UsdGeomCylinder*)prim;
    int axis = *((int*)attrib->mData);
    if (axis ==  0) {
        geom->GetAxisAttr().Set(OmniPvd::XToken);
    } else if (axis ==  1) {
        geom->GetAxisAttr().Set(OmniPvd::YToken);
    } else if (axis ==  2) {
        geom->GetAxisAttr().Set(OmniPvd::ZToken);
    }
}

int getComponentByteSize(const OmniPvdDataType::Enum dataEnumVal)
{
    int bytesPerComponent = 1;
    switch (dataEnumVal)
    {
    case OmniPvdDataType::eINT8:
    case OmniPvdDataType::eUINT8:
    {
        bytesPerComponent = 1;
    }
    break;
    case OmniPvdDataType::eINT16:
    case OmniPvdDataType::eUINT16:
    {
        bytesPerComponent = 2;
    }
    break;
    case OmniPvdDataType::eINT32:
    case OmniPvdDataType::eUINT32:
    {
        bytesPerComponent = 4;
    }
    break;
    case OmniPvdDataType::eINT64:
    case OmniPvdDataType::eUINT64:
    {
        bytesPerComponent = 8;
    }
    break;
    case OmniPvdDataType::eFLOAT32:
    {
        bytesPerComponent = 4;
    }
    break;
    case OmniPvdDataType::eFLOAT64:
    {
        bytesPerComponent = 8;
    }
    break;
    case OmniPvdDataType::eSTRING:
    {
        bytesPerComponent = 1;
    }
    break;
    case OmniPvdDataType::eOBJECT_HANDLE:
    {
        bytesPerComponent = 8;
    }
    break;
    case OmniPvdDataType::eENUM_VALUE:
    {
        bytesPerComponent = 4; // Double check this!
    }
    break;
    case OmniPvdDataType::eFLAGS_WORD:
    {
        bytesPerComponent = 4; // Double check this!
    }
    break;
    default:
    {
        bytesPerComponent = 1;
    }
    break;
    }
    return bytesPerComponent;
}

using namespace pxr;


bool processSpecialEnums(
    pxr::UsdPrim &prim,
    OmniPvdAttributeSample *attrib,
    OmniPvdAttributeDef* attribDef
)
{
    bool isSpecialProcessing = false;

    if (!prim) return isSpecialProcessing;

    TfHashMap<uint32_t, TfToken> *mapPtr;
    
    if (isSameString(attribDef->mClass->mClassName.c_str(), "PxArticulationJointReducedCoordinate"))
    {
        if (isSameString(attribDef->mAttributeName.c_str(), "motion"))
        {
            mapPtr = &OmniPvd::articulationJointMotionMap;
            isSpecialProcessing = true;
        }
        else if (isSameString(attribDef->mAttributeName.c_str(), "driveType"))
        {
            mapPtr = &OmniPvd::articulationJointDriveTypeMap;
            isSpecialProcessing = true;
        }
    }
    else if (isSameString(attribDef->mClass->mClassName.c_str(), "PxJoint"))
    {
        if (isSameString(attribDef->mAttributeName.c_str(), "motions") || isSameString(attribDef->mAttributeName.c_str(), "d6Motions"))
        {
            mapPtr = &OmniPvd::jointD6MotionMap;
            isSpecialProcessing = true;
        }
    }

    if (isSpecialProcessing)
    {
        const OmniPvdDataType::Enum dataEnumVal = OmniPvdDataType::Enum(attribDef->mDataType);
        int componentByteSize = getComponentByteSize(dataEnumVal);
        int nbrValsIncoming = attrib->mDataLen / componentByteSize;
        if (nbrValsIncoming >= 1)
        {
            pxr::UsdAttribute customAttr = prim.GetAttribute(*(attribDef->mPxrToken));
            /////////////////////////////////////////////////////////////////////////////////
            // Create the attribute if it was not already created
            /////////////////////////////////////////////////////////////////////////////////
            if (!customAttr)
            {
                customAttr = prim.CreateAttribute(*(attribDef->mPxrToken), pxr::SdfValueTypeNames->TokenArray);
            }
            VtArray<TfToken> tokenArray;
            int nbrMappings = 0;
            uint32_t * enumData = (uint32_t*)attrib->mData;
            for (int i=0; i < nbrValsIncoming; i++)
            {
                uint32_t enumVal = enumData[i];
                auto mapIt = mapPtr->find(enumVal);
                if (mapIt != mapPtr->end())
                {
                    const TfToken enumToken = mapIt->second;
                    // map the enum value to a string
                    tokenArray.push_back(enumToken);
                    nbrMappings++;
                }                
            }
            if (nbrMappings)
            {
                customAttr.Set(tokenArray, (double)attrib->mTimeStamp);
            }
        }
    }

    return isSpecialProcessing;
}

void processCustomAttribute(
    pxr::UsdPrim &prim,
    OmniPvdAttributeSample *attrib,
    OmniPvdAttributeDef* attribDef
)
{
    if (!prim) return;
    if (!attribDef->mPxrToken)
    {
        // should never happen
        return;
    }
    if (processSpecialEnums(prim, attrib, attribDef))
    {
        return;
    }
    // = attributeInstList->mAttributeDef;
    const OmniPvdDataType::Enum dataEnumVal = OmniPvdDataType::Enum(attribDef->mDataType);
    int componentByteSize = getComponentByteSize(dataEnumVal);
    int nbrValsIncoming = attrib->mDataLen / componentByteSize;
    if (nbrValsIncoming >= 1)
    {
        pxr::UsdAttribute customAttr = prim.GetAttribute(*(attribDef->mPxrToken));
        /////////////////////////////////////////////////////////////////////////////////
        // Create the attribute if it was not already created
        /////////////////////////////////////////////////////////////////////////////////
        if (!customAttr)
        {
            switch (dataEnumVal)
            {
            case OmniPvdDataType::eINT8:
            case OmniPvdDataType::eINT16:
            case OmniPvdDataType::eINT32:
            {
                if (attribDef->mNbrFields == 1)
                {
                    customAttr = prim.CreateAttribute(*(attribDef->mPxrToken), pxr::SdfValueTypeNames->Int);
                }
                else
                {
                    customAttr = prim.CreateAttribute(*(attribDef->mPxrToken), pxr::SdfValueTypeNames->IntArray);
                }
            }
            break;
            case OmniPvdDataType::eINT64:
            {
                if (attribDef->mNbrFields == 1)
                {
                    customAttr = prim.CreateAttribute(*(attribDef->mPxrToken), pxr::SdfValueTypeNames->Int64);
                }
                else
                {
                    customAttr = prim.CreateAttribute(*(attribDef->mPxrToken), pxr::SdfValueTypeNames->Int64Array);
                }
            }
            break;
            case OmniPvdDataType::eUINT8:
            case OmniPvdDataType::eUINT16:
            case OmniPvdDataType::eUINT32:
            {
                if (attribDef->mNbrFields == 1)
                {
                    customAttr = prim.CreateAttribute(*(attribDef->mPxrToken), pxr::SdfValueTypeNames->UInt);
                }
                else
                {
                    customAttr = prim.CreateAttribute(*(attribDef->mPxrToken), pxr::SdfValueTypeNames->UIntArray);
                }
            }
            break;
            case OmniPvdDataType::eUINT64:
            {
                if (attribDef->mNbrFields == 1)
                {
                    customAttr = prim.CreateAttribute(*(attribDef->mPxrToken), pxr::SdfValueTypeNames->UInt64);
                }
                else
                {
                    customAttr = prim.CreateAttribute(*(attribDef->mPxrToken), pxr::SdfValueTypeNames->UInt64Array);
                }
            }
            break;
            case OmniPvdDataType::eFLOAT32:
            {
                if (attribDef->mNbrFields == 1)
                {
                    customAttr = prim.CreateAttribute(*(attribDef->mPxrToken), pxr::SdfValueTypeNames->Float);
                }
                else
                {
                    customAttr = prim.CreateAttribute(*(attribDef->mPxrToken), pxr::SdfValueTypeNames->FloatArray);
                }
            }
            break;
            case OmniPvdDataType::eFLOAT64:
            {
                if (attribDef->mNbrFields == 1)
                {
                    customAttr = prim.CreateAttribute(*(attribDef->mPxrToken), pxr::SdfValueTypeNames->Double);
                }
                else
                {
                    customAttr = prim.CreateAttribute(*(attribDef->mPxrToken), pxr::SdfValueTypeNames->DoubleArray);
                }
            }
            break;
            case OmniPvdDataType::eSTRING:
            {
                if (attribDef->mNbrFields == 1)
                {
                    customAttr = prim.CreateAttribute(*(attribDef->mPxrToken), pxr::SdfValueTypeNames->String);
                }
                else
                {
                    customAttr = prim.CreateAttribute(*(attribDef->mPxrToken), pxr::SdfValueTypeNames->StringArray);
                }

            }
            break;
            case OmniPvdDataType::eOBJECT_HANDLE:
            {
                if (attribDef->mNbrFields == 1)
                {
                    customAttr = prim.CreateAttribute(*(attribDef->mPxrToken), pxr::SdfValueTypeNames->UInt64);
                }
                else
                {
                    customAttr = prim.CreateAttribute(*(attribDef->mPxrToken), pxr::SdfValueTypeNames->UInt64Array);
                }
            }
            break;
            // What about enum?
            default:
                break;
            }
        }

        if (customAttr)
        {
            switch (dataEnumVal)
            {
            case OmniPvdDataType::eINT8:
            {
                if (attribDef->mNbrFields == 1)
                {
                    setSingleValueAttrib<int32_t, int8_t>(customAttr, attrib);
                }
                else
                {
                    setMultiValueAttrib<int32_t, int8_t>(customAttr, attrib, nbrValsIncoming);
                }
            }
            break;
            case OmniPvdDataType::eINT16:
            {
                if (attribDef->mNbrFields == 1)
                {
                    setSingleValueAttrib<int32_t, int16_t>(customAttr, attrib);
                }
                else
                {
                    setMultiValueAttrib<int32_t, int16_t>(customAttr, attrib, nbrValsIncoming);
                }
            }
            case OmniPvdDataType::eINT32:
            {
                if (attribDef->mNbrFields == 1)
                {
                    setSingleValueAttrib<int32_t, int32_t>(customAttr, attrib);
                }
                else
                {
                    setMultiValueAttrib<int32_t, int32_t>(customAttr, attrib, nbrValsIncoming);
                }
            }
            break;
            case OmniPvdDataType::eINT64:
            {
                if (attribDef->mNbrFields == 1)
                {
                    setSingleValueAttrib<int64_t, int64_t>(customAttr, attrib);
                }
                else
                {
                    setMultiValueAttrib<int64_t, int64_t>(customAttr, attrib, nbrValsIncoming);
                }
            }
            break;
            case OmniPvdDataType::eUINT8:
            {
                if (attribDef->mNbrFields == 1)
                {
                    setSingleValueAttrib<uint32_t, uint8_t>(customAttr, attrib);
                }
                else
                {
                    setMultiValueAttrib<uint32_t, uint8_t>(customAttr, attrib, nbrValsIncoming);
                }
            }
            break;
            case OmniPvdDataType::eUINT16:
            {
                if (attribDef->mNbrFields == 1)
                {
                    setSingleValueAttrib<uint32_t, uint16_t>(customAttr, attrib);
                }
                else
                {
                    setMultiValueAttrib<uint32_t, uint16_t>(customAttr, attrib, nbrValsIncoming);
                }
            }
            break;
            case OmniPvdDataType::eUINT32:
            {
                if (attribDef->mNbrFields == 1)
                {
                    setSingleValueAttrib<uint32_t, uint32_t>(customAttr, attrib);
                }
                else
                {
                    setMultiValueAttrib<uint32_t, uint32_t>(customAttr, attrib, nbrValsIncoming);
                }
            }
            break;
            case OmniPvdDataType::eUINT64:
            {
                if (attribDef->mNbrFields == 1)
                {
                    setSingleValueAttrib<uint64_t, uint64_t>(customAttr, attrib);
                }
                else
                {
                    setMultiValueAttrib<uint64_t, uint64_t>(customAttr, attrib, nbrValsIncoming);
                }
            }
            break;
            case OmniPvdDataType::eFLOAT32:
            {
                if (attribDef->mNbrFields == 1)
                {
                    setSingleValueAttrib<float, float>(customAttr, attrib);
                }
                else
                {
                    setMultiValueAttrib<float, float>(customAttr, attrib, nbrValsIncoming);
                }
            }
            break;
            case OmniPvdDataType::eFLOAT64:
            {
                if (attribDef->mNbrFields == 1)
                {
                    setSingleValueAttrib<double, double>(customAttr, attrib);
                }
                else
                {
                    setMultiValueAttrib<double, double>(customAttr, attrib, nbrValsIncoming);
                }
            }
            break;
            case OmniPvdDataType::eSTRING:
            {
                if (attribDef->mNbrFields == 1)
                {
                    char* srcVal = reinterpret_cast<char*>(attrib->mData);
                    std::string dstVal(srcVal);
                    customAttr.Set(dstVal, (double)attrib->mTimeStamp);
                }
                else
                {
                    // How to decode the string from the incoming data?
                }
            }
            break;
            case OmniPvdDataType::eOBJECT_HANDLE:
            {
                if (attribDef->mNbrFields == 1)
                {
                    setSingleValueAttrib<uint64_t, uint64_t>(customAttr, attrib);
                }
                else
                {
                    setMultiValueAttrib<uint64_t, uint64_t>(customAttr, attrib, nbrValsIncoming);
                }
            }
            break;
            default:
                break;
            }
        }
    }
}

void processEnum(
    pxr::UsdPrim* prim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject,
    OmniPvdAttributeDef* attribDef
)
{
    if (!prim) return;
    if (!*prim) return;

    if (!attribDef->mDerivedFromClass) return;
    if (!attribDef->mDerivedFromClass->mIsEnumClass) return;

    pxr::UsdAttribute enumAttr = prim->GetAttribute(*(attribDef->mPxrToken));

    const uint32_t flagVal = *((uint32_t*)attrib->mData);

    if (attribDef->mDerivedFromClass->mIsBitFieldEnum)
    {
        /////////////////////////////////////////////////////////////////////////////////
        // Create the attribute if it was not already created
        /////////////////////////////////////////////////////////////////////////////////
        if (!enumAttr)
        {
            enumAttr = prim->CreateAttribute(*(attribDef->mPxrToken), pxr::SdfValueTypeNames->TokenArray);
        }

        VtTokenArray tokenArray;

        std::vector<OmniPvdAttributeDef*>* bitFieldAttribs = &attribDef->mDerivedFromClass->mBitFieldAttribs;
        const int nbrMaxFlagBits = (int)bitFieldAttribs->size();
        uint32_t nbrBitsToProcess = nbrBitsSet(flagVal);
        int nbrKnownTokens = 0;
        int nbrUnKnownTokens = 0;
        {
            uint32_t bitProcessed = 0;
            uint32_t flagValProcessed = flagVal;
            while (bitProcessed < nbrBitsToProcess)
            {
                int bitPos = findFirstBitPos(flagValProcessed);
                flagValProcessed &= ~(1 << bitPos);
                if (nbrMaxFlagBits > bitPos)
                {
                    OmniPvdAttributeDef* attribDef = (*bitFieldAttribs)[bitPos];
                    if (attribDef)
                    {
                        nbrKnownTokens++;
                    }
                    else
                    {
                        nbrUnKnownTokens++;
                    }
                }
                else
                {
                    nbrUnKnownTokens++;
                }
                bitProcessed++;
            }
        }
        tokenArray.resize(nbrKnownTokens);
        uint32_t bitProcessed = 0;
        uint32_t flagValProcessed = flagVal;
        uint32_t knownFilledTokens = 0;
        while (bitProcessed < nbrBitsToProcess)
        {
            int bitPos = findFirstBitPos(flagValProcessed);
            flagValProcessed &= ~(1 << bitPos);
            if (nbrMaxFlagBits > bitPos)
            {
                OmniPvdAttributeDef* attribDef = (*bitFieldAttribs)[bitPos];
                if (attribDef)
                {
                    tokenArray[knownFilledTokens] = *(attribDef->mPxrToken);
                    knownFilledTokens++;
                }
            }
            bitProcessed++;
        }
        enumAttr.Set(tokenArray, (double)attrib->mTimeStamp);
    }
    else // just a regular one flag enum
    {
        /////////////////////////////////////////////////////////////////////////////////
        // Create the attribute if it was not already created
        /////////////////////////////////////////////////////////////////////////////////
        if (!enumAttr)
        {
            enumAttr = prim->CreateAttribute(*(attribDef->mPxrToken), pxr::SdfValueTypeNames->Token);
        }
        TfToken token;

        // See if the flag value corresponds to one of the attribute definitions in the derived class
        std::vector<OmniPvdAttributeDef*>* enumAttribs = &attribDef->mDerivedFromClass->mAttributeDefinitions;
        const int nbrAttribs = (int)enumAttribs->size();
        bool foundToken = false;
        for (int a = 0; a < nbrAttribs; a++)
        {
            OmniPvdAttributeDef* attribDef = (*enumAttribs)[a];
            if (attribDef->mNbrFields == flagVal)
            {
                token = *(attribDef->mPxrToken);
                foundToken = true;
                break;
            }
        }
        enumAttr.Set(token, (double)attrib->mTimeStamp);
    }
}

void processXFormFork(
    pxr::UsdPrim* prim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject
    )
{
    if (!prim) return;
    if (!*prim) return;

    pxr::UsdGeomXformable *xformable = (pxr::UsdGeomXformable*)prim;

    // If this is an articulation link, make sure we do no inherit the parent tform
    if (isSameString(omniPvdObject->mOmniPvdClass->mClassName.c_str(),"PxArticulationLink"))
    {
        if (!xformable->GetResetXformStack())
        {
            xformable->SetResetXformStack(true);
        }
    }


    physx::PxTransform* tform= (physx::PxTransform*)attrib->mData;

    {
        static TfToken const &attribToken = UsdGeomXformOp::GetOpName(UsdGeomXformOp::Type::TypeTranslate);    
        pxr::UsdAttribute usdAttrib = prim->GetAttribute(attribToken);
        if (!usdAttrib) {
            pxr::UsdGeomXformOp translation = xformable->AddTranslateOp(pxr::UsdGeomXformOp::PrecisionFloat);
        }
        // Offset by 4 floats to get to the translation part
        float *pos = (float*)&tform->p;
        usdAttrib.Set(pxr::GfVec3f(pos[0], pos[1], pos[2]), (double)attrib->mTimeStamp);
    }
    {
        static TfToken const &attribToken = UsdGeomXformOp::GetOpName(UsdGeomXformOp::Type::TypeOrient);
        pxr::UsdAttribute usdAttrib = prim->GetAttribute(attribToken);        
        if (!usdAttrib) {
            pxr::UsdGeomXformOp translation = xformable->AddOrientOp(pxr::UsdGeomXformOp::PrecisionFloat);
        }
        float *quat = (float*)&tform->q;
        usdAttrib.Set(pxr::GfQuatf(quat[3], quat[0], quat[1], quat[2]), (double)attrib->mTimeStamp);
    }
}

void processTranslation(
    pxr::UsdPrim* prim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject
    )
{
    if (!prim) return;
    if (!*prim) return;

    static TfToken const &attribToken = UsdGeomXformOp::GetOpName(UsdGeomXformOp::Type::TypeTranslate);
    pxr::UsdAttribute usdAttrib = prim->GetAttribute(attribToken);
    pxr::UsdGeomXformable *xformable = (pxr::UsdGeomXformable*)prim;
    if (!usdAttrib) {
        pxr::UsdGeomXformOp translation = xformable->AddTranslateOp(pxr::UsdGeomXformOp::PrecisionFloat);
    }    
    float *pos = (float*)attrib->mData;
    usdAttrib.Set(pxr::GfVec3f(pos[0], pos[1], pos[2]), (double)attrib->mTimeStamp);
}

void processRotation(
    pxr::UsdPrim* prim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject
    )
{
    if (!prim) return;
    if (!*prim) return;

    static TfToken const &attribToken = UsdGeomXformOp::GetOpName(UsdGeomXformOp::Type::TypeOrient);
    pxr::UsdAttribute usdAttrib = prim->GetAttribute(attribToken);
    pxr::UsdGeomXformable *xformable = (pxr::UsdGeomXformable*)prim;
    if (!usdAttrib) {
        pxr::UsdGeomXformOp translation = xformable->AddOrientOp(pxr::UsdGeomXformOp::PrecisionFloat);
    }
    float *quat = (float*)attrib->mData;
    usdAttrib.Set(pxr::GfQuatf(quat[3], quat[0], quat[1], quat[2]), (double)attrib->mTimeStamp);
}

void processScale(
    pxr::UsdPrim* prim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject
    )
{
    if (!prim) return;
    if (!*prim) return;

    static TfToken const &attribToken = UsdGeomXformOp::GetOpName(UsdGeomXformOp::Type::TypeScale);
    pxr::UsdAttribute usdAttrib = prim->GetAttribute(attribToken);
    pxr::UsdGeomXformable *xformable = (pxr::UsdGeomXformable*)prim;
    if (!usdAttrib) {
        pxr::UsdGeomXformOp scaleOp = xformable->AddScaleOp(pxr::UsdGeomXformOp::PrecisionFloat);
    }
    float *scale = (float*)attrib->mData;
    usdAttrib.Set(pxr::GfVec3f(scale[0], scale[1], scale[2]), (double)attrib->mTimeStamp);
}
void processVisibility(
    pxr::UsdPrim* prim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject,
    OmniPvdDOMState& domState
    )
{
    if (!prim) return;
    if (!*prim) return;

    pxr::UsdGeomImageable imageableTest = (pxr::UsdGeomImageable)(*prim);
    if (!imageableTest) return;
    pxr::UsdGeomImageable* imageable = (pxr::UsdGeomImageable*)prim;
    if (omniPvdObject->mIsStaticVisibility) {
        if (omniPvdObject->mIsStaticVisible) {
            prim->CreateAttribute(TfToken("omni:pvdi:viz"), SdfValueTypeNames->Bool).Set(true, -1.0f);
        } else {
            prim->CreateAttribute(TfToken("omni:pvdi:viz"), SdfValueTypeNames->Bool).Set(false, -1.0f);
        }
    } else {
        const int nbrLifeSpans = (int)omniPvdObject->mLifeSpans.size();
        if (nbrLifeSpans == 1) {            
            prim->CreateAttribute(TfToken("omni:pvdi:viz"), SdfValueTypeNames->Bool).Set(false, ((double)omniPvdObject->mLifeSpans[0].mFrameStart) - 1.0);
            prim->CreateAttribute(TfToken("omni:pvdi:viz"), SdfValueTypeNames->Bool).Set(true, ((double)omniPvdObject->mLifeSpans[0].mFrameStart));
            if (omniPvdObject->mLifeSpans[0].mFrameStart < omniPvdObject->mLifeSpans[0].mFrameStop) {
                prim->CreateAttribute(TfToken("omni:pvdi:viz"), SdfValueTypeNames->Bool).Set(false, (double)omniPvdObject->mLifeSpans[0].mFrameStop);
            }
        } else { // nbrLifeSpans > 1
            ////////////////////////////////////////////////////////////////////////////////
            // For loop over the life spans
            ////////////////////////////////////////////////////////////////////////////////
            prim->CreateAttribute(TfToken("omni:pvdi:viz"), SdfValueTypeNames->Bool).Set(false, ((double)omniPvdObject->mLifeSpans[0].mFrameStart) - 1.0);
            for (int span = 0; span < nbrLifeSpans; span++) {
                prim->CreateAttribute(TfToken("omni:pvdi:viz"), SdfValueTypeNames->Bool).Set(true, ((double)omniPvdObject->mLifeSpans[span].mFrameStart));
                if (omniPvdObject->mLifeSpans[span].mFrameStart < omniPvdObject->mLifeSpans[span].mFrameStop) {
                    prim->CreateAttribute(TfToken("omni:pvdi:viz"), SdfValueTypeNames->Bool).Set(false, ((double)omniPvdObject->mLifeSpans[span].mFrameStop));
                }
            }
        }
    }
}

void processPlane(
    pxr::UsdPrim* prim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject
)
{
    if (!prim) return;
    if (!*prim) return;

    pxr::UsdGeomMesh* geom = (pxr::UsdGeomMesh*)prim;

    const float width = 10000.0f;
    // 3  2
    //      
    // 0  1
    // tri_1 : 0 1 2
    // tri_2 : 0 2 3

    const int nbrTri = 2;
    const int nbrVert = 4;

    float verts[4 * 3];
    int tris[6];

    tris[0] = 0;
    tris[1] = 1;
    tris[2] = 2;

    tris[3] = 0;
    tris[4] = 2;
    tris[5] = 3;

    verts[0] = 0.0f;
    verts[1] = -width * 0.5f;
    verts[2] = -width * 0.5f;

    verts[3] = 0.0f;
    verts[4] = width * 0.5f;
    verts[5] = -width * 0.5f;

    verts[6] = 0.0f;
    verts[7] = width * 0.5f;
    verts[8] = width * 0.5f;

    verts[9] = 0.0f;
    verts[10] = -width * 0.5f;
    verts[11] = width * 0.5f;

    int *flatIndices = new int[nbrTri * 3];
    const int nbrFlatVerts = nbrTri * 3;
    float *flatVerts = new float[nbrFlatVerts * 3];

    for (int ind = 0; ind < nbrFlatVerts; ind++) {
        flatIndices[ind] = ind;
    }
    const int nbrTris = nbrTri;
    for (int tri = 0; tri < nbrTris; tri++) {
        int v0Index = tris[tri * 3 + 0];
        int v1Index = tris[tri * 3 + 1];
        int v2Index = tris[tri * 3 + 2];
        // v0
        flatVerts[tri * 9 + 0] = verts[v0Index * 3 + 0];
        flatVerts[tri * 9 + 1] = verts[v0Index * 3 + 1];
        flatVerts[tri * 9 + 2] = verts[v0Index * 3 + 2];

        flatVerts[tri * 9 + 3] = verts[v1Index * 3 + 0];
        flatVerts[tri * 9 + 4] = verts[v1Index * 3 + 1];
        flatVerts[tri * 9 + 5] = verts[v1Index * 3 + 2];

        flatVerts[tri * 9 + 6] = verts[v2Index * 3 + 0];
        flatVerts[tri * 9 + 7] = verts[v2Index * 3 + 1];
        flatVerts[tri * 9 + 8] = verts[v2Index * 3 + 2];
    }

    pxr::VtArray<int> vertexIndicesVt;
    vertexIndicesVt.assign(flatIndices, &flatIndices[nbrTri * 3]);

    pxr::VtArray<int> vertexCounts = pxr::VtArray<int>(2, 3);
    geom->CreateFaceVertexCountsAttr().Set(vertexCounts); // vertex count
    geom->CreateFaceVertexIndicesAttr().Set(vertexIndicesVt); // triangle indices

    pxr::GfVec3f *usdVerts = new pxr::GfVec3f[nbrFlatVerts];
    memcpy(usdVerts, flatVerts, sizeof(float) * 3 * nbrFlatVerts);
    pxr::VtArray<pxr::GfVec3f> pointArrayVt;
    pointArrayVt.assign(usdVerts, &usdVerts[nbrFlatVerts]);
    geom->CreatePointsAttr().Set(pointArrayVt); // vertices

    float minExt[3];
    float maxExt[3];

    minExt[0] = -width * 0.5f;
    minExt[1] = 0.0f;
    minExt[2] = -width * 0.5f;

    maxExt[0] = width * 0.5f;
    maxExt[1] = 0.0f;
    maxExt[2] = width * 0.5f;

    pxr::GfVec3f extents[2];
    extents[0].Set(minExt);
    extents[1].Set(maxExt);
    pxr::VtArray<pxr::GfVec3f> extentsVt;
    extentsVt.assign(extents, &extents[2]);

    float *normals = new float[nbrFlatVerts * 3];

    getNormals(normals, nbrFlatVerts, flatVerts, flatIndices, nbrTri);

    pxr::GfVec3f *usdNorms = new pxr::GfVec3f[6];
    memcpy(usdNorms, normals, sizeof(float) * 3 * 6);

    pxr::VtArray<pxr::GfVec3f> normalsVt;
    normalsVt.assign(usdNorms, &usdNorms[nbrFlatVerts]);
    geom->CreateNormalsAttr().Set(normalsVt); // normals

    geom->CreateSubdivisionSchemeAttr().Set(pxr::TfToken("none")); // subdivision

    delete[] normals;
    delete[] flatIndices;
    delete[] flatVerts;
    delete[] usdVerts;
    delete[] usdNorms;
}

void processMesh(
    pxr::UsdPrim* prim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject
)
{
    if (!prim) return;
    if (!*prim) return;

    float *verts = 0;
    int  nbrVert = 0;
    int    *tris = 0;
    int   nbrTri = 0;

    const int nbrInheritedClasses = static_cast<int>(omniPvdObject->mInheritedClassInstances.size());
    for (int c = 0; c < nbrInheritedClasses; c++)
    {
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
                    switch (attributeInstList->mAttributeDef->mUsdAttributeId)
                    {
                    case OmniPvdUsdAttributeEnum::eUSDAttributeVerts:
                    {
                        verts = (float*)attrib->mData;
                        nbrVert = attrib->mDataLen / (sizeof(float) * 3);
                    }
                    break;
                    case OmniPvdUsdAttributeEnum::eUSDAttributeTris:
                    {
                        tris = (int*)attrib->mData;
                        nbrTri = attrib->mDataLen / (sizeof(int) * 3);
                    }
                    break;
                    }
                }
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Defensive coding againt an ill-defined mesh
    ////////////////////////////////////////////////////////////////////////////////
    if (!((verts!=nullptr) && (tris!=nullptr) && (nbrVert>0) && (nbrTri>0)))
    {
        return;
    }

    pxr::UsdGeomMesh* geom = (pxr::UsdGeomMesh*)prim;
    ////////////////////////////////////////////////////////////////////////////////
    // Get the extents (bounding box of the mesh)
    ////////////////////////////////////////////////////////////////////////////////
    float minExt[3];
    float maxExt[3];
    {
        float *vertPtr = verts;
        setFloatVec3(minExt, verts);
        setFloatVec3(maxExt, verts);
        vertPtr += 3;
        for (int i = 1; i < nbrVert; i++)
        {
            minFloatVec3(minExt, vertPtr);
            maxFloatVec3(maxExt, vertPtr);
            vertPtr += 3;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////
    // Expand the vertices from the compact description to 3 vertices per triangle
    // This is so that flat shading can have per face independent normals, not tied to
    // shared vertices between triangles
    ////////////////////////////////////////////////////////////////////////////////

    int *flatIndices = new int[nbrTri * 3];
    const int nbrFlatVerts = nbrTri * 3;
    float *flatVerts = new float[nbrFlatVerts * 3];

    for (int ind = 0; ind < nbrFlatVerts; ind++)
    {
        flatIndices[ind] = ind;
    }
    const int nbrTris = nbrTri;
    for (int tri = 0; tri < nbrTris; tri++)
    {
        int v0Index = tris[tri * 3 + 0];
        int v1Index = tris[tri * 3 + 1];
        int v2Index = tris[tri * 3 + 2];
        // v0
        flatVerts[tri * 9 + 0] = verts[v0Index * 3 + 0];
        flatVerts[tri * 9 + 1] = verts[v0Index * 3 + 1];
        flatVerts[tri * 9 + 2] = verts[v0Index * 3 + 2];

        flatVerts[tri * 9 + 3] = verts[v1Index * 3 + 0];
        flatVerts[tri * 9 + 4] = verts[v1Index * 3 + 1];
        flatVerts[tri * 9 + 5] = verts[v1Index * 3 + 2];

        flatVerts[tri * 9 + 6] = verts[v2Index * 3 + 0];
        flatVerts[tri * 9 + 7] = verts[v2Index * 3 + 1];
        flatVerts[tri * 9 + 8] = verts[v2Index * 3 + 2];
    }

    pxr::VtArray<int> vertexIndicesVt;
    vertexIndicesVt.assign(flatIndices, &flatIndices[nbrTri * 3]);

    pxr::VtArray<int> vertexCounts = pxr::VtArray<int>(nbrTri, 3);
    geom->CreateFaceVertexCountsAttr().Set(vertexCounts);
    geom->CreateFaceVertexIndicesAttr().Set(vertexIndicesVt);

    ////////////////////////////////////////////////////////////////////////////////
    // Set vertex position (point) attribute using the flat shaded verts
    ////////////////////////////////////////////////////////////////////////////////

    pxr::GfVec3f *usdVerts = new pxr::GfVec3f[nbrFlatVerts];
    memcpy(usdVerts, flatVerts, sizeof(float) * 3 * nbrFlatVerts);
    pxr::VtArray<pxr::GfVec3f> pointArrayVt;
    pointArrayVt.assign(usdVerts, &usdVerts[nbrFlatVerts]);
    geom->CreatePointsAttr().Set(pointArrayVt);

    pxr::GfVec3f extents[2];
    extents[0].Set(minExt);
    extents[1].Set(maxExt);
    pxr::VtArray<pxr::GfVec3f> extentsVt;
    extentsVt.assign(extents, &extents[2]);
    ////////////////////////////////////////////////////////////////////////////////
    // After computing it, set the extents attribute
    ////////////////////////////////////////////////////////////////////////////////
    geom->CreateExtentAttr().Set(extentsVt);

    ////////////////////////////////////////////////////////////////////////////////
    // Set the double sided attribute
    ////////////////////////////////////////////////////////////////////////////////
    geom->CreateDoubleSidedAttr().Set(true);

    ////////////////////////////////////////////////////////////////////////////////
    // Calculate and set normals
    ////////////////////////////////////////////////////////////////////////////////
    float *normals = new float[nbrFlatVerts * 3];

    getNormals(normals, nbrFlatVerts, flatVerts, flatIndices, nbrTri);

    pxr::GfVec3f *usdNorms = new pxr::GfVec3f[nbrFlatVerts];
    memcpy(usdNorms, normals, sizeof(float) * 3 * nbrFlatVerts);

    pxr::VtArray<pxr::GfVec3f> normalsVt;
    normalsVt.assign(usdNorms, &usdNorms[nbrFlatVerts]);
    geom->CreateNormalsAttr().Set(normalsVt);

    geom->CreateSubdivisionSchemeAttr().Set(pxr::TfToken("none"));

    delete[] flatVerts;
    delete[] flatIndices;
    delete[] normals;
    delete[] usdVerts;
    delete[] usdNorms;
}

void processPoints(
    pxr::UsdPrim* prim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject
)
{
    if (!prim) return;
    if (!*prim) return;

    pxr::UsdGeomPoints* geom = (pxr::UsdGeomPoints*)prim;

    std::vector<float> radiiValues;
    std::vector<uint64_t> radiiTimeStamps;

    //read rest offset from ancestor particle system
    {
        float lastValue = 0.0f;
        uint64_t lastTimeStamp = 0;
        const int nbrInheritedClasses = static_cast<int>(omniPvdObject->mAncestor->mInheritedClassInstances.size());
        int32_t attribIndex = -1;
        int32_t classIndex = -1;
        OmniPvdAttributeInstList* attribList = getAttribList(attribIndex, classIndex, "restOffset", omniPvdObject->mAncestor);
        OmniPvdAttributeSample* attrib = (OmniPvdAttributeSample*)attribList->mFirst;
        while (attrib)
        {
            float* radiusPtr = (float*)attrib->mData;
            if (!radiiTimeStamps.empty() && radiiTimeStamps.back() == attrib->mTimeStamp)
            {
                //replace values with identical time stamps with last value
                radiiValues.back() = *radiusPtr;
            }
            else
            {
                radiiTimeStamps.push_back(attrib->mTimeStamp);
                radiiValues.push_back(*radiusPtr);
            }
            lastValue = *radiusPtr;
            lastTimeStamp = attrib->mTimeStamp;

            attrib = (OmniPvdAttributeSample*)(attrib->mNextAttribute);
        }
        //push last element for rest of time stamp domain
        radiiTimeStamps.push_back(std::numeric_limits<uint64_t>::max());
        radiiValues.push_back(lastValue);
        //push first element if timestamp 0 is missing
        if (radiiTimeStamps.front() != 0)
        {
            //don't really need to optimize any of this, since we really don't expect more than a handfull entries.
            radiiTimeStamps.insert(radiiTimeStamps.begin(), 0);
            radiiValues.insert(radiiValues.begin(), 0.0f);
        }
    }

    //copy points to geom and set widths
    {
        pxr::UsdAttribute pointsAttr = geom->GetPointsAttr();
        pxr::UsdAttribute widthsAttr = geom->GetWidthsAttr();
        VtArray<GfVec3f> usdPoints;
        VtArray<float> usdWidths;
        size_t radiiIndex = 0;

        const int nbrInheritedClasses = static_cast<int>(omniPvdObject->mInheritedClassInstances.size());
        for (int c = 0; c < nbrInheritedClasses; c++)
        {
            std::vector<OmniPvdAttributeInstList*>& classAttributeLists = omniPvdObject->mInheritedClassInstances[c].mClassAttributeLists;
            const int nbrAttributes = (int)classAttributeLists.size();
            for (int j = 0; j < nbrAttributes; j++)
            {
                OmniPvdAttributeInstList* attributeInstList = classAttributeLists[j];
                if (attributeInstList)
                {
                    if (!attributeInstList->mAttributeDef->mIsUniqueList)
                    {
                        OmniPvdAttributeSample* attrib = (OmniPvdAttributeSample*)attributeInstList->mFirst;
                        while (attrib)
                        {
                            switch (attributeInstList->mAttributeDef->mUsdAttributeId)
                            {
                            case OmniPvdUsdAttributeEnum::eUSDAttributePoints:
                            {
                                float* points = (float*)attrib->mData;
                                int nbrPoint = attrib->mDataLen / (sizeof(float) * 4);
                                usdPoints.resize(nbrPoint);
                                usdWidths.resize(nbrPoint);
                                float* srcPtr = points;
                                for (int i = 0; i < nbrPoint; ++i)
                                {
                                    usdPoints[i] = GfVec3f(srcPtr[0], srcPtr[1], srcPtr[2]);
                                    srcPtr += 4;
                                }

                                float radius = 0.0f;
                                while (radiiIndex + 1 < radiiTimeStamps.size() && attrib->mTimeStamp >= radiiTimeStamps[radiiIndex + 1])
                                {
                                    radiiIndex++;
                                }
                                radius = radiiValues[radiiIndex];

                                for (size_t i = 0; i < usdPoints.size(); ++i)
                                {
                                    usdWidths[i] = radius * 2;
                                }
                                pointsAttr.Set(usdPoints, (double)attrib->mTimeStamp);
                                widthsAttr.Set(usdWidths, (double)attrib->mTimeStamp);
                            }
                            break;
                            default:
                            {
                                processCustomAttribute(*prim, attrib, attributeInstList->mAttributeDef);
                            }
                            break;
                            }
                            attrib = (OmniPvdAttributeSample*)(attrib->mNextAttribute);
                        }
                    }
                }
            }
        }
    }


  

}
