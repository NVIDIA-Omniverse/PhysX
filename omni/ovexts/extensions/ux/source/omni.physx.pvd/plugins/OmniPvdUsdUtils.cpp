// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "OmniPvdUsdUtils.h"

#include "foundation/PxTransform.h"
#include <unordered_map>


namespace OmniPvd
{
PXR_NS::TfHashMap<uint32_t, PXR_NS::TfToken> articulationJointMotionMap;
PXR_NS::TfHashMap<uint32_t, PXR_NS::TfToken> articulationJointDriveTypeMap;
PXR_NS::TfHashMap<uint32_t, PXR_NS::TfToken> jointD6MotionMap;
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
void processConeRadius(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;

    PXR_NS::UsdGeomCone *geom = (PXR_NS::UsdGeomCone*)prim;
    float radius = *((float*)attrib->mData);
    geom->GetRadiusAttr().Set(double(radius));
}

void processConeHeight(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;

    PXR_NS::UsdGeomCone *geom = (PXR_NS::UsdGeomCone*)prim;
    float height = *((float*)attrib->mData);
    geom->GetHeightAttr().Set(double(height));
}

void processConeAxis(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;

    PXR_NS::UsdGeomCone *geom = (PXR_NS::UsdGeomCone*)prim;
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
void processSphereRadius(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;

    PXR_NS::UsdGeomSphere *geom = (PXR_NS::UsdGeomSphere*)prim;
    float radius = *((float*)attrib->mData);
    geom->GetRadiusAttr().Set(double(radius));
}

////////////////////////////////////////////////////////////////////////////////
// Capsule
////////////////////////////////////////////////////////////////////////////////
void processCapsuleHeight(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;

    PXR_NS::UsdGeomCapsule *geom = (PXR_NS::UsdGeomCapsule*)prim;
    float height = *((float*)attrib->mData);
    geom->GetHeightAttr().Set(2.0*double(height));
}

void processCapsuleRadius(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;

    PXR_NS::UsdGeomCapsule *geom = (PXR_NS::UsdGeomCapsule*)prim;
    float radius = *((float*)attrib->mData);
    geom->GetRadiusAttr().Set(double(radius));
}

void processDisplayColour(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;

    PXR_NS::UsdGeomGprim *geomPrim = (PXR_NS::UsdGeomGprim*)prim;
    float *col = (float*)attrib->mData;
    PXR_NS::VtArray<PXR_NS::GfVec3f> colourArrayVt;
    colourArrayVt.push_back(PXR_NS::GfVec3f(col[0], col[1], col[2]));
    geomPrim->GetDisplayColorAttr().Set(colourArrayVt);
}

////////////////////////////////////////////////////////////////////////////////
// Cylinder
////////////////////////////////////////////////////////////////////////////////
void processCylinderRadius(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;

    PXR_NS::UsdGeomCylinder *geom = (PXR_NS::UsdGeomCylinder*)prim;
    float radius = *((float*)attrib->mData);
    geom->GetRadiusAttr().Set(double(radius));
}

void processCylinderHeight(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;

    PXR_NS::UsdGeomCylinder *geom = (PXR_NS::UsdGeomCylinder*)prim;
    float height = *((float*)attrib->mData);
    geom->GetHeightAttr().Set(double(height));
}

void processCylinderAxis(PXR_NS::UsdPrim* prim, OmniPvdAttributeSample* attrib, OmniPvdObject* omniPvdObject)
{
    if (!prim) return;
    if (!*prim) return;

    PXR_NS::UsdGeomCylinder *geom = (PXR_NS::UsdGeomCylinder*)prim;
    int axis = *((int*)attrib->mData);
    if (axis ==  0) {
        geom->GetAxisAttr().Set(OmniPvd::XToken);
    } else if (axis ==  1) {
        geom->GetAxisAttr().Set(OmniPvd::YToken);
    } else if (axis ==  2) {
        geom->GetAxisAttr().Set(OmniPvd::ZToken);
    }
}

PXR_NS::UsdAttribute getOrCreateAttribute(PXR_NS::UsdPrim& prim, const PXR_NS::TfToken& token, OmniPvdDataType::Enum dataType, bool isArray)
{
    PXR_NS::UsdAttribute attr = prim.GetAttribute(token);
    if (!attr)
    {
        switch (dataType)
        {
        case OmniPvdDataType::eINT8:
        case OmniPvdDataType::eINT16:
        case OmniPvdDataType::eINT32:
            attr = prim.CreateAttribute(token, isArray ? PXR_NS::SdfValueTypeNames->IntArray : PXR_NS::SdfValueTypeNames->Int);
            break;
        case OmniPvdDataType::eINT64:
            attr = prim.CreateAttribute(token, isArray ? PXR_NS::SdfValueTypeNames->Int64Array : PXR_NS::SdfValueTypeNames->Int64);
            break;
        case OmniPvdDataType::eUINT8:
        case OmniPvdDataType::eUINT16:
        case OmniPvdDataType::eUINT32:
            attr = prim.CreateAttribute(token, isArray ? PXR_NS::SdfValueTypeNames->UIntArray : PXR_NS::SdfValueTypeNames->UInt);
            break;
        case OmniPvdDataType::eUINT64:
        case OmniPvdDataType::eOBJECT_HANDLE:
            attr = prim.CreateAttribute(token, isArray ? PXR_NS::SdfValueTypeNames->UInt64Array : PXR_NS::SdfValueTypeNames->UInt64);
            break;
        case OmniPvdDataType::eFLOAT32:
            attr = prim.CreateAttribute(token, isArray ? PXR_NS::SdfValueTypeNames->FloatArray : PXR_NS::SdfValueTypeNames->Float);
            break;
        case OmniPvdDataType::eFLOAT64:
            attr = prim.CreateAttribute(token, isArray ? PXR_NS::SdfValueTypeNames->DoubleArray : PXR_NS::SdfValueTypeNames->Double);
            break;
        case OmniPvdDataType::eSTRING:
            attr = prim.CreateAttribute(token, isArray ? PXR_NS::SdfValueTypeNames->StringArray : PXR_NS::SdfValueTypeNames->String);
            break;
        default:
            break;
        }
    }
    return attr;
}

// Explicit template instantiations
template void processCustomAttribute<OmniPvdAttributeSample>(PXR_NS::UsdPrim& prim, OmniPvdAttributeSample* attrib, OmniPvdAttributeDef* attribDef);
template void processCustomAttribute<OmniPvdUniqueList>(PXR_NS::UsdPrim& prim, OmniPvdUniqueList* attrib, OmniPvdAttributeDef* attribDef);

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

PXR_NAMESPACE_USING_DIRECTIVE;


bool processSpecialEnums(
    PXR_NS::UsdPrim &prim,
    OmniPvdAttributeSample *attrib,
    OmniPvdAttributeDef* attribDef
)
{
    if (!prim) return false;

    TfHashMap<uint32_t, TfToken> *mapPtr = nullptr;
    
    // Check for known enum attributes by class and attribute name
    if (isSameString(attribDef->mClass->mClassName.c_str(), "PxArticulationJointReducedCoordinate"))
    {
        if (isSameString(attribDef->mAttributeName.c_str(), "motion"))
        {
            mapPtr = &OmniPvd::articulationJointMotionMap;
        }
        else if (isSameString(attribDef->mAttributeName.c_str(), "driveType"))
        {
            mapPtr = &OmniPvd::articulationJointDriveTypeMap;
        }
    }
    else if (isSameString(attribDef->mClass->mClassName.c_str(), "PxJoint"))
    {
        if (isSameString(attribDef->mAttributeName.c_str(), "motions") || isSameString(attribDef->mAttributeName.c_str(), "d6Motions"))
        {
            mapPtr = &OmniPvd::jointD6MotionMap;
        }
    }

    if (!mapPtr) return false;

    if (mapPtr)
    {
        const OmniPvdDataType::Enum dataEnumVal = OmniPvdDataType::Enum(attribDef->mDataType);
        int componentByteSize = getComponentByteSize(dataEnumVal);
        int nbrValsIncoming = attrib->mDataLen / componentByteSize;
        if (nbrValsIncoming >= 1)
        {
            PXR_NS::UsdAttribute customAttr = prim.GetAttribute(*OmniPvdUsd::getToken(attribDef));
            /////////////////////////////////////////////////////////////////////////////////
            // Create the attribute if it was not already created
            /////////////////////////////////////////////////////////////////////////////////
            if (!customAttr)
            {
                customAttr = prim.CreateAttribute(*OmniPvdUsd::getToken(attribDef), PXR_NS::SdfValueTypeNames->TokenArray);
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

    return true;
}

void processEnum(
    PXR_NS::UsdPrim* prim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject,
    OmniPvdAttributeDef* attribDef
)
{
    if (!prim) return;
    if (!*prim) return;

    if (!attribDef->mDerivedFromClass) return;
    if (!attribDef->mDerivedFromClass->mIsEnumClass) return;

    PXR_NS::UsdAttribute enumAttr = prim->GetAttribute(*OmniPvdUsd::getToken(attribDef));

    const uint32_t flagVal = *((uint32_t*)attrib->mData);

    if (attribDef->mDerivedFromClass->mIsBitFieldEnum)
    {
        /////////////////////////////////////////////////////////////////////////////////
        // Create the attribute if it was not already created
        /////////////////////////////////////////////////////////////////////////////////
        if (!enumAttr)
        {
            enumAttr = prim->CreateAttribute(*OmniPvdUsd::getToken(attribDef), PXR_NS::SdfValueTypeNames->TokenArray);
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
                    tokenArray[knownFilledTokens] = *OmniPvdUsd::getToken(attribDef);
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
            enumAttr = prim->CreateAttribute(*OmniPvdUsd::getToken(attribDef), PXR_NS::SdfValueTypeNames->Token);
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
                token = *OmniPvdUsd::getToken(attribDef);
                foundToken = true;
                break;
            }
        }
        enumAttr.Set(token, (double)attrib->mTimeStamp);
    }
}

void processXFormFork(
    PXR_NS::UsdPrim* prim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject
    )
{
    if (!prim) return;
    if (!*prim) return;

    PXR_NS::UsdGeomXformable *xformable = (PXR_NS::UsdGeomXformable*)prim;

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
        PXR_NS::UsdAttribute usdAttrib = prim->GetAttribute(attribToken);
        if (!usdAttrib) {
            PXR_NS::UsdGeomXformOp translation = xformable->AddTranslateOp(PXR_NS::UsdGeomXformOp::PrecisionFloat);
        }
        // Offset by 4 floats to get to the translation part
        float *pos = (float*)&tform->p;
        usdAttrib.Set(PXR_NS::GfVec3f(pos[0], pos[1], pos[2]), (double)attrib->mTimeStamp);
    }
    {
        static TfToken const &attribToken = UsdGeomXformOp::GetOpName(UsdGeomXformOp::Type::TypeOrient);
        PXR_NS::UsdAttribute usdAttrib = prim->GetAttribute(attribToken);        
        if (!usdAttrib) {
            PXR_NS::UsdGeomXformOp translation = xformable->AddOrientOp(PXR_NS::UsdGeomXformOp::PrecisionFloat);
        }
        float *quat = (float*)&tform->q;
        usdAttrib.Set(PXR_NS::GfQuatf(quat[3], quat[0], quat[1], quat[2]), (double)attrib->mTimeStamp);
    }
}

void processTranslation(
    PXR_NS::UsdPrim* prim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject
    )
{
    if (!prim) return;
    if (!*prim) return;

    static TfToken const &attribToken = UsdGeomXformOp::GetOpName(UsdGeomXformOp::Type::TypeTranslate);
    PXR_NS::UsdAttribute usdAttrib = prim->GetAttribute(attribToken);
    PXR_NS::UsdGeomXformable *xformable = (PXR_NS::UsdGeomXformable*)prim;
    if (!usdAttrib) {
        PXR_NS::UsdGeomXformOp translation = xformable->AddTranslateOp(PXR_NS::UsdGeomXformOp::PrecisionFloat);
    }    
    float *pos = (float*)attrib->mData;
    usdAttrib.Set(PXR_NS::GfVec3f(pos[0], pos[1], pos[2]), (double)attrib->mTimeStamp);
}

void processRotation(
    PXR_NS::UsdPrim* prim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject
    )
{
    if (!prim) return;
    if (!*prim) return;

    static TfToken const &attribToken = UsdGeomXformOp::GetOpName(UsdGeomXformOp::Type::TypeOrient);
    PXR_NS::UsdAttribute usdAttrib = prim->GetAttribute(attribToken);
    PXR_NS::UsdGeomXformable *xformable = (PXR_NS::UsdGeomXformable*)prim;
    if (!usdAttrib) {
        PXR_NS::UsdGeomXformOp translation = xformable->AddOrientOp(PXR_NS::UsdGeomXformOp::PrecisionFloat);
    }
    float *quat = (float*)attrib->mData;
    usdAttrib.Set(PXR_NS::GfQuatf(quat[3], quat[0], quat[1], quat[2]), (double)attrib->mTimeStamp);
}

void processScale(
    PXR_NS::UsdPrim* prim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject
    )
{
    if (!prim) return;
    if (!*prim) return;

    static TfToken const &attribToken = UsdGeomXformOp::GetOpName(UsdGeomXformOp::Type::TypeScale);
    PXR_NS::UsdAttribute usdAttrib = prim->GetAttribute(attribToken);
    PXR_NS::UsdGeomXformable *xformable = (PXR_NS::UsdGeomXformable*)prim;
    if (!usdAttrib) {
        PXR_NS::UsdGeomXformOp scaleOp = xformable->AddScaleOp(PXR_NS::UsdGeomXformOp::PrecisionFloat);
    }
    float *scale = (float*)attrib->mData;
    usdAttrib.Set(PXR_NS::GfVec3f(scale[0], scale[1], scale[2]), (double)attrib->mTimeStamp);
}
void processVisibility(
    PXR_NS::UsdPrim* prim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject,
    OmniPvdDOMState& domState
    )
{
    if (!prim) return;
    if (!*prim) return;

    PXR_NS::UsdGeomImageable imageableTest = (PXR_NS::UsdGeomImageable)(*prim);
    if (!imageableTest) return;
    PXR_NS::UsdGeomImageable* imageable = (PXR_NS::UsdGeomImageable*)prim;
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
    PXR_NS::UsdPrim* prim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject
)
{
    if (!prim) return;
    if (!*prim) return;

    PXR_NS::UsdGeomMesh* geom = (PXR_NS::UsdGeomMesh*)prim;

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

    PXR_NS::VtArray<int> vertexIndicesVt;
    vertexIndicesVt.assign(flatIndices, &flatIndices[nbrTri * 3]);

    PXR_NS::VtArray<int> vertexCounts = PXR_NS::VtArray<int>(2, 3);
    geom->CreateFaceVertexCountsAttr().Set(vertexCounts); // vertex count
    geom->CreateFaceVertexIndicesAttr().Set(vertexIndicesVt); // triangle indices

    PXR_NS::GfVec3f *usdVerts = new PXR_NS::GfVec3f[nbrFlatVerts];
    memcpy(usdVerts, flatVerts, sizeof(float) * 3 * nbrFlatVerts);
    PXR_NS::VtArray<PXR_NS::GfVec3f> pointArrayVt;
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

    PXR_NS::GfVec3f extents[2];
    extents[0].Set(minExt);
    extents[1].Set(maxExt);
    PXR_NS::VtArray<PXR_NS::GfVec3f> extentsVt;
    extentsVt.assign(extents, &extents[2]);

    float *normals = new float[nbrFlatVerts * 3];

    getNormals(normals, nbrFlatVerts, flatVerts, flatIndices, nbrTri);

    PXR_NS::GfVec3f *usdNorms = new PXR_NS::GfVec3f[6];
    memcpy(usdNorms, normals, sizeof(float) * 3 * 6);

    PXR_NS::VtArray<PXR_NS::GfVec3f> normalsVt;
    normalsVt.assign(usdNorms, &usdNorms[nbrFlatVerts]);
    geom->CreateNormalsAttr().Set(normalsVt); // normals

    geom->CreateSubdivisionSchemeAttr().Set(PXR_NS::TfToken("none")); // subdivision

    delete[] normals;
    delete[] flatIndices;
    delete[] flatVerts;
    delete[] usdVerts;
    delete[] usdNorms;
}

void processMesh(
    PXR_NS::UsdPrim* prim,
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
                    switch (OmniPvdUsd::getAttributeId(attributeInstList->mAttributeDef))
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

    PXR_NS::UsdGeomMesh* geom = (PXR_NS::UsdGeomMesh*)prim;
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

    PXR_NS::VtArray<int> vertexIndicesVt;
    vertexIndicesVt.assign(flatIndices, &flatIndices[nbrTri * 3]);

    PXR_NS::VtArray<int> vertexCounts = PXR_NS::VtArray<int>(nbrTri, 3);
    geom->CreateFaceVertexCountsAttr().Set(vertexCounts);
    geom->CreateFaceVertexIndicesAttr().Set(vertexIndicesVt);

    ////////////////////////////////////////////////////////////////////////////////
    // Set vertex position (point) attribute using the flat shaded verts
    ////////////////////////////////////////////////////////////////////////////////

    PXR_NS::GfVec3f *usdVerts = new PXR_NS::GfVec3f[nbrFlatVerts];
    memcpy(usdVerts, flatVerts, sizeof(float) * 3 * nbrFlatVerts);
    PXR_NS::VtArray<PXR_NS::GfVec3f> pointArrayVt;
    pointArrayVt.assign(usdVerts, &usdVerts[nbrFlatVerts]);
    geom->CreatePointsAttr().Set(pointArrayVt);

    PXR_NS::GfVec3f extents[2];
    extents[0].Set(minExt);
    extents[1].Set(maxExt);
    PXR_NS::VtArray<PXR_NS::GfVec3f> extentsVt;
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

    PXR_NS::GfVec3f *usdNorms = new PXR_NS::GfVec3f[nbrFlatVerts];
    memcpy(usdNorms, normals, sizeof(float) * 3 * nbrFlatVerts);

    PXR_NS::VtArray<PXR_NS::GfVec3f> normalsVt;
    normalsVt.assign(usdNorms, &usdNorms[nbrFlatVerts]);
    geom->CreateNormalsAttr().Set(normalsVt);

    geom->CreateSubdivisionSchemeAttr().Set(PXR_NS::TfToken("none"));

    delete[] flatVerts;
    delete[] flatIndices;
    delete[] normals;
    delete[] usdVerts;
    delete[] usdNorms;
}

void processTetMesh(
    PXR_NS::UsdPrim* prim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject
)
{
    if (!prim) return;
    if (!*prim) return;

    PXR_NS::UsdGeomMesh geom(*prim);

    // First pass: find tet indices and/or tri indices (topology is static, set once)
    int* tetIndices = nullptr;
    int nbrTets = 0;
    int* triIndices = nullptr;
    int nbrTris = 0;

    const int nbrInheritedClasses = static_cast<int>(omniPvdObject->mInheritedClassInstances.size());
    for (int c = 0; c < nbrInheritedClasses; c++)
    {
        std::vector<OmniPvdAttributeInstList*>& classAttributeLists = omniPvdObject->mInheritedClassInstances[c].mClassAttributeLists;
        const int nbrAttributes = (int)classAttributeLists.size();
        for (int j = 0; j < nbrAttributes; j++)
        {
            OmniPvdAttributeInstList* attributeInstList = classAttributeLists[j];
            if (attributeInstList && !attributeInstList->mAttributeDef->mIsUniqueList)
            {
                if (OmniPvdUsd::getAttributeId(attributeInstList->mAttributeDef) == OmniPvdUsdAttributeEnum::eUSDAttributeTets)
                {
                    OmniPvdAttributeSample* sample = (OmniPvdAttributeSample*)attributeInstList->mFirst;
                    if (sample && sample->mDataLen > 0)
                    {
                        tetIndices = (int*)sample->mData;
                        nbrTets = sample->mDataLen / (sizeof(int) * 4);
                    }
                }
                else if (OmniPvdUsd::getAttributeId(attributeInstList->mAttributeDef) == OmniPvdUsdAttributeEnum::eUSDAttributeTris)
                {
                    OmniPvdAttributeSample* sample = (OmniPvdAttributeSample*)attributeInstList->mFirst;
                    if (sample && sample->mDataLen > 0)
                    {
                        triIndices = (int*)sample->mData;
                        nbrTris = sample->mDataLen / (sizeof(int) * 3);
                    }
                }
            }
        }
    }

    if (tetIndices != nullptr && nbrTets > 0)
    {
        // Store tet topology as a custom attribute on the Mesh prim
        static const PXR_NS::TfToken tetVertexIndicesToken("omni:pvd:tetVertexIndices");
        PXR_NS::VtArray<PXR_NS::GfVec4i> tetVtxIndices(nbrTets);
        for (int t = 0; t < nbrTets; t++)
        {
            tetVtxIndices[t] = PXR_NS::GfVec4i(tetIndices[t*4], tetIndices[t*4+1], tetIndices[t*4+2], tetIndices[t*4+3]);
        }
        PXR_NS::UsdAttribute tetAttr = prim->CreateAttribute(tetVertexIndicesToken, PXR_NS::SdfValueTypeNames->Int4Array, PXR_NS::SdfVariability::SdfVariabilityUniform);
        tetAttr.Set(tetVtxIndices);

        // Extract surface faces: a triangle face is on the surface if it belongs to exactly one tet.
        // Each tet (v0,v1,v2,v3) has 4 faces with indices sorted for canonical keys.
        struct FaceKey {
            int v[3];
            bool operator==(const FaceKey& o) const { return v[0]==o.v[0] && v[1]==o.v[1] && v[2]==o.v[2]; }
        };
        struct FaceKeyHash {
            size_t operator()(const FaceKey& f) const {
                return size_t(f.v[0]) * 73856093u ^ size_t(f.v[1]) * 19349663u ^ size_t(f.v[2]) * 83492791u;
            }
        };
        // Store the original (unsorted) winding for each face
        std::unordered_map<FaceKey, std::pair<int, PXR_NS::GfVec3i>, FaceKeyHash> faceCount;
        for (int t = 0; t < nbrTets; t++)
        {
            int v0 = tetIndices[t*4+0], v1 = tetIndices[t*4+1], v2 = tetIndices[t*4+2], v3 = tetIndices[t*4+3];
            // 4 faces per tet with outward-facing winding
            PXR_NS::GfVec3i faces[4] = {
                {v0, v2, v1}, {v0, v1, v3}, {v0, v3, v2}, {v1, v2, v3}
            };
            for (int f = 0; f < 4; f++)
            {
                FaceKey key;
                key.v[0] = faces[f][0]; key.v[1] = faces[f][1]; key.v[2] = faces[f][2];
                // Sort for canonical key
                if (key.v[0] > key.v[1]) std::swap(key.v[0], key.v[1]);
                if (key.v[1] > key.v[2]) std::swap(key.v[1], key.v[2]);
                if (key.v[0] > key.v[1]) std::swap(key.v[0], key.v[1]);
                auto it = faceCount.find(key);
                if (it == faceCount.end())
                    faceCount[key] = {1, faces[f]};
                else
                    it->second.first++;
            }
        }
        // Collect faces that appear exactly once (surface boundary)
        PXR_NS::VtArray<int> faceVertexCounts;
        PXR_NS::VtArray<int> faceVertexIndices;
        for (const auto& entry : faceCount)
        {
            if (entry.second.first == 1)
            {
                faceVertexCounts.push_back(3);
                faceVertexIndices.push_back(entry.second.second[0]);
                faceVertexIndices.push_back(entry.second.second[1]);
                faceVertexIndices.push_back(entry.second.second[2]);
            }
        }
        if (faceVertexCounts.size() > 0)
        {
            geom.CreateFaceVertexCountsAttr().Set(faceVertexCounts);
            geom.CreateFaceVertexIndicesAttr().Set(faceVertexIndices);
        }
    }
    else if (triIndices != nullptr && nbrTris > 0)
    {
        PXR_NS::VtArray<int> vertexCounts(nbrTris, 3);
        PXR_NS::VtArray<int> vertexIndices;
        vertexIndices.assign(triIndices, triIndices + nbrTris * 3);
        geom.CreateFaceVertexCountsAttr().Set(vertexCounts);
        geom.CreateFaceVertexIndicesAttr().Set(vertexIndices);
    }
    geom.CreateSubdivisionSchemeAttr().Set(PXR_NS::TfToken("none"));
    geom.CreateDoubleSidedAttr().Set(true);
    // Create velocities attr so Fabric doesn't warn about missing attribute
    geom.CreateVelocitiesAttr();

    // Second pass: set per-frame animated positions and velocities
    for (int c = 0; c < nbrInheritedClasses; c++)
    {
        std::vector<OmniPvdAttributeInstList*>& classAttributeLists = omniPvdObject->mInheritedClassInstances[c].mClassAttributeLists;
        const int nbrAttributes = (int)classAttributeLists.size();
        for (int j = 0; j < nbrAttributes; j++)
        {
            OmniPvdAttributeInstList* attributeInstList = classAttributeLists[j];
            if (attributeInstList && !attributeInstList->mAttributeDef->mIsUniqueList)
            {
                OmniPvdAttributeSample* sample = (OmniPvdAttributeSample*)attributeInstList->mFirst;
                while (sample)
                {
                    switch (OmniPvdUsd::getAttributeId(attributeInstList->mAttributeDef))
                    {
                    case OmniPvdUsdAttributeEnum::eUSDAttributeDeformablePositions:
                    {
                        // Data is PxVec4 array: x,y,z,invMass per vertex (4 floats each)
                        float* positions = (float*)sample->mData;
                        int nbrVerts = sample->mDataLen / (sizeof(float) * 4);
                        if (nbrVerts > 0)
                        {
                            // Extract xyz from PxVec4 (skip w/invMass)
                            PXR_NS::VtArray<PXR_NS::GfVec3f> usdPoints(nbrVerts);
                            float* srcPtr = positions;
                            for (int i = 0; i < nbrVerts; i++)
                            {
                                usdPoints[i] = PXR_NS::GfVec3f(srcPtr[0], srcPtr[1], srcPtr[2]);
                                srcPtr += 4;
                            }
                            geom.CreatePointsAttr().Set(usdPoints, (double)sample->mTimeStamp);
                        }
                        // Also write as OmniPVD custom attribute
                        processCustomAttribute(*prim, sample, attributeInstList->mAttributeDef);
                    }
                    break;
                    case OmniPvdUsdAttributeEnum::eUSDAttributeDeformableVelocities:
                    {
                        // Data is PxVec4 array: vx,vy,vz,w per vertex (4 floats each)
                        float* velocities = (float*)sample->mData;
                        int nbrVerts = sample->mDataLen / (sizeof(float) * 4);
                        if (nbrVerts > 0)
                        {
                            PXR_NS::VtArray<PXR_NS::GfVec3f> usdVelocities(nbrVerts);
                            float* srcPtr = velocities;
                            for (int i = 0; i < nbrVerts; i++)
                            {
                                usdVelocities[i] = PXR_NS::GfVec3f(srcPtr[0], srcPtr[1], srcPtr[2]);
                                srcPtr += 4;
                            }
                            geom.CreateVelocitiesAttr().Set(usdVelocities, (double)sample->mTimeStamp);
                        }
                        // Also write as OmniPVD custom attribute
                        processCustomAttribute(*prim, sample, attributeInstList->mAttributeDef);
                    }
                    break;
                    case OmniPvdUsdAttributeEnum::eUSDAttributeTets:
                    case OmniPvdUsdAttributeEnum::eUSDAttributeTris:
                        // Already handled in first pass
                        break;
                    default:
                    {
                        processCustomAttribute(*prim, sample, attributeInstList->mAttributeDef);
                    }
                    break;
                    }
                    sample = (OmniPvdAttributeSample*)(sample->mNextAttribute);
                }
            }
        }
    }
}

void processPoints(
    PXR_NS::UsdPrim* prim,
    OmniPvdAttributeSample* attrib,
    OmniPvdObject* omniPvdObject
)
{
    if (!prim) return;
    if (!*prim) return;

    PXR_NS::UsdGeomPoints* geom = (PXR_NS::UsdGeomPoints*)prim;

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
        PXR_NS::UsdAttribute pointsAttr = geom->GetPointsAttr();
        PXR_NS::UsdAttribute widthsAttr = geom->GetWidthsAttr();
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
                            switch (OmniPvdUsd::getAttributeId(attributeInstList->mAttributeDef))
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
