// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"
#include "OmniPvdUsdAdapter.h"

// Static caches — one OVD file processed at a time, no thread safety needed
static std::unordered_map<const OmniPvdObject*, pxr::SdfPath> sSdfPathMap;
static std::unordered_map<const OmniPvdAttributeDef*, pxr::TfToken*> sTokenCache;
static std::unordered_map<std::string, pxr::TfToken*> sTokenMap;

// Single shared definition (declared extern in OmniPvdUsdAdapter.h).
// Classes whose name contains "_ref" are USD Over prims (they overlay the shared-
// layer prim they reference). PxFrictionType/PxActorType are enums that used to
// map through ePxSceneFlag; keep them rendered as USD Enum prims.
const std::unordered_map<std::string, OmniPvdUsdClassEnum> sUsdClassOverrides = {
    {"object_ref",        eUSDClassOver},
    {"shape_ref",         eUSDClassOver},
    {"convexmesh_ref",    eUSDClassOver},
    {"heightfield_ref",   eUSDClassOver},
    {"trianglemesh_ref",  eUSDClassOver},
    {"tetmesh_ref",       eUSDClassOver},
    {"PxFrictionType",    eUSDClassEnum},
    {"PxActorType",       eUSDClassEnum},
};

// ============================================================================
// PhysX → USD class mapping
// ============================================================================

static OmniPvdUsdClassEnum physxToUsdClass(OmniPvdPhysXClassEnum physxClass)
{
    switch (physxClass)
    {
    case OmniPvdPhysXClassEnum::ePxScene:           return eUSDClassGeomScope;
    case OmniPvdPhysXClassEnum::ePxSceneFlag:       return eUSDClassEnum;
    case OmniPvdPhysXClassEnum::ePxMaterial:         return eUSDClassGeomScope;
    case OmniPvdPhysXClassEnum::ePxActor:            return eUSDClassXform;
    case OmniPvdPhysXClassEnum::ePxShape:            return eUSDClassXform;
    case OmniPvdPhysXClassEnum::ePxGeomSphere:       return eUSDClassGeomSphere;
    case OmniPvdPhysXClassEnum::ePxGeomCapsule:      return eUSDClassGeomCapsule;
    case OmniPvdPhysXClassEnum::ePxGeomBox:          return eUSDClassGeomCube;
    case OmniPvdPhysXClassEnum::ePxGeomPlane:        return eUSDClassGeomPlane;
    case OmniPvdPhysXClassEnum::ePxGeomConvexMesh:   return eUSDClassXform;
    case OmniPvdPhysXClassEnum::ePxGeomHeightfield:  return eUSDClassXform;
    case OmniPvdPhysXClassEnum::ePxGeomTriangleMesh: return eUSDClassXform;
    case OmniPvdPhysXClassEnum::ePxConvexMesh:       return eUSDClassGeomMesh;
    case OmniPvdPhysXClassEnum::ePxHeightfield:      return eUSDClassGeomMesh;
    case OmniPvdPhysXClassEnum::ePxTriangleMesh:     return eUSDClassGeomMesh;
    case OmniPvdPhysXClassEnum::ePxTetrahedronMesh:  return eUSDClassGeomTetMesh;
    case OmniPvdPhysXClassEnum::ePxGeomTetMesh:      return eUSDClassXform;
    case OmniPvdPhysXClassEnum::ePxArticulation:     return eUSDClassGeomScope;
    case OmniPvdPhysXClassEnum::ePxArticulationJoint:return eUSDClassGeomScope;
    case OmniPvdPhysXClassEnum::ePxArticulationLink: return eUSDClassXform;
    case OmniPvdPhysXClassEnum::ePxJoint:            return eUSDClassGeomScope;
    case OmniPvdPhysXClassEnum::ePxParticleBuffer:   return eUSDClassGeomPoints;
    case OmniPvdPhysXClassEnum::ePxDeformableVolume: return eUSDClassXform;
    case OmniPvdPhysXClassEnum::ePxDeformableSurface:return eUSDClassXform;
    case OmniPvdPhysXClassEnum::ePxDeformableVolumeMesh: return eUSDClassGeomScope;
    case OmniPvdPhysXClassEnum::ePxCustomGeometryCylinder: return eUSDClassCylinder;
    case OmniPvdPhysXClassEnum::ePxCustomGeometryCone:     return eUSDClassCone;
    case OmniPvdPhysXClassEnum::ePxConvexCoreCylinder:     return eUSDClassCylinder;
    case OmniPvdPhysXClassEnum::ePxConvexCoreCone:         return eUSDClassCone;
    case OmniPvdPhysXClassEnum::ePxConvexCoreGeometry:     return eUSDClassXform;
    case OmniPvdPhysXClassEnum::ePxInternalOmnniPvd: return eUSDClassGeomScope;
    case OmniPvdPhysXClassEnum::ePxUndefined:
    default:
        return eUSDClassGeomScope;
    }
}

// ============================================================================
// Adapter function implementations
// ============================================================================

OmniPvdUsdClassEnum OmniPvdUsd::getClassId(const OmniPvdClass* cls)
{
    // Check explicit overrides first (visible in OmniPvdUsdAdapter.h)
    auto it = sUsdClassOverrides.find(cls->mClassName);
    if (it != sUsdClassOverrides.end())
        return it->second;
    return physxToUsdClass(cls->mPhysXBaseProcessingClassId);
}

OmniPvdUsdAttributeEnum OmniPvdUsd::getAttributeId(const OmniPvdAttributeDef* attrib)
{
    // PvdDomAttributeEnum values are in the same order as OmniPvdUsdAttributeEnum.
    // Enforce per-entry at compile time so independent edits to either enum break
    // the build instead of silently producing the wrong USD attribute.
    #define PVDDOM_USD_ENUM_CHECK(dom, usd) \
        static_assert(static_cast<int>(dom) == static_cast<int>(usd), \
            #dom " and " #usd " must have the same integer value")
    PVDDOM_USD_ENUM_CHECK(ePvdDomAttribTranslateOp,         eUSDAttributeTranslateOp);
    PVDDOM_USD_ENUM_CHECK(ePvdDomAttribScaleOp,             eUSDAttributeScaleOp);
    PVDDOM_USD_ENUM_CHECK(ePvdDomAttribRotationOp,          eUSDAttributeRotationOp);
    PVDDOM_USD_ENUM_CHECK(ePvdDomAttribDisplayColor,        eUSDAttributeDisplayColor);
    PVDDOM_USD_ENUM_CHECK(ePvdDomAttribCustom,              eUSDAttributeCustom);
    PVDDOM_USD_ENUM_CHECK(ePvdDomAttribChildNode,           eUSDAttributeChildNode);
    PVDDOM_USD_ENUM_CHECK(ePvdDomAttribAxis,                eUSDAttributeAxis);
    PVDDOM_USD_ENUM_CHECK(ePvdDomAttribHeight,              eUSDAttributeHeight);
    PVDDOM_USD_ENUM_CHECK(ePvdDomAttribRadius,              eUSDAttributeRadius);
    PVDDOM_USD_ENUM_CHECK(ePvdDomAttribVerts,               eUSDAttributeVerts);
    PVDDOM_USD_ENUM_CHECK(ePvdDomAttribTris,                eUSDAttributeTris);
    PVDDOM_USD_ENUM_CHECK(ePvdDomAttribPoints,              eUSDAttributePoints);
    PVDDOM_USD_ENUM_CHECK(ePvdDomAttribDeformablePositions, eUSDAttributeDeformablePositions);
    PVDDOM_USD_ENUM_CHECK(ePvdDomAttribDeformableVelocities,eUSDAttributeDeformableVelocities);
    PVDDOM_USD_ENUM_CHECK(ePvdDomAttribTets,                eUSDAttributeTets);
    PVDDOM_USD_ENUM_CHECK(ePvdDomAttribEnum,                eUSDAttributeEnum);
    PVDDOM_USD_ENUM_CHECK(ePvdDomAttribTransformFork,       eUSDAttributeTransformFork);
    PVDDOM_USD_ENUM_CHECK(ePvdDomAttribNone,                eUSDAttributeNone);
    #undef PVDDOM_USD_ENUM_CHECK
    return static_cast<OmniPvdUsdAttributeEnum>(static_cast<int>(attrib->mAttributeId));
}

pxr::SdfPath& OmniPvdUsd::getPrimPath(OmniPvdObject* obj)
{
    return sSdfPathMap[obj];
}

void OmniPvdUsd::setPrimPath(OmniPvdObject* obj, const pxr::SdfPath& path)
{
    sSdfPathMap[obj] = path;
}

pxr::TfToken* OmniPvdUsd::getToken(OmniPvdAttributeDef* attrib, const std::string& prefix)
{
    auto it = sTokenCache.find(attrib);
    if (it != sTokenCache.end())
        return it->second;

    // Derive prefix from attribute role if not provided explicitly.
    // Prefix constants are in OmniPvdUsdAdapter.h for visibility.
    //   - Enum values (display tokens, not attribute names) → no prefix
    //   - eOBJECT_HANDLE attributes → "omni:pvdh:"
    //   - Everything else → "omni:pvd:"
    std::string effectivePrefix = prefix;
    if (effectivePrefix.empty())
    {
        bool isEnumValue = attrib->mClass && attrib->mClass->mIsEnumClass;
        if (isEnumValue)
            effectivePrefix = OmniPvdUsdPrefix::None;
        else if (attrib->mDataType == OmniPvdDataType::eOBJECT_HANDLE)
            effectivePrefix = OmniPvdUsdPrefix::Handle;
        else
            effectivePrefix = OmniPvdUsdPrefix::Attrib;
    }

    pxr::TfToken* token = getTokenByName(attrib->mAttributeName, effectivePrefix);
    sTokenCache[attrib] = token;
    return token;
}

pxr::TfToken* OmniPvdUsd::getTokenByName(const std::string& name, const std::string& prefix)
{
    const std::string key = prefix + name;
    auto it = sTokenMap.find(key);
    if (it != sTokenMap.end())
        return it->second;

    pxr::TfToken* token = new pxr::TfToken(key);
    sTokenMap[key] = token;
    return token;
}

void OmniPvdUsd::reset()
{
    sSdfPathMap.clear();
    sTokenCache.clear();
    for (auto& pair : sTokenMap)
    {
        delete pair.second;
    }
    sTokenMap.clear();
}
