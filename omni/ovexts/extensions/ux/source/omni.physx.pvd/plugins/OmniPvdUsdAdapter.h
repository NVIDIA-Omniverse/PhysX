// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

// USD adapter layer for pvddom.
// Provides USD-specific types (SdfPath, TfToken, UsdClassEnum, UsdAttributeEnum)
// as static caches wrapping the pure pvddom DOM types. This keeps pvddom free of
// USD dependencies while giving omni.physx.pvd the USD integration it needs.
//
// Usage: replace direct member access with adapter function calls:
//   omniPvdClass->mUsdClassId          -->  OmniPvdUsd::getClassId(omniPvdClass)
//   attribDef->mUsdAttributeId         -->  OmniPvdUsd::getAttributeId(attribDef)
//   attribDef->mPxrToken               -->  OmniPvdUsd::getToken(attribDef)
//   omniPvdObject->mPrimPath           -->  OmniPvdUsd::getPrimPath(omniPvdObject)

#include "PvdDom.h"
#include "PvdDomUtils.h"
#include "PvdDomParser.h"

#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/path.h>

#include <unordered_map>
#include <string>

// ============================================================================
// USD class enum — maps PhysX classes to USD prim types
// ============================================================================
enum OmniPvdUsdClassEnum
{
    eUSDClassXform,
    eUSDClassGeomScope,
    eUSDClassOver,
    eUSDClassGeomSphere,
    eUSDClassGeomCapsule,
    eUSDClassGeomCube,
    eUSDClassGeomPlane,
    eUSDClassGeomMesh,
    eUSDClassGeomTetMesh,
    eUSDClassGeomPoints,
    eUSDClassEnum,
    eUSDClassDistantLight,
    eUSDClassCylinder,
    eUSDClassCone
};

// ============================================================================
// USD attribute enum — maps DOM attribute types to USD attribute semantics
// ============================================================================
enum OmniPvdUsdAttributeEnum
{
    eUSDAttributeTranslateOp,
    eUSDAttributeScaleOp,
    eUSDAttributeRotationOp,
    eUSDAttributeDisplayColor,
    eUSDAttributeCustom,
    eUSDAttributeChildNode,
    eUSDAttributeAxis,
    eUSDAttributeHeight,
    eUSDAttributeRadius,
    eUSDAttributeVerts,
    eUSDAttributeTris,
    eUSDAttributePoints,
    eUSDAttributeDeformablePositions,
    eUSDAttributeDeformableVelocities,
    eUSDAttributeTets,
    eUSDAttributeEnum,
    eUSDAttributeTransformFork,
    eUSDAttributeNone
};

// ============================================================================
// Adapter functions — static caches, no function signature changes needed
// ============================================================================
// Token prefixes — applied to USD attribute names.
// Visible here so the convention is explicit and not buried in functions.
// ============================================================================
namespace OmniPvdUsdPrefix
{
    static const std::string Attrib("omni:pvd:");    // regular attributes
    static const std::string Handle("omni:pvdh:");   // eOBJECT_HANDLE attributes
    static const std::string None("");               // enum values, no prefix
}

// ============================================================================
// USD class overrides for DOM classes. Takes priority over physxToUsdClass().
// Single shared definition lives in OmniPvdUsdAdapter.cpp; declared extern here
// so every TU sees the same object rather than its own per-TU copy.
// ============================================================================
extern const std::unordered_map<std::string, OmniPvdUsdClassEnum> sUsdClassOverrides;

// ============================================================================
// Adapter functions — static caches, no function signature changes needed
// ============================================================================
namespace OmniPvdUsd
{
    // Get USD class enum for a DOM class (based on PhysX class mapping + overrides)
    OmniPvdUsdClassEnum getClassId(const OmniPvdClass* cls);

    // Get USD attribute enum for a DOM attribute
    OmniPvdUsdAttributeEnum getAttributeId(const OmniPvdAttributeDef* attrib);

    // Get/set pxr::SdfPath for a DOM object (replaces mPrimPath)
    pxr::SdfPath& getPrimPath(OmniPvdObject* obj);
    void setPrimPath(OmniPvdObject* obj, const pxr::SdfPath& path);

    // Get pxr::TfToken for an attribute def (replaces mPxrToken).
    // Prefix is derived automatically from data type if not provided.
    pxr::TfToken* getToken(OmniPvdAttributeDef* attrib, const std::string& prefix = std::string(""));

    // Get pxr::TfToken by name directly (replaces getAttributeToken / mTokenMap lookups)
    pxr::TfToken* getTokenByName(const std::string& name, const std::string& prefix = std::string(""));

    // Clear all cached state (call when DOM state is destroyed)
    void reset();
}
