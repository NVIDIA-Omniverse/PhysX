// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-1
 *
 * @implements REQ-PARSE-SHAPE-002
 * @covers AC-4
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-10 AC-11 AC-12 AC-13
 */

#include <carb/logging/Log.h>

#include "PhysXCustomGeometry.h"
#include "OmniPhysX.h"
#include "usdLoad/LoadUsd.h"

#include <omni/physics/parse/CustomTokens.h>

using namespace ::physx;

namespace omni
{
namespace physx
{


CustomPhysXGeometryCallback::CustomPhysXGeometryCallback(omni::physics::parse::ObjectKey key, const usdparser::CustomPhysxShapeDesc& shapeDesc, const CustomGeometryInfo& geometryInfo, void* userObject)
{
    mCustomGeometryKey = key;
    mCustomGeometryInfo = geometryInfo;
    mUserObject = userObject;
}

void CustomPhysXGeometryCallback::release()
{
}

::physx::PxBounds3 CustomPhysXGeometryCallback::getLocalBounds(const ::physx::PxGeometry& geometry) const
{
    return mCustomGeometryInfo.customGeometryCb.computeCustomGeometryLocalBoundsFn(geometry, mUserObject);
}

bool CustomPhysXGeometryCallback::generateContacts(const ::physx::PxGeometry& geom0, const ::physx::PxGeometry& geom1, const ::physx::PxTransform& pose0, const ::physx::PxTransform& pose1,
    const float contactDistance, const float meshContactMargin, const float toleranceLength,
    ::physx::PxContactBuffer& contactBuffer) const
{
    void* userObject1 = nullptr;
    if (geom1.getType() == PxGeometryType::eCUSTOM)
    {
        const PxCustomGeometry& customGeom1 = static_cast<const PxCustomGeometry&>(geom1);
        const ::physx::PxCustomGeometry::Type customType = customGeom1.callbacks->getCustomType();
        const bool knownType = OmniPhysX::getInstance().getCustomGeometryManager().isKnownType(customType);
        if (knownType)
        {
            const CustomPhysXGeometryCallback* cb = (const CustomPhysXGeometryCallback*)customGeom1.callbacks;
            userObject1 = cb->getUserObject();
        }
    }
    return mCustomGeometryInfo.customGeometryCb.generateCustomGeometryContactsFn(geom0, geom1, pose0, pose1, contactDistance, meshContactMargin, toleranceLength, contactBuffer, mUserObject, userObject1);
}

::physx::PxU32 CustomPhysXGeometryCallback::raycast(const ::physx::PxVec3& origin, const ::physx::PxVec3& unitDir, const ::physx::PxGeometry& geom, const ::physx::PxTransform& pose,
    ::physx::PxReal maxDist, ::physx::PxHitFlags hitFlags, ::physx::PxU32 maxHits, ::physx::PxGeomRaycastHit* rayHits, ::physx::PxU32 stride, ::physx::PxRaycastThreadContext* threadContext) const
{
    if (!mCustomGeometryInfo.customGeometryCb.raycastCustomGeometryFn)
        return 0;

    return mCustomGeometryInfo.customGeometryCb.raycastCustomGeometryFn(origin, unitDir, geom, pose, maxDist, hitFlags, maxHits, rayHits, stride, threadContext, mUserObject);
}

bool CustomPhysXGeometryCallback::overlap(const ::physx::PxGeometry& geom0, const ::physx::PxTransform& pose0, const ::physx::PxGeometry& geom1, const ::physx::PxTransform& pose1, ::physx::PxOverlapThreadContext* threadContext) const
{
    if(!mCustomGeometryInfo.customGeometryCb.overlapCustomGeometryFn)
        return false;

    return mCustomGeometryInfo.customGeometryCb.overlapCustomGeometryFn(geom0, pose0, geom1, pose1, threadContext, mUserObject);
}

bool CustomPhysXGeometryCallback::sweep(const ::physx::PxVec3& unitDir, const ::physx::PxReal maxDist,
    const ::physx::PxGeometry& geom0, const ::physx::PxTransform& pose0, const ::physx::PxGeometry& geom1, const ::physx::PxTransform& pose1,
    ::physx::PxGeomSweepHit& sweepHit, ::physx::PxHitFlags hitFlags, const ::physx::PxReal inflation, ::physx::PxSweepThreadContext* threadContext) const
{
    if (!mCustomGeometryInfo.customGeometryCb.sweepCustomGeometryFn)
        return false;

    return mCustomGeometryInfo.customGeometryCb.sweepCustomGeometryFn(unitDir, maxDist, geom0, pose0, geom1, pose1, sweepHit, hitFlags, inflation, threadContext, mUserObject);
}

void CustomPhysXGeometryCallback::visualize(const ::physx::PxGeometry& geometry, ::physx::PxRenderOutput& out, const ::physx::PxTransform& absPose, const ::physx::PxBounds3& cullbox) const
{
    if (mCustomGeometryInfo.customGeometryCb.visualizeCustomGeometryFn)
    {
        mCustomGeometryInfo.customGeometryCb.visualizeCustomGeometryFn(geometry, absPose, out, cullbox, mUserObject);
    }
}

void CustomPhysXGeometryCallback::computeMassProperties(const ::physx::PxGeometry& geometry, ::physx::PxMassProperties& massProperties) const
{
    mCustomGeometryInfo.customGeometryCb.computeCustomGeometryMassPropertiesFn(geometry, massProperties, mUserObject);
}

bool CustomPhysXGeometryCallback::usePersistentContactManifold(const ::physx::PxGeometry& geometry, ::physx::PxReal& breakingThreshold) const
{
    return mCustomGeometryInfo.customGeometryCb.useCustomGeometryPersistentContactManifoldFn(geometry, breakingThreshold, mUserObject);
}

PhysXCustomGeometryManager::PhysXCustomGeometryManager()
    : mGeometryRegistryCounter(1)
{
}

PhysXCustomGeometryManager::~PhysXCustomGeometryManager()
{
}

CustomPhysXGeometryCallback* PhysXCustomGeometryManager::createCustomGeometry(omni::physics::parse::ObjectKey primKey, const usdparser::CustomPhysxShapeDesc& customShapeDesc)
{
    CustomPhysXGeometryCallback* customGeom = nullptr;
    CustomGeometryTypeMap::const_iterator fit =
        mCustomGeometryTypeMap.find(customShapeDesc.customGeometryTokenHash);
    if (fit != mCustomGeometryTypeMap.end())
    {
        const CustomGeometryInfo& geomInfo = fit->second;
        // The consumer callback receives the attach the geometry belongs to, not a stage id (ADR-0016).
        // No AttachedStage reaches this manager, so the handle comes from the active-attach accessor;
        // it is kNoAttach when nothing (or more than one thing) is attached, matching what the stage
        // id used to report in that case.
        const AttachHandle attachHandle = omni::physx::usdparser::UsdLoad::getUsdLoad()->getActiveAttachHandle();
        void* userObject = geomInfo.customGeometryCb.createCustomGeometryFn(primKey, attachHandle, *geomInfo.typeId, geomInfo.customGeometryCb.userData);
        if (userObject)
        {
            customGeom = ICE_NEW(CustomPhysXGeometryCallback)(primKey, customShapeDesc, geomInfo, userObject);
            mCustomGeometryMap[primKey] = customGeom;
        }
    }
    return customGeom;
}

void PhysXCustomGeometryManager::removeCustomGeometry(omni::physics::parse::ObjectKey primKey)
{
    CustomGeometryMap::iterator fit = mCustomGeometryMap.find(primKey);
    if (fit != mCustomGeometryMap.end())
    {
        mCustomGeometryMap.erase(fit);
    }
}

size_t PhysXCustomGeometryManager::registerCustomGeometry(const char* customGeometryAPIToken, ICustomGeometryCallback& geometryCallback)
{
    const size_t tokenHash = computeCustomGeometryHash(customGeometryAPIToken);
    if (mCustomGeometryTypeMap.find(tokenHash) != mCustomGeometryTypeMap.end())
    {
        CARB_LOG_ERROR("Custom Geometry Type (%s) already registered.", customGeometryAPIToken);
        return kInvalidCustomGeometryRegId;
    }

    if (!geometryCallback.createCustomGeometryFn|| !geometryCallback.computeCustomGeometryMassPropertiesFn || !geometryCallback.generateCustomGeometryContactsFn
        || !geometryCallback.releaseCustomGeometryFn || !geometryCallback.computeCustomGeometryLocalBoundsFn || !geometryCallback.useCustomGeometryPersistentContactManifoldFn)
    {
        CARB_LOG_ERROR("Custom Geometry Type (%s) has invalid custom geometry callback, please provide all required functions.", customGeometryAPIToken);
        return kInvalidCustomGeometryRegId;
    }

    const size_t currentRegistryCounter = mGeometryRegistryCounter;
    // A.B. the default typeId generate code does not really work here
    void* typeMem = ICE_ALLOC(sizeof(PxCustomGeometry::Type));
    const ::physx::PxU32 typeId = OmniPhysX::getInstance().getFreeTypeId();
    PX_COMPILE_TIME_ASSERT(sizeof(PxCustomGeometry::Type) == sizeof(::physx::PxU32));
    memcpy(typeMem, &typeId, sizeof(::physx::PxU32));
    PxCustomGeometry::Type* uniqueType = reinterpret_cast<PxCustomGeometry::Type*>(typeMem);
    CustomGeometryInfo info = { customGeometryAPIToken, geometryCallback, uniqueType };
    mCustomGeometryRegistryMap[currentRegistryCounter] = info;
    mCustomGeometryTypeMap[tokenHash] = info;
    mGeometryRegistryCounter++;

    // Route the registration into the USD-free parse-core custom-token registry so
    // both the native USD walker and the ovstage walker recognize this custom
    // geometry token (REQ-PARSE-CORE-005); the schema parser's
    // `IUsdPhysics::addCustomShapeToken` surface is retired.
    omni::physics::parse::registerCustomToken(omni::physics::parse::CustomTokenKind::eShape, customGeometryAPIToken);

    return currentRegistryCounter;
}

void PhysXCustomGeometryManager::unregisterCustomGeometry(size_t id)
{
    CustomGeometryRegistryMap::const_iterator fit = mCustomGeometryRegistryMap.find(id);
    if (fit != mCustomGeometryRegistryMap.end())
    {
        const std::string& jt = fit->second.customGeomtryAPIToken;
        PxCustomGeometry::Type* id = fit->second.typeId;
        ICE_FREE(id);
        mCustomGeometryTypeMap.erase(computeCustomGeometryHash(jt));
        omni::physics::parse::unregisterCustomToken(omni::physics::parse::CustomTokenKind::eShape, jt);
        mCustomGeometryRegistryMap.erase(fit);
    }
}

size_t registerCustomGeometry(const char* schemaAPIToken, ICustomGeometryCallback& geomCallback)
{
    return OmniPhysX::getInstance().getCustomGeometryManager().registerCustomGeometry(schemaAPIToken, geomCallback);
}

void unregisterCustomGeometry(size_t id)
{
    OmniPhysX::getInstance().getCustomGeometryManager().unregisterCustomGeometry(id);
}

}
}

void fillInterface(omni::physx::IPhysxCustomGeometry& iface)
{
    iface.registerCustomGeometry = omni::physx::registerCustomGeometry;
    iface.unregisterCustomGeometry = omni::physx::unregisterCustomGeometry;
}
