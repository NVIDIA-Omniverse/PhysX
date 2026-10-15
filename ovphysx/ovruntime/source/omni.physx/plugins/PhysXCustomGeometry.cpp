// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CUSTOM-PARAMETERS-001
// @covers AC-1

// @implements REQ-CUSTOM-LIFETIME-001
// @covers AC-4 AC-1 AC-2 AC-3

// @implements REQ-ATTACH-IO-001
// @covers AC-3

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
#include <memory>
#include <algorithm>

#include "PhysXCustomGeometry.h"
#include "OmniPhysX.h"
#include "RuntimeExecution.h"
#include "usdLoad/LoadUsd.h"

#include <omni/physics/parse/CustomTokens.h>

using namespace ::physx;

namespace omni
{
namespace physx
{


CustomPhysXGeometryCallback::CustomPhysXGeometryCallback(PhysXCustomGeometryManager& manager, const usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey key, const usdparser::CustomPhysxShapeDesc& shapeDesc, const CustomGeometryInfo& geometryInfo, void* userObject)
    : mManager(manager), mAttachedStage(owner)
{
    RuntimeCallbackScope callbackScope;
    mCustomGeometryKey = key;
    mCustomGeometryInfo = geometryInfo;
    mUserObject = userObject;
}

CustomPhysXGeometryCallback::~CustomPhysXGeometryCallback()
{
    mManager.forgetGeometry(*this);
    RuntimeCallbackScope callbackScope;
    try
    {
        const omni::physx::ICustomGeometryCallback & callback = mCustomGeometryInfo.customGeometryCb;
        if (callback.releaseCustomGeometryObjectFn)
            callback.releaseCustomGeometryObjectFn(mCustomGeometryKey, mAttachedStage.getAttachHandle(),
                mUserObject, callback.userData);
        else
            callback.releaseCustomGeometryFn(mCustomGeometryKey, mAttachedStage.getAttachHandle(), callback.userData);
    }
    catch (...)
    {
        CARB_LOG_ERROR("Custom geometry release callback threw an exception.");
    }
}

void CustomPhysXGeometryCallback::release()
{
    RuntimeCallbackScope callbackScope;
}

::physx::PxBounds3 CustomPhysXGeometryCallback::getLocalBounds(const ::physx::PxGeometry& geometry) const
{
    RuntimeCallbackScope callbackScope;
    return mCustomGeometryInfo.customGeometryCb.computeCustomGeometryLocalBoundsFn(geometry, mUserObject);
}

bool CustomPhysXGeometryCallback::generateContacts(const ::physx::PxGeometry& geom0, const ::physx::PxGeometry& geom1, const ::physx::PxTransform& pose0, const ::physx::PxTransform& pose1,
    const float contactDistance, const float meshContactMargin, const float toleranceLength,
    ::physx::PxContactBuffer& contactBuffer) const
{
    RuntimeCallbackScope callbackScope;
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
    RuntimeCallbackScope callbackScope;
    if (!mCustomGeometryInfo.customGeometryCb.raycastCustomGeometryFn)
        return 0;

    return mCustomGeometryInfo.customGeometryCb.raycastCustomGeometryFn(origin, unitDir, geom, pose, maxDist, hitFlags, maxHits, rayHits, stride, threadContext, mUserObject);
}

bool CustomPhysXGeometryCallback::overlap(const ::physx::PxGeometry& geom0, const ::physx::PxTransform& pose0, const ::physx::PxGeometry& geom1, const ::physx::PxTransform& pose1, ::physx::PxOverlapThreadContext* threadContext) const
{
    RuntimeCallbackScope callbackScope;
    if(!mCustomGeometryInfo.customGeometryCb.overlapCustomGeometryFn)
        return false;

    return mCustomGeometryInfo.customGeometryCb.overlapCustomGeometryFn(geom0, pose0, geom1, pose1, threadContext, mUserObject);
}

bool CustomPhysXGeometryCallback::sweep(const ::physx::PxVec3& unitDir, const ::physx::PxReal maxDist,
    const ::physx::PxGeometry& geom0, const ::physx::PxTransform& pose0, const ::physx::PxGeometry& geom1, const ::physx::PxTransform& pose1,
    ::physx::PxGeomSweepHit& sweepHit, ::physx::PxHitFlags hitFlags, const ::physx::PxReal inflation, ::physx::PxSweepThreadContext* threadContext) const
{
    RuntimeCallbackScope callbackScope;
    if (!mCustomGeometryInfo.customGeometryCb.sweepCustomGeometryFn)
        return false;

    return mCustomGeometryInfo.customGeometryCb.sweepCustomGeometryFn(unitDir, maxDist, geom0, pose0, geom1, pose1, sweepHit, hitFlags, inflation, threadContext, mUserObject);
}

void CustomPhysXGeometryCallback::visualize(const ::physx::PxGeometry& geometry, ::physx::PxRenderOutput& out, const ::physx::PxTransform& absPose, const ::physx::PxBounds3& cullbox) const
{
    RuntimeCallbackScope callbackScope;
    if (mCustomGeometryInfo.customGeometryCb.visualizeCustomGeometryFn)
    {
        mCustomGeometryInfo.customGeometryCb.visualizeCustomGeometryFn(geometry, absPose, out, cullbox, mUserObject);
    }
}

void CustomPhysXGeometryCallback::computeMassProperties(const ::physx::PxGeometry& geometry, ::physx::PxMassProperties& massProperties) const
{
    RuntimeCallbackScope callbackScope;
    mCustomGeometryInfo.customGeometryCb.computeCustomGeometryMassPropertiesFn(geometry, massProperties, mUserObject);
}

bool CustomPhysXGeometryCallback::usePersistentContactManifold(const ::physx::PxGeometry& geometry, ::physx::PxReal& breakingThreshold) const
{
    RuntimeCallbackScope callbackScope;
    return mCustomGeometryInfo.customGeometryCb.useCustomGeometryPersistentContactManifoldFn(geometry, breakingThreshold, mUserObject);
}

PhysXCustomGeometryManager::PhysXCustomGeometryManager()
    : mGeometryRegistryCounter(1)
{
}

PhysXCustomGeometryManager::~PhysXCustomGeometryManager()
{
}

CustomPhysXGeometryCallback* PhysXCustomGeometryManager::createCustomGeometry(const usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey primKey, const usdparser::CustomPhysxShapeDesc& customShapeDesc)
{
    CustomPhysXGeometryCallback* customGeom = nullptr;
    const auto types = getCustomGeometryTypeMap();
    CustomGeometryTypeMap::const_iterator fit =
        types.find(customShapeDesc.customGeometryTokenHash);
    if (fit != types.end())
    {
        const CustomGeometryInfo geomInfo = fit->second;
        // The consumer callback receives the attach the geometry belongs to, not a stage id (ADR-0016).
        const AttachHandle attachHandle = owner.getAttachHandle();
        RuntimeCallbackScope callbackScope(attachHandle, false, &owner);
        void* userObject = geomInfo.customGeometryCb.createCustomGeometryFn(primKey, attachHandle, *geomInfo.typeId, geomInfo.customGeometryCb.userData);
        if (userObject)
        {
            std::unique_ptr<CustomPhysXGeometryCallback> pending;
            try
            {
                pending.reset(ICE_NEW(CustomPhysXGeometryCallback)(*this, owner, primKey, customShapeDesc, geomInfo, userObject));
            }
            catch (...)
            {
                const omni::physx::ICustomGeometryCallback & callback = geomInfo.customGeometryCb;
                if (callback.releaseCustomGeometryObjectFn)
                    callback.releaseCustomGeometryObjectFn(primKey, attachHandle, userObject, callback.userData);
                else
                    callback.releaseCustomGeometryFn(primKey, attachHandle, callback.userData);
                throw;
            }
            // Validate callback output while ownership is still provisional.
            // Invalid bounds or a throwing callback cannot leave an orphaned
            // plugin state or a discoverable geometry in the owner's registry.
            const PxBounds3 bounds = pending->getLocalBounds(PxCustomGeometry(*pending));
            if (!bounds.isValid() || bounds.isEmpty())
            {
                CARB_LOG_ERROR("Custom geometry returned invalid or empty local bounds.");
                return nullptr;
            }
            {
                std::lock_guard<std::mutex> lock(mShapesMutex);
                mShapes.emplace(GeometryTrackingKey{GeometryTrackingKind::Unbound,pending.get()}, pending.get());
                pending->mTrackingKey = {GeometryTrackingKind::Unbound,pending.get()};
            }
            owner.customGeometries.emplace(primKey, pending.get());
            customGeom = pending.release();
        }
    }
    return customGeom;
}

void PhysXCustomGeometryManager::bindShape(CustomPhysXGeometryCallback& callback, const PxShape& shape)
{
    std::lock_guard<std::mutex> lock(mShapesMutex);
    std::map<GeometryTrackingKey, CustomPhysXGeometryCallback*>::node_type entry = mShapes.extract(callback.mTrackingKey);
    CARB_ASSERT(!entry.empty());
    entry.key() = {GeometryTrackingKind::Shape,static_cast<const PxBase*>(&shape)};
    callback.mTrackingKey = entry.key();
    mShapes.insert(std::move(entry)); // reuse the provisional node; no allocation
}

void PhysXCustomGeometryManager::forgetGeometry(CustomPhysXGeometryCallback& callback) noexcept
{
    {
        std::lock_guard<std::mutex> lock(mShapesMutex);
        if (callback.mTrackingKey.address) mShapes.erase(callback.mTrackingKey);
        callback.mTrackingKey = {};
    }
    std::unordered_multimap<omni::physics::parse::ObjectKey, omni::physx::CustomPhysXGeometryCallback *, omni::physics::parse::ObjectKey::Hash> & geometries = callback.mAttachedStage.customGeometries;
    using GeometryIterator = std::unordered_multimap<omni::physics::parse::ObjectKey, CustomPhysXGeometryCallback*, omni::physics::parse::ObjectKey::Hash>::iterator;
    const std::pair<GeometryIterator, GeometryIterator> range = geometries.equal_range(callback.mCustomGeometryKey);
    for (GeometryIterator it=range.first; it!=range.second; ++it)
        if (it->second == &callback) { geometries.erase(it); break; }
}

void PhysXCustomGeometryManager::releaseUnboundGeometry(CustomPhysXGeometryCallback* callback)
{
    // Use the provisional identity as a key, without dereferencing it: native
    // construction rollback may already have destroyed a bound shape/callback.
    bool unbound = false;
    {
        std::lock_guard<std::mutex> lock(mShapesMutex);
        const std::map<GeometryTrackingKey, omni::physx::CustomPhysXGeometryCallback *>::iterator found = mShapes.find({GeometryTrackingKind::Unbound,callback});
        unbound = found != mShapes.end() && found->second == callback;
    }
    if (unbound) delete callback;
}

void PhysXCustomGeometryManager::onRelease(const PxBase* object, void*, PxDeletionEventFlag::Enum event)
{
    if (event != PxDeletionEventFlag::eMEMORY_RELEASE) return;
    CustomPhysXGeometryCallback* callback = nullptr;
    {
        std::lock_guard<std::mutex> lock(mShapesMutex);
        const std::map<GeometryTrackingKey, omni::physx::CustomPhysXGeometryCallback *>::iterator found = mShapes.find({GeometryTrackingKind::Shape,object});
        if (found == mShapes.end()) return;
        callback = found->second;
        mShapes.erase(found);
        callback->mTrackingKey = {};
    }
    // Do not register/unregister SDK listeners from their dispatch. A plugin
    // release runs under the existing callback reentry guard in the destructor.
    delete callback;
}

void PhysXCustomGeometryManager::clear(const usdparser::AttachedStage& owner)
{
    CustomGeometryMap retiring;
    retiring.swap(owner.customGeometries);
    for (const CustomGeometryMap::value_type& entry : retiring)
        delete entry.second;
}

void PhysXCustomGeometryManager::clear()
{
    for (const auto& entry : usdparser::UsdLoad::getUsdLoad()->getAttachedStages())
        clear(*entry.second);
}

void PhysXCustomGeometryManager::removeCustomGeometry(const usdparser::AttachedStage& owner, omni::physics::parse::ObjectKey primKey)
{
    auto& geometries = owner.customGeometries;
    auto fit = geometries.find(primKey);
    while (fit != geometries.end())
    {
        CustomPhysXGeometryCallback* geometry = fit->second;
        geometries.erase(fit);
        delete geometry;
        fit = geometries.find(primKey);
    }
}

size_t PhysXCustomGeometryManager::registerCustomGeometry(const char* customGeometryAPIToken, ICustomGeometryCallback& geometryCallback)
{
    std::lock_guard<carb::tasking::MutexWrapper> lock(mRegistryMutex);
    const size_t tokenHash = computeCustomGeometryHash(customGeometryAPIToken);
    if (mCustomGeometryTypeMap.find(tokenHash) != mCustomGeometryTypeMap.end())
    {
        CARB_LOG_ERROR("Custom Geometry Type (%s) already registered.", customGeometryAPIToken);
        return kInvalidCustomGeometryRegId;
    }

    if (!geometryCallback.createCustomGeometryFn|| !geometryCallback.computeCustomGeometryMassPropertiesFn || !geometryCallback.generateCustomGeometryContactsFn
        || (!geometryCallback.releaseCustomGeometryFn && !geometryCallback.releaseCustomGeometryObjectFn)
        || !geometryCallback.computeCustomGeometryLocalBoundsFn || !geometryCallback.useCustomGeometryPersistentContactManifoldFn)
    {
        CARB_LOG_ERROR("Custom Geometry Type (%s) has invalid custom geometry callback, please provide all required functions.", customGeometryAPIToken);
        return kInvalidCustomGeometryRegId;
    }

    const size_t currentRegistryCounter = mGeometryRegistryCounter;
    // A.B. the default typeId generate code does not really work here
    void* typeMem = ICE_ALLOC(sizeof(PxCustomGeometry::Type));
    if (!typeMem) throw std::bad_alloc();
    const ::physx::PxU32 typeId = OmniPhysX::getInstance().getFreeTypeId();
    PX_COMPILE_TIME_ASSERT(sizeof(PxCustomGeometry::Type) == sizeof(::physx::PxU32));
    memcpy(typeMem, &typeId, sizeof(::physx::PxU32));
    PxCustomGeometry::Type* uniqueType = reinterpret_cast<PxCustomGeometry::Type*>(typeMem);
    std::shared_ptr<PxCustomGeometry::Type> retainedType(uniqueType,
        [](PxCustomGeometry::Type* type) { ICE_FREE(type); });
    CustomGeometryInfo info = { customGeometryAPIToken, geometryCallback, retainedType };
    mKnownTypes.erase(std::remove_if(mKnownTypes.begin(), mKnownTypes.end(),
        [](const std::weak_ptr<PxCustomGeometry::Type>& type) { return type.expired(); }), mKnownTypes.end());
    mKnownTypes.push_back(retainedType);
    try
    {
        mCustomGeometryRegistryMap.emplace(currentRegistryCounter, info);
        mCustomGeometryTypeMap.emplace(tokenHash, info);
        omni::physics::parse::registerCustomToken(omni::physics::parse::CustomTokenKind::eShape, customGeometryAPIToken);
    }
    catch (...)
    {
        mCustomGeometryTypeMap.erase(tokenHash);
        mCustomGeometryRegistryMap.erase(currentRegistryCounter);
        throw;
    }
    mGeometryRegistryCounter++;

    return currentRegistryCounter;
}

void PhysXCustomGeometryManager::unregisterCustomGeometry(size_t id)
{
    std::lock_guard<carb::tasking::MutexWrapper> lock(mRegistryMutex);
    CustomGeometryRegistryMap::const_iterator fit = mCustomGeometryRegistryMap.find(id);
    if (fit != mCustomGeometryRegistryMap.end())
    {
        const std::string& jt = fit->second.customGeomtryAPIToken;
        mCustomGeometryTypeMap.erase(computeCustomGeometryHash(jt));
        omni::physics::parse::unregisterCustomToken(omni::physics::parse::CustomTokenKind::eShape, jt);
        mCustomGeometryRegistryMap.erase(fit);
    }
}

bool PhysXCustomGeometryManager::tryUnregisterCustomGeometry(size_t id)
{
    if (getRuntimeCallbackDepth())
        return false;
    std::lock_guard<carb::tasking::MutexWrapper> lock(mRegistryMutex);
    const std::unordered_map<uint64_t, omni::physx::CustomGeometryInfo>::iterator found = mCustomGeometryRegistryMap.find(id);
    if (found == mCustomGeometryRegistryMap.end() || found->second.typeId.use_count() != 2)
        return false;
    const std::string& token = found->second.customGeomtryAPIToken;
    mCustomGeometryTypeMap.erase(computeCustomGeometryHash(token));
    omni::physics::parse::unregisterCustomToken(omni::physics::parse::CustomTokenKind::eShape, token);
    mCustomGeometryRegistryMap.erase(found);
    return true;
}

size_t registerCustomGeometry(const char* schemaAPIToken, ICustomGeometryCallback& geomCallback)
{
    return OmniPhysX::getInstance().getCustomGeometryManager().registerCustomGeometry(schemaAPIToken, geomCallback);
}

void unregisterCustomGeometry(size_t id)
{
    OmniPhysX::getInstance().getCustomGeometryManager().unregisterCustomGeometry(id);
}

bool tryUnregisterCustomGeometry(size_t id)
{
    return OmniPhysX::getInstance().getCustomGeometryManager().tryUnregisterCustomGeometry(id);
}

}
}

void fillInterface(omni::physx::IPhysxCustomGeometry& iface)
{
    iface.registerCustomGeometry = omni::physx::registerCustomGeometry;
    iface.unregisterCustomGeometry = omni::physx::unregisterCustomGeometry;
    iface.tryUnregisterCustomGeometry = omni::physx::tryUnregisterCustomGeometry;
}
