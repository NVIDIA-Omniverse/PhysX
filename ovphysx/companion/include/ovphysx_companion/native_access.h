// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-INTERACTION-001
// @covers AC-1 AC-4
#pragma once
// Optional build-matched C++17 SDK surface. This is deliberately not a Python or
// stable C ABI object-pointer API. Do not link a second PhysX runtime into clients.
#include "inspection.h"
#include "native_compatibility.h"
namespace ovphysx::companion::native
{
enum class AccessKind:uint32_t {RigidActor=1,Dynamic=2,Static=3,Link=4,Joint=5,Articulation=6,Shape=7,Material=8};
// Trusted C++ adapters only; ordinary widgets must use copied inspection data.
// Constness/noncopyability cannot enforce pointer non-retention or child mutation;
// callers are responsible for obeying those restrictions.
// A synchronous, read-only SDK borrow. No pointer/reference obtained here may
// outlive this call. No mutation, object release, physics wait, nested API or
// interpreter entry is permitted. Even const SDK methods can return mutable child
// pointers; those remain read-only under this contract. The host owns scheduling.
class ObjectLease
{
public:
    ObjectLease(const ObjectLease&)=delete;
    ObjectLease& operator=(const ObjectLease&)=delete;
    const ovphysx_companion_object_t& identity() const noexcept{return mIdentity;}
    AccessKind kind() const noexcept{return mKind;}
    const physx::PxRigidActor* actor() const noexcept
    {
        switch(mKind) {
        case AccessKind::RigidActor:return static_cast<const physx::PxRigidActor*>(mObject);
        case AccessKind::Dynamic:return static_cast<const physx::PxRigidDynamic*>(mObject);
        case AccessKind::Static:return static_cast<const physx::PxRigidStatic*>(mObject);
        case AccessKind::Link:return static_cast<const physx::PxArticulationLink*>(mObject);
        default:return nullptr;
        }
    }
    const physx::PxRigidDynamic* dynamic() const noexcept{return mKind==AccessKind::Dynamic?static_cast<const physx::PxRigidDynamic*>(mObject):nullptr;}
    const physx::PxRigidStatic* rigidStatic() const noexcept{return mKind==AccessKind::Static?static_cast<const physx::PxRigidStatic*>(mObject):nullptr;}
    const physx::PxArticulationLink* link() const noexcept{return mKind==AccessKind::Link?static_cast<const physx::PxArticulationLink*>(mObject):nullptr;}
    const physx::PxJoint* joint() const noexcept{return mKind==AccessKind::Joint?static_cast<const physx::PxJoint*>(mObject):nullptr;}
    const physx::PxArticulationReducedCoordinate* articulation() const noexcept{return mKind==AccessKind::Articulation?static_cast<const physx::PxArticulationReducedCoordinate*>(mObject):nullptr;}
    const physx::PxShape* shape() const noexcept{return mKind==AccessKind::Shape?static_cast<const physx::PxShape*>(mObject):nullptr;}
    const physx::PxMaterial* material() const noexcept{return mKind==AccessKind::Material?static_cast<const physx::PxMaterial*>(mObject):nullptr;}
private:
    friend struct AccessLeaseFactory;
    ObjectLease(const ovphysx_companion_object_t& identity,AccessKind kind,const void* object):mIdentity(identity),mKind(kind),mObject(object){}
    ovphysx_companion_object_t mIdentity;
    AccessKind mKind;
    const void* mObject;
};
using AccessVisitor=bool(*)(const ObjectLease&,void*) noexcept;
struct NativeAccessApi
{
    uint32_t size,version;
    uint64_t capabilities; // bit0 read-only CPU object leases (shape/material are host metadata)
    ovphysx_companion_status_t (*visit)(const ovphysx_companion_object_t*,uint32_t referenceSize,
        AccessKind,AccessVisitor,void* context);
};
// Negotiate before use. Retain an existing companion owner through every visit.
// The table borrows the loaded library; it creates no owner or background work.
}
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_native_access_acquire(
    const ovphysx::companion::native::Compatibility* compatibility,uint32_t compatibilitySize,
    uint32_t tableSize,ovphysx::companion::native::NativeAccessApi* output);
