// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-QUERY-001
// @covers AC-2 AC-3 AC-4
// @implements REQ-CAPI-COMPANION-001
// @covers AC-1
#include "ovphysx_companion/inspection.h"
#include "internal/sdk/CompanionRuntime.hpp"
#include <omni/physx/NativeInspection.h>
#include <algorithm>
#include <cstring>
namespace
{
using namespace omni::physx::native;
ovphysx_companion_object_t convert(const ObjectReference& value) noexcept
{
    return {value.attachment,value.object,value.key,value.generation,value.kind,value.instance};
}
ObjectReference convert(const ovphysx_companion_object_t& value) noexcept
{
    return {value.attachment,value.object,value.key,value.generation,value.kind,value.instance};
}
ovphysx_companion_body_t convert(const BodyInspection& value) noexcept
{
    ovphysx_companion_body_t output{};
    output.identity=convert(value.identity);
    output.step=value.step;
    output.type=value.type;
    output.flags=value.flags;
    output.locks=value.locks;
    output.reserved=value.reserved;
    output.mass=value.mass;
    std::copy_n(value.pose,7,output.pose);
    std::copy_n(value.centerOfMass,7,output.centerOfMass);
    std::copy_n(value.inertia,3,output.inertia);
    std::copy_n(value.linearVelocity,3,output.linearVelocity);
    std::copy_n(value.angularVelocity,3,output.angularVelocity);
    return output;
}
ovphysx_companion_joint_t convert(const JointInspection& value) noexcept
{
    ovphysx_companion_joint_t output{};
    output.identity=convert(value.identity);
    output.body0=convert(value.body0);
    output.body1=convert(value.body1);
    output.step=value.step;
    output.type=value.type;
    output.flags=value.flags;
    output.validFields=value.validFields;
    output.reserved=value.reserved;
    output.breakForce=value.breakForce;
    output.breakTorque=value.breakTorque;
    std::copy_n(value.local0,7,output.local0);
    std::copy_n(value.local1,7,output.local1);
    std::copy_n(value.world0,7,output.world0);
    std::copy_n(value.world1,7,output.world1);
    return output;
}
ovphysx_companion_material_t convert(const MaterialInspection& value) noexcept
{
    ovphysx_companion_material_t output{};
    output.identity=convert(value.identity);
    output.step=value.step;
    output.staticFriction=value.staticFriction;
    output.dynamicFriction=value.dynamicFriction;
    output.restitution=value.restitution;
    output.frictionCombine=value.frictionCombine;
    output.restitutionCombine=value.restitutionCombine;
    output.flags=value.flags;
    return output;
}
ovphysx_companion_dof_t convert(const ArticulationDof& value) noexcept
{
    ovphysx_companion_dof_t output{};
    output.link=convert(value.link);
    output.joint=convert(value.joint);
    output.index=value.index;
    output.axis=value.axis;
    output.position=value.position;
    output.velocity=value.velocity;
    output.acceleration=value.acceleration;
    output.force=value.force;
    return output;
}
ovphysx_companion_status_t status(InspectionResult value)
{
    switch(value) {
    case InspectionResult::Success:return OVPHYSX_COMPANION_OK;
    case InspectionResult::Invalid:return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    case InspectionResult::Stale:return OVPHYSX_COMPANION_STALE;
    case InspectionResult::Busy:return OVPHYSX_COMPANION_BUSY;
    case InspectionResult::NotFound:return OVPHYSX_COMPANION_NOT_FOUND;
    case InspectionResult::Unavailable:return OVPHYSX_COMPANION_UNAVAILABLE;
    default:return OVPHYSX_COMPANION_ERROR;
    }
}
ovphysx_companion_status_t resolve(uint64_t attach,const char* path,uint32_t kind,uint32_t instance,
    uint32_t size,ovphysx_companion_object_t* output) noexcept
{
    if (!output || size!=sizeof(*output)) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    CompanionOperationGuard guard(companionRuntime());if(!guard)return OVPHYSX_COMPANION_BUSY;
    ObjectReference value;
    const ovphysx_companion_status_t result=status(resolveObject(attach,path,ObjectKind(kind),instance,value));
    if (result==OVPHYSX_COMPANION_OK) *output=convert(value);
    return result;
}
template<class Public,class Native> ovphysx_companion_status_t readValue(const ObjectReference& ref,void* output,
    uint32_t size,InspectionResult(*fn)(const ObjectReference&,Native&) noexcept)
{
    if (size!=sizeof(Public)) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    Native value;const ovphysx_companion_status_t result=status(fn(ref,value));
    if (result==OVPHYSX_COMPANION_OK) *static_cast<Public*>(output)=convert(value);
    return result;
}
ovphysx_companion_status_t read(const ovphysx_companion_object_t* reference,uint32_t refSize,
    void* output,uint32_t size) noexcept
{
    if (!reference || refSize!=sizeof(*reference) || !output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    CompanionOperationGuard guard(companionRuntime());if(!guard)return OVPHYSX_COMPANION_BUSY;
    omni::physx::native::ObjectReference const ref=convert(*reference);
    switch(ObjectKind(ref.kind)) {
    case ObjectKind::Body: static_assert(sizeof(BodyInspection)==sizeof(ovphysx_companion_body_t));
        return readValue<ovphysx_companion_body_t>(ref,output,size,inspectBody);
    case ObjectKind::Joint:static_assert(sizeof(JointInspection)==sizeof(ovphysx_companion_joint_t));
        return readValue<ovphysx_companion_joint_t>(ref,output,size,inspectJoint);
    case ObjectKind::Material:static_assert(sizeof(MaterialInspection)==sizeof(ovphysx_companion_material_t));
        return readValue<ovphysx_companion_material_t>(ref,output,size,inspectMaterial);
    default:return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    }
}
ovphysx_companion_status_t articulation(const ovphysx_companion_object_t* ref,uint32_t refSize,
    ovphysx_companion_articulation_t* output,uint32_t size,ovphysx_companion_body_t* links,
    uint32_t linkCapacity,uint32_t* requiredLinks,ovphysx_companion_dof_t* dofs,
    uint32_t dofCapacity,uint32_t* requiredDofs) noexcept
{
    if (!ref || refSize!=sizeof(*ref) || !output || size!=sizeof(*output) || !requiredLinks || !requiredDofs ||
        (linkCapacity && !links) || (dofCapacity && !dofs)) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    CompanionOperationGuard guard(companionRuntime());if(!guard)return OVPHYSX_COMPANION_BUSY;
    ArticulationInspection value;
    const ovphysx_companion_status_t result=status(inspectArticulation(convert(*ref),value));
    if(result!=OVPHYSX_COMPANION_OK)return result;
    if (linkCapacity<value.links.size() || dofCapacity<value.dofs.size()) {
        *requiredLinks=uint32_t(value.links.size());*requiredDofs=uint32_t(value.dofs.size());
        return OVPHYSX_COMPANION_INSPECTION_CAPACITY;
    }
    ovphysx_companion_articulation_t summary{};
    summary.identity=convert(value.identity);summary.step=value.step;
    summary.flags=value.flags;summary.sleeping=value.sleeping;
    std::copy_n(value.rootPose,7,summary.rootPose);std::copy_n(value.rootLinearVelocity,3,summary.rootLinearVelocity);
    std::copy_n(value.rootAngularVelocity,3,summary.rootAngularVelocity);
    for(size_t i=0;i<value.links.size();++i)links[i]=convert(value.links[i]);
    for(size_t i=0;i<value.dofs.size();++i)dofs[i]=convert(value.dofs[i]);
    *output=summary;*requiredLinks=uint32_t(value.links.size());*requiredDofs=uint32_t(value.dofs.size());
    return OVPHYSX_COMPANION_OK;
}
ovphysx_companion_status_t path(const ovphysx_companion_object_t* ref,uint32_t refSize,
    char* output,uint32_t capacity,uint32_t* required) noexcept
{
    if(!ref || refSize!=sizeof(*ref) || !required || (capacity && !output))return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    CompanionOperationGuard guard(companionRuntime());if(!guard)return OVPHYSX_COMPANION_BUSY;
    std::string value;const ovphysx_companion_status_t result=status(objectPath(convert(*ref),value));
    if(result!=OVPHYSX_COMPANION_OK)return result;
    if(value.size()>=UINT32_MAX)return OVPHYSX_COMPANION_ERROR;
    if(capacity<=value.size()) {*required=uint32_t(value.size()+1);return OVPHYSX_COMPANION_INSPECTION_CAPACITY;}
    std::memcpy(output,value.c_str(),value.size()+1);*required=uint32_t(value.size()+1);return OVPHYSX_COMPANION_OK;
}

}
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_inspection_acquire(
    uint32_t version,uint32_t size,ovphysx_companion_inspection_api_t* output)
{
    if (!output)return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if(version!=1 || size!=sizeof(*output))return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output={sizeof(*output),1,15,resolve,read,articulation,path};return OVPHYSX_COMPANION_OK;
}
