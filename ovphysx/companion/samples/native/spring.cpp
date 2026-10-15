// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-JOINT-001
// @covers AC-1 AC-3
// @implements REQ-COMP-NATIVE-SCHEMA-001
// @covers AC-3
#include <ovphysx_companion/native_sdk.h>
#include <atomic>
#include <initializer_list>
#include <new>
#include <thread>
using namespace physx;
namespace sdk = ovphysx::companion::native;
namespace {
std::atomic<uint32_t> creates{0}, releases{0}, preparations{0}, starts{0}, stops{0}, refreshes{0}, attempts{0}, comShifts{0}, originShifts{0};
std::atomic<float> lastRefresh{0}, lastComX{0};
std::atomic<bool> holdPreparation{false}, preparationEntered{false};
struct alignas(16) Data {
    PxTransform frames[2];
    float stiffness, damping, restLength;
};
struct Joint {
    Data data;
    PxRigidActor* actors[2];
    PxTransform local[2];
};
void updateFrame(Joint& j, uint32_t index) noexcept {
    PxRigidActor * actor = j.actors[index];
    if (!actor) j.data.frames[index] = j.local[index];
    else if (actor->getType() == PxActorType::eRIGID_STATIC)
        j.data.frames[index] = actor->getGlobalPose() * j.local[index];
    else j.data.frames[index] = static_cast<PxRigidBody*>(actor)->getCMassLocalPose().getInverse() * j.local[index];
}
void* create(const sdk::ObjectContext& input, PxRigidActor* a, const PxTransform& fa,
             PxRigidActor* b, const PxTransform& fb, uint32_t& flags, void*) noexcept {
    ++attempts;
    if (input.parameterCount != 3) return nullptr;
    for (uint32_t i=0;i<3;++i) if (input.parameters[i].scalar < 0) return nullptr;
    // This sample implements ordinary rigid constraints, not articulation DOFs.
    for (physx::PxRigidActor * actor : {a,b})
        if (actor && actor->getType() == PxActorType::eARTICULATION_LINK) return nullptr;
    Joint * j = new(std::nothrow) Joint;
    if (!j) return nullptr;
    j->actors[0]=a; j->actors[1]=b; j->local[0]=fa; j->local[1]=fb;
    updateFrame(*j,0); updateFrame(*j,1);
    j->data.stiffness=input.parameters[0].scalar;
    j->data.damping=input.parameters[1].scalar;
    j->data.restLength=input.parameters[2].scalar;
    flags=0;
#ifdef COMPANION_TEST_BAD_FLAGS
    flags=2;
#endif
    ++creates;
    return j;
}
void release(void* object, void*) noexcept { ++releases; delete static_cast<Joint*>(object); }
void* prepare(void* object) noexcept {
    ++preparations;
    preparationEntered.store(true, std::memory_order_release);
    // Sample-only controllable callback used to verify retirement while code is
    // executing. Production callbacks must not block or wait on the owner.
    while (holdPreparation.load(std::memory_order_acquire)) std::this_thread::yield();
    return &static_cast<Joint*>(object)->data;
}
const void* block(void* object) noexcept {
#ifdef COMPANION_TEST_BAD_BLOCK
    return reinterpret_cast<const char*>(&static_cast<Joint*>(object)->data)+1;
#else
    return &static_cast<Joint*>(object)->data;
#endif
}
void comShift(void* object,uint32_t actor) noexcept {
    ++comShifts;
    if(actor<2) { Joint & j=*static_cast<Joint*>(object);updateFrame(j,actor);lastComX=j.data.frames[actor].p.x; }
}
void originShift(void* object,const PxVec3& shift) noexcept {
    ++originShifts;
    Joint & j=*static_cast<Joint*>(object);
    for(uint32_t i=0;i<2;++i) {
        if (!j.actors[i]) { j.local[i].p-=shift; j.data.frames[i].p-=shift; }
        else if(j.actors[i]->getType()==PxActorType::eRIGID_STATIC) j.data.frames[i].p-=shift;
    }
}
PxU32 solver(Px1DConstraint* rows,PxVec3p& offset,PxU32 maxRows,PxConstraintInvMassScale& mass,
             const void* block,const PxTransform& a,const PxTransform& b,bool,PxVec3p& ca,PxVec3p& cb) {
    if (!maxRows) return 0;
    const Data & data=*static_cast<const Data*>(block);
    const PxVec3 p0=(a*data.frames[0]).p, p1=(b*data.frames[1]).p;
    PxVec3 direction=p0-p1;
    const float length=direction.normalize();
    if(length<1e-6f) direction=PxVec3(1,0,0);
    ca=p0;cb=p1;offset=p1-a.p;
    mass={1,1,1,1};
    Px1DConstraint & row=rows[0];row={};
    row.linear0=row.linear1=direction;
    row.angular0=(p0-a.p).cross(direction);row.angular1=(p1-b.p).cross(direction);
    row.geometricError=length-data.restLength;
    row.minImpulse=-PX_MAX_F32;row.maxImpulse=PX_MAX_F32;
    row.flags=Px1DConstraintFlag::eSPRING|Px1DConstraintFlag::eOUTPUT_FORCE;
    row.mods.spring.stiffness=data.stiffness;row.mods.spring.damping=data.damping;
    return 1;
}
sdk::ParameterSpec parameter(const char* name,float value) {
    sdk::ParameterSpec result{};result.name=name;result.kind=result.fallback.kind=sdk::ParameterKind::Float;
    result.fallback.scalar=value;return result;
}
#ifdef COMPANION_TEST_BAD_OUTPUT_PARAM
const sdk::ParameterSpec specs[]={parameter("companion:output:stiffness",50),parameter("companion:damping",2),parameter("companion:restLength",1)};
#else
const sdk::ParameterSpec specs[]={parameter("companion:stiffness",50),parameter("companion:damping",2),parameter("companion:restLength",1)};
#endif
sdk::Plugin descriptor() {
    sdk::Plugin p;p.schema="CompanionSpringJoint";p.parameters=specs;p.parameterCount=3;
    p.start=[](const sdk::HostApi&) noexcept -> void* {++starts;return &starts;};
    p.stop=[](void*) noexcept {++stops;};
    p.joint={create,release,prepare,block,comShift,originShift,solver,sizeof(Data),alignof(Data)};
    p.joint.refresh=[](const sdk::ObjectContext& input,void* object,void*) noexcept {
        if(input.parameterCount!=3) return false;
        for(uint32_t i=0;i<3;++i) if(input.parameters[i].scalar<0) return false;
        Data & data=static_cast<Joint*>(object)->data;
        data.stiffness=input.parameters[0].scalar;data.damping=input.parameters[1].scalar;
        data.restLength=input.parameters[2].scalar;lastRefresh=data.stiffness;++refreshes;return true;
    };
#ifdef COMPANION_TEST_BAD_VERSION
    ++p.version;
#endif
#ifdef COMPANION_TEST_BAD_BUILD
    p.compatibility.physxBuild[0]='?';
#endif
#ifdef COMPANION_TEST_BAD_COMPILER
    p.compatibility.compiler[0]='?';
#endif
#ifdef COMPANION_TEST_BAD_ALIGNMENT
    p.joint.dataAlignment=3;
#endif
#ifdef COMPANION_TEST_BAD_CALLBACK
    p.joint.release=nullptr;
#endif
    return p;
}
}
OVPHYSX_COMPANION_PLUGIN_EXPORT const sdk::Plugin ovphysx_companion_plugin_v1=descriptor();
// Sample diagnostics let integration tests prove behavior and balanced lifetime.
OVPHYSX_COMPANION_PLUGIN_EXPORT uint32_t companion_sample_count(uint32_t which) {
    switch(which){case 0:return creates;case 1:return releases;case 2:return preparations;case 3:return starts;case 4:return stops;case 5:return refreshes;case 6:return attempts;case 7:return comShifts;case 8:return originShifts;default:return 0;}
}

OVPHYSX_COMPANION_PLUGIN_EXPORT float companion_sample_last_refresh() { return lastRefresh; }

OVPHYSX_COMPANION_PLUGIN_EXPORT float companion_sample_last_com_x() { return lastComX; }

OVPHYSX_COMPANION_PLUGIN_EXPORT void companion_sample_hold_prepare(bool hold) {
    if (hold) preparationEntered.store(false, std::memory_order_release);
    holdPreparation.store(hold, std::memory_order_release);
}
OVPHYSX_COMPANION_PLUGIN_EXPORT bool companion_sample_prepare_entered() {
    return preparationEntered.load(std::memory_order_acquire);
}
