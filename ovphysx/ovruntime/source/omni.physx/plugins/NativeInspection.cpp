// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-NATIVE-INSPECTION-001
// @covers AC-1 AC-2 AC-3 AC-4
#include <omni/physx/NativeInspection.h>
#include <omni/physx/PhysXRuntime.h>
#include "NativeObjects.h"
#include "RuntimeExecution.h"
#include "usdLoad/LoadUsd.h"
#include "internal/InternalActor.h"
#include <PxPhysicsAPI.h>
#include <algorithm>
#include <memory>
namespace omni::physx::native
{
namespace
{
using namespace ::physx;
using Owner=usdparser::AttachedStage;
using Record=internal::InternalDatabase::Record;
uint32_t instanceIndex(const Record& record)
{
    if ((record.mType==ePTActor || record.mType==ePTLink) && record.mInternalPtr)
        return static_cast<const internal::InternalActor*>(record.mInternalPtr)->mInstanceIndex;
    return UINT32_MAX;
}
bool cpuScene(const PxScene* scene)
{
    return scene && !scene->getFlags().isSet(PxSceneFlag::eENABLE_GPU_DYNAMICS) &&
        !scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API);
}
void vector(float* result,const PxVec3& value)
{
    result[0]=value.x;result[1]=value.y;result[2]=value.z;
}
void pose(float* result,const PxTransform& value)
{
    vector(result,value.p);result[3]=value.q.x;result[4]=value.q.y;result[5]=value.q.z;result[6]=value.q.w;
}
template<class Fn> InspectionResult withOwner(uint64_t attachment,Fn fn) noexcept
try
{
    if (!attachment || !runtime::tryGetPhysxInterface()) return InspectionResult::Stale;
    std::shared_ptr<omni::physx::usdparser::AttachedStage> owner=usdparser::UsdLoad::getUsdLoad()->acquireAttachment(attachment);
    if (!owner) return InspectionResult::Stale;
    RuntimeExecutionScope admission(attachment);
    if (!admission) return InspectionResult::Busy;
    if (usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachment)!=owner.get()) return InspectionResult::Stale;
    return fn(*owner);
}
catch (...) {return InspectionResult::Error;}
template<class Fn> InspectionResult withObject(const ObjectReference& reference,ObjectKind kind,Fn fn) noexcept
{
    if (reference.kind!=uint32_t(kind) || !reference.object || !reference.key) return InspectionResult::Invalid;
    return withOwner(reference.attachment,[&](Owner& owner) {
        const omni::physx::internal::InternalDatabase::Record * record=objectRecord(owner,reference);
        if (!record) return InspectionResult::Stale;
        return fn(owner,*record);
    });
}
InspectionResult bodyState(const Owner& owner,const ObjectReference& reference,BodyInspection& output)
{
    const omni::physx::internal::InternalDatabase::Record * record=objectRecord(owner,reference);
    if (!record || !matchesObjectKind(record->mType,ObjectKind::Body)) return InspectionResult::Stale;
    ::physx::PxRigidActor * actor=static_cast<PxRigidActor*>(record->mPtr);
    if (!cpuScene(actor->getScene())) return InspectionResult::Unavailable;
    BodyInspection value;
    value.identity=reference;value.step=owner.simulationTimestamp();
    pose(value.pose,actor->getGlobalPose());pose(value.centerOfMass,PxTransform(PxIdentity));
    if (actor->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION)) value.flags|=8;
    if (::physx::PxRigidBody * body=actor->is<PxRigidBody>())
    {
        value.mass=body->getMass();vector(value.inertia,body->getMassSpaceInertiaTensor());
        pose(value.centerOfMass,body->getCMassLocalPose());
        vector(value.linearVelocity,body->getLinearVelocity());vector(value.angularVelocity,body->getAngularVelocity());
        if (body->getRigidBodyFlags().isSet(PxRigidBodyFlag::eENABLE_CCD)) value.flags|=4;
        if (::physx::PxRigidDynamic * dynamic=body->is<PxRigidDynamic>())
        {
            value.type=1;value.locks=uint32_t(dynamic->getRigidDynamicLockFlags());
            if (dynamic->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC)) value.flags|=1;
            if (dynamic->isSleeping()) value.flags|=2;
        }
        else if (::physx::PxArticulationLink * link=body->is<PxArticulationLink>())
        {
            value.type=2;if (link->getArticulation().isSleeping()) value.flags|=2;
        }
    }
    output=value;return InspectionResult::Success;
}
}

bool matchesObjectKind(PhysXType type,ObjectKind kind)
{
    switch(kind)
    {
    case ObjectKind::Body:return type==ePTActor || type==ePTLink;
    case ObjectKind::Joint:return type==ePTJoint || type==ePTLinkJoint;
    case ObjectKind::Material:return type==ePTMaterial;
    case ObjectKind::Articulation:return type==ePTArticulation;
    case ObjectKind::Shape:return type==ePTShape || type==ePTCompoundShape;
    }
    return false;
}
ObjectReference objectReference(const usdparser::AttachedStage& owner,uint64_t object)
{
    const std::vector<omni::physx::internal::InternalDatabase::Record> & records=owner.getInternalPhysXDatabase().getRecords();
    if (!object || object>=records.size() || !records[object].mPtr) return {};
    const omni::physx::internal::InternalDatabase::Record & record=records[object];
    for (uint32_t kind=1;kind<=5;++kind)
        if (matchesObjectKind(record.mType,ObjectKind(kind)))
            return {owner.getAttachHandle(),object,record.mKey.handle,record.mGeneration,kind,instanceIndex(record)};
    return {};
}
const internal::InternalDatabase::Record* objectRecord(const usdparser::AttachedStage& owner,const ObjectReference& reference)
{
    if (reference.attachment!=owner.getAttachHandle()) return nullptr;
    const omni::physx::native::ObjectReference actual=objectReference(owner,reference.object);
    if (!actual.object || actual.key!=reference.key || actual.generation!=reference.generation ||
        actual.kind!=reference.kind || actual.instance!=reference.instance) return nullptr;
    return &owner.getInternalPhysXDatabase().getRecords()[reference.object];
}
InspectionResult resolveObject(uint64_t attachment,const char* path,ObjectKind kind,uint32_t instance,ObjectReference& output) noexcept
{
    if (!path || path[0]!='/' || uint32_t(kind)<1 || uint32_t(kind)>5) return InspectionResult::Invalid;
    return withOwner(attachment,[&](Owner& owner) {
        const omni::physics::parse::IPhysicsSource * source=owner.getSource();
        if (!source) return InspectionResult::NotFound;
        const omni::physics::parse::ObjectKey key=source->findByPath(path);
        if (!key.valid()) return InspectionResult::NotFound;
        ObjectReference result;
        const std::vector<omni::physx::internal::InternalDatabase::Record> & records=owner.getInternalPhysXDatabase().getRecords();
        for (uint64_t i=1;i<records.size();++i)
        {
            const omni::physx::internal::InternalDatabase::Record & record=records[i];
            if (!record.mPtr || record.mKey!=key || !matchesObjectKind(record.mType,kind)) continue;
            const unsigned int index=instanceIndex(record);
            if (index!=instance) continue;
            if (result.object) return InspectionResult::Invalid;
            result=objectReference(owner,i);
        }
        if (!result.object) return InspectionResult::NotFound;
        output=result;return InspectionResult::Success;
    });
}
InspectionResult objectPath(const ObjectReference& reference,std::string& output) noexcept
{
    return withOwner(reference.attachment,[&](Owner& owner) {
        const omni::physx::internal::InternalDatabase::Record * record=objectRecord(owner,reference);
        if(!record)return InspectionResult::Stale;
        const char* path=owner.textFor(record->mKey);
        if(!path || !*path)return InspectionResult::NotFound;
        std::string value(path);output=std::move(value);return InspectionResult::Success;
    });
}
InspectionResult inspectBody(const ObjectReference& reference,BodyInspection& output) noexcept
{
    return withObject(reference,ObjectKind::Body,[&](Owner& owner,const Record&) {return bodyState(owner,reference,output);});
}
InspectionResult inspectJoint(const ObjectReference& reference,JointInspection& output) noexcept
{
    return withObject(reference,ObjectKind::Joint,[&](Owner& owner,const Record& record) {
        JointInspection value;value.identity=reference;value.step=owner.simulationTimestamp();
        PxRigidActor *a=nullptr,*b=nullptr;
        PxTransform local0(PxIdentity),local1(PxIdentity);
        if (record.mType==ePTLinkJoint)
        {
            ::physx::PxArticulationJointReducedCoordinate * joint=static_cast<PxArticulationJointReducedCoordinate*>(record.mPtr);
            a=&joint->getParentArticulationLink();b=&joint->getChildArticulationLink();
            if (!cpuScene(a->getScene())) return InspectionResult::Unavailable;
            local0=joint->getParentPose();local1=joint->getChildPose();value.type=7;
        }
        else
        {
            ::physx::PxJoint * joint=static_cast<PxJoint*>(record.mPtr);
            joint->getActors(a,b);
            if ((a && !cpuScene(a->getScene())) || (b && !cpuScene(b->getScene()))) return InspectionResult::Unavailable;
            local0=joint->getLocalPose(PxJointActorIndex::eACTOR0);local1=joint->getLocalPose(PxJointActorIndex::eACTOR1);
            value.flags=uint32_t(joint->getConstraintFlags());value.validFields=1;
            joint->getBreakForce(value.breakForce,value.breakTorque);
            if (joint->is<PxFixedJoint>()) value.type=1;
            else if (joint->is<PxRevoluteJoint>()) value.type=2;
            else if (joint->is<PxPrismaticJoint>()) value.type=3;
            else if (joint->is<PxSphericalJoint>()) value.type=4;
            else if (joint->is<PxDistanceJoint>()) value.type=5;
            else if (joint->is<PxD6Joint>()) value.type=6;
        }
        if (a) value.body0=objectReference(owner,uint64_t(a->userData));
        if (b) value.body1=objectReference(owner,uint64_t(b->userData));
        pose(value.local0,local0);pose(value.local1,local1);
        pose(value.world0,a ? a->getGlobalPose()*local0 : local0);
        pose(value.world1,b ? b->getGlobalPose()*local1 : local1);
        output=value;return InspectionResult::Success;
    });
}
InspectionResult inspectMaterial(const ObjectReference& reference,MaterialInspection& output) noexcept
{
    return withObject(reference,ObjectKind::Material,[&](Owner& owner,const Record& record) {
        const ::physx::PxMaterial * material=static_cast<const PxMaterial*>(record.mPtr);
        MaterialInspection value;value.identity=reference;value.step=owner.simulationTimestamp();
        value.staticFriction=material->getStaticFriction();value.dynamicFriction=material->getDynamicFriction();
        value.restitution=material->getRestitution();value.frictionCombine=uint32_t(material->getFrictionCombineMode());
        value.restitutionCombine=uint32_t(material->getRestitutionCombineMode());value.flags=uint32_t(material->getFlags());
        output=value;return InspectionResult::Success;
    });
}
InspectionResult inspectArticulation(const ObjectReference& reference,ArticulationInspection& output) noexcept
{
    return withObject(reference,ObjectKind::Articulation,[&](Owner& owner,const Record& record) {
        ::physx::PxArticulationReducedCoordinate * articulation=static_cast<PxArticulationReducedCoordinate*>(record.mPtr);
        if (!cpuScene(articulation->getScene())) return InspectionResult::Unavailable;
        const uint32_t count=articulation->getNbLinks(),dofs=articulation->getDofs();
        if (!count || count>65536 || dofs>65536) return InspectionResult::Unavailable;
        std::vector<PxArticulationLink*> links(count);
        if (articulation->getLinks(links.data(),count)!=count) return InspectionResult::Error;
        std::sort(links.begin(),links.end(),[](PxArticulationLink* a,PxArticulationLink* b){return a->getLinkIndex()<b->getLinkIndex();});
        struct CacheDeleter { void operator()(PxArticulationCache* value) const { if(value)value->release(); } };
        std::unique_ptr<PxArticulationCache,CacheDeleter> cache(articulation->createCache());
        if (!cache) return InspectionResult::Error;
        articulation->copyInternalStateToCache(*cache,PxArticulationCacheFlag::ePOSITION|PxArticulationCacheFlag::eVELOCITY|
            PxArticulationCacheFlag::eACCELERATION|PxArticulationCacheFlag::eFORCE);
        ArticulationInspection value;value.identity=reference;value.step=owner.simulationTimestamp();
        value.flags=uint32_t(articulation->getArticulationFlags());value.sleeping=articulation->isSleeping();
        pose(value.rootPose,articulation->getRootGlobalPose());vector(value.rootLinearVelocity,articulation->getRootLinearVelocity());
        vector(value.rootAngularVelocity,articulation->getRootAngularVelocity());
        value.links.reserve(count);value.dofs.reserve(dofs);
        uint32_t offset=0;
        for (uint32_t i=0;i<count;++i)
        {
            ::physx::PxArticulationLink * link=links[i];
            if (link->getLinkIndex()!=i) return InspectionResult::Error;
            BodyInspection body;
            const omni::physx::native::InspectionResult status=bodyState(owner,objectReference(owner,uint64_t(link->userData)),body);
            if (status!=InspectionResult::Success) return status;
            value.links.push_back(body);
            if (::physx::PxArticulationJointReducedCoordinate * joint=link->getInboundJoint())
            {
                uint32_t active=0;
                for (uint32_t axis=0;axis<6;++axis)
                    if (joint->getMotion(PxArticulationAxis::Enum(axis))!=PxArticulationMotion::eLOCKED)
                    {
                        if (offset>=dofs) return InspectionResult::Error;
                        value.dofs.push_back({body.identity,objectReference(owner,uint64_t(joint->userData)),offset,axis,
                            cache->jointPosition[offset],cache->jointVelocity[offset],cache->jointAcceleration[offset],cache->jointForce[offset]});
                        ++offset;++active;
                    }
                if (active!=link->getInboundJointDof()) return InspectionResult::Error;
            }
        }
        if (offset!=dofs) return InspectionResult::Error;
        output=std::move(value);return InspectionResult::Success;
    });
}
}
