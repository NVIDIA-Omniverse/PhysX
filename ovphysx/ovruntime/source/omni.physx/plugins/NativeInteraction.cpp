// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-NATIVE-INTERACTION-001
// @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6
#include <omni/physx/NativeInteraction.h>
#include <omni/physx/PhysXRuntime.h>
#include "NativeObjects.h"
#include "NativeInteractionStep.h"
#include "RuntimeExecution.h"
#include "usdLoad/LoadUsd.h"
#include "usdInterface/UsdInterface.h"
#include <PxPhysicsAPI.h>
#include <map>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <limits>
namespace omni::physx::native
{
using namespace ::physx;
using Owner=usdparser::AttachedStage;
class Drag
{
public:
    ObjectReference body;
    std::weak_ptr<Owner> owner;
    std::mutex mutex;
    PxD6Joint* joint=nullptr;
    SubscriptionId observer=kInvalidSubscriptionId;
    PxRigidDynamicLockFlags savedLocks;
    bool savedCcd=false,closed=false;
    uint32_t flags=0,velocityPolicy=0;
    PxVec3 linear,angular,target,localPoint;
    bool forceGrab=false,forceStarted=false;
    float pickingForce=1,pickDistance=0,savedAngularDamping=0;
};
namespace
{
bool finite(const float* p){return p && std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);}
PxVec3 vec(const float* p){return PxVec3(p[0],p[1],p[2]);}
bool cpu(const PxScene* scene)
{return scene && !(scene->getFlags()&(PxSceneFlag::eENABLE_GPU_DYNAMICS|PxSceneFlag::eENABLE_DIRECT_GPU_API));}
PxRigidDynamic* dynamic(const Owner& owner,const ObjectReference& ref)
{
    const omni::physx::internal::InternalDatabase::Record * record=objectRecord(owner,ref);
    if(!record || record->mType!=ePTActor)return nullptr;
    ::physx::PxRigidDynamic * body=static_cast<PxRigidActor*>(record->mPtr)->is<PxRigidDynamic>();
    if(!body || !cpu(body->getScene()) || body->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC) ||
       body->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION))return nullptr;
    return body;
}
PxRigidBody* forceBody(const Owner& owner,const ObjectReference& ref)
{
    const internal::InternalDatabase::Record* record=objectRecord(owner,ref);
    if(!record || !matchesObjectKind(record->mType,ObjectKind::Body))return nullptr;
    PxRigidBody* body=static_cast<PxRigidActor*>(record->mPtr)->is<PxRigidBody>();
    if(!body || !cpu(body->getScene()) || body->getRigidBodyFlags().isSet(PxRigidBodyFlag::eKINEMATIC) ||
       body->getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION))return nullptr;
    return body;
}
bool rayTarget(const float* origin,const float* direction,float distance,PxVec3& target)
{
    if(!finite(origin) || !finite(direction))return false;
    const double length=std::sqrt(double(direction[0])*direction[0]+double(direction[1])*direction[1]+double(direction[2])*direction[2]);
    if(length==0)return false;
    const PxVec3 normalized(float(direction[0]/length),float(direction[1]/length),float(direction[2]/length));
    target=vec(origin)+normalized*distance;
    return target.isFinite();
}
template<class Fn> InspectionResult withObject(const ObjectReference& ref,Fn fn) noexcept
try
{
    if(!runtime::tryGetPhysxInterface())return InspectionResult::Stale;
    std::shared_ptr<omni::physx::usdparser::AttachedStage> owner=usdparser::UsdLoad::getUsdLoad()->acquireAttachment(ref.attachment);
    if(!owner)return InspectionResult::Stale;
    RuntimeExecutionScope admission(ref.attachment);if(!admission)return InspectionResult::Busy;
    if(usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(ref.attachment)!=owner.get())return InspectionResult::Stale;
    const omni::physx::internal::InternalDatabase::Record * record=objectRecord(*owner,ref);if(!record)return InspectionResult::Stale;
    return fn(owner,*record);
}
catch(...){return InspectionResult::Error;}
// Caller holds native owner admission or its synchronous pre-destruction callback.
void cleanup(Drag& drag,bool restore) noexcept
{
    if(drag.closed)return;
    if(drag.joint){drag.joint->release();drag.joint=nullptr;}
    if(std::shared_ptr<omni::physx::usdparser::AttachedStage> owner=drag.owner.lock()) {
        const internal::InternalDatabase::Record* record=restore?objectRecord(*owner,drag.body):nullptr;
        PxRigidActor* actor=record && matchesObjectKind(record->mType,ObjectKind::Body)?static_cast<PxRigidActor*>(record->mPtr):nullptr;
        PxRigidBody* grabbed=actor?actor->is<PxRigidBody>():nullptr;
        if(grabbed && drag.forceStarted)grabbed->setAngularDamping(drag.savedAngularDamping);
        PxRigidDynamic* body=actor?actor->is<PxRigidDynamic>():nullptr;
        if(body) {
            if(drag.flags&1)body->setRigidDynamicLockFlags(drag.savedLocks);
            if(drag.flags&2)body->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD,drag.savedCcd);
            if(dynamic(*owner,drag.body) && (!drag.forceGrab || drag.forceStarted)) {
                if(drag.forceStarted)body->wakeUp();
                if(drag.velocityPolicy==1) {body->setLinearVelocity(PxVec3(0));body->setAngularVelocity(PxVec3(0));}
                else if(drag.velocityPolicy==2) {body->setLinearVelocity(drag.linear);body->setAngularVelocity(drag.angular);}
            }
        }
        if(drag.observer!=kInvalidSubscriptionId)
            owner->getPhysXPhysicsInterface()->unsubscribeToObjectChangeNotifications(drag.observer);
    }
    drag.observer=kInvalidSubscriptionId;drag.closed=true;
    if (std::shared_ptr<Owner> owner = drag.owner.lock())
    {
        const std::unordered_map<uint64_t, std::weak_ptr<Drag>>::iterator found = owner->interactionDrags.find(drag.body.object);
        if (found != owner->interactionDrags.end() && found->second.lock().get() == &drag)
            owner->interactionDrags.erase(found);
    }
}
bool observeDrag(const std::shared_ptr<Owner>& owner,const std::shared_ptr<Drag>& drag)
{
    IPhysicsObjectChangeCallback callback;callback.stopCallbackWhenSimStopped=false;callback.notifyInitialPopulation=false;
    callback.objectDestructionNotifyFn=[drag](const PhysicsObjectChangeEvent& event,void*) {
        if(uint64_t(event.objectId)==drag->body.object){std::lock_guard<std::mutex> lock(drag->mutex);cleanup(*drag,false);}
    };
    callback.allObjectsDestructionNotifyFn=[drag](AttachHandle,void*){std::lock_guard<std::mutex> lock(drag->mutex);cleanup(*drag,false);};
    drag->observer=owner->getPhysXPhysicsInterface()->subscribeToObjectChangeNotifications(callback,drag->body.attachment);
    return drag->observer!=kInvalidSubscriptionId;
}
DragState state(const Drag& drag)
{return {drag.body,{drag.target.x,drag.target.y,drag.target.z},uint32_t(!drag.closed)};}
template<class Fn> InspectionResult withDrag(const std::shared_ptr<Drag>& drag,Fn fn) noexcept
{
    if(!drag)return InspectionResult::Invalid;
    return withObject(drag->body,[&](const std::shared_ptr<Owner>&,const internal::InternalDatabase::Record& ) {
        std::lock_guard<std::mutex> lock(drag->mutex);
        if(drag->closed)return InspectionResult::Stale;
        return fn(*drag);
    });
}
}
InspectionResult visitNativeObject(const ObjectReference& ref,LeaseKind kind,NativeVisitor callback,void* context) noexcept
{
    if(!callback || uint32_t(kind)<1 || uint32_t(kind)>8)return InspectionResult::Invalid;
    return withObject(ref,[&](const std::shared_ptr<Owner>&,const internal::InternalDatabase::Record& record) {
        const void* pointer=nullptr;
        if(matchesObjectKind(record.mType,ObjectKind::Body)) {
            ::physx::PxRigidActor * actor=static_cast<PxRigidActor*>(record.mPtr);
            if(!cpu(actor->getScene()))return InspectionResult::Unavailable;
            if(kind==LeaseKind::RigidActor)pointer=actor;
            else if(kind==LeaseKind::Dynamic)pointer=actor->is<PxRigidDynamic>();
            else if(kind==LeaseKind::Static)pointer=actor->is<PxRigidStatic>();
            else if(kind==LeaseKind::Link)pointer=actor->is<PxArticulationLink>();
        }
        else if(kind==LeaseKind::Joint && record.mType==ePTJoint) {
            ::physx::PxJoint * joint=static_cast<PxJoint*>(record.mPtr);
            if(!cpu(joint->getScene()))return InspectionResult::Unavailable;
            pointer=joint;
        }
        else if(kind==LeaseKind::Articulation && record.mType==ePTArticulation) {
            ::physx::PxArticulationReducedCoordinate * articulation=static_cast<PxArticulationReducedCoordinate*>(record.mPtr);
            if(!cpu(articulation->getScene()))return InspectionResult::Unavailable;
            pointer=articulation;
        }
        else if(kind==LeaseKind::Shape && record.mType==ePTShape)pointer=record.mPtr;
        else if(kind==LeaseKind::Material && record.mType==ePTMaterial)pointer=record.mPtr;
        if(!pointer)return InspectionResult::Invalid;
        RuntimeCallbackScope scope(ref.attachment);
        return callback(kind,pointer,context)?InspectionResult::Success:InspectionResult::Error;
    });
}
InspectionResult applyBodyForce(const ObjectReference& ref,const BodyForce& command) noexcept
{
    if(ref.kind!=uint32_t(ObjectKind::Body) || command.mode>1 || command.frame>1 || command.atPoint>1 ||
        command.wake>1 || !finite(command.force) || !finite(command.point))return InspectionResult::Invalid;
    return withObject(ref,[&](const std::shared_ptr<Owner>& owner,const internal::InternalDatabase::Record& ) {
        ::physx::PxRigidDynamic * body=dynamic(*owner,ref);if(!body)return InspectionResult::Unavailable;
        const PxTransform pose=body->getGlobalPose();
        const PxVec3 force=command.frame?pose.q.rotate(vec(command.force)):vec(command.force);
        const ::physx::PxForceMode::Enum mode=command.mode?PxForceMode::eIMPULSE:PxForceMode::eFORCE;
        if(command.atPoint)PxRigidBodyExt::addForceAtPos(*body,force,
            command.frame?pose.transform(vec(command.point)):vec(command.point),mode,command.wake!=0);
        else body->addForce(force,mode,command.wake!=0);
        return InspectionResult::Success;
    });
}
InspectionResult beginDrag(const ObjectReference& ref,const DragOptions& options,std::shared_ptr<Drag>& output) noexcept
{
    if(ref.kind!=uint32_t(ObjectKind::Body) || !finite(options.target) || !std::isfinite(options.stiffness) || options.stiffness<=0 ||
       !std::isfinite(options.damping) || options.damping<0 || !std::isfinite(options.maxForce) || options.maxForce<=0 ||
       options.flags&~3u || options.velocityPolicy>2)return InspectionResult::Invalid;
    return withObject(ref,[&](const std::shared_ptr<Owner>& owner,const internal::InternalDatabase::Record& ) {
        ::physx::PxRigidDynamic * body=dynamic(*owner,ref);if(!body)return InspectionResult::Unavailable;
        const std::unordered_map<uint64_t, std::weak_ptr<Drag>>::iterator found = owner->interactionDrags.find(ref.object);
        if (found != owner->interactionDrags.end())
            if (std::shared_ptr<Drag> existing = found->second.lock())
                if (!existing->closed) return InspectionResult::Busy;
        std::shared_ptr<omni::physx::native::Drag> drag=std::make_shared<Drag>();drag->body=ref;drag->owner=owner;drag->flags=options.flags;
        drag->velocityPolicy=options.velocityPolicy;drag->target=vec(options.target);
        drag->savedLocks=body->getRigidDynamicLockFlags();drag->savedCcd=body->getRigidBodyFlags().isSet(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD);
        drag->linear=body->getLinearVelocity();drag->angular=body->getAngularVelocity();
        struct Rollback {std::shared_ptr<Drag> value;bool committed=false;~Rollback(){if(!committed)cleanup(*value,true);}} rollback{drag};
        drag->joint=PxD6JointCreate(body->getScene()->getPhysics(),nullptr,PxTransform(drag->target),body,
            PxTransform(body->getGlobalPose().transformInv(drag->target)));
        if(!drag->joint)return InspectionResult::Error;
        for(uint32_t axis=0;axis<6;++axis)drag->joint->setMotion(PxD6Axis::Enum(axis),PxD6Motion::eFREE);
        const PxD6JointDrive drive(options.stiffness,options.damping,options.maxForce,false);
        for(uint32_t axis=0;axis<3;++axis)drag->joint->setDrive(PxD6Drive::Enum(axis),drive);
        drag->joint->setConstraintFlag(PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES,true);
        if(!observeDrag(owner,drag))return InspectionResult::Error;
        if(options.flags&1)body->setRigidDynamicLockFlags(PxRigidDynamicLockFlags());
        if(options.flags&2)body->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD,true);
        owner->interactionDrags[ref.object]=drag;
        body->wakeUp();
        output=std::move(drag);rollback.committed=true;return InspectionResult::Success;
    });
}
InspectionResult beginForceDrag(const ObjectReference& ref,const ForceDragOptions& options,std::shared_ptr<Drag>& output) noexcept
{
    if(ref.kind!=uint32_t(ObjectKind::Body) || !finite(options.point) || !std::isfinite(options.pickingForce) ||
       options.pickingForce<0 || options.pickingForce>std::numeric_limits<float>::max()/50 ||
       !std::isfinite(options.pickDistance) || options.pickDistance<0 || options.pickDistance>1000000 ||
       options.velocityPolicy>2)return InspectionResult::Invalid;
    return withObject(ref,[&](const std::shared_ptr<Owner>& owner,const internal::InternalDatabase::Record&) {
        PxRigidBody* body=forceBody(*owner,ref);if(!body)return InspectionResult::Unavailable;
        PxRigidDynamic* rigid=body->is<PxRigidDynamic>();
        if(!rigid && options.velocityPolicy!=0)return InspectionResult::Unavailable;
        const std::unordered_map<uint64_t,std::weak_ptr<Drag>>::iterator found=owner->interactionDrags.find(ref.object);
        if(found!=owner->interactionDrags.end())
            if(std::shared_ptr<Drag> existing=found->second.lock())
                if(!existing->closed)return InspectionResult::Busy;
        std::shared_ptr<Drag> drag=std::make_shared<Drag>();drag->body=ref;drag->owner=owner;drag->forceGrab=true;
        drag->target=vec(options.point);drag->localPoint=body->getGlobalPose().transformInv(drag->target);
        drag->pickingForce=options.pickingForce;drag->pickDistance=options.pickDistance;drag->velocityPolicy=options.velocityPolicy;
        drag->savedAngularDamping=body->getAngularDamping();drag->linear=body->getLinearVelocity();drag->angular=body->getAngularVelocity();
        struct Rollback{std::shared_ptr<Drag> value;bool committed=false;~Rollback(){if(!committed)cleanup(*value,true);}} rollback{drag};
        if(!observeDrag(owner,drag))return InspectionResult::Error;
        // Allocate the registry node before changing any actor property.
        owner->interactionDrags[ref.object]=drag;
        body->setAngularDamping(std::max(drag->savedAngularDamping,0.025f*50*options.pickingForce));
        drag->forceStarted=true;
        output=std::move(drag);rollback.committed=true;return InspectionResult::Success;
    });
}
void stepForceDrags(Owner& owner,PxScene& scene,float timeStep) noexcept
{
    if(!std::isfinite(timeStep) || timeStep<=0)return;
    for(const std::pair<const uint64_t,std::weak_ptr<Drag>>& entry:owner.interactionDrags)
    {
        std::shared_ptr<Drag> drag=entry.second.lock();if(!drag)continue;
        std::lock_guard<std::mutex> lock(drag->mutex);
        if(drag->closed || !drag->forceGrab)continue;
        PxRigidBody* body=forceBody(owner,drag->body);
        if(!body || body->getScene()!=&scene)continue;
        const PxVec3 point=body->getGlobalPose().transform(drag->localPoint);
        const PxVec3 pointVelocity=PxRigidBodyExt::getVelocityAtPos(*body,point);
        const float k=50*drag->pickingForce;
        const PxVec3 impulse=(drag->target-point-0.02f*pointVelocity)*(k*body->getMass()*timeStep);
        PxRigidBodyExt::addForceAtPos(*body,impulse,point,PxForceMode::eIMPULSE);
        if(!body->getActorFlags().isSet(PxActorFlag::eDISABLE_GRAVITY))
        {
            const PxVec3 gravity=scene.getGravity();const float magnitude=gravity.magnitude();
            if(magnitude>0)
            {
                const PxVec3 compensation=-gravity*(std::max(0.0f,magnitude-0.1f*k*drag->pickDistance)/magnitude);
                body->addForce(compensation*timeStep,PxForceMode::eVELOCITY_CHANGE);
            }
        }
    }
}
InspectionResult setForceDragCoefficient(const std::shared_ptr<Drag>& drag,float coefficient) noexcept
{
    if(!std::isfinite(coefficient) || coefficient<0 || coefficient>std::numeric_limits<float>::max()/50)return InspectionResult::Invalid;
    return withDrag(drag,[&](Drag& value) {
        std::shared_ptr<Owner> owner=value.owner.lock();
        if(!value.forceGrab || !owner || !forceBody(*owner,value.body))return InspectionResult::Unavailable;
        // Legacy settings changes affect the impulse law; angular damping was
        // captured and raised once on press, and restores to that original value.
        value.pickingForce=coefficient;return InspectionResult::Success;
    });
}
InspectionResult moveDragRay(const std::shared_ptr<Drag>& drag,const float origin[3],const float direction[3],DragState& output) noexcept
{
    PxVec3 target;
    if(!drag || !drag->forceGrab || !rayTarget(origin,direction,drag->pickDistance,target))return InspectionResult::Invalid;
    const float position[3]={target.x,target.y,target.z};return moveDrag(drag,position,output);
}
InspectionResult moveDrag(const std::shared_ptr<Drag>& drag,const float target[3],DragState& output) noexcept
{
    if(!finite(target))return InspectionResult::Invalid;
    return withDrag(drag,[&](Drag& value) {
        std::shared_ptr<omni::physx::usdparser::AttachedStage> owner=value.owner.lock();
        if(!owner || !(value.forceGrab?forceBody(*owner,value.body):dynamic(*owner,value.body)))return InspectionResult::Unavailable;
        if(value.joint) {
            value.joint->setLocalPose(PxJointActorIndex::eACTOR0,PxTransform(vec(target)));
            dynamic(*owner,value.body)->wakeUp();
        }
        value.target=vec(target);
        output=state(value);return InspectionResult::Success;
    });
}
InspectionResult inspectDrag(const std::shared_ptr<Drag>& drag,DragState& output) noexcept
{
    if(!drag)return InspectionResult::Invalid;
    {std::lock_guard<std::mutex> lock(drag->mutex);if(drag->closed){output=state(*drag);return InspectionResult::Success;}}
    return withDrag(drag,[&](Drag& value){output=state(value);return InspectionResult::Success;});
}
InspectionResult closeDrag(const std::shared_ptr<Drag>& drag) noexcept
{
    if(!drag)return InspectionResult::Invalid;
    {std::lock_guard<std::mutex> lock(drag->mutex);if(drag->closed)return InspectionResult::Success;}
    return withDrag(drag,[&](Drag& value){cleanup(value,true);return InspectionResult::Success;});
}
}
