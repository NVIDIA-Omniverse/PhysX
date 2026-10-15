// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-NATIVE-QUERY-001
// @covers AC-1 AC-2 AC-3 AC-4
#include <omni/physx/NativeQueries.h>
#include <omni/physx/PhysXRuntime.h>
#include "NativeObjects.h"
#include "RuntimeExecution.h"
#include "PhysXScene.h"
#include "PhysXTools.h"
#include "usdLoad/LoadUsd.h"
#include <algorithm>
#include <cmath>
#include <tuple>
#include <type_traits>
namespace omni::physx::native
{
namespace
{
using namespace ::physx;
using Owner=usdparser::AttachedStage;
bool less(const SceneQueryHit& a,const SceneQueryHit& b)
{
    return std::tie(a.distance,a.body.object,a.body.instance,a.shape.object,a.face)<
           std::tie(b.distance,b.body.object,b.body.instance,b.shape.object,b.face);
}
struct Filter:PxQueryFilterCallback
{
    const SceneQuery& query;
    explicit Filter(const SceneQuery& value):query(value){}
    bool contains(const std::vector<ObjectReference>& refs,const PxShape* shape,const PxRigidActor* body)
    {
        return std::any_of(refs.begin(),refs.end(),[&](const ObjectReference& r) {
            return r.object==uint64_t(r.kind==uint32_t(ObjectKind::Body)?body->userData:shape->userData);
        });
    }
    PxQueryHitType::Enum preFilter(const PxFilterData&,const PxShape* shape,const PxRigidActor* actor,PxHitFlags&) override
    {
        return (query.include.empty() || contains(query.include,shape,actor)) && !contains(query.exclude,shape,actor)
            ? PxQueryHitType::eTOUCH : PxQueryHitType::eNONE;
    }
    PxQueryHitType::Enum postFilter(const PxFilterData&,const PxQueryHit&,const PxShape*,const PxRigidActor*) override
    {return PxQueryHitType::eTOUCH;}
};
template<class Hit> struct Collector:PxHitCallback<Hit>
{
    Hit buffer[128];
    const Owner& owner;
    const SceneQuery& query;
    SceneQueryResult& result;
    Collector(const Owner& o,const SceneQuery& q,SceneQueryResult& r):PxHitCallback<Hit>(buffer,128),owner(o),query(q),result(r){}
    PxAgain processTouches(const Hit* hits,PxU32 count) override
    {
        for(PxU32 i=0;i<count;++i)
        {
            const Hit& h=hits[i];SceneQueryHit value;
            value.body=objectReference(owner,uint64_t(h.actor->userData));
            value.shape=objectReference(owner,uint64_t(h.shape->userData));
            if constexpr(!std::is_same_v<Hit,PxOverlapHit>)
            {
                value.distance=h.distance;value.validFields=1;
                if(h.flags.isSet(PxHitFlag::ePOSITION)) {
                    value.validFields|=4;value.position[0]=h.position.x;value.position[1]=h.position.y;value.position[2]=h.position.z;
                }
                if(h.flags.isSet(PxHitFlag::eNORMAL)) {
                    value.validFields|=8;value.normal[0]=h.normal.x;value.normal[1]=h.normal.y;value.normal[2]=h.normal.z;
                }
                if(h.shape->getGeometry().getType()==PxGeometryType::eTRIANGLEMESH && h.faceIndex!=UINT32_MAX) {
                    const ::physx::PxTriangleMeshGeometry & meshGeometry=static_cast<const PxTriangleMeshGeometry&>(h.shape->getGeometry());
                    const ::physx::PxTriangleMesh * mesh=meshGeometry.triangleMesh;
                    const unsigned int * remap=mesh?mesh->getTrianglesRemap():nullptr;
                    const unsigned int * faces=mesh?getMeshCache()->getTriangleMeshFaceMap(mesh):nullptr;
                    if(remap && faces && h.faceIndex<mesh->getNbTriangles()) {
                        value.face=faces[remap[h.faceIndex]];value.validFields|=2;
                    }
                }
                const PxBaseMaterial* base=h.shape->getMaterialFromInternalFaceIndex(h.faceIndex);
                const PxMaterial* material=base?base->template is<PxMaterial>():nullptr;
                if(material)value.material=objectReference(owner,uint64_t(material->userData));
            }
            ++result.total;
            if(query.mode==1) {result.hits.push_back(value);return false;}
            const size_t capacity=query.mode==0?1:query.capacity;
            if(result.hits.size()<capacity) {
                result.hits.push_back(value);std::push_heap(result.hits.begin(),result.hits.end(),less);
            }
            else if(less(value,result.hits.front())) {
                std::pop_heap(result.hits.begin(),result.hits.end(),less);result.hits.back()=value;
                std::push_heap(result.hits.begin(),result.hits.end(),less);
            }
        }
        return true;
    }
};
bool finite(const float* values,size_t count)
{
    for(size_t i=0;i<count;++i)if(!std::isfinite(values[i]))return false;
    return true;
}
}
InspectionResult queryScene(uint64_t attachment,const SceneQuery& query,SceneQueryResult& output) noexcept
try
{
    if(query.kind>2 || query.mode>2 || !query.capacity || query.capacity>65536 || query.flags&~7u || !(query.flags&3) ||
       query.include.size()>256 || query.exclude.size()>256 || !finite(query.pose,7))return InspectionResult::Invalid;
    if(query.kind==2 && (query.mode==0 || query.flags&4))return InspectionResult::Unavailable;
    PxTransform pose(PxVec3(query.pose[0],query.pose[1],query.pose[2]),PxQuat(query.pose[3],query.pose[4],query.pose[5],query.pose[6]));
    if(!pose.isValid())return InspectionResult::Invalid;
    PxVec3 direction(1,0,0);
    if(query.kind!=2) {
        if(!finite(query.direction,3) || !std::isfinite(query.distance) || query.distance<=0)return InspectionResult::Invalid;
        if(query.kind==1 && query.distance>PX_MAX_SWEEP_DISTANCE)return InspectionResult::Invalid;
        direction=PxVec3(query.direction[0],query.direction[1],query.direction[2]);
        if(!direction.isNormalized())return InspectionResult::Invalid;
    }
    if(!runtime::tryGetPhysxInterface())return InspectionResult::Stale;
    std::shared_ptr<omni::physx::usdparser::AttachedStage> owner=usdparser::UsdLoad::getUsdLoad()->acquireAttachment(attachment);
    if(!owner)return InspectionResult::Stale;
    RuntimeExecutionScope admission(attachment);if(!admission)return InspectionResult::Busy;
    if(usdparser::UsdLoad::getUsdLoad()->getAttachedStageByHandle(attachment)!=owner.get())return InspectionResult::Stale;
    for(const std::vector<omni::physx::native::ObjectReference> * refs:{&query.include,&query.exclude})for(const omni::physx::native::ObjectReference & ref:*refs) {
        if(ref.kind!=uint32_t(ObjectKind::Body) && ref.kind!=uint32_t(ObjectKind::Shape))return InspectionResult::Invalid;
        if(!objectRecord(*owner,ref))return InspectionResult::Stale;
    }
    if(owner->getPhysXScenes().empty())return InspectionResult::Unavailable;
    for(const std::pair<const uint64_t, omni::physx::PhysXScene *> & item:owner->getPhysXScenes()) {
        const ::physx::PxScene * scene=item.second->getScene();
        if(scene && (scene->getFlags().isSet(PxSceneFlag::eENABLE_GPU_DYNAMICS) ||
            scene->getFlags().isSet(PxSceneFlag::eENABLE_DIRECT_GPU_API)))return InspectionResult::Unavailable;
    }
    PxGeometryHolder geometry;
    if(query.kind) {
        if(!finite(query.dimensions,3))return InspectionResult::Invalid;
        switch(query.geometry) {
        case 1: {PxSphereGeometry g(query.dimensions[0]);if(!g.isValid())return InspectionResult::Invalid;geometry.storeAny(g);break;}
        case 2: {PxBoxGeometry g(query.dimensions[0],query.dimensions[1],query.dimensions[2]);if(!g.isValid())return InspectionResult::Invalid;geometry.storeAny(g);break;}
        case 3: {PxCapsuleGeometry g(query.dimensions[0],query.dimensions[1]);if(!g.isValid())return InspectionResult::Invalid;geometry.storeAny(g);break;}
        case 4: {
            if(query.shape.kind!=uint32_t(ObjectKind::Shape))return InspectionResult::Invalid;
            const omni::physx::internal::InternalDatabase::Record * record=objectRecord(*owner,query.shape);if(!record)return InspectionResult::Stale;
            if(record->mType!=ePTShape)return InspectionResult::Unavailable;
            geometry.storeAny(static_cast<const PxShape*>(record->mPtr)->getGeometry());
            const ::physx::PxGeometryType::Enum type=geometry.getType();
            if(type!=PxGeometryType::eSPHERE && type!=PxGeometryType::eBOX && type!=PxGeometryType::eCAPSULE &&
                type!=PxGeometryType::eCONVEXMESH)return InspectionResult::Unavailable;
            break;
        }
        default:return InspectionResult::Unavailable;
        }
    }
    SceneQueryResult result;result.attachment=attachment;result.step=owner->simulationTimestamp();
    result.revision=owner->recordVersion.load(std::memory_order_relaxed);
    result.hits.reserve(query.mode==2?query.capacity:1);
    PxQueryFilterData data;data.flags=PxQueryFlag::ePREFILTER|PxQueryFlag::eNO_BLOCK;
    if(query.flags&1)data.flags|=PxQueryFlag::eSTATIC;
    if(query.flags&2)data.flags|=PxQueryFlag::eDYNAMIC;
    Filter filter(query);PxHitFlags flags=PxHitFlag::eDEFAULT;
    if(query.flags&4)flags|=PxHitFlag::eMESH_BOTH_SIDES;
    for(const std::pair<const uint64_t, omni::physx::PhysXScene *> & item:owner->getPhysXScenes()) {
        const ::physx::PxScene * scene=item.second->getScene();if(!scene)continue;
        // SDK queries can invoke custom geometry; reject reentrant owner work.
        RuntimeCallbackScope callbacks(attachment);
        if(query.kind==0) {
            Collector<PxRaycastHit> collector(*owner,query,result);
            scene->raycast(pose.p,direction,query.distance,collector,flags,data,&filter);
        } else if(query.kind==1) {
            Collector<PxSweepHit> collector(*owner,query,result);
            scene->sweep(geometry.any(),pose,direction,query.distance,collector,flags,data,&filter);
        } else {
            Collector<PxOverlapHit> collector(*owner,query,result);
            scene->overlap(geometry.any(),pose,collector,data,&filter);
        }
        if(query.mode==1 && result.total)break;
    }
    std::sort(result.hits.begin(),result.hits.end(),less);
    result.truncated=query.mode==2 && result.total>result.hits.size();
    output=std::move(result);return InspectionResult::Success;
}
catch(...){return InspectionResult::Error;}
}
