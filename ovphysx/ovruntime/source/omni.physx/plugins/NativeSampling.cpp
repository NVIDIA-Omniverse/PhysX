// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-NATIVE-SAMPLING-001
// @covers AC-1 AC-2 AC-3
#include <omni/physx/NativeSampling.h>
#include <omni/physx/PhysXRuntime.h>
#include "RuntimeExecution.h"
#include <extensions/PxSamplingExt.h>
#include <map>
#include <cmath>
#include <functional>
namespace omni::physx::native
{
InspectionResult samplePoints(const float* points,uint32_t pointCount,const uint32_t* triangles,
    uint32_t triangleCount,const SamplingOptions& options,SampledPoints& output) noexcept
try
{
    if(!points || !triangles || pointCount<3 || !triangleCount || pointCount>10000000 || triangleCount>10000000 ||
       !std::isfinite(options.radius) || options.radius<=0 || !std::isfinite(options.volumeRadius) || options.volumeRadius<0 ||
       !options.maxSamples || options.maxSamples>1000000 || !options.attempts || options.attempts>100)
        return InspectionResult::Invalid;
    std::vector<::physx::PxVec3> vertices;vertices.reserve(pointCount);
    for(uint32_t i=0;i<pointCount;++i) {
        const float * p=points+size_t(i)*3;
        if(!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]))return InspectionResult::Invalid;
        vertices.emplace_back(p[0],p[1],p[2]);
    }
    struct Edge {uint32_t count=0;int orientation=0;uint32_t triangle=0;};
    std::vector<uint32_t> parents(triangleCount);
    for(uint32_t i=0;i<triangleCount;++i)parents[i]=i;
    const std::function<uint32_t(uint32_t)> root=[&](uint32_t index) {
        while(parents[index]!=index) {parents[index]=parents[parents[index]];index=parents[index];}
        return index;
    };
    std::map<std::pair<uint32_t,uint32_t>,Edge> edges;
    for(uint32_t i=0;i<triangleCount;++i) {
        const unsigned int * t=triangles+size_t(i)*3;
        if(t[0]>=pointCount || t[1]>=pointCount || t[2]>=pointCount)return InspectionResult::Invalid;
        const float area=(vertices[t[1]]-vertices[t[0]]).cross(vertices[t[2]]-vertices[t[0]]).magnitudeSquared();
        if(!std::isfinite(area) || area<=0)return InspectionResult::Invalid;
        if(options.volumeRadius>0)for(uint32_t e=0;e<3;++e) {
            const unsigned int a=t[e],b=t[(e+1)%3];Edge & edge=edges[{std::min(a,b),std::max(a,b)}];
            if(edge.count)parents[root(i)]=root(edge.triangle);else edge.triangle=i;
            ++edge.count;edge.orientation+=a<b?1:-1;
        }
    }
    for(const std::pair<const std::pair<unsigned int, unsigned int>, Edge> & item:edges)if(item.second.count!=2 || item.second.orientation!=0)return InspectionResult::Invalid;
    if(options.volumeRadius>0) {
        // The SDK inside test requires outward winding. Reject every inward
        // connected component (including mirrored input), never empty-success.
        std::map<uint32_t,double> volumes;
        const ::physx::PxVec3 reference=vertices[0];
        for(uint32_t i=0;i<triangleCount;++i) {
            const uint32_t* t=triangles+size_t(i)*3;
            const ::physx::PxVec3 a=vertices[t[0]]-reference,b=vertices[t[1]]-reference,c=vertices[t[2]]-reference;
            volumes[root(i)]+=double(a.x)*(double(b.y)*c.z-double(b.z)*c.y)+
                double(a.y)*(double(b.z)*c.x-double(b.x)*c.z)+double(a.z)*(double(b.x)*c.y-double(b.y)*c.x);
        }
        for(const std::pair<const uint32_t,double>& volume:volumes)
            if(!std::isfinite(volume.second) || volume.second<=0)return InspectionResult::Invalid;
    }
    if(!runtime::tryGetPhysxInterface())return InspectionResult::Unavailable;
    RuntimeAuthoringScope authoring;
    if (!authoring) return InspectionResult::Busy;
    ::physx::PxSimpleTriangleMesh mesh;
    mesh.points.data=vertices.data();mesh.points.count=pointCount;mesh.points.stride=sizeof(vertices[0]);
    mesh.triangles.data=triangles;mesh.triangles.count=triangleCount;mesh.triangles.stride=3*sizeof(uint32_t);
    ::physx::PxArray<::physx::PxVec3> sampled,weights;::physx::PxArray<::physx::PxI32> ids;
    if(!::physx::PxSamplingExt::poissonSample(mesh,options.radius,sampled,options.volumeRadius,&ids,&weights,
         nullptr,nullptr,options.maxSamples,options.attempts))return InspectionResult::Error;
    if(weights.size()!=ids.size() || ids.size()>sampled.size())return InspectionResult::Error;
    const uint32_t count=std::min(sampled.size(),options.maxSamples);
    SampledPoints result;result.points.reserve(size_t(count)*3);
    result.weights.resize(size_t(count)*3,0);result.triangles.resize(count,-1);
    for(uint32_t i=0;i<count;++i) {
        if(!sampled[i].isFinite())return InspectionResult::Error;
        result.points.insert(result.points.end(),{sampled[i].x,sampled[i].y,sampled[i].z});
        if(i<ids.size()) {
            if(ids[i]==-1 && options.volumeRadius>0)continue;
            if(ids[i]<0 || uint32_t(ids[i])>=triangleCount || !weights[i].isFinite())return InspectionResult::Error;
            result.triangles[i]=ids[i];result.weights[3*i]=weights[i].x;
            result.weights[3*i+1]=weights[i].y;result.weights[3*i+2]=weights[i].z;
        }
    }
    output=std::move(result);return InspectionResult::Success;
}
catch(...){return InspectionResult::Error;}
}
