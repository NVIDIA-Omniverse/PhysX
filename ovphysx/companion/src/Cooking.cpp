// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-COOK-001
// @covers AC-1 AC-2 AC-3 AC-4
// @implements REQ-COMP-DEFORM-COOK-001
// @covers AC-1 AC-2
// @implements REQ-COMP-PARTICLES-001
// @covers AC-1 AC-2 AC-4
// @implements REQ-CAPI-COMPANION-001
// @covers AC-1 AC-4
#include "ovphysx_companion/cooking.h"
#include "ovphysx_companion/particles.h"
#include <omni/physx/NativeSampling.h>
#include <ovphysx_companion/native_build.h>
#include "internal/sdk/ovphysxSDK.hpp"
#include "internal/sdk/CompanionRuntime.hpp"
#include <omni/physx/NativeCooking.h>
#include <omni/physx/PhysXRuntime.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <functional>

namespace ovphysx::companion::cooking
{
using Status = ovphysx_companion_status_t;
using Options = ovphysx_companion_cook_options_t;
using Mesh = ovphysx_companion_cook_mesh_t;
namespace rt = omni::physx;
struct Service {
    CompanionServiceLease lease{companionRuntime()};
    std::shared_ptr<ovphysx::CarboniteLoader> runtime;
    explicit Service(std::shared_ptr<ovphysx::CarboniteLoader> value) : runtime(std::move(value))
    {}
    ~Service() { runtime.reset(); }
};
struct Result {
    ovphysx_companion_cook_info_t info{};
    std::vector<float> points;
    std::vector<uint32_t> indices, faces;
    std::vector<int32_t> embedding;
    std::vector<std::vector<unsigned char>> blobs;
    explicit Result(uint32_t kind) {
        info.size = sizeof(info); info.version = 1; info.kind = kind;
        std::memcpy(info.sdk_build, OVPHYSX_COMPANION_PHYSX_BUILD, sizeof(info.sdk_build));
    }
    void finish() {
        info.point_count = uint32_t(points.size()/3);
        info.index_count = uint32_t(indices.size());
        info.face_map_count = uint32_t(faces.size());
        info.embedding_count = uint32_t(embedding.size());
        info.blob_count = uint32_t(blobs.size());
    }
};

Status defaults(uint32_t kind, Options* output) noexcept {
    if (!output || kind < 1 || kind > 7) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    Options p{};
    p.size=sizeof(p); p.version=1; p.kind=kind;
    p.max_hull_vertices=64; p.max_hulls=16; p.voxel_resolution=kind==7 ? 16 : 100000;
    p.sdf_resolution=256; p.sdf_subgrid_resolution=6; p.sdf_bits=16;
    p.max_spheres=128; p.seed_count=1000;
    p.simplification=.55f; p.weld_tolerance=-std::numeric_limits<float>::max();
    p.min_thickness=.001f; p.error_percentage=10;
    p.sdf_band=.01f; p.sdf_margin=.01f; p.sdf_reduction=1;
    p.scale[0]=p.scale[1]=p.scale[2]=1;
    *output=p;
    return OVPHYSX_COMPANION_OK;
}
Status open(uint64_t source, uint64_t* output) noexcept {
    if (!output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    try {
        CompanionOperationGuard admission(companionRuntime());
        if (!admission || rt::runtime::isRuntimeBusy(0)) return OVPHYSX_COMPANION_BUSY;
        std::shared_ptr<Service> service;
        {
            std::unique_lock<std::shared_mutex> instances(g_instances_mutex);
            const std::unordered_map<uint64_t, std::shared_ptr<InstanceData>>::iterator found=g_instances.find(source);
            if (found==g_instances.end()) return OVPHYSX_COMPANION_NOT_FOUND;
            if (!companionRuntime().admissionOpen() || found->second->companionDestroying) return OVPHYSX_COMPANION_BUSY;
            service=std::make_shared<Service>(found->second->carbonite);
        }
        if (!service->runtime || !rt::runtime::tryGetPhysxCookingServiceInterface()) return OVPHYSX_COMPANION_UNAVAILABLE;
        std::lock_guard<std::mutex> lock(companionRuntime().cookingServices().mutex);
        if (companionRuntime().cookingServices().nextHandle==UINT64_MAX) return OVPHYSX_COMPANION_ERROR;
        const uint64_t handle=companionRuntime().cookingServices().nextHandle++;
        companionRuntime().cookingServices().entries.emplace(handle,std::move(service));
        *output=handle;
        return OVPHYSX_COMPANION_OK;
    } catch (...) { return OVPHYSX_COMPANION_ERROR; }
}
Status close(uint64_t handle) noexcept {
    try {
        CompanionOperationGuard admission(companionRuntime());
        if (!admission) return OVPHYSX_COMPANION_BUSY;
        std::lock_guard<std::mutex> lock(companionRuntime().cookingServices().mutex);
        const std::map<uint64_t, std::shared_ptr<Service>>::iterator found=companionRuntime().cookingServices().entries.find(handle);
        if (found==companionRuntime().cookingServices().entries.end()) return OVPHYSX_COMPANION_NOT_FOUND;
        if (found->second.use_count()!=1) return OVPHYSX_COMPANION_BUSY;
        companionRuntime().cookingServices().entries.erase(found);
        return OVPHYSX_COMPANION_OK;
    } catch (...) { return OVPHYSX_COMPANION_ERROR; }
}

Status cook(uint64_t handle,const Mesh* mesh,const Options* options,uint64_t* output) noexcept {
    if (!mesh || !options || !output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    try {
        if (mesh->size!=sizeof(*mesh) || mesh->version!=1 || options->size!=sizeof(*options) || options->version!=1)
            return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        CompanionOperationGuard admission(companionRuntime());
        if (!admission) return OVPHYSX_COMPANION_BUSY;
        std::shared_ptr<Service> service;
        {
            std::lock_guard<std::mutex> lock(companionRuntime().cookingServices().mutex);
            const std::map<uint64_t, std::shared_ptr<Service>>::iterator found=companionRuntime().cookingServices().entries.find(handle);
            if (found==companionRuntime().cookingServices().entries.end()) return OVPHYSX_COMPANION_NOT_FOUND;
            service=found->second;
        }
        const Options & p=*options;
        std::shared_ptr<Result> result=std::make_shared<Result>(p.kind);
        rt::native::RawCookMesh input{mesh->point_count,mesh->index_count,mesh->face_count,mesh->right_handed,
            mesh->points,mesh->indices,mesh->faces,mesh->meters_per_unit};
        rt::native::RawCookOptions recipe{};
        recipe.kind=p.kind;
        recipe.triangle_mode=p.triangle_mode;
        recipe.max_hull_vertices=p.max_hull_vertices;
        recipe.max_hulls=p.max_hulls;
        recipe.voxel_resolution=p.voxel_resolution;
        recipe.shrink_wrap=p.shrink_wrap;
        recipe.sdf_resolution=p.sdf_resolution;
        recipe.sdf_subgrid_resolution=p.sdf_subgrid_resolution;
        recipe.sdf_bits=p.sdf_bits;
        recipe.sdf_remesh=p.sdf_remesh;
        recipe.max_spheres=p.max_spheres;
        recipe.seed_count=p.seed_count;
        recipe.sphere_mode=p.sphere_mode;
        recipe.reserved=p.reserved;
        recipe.simplification=p.simplification;
        recipe.weld_tolerance=p.weld_tolerance;
        recipe.min_thickness=p.min_thickness;
        recipe.error_percentage=p.error_percentage;
        recipe.sdf_band=p.sdf_band;
        recipe.sdf_margin=p.sdf_margin;
        recipe.sdf_reduction=p.sdf_reduction;
        std::copy_n(p.scale,3,recipe.scale);
        rt::native::RawCookResult cooked;
        switch (rt::native::cookRawMesh(input,recipe,cooked)) {
        case rt::native::InspectionResult::Success:break;
        case rt::native::InspectionResult::Invalid:return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        case rt::native::InspectionResult::Busy:return OVPHYSX_COMPANION_BUSY;
        case rt::native::InspectionResult::Unavailable:return OVPHYSX_COMPANION_UNAVAILABLE;
        default:return OVPHYSX_COMPANION_ERROR;
        }
        result->points=std::move(cooked.points);result->indices=std::move(cooked.indices);
        result->faces=std::move(cooked.faces);result->embedding=std::move(cooked.embedding);result->blobs=std::move(cooked.blobs);
        std::copy_n(cooked.recipe_hash,2,result->info.recipe_hash);
        result->info.warnings=cooked.warnings;result->info.element_width=cooked.element_width;
        result->finish();
        std::lock_guard<std::mutex> lock(companionRuntime().cookingResults().mutex);
        if (companionRuntime().cookingResults().nextHandle==UINT64_MAX) return OVPHYSX_COMPANION_ERROR;
        const uint64_t id=companionRuntime().cookingResults().nextHandle++;
        companionRuntime().cookingResults().entries.emplace(id,std::move(result));
        *output=id;
        return OVPHYSX_COMPANION_OK;
    } catch (...) { return OVPHYSX_COMPANION_ERROR; }
}
std::shared_ptr<const Result> findResult(uint64_t handle) {
    std::lock_guard<std::mutex> lock(companionRuntime().cookingResults().mutex);
    const std::map<uint64_t, std::shared_ptr<const Result>>::iterator found=companionRuntime().cookingResults().entries.find(handle);
    return found==companionRuntime().cookingResults().entries.end() ? nullptr : found->second;
}
Status describe(uint64_t handle,uint32_t size,ovphysx_companion_cook_info_t* output) noexcept {
    if (!output || size!=sizeof(*output)) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    try {
        std::shared_ptr<const Result> result=findResult(handle);
        if (!result) return OVPHYSX_COMPANION_NOT_FOUND;
        *output=result->info;
        return OVPHYSX_COMPANION_OK;
    } catch (...) { return OVPHYSX_COMPANION_ERROR; }
}
Status copy(uint64_t handle,uint32_t kind,uint32_t index,void* output,uint64_t capacity,uint64_t* required) noexcept {
    if (!required || kind>4 || (!output && capacity)) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    try {
        std::shared_ptr<const Result> result=findResult(handle);
        if (!result) return OVPHYSX_COMPANION_NOT_FOUND;
        const void* data=nullptr;
        uint64_t size=0;
        if (kind!=4 && index) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        switch (kind) {
        case 0:data=result->points.data();size=result->points.size()*sizeof(float);break;
        case 1:data=result->indices.data();size=result->indices.size()*sizeof(uint32_t);break;
        case 2:data=result->faces.data();size=result->faces.size()*sizeof(uint32_t);break;
        case 3:data=result->embedding.data();size=result->embedding.size()*sizeof(int32_t);break;
        case 4:
            if (index>=result->blobs.size()) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
            data=result->blobs[index].data();size=result->blobs[index].size();break;
        }
        *required=size;
        if (!output && !capacity) return OVPHYSX_COMPANION_OK;
        if (capacity<size) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        if (size) std::memcpy(output,data,size);
        return OVPHYSX_COMPANION_OK;
    } catch (...) { return OVPHYSX_COMPANION_ERROR; }
}
Status release(uint64_t handle) noexcept {
    try {
        std::lock_guard<std::mutex> lock(companionRuntime().cookingResults().mutex);
        return companionRuntime().cookingResults().entries.erase(handle) ? OVPHYSX_COMPANION_OK : OVPHYSX_COMPANION_NOT_FOUND;
    } catch (...) { return OVPHYSX_COMPANION_ERROR; }
}
}
using namespace ovphysx::companion::cooking;
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_cooking_acquire(
    uint32_t version,uint32_t size,ovphysx_companion_cooking_api_t* output)
{
    if (!output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if (version!=1 || size!=sizeof(*output)) return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output={sizeof(*output),1,0xfeu,OVPHYSX_COMPANION_PHYSX_BUILD,defaults,open,close,cook,describe,copy,release};
    return OVPHYSX_COMPANION_OK;
}

namespace {
Status sample(uint64_t handle,const float* points,uint32_t pointCount,const uint32_t* triangles,uint32_t triangleCount,
    const ovphysx_companion_sampling_options_t* options,uint32_t optionsSize,
    float* outputPoints,int32_t* outputTriangles,float* outputWeights,uint32_t* outputCount) noexcept
try {
    if(!options || optionsSize!=sizeof(*options) || !outputPoints || !outputTriangles || !outputWeights || !outputCount)
        return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    CompanionOperationGuard admission(companionRuntime());if(!admission)return OVPHYSX_COMPANION_BUSY;
    std::shared_ptr<Service> service;
    {std::lock_guard<std::mutex> lock(companionRuntime().cookingServices().mutex);std::map<uint64_t, std::shared_ptr<Service>>::iterator found=companionRuntime().cookingServices().entries.find(handle);
     if(found==companionRuntime().cookingServices().entries.end())return OVPHYSX_COMPANION_NOT_FOUND;
     service=found->second;}
    rt::native::SamplingOptions params{options->radius,options->volume_radius,options->max_samples,options->attempts};
    rt::native::SampledPoints result;
    const omni::physx::native::InspectionResult status=rt::native::samplePoints(points,pointCount,triangles,triangleCount,params,result);
    using R=rt::native::InspectionResult;
    if(status!=R::Success) {
        if(status==R::Invalid)return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        if(status==R::Busy)return OVPHYSX_COMPANION_BUSY;
        if(status==R::Unavailable)return OVPHYSX_COMPANION_UNAVAILABLE;
        return OVPHYSX_COMPANION_ERROR;
    }
    std::copy(result.points.begin(),result.points.end(),outputPoints);
    std::copy(result.triangles.begin(),result.triangles.end(),outputTriangles);
    std::copy(result.weights.begin(),result.weights.end(),outputWeights);
    *outputCount=uint32_t(result.triangles.size());return OVPHYSX_COMPANION_OK;
} catch(...){return OVPHYSX_COMPANION_ERROR;}
}
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_particles_acquire(
    uint32_t version,uint32_t size,ovphysx_companion_particles_api_t* output)
{
    if(!output)return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if(version!=1 || size!=sizeof(*output))return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output={sizeof(*output),1,3,open,close,sample};return OVPHYSX_COMPANION_OK;
}
