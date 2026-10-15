// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-NATIVE-COOKING-001
// @covers AC-1 AC-2 AC-3 AC-4
#include <omni/physx/NativeCooking.h>
#include <omni/physx/IPhysxCooking.h>
#include <omni/physx/IPhysxCookingService.h>
#include <omni/physx/PhysXRuntime.h>
#include "RuntimeExecution.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
namespace omni::physx::native
{
namespace rt=omni::physx;
using Mesh=RawCookMesh;
using Options=RawCookOptions;
using Result=RawCookResult;
namespace
{
bool validate(const Mesh& m, const Options& p) {
    if (p.kind<1 || p.kind>7 || !m.point_count || !m.index_count || !m.points || !m.indices ||
        m.right_handed>1 || !std::isfinite(m.meters_per_unit) || m.meters_per_unit<=0 ||
        p.reserved) return false;
    const std::function<bool(float)> finite=[](float value) {return std::isfinite(value);};
    const std::function<bool(float)> nonnegative=[&](float value) {return finite(value) && value>=0;};
    if ((p.kind==uint32_t(RawCookKind::Triangle) || p.kind==4) && (p.triangle_mode>1 || !finite(p.simplification) || p.simplification<=0 ||
        p.simplification>1 || !finite(p.weld_tolerance))) return false;
    if ((p.kind==uint32_t(RawCookKind::Convex) || p.kind==3) && (p.max_hull_vertices<4 || p.max_hull_vertices>255 ||
        !nonnegative(p.min_thickness))) return false;
    if (p.kind==3 && (!p.max_hulls || p.shrink_wrap>1 || !nonnegative(p.error_percentage) ||
        p.error_percentage>100)) return false;
    if ((p.kind==3 || p.kind==5) && (p.voxel_resolution<10000 || p.voxel_resolution>10000000)) return false;
    if (p.kind==4 && (!p.sdf_resolution || (p.sdf_bits!=8 && p.sdf_bits!=16 && p.sdf_bits!=32) ||
        p.sdf_remesh>1 || !nonnegative(p.sdf_band) || !nonnegative(p.sdf_margin) ||
        !finite(p.sdf_reduction) || p.sdf_reduction<=0 || p.sdf_reduction>1)) return false;
    if (p.kind==5 && (p.sphere_mode>2 || !p.max_spheres || !p.seed_count)) return false;
    if (p.kind==7 && (!p.voxel_resolution || p.voxel_resolution>uint32_t(INT32_MAX))) return false;
    if (p.kind==uint32_t(RawCookKind::Convex) || p.kind==3 || p.kind==5 || p.kind==7)
        for (float scale:p.scale) if (!finite(scale) || (p.kind==7 ? scale<=0 : std::abs(scale)!=1)) return false;
    for (uint64_t i=0;i<uint64_t(m.point_count)*3;++i) if (!std::isfinite(m.points[i])) return false;
    for (uint32_t i=0;i<m.index_count;++i) if (m.indices[i]<0 || uint32_t(m.indices[i])>=m.point_count) return false;
    if (p.kind>=6) return !m.face_count && !m.faces && m.index_count%(p.kind==6 ? 3 : 4)==0;
    if (!m.faces || !m.face_count) return false;
    uint64_t count=0;
    for (uint32_t i=0;i<m.face_count;++i) {
        if (m.faces[i]<3) return false;
        count+=uint32_t(m.faces[i]);
    }
    return count==m.index_count;
}
bool tetrahedral(const Mesh& mesh,const Options& p,Result& result) {
    // The runtime allocator callback may throw. Retain partially allocated output
    // fields in this scope so a later allocation failure still frees earlier ones.
    struct Buffers {
        carb::Float3* points=nullptr;
        uint32_t* indices=nullptr;
        int32_t* embedding=nullptr;
        ~Buffers() { ::operator delete(points); ::operator delete(indices); ::operator delete(embedding); }
    } data;
    uint32_t pointCount=0,indexCount=0,embeddingCount=0;
    std::vector<carb::Float3> source(mesh.point_count);
    std::vector<uint32_t> topology(mesh.indices,mesh.indices+mesh.index_count);
    for (size_t i=0;i<mesh.point_count;++i) source[i]={mesh.points[3*i],mesh.points[3*i+1],mesh.points[3*i+2]};
    void* (*allocate)(size_t)=[](size_t size)->void* { return ::operator new(size); };
    omni::physx::IPhysxCooking & cooking=rt::runtime::getPhysxCookingInterface();
    bool success;
    if (p.kind==uint32_t(RawCookKind::Conforming)) {
        if (!mesh.right_handed) for (size_t i=0;i<topology.size();i+=3) std::swap(topology[i+1],topology[i+2]);
        success=cooking.computeConformingTetrahedralMesh(data.points,pointCount,data.indices,indexCount,
            source.data(),uint32_t(source.size()),topology.data(),uint32_t(topology.size()),allocate);
    } else {
        const carb::Float3 scale{p.scale[0],p.scale[1],p.scale[2]};
        success=cooking.computeVoxelTetrahedralMesh(data.points,pointCount,data.indices,indexCount,
            data.embedding,embeddingCount,source.data(),uint32_t(source.size()),topology.data(),uint32_t(topology.size()),
            scale,int(p.voxel_resolution),allocate);
    }
    if (!success || !data.points || !data.indices || !pointCount || indexCount%4) return false;
    result.points.reserve(size_t(pointCount)*3);
    for (uint32_t i=0;i<pointCount;++i) {
        result.points.push_back(data.points[i].x);result.points.push_back(data.points[i].y);result.points.push_back(data.points[i].z);
    }
    result.indices.assign(data.indices,data.indices+indexCount);
    if (embeddingCount) result.embedding.assign(data.embedding,data.embedding+embeddingCount);
    result.element_width=4;
    return true;
}

}
InspectionResult cookRawMesh(const RawCookMesh& mesh,const RawCookOptions& p,RawCookResult& output) noexcept
try
{
    if (!validate(mesh,p)) return InspectionResult::Invalid;
    if (!runtime::tryGetPhysxCookingServiceInterface()) return InspectionResult::Unavailable;
    RuntimeAuthoringScope authoring;
    if (!authoring) return InspectionResult::Busy;
    RawCookResult result;
        if (p.kind>=6) {
            if (!tetrahedral(mesh,p,result)) return InspectionResult::Error;
        } else {
            std::vector<carb::Float3> points(mesh.point_count);
            for (size_t i=0;i<mesh.point_count;++i) points[i]={mesh.points[3*i],mesh.points[3*i+1],mesh.points[3*i+2]};
            rt::PhysxCookingComputeRequest request;
            request.dataType=rt::PhysxCookingDataType::Enum(p.kind);
            request.options.flags=0; // CPU, synchronous, no global cache side effects
            request.primMeshView.points={points.data(),points.size()};
            request.primMeshView.indices={mesh.indices,mesh.index_count};
            request.primMeshView.faces={mesh.faces,mesh.face_count};
            request.primMeshView.rightHandedOrientation=mesh.right_handed!=0;
            request.primMeshMetersPerUnit=mesh.meters_per_unit;
            request.primMeshText={"companion/raw",13};
            if (p.kind==uint32_t(RawCookKind::Triangle)) {
                request.triangulation.needsVertices=true;
                request.triangulation.needsTriangles=true;
                request.triangulation.needsTriangleFaceMap=true;
            }
            bool completed=false,failed=false;
            request.onFinished=[&](const rt::PhysxCookingComputeResult& cooked) {
                if (completed) { failed=true; return; }
                completed=true;
                if (cooked.result!=rt::PhysxCookingResult::eVALID || !cooked.isSynchronousResult) { failed=true; return; }
                try {
                    const carb::extras::hash128_t hash=cooked.cookedDataCRC.getFullHash();
                    std::memcpy(result.recipe_hash,hash.d,sizeof(result.recipe_hash));
                    result.warnings=cooked.resultWarnings.flags;
                    for (uint64_t i=0;i<cooked.cookedDataNumElements;++i) {
                        const omni::physx::PhysxCookedDataSpan & blob=cooked.cookedData[i];
                        const unsigned char * begin=static_cast<const unsigned char*>(blob.data);
                        if (!begin || !blob.sizeInBytes) { failed=true; return; }
                        result.blobs.emplace_back(begin,begin+blob.sizeInBytes);
                    }
                    // The triangulation view precedes simplification; it is not
                    // the cooked surface. Export blobs only for that recipe.
                    if (p.kind==uint32_t(RawCookKind::Triangle) && p.triangle_mode==1) return;
                    const omni::physx::PhysxCookingMeshTriangulationView & tri=cooked.triangulationView;
                    for (carb::Float3 const & point:tri.points) {
                        result.points.push_back(point.x);result.points.push_back(point.y);result.points.push_back(point.z);
                    }
                    for (const uint32_t (&triangle)[3]:tri.triangles) result.indices.insert(result.indices.end(),triangle,triangle+3);
                    for (uint32_t face:tri.trianglesFaceMap) result.faces.push_back(face);
                    if (!result.indices.empty()) result.element_width=3;
                } catch (...) { failed=true; }
            };
            omni::physx::IPhysxCookingService & cooking=rt::runtime::getPhysxCookingServiceInterface();
            rt::TriangleMeshCookingParams triangle;
            triangle.mode=rt::TriangleMeshMode::Enum(p.triangle_mode);
            triangle.simplificationMetric=p.simplification;
            triangle.meshWeldTolerance=p.weld_tolerance;
            if (p.kind==uint32_t(RawCookKind::Triangle)) {
                cooking.requestTriangleMeshCookedData(nullptr,request,triangle);
            } else if (p.kind==uint32_t(RawCookKind::Convex)) {
                rt::ConvexMeshCookingParams params;
                params.maxHullVertices=p.max_hull_vertices;params.minThickness=p.min_thickness;
                params.signScale={p.scale[0],p.scale[1],p.scale[2]};
                cooking.requestConvexMeshCookedData(nullptr,request,params);
            } else if (p.kind==uint32_t(RawCookKind::Decomposition)) {
                rt::ConvexDecompositionCookingParams params;
                params.maxHullVertices=p.max_hull_vertices;params.maxConvexHulls=p.max_hulls;
                params.voxelResolution=p.voxel_resolution;params.errorPercentage=p.error_percentage;
                params.minThickness=p.min_thickness;params.shrinkWrap=p.shrink_wrap!=0;
                params.signScale={p.scale[0],p.scale[1],p.scale[2]};
                cooking.requestConvexMeshDecompositionCookedData(nullptr,request,params);
            } else if (p.kind==uint32_t(RawCookKind::Sdf)) {
                rt::SdfMeshCookingParams params;
                params.sdfResolution=p.sdf_resolution;params.sdfSubgridResolution=p.sdf_subgrid_resolution;
                params.sdfBitsPerSubgridPixel=p.sdf_bits;params.sdfNarrowBandThickness=p.sdf_band;
                params.sdfMargin=p.sdf_margin;params.sdfEnableRemeshing=p.sdf_remesh!=0;
                params.sdfTriangleCountReductionFactor=p.sdf_reduction;
                cooking.requestSdfMeshCookedData(nullptr,request,triangle,params);
            } else {
                rt::SphereFillCookingParams params;
                params.maxSpheres=p.max_spheres;params.seedCount=p.seed_count;
                params.voxelResolution=p.voxel_resolution;params.fillMode=rt::SphereFillMode::Enum(p.sphere_mode);
                params.signScale={p.scale[0],p.scale[1],p.scale[2]};
                cooking.requestSphereFillCookedData(nullptr,request,params);
            }
            if (!completed || failed || result.blobs.empty()) return InspectionResult::Error;
        }
    output=std::move(result);
    return InspectionResult::Success;
}
catch (...) {return InspectionResult::Error;}
}
