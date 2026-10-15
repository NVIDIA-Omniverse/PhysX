// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-AUTHORING-ELEMENTS-001
// @covers AC-1 AC-2 AC-3 AC-4 AC-5
#include <omni/physx/AuthoringElements.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace omni::physx::authoring
{
namespace {
struct Vec {
    double x, y, z;
    Vec operator+(Vec b) const { return {x+b.x,y+b.y,z+b.z}; }
    Vec operator-(Vec b) const { return {x-b.x,y-b.y,z-b.z}; }
    Vec operator*(double s) const { return {x*s,y*s,z*s}; }
};
Vec toVec(const ::physx::PxVec3& v) { return {v.x,v.y,v.z}; }
double dot(Vec a, Vec b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
Vec cross(Vec a, Vec b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
using Weights = std::array<double, 4>;
Weights triangleWeights(Vec p, Vec a, Vec b, Vec c) {
    const Vec ab=b-a, ac=c-a, ap=p-a;
    const double d1=dot(ab,ap), d2=dot(ac,ap);
    if (d1<=0 && d2<=0) return {1,0,0,0};
    const Vec bp=p-b;
    const double d3=dot(ab,bp), d4=dot(ac,bp);
    if (d3>=0 && d4<=d3) return {0,1,0,0};
    const double vc=d1*d4-d3*d2;
    if (vc<=0 && d1>=0 && d3<=0) {
        const double v=d1/(d1-d3); return {1-v,v,0,0};
    }
    const Vec cp=p-c;
    const double d5=dot(ab,cp), d6=dot(ac,cp);
    if (d6>=0 && d5<=d6) return {0,0,1,0};
    const double vb=d5*d2-d1*d6;
    if (vb<=0 && d2>=0 && d6<=0) {
        const double w=d2/(d2-d6); return {1-w,0,w,0};
    }
    const double va=d3*d6-d5*d4;
    if (va<=0 && d4-d3>=0 && d5-d6>=0) {
        const double w=(d4-d3)/((d4-d3)+(d5-d6)); return {0,1-w,w,0};
    }
    const double sum=va+vb+vc;
    const double v=vb/sum, w=vc/sum;
    return {1-v-w,v,w,0};
}
Vec evaluate(const std::array<Vec,4>& v, const Weights& w, uint32_t width) {
    Vec result{0,0,0};
    for (uint32_t k=0;k<width;++k) result=result+v[k]*w[k];
    return result;
}
bool topology(const ::physx::PxVec3* vertices, uint32_t vertexCount,
              const uint32_t* indices, uint32_t elementCount, uint32_t width, uint32_t& invalid, bool checkDegeneracy=true) {
    invalid=UINT32_MAX;
    if ((width!=3 && width!=4) || (!vertices && vertexCount) || (!indices && elementCount) ||
        elementCount>uint32_t(INT32_MAX)) return false;
    for (uint32_t i=0;i<vertexCount;++i) if (!vertices[i].isFinite()) return false;
    for (uint32_t i=0;i<elementCount;++i) {
        std::array<Vec,4> v{};
        for (uint32_t k=0;k<width;++k) {
            const unsigned int index=indices[size_t(i)*width+k];
            if (index>=vertexCount) { invalid=i; return false; }
            v[k]=toVec(vertices[index]);
        }
        const Vec a=v[1]-v[0], b=v[2]-v[0], c=v[3]-v[0];
        double scale=std::max(dot(a,a),dot(b,b));
        const Vec normal=cross(a,b);
        const double det=dot(normal,c);
        if (width==4) scale=std::max(scale,dot(c,c));
        const bool degenerate=width==3 ? dot(normal,normal)<=1e-24*scale*scale :
            det*det<=1e-24*scale*scale*scale;
        if (checkDegeneracy && degenerate) { invalid=i; return false; }
    }
    return true;
}
}

bool embedPoints(const ::physx::PxVec3* vertices, uint32_t vertexCount,
    const uint32_t* indices, uint32_t elementCount, uint32_t width,
    const ::physx::PxVec3* points, uint32_t pointCount, float maxDistance,
    ElementMapping& output, uint32_t& invalidElement)
{
    invalidElement=UINT32_MAX;
    if (pointCount > 1000000 || vertexCount > 1000000 || elementCount > 1000000 ||
        uint64_t(pointCount) * elementCount > 10000000 ||
        (!points && pointCount) || !std::isfinite(maxDistance) || maxDistance<0 ||
        !topology(vertices,vertexCount,indices,elementCount,width,invalidElement)) return false;
    for (uint32_t i=0;i<pointCount;++i) if (!points[i].isFinite()) return false;
    ElementMapping result;
    result.elements.assign(pointCount,-1);
    result.weights.assign(size_t(pointCount)*4,0);
    result.distances.assign(pointCount,std::numeric_limits<float>::infinity());
    result.normalOffsets.assign(pointCount,0);
    for (uint32_t i=0;i<pointCount;++i) {
        const Vec point=toVec(points[i]);
        double best=double(maxDistance)*maxDistance;
        for (uint32_t e=0;e<elementCount;++e) {
            std::array<Vec,4> v{};
            for (uint32_t k=0;k<width;++k) v[k]=toVec(vertices[indices[size_t(e)*width+k]]);
            Weights weights{};
            double distance=0, offset=0;
            if (width==3) {
                weights=triangleWeights(point,v[0],v[1],v[2]);
                const Vec difference=point-evaluate(v,weights,width);
                distance=dot(difference,difference);
                const Vec normal=cross(v[1]-v[0],v[2]-v[0]);
                offset=dot(difference,normal)/std::sqrt(dot(normal,normal));
            } else {
                const Vec a=v[1]-v[0], b=v[2]-v[0], c=v[3]-v[0], p=point-v[0];
                const double det=dot(a,cross(b,c));
                weights[1]=dot(p,cross(b,c))/det;
                weights[2]=dot(a,cross(p,c))/det;
                weights[3]=dot(a,cross(b,p))/det;
                weights[0]=1-weights[1]-weights[2]-weights[3];
                if (*std::min_element(weights.begin(),weights.end())<0) {
                    distance=std::numeric_limits<double>::infinity();
                    constexpr uint32_t faces[4][3]={{0,1,2},{0,1,3},{0,2,3},{1,2,3}};
                    for (const uint32_t (&face)[3]:faces) {
                        const std::array<double, 4> w=triangleWeights(point,v[face[0]],v[face[1]],v[face[2]]);
                        Weights candidate{};
                        for (uint32_t k=0;k<3;++k) candidate[face[k]]=w[k];
                        const Vec difference=point-evaluate(v,candidate,width);
                        const double squared=dot(difference,difference);
                        if (squared<distance) { distance=squared; weights=candidate; }
                    }
                }
            }
            if (distance<best || (distance==best && result.elements[i]<0)) {
                best=distance;
                result.elements[i]=int32_t(e);
                for (uint32_t k=0;k<4;++k) result.weights[size_t(i)*4+k]=float(weights[k]);
                result.distances[i]=float(std::sqrt(distance));
                result.normalOffsets[i]=float(offset);
            }
        }
    }
    output=std::move(result);
    return true;
}

bool interpolateElements(const ::physx::PxVec3* vertices, uint32_t vertexCount,
    const uint32_t* indices, uint32_t elementCount, uint32_t width,
    const int32_t* elements, const float* weights, const float* normalOffsets,
    uint32_t pointCount, std::vector<::physx::PxVec3>& output)
{
    uint32_t invalid;
    if ((pointCount && (!elements || !weights || !normalOffsets)) ||
        !topology(vertices,vertexCount,indices,elementCount,width,invalid,false)) return false;
    std::vector<::physx::PxVec3> result;
    result.reserve(pointCount);
    for (uint32_t i=0;i<pointCount;++i) {
        if (elements[i]<0 || uint32_t(elements[i])>=elementCount || !std::isfinite(normalOffsets[i]) ||
            (width==4 && normalOffsets[i]!=0)) return false;
        Weights w{};
        double sum=0;
        std::array<Vec,4> v{};
        for (uint32_t k=0;k<4;++k) {
            w[k]=weights[size_t(i)*4+k];
            if (!std::isfinite(w[k]) || w[k]<-1e-6 || w[k]>1+1e-6 || (k>=width && w[k]!=0)) return false;
            sum+=w[k];
            if (k<width) v[k]=toVec(vertices[indices[size_t(elements[i])*width+k]]);
        }
        if (std::abs(sum-1)>1e-5) return false;
        Vec point=evaluate(v,w,width);
        if (width==3 && normalOffsets[i]!=0) {
            const Vec normal=cross(v[1]-v[0],v[2]-v[0]);
            const double magnitude=dot(normal,normal);
            if (magnitude==0) return false;
            point=point+normal*(normalOffsets[i]/std::sqrt(magnitude));
        }
        const double limit=std::numeric_limits<float>::max();
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
            std::abs(point.x)>limit || std::abs(point.y)>limit || std::abs(point.z)>limit) return false;
        result.emplace_back(float(point.x),float(point.y),float(point.z));
    }
    output=std::move(result);
    return true;
}

bool interpolateElementVelocities(const ::physx::PxVec3* vertices,
    const ::physx::PxVec3* velocities, uint32_t vertexCount,
    const uint32_t* indices, uint32_t elementCount, uint32_t width,
    const int32_t* elements, const float* weights, const float* normalOffsets,
    uint32_t pointCount, std::vector<::physx::PxVec3>& output)
{
    // Validate geometry, topology, mapping and vector fields before publishing.
    std::vector<::physx::PxVec3> result;
    if (!interpolateElements(vertices, vertexCount, indices, elementCount, width,
            elements, weights, normalOffsets, pointCount, result)) return false;
    std::vector<float> zeroOffsets(pointCount, 0);
    if (!interpolateElements(velocities, vertexCount, indices, elementCount, width,
            elements, weights, zeroOffsets.data(), pointCount, result)) return false;
    for (uint32_t i=0; i<pointCount; ++i)
    {
        if (width != 3 || normalOffsets[i] == 0) continue;
        const unsigned int * tri = indices + size_t(elements[i])*3;
        const Vec a = toVec(vertices[tri[1]])-toVec(vertices[tri[0]]);
        const Vec b = toVec(vertices[tri[2]])-toVec(vertices[tri[0]]);
        const Vec da = toVec(velocities[tri[1]])-toVec(velocities[tri[0]]);
        const Vec db = toVec(velocities[tri[2]])-toVec(velocities[tri[0]]);
        const Vec crossProduct = cross(a,b);
        const double length = std::sqrt(dot(crossProduct,crossProduct));
        const Vec normal = crossProduct*(1/length);
        const Vec derivative = cross(da,b)+cross(a,db);
        const Vec value = toVec(result[i]) +
            (derivative-normal*dot(normal,derivative))*(normalOffsets[i]/length);
        const double limit=std::numeric_limits<float>::max();
        if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z) ||
            std::abs(value.x)>limit || std::abs(value.y)>limit || std::abs(value.z)>limit) return false;
        result[i] = ::physx::PxVec3(float(value.x),float(value.y),float(value.z));
    }
    output=std::move(result);
    return true;
}
namespace {
double segmentDistanceSquared(Vec p, Vec q, Vec a, Vec b) {
    const Vec d=q-p, e=b-a, r=p-a;
    const double dd=dot(d,d), ee=dot(e,e), de=dot(d,e), dr=dot(d,r), er=dot(e,r);
    const double denominator=dd*ee-de*de;
    double s=denominator>0 ? std::clamp((de*er-dr*ee)/denominator,0.0,1.0) : 0;
    double t=(de*s+er)/ee;
    if (t<0) { t=0; s=std::clamp(-dr/dd,0.0,1.0); }
    else if (t>1) { t=1; s=std::clamp((de-dr)/dd,0.0,1.0); }
    const Vec difference=r+d*s-e*t;
    return dot(difference,difference);
}
bool segmentIntersectsTriangle(Vec p, Vec q, const std::array<Vec,4>& tri) {
    const Vec ab=tri[1]-tri[0], ac=tri[2]-tri[0], n=cross(ab,ac);
    const double start=dot(p-tri[0],n), finish=dot(q-tri[0],n);
    if ((start>0 && finish>0) || (start<0 && finish<0) || start==finish) return false;
    const Vec point=p+(q-p)*(start/(start-finish)), ap=point-tri[0];
    const double nn=dot(n,n);
    const double u=dot(cross(ap,ac),n)/nn, v=dot(cross(ab,ap),n)/nn;
    return u>=0 && v>=0 && u+v<=1;
}
double triangleDistanceSquared(const std::array<Vec,4>& a, const std::array<Vec,4>& b) {
    double best=std::numeric_limits<double>::infinity();
    for (uint32_t i=0;i<3;++i) {
        if (segmentIntersectsTriangle(a[i],a[(i+1)%3],b) ||
            segmentIntersectsTriangle(b[i],b[(i+1)%3],a)) return 0;
        const Vec da=a[i]-evaluate(b,triangleWeights(a[i],b[0],b[1],b[2]),3);
        const Vec db=b[i]-evaluate(a,triangleWeights(b[i],a[0],a[1],a[2]),3);
        best=std::min({best,dot(da,da),dot(db,db)});
        for (uint32_t j=0;j<3;++j)
            best=std::min(best,segmentDistanceSquared(a[i],a[(i+1)%3],b[j],b[(j+1)%3]));
    }
    return best;
}
}

bool triangleFilterPairs(const ::physx::PxVec3* vertices0, uint32_t vertexCount0,
    const uint32_t* triangles0, uint32_t triangleCount0,
    const ::physx::PxVec3* vertices1, uint32_t vertexCount1,
    const uint32_t* triangles1, uint32_t triangleCount1,
    float maxDistance, uint32_t maxPairs, std::vector<uint32_t>& output)
{
    uint32_t invalid;
    if (!std::isfinite(maxDistance) || maxDistance<0 || maxPairs>1000000 ||
        uint64_t(triangleCount0)*triangleCount1>10000000 ||
        !topology(vertices0,vertexCount0,triangles0,triangleCount0,3,invalid) ||
        !topology(vertices1,vertexCount1,triangles1,triangleCount1,3,invalid)) return false;
    std::vector<uint32_t> pairs;
    for (uint32_t i=0;i<triangleCount0;++i) {
        std::array<Vec,4> a{};
        for (uint32_t k=0;k<3;++k) a[k]=toVec(vertices0[triangles0[size_t(i)*3+k]]);
        for (uint32_t j=0;j<triangleCount1;++j) {
            std::array<Vec,4> b{};
            for (uint32_t k=0;k<3;++k) b[k]=toVec(vertices1[triangles1[size_t(j)*3+k]]);
            bool apart=false;
            const double aa[3][3]={{a[0].x,a[1].x,a[2].x},{a[0].y,a[1].y,a[2].y},{a[0].z,a[1].z,a[2].z}};
            const double bb[3][3]={{b[0].x,b[1].x,b[2].x},{b[0].y,b[1].y,b[2].y},{b[0].z,b[1].z,b[2].z}};
            for (uint32_t axis=0;axis<3;++axis) {
                const std::pair<const double *, const double *> aminmax=std::minmax_element(aa[axis],aa[axis]+3);
                const std::pair<const double *, const double *> bminmax=std::minmax_element(bb[axis],bb[axis]+3);
                apart |= *aminmax.first>*bminmax.second+maxDistance || *bminmax.first>*aminmax.second+maxDistance;
            }
            if (apart || triangleDistanceSquared(a,b)>double(maxDistance)*maxDistance) continue;
            if (pairs.size()/2==maxPairs) return false;
            pairs.push_back(i); pairs.push_back(j);
        }
    }
    output=std::move(pairs);
    return true;
}

}
