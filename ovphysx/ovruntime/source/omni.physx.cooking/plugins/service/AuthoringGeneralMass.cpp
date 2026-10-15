// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-AUTHORING-MASS-001
// @covers AC-1 AC-2 AC-3 AC-4
#include <omni/physx/AuthoringGeneralMass.h>
#include <extensions/PxMassProperties.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <vector>
namespace omni::physx::authoring
{
namespace
{
using namespace ::physx;
struct Part
{
    double mass = 0, volume = 0, center[3]{}, tensor[3][3]{};
    PxBounds3 bounds = PxBounds3::empty();
};
bool positive(double value)
{
    return std::isfinite(value) && value >= std::numeric_limits<float>::min() &&
           value <= std::numeric_limits<float>::max();
}
bool diagonalize(const Part& part, PxVec3& diagonal, PxQuat& axes)
{
    double scale = 0;
    for (const double (&row)[3] : part.tensor) for (double value : row)
    {
        if (!std::isfinite(value)) return false;
        scale = std::max(scale, std::abs(value));
    }
    if (!positive(scale)) return false;
    PxMat33 normalized(PxZero);
    for (uint32_t r = 0; r < 3; ++r) for (uint32_t c = 0; c < 3; ++c)
        normalized[c][r] = float(part.tensor[r][c]/scale);
    const ::physx::PxVec3T<float> values = PxMassProperties::getMassSpaceInertia(normalized, axes);
    for (uint32_t i = 0; i < 3; ++i)
    {
        if (!positive(double(values[i])*scale)) return false;
        diagonal[i] = float(double(values[i])*scale);
    }
    return axes.isUnit();
}
void setTensor(Part& part, const PxVec3& diagonal, const PxQuat& axes)
{
    const PxMat33 rotation(axes);
    for (uint32_t r = 0; r < 3; ++r) for (uint32_t c = 0; c < 3; ++c)
    {
        part.tensor[r][c] = 0;
        for (uint32_t k = 0; k < 3; ++k)
            part.tensor[r][c] += double(rotation[k][r])*diagonal[k]*rotation[k][c];
    }
}
void shift(Part& part, const double* destination)
{
    double delta[3], squared = 0;
    for (uint32_t i = 0; i < 3; ++i) { delta[i] = part.center[i]-destination[i]; squared += delta[i]*delta[i]; }
    for (uint32_t r = 0; r < 3; ++r) for (uint32_t c = 0; c < 3; ++c)
        part.tensor[r][c] += part.mass*((r == c ? squared : 0)-delta[r]*delta[c]);
    std::copy_n(destination, 3, part.center);
}
bool overridePart(Part& part, const MassOverrides& authored)
{
    if (authored.flags & ~15u) return false;
    if (authored.flags & 1u)
    {
        if (!positive(authored.mass)) return false;
        const double ratio = authored.mass/part.mass;
        for (double (&row)[3] : part.tensor) for (double& value : row) value *= ratio;
        part.mass = authored.mass;
    }
    if (authored.flags & 2u)
    {
        if (!authored.center.isFinite()) return false;
        const double center[] = {authored.center.x, authored.center.y, authored.center.z};
        shift(part, center);
    }
    PxVec3 diagonal;
    PxQuat axes;
    if (!diagonalize(part, diagonal, axes)) return false;
    if (authored.flags & 4u)
    {
        for (uint32_t i = 0; i < 3; ++i) if (!positive(authored.diagonal[i])) return false;
        diagonal = authored.diagonal;
    }
    if (authored.flags & 8u)
    {
        if (!authored.axes.isUnit()) return false;
        axes = authored.axes;
    }
    if (authored.flags & 12u) setTensor(part, diagonal, axes);
    return true;
}
bool meshPart(const GeneralMassShape& shape, Part& part)
{
    if (!shape.points || !shape.triangles || shape.pointCount < 4 || shape.pointCount > 1000000 ||
        shape.triangleCount < 4 || shape.triangleCount > 2000000) return false;
    struct Edge { uint32_t face, from, to, count; };
    std::map<std::pair<uint32_t,uint32_t>, Edge> edges;
    std::vector<uint32_t> parent(shape.triangleCount);
    for (uint32_t i = 0; i < parent.size(); ++i) parent[i] = i;
    struct Root
    {
        std::vector<uint32_t>& parent;
        uint32_t operator()(uint32_t i) const
        {
            while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; }
            return i;
        }
    } root{parent};
    PxBounds3 local = PxBounds3::empty();
    for (uint32_t i = 0; i < shape.pointCount; ++i) if (!shape.points[i].isFinite()) return false;
    for (uint32_t face = 0; face < shape.triangleCount; ++face)
    {
        const unsigned int * ids = shape.triangles+face*3;
        for (uint32_t k = 0; k < 3; ++k)
        {
            const unsigned int a = ids[k], b = ids[(k+1)%3];
            if (a >= shape.pointCount || b >= shape.pointCount || a == b) return false;
            local.include(shape.points[a]);
            const std::pair<unsigned int, unsigned int> key = std::make_pair(std::min(a,b), std::max(a,b));
            const std::pair<std::map<std::pair<uint32_t, uint32_t>, Edge>::iterator, bool> inserted = edges.emplace(key, Edge{face,a,b,1});
            if (!inserted.second)
            {
                Edge& edge = inserted.first->second;
                if (edge.count != 1 || edge.from != b || edge.to != a) return false;
                ++edge.count; parent[root(face)] = root(edge.face);
            }
        }
    }
    for (const std::pair<const std::pair<unsigned int, unsigned int>, Edge> & edge : edges) if (edge.second.count != 2) return false;
    for (uint32_t i = 1; i < parent.size(); ++i) if (root(i) != root(0)) return false;
    double origin[3];
    for (uint32_t i = 0; i < 3; ++i) origin[i] = (double(local.minimum[i])+local.maximum[i])*0.5;
    double first[3]{}, second[3][3]{}, volume = 0;
    for (uint32_t face = 0; face < shape.triangleCount; ++face)
    {
        double p[3][3], sum[3];
        for (uint32_t k = 0; k < 3; ++k) for (uint32_t i = 0; i < 3; ++i)
            p[k][i] = double(shape.points[shape.triangles[face*3+k]][i])-origin[i];
        double ab[3], ac[3];
        for (uint32_t i = 0; i < 3; ++i) {ab[i]=p[1][i]-p[0][i]; ac[i]=p[2][i]-p[0][i]; sum[i]=p[0][i]+p[1][i]+p[2][i];}
        const double cross[] = {ab[1]*ac[2]-ab[2]*ac[1], ab[2]*ac[0]-ab[0]*ac[2], ab[0]*ac[1]-ab[1]*ac[0]};
        if (cross[0]*cross[0]+cross[1]*cross[1]+cross[2]*cross[2] == 0) return false;
        const double v = (p[0][0]*(p[1][1]*p[2][2]-p[1][2]*p[2][1]) +
                          p[0][1]*(p[1][2]*p[2][0]-p[1][0]*p[2][2]) +
                          p[0][2]*(p[1][0]*p[2][1]-p[1][1]*p[2][0]))/6;
        volume += v;
        for (uint32_t r = 0; r < 3; ++r)
        {
            first[r] += v*sum[r]/4;
            for (uint32_t c = 0; c < 3; ++c)
                second[r][c] += v*(sum[r]*sum[c]+p[0][r]*p[0][c]+p[1][r]*p[1][c]+p[2][r]*p[2][c])/20;
        }
    }
    if (!positive(std::abs(volume)) || !positive(std::abs(volume)*shape.density)) return false;
    const double sign = volume < 0 ? -1 : 1;
    volume *= sign;
    double covariance[3][3], center[3], trace = 0;
    for (uint32_t r = 0; r < 3; ++r) center[r] = sign*first[r]/volume;
    for (uint32_t r = 0; r < 3; ++r) for (uint32_t c = 0; c < 3; ++c)
        covariance[r][c] = sign*second[r][c]-volume*center[r]*center[c];
    for (uint32_t r = 0; r < 3; ++r) trace += covariance[r][r];
    part.mass = volume*shape.density; part.volume = volume; part.bounds = local;
    for (uint32_t r = 0; r < 3; ++r)
    {
        part.center[r] = center[r]+origin[r];
        for (uint32_t c = 0; c < 3; ++c) part.tensor[r][c] = shape.density*((r==c ? trace : 0)-covariance[r][c]);
    }
    return true;
}
}

bool computeGeneralMass(const GeneralMassShape* shapes, uint32_t count, const MassOverrides& body,
                        MassResult& output, uint32_t& invalidShape)
{
    using namespace ::physx;
    invalidShape = UINT32_MAX;
    if (!shapes || !count || count > 65536) return false;
    std::vector<Part> parts;
    parts.reserve(count);
    Part total;
    for (uint32_t i = 0; i < count; ++i)
    {
        invalidShape = i;
        const omni::physx::authoring::GeneralMassShape & shape = shapes[i];
        if (!shape.pose.isValid() || !shape.pose.q.isUnit() || !positive(shape.density)) return false;
        Part part;
        if (shape.kind == 3)
        {
            if (!meshPart(shape, part)) return false;
        }
        else
        {
            MassShape primitive{PrimitiveKind(shape.kind),shape.dimensions,PxTransform(PxIdentity),shape.density};
            MassResult result;
            uint32_t bad;
            if (!computePrimitiveMass(&primitive,1,result,bad)) return false;
            part.mass = result.mass; part.volume = result.volume; part.bounds = result.bounds;
            setTensor(part,result.diagonalInertia,result.principalAxes);
        }
        if (!overridePart(part,shape.overrides)) return false;
        const PxMat33 rotation(shape.pose.q);
        double center[3]{}, tensor[3][3]{};
        for (uint32_t r = 0; r < 3; ++r)
        {
            center[r] = shape.pose.p[r];
            for (uint32_t k = 0; k < 3; ++k) center[r] += double(rotation[k][r])*part.center[k];
            for (uint32_t c = 0; c < 3; ++c) for (uint32_t k = 0; k < 3; ++k) for (uint32_t l = 0; l < 3; ++l)
                tensor[r][c] += double(rotation[k][r])*part.tensor[k][l]*rotation[l][c];
        }
        std::copy_n(center,3,part.center);
        for (uint32_t r = 0; r < 3; ++r) std::copy_n(tensor[r],3,part.tensor[r]);
        part.bounds = PxBounds3::empty();
        if (shape.kind == 3)
        {
            for (uint32_t j = 0; j < shape.triangleCount*3; ++j)
                part.bounds.include(shape.pose.transform(shape.points[shape.triangles[j]]));
        }
        else
        {
            MassShape primitive{PrimitiveKind(shape.kind),shape.dimensions,shape.pose,shape.density};
            MassResult bounds; uint32_t bad;
            if (!computePrimitiveMass(&primitive,1,bounds,bad)) return false;
            part.bounds = bounds.bounds;
        }
        if (!part.bounds.isFinite()) return false;
        parts.push_back(part); total.mass += part.mass; total.volume += part.volume;
        total.bounds.include(part.bounds);
        for (uint32_t r = 0; r < 3; ++r) total.center[r] += part.mass*part.center[r];
    }
    invalidShape = UINT32_MAX;
    if (!positive(total.mass) || !positive(total.volume)) return false;
    for (double& value : total.center) value /= total.mass;
    for (omni::physx::authoring::Part & part : parts)
    {
        shift(part,total.center);
        for (uint32_t r = 0; r < 3; ++r) for (uint32_t c = 0; c < 3; ++c) total.tensor[r][c] += part.tensor[r][c];
    }
    if (!overridePart(total,body)) return false;
    MassResult result;
    if (!diagonalize(total,result.diagonalInertia,result.principalAxes)) return false;
    result.mass = float(total.mass); result.volume = float(total.volume); result.bounds = total.bounds;
    for (uint32_t i = 0; i < 3; ++i)
    {
        if (!std::isfinite(total.center[i]) || std::abs(total.center[i]) > std::numeric_limits<float>::max()) return false;
        result.centerOfMass[i] = float(total.center[i]);
    }
    output = result;
    return true;
}
}
