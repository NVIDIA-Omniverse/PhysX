// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-AUTHORING-MASS-001
// @covers AC-1 AC-2 AC-3

#include <omni/physx/AuthoringMass.h>
#include <extensions/PxMassProperties.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace omni::physx::authoring
{
namespace
{
struct Item
{
    double mass;
    double inertia[3];
};
constexpr double kPi = 3.14159265358979323846;
bool positiveFloat(double value)
{
    return std::isfinite(value) && value >= std::numeric_limits<float>::min() &&
           value <= std::numeric_limits<float>::max();
}
}

bool computePrimitiveMass(const MassShape* shapes, uint32_t count, MassResult& output,
                          uint32_t& invalidShape)
{
    using namespace ::physx;
    invalidShape = UINT32_MAX;
    if (!shapes || count == 0)
        return false;
    // Integrate in double precision before narrowing. This keeps malformed or
    // extreme inputs away from assertions inside the float PhysX math helpers.
    std::vector<Item> items;
    items.reserve(count);
    double totalMass = 0, volume = 0, weighted[3] = {};
    double lower[3] = {INFINITY, INFINITY, INFINITY};
    double upper[3] = {-INFINITY, -INFINITY, -INFINITY};
    for (uint32_t i = 0; i < count; ++i)
    {
        invalidShape = i;
        const omni::physx::authoring::MassShape & s = shapes[i];
        if (!s.dimensions.isFinite() || !s.pose.p.isFinite() || !s.pose.q.isUnit() ||
            !std::isfinite(s.density) || s.density <= 0)
            return false;
        const double x = s.dimensions.x, y = s.dimensions.y, z = s.dimensions.z;
        Item item{};
        double v = 0;
        switch (s.kind)
        {
        case PrimitiveKind::Box:
            if (x <= 0 || y <= 0 || z <= 0) return false;
            v = 8*x*y*z;
            item.inertia[0] = v*(y*y+z*z)/3;
            item.inertia[1] = v*(x*x+z*z)/3;
            item.inertia[2] = v*(x*x+y*y)/3;
            break;
        case PrimitiveKind::Sphere:
            if (x <= 0) return false;
            v = 4*kPi*x*x*x/3;
            std::fill_n(item.inertia, 3, 0.4*v*x*x);
            break;
        case PrimitiveKind::Capsule:
            if (x <= 0 || y < 0) return false;
            v = (4*x/3+2*y)*kPi*x*x;
            item.inertia[0] = (8*x*x*x/15+y*x*x)*kPi*x*x;
            item.inertia[1] = item.inertia[2] =
                (8*x*x*x/15+1.5*y*x*x+4*y*y*x/3+2*y*y*y/3)*kPi*x*x;
            break;
        default: return false;
        }
        item.mass = v*s.density;
        if (!positiveFloat(item.mass) || !positiveFloat(v)) return false;
        for (double & inertia : item.inertia)
        {
            inertia *= s.density;
            if (!positiveFloat(inertia)) return false;
        }
        items.push_back(item);
        totalMass += item.mass;
        volume += v;
        const PxMat33 rotation(s.pose.q);
        for (uint32_t axis = 0; axis < 3; ++axis)
        {
            weighted[axis] += item.mass*s.pose.p[axis];
            double extent = x;
            if (s.kind == PrimitiveKind::Box)
                extent = std::abs(double(rotation[0][axis]))*x +
                         std::abs(double(rotation[1][axis]))*y + std::abs(double(rotation[2][axis]))*z;
            else if (s.kind == PrimitiveKind::Capsule)
                extent += std::abs(double(rotation[0][axis]))*y;
            lower[axis] = std::min(lower[axis], double(s.pose.p[axis])-extent);
            upper[axis] = std::max(upper[axis], double(s.pose.p[axis])+extent);
        }
    }
    invalidShape = UINT32_MAX;
    if (!positiveFloat(totalMass) || !positiveFloat(volume)) return false;
    double center[3];
    for (uint32_t axis = 0; axis < 3; ++axis) center[axis] = weighted[axis]/totalMass;
    double tensor[3][3] = {};
    for (uint32_t i = 0; i < count; ++i)
    {
        const PxMat33 rotation(shapes[i].pose.q);
        double delta[3];
        for (uint32_t axis = 0; axis < 3; ++axis) delta[axis] = shapes[i].pose.p[axis]-center[axis];
        const double squared = delta[0]*delta[0]+delta[1]*delta[1]+delta[2]*delta[2];
        for (uint32_t row = 0; row < 3; ++row)
            for (uint32_t col = 0; col < 3; ++col)
            {
                for (uint32_t k = 0; k < 3; ++k)
                    tensor[row][col] += double(rotation[k][row])*items[i].inertia[k]*rotation[k][col];
                tensor[row][col] += items[i].mass*((row == col ? squared : 0)-delta[row]*delta[col]);
            }
    }
    double scale = 0;
    for (const double (&row)[3] : tensor) for (double value : row) scale = std::max(scale, std::abs(value));
    if (!positiveFloat(scale)) return false;
    PxMat33 normalized(PxZero);
    for (uint32_t row = 0; row < 3; ++row)
        for (uint32_t col = 0; col < 3; ++col)
            normalized[col][row] = float(tensor[row][col]/scale);
    MassResult result;
    result.mass = float(totalMass);
    result.volume = float(volume);
    const PxVec3 diagonal = PxMassProperties::getMassSpaceInertia(normalized, result.principalAxes);
    for (uint32_t axis = 0; axis < 3; ++axis)
    {
        const double inertia = double(diagonal[axis])*scale;
        if (!positiveFloat(inertia) || std::abs(lower[axis]) > std::numeric_limits<float>::max() ||
            std::abs(upper[axis]) > std::numeric_limits<float>::max()) return false;
        result.diagonalInertia[axis] = float(inertia);
        result.centerOfMass[axis] = float(center[axis]);
        result.bounds.minimum[axis] = float(lower[axis]);
        result.bounds.maximum[axis] = float(upper[axis]);
    }
    output = result;
    return true;
}
}
