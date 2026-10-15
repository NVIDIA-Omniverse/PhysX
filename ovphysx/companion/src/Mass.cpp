// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-MASS-001
// @covers AC-1 AC-2 AC-3
// @implements REQ-CAPI-COMPANION-001
// @covers AC-1
#include "ovphysx_companion/mass.h"
#include <omni/physx/AuthoringGeneralMass.h>
#include <vector>
namespace
{
using namespace omni::physx::authoring;
using ::physx::PxVec3;
using ::physx::PxQuat;
MassOverrides overrides(const ovphysx_companion_mass_overrides_t& value)
{
    return {value.flags,value.mass,PxVec3(value.center[0],value.center[1],value.center[2]),
        PxVec3(value.diagonal[0],value.diagonal[1],value.diagonal[2]),
        PxQuat(value.axes[0],value.axes[1],value.axes[2],value.axes[3])};
}
ovphysx_companion_status_t compute(const ovphysx_companion_general_mass_shape_t* input,uint32_t count,
    uint32_t bodySize,const ovphysx_companion_mass_overrides_t* body,uint32_t outputSize,
    ovphysx_companion_mass_result_t* output,uint32_t* invalid) noexcept
try
{
    if (!invalid) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    *invalid=UINT32_MAX;
    if (!input || !count || count>65536 || !body || !output || bodySize!=sizeof(*body) || outputSize!=sizeof(*output))
        return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    std::vector<GeneralMassShape> shapes(count);
    std::vector<std::vector<PxVec3>> points(count);
    for (uint32_t i=0;i<count;++i)
    {
        *invalid=i;
        const ovphysx_companion_general_mass_shape_t & source=input[i];omni::physx::authoring::GeneralMassShape & shape=shapes[i];
        if (source.size!=sizeof(source) || source.kind>3) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        shape.kind=source.kind;shape.density=source.density;shape.overrides=overrides(source.overrides);
        shape.dimensions=PxVec3(source.dimensions[0],source.dimensions[1],source.dimensions[2]);
        shape.pose.p=PxVec3(source.position[0],source.position[1],source.position[2]);
        shape.pose.q=PxQuat(source.rotation[0],source.rotation[1],source.rotation[2],source.rotation[3]);
        if (source.kind==3)
        {
            if (!source.points || !source.triangles || source.point_count<4 || source.point_count>1000000 ||
                source.triangle_count<4 || source.triangle_count>2000000) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
            points[i].reserve(source.point_count);
            for(uint32_t j=0;j<source.point_count;++j)
                points[i].emplace_back(source.points[3*j],source.points[3*j+1],source.points[3*j+2]);
            shape.points=points[i].data();shape.pointCount=source.point_count;
            shape.triangles=source.triangles;shape.triangleCount=source.triangle_count;
        }
    }
    MassResult result;
    if (!computeGeneralMass(shapes.data(),count,overrides(*body),result,*invalid)) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    ovphysx_companion_mass_result_t value{};
    value.size=sizeof(value);value.version=1;value.mass=result.mass;value.volume=result.volume;
    for (uint32_t i=0;i<3;++i)
    {
        value.center_of_mass[i]=result.centerOfMass[i];value.diagonal_inertia[i]=result.diagonalInertia[i];
        value.bounds_min[i]=result.bounds.minimum[i];value.bounds_max[i]=result.bounds.maximum[i];
    }
    value.principal_axes[0]=result.principalAxes.x;value.principal_axes[1]=result.principalAxes.y;
    value.principal_axes[2]=result.principalAxes.z;value.principal_axes[3]=result.principalAxes.w;
    *output=value;return OVPHYSX_COMPANION_OK;
}
catch (...) {return OVPHYSX_COMPANION_ERROR;}
}
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_mass_acquire(
    uint32_t version,uint32_t size,ovphysx_companion_mass_api_t* output)
{
    if (!output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if (version!=1 || size!=sizeof(*output)) return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output={sizeof(*output),1,7,compute};return OVPHYSX_COMPANION_OK;
}
