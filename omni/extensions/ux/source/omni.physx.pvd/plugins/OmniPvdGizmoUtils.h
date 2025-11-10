// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

namespace OmniPvdGizmoUtils
{
struct OmniPvdJointType
{
    enum Enum
    {
        PxJoint,
        PxFixedJoint,
        PxPrismaticJoint,
        PxRevoluteJoint,
        PxSphericalJoint,
        PxDistanceJoint,
        PxGearJoint,
        PxRackAndPinionJoint,
        PxD6Joint,
        UndefinedJointType
    };
};

struct OmniPvdActorType
{
    enum Enum
    {
        PxActor,
        PxRigidDynamic,
        PxRigidStatic,
        PxArticulationLink,
        UndefinedActorType
    };
};

struct OmniPvdGeometricSelectableType
{
    enum Enum
    {
        PxGeomBox,
        PxGeomCapsule,
        PxGeomPlane,
        PxGeomSphere,
        PxBoxGeometry,
        PxCapsuleGeometry,
        PxPlaneGeometry,
        PxSphereGeometry,
        convexmesh_ref,
        heightfield_ref,
        trianglemesh_ref,
        UndefinedGeometricSelectableType
    };
};

OmniPvdJointType::Enum resolveJointType(std::string input);
OmniPvdActorType::Enum resolveActorType(std::string input);
OmniPvdGeometricSelectableType::Enum resolveGeometricSelectableType(std::string input);
} // namespace OmniPvdGizmoUtils
