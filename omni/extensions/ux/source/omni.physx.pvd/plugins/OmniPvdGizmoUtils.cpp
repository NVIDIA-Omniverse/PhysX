// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "OmniPvdGizmoUtils.h"

using namespace OmniPvdGizmoUtils;

OmniPvdGizmoUtils::OmniPvdJointType::Enum OmniPvdGizmoUtils::resolveJointType(std::string input) {
    static const std::map<std::string, OmniPvdJointType::Enum> jointTypeMap {
        { "PxJoint", OmniPvdJointType::PxJoint },
        { "PxFixedJoint", OmniPvdJointType::PxFixedJoint },
        { "PxPrismaticJoint", OmniPvdJointType::PxPrismaticJoint },
        { "PxRevoluteJoint", OmniPvdJointType::PxRevoluteJoint },
        { "PxSphericalJoint", OmniPvdJointType::PxSphericalJoint },
        { "PxDistanceJoint", OmniPvdJointType::PxDistanceJoint },
        { "PxGearJoint", OmniPvdJointType::PxGearJoint },
        { "PxRackAndPinionJoint", OmniPvdJointType::PxRackAndPinionJoint },
        { "PxD6Joint", OmniPvdJointType::PxD6Joint }
    };

    auto itr = jointTypeMap.find(input);
    if( itr != jointTypeMap.end() ) {
        return itr->second;
    }
    return OmniPvdJointType::UndefinedJointType; 
}

OmniPvdGizmoUtils::OmniPvdActorType::Enum OmniPvdGizmoUtils::resolveActorType(std::string input) {
    static const std::map<std::string, OmniPvdActorType::Enum> actorTypeMap {
        { "PxActor", OmniPvdActorType::PxActor },
        { "PxRigidDynamic", OmniPvdActorType::PxRigidDynamic },
        { "PxRigidStatic", OmniPvdActorType::PxRigidStatic },
        { "PxArticulationLink", OmniPvdActorType::PxArticulationLink }
    };

    auto itr = actorTypeMap.find(input);
    if( itr != actorTypeMap.end() ) {
        return itr->second;
    }
    return OmniPvdActorType::UndefinedActorType; 
}

OmniPvdGizmoUtils::OmniPvdGeometricSelectableType::Enum OmniPvdGizmoUtils::resolveGeometricSelectableType(std::string input) {
    static const std::map<std::string, OmniPvdGeometricSelectableType::Enum> geometricTypeMap {
        { "PxGeomBox", OmniPvdGeometricSelectableType::PxGeomBox },
        { "PxGeomCapsule", OmniPvdGeometricSelectableType::PxGeomCapsule },
        { "PxGeomPlane", OmniPvdGeometricSelectableType::PxGeomPlane },
        { "PxGeomSphere", OmniPvdGeometricSelectableType::PxGeomSphere },
        { "PxBoxGeometry", OmniPvdGeometricSelectableType::PxBoxGeometry },
        { "PxCapsuleGeometry", OmniPvdGeometricSelectableType::PxCapsuleGeometry },
        { "PxPlaneGeometry", OmniPvdGeometricSelectableType::PxPlaneGeometry },
        { "PxSphereGeometry", OmniPvdGeometricSelectableType::PxSphereGeometry },
        { "PxSphereGeometry", OmniPvdGeometricSelectableType::PxSphereGeometry },
        { "convexmesh_ref", OmniPvdGeometricSelectableType::convexmesh_ref },
        { "heightfield_ref", OmniPvdGeometricSelectableType::heightfield_ref },
        { "trianglemesh_ref", OmniPvdGeometricSelectableType::trianglemesh_ref }
    };

    auto itr = geometricTypeMap.find(input);
    if( itr != geometricTypeMap.end() ) {
        return itr->second;
    }
    return OmniPvdGeometricSelectableType::UndefinedGeometricSelectableType; 
}

