// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "OmniPvdOvdParserConfig.h"
#include "OmniPvdDomUtils.h"

#include "omnipvd/PxOmniPvd.h"

void initClassConfigMap(std::unordered_map<std::string, OmniPvdClassConfig*> &classConfigMap)
{
    classConfigMap["PxSceneFlag"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxSceneFlag, OmniPvdUsdClassEnum::eUSDClassEnum);
    classConfigMap["PxFrictionType"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxSceneFlag, OmniPvdUsdClassEnum::eUSDClassEnum);
    classConfigMap["PxActorType"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxSceneFlag, OmniPvdUsdClassEnum::eUSDClassEnum);

    classConfigMap["PxScene"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxScene, OmniPvdUsdClassEnum::eUSDClassGeomScope);
    classConfigMap["PxMaterial"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxMaterial, OmniPvdUsdClassEnum::eUSDClassGeomScope);
    classConfigMap["PxArticulationReducedCoordinate"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxArticulation, OmniPvdUsdClassEnum::eUSDClassGeomScope);
    classConfigMap["PxArticulationJointReducedCoordinate"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxArticulationJoint, OmniPvdUsdClassEnum::eUSDClassGeomScope);

    ////////////////////////////////////////////////////////////////////////////////
    // All deriving from PxActor
    ////////////////////////////////////////////////////////////////////////////////
    classConfigMap["PxActor"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxActor, OmniPvdUsdClassEnum::eUSDClassXform);
    classConfigMap["PxRigidActor"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxActor, OmniPvdUsdClassEnum::eUSDClassXform);
    classConfigMap["PxRigidStatic"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxActor, OmniPvdUsdClassEnum::eUSDClassXform);
    classConfigMap["PxRigidDynamic"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxActor, OmniPvdUsdClassEnum::eUSDClassXform);
    classConfigMap["PxArticulationLink"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxActor, OmniPvdUsdClassEnum::eUSDClassXform);
    classConfigMap["PxPBDParticleSystem"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxActor, OmniPvdUsdClassEnum::eUSDClassXform);
    
    classConfigMap["PxShape"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxShape, OmniPvdUsdClassEnum::eUSDClassXform);

    classConfigMap["PxSphereGeometry"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxGeomSphere, OmniPvdUsdClassEnum::eUSDClassGeomSphere);
    classConfigMap["PxCapsuleGeometry"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxGeomCapsule, OmniPvdUsdClassEnum::eUSDClassGeomCapsule);
    classConfigMap["PxBoxGeometry"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxGeomBox, OmniPvdUsdClassEnum::eUSDClassGeomCube);
    classConfigMap["PxPlaneGeometry"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxGeomPlane, OmniPvdUsdClassEnum::eUSDClassGeomPlane);
    classConfigMap["PxConvexMeshGeometry"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxGeomConvexMesh, OmniPvdUsdClassEnum::eUSDClassXform);
    classConfigMap["PxHeightFieldGeometry"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxGeomHeightfield, OmniPvdUsdClassEnum::eUSDClassXform);
    classConfigMap["PxTriangleMeshGeometry"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxGeomTriangleMesh, OmniPvdUsdClassEnum::eUSDClassXform);

    classConfigMap["PxCustomGeometryExtCylinderCallbacks"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxCustomGeometryCylinder, OmniPvdUsdClassEnum::eUSDClassCylinder);
    classConfigMap["PxCustomGeometryExtConeCallbacks"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxCustomGeometryCone, OmniPvdUsdClassEnum::eUSDClassCone);

    classConfigMap["PxConvexCoreCylinder"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxConvexCoreCylinder, OmniPvdUsdClassEnum::eUSDClassCylinder);
    classConfigMap["PxConvexCoreCone"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxConvexCoreCone, OmniPvdUsdClassEnum::eUSDClassCone);

    classConfigMap["PxConvexMesh"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxConvexMesh, OmniPvdUsdClassEnum::eUSDClassGeomMesh);
    classConfigMap["PxHeightField"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxHeightfield, OmniPvdUsdClassEnum::eUSDClassGeomMesh);
    classConfigMap["PxTriangleMesh"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxTriangleMesh, OmniPvdUsdClassEnum::eUSDClassGeomMesh);

    classConfigMap["PxPhysics"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxUndefined, OmniPvdUsdClassEnum::eUSDClassGeomScope);
    classConfigMap["PxAggregate"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxUndefined, OmniPvdUsdClassEnum::eUSDClassGeomScope);

    ////////////////////////////////////////////////////////////////////////////////
    // All deriving from PxJoint
    ////////////////////////////////////////////////////////////////////////////////
    classConfigMap["PxFixedJoint"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxJoint, OmniPvdUsdClassEnum::eUSDClassGeomScope);
    classConfigMap["PxPrismaticJoint"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxJoint, OmniPvdUsdClassEnum::eUSDClassGeomScope);
    classConfigMap["PxRevoluteJoint"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxJoint, OmniPvdUsdClassEnum::eUSDClassGeomScope);
    classConfigMap["PxSphericalJoint"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxJoint, OmniPvdUsdClassEnum::eUSDClassGeomScope);
    classConfigMap["PxDistanceJoint"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxJoint, OmniPvdUsdClassEnum::eUSDClassGeomScope);
    classConfigMap["PxGearJoint"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxJoint, OmniPvdUsdClassEnum::eUSDClassGeomScope);
    classConfigMap["PxRackAndPinionJoint"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxJoint, OmniPvdUsdClassEnum::eUSDClassGeomScope);
    classConfigMap["PxD6Joint"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxJoint, OmniPvdUsdClassEnum::eUSDClassGeomScope);

    ////////////////////////////////////////////////////////////////////////////////
    // All deriving from PxParticleBuffer
    ////////////////////////////////////////////////////////////////////////////////
    classConfigMap["PxParticleBuffer"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxParticleBuffer, OmniPvdUsdClassEnum::eUSDClassGeomPoints);
    classConfigMap["PxParticleAndDiffuseBuffer"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxParticleBuffer, OmniPvdUsdClassEnum::eUSDClassGeomPoints);
    classConfigMap["PxParticleClothBuffer"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxParticleBuffer, OmniPvdUsdClassEnum::eUSDClassGeomPoints);
    classConfigMap["PxParticleRigidBuffer"] = new OmniPvdClassConfig(OmniPvdPhysXClassEnum::ePxParticleBuffer, OmniPvdUsdClassEnum::eUSDClassGeomPoints);
}

void initAttributeConfigMap(std::unordered_map<std::string, OmniPvdUsdAttributeEnum> &attributeConfigMap)
{
    ////////////////////////////////////////////////////////////////////////////////
    // PxScene
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxScene.flags"] = OmniPvdUsdAttributeEnum::eUSDAttributeEnum;
    attributeConfigMap["PxScene.frictionType"] = OmniPvdUsdAttributeEnum::eUSDAttributeEnum;
    //attributeConfigMap["scene.gravity"] = custom?;
    //attributeConfigMap["scene.actors"] = not displayed in USD;

    ////////////////////////////////////////////////////////////////////////////////
    // PxActor
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxActor.type"] = OmniPvdUsdAttributeEnum::eUSDAttributeEnum;

    ////////////////////////////////////////////////////////////////////////////////
    // PxRigidActor
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxRigidActor.globalPose"] = OmniPvdUsdAttributeEnum::eUSDAttributeTransformFork;
    // deprecate, kept for backards compatibility
    attributeConfigMap["PxRigidActor.translation"] = OmniPvdUsdAttributeEnum::eUSDAttributeTranslateOp;
    attributeConfigMap["PxRigidActor.rotation"] = OmniPvdUsdAttributeEnum::eUSDAttributeRotationOp;

    ////////////////////////////////////////////////////////////////////////////////
    // PxShape
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxShape.localPose"] = OmniPvdUsdAttributeEnum::eUSDAttributeTransformFork;
    // deprecate, kept for backards compatibility
    attributeConfigMap["PxShape.translation"] = OmniPvdUsdAttributeEnum::eUSDAttributeTranslateOp;
    attributeConfigMap["PxShape.rotation"] = OmniPvdUsdAttributeEnum::eUSDAttributeRotationOp;

    ////////////////////////////////////////////////////////////////////////////////
    // PxSphereGeometry
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxSphereGeometry.radius"] = OmniPvdUsdAttributeEnum::eUSDAttributeRadius;

    ////////////////////////////////////////////////////////////////////////////////
    // PxCapsuleGeometry
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxCapsuleGeometry.halfHeight"] = OmniPvdUsdAttributeEnum::eUSDAttributeHeight;
    attributeConfigMap["PxCapsuleGeometry.radius"] = OmniPvdUsdAttributeEnum::eUSDAttributeRadius;
    // axis is always X in PhysX, so don't use the value from the OVD object if it has any axis defined

    ////////////////////////////////////////////////////////////////////////////////
    // PxBoxGeometry
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxBoxGeometry.halfExtents"] = OmniPvdUsdAttributeEnum::eUSDAttributeScaleOp;

    ////////////////////////////////////////////////////////////////////////////////
    // PxConvexMeshGeometry
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxConvexMeshGeometry.scale"] = OmniPvdUsdAttributeEnum::eUSDAttributeScaleOp;

    ////////////////////////////////////////////////////////////////////////////////
    // PxHeightfieldGeometry
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxHeightFieldGeometry.scale"] = OmniPvdUsdAttributeEnum::eUSDAttributeScaleOp;

    ////////////////////////////////////////////////////////////////////////////////
    // PxTriangleMeshGeometry
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxTriangleMeshGeometry.scale"] = OmniPvdUsdAttributeEnum::eUSDAttributeScaleOp;

    ////////////////////////////////////////////////////////////////////////////////
    // PxConvexMesh
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxConvexMesh.verts"] = OmniPvdUsdAttributeEnum::eUSDAttributeVerts;
    attributeConfigMap["PxConvexMesh.tris"] = OmniPvdUsdAttributeEnum::eUSDAttributeTris;

    ////////////////////////////////////////////////////////////////////////////////
    // PxHeightfield
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxHeightField.verts"] = OmniPvdUsdAttributeEnum::eUSDAttributeVerts;
    attributeConfigMap["PxHeightField.tris"] = OmniPvdUsdAttributeEnum::eUSDAttributeTris;

    ////////////////////////////////////////////////////////////////////////////////
    // PxTriangleMesh
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxTriangleMesh.verts"] = OmniPvdUsdAttributeEnum::eUSDAttributeVerts;
    attributeConfigMap["PxTriangleMesh.tris"] = OmniPvdUsdAttributeEnum::eUSDAttributeTris;

    ////////////////////////////////////////////////////////////////////////////////
    // PxCustomGeometryExtCylinderCallbacks
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxCustomGeometryExtCylinderCallbacks.height"] = OmniPvdUsdAttributeEnum::eUSDAttributeHeight;
    attributeConfigMap["PxCustomGeometryExtCylinderCallbacks.radius"] = OmniPvdUsdAttributeEnum::eUSDAttributeRadius;
    attributeConfigMap["PxCustomGeometryExtCylinderCallbacks.axis"] = OmniPvdUsdAttributeEnum::eUSDAttributeAxis;

    ////////////////////////////////////////////////////////////////////////////////
    // PxCustomGeometryExtConeCallbacks
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxCustomGeometryExtConeCallbacks.height"] = OmniPvdUsdAttributeEnum::eUSDAttributeHeight;
    attributeConfigMap["PxCustomGeometryExtConeCallbacks.radius"] = OmniPvdUsdAttributeEnum::eUSDAttributeRadius;
    attributeConfigMap["PxCustomGeometryExtConeCallbacks.axis"] = OmniPvdUsdAttributeEnum::eUSDAttributeAxis;

    ////////////////////////////////////////////////////////////////////////////////
    // PxConvexCoreCylinder
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxConvexCoreCylinder.height"] = OmniPvdUsdAttributeEnum::eUSDAttributeHeight;
    attributeConfigMap["PxConvexCoreCylinder.radius"] = OmniPvdUsdAttributeEnum::eUSDAttributeRadius;

    ////////////////////////////////////////////////////////////////////////////////
    // PxConvexCoreCone
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxConvexCoreCone.height"] = OmniPvdUsdAttributeEnum::eUSDAttributeHeight;
    attributeConfigMap["PxConvexCoreCone.radius"] = OmniPvdUsdAttributeEnum::eUSDAttributeRadius;

    ////////////////////////////////////////////////////////////////////////////////
    // PxParticleBuffer
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxParticleBuffer.positionInvMasses"] = OmniPvdUsdAttributeEnum::eUSDAttributePoints;
}

void initPvdDomState(OmniPvdDOMState &domState)
{
    domState.mNextInternalHandle = 1;
    //domState.mCurrentFrameId = 0;

    domState.mMinFrame = 10000000;
    domState.mMaxFrame = 0;

    domState.mSceneRootClass = createInternalClass("Scenes", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd, OmniPvdUsdClassEnum::eUSDClassGeomScope);

    domState.mRigidDynamicBranchClass = createInternalClass("RigidDynamic", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd, OmniPvdUsdClassEnum::eUSDClassGeomScope);
    domState.mRigidStaticBranchClass = createInternalClass("RigidStatic", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd, OmniPvdUsdClassEnum::eUSDClassGeomScope);
    domState.mArticulationBranchClass = createInternalClass("Articulations", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd, OmniPvdUsdClassEnum::eUSDClassGeomScope);
    domState.mParticleSystemBranchClass = createInternalClass("ParticleSystems", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd, OmniPvdUsdClassEnum::eUSDClassGeomScope);

    domState.mSharedRootClass = createInternalClass("Shared", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd, OmniPvdUsdClassEnum::eUSDClassGeomScope);

    domState.mSharedMaterialsClass = createInternalClass("Materials", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd, OmniPvdUsdClassEnum::eUSDClassGeomScope);

    domState.mSharedMeshesClass = createInternalClass("Meshes", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd, OmniPvdUsdClassEnum::eUSDClassGeomScope);
    domState.mSharedConvexMeshesClass = createInternalClass("ConvexMeshes", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd, OmniPvdUsdClassEnum::eUSDClassGeomScope);
    domState.mSharedHeightfieldsClass = createInternalClass("Heightfields", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd, OmniPvdUsdClassEnum::eUSDClassGeomScope);
    domState.mSharedTriangleMeshesClass = createInternalClass("TriangleMeshes", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd, OmniPvdUsdClassEnum::eUSDClassGeomScope);

    domState.mSharedShapesClass = createInternalClass("Shapes", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd, OmniPvdUsdClassEnum::eUSDClassGeomScope);

    domState.mAttributeNameClass = createInternalClass("attrib", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd, OmniPvdUsdClassEnum::eUSDClassGeomScope);
    domState.mAttributeRefClass = createInternalClass("object_ref", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd, OmniPvdUsdClassEnum::eUSDClassOver);

    domState.mSharedShapeRefClass = createInternalClass("shape_ref", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd, OmniPvdUsdClassEnum::eUSDClassOver);
    domState.mConvexMeshRefClass = createInternalClass("convexmesh_ref", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd, OmniPvdUsdClassEnum::eUSDClassOver);
    domState.mHeightfieldRefClass = createInternalClass("heightfield_ref", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd, OmniPvdUsdClassEnum::eUSDClassOver);
    domState.mTriangleMeshRefClass = createInternalClass("trianglemesh_ref", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd, OmniPvdUsdClassEnum::eUSDClassOver);


    ////////////////////////////////////////////////////////////////////////////////
    // Create the shared layer nodes, top node with Prim path "/shared" is invisible
    ////////////////////////////////////////////////////////////////////////////////
    // /shared                       : mSharedLayerRoot

    // /shared/materials             : mSharedMaterialsBranch

    // /shared/meshes                : mSharedMeshesBranch
    // /shared/meshes/convexmeshes   : mSharedConvexMeshesBranch
    // /shared/meshes/heightfields   : mSharedHeightfieldsBranch
    // /shared/meshes/trianglemeshes : mSharedTriangleMeshesBranch

    // /shared/shapes                : mSharedShapesBranch

    domState.mSharedLayerRoot = createInternalNode(domState.mObjectCreations, 0, domState.mSharedRootClass, 0, 1, 0);

    domState.mSharedMaterialsBranch = createInternalNode(domState.mObjectCreations, domState.mSharedLayerRoot, domState.mSharedMaterialsClass, 0, 1, 1);

    domState.mSharedMeshesBranch = createInternalNode(domState.mObjectCreations, domState.mSharedLayerRoot, domState.mSharedMeshesClass, 0, 1, 1);
    domState.mSharedConvexMeshesBranch = createInternalNode(domState.mObjectCreations, domState.mSharedMeshesBranch, domState.mSharedConvexMeshesClass, 0, 1, 1);
    domState.mSharedHeightfieldsBranch = createInternalNode(domState.mObjectCreations, domState.mSharedMeshesBranch, domState.mSharedHeightfieldsClass, 0, 1, 1);
    domState.mSharedTriangleMeshesBranch = createInternalNode(domState.mObjectCreations, domState.mSharedMeshesBranch, domState.mSharedTriangleMeshesClass, 0, 1, 1);

    domState.mSharedShapesBranch = createInternalNode(domState.mObjectCreations, domState.mSharedLayerRoot, domState.mSharedShapesClass, 0, 1, 1);

    ////////////////////////////////////////////////////////////////////////////////
    // Create the scener layer root nodes, top node with Prim path "/scenes" is visible
    ////////////////////////////////////////////////////////////////////////////////
    domState.mSceneLayerRoot = createInternalNode(domState.mObjectCreations, 0, domState.mSceneRootClass, 0, 0, 1);

    initClassConfigMap(domState.mClassConfigMap);
    initAttributeConfigMap(domState.mAttributeConfigMap);

    domState.mActorTypeEnumRigidDynamic = 0;

    domState.mOvdIntegrationVersionMajor = PX_PHYSICS_OVD_INTEGRATION_VERSION_MAJOR;

    domState.mStreamOvdIntegVersionMajor = 0;
    domState.mStreamOvdIntegVersionMinor = 0;

    domState.mPxSceneClass = nullptr;
    domState.mPxArticulationReducedCoordinateClass = nullptr;
    domState.mPxArticulationLinkClass = nullptr;
}

