// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.


// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "PvdDomParserConfig.h"
#include "PvdDomUtils.h"
#include "PvdDomLog.h"

void initClassConfigMap(std::unordered_map<std::string, OmniPvdPhysXClassEnum> &classConfigMap)
{
    classConfigMap["PxSceneFlag"] = OmniPvdPhysXClassEnum::ePxSceneFlag;
    classConfigMap["PxFrictionType"] = OmniPvdPhysXClassEnum::ePxUndefined;
    classConfigMap["PxActorType"] = OmniPvdPhysXClassEnum::ePxActor;

    classConfigMap["PxScene"] = OmniPvdPhysXClassEnum::ePxScene;
    classConfigMap["PxMaterial"] = OmniPvdPhysXClassEnum::ePxMaterial;
    classConfigMap["PxDeformableSurfaceMaterial"] = OmniPvdPhysXClassEnum::ePxMaterial;
    classConfigMap["PxDeformableVolumeMaterial"] = OmniPvdPhysXClassEnum::ePxMaterial;
    classConfigMap["PxPBDMaterial"] = OmniPvdPhysXClassEnum::ePxMaterial;
    classConfigMap["PxArticulationReducedCoordinate"] = OmniPvdPhysXClassEnum::ePxArticulation;
    classConfigMap["PxArticulationJointReducedCoordinate"] = OmniPvdPhysXClassEnum::ePxArticulationJoint;

    ////////////////////////////////////////////////////////////////////////////////
    // All deriving from PxActor
    ////////////////////////////////////////////////////////////////////////////////
    classConfigMap["PxActor"] = OmniPvdPhysXClassEnum::ePxActor;
    classConfigMap["PxRigidActor"] = OmniPvdPhysXClassEnum::ePxActor;
    classConfigMap["PxRigidStatic"] = OmniPvdPhysXClassEnum::ePxActor;
    classConfigMap["PxRigidDynamic"] = OmniPvdPhysXClassEnum::ePxActor;
    classConfigMap["PxArticulationLink"] = OmniPvdPhysXClassEnum::ePxActor;
    classConfigMap["PxPBDParticleSystem"] = OmniPvdPhysXClassEnum::ePxActor;
    classConfigMap["PxDeformableBody"] = OmniPvdPhysXClassEnum::ePxActor;
    classConfigMap["PxDeformableVolume"] = OmniPvdPhysXClassEnum::ePxDeformableVolume;
    classConfigMap["PxDeformableSurface"] = OmniPvdPhysXClassEnum::ePxDeformableSurface;

    classConfigMap["PxShape"] = OmniPvdPhysXClassEnum::ePxShape;

    classConfigMap["PxSphereGeometry"] = OmniPvdPhysXClassEnum::ePxGeomSphere;
    classConfigMap["PxCapsuleGeometry"] = OmniPvdPhysXClassEnum::ePxGeomCapsule;
    classConfigMap["PxBoxGeometry"] = OmniPvdPhysXClassEnum::ePxGeomBox;
    classConfigMap["PxPlaneGeometry"] = OmniPvdPhysXClassEnum::ePxGeomPlane;
    classConfigMap["PxConvexMeshGeometry"] = OmniPvdPhysXClassEnum::ePxGeomConvexMesh;
    classConfigMap["PxHeightFieldGeometry"] = OmniPvdPhysXClassEnum::ePxGeomHeightfield;
    classConfigMap["PxTriangleMeshGeometry"] = OmniPvdPhysXClassEnum::ePxGeomTriangleMesh;

    classConfigMap["PxCustomGeometryExtCylinderCallbacks"] = OmniPvdPhysXClassEnum::ePxCustomGeometryCylinder;
    classConfigMap["PxCustomGeometryExtConeCallbacks"] = OmniPvdPhysXClassEnum::ePxCustomGeometryCone;

    classConfigMap["PxConvexCoreCylinder"] = OmniPvdPhysXClassEnum::ePxConvexCoreCylinder;
    classConfigMap["PxConvexCoreCone"] = OmniPvdPhysXClassEnum::ePxConvexCoreCone;

    classConfigMap["PxConvexMesh"] = OmniPvdPhysXClassEnum::ePxConvexMesh;
    classConfigMap["PxHeightField"] = OmniPvdPhysXClassEnum::ePxHeightfield;
    classConfigMap["PxTriangleMesh"] = OmniPvdPhysXClassEnum::ePxTriangleMesh;
    classConfigMap["PxTetrahedronMeshGeometry"] = OmniPvdPhysXClassEnum::ePxGeomTetMesh;
    classConfigMap["PxTetrahedronMesh"] = OmniPvdPhysXClassEnum::ePxTetrahedronMesh;
    classConfigMap["PxDeformableVolumeMesh"] = OmniPvdPhysXClassEnum::ePxDeformableVolumeMesh;

    classConfigMap["PxPhysics"] = OmniPvdPhysXClassEnum::ePxUndefined;
    classConfigMap["PxAggregate"] = OmniPvdPhysXClassEnum::ePxUndefined;

    ////////////////////////////////////////////////////////////////////////////////
    // All deriving from PxJoint
    ////////////////////////////////////////////////////////////////////////////////
    classConfigMap["PxFixedJoint"] = OmniPvdPhysXClassEnum::ePxJoint;
    classConfigMap["PxPrismaticJoint"] = OmniPvdPhysXClassEnum::ePxJoint;
    classConfigMap["PxRevoluteJoint"] = OmniPvdPhysXClassEnum::ePxJoint;
    classConfigMap["PxSphericalJoint"] = OmniPvdPhysXClassEnum::ePxJoint;
    classConfigMap["PxDistanceJoint"] = OmniPvdPhysXClassEnum::ePxJoint;
    classConfigMap["PxGearJoint"] = OmniPvdPhysXClassEnum::ePxJoint;
    classConfigMap["PxRackAndPinionJoint"] = OmniPvdPhysXClassEnum::ePxJoint;
    classConfigMap["PxD6Joint"] = OmniPvdPhysXClassEnum::ePxJoint;

    ////////////////////////////////////////////////////////////////////////////////
    // All deriving from PxParticleBuffer
    ////////////////////////////////////////////////////////////////////////////////
    classConfigMap["PxParticleBuffer"] = OmniPvdPhysXClassEnum::ePxParticleBuffer;
    classConfigMap["PxParticleAndDiffuseBuffer"] = OmniPvdPhysXClassEnum::ePxParticleBuffer;
    classConfigMap["PxParticleRigidBuffer"] = OmniPvdPhysXClassEnum::ePxParticleBuffer;
}

void initAttributeConfigMap(std::unordered_map<std::string, PvdDomAttributeEnum> &attributeConfigMap)
{
    ////////////////////////////////////////////////////////////////////////////////
    // PxScene
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxScene.flags"] = PvdDomAttributeEnum::ePvdDomAttribEnum;
    attributeConfigMap["PxScene.frictionType"] = PvdDomAttributeEnum::ePvdDomAttribEnum;
    //attributeConfigMap["scene.gravity"] = custom?;
    //attributeConfigMap["scene.actors"] = not displayed in USD;

    ////////////////////////////////////////////////////////////////////////////////
    // PxActor
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxActor.type"] = PvdDomAttributeEnum::ePvdDomAttribEnum;

    ////////////////////////////////////////////////////////////////////////////////
    // PxRigidActor
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxRigidActor.globalPose"] = PvdDomAttributeEnum::ePvdDomAttribTransformFork;
    // deprecate, kept for backards compatibility
    attributeConfigMap["PxRigidActor.translation"] = PvdDomAttributeEnum::ePvdDomAttribTranslateOp;
    attributeConfigMap["PxRigidActor.rotation"] = PvdDomAttributeEnum::ePvdDomAttribRotationOp;

    ////////////////////////////////////////////////////////////////////////////////
    // PxShape
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxShape.localPose"] = PvdDomAttributeEnum::ePvdDomAttribTransformFork;
    // deprecate, kept for backards compatibility
    attributeConfigMap["PxShape.translation"] = PvdDomAttributeEnum::ePvdDomAttribTranslateOp;
    attributeConfigMap["PxShape.rotation"] = PvdDomAttributeEnum::ePvdDomAttribRotationOp;

    ////////////////////////////////////////////////////////////////////////////////
    // PxSphereGeometry
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxSphereGeometry.radius"] = PvdDomAttributeEnum::ePvdDomAttribRadius;

    ////////////////////////////////////////////////////////////////////////////////
    // PxCapsuleGeometry
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxCapsuleGeometry.halfHeight"] = PvdDomAttributeEnum::ePvdDomAttribHeight;
    attributeConfigMap["PxCapsuleGeometry.radius"] = PvdDomAttributeEnum::ePvdDomAttribRadius;
    // axis is always X in PhysX, so don't use the value from the OVD object if it has any axis defined

    ////////////////////////////////////////////////////////////////////////////////
    // PxBoxGeometry
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxBoxGeometry.halfExtents"] = PvdDomAttributeEnum::ePvdDomAttribScaleOp;

    ////////////////////////////////////////////////////////////////////////////////
    // PxConvexMeshGeometry
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxConvexMeshGeometry.scale"] = PvdDomAttributeEnum::ePvdDomAttribScaleOp;

    ////////////////////////////////////////////////////////////////////////////////
    // PxHeightfieldGeometry
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxHeightFieldGeometry.scale"] = PvdDomAttributeEnum::ePvdDomAttribScaleOp;

    ////////////////////////////////////////////////////////////////////////////////
    // PxTriangleMeshGeometry
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxTriangleMeshGeometry.scale"] = PvdDomAttributeEnum::ePvdDomAttribScaleOp;

    ////////////////////////////////////////////////////////////////////////////////
    // PxConvexMesh
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxConvexMesh.verts"] = PvdDomAttributeEnum::ePvdDomAttribVerts;
    attributeConfigMap["PxConvexMesh.tris"] = PvdDomAttributeEnum::ePvdDomAttribTris;

    ////////////////////////////////////////////////////////////////////////////////
    // PxHeightfield
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxHeightField.verts"] = PvdDomAttributeEnum::ePvdDomAttribVerts;
    attributeConfigMap["PxHeightField.tris"] = PvdDomAttributeEnum::ePvdDomAttribTris;

    ////////////////////////////////////////////////////////////////////////////////
    // PxTriangleMesh
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxTriangleMesh.verts"] = PvdDomAttributeEnum::ePvdDomAttribVerts;
    attributeConfigMap["PxTriangleMesh.tris"] = PvdDomAttributeEnum::ePvdDomAttribTris;

    ////////////////////////////////////////////////////////////////////////////////
    // PxCustomGeometryExtCylinderCallbacks
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxCustomGeometryExtCylinderCallbacks.height"] = PvdDomAttributeEnum::ePvdDomAttribHeight;
    attributeConfigMap["PxCustomGeometryExtCylinderCallbacks.radius"] = PvdDomAttributeEnum::ePvdDomAttribRadius;
    attributeConfigMap["PxCustomGeometryExtCylinderCallbacks.axis"] = PvdDomAttributeEnum::ePvdDomAttribAxis;

    ////////////////////////////////////////////////////////////////////////////////
    // PxCustomGeometryExtConeCallbacks
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxCustomGeometryExtConeCallbacks.height"] = PvdDomAttributeEnum::ePvdDomAttribHeight;
    attributeConfigMap["PxCustomGeometryExtConeCallbacks.radius"] = PvdDomAttributeEnum::ePvdDomAttribRadius;
    attributeConfigMap["PxCustomGeometryExtConeCallbacks.axis"] = PvdDomAttributeEnum::ePvdDomAttribAxis;

    ////////////////////////////////////////////////////////////////////////////////
    // PxConvexCoreCylinder
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxConvexCoreCylinder.height"] = PvdDomAttributeEnum::ePvdDomAttribHeight;
    attributeConfigMap["PxConvexCoreCylinder.radius"] = PvdDomAttributeEnum::ePvdDomAttribRadius;

    ////////////////////////////////////////////////////////////////////////////////
    // PxConvexCoreCone
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxConvexCoreCone.height"] = PvdDomAttributeEnum::ePvdDomAttribHeight;
    attributeConfigMap["PxConvexCoreCone.radius"] = PvdDomAttributeEnum::ePvdDomAttribRadius;

    ////////////////////////////////////////////////////////////////////////////////
    // PxParticleBuffer
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxParticleBuffer.positionInvMasses"] = PvdDomAttributeEnum::ePvdDomAttribPoints;

    ////////////////////////////////////////////////////////////////////////////////
    // PxTetrahedronMesh
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxTetrahedronMesh.verts"] = PvdDomAttributeEnum::ePvdDomAttribVerts;
    attributeConfigMap["PxTetrahedronMesh.tets"] = PvdDomAttributeEnum::ePvdDomAttribTets;
    attributeConfigMap["PxTetrahedronMesh.positions"] = PvdDomAttributeEnum::ePvdDomAttribDeformablePositions;
    attributeConfigMap["PxTetrahedronMesh.velocities"] = PvdDomAttributeEnum::ePvdDomAttribDeformableVelocities;

    ////////////////////////////////////////////////////////////////////////////////
    // PxTriangleMesh (animated for deformable surfaces)
    ////////////////////////////////////////////////////////////////////////////////
    attributeConfigMap["PxTriangleMesh.positions"] = PvdDomAttributeEnum::ePvdDomAttribDeformablePositions;
    attributeConfigMap["PxTriangleMesh.velocities"] = PvdDomAttributeEnum::ePvdDomAttribDeformableVelocities;
}

void initPvdDomState(OmniPvdDOMState &domState)
{
    domState.mNextInternalHandle = 1;
    //domState.mCurrentFrameId = 0;

    domState.mMinFrame = 10000000;
    domState.mMaxFrame = 0;

    domState.mSceneRootClass = createInternalClass("Scenes", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);

    domState.mRigidDynamicBranchClass = createInternalClass("RigidDynamic", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);
    domState.mRigidStaticBranchClass = createInternalClass("RigidStatic", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);
    domState.mArticulationBranchClass = createInternalClass("Articulations", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);
    domState.mParticleSystemBranchClass = createInternalClass("ParticleSystems", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);
    domState.mDeformableVolumeBranchClass = createInternalClass("DeformableVolumes", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);
    domState.mDeformableSurfaceBranchClass = createInternalClass("DeformableSurfaces", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);

    domState.mSharedRootClass = createInternalClass("Shared", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);

    domState.mSharedMaterialsClass = createInternalClass("Materials", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);

    domState.mSharedMeshesClass = createInternalClass("Meshes", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);
    domState.mSharedConvexMeshesClass = createInternalClass("ConvexMeshes", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);
    domState.mSharedHeightfieldsClass = createInternalClass("Heightfields", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);
    domState.mSharedTriangleMeshesClass = createInternalClass("TriangleMeshes", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);
    domState.mSharedTetrahedronMeshesClass = createInternalClass("TetrahedronMeshes", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);
    domState.mSharedDeformableVolumeMeshesClass = createInternalClass("DeformableVolumeMeshes", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);

    domState.mSharedShapesClass = createInternalClass("Shapes", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);

    domState.mAttributeNameClass = createInternalClass("attrib", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);
    domState.mAttributeRefClass = createInternalClass("object_ref", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);

    domState.mSharedShapeRefClass = createInternalClass("shape_ref", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);
    domState.mConvexMeshRefClass = createInternalClass("convexmesh_ref", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);
    domState.mHeightfieldRefClass = createInternalClass("heightfield_ref", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);
    domState.mTriangleMeshRefClass = createInternalClass("trianglemesh_ref", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);
    domState.mTetrahedronMeshRefClass = createInternalClass("tetmesh_ref", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);

    domState.mSimulationMeshClass = createInternalClass("SimulationMesh", OmniPvdPhysXClassEnum::ePxInternalOmnniPvd);


    ////////////////////////////////////////////////////////////////////////////////
    // Create the shared layer nodes, top node with Prim path "/shared" is invisible
    ////////////////////////////////////////////////////////////////////////////////
    // /shared                       : mSharedRoot

    // /shared/materials             : mSharedMaterialsBranch

    // /shared/meshes                : mSharedMeshesBranch
    // /shared/meshes/convexmeshes   : mSharedConvexMeshesBranch
    // /shared/meshes/heightfields   : mSharedHeightfieldsBranch
    // /shared/meshes/trianglemeshes : mSharedTriangleMeshesBranch

    // /shared/shapes                : mSharedShapesBranch

    domState.mSharedRoot = createInternalNode(domState.mObjectCreations, 0, domState.mSharedRootClass, 0, 1, 0);

    domState.mSharedMaterialsBranch = createInternalNode(domState.mObjectCreations, domState.mSharedRoot, domState.mSharedMaterialsClass, 0, 1, 1);

    domState.mSharedMeshesBranch = createInternalNode(domState.mObjectCreations, domState.mSharedRoot, domState.mSharedMeshesClass, 0, 1, 1);
    domState.mSharedConvexMeshesBranch = createInternalNode(domState.mObjectCreations, domState.mSharedMeshesBranch, domState.mSharedConvexMeshesClass, 0, 1, 1);
    domState.mSharedHeightfieldsBranch = createInternalNode(domState.mObjectCreations, domState.mSharedMeshesBranch, domState.mSharedHeightfieldsClass, 0, 1, 1);
    domState.mSharedTriangleMeshesBranch = createInternalNode(domState.mObjectCreations, domState.mSharedMeshesBranch, domState.mSharedTriangleMeshesClass, 0, 1, 1);
    domState.mSharedTetrahedronMeshesBranch = createInternalNode(domState.mObjectCreations, domState.mSharedMeshesBranch, domState.mSharedTetrahedronMeshesClass, 0, 1, 1);
    domState.mSharedDeformableVolumeMeshesBranch = createInternalNode(domState.mObjectCreations, domState.mSharedMeshesBranch, domState.mSharedDeformableVolumeMeshesClass, 0, 1, 1);

    domState.mSharedShapesBranch = createInternalNode(domState.mObjectCreations, domState.mSharedRoot, domState.mSharedShapesClass, 0, 1, 1);

    ////////////////////////////////////////////////////////////////////////////////
    // Create the scene layer root nodes, top node with Prim path "/scenes" is visible
    ////////////////////////////////////////////////////////////////////////////////
    domState.mSceneRoot = createInternalNode(domState.mObjectCreations, 0, domState.mSceneRootClass, 0, 0, 1);

    initClassConfigMap(domState.mClassConfigMap);
    initAttributeConfigMap(domState.mAttributeConfigMap);

    domState.mActorTypeEnumRigidDynamic = 0;
    domState.mActorTypeEnumRigidStatic = UINT32_MAX;
    domState.mActorTypeEnumParticleSystem = UINT32_MAX;
    domState.mActorTypeEnumDeformableVolume = UINT32_MAX;
    domState.mActorTypeEnumDeformableSurface = UINT32_MAX;

    domState.mOvdIntegVersionWasChecked = false;
    domState.mOvdIntegVersionPassed = false;

    domState.mOvdIntegrationVersionMajor = PX_PHYSICS_OVD_INTEGRATION_VERSION_MAJOR;

    domState.mStreamOvdIntegVersionMajor = 0;
    domState.mStreamOvdIntegVersionMinor = 0;

    domState.mPxSceneClass = nullptr;
    domState.mPxArticulationReducedCoordinateClass = nullptr;
    domState.mPxArticulationLinkClass = nullptr;
}
