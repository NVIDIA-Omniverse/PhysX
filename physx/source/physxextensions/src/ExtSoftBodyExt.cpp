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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "extensions/PxSoftBodyExt.h"
#include "extensions/PxTetMakerExt.h"

#include "GuTetrahedronMesh.h"
#include "cooking/PxCooking.h"
#include "PxPhysics.h"
#include "extensions/PxRemeshingExt.h"
#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"

using namespace physx;
using namespace Cm;

//Computes the volume of the simulation mesh defined as the sum of the volumes of all tetrahedra
static PxReal computeSimulationMeshVolume(PxSoftBody& sb, PxVec4* simPositions)
{
	const PxU32 numTetsGM = sb.getSimulationMesh()->getNbTetrahedrons();

	const PxU32* tetPtr32 = reinterpret_cast<const PxU32*>(sb.getSimulationMesh()->getTetrahedrons());
	const PxU16* tetPtr16 = reinterpret_cast<const PxU16*>(sb.getSimulationMesh()->getTetrahedrons());
	const bool sixteenBit = sb.getSimulationMesh()->getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES;

	PxReal volume = 0;
	for (PxU32 i = 0; i < numTetsGM; ++i)
	{
		PxVec4& x0 = simPositions[sixteenBit ? tetPtr16[4 * i] : tetPtr32[4 * i]];
		PxVec4& x1 = simPositions[sixteenBit ? tetPtr16[4 * i + 1] : tetPtr32[4 * i + 1]];
		PxVec4& x2 = simPositions[sixteenBit ? tetPtr16[4 * i + 2] : tetPtr32[4 * i + 2]];
		PxVec4& x3 = simPositions[sixteenBit ? tetPtr16[4 * i + 3] : tetPtr32[4 * i + 3]];

		const PxVec3 u1 = x1.getXYZ() - x0.getXYZ();
		const PxVec3 u2 = x2.getXYZ() - x0.getXYZ();
		const PxVec3 u3 = x3.getXYZ() - x0.getXYZ();

		PxMat33 Q = PxMat33(u1, u2, u3);

		const PxReal det = Q.getDeterminant();
		volume += det;
	}
	volume /= 6.0f;

	return volume;
}

//Recomputes the volume associated with a vertex. Every tetrahedron distributes a quarter of its volume to
//each vertex it is connected to. Finally the volume stored for every vertex is inverted.
static void updateNodeInverseVolumes(PxSoftBody& sb, PxVec4* simPositions)
{
	const PxU32 numVertsGM = sb.getSimulationMesh()->getNbVertices();
	const PxU32 numTetsGM = sb.getSimulationMesh()->getNbTetrahedrons();

	const PxU32* tetPtr32 = reinterpret_cast<const PxU32*>(sb.getSimulationMesh()->getTetrahedrons());
	const PxU16* tetPtr16 = reinterpret_cast<const PxU16*>(sb.getSimulationMesh()->getTetrahedrons());
	const bool sixteenBit = sb.getSimulationMesh()->getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES;

	for (PxU32 i = 0; i < numVertsGM; ++i)
		simPositions[i].w = 0.0f;

	for (PxU32 i = 0; i < numTetsGM; ++i)
	{
		PxVec4& x0 = simPositions[sixteenBit ? tetPtr16[4 * i] : tetPtr32[4 * i]];
		PxVec4& x1 = simPositions[sixteenBit ? tetPtr16[4 * i + 1] : tetPtr32[4 * i + 1]];
		PxVec4& x2 = simPositions[sixteenBit ? tetPtr16[4 * i + 2] : tetPtr32[4 * i + 2]];
		PxVec4& x3 = simPositions[sixteenBit ? tetPtr16[4 * i + 3] : tetPtr32[4 * i + 3]];

		const PxVec3 u1 = x1.getXYZ() - x0.getXYZ();
		const PxVec3 u2 = x2.getXYZ() - x0.getXYZ();
		const PxVec3 u3 = x3.getXYZ() - x0.getXYZ();
	
		PxMat33 Q = PxMat33(u1, u2, u3);

		//det should be positive
		const PxReal det = Q.getDeterminant();

		if (det <= 1.e-9f)
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "updateNodeInverseVolumes(): tetrahedron is degenerate or inverted");

		//Distribute one quarter of the volume to each vertex the tetrahedron is connected to
		const PxReal volume = det / 6.0f;
		x0.w += 0.25f * volume;
		x1.w += 0.25f * volume;
		x2.w += 0.25f * volume;
		x3.w += 0.25f * volume;
	}

	//Invert the volumes stored per vertex and copy it to the velocity buffer
	for (PxU32 i = 0; i < numVertsGM; ++i)
	{
		if (simPositions[i].w > 0) 
		{
			simPositions[i].w = 1.0f / simPositions[i].w;			
		}
	}
}

void PxSoftBodyExt::updateMass(PxSoftBody& sb, const PxReal density, const PxReal maxInvMassRatio, PxVec4* simPositionsPinned)
{
	//Inverse volumes are recomputed to ensure that multiple subsequent calls of this method lead to correct results
	updateNodeInverseVolumes(sb, simPositionsPinned);

	const PxU32 numVertsGM = sb.getSimulationMesh()->getNbVertices();

	PxReal minInvMass = PX_MAX_F32, maxInvMass = 0.f;
	for (PxU32 i = 0; i < numVertsGM; ++i)
	{
		const PxVec4& vert = simPositionsPinned[i];
		PxReal invMass = vert.w;
		invMass = invMass / (density);
		minInvMass = invMass > 0.f ? PxMin(invMass, minInvMass) : minInvMass;
		maxInvMass = PxMax(invMass, maxInvMass);
		simPositionsPinned[i] = PxVec4(vert.x, vert.y, vert.z, invMass);
	}
	if (minInvMass != PX_MAX_F32)
	{
		const PxReal ratio = maxInvMass / minInvMass;
		if (ratio > maxInvMassRatio)
		{
			//Clamp the upper limit...
			maxInvMass = minInvMass * maxInvMassRatio;
			for (PxU32 i = 0; i < numVertsGM; ++i)
			{
				PxVec4& posInvMass = simPositionsPinned[i];
				posInvMass.w = PxMin(posInvMass.w, maxInvMass);
			}
		}
	}
}

void PxSoftBodyExt::setMass(PxSoftBody& sb, const PxReal mass, const PxReal maxInvMassRatio, PxVec4* simPositionsPinned)
{
	//Compute the density such that density times volume is equal to the desired mass
	updateMass(sb, mass / computeSimulationMeshVolume(sb, simPositionsPinned), maxInvMassRatio, simPositionsPinned);
}

void PxSoftBodyExt::transform(PxSoftBody& sb, const PxTransform& transform, const PxReal scale, PxVec4* simPositionsPinned, PxVec4* simVelocitiesPinned, PxVec4* collPositionsPinned, PxVec4* restPositionsPinned)
{
	const PxU32 numVertsGM = sb.getSimulationMesh()->getNbVertices();
	const PxU32 numVerts = sb.getCollisionMesh()->getNbVertices();
	
	for (PxU32 i = 0; i < numVertsGM; ++i)
	{
		const PxVec4 tpInvMass = simPositionsPinned[i];
		const PxReal invMass = tpInvMass.w;
		PxVec3 vert = PxVec3(tpInvMass.x * scale, tpInvMass.y * scale, tpInvMass.z * scale);
		//Transform the vertex position and keep the inverse mass
		vert = transform.transform(vert);
		simPositionsPinned[i] = PxVec4(vert.x, vert.y, vert.z, invMass);

		PxVec4 vel = simVelocitiesPinned[i];
		PxVec3 v = PxVec3(vel.x * scale, vel.y * scale, vel.z * scale);
		//Velocities are translation invariant, therefore only the direction needs to get adjusted. 
		//The inverse mass is stored as well to optimize memory access on the GPU
		v = transform.rotate(v);
		simVelocitiesPinned[i] = PxVec4(v.x, v.y, v.z, invMass);
	}

	for (PxU32 i = 0; i < numVerts; ++i)
	{
		restPositionsPinned[i] = PxVec4(restPositionsPinned[i].x*scale, restPositionsPinned[i].y*scale, restPositionsPinned[i].z*scale, 1.f);

		const PxVec4 tpInvMass = collPositionsPinned[i];
		PxVec3 vert = PxVec3(tpInvMass.x * scale, tpInvMass.y * scale, tpInvMass.z * scale);
		vert = transform.transform(vert);
		collPositionsPinned[i] = PxVec4(vert.x, vert.y, vert.z, tpInvMass.w);
	}

	PxMat33* tetraRestPosesGM = static_cast<Gu::SoftBodyAuxData*>(sb.getSoftBodyAuxData())->getGridModelRestPosesFast(); // reinterpret_cast<PxMat33*>(simMeshData->softBodyAuxData.getGridModelRestPosesFast());
	const PxU32 nbTetraGM = sb.getSimulationMesh()->getNbTetrahedrons();

	const PxReal invScale = 1.0f / scale;
	for (PxU32 i = 0; i < nbTetraGM; ++i)
	{
		PxMat33& m = tetraRestPosesGM[i];
		//Scale the rest pose
		m.column0 = m.column0 * invScale;
		m.column1 = m.column1 * invScale;
		m.column2 = m.column2 * invScale;

		//The rest pose is translation invariant, it only needs be rotated
		PxVec3 row0 = transform.rotateInv(PxVec3(m.column0.x, m.column1.x, m.column2.x));
		PxVec3 row1 = transform.rotateInv(PxVec3(m.column0.y, m.column1.y, m.column2.y));
		PxVec3 row2 = transform.rotateInv(PxVec3(m.column0.z, m.column1.z, m.column2.z));

		m.column0 = PxVec3(row0.x, row1.x, row2.x);
		m.column1 = PxVec3(row0.y, row1.y, row2.y);
		m.column2 = PxVec3(row0.z, row1.z, row2.z);
	}	


	PxMat33* tetraRestPoses = static_cast<Gu::SoftBodyAuxData*>(sb.getSoftBodyAuxData())->getRestPosesFast(); // reinterpret_cast<PxMat33*>(simMeshData->softBodyAuxData.getGridModelRestPosesFast());
	const PxU32 nbTetra = sb.getCollisionMesh()->getNbTetrahedrons();

	for (PxU32 i = 0; i < nbTetra; ++i)
	{
		PxMat33& m = tetraRestPoses[i];
		//Scale the rest pose
		m.column0 = m.column0 * invScale;
		m.column1 = m.column1 * invScale;
		m.column2 = m.column2 * invScale;

		//The rest pose is translation invariant, it only needs be rotated
		PxVec3 row0 = transform.rotateInv(PxVec3(m.column0.x, m.column1.x, m.column2.x));
		PxVec3 row1 = transform.rotateInv(PxVec3(m.column0.y, m.column1.y, m.column2.y));
		PxVec3 row2 = transform.rotateInv(PxVec3(m.column0.z, m.column1.z, m.column2.z));

		m.column0 = PxVec3(row0.x, row1.x, row2.x);
		m.column1 = PxVec3(row0.y, row1.y, row2.y);
		m.column2 = PxVec3(row0.z, row1.z, row2.z);
	}

}

void PxSoftBodyExt::updateEmbeddedCollisionMesh(PxSoftBody& sb, PxVec4* simPositionsPinned, PxVec4* collPositionsPinned)
{
	Gu::SoftBodyAuxData* softBodyAuxData = static_cast<Gu::SoftBodyAuxData*>(sb.getSoftBodyAuxData());
	const PxU32* remapTable = softBodyAuxData->mVertsRemapInGridModel;
	PxReal* barycentricCoordinates = softBodyAuxData->mVertsBarycentricInGridModel;

	PxTetrahedronMesh* simMesh = sb.getSimulationMesh();
	const void* tets = simMesh->getTetrahedrons();
	const PxU32* tets32 = static_cast<const PxU32*>(tets);
	const PxU16* tets16 = static_cast<const PxU16*>(tets);

	bool sixteenBit = simMesh->getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES;

	const PxU32 numVerts = sb.getCollisionMesh()->getNbVertices();
	for (PxU32 i = 0; i < numVerts; ++i)
	{
		//The tetrahedra are ordered differently on the GPU, therefore the index must be taken from the remap table
		const PxU32 tetrahedronIdx = remapTable[i];		
		const PxVec4 p0 = simPositionsPinned[sixteenBit ? tets16[4 * tetrahedronIdx] : tets32[4 * tetrahedronIdx]];
		const PxVec4 p1 = simPositionsPinned[sixteenBit ? tets16[4 * tetrahedronIdx + 1] : tets32[4 * tetrahedronIdx + 1]];
		const PxVec4 p2 = simPositionsPinned[sixteenBit ? tets16[4 * tetrahedronIdx + 2] : tets32[4 * tetrahedronIdx + 2]];
		const PxVec4 p3 = simPositionsPinned[sixteenBit ? tets16[4 * tetrahedronIdx + 3] : tets32[4 * tetrahedronIdx + 3]];

		const PxReal* barycentric = &barycentricCoordinates[4*i];

		//Compute the embedded position as a weigted sum of vertices from the simulation mesh
		//This ensures that all tranformations and scale changes applied to the simulation mesh get transferred
		//to the collision mesh
		collPositionsPinned[i] = p0 * barycentric[0] + p1 * barycentric[1] + p2 * barycentric[2] + p3 * barycentric[3];
		collPositionsPinned[i].w = 1.0f;
	}
}

void PxSoftBodyExt::commit(PxSoftBody& sb, PxSoftBodyDataFlags flags, PxVec4* simPositionsPinned, PxVec4* simVelocitiesPinned, PxVec4* collPositionsPinned, PxVec4* restPositionsPinned, CUstream stream)
{
	copyToDevice(sb, flags, simPositionsPinned, simVelocitiesPinned, collPositionsPinned, restPositionsPinned, stream);
}

void PxSoftBodyExt::copyToDevice(PxSoftBody& sb, PxSoftBodyDataFlags flags, PxVec4* simPositionsPinned, PxVec4* simVelocitiesPinned, PxVec4* collPositionsPinned, PxVec4* restPositionsPinned, CUstream stream)
{
	//Updating the collision mesh's vertices ensures that simulation mesh and collision mesh are
	//represented in the same coordinate system and the same scale
	updateEmbeddedCollisionMesh(sb, simPositionsPinned, collPositionsPinned);

#if PX_SUPPORT_GPU_PHYSX
	PxScopedCudaLock _lock(*sb.getCudaContextManager());
	PxCudaContext* ctx = sb.getCudaContextManager()->getCudaContext();

	if (flags & PxSoftBodyDataFlag::ePOSITION_INVMASS && collPositionsPinned)
		ctx->memcpyHtoDAsync(reinterpret_cast<CUdeviceptr>(sb.getPositionInvMassBufferD()), collPositionsPinned, sb.getCollisionMesh()->getNbVertices() * sizeof(PxVec4), stream);

	if (flags & PxSoftBodyDataFlag::eREST_POSITION_INVMASS && restPositionsPinned)
		ctx->memcpyHtoDAsync(reinterpret_cast<CUdeviceptr>(sb.getRestPositionBufferD()), restPositionsPinned, sb.getCollisionMesh()->getNbVertices() * sizeof(PxVec4), stream);

	if (flags & PxSoftBodyDataFlag::eSIM_POSITION_INVMASS && simPositionsPinned)
		ctx->memcpyHtoDAsync(reinterpret_cast<CUdeviceptr>(sb.getSimPositionInvMassBufferD()), simPositionsPinned, sb.getSimulationMesh()->getNbVertices() * sizeof(PxVec4), stream);

	if (flags & PxSoftBodyDataFlag::eSIM_VELOCITY && simVelocitiesPinned)
		ctx->memcpyHtoDAsync(reinterpret_cast<CUdeviceptr>(sb.getSimVelocityBufferD()), simVelocitiesPinned, sb.getSimulationMesh()->getNbVertices() * sizeof(PxVec4), stream);

	// we need to synchronize if the stream is the default argument.
	if (stream == 0)
	{
		ctx->streamSynchronize(stream);
	}
#else
	PX_UNUSED(restPositionsPinned);
	PX_UNUSED(simVelocitiesPinned);
	PX_UNUSED(stream);
#endif

	sb.markDirty(flags);
}

PxSoftBodyMesh* PxSoftBodyExt::createSoftBodyMesh(const PxCookingParams& params, const PxSimpleTriangleMesh& surfaceMesh, PxU32 numVoxelsAlongLongestAABBAxis, PxInsertionCallback& insertionCallback, const bool validate)
{
	//Compute collision mesh
	physx::PxArray<physx::PxVec3> collisionMeshVertices;
	physx::PxArray<physx::PxU32> collisionMeshIndices;
	if (!PxTetMaker::createConformingTetrahedronMesh(surfaceMesh, collisionMeshVertices, collisionMeshIndices, validate))
		return NULL;
	PxTetrahedronMeshDesc meshDesc(collisionMeshVertices, collisionMeshIndices);
	
	//Compute simulation mesh
	physx::PxArray<physx::PxI32> vertexToTet;
	vertexToTet.resize(meshDesc.points.count);
	physx::PxArray<physx::PxVec3> simulationMeshVertices;
	physx::PxArray<physx::PxU32> simulationMeshIndices;
	PxTetMaker::createVoxelTetrahedronMesh(meshDesc, numVoxelsAlongLongestAABBAxis, simulationMeshVertices, simulationMeshIndices, vertexToTet.begin());
	PxTetrahedronMeshDesc simMeshDesc(simulationMeshVertices, simulationMeshIndices);
	PxSoftBodySimulationDataDesc simDesc(vertexToTet);

	physx::PxSoftBodyMesh* softBodyMesh = PxCreateSoftBodyMesh(params, simMeshDesc, meshDesc, simDesc, insertionCallback);
	
	return softBodyMesh;
}

PxSoftBodyMesh* PxSoftBodyExt::createSoftBodyMeshNoVoxels(const PxCookingParams& params, const PxSimpleTriangleMesh& surfaceMesh, PxInsertionCallback& insertionCallback, PxReal maxWeightRatioInTet, const bool validate)
{
	PxCookingParams p = params;
	p.maxWeightRatioInTet = maxWeightRatioInTet;

	physx::PxArray<physx::PxVec3> collisionMeshVertices;
	physx::PxArray<physx::PxU32> collisionMeshIndices;
	if (!PxTetMaker::createConformingTetrahedronMesh(surfaceMesh, collisionMeshVertices, collisionMeshIndices, validate))
		return NULL;
	PxTetrahedronMeshDesc meshDesc(collisionMeshVertices, collisionMeshIndices);
	PxSoftBodySimulationDataDesc simDesc;

	physx::PxSoftBodyMesh* softBodyMesh = PxCreateSoftBodyMesh(p, meshDesc, meshDesc, simDesc, insertionCallback);

	return softBodyMesh;
}

PxSoftBody* PxSoftBodyExt::createSoftBodyFromMesh(PxSoftBodyMesh* softBodyMesh, const PxTransform& transform, const PxFEMSoftBodyMaterial& material, PxCudaContextManager& cudaContextManager,
	PxReal density, PxU32 solverIterationCount, const PxFEMParameters& femParams, PxReal scale)
{
	PxSoftBody* softBody = PxGetPhysics().createSoftBody(cudaContextManager);
	if (softBody)
	{
		PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE;


		PxTetrahedronMeshGeometry geometry(softBodyMesh->getCollisionMesh());
		PxFEMSoftBodyMaterial* materialPointer = const_cast<PxFEMSoftBodyMaterial*>(&material);
		PxShape* shape = PxGetPhysics().createShape(geometry, &materialPointer, 1, true, shapeFlags);
		if (shape)
		{
			softBody->attachShape(*shape);
		}
		softBody->attachSimulationMesh(*softBodyMesh->getSimulationMesh(), *softBodyMesh->getSoftBodyAuxData());

		PxVec4* simPositionInvMassPinned;
		PxVec4* simVelocityPinned;
		PxVec4* collPositionInvMassPinned;
		PxVec4* restPositionPinned;

		PxSoftBodyExt::allocateAndInitializeHostMirror(*softBody, &cudaContextManager, simPositionInvMassPinned, simVelocityPinned, collPositionInvMassPinned, restPositionPinned);

		const PxReal maxInvMassRatio = 50.f;

		softBody->setParameter(femParams);
		softBody->setSolverIterationCounts(solverIterationCount);

		PxSoftBodyExt::transform(*softBody, transform, scale, simPositionInvMassPinned, simVelocityPinned, collPositionInvMassPinned, restPositionPinned);
		PxSoftBodyExt::updateMass(*softBody, density, maxInvMassRatio, simPositionInvMassPinned);
		PxSoftBodyExt::copyToDevice(*softBody, PxSoftBodyDataFlag::eALL, simPositionInvMassPinned, simVelocityPinned, collPositionInvMassPinned, restPositionPinned);

#if PX_SUPPORT_GPU_PHYSX
		PxCudaContextManager* mgr = &cudaContextManager;
		PX_PINNED_HOST_FREE(mgr, simPositionInvMassPinned);
		PX_PINNED_HOST_FREE(mgr, simVelocityPinned);
		PX_PINNED_HOST_FREE(mgr, collPositionInvMassPinned);
		PX_PINNED_HOST_FREE(mgr, restPositionPinned)
#endif
	}
	return softBody;
}

PxSoftBody* PxSoftBodyExt::createSoftBodyBox(const PxTransform& transform, const PxVec3& boxDimensions, const PxFEMSoftBodyMaterial& material,
	PxCudaContextManager& cudaContextManager, PxReal maxEdgeLength, PxReal density, PxU32 solverIterationCount, const PxFEMParameters& femParams, PxU32 numVoxelsAlongLongestAABBAxis, PxReal scale)
{
	PxArray<PxVec3> triVerts;
	triVerts.reserve(8);
	triVerts.pushBack(PxVec3(0.5f, -0.5f, -0.5f).multiply(boxDimensions));
	triVerts.pushBack(PxVec3(0.5f, -0.5f, 0.5f).multiply(boxDimensions));
	triVerts.pushBack(PxVec3(-0.5f, -0.5f, 0.5f).multiply(boxDimensions));
	triVerts.pushBack(PxVec3(-0.5f, -0.5f, -0.5f).multiply(boxDimensions));
	triVerts.pushBack(PxVec3(0.5f, 0.5f, -0.5f).multiply(boxDimensions));
	triVerts.pushBack(PxVec3(0.5f, 0.5f, 0.5f).multiply(boxDimensions));
	triVerts.pushBack(PxVec3(-0.5f, 0.5f, 0.5f).multiply(boxDimensions));
	triVerts.pushBack(PxVec3(-0.5f, 0.5f, -0.5f).multiply(boxDimensions));

	PxArray<PxU32> triIndices;
	triIndices.reserve(12 * 3);
	triIndices.pushBack(1); triIndices.pushBack(2); triIndices.pushBack(3);
	triIndices.pushBack(7); triIndices.pushBack(6); triIndices.pushBack(5);
	triIndices.pushBack(4); triIndices.pushBack(5); triIndices.pushBack(1);
	triIndices.pushBack(5); triIndices.pushBack(6); triIndices.pushBack(2);

	triIndices.pushBack(2); triIndices.pushBack(6); triIndices.pushBack(7);
	triIndices.pushBack(0); triIndices.pushBack(3); triIndices.pushBack(7);
	triIndices.pushBack(0); triIndices.pushBack(1); triIndices.pushBack(3);
	triIndices.pushBack(4); triIndices.pushBack(7); triIndices.pushBack(5);

	triIndices.pushBack(0); triIndices.pushBack(4); triIndices.pushBack(1);
	triIndices.pushBack(1); triIndices.pushBack(5); triIndices.pushBack(2);
	triIndices.pushBack(3); triIndices.pushBack(2); triIndices.pushBack(7);
	triIndices.pushBack(4); triIndices.pushBack(0); triIndices.pushBack(7);

	if (maxEdgeLength > 0.0f)
		PxRemeshingExt::limitMaxEdgeLength(triIndices, triVerts, maxEdgeLength, 3);

	PxSimpleTriangleMesh surfaceMesh;
	surfaceMesh.points.count = triVerts.size();
	surfaceMesh.points.data = triVerts.begin();
	surfaceMesh.triangles.count = triIndices.size() / 3;
	surfaceMesh.triangles.data = triIndices.begin();

	PxTolerancesScale tolerancesScale;
	PxCookingParams params(tolerancesScale);
	params.meshWeldTolerance = 0.001f;
	params.meshPreprocessParams = PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES);
	params.buildTriangleAdjacencies = false;
	params.buildGPUData = true;
	params.midphaseDesc = PxMeshMidPhase::eBVH34;

	PxSoftBodyMesh* softBodyMesh = createSoftBodyMesh(params, surfaceMesh, numVoxelsAlongLongestAABBAxis, PxGetPhysics().getPhysicsInsertionCallback());

	return createSoftBodyFromMesh(softBodyMesh, transform, material, cudaContextManager, density, solverIterationCount, femParams, scale);
}

void PxSoftBodyExt::allocateAndInitializeHostMirror(PxSoftBody& softBody, PxCudaContextManager* cudaContextManager, PxVec4*& simPositionInvMassPinned, PxVec4*& simVelocityPinned, PxVec4*& collPositionInvMassPinned, PxVec4*& restPositionPinned)
{
	PX_ASSERT(softBody.getCollisionMesh() != NULL);
	PX_ASSERT(softBody.getSimulationMesh() != NULL);

	PxU32 nbCollVerts = softBody.getCollisionMesh()->getNbVertices();
	PxU32 nbSimVerts = softBody.getSimulationMesh()->getNbVertices();

#if PX_SUPPORT_GPU_PHYSX
	simPositionInvMassPinned = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, nbSimVerts);
	simVelocityPinned = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, nbSimVerts);
	collPositionInvMassPinned = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, nbCollVerts);
	restPositionPinned = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, nbCollVerts);
#else
	PX_UNUSED(cudaContextManager);
#endif

	// write positionInvMass into CPU part.
	const PxVec3* positions = softBody.getCollisionMesh()->getVertices();

	for (PxU32 i = 0; i < nbCollVerts; ++i)
	{
		PxVec3 vert = positions[i];
		collPositionInvMassPinned[i] = PxVec4(vert, 1.f);
		restPositionPinned[i] = PxVec4(vert, 1.f);
	}

	// write sim mesh part.
	PxSoftBodyAuxData* s = softBody.getSoftBodyAuxData();
	PxReal* invMassGM = s->getGridModelInvMass();

	const PxVec3* simPositions = softBody.getSimulationMesh()->getVertices();
	for (PxU32 i = 0; i < nbSimVerts; ++i)
	{
		PxReal invMass = invMassGM ? invMassGM[i] : 1.f;
		simPositionInvMassPinned[i] = PxVec4(simPositions[i], invMass);
		simVelocityPinned[i] = PxVec4(0.f, 0.f, 0.f, invMass);
	}
}

struct InternalSoftBodyState
{
	PxVec4* mVertices;
	PxArray<PxReal> mInvMasses;
	const PxU32* mTetrahedra;
	PxArray<PxMat33> mInvRestPose;
	PxArray<PxVec3> mPrevPos;

	InternalSoftBodyState(const PxVec4* verticesOriginal, PxVec4* verticesDeformed, PxU32 nbVertices, const PxU32* tetrahedra, PxU32 nbTetraheda, const bool* vertexIsFixed) :
		mVertices(verticesDeformed), mTetrahedra(tetrahedra)
	{
		PxReal density = 1.0f;
		
		mInvMasses.resize(nbVertices, 0.0f);
		mInvRestPose.resize(nbTetraheda, PxMat33(PxZero));
		for (PxU32 i = 0; i < nbTetraheda; i++)
		{
			const PxU32* t = &mTetrahedra[4 * i];
			const PxVec3 a = verticesOriginal[t[0]].getXYZ();
			PxMat33 ir(verticesOriginal[t[1]].getXYZ() - a, verticesOriginal[t[2]].getXYZ() - a, verticesOriginal[t[3]].getXYZ() - a);
			PxReal volume  = ir.getDeterminant() / 6.0f;
			if (volume > 1e-8f)
				mInvRestPose[i] = ir.getInverse();

			PxReal m = 0.25f * volume * density;
			mInvMasses[t[0]] += m;
			mInvMasses[t[1]] += m;
			mInvMasses[t[2]] += m;
			mInvMasses[t[3]] += m;
		}
		
		for (PxU32 i = 0; i < nbVertices; i++)
		{
			bool fixed = vertexIsFixed ? vertexIsFixed[i] : verticesOriginal[i].w == 0.0f;
			if (mInvMasses[i] != 0.0f && !fixed)
				mInvMasses[i] = 1.0f / mInvMasses[i];
			else
				mInvMasses[i] = 0.0f;
		}		

		mPrevPos.resize(nbVertices);
		for (PxU32 i = 0; i < mPrevPos.size(); ++i)
			mPrevPos[i] = mVertices[i].getXYZ();
	}

	void applyDelta()
	{
		for (PxU32 i = 0; i < mPrevPos.size(); ++i)
		{
			PxVec3 delta = mVertices[i].getXYZ() - mPrevPos[i];
			mPrevPos[i] = mVertices[i].getXYZ();
			mVertices[i] += PxVec4(0.99f * delta, 0.0f);
		}
	}

	PX_FORCE_INLINE void applyToElem(PxU32 elemNr, PxReal C, PxReal compliance, const PxVec3& g1, const PxVec3& g2, const PxVec3& g3, const PxVec4& invMasses)
	{
		if (C == 0.0f)
			return;
		const PxVec3 g0 = -g1 - g2 - g3;

		const PxU32* t = &mTetrahedra[4 * elemNr];
		const PxReal w = g0.magnitudeSquared() * invMasses.x + g1.magnitudeSquared() * invMasses.y + g2.magnitudeSquared() * invMasses.z + g3.magnitudeSquared() * invMasses.w;

		if (w == 0.0f)
			return;

		const PxReal alpha = compliance;
		const PxReal dlambda = -C / (w + alpha);

		if (invMasses.x != 0.0f) 
			mVertices[t[0]] += PxVec4(g0 * dlambda * invMasses.x, 0.0f);
		if (invMasses.y != 0.0f) 
			mVertices[t[1]] += PxVec4(g1 * dlambda * invMasses.y, 0.0f);
		if (invMasses.z != 0.0f) 
			mVertices[t[2]] += PxVec4(g2 * dlambda * invMasses.z, 0.0f);
		if (invMasses.w != 0.0f) 
			mVertices[t[3]] += PxVec4(g3 * dlambda * invMasses.w, 0.0f);
	}

	void solveElem(PxU32 elemNr)
	{
		const PxMat33& ir = mInvRestPose[elemNr];
		if (ir == PxMat33(PxZero))
			return;

		const PxU32* tet = &mTetrahedra[4 * elemNr];

		PxVec4 invMasses(mInvMasses[tet[0]], mInvMasses[tet[1]], mInvMasses[tet[2]], mInvMasses[tet[3]]);

		PxMat33 P;		 
		P.column0 = mVertices[tet[1]].getXYZ() - mVertices[tet[0]].getXYZ();
		P.column1 = mVertices[tet[2]].getXYZ() - mVertices[tet[0]].getXYZ();
		P.column2 = mVertices[tet[3]].getXYZ() - mVertices[tet[0]].getXYZ();
		
		PxMat33 F = P * ir;

		PxVec3 g1 = F.column0 * 2.0f * ir.column0.x + F.column1 * 2.0f * ir.column1.x + F.column2 * 2.0f * ir.column2.x;
		PxVec3 g2 = F.column0 * 2.0f * ir.column0.y + F.column1 * 2.0f * ir.column1.y + F.column2 * 2.0f * ir.column2.y;
		PxVec3 g3 = F.column0 * 2.0f * ir.column0.z + F.column1 * 2.0f * ir.column1.z + F.column2 * 2.0f * ir.column2.z;

		PxReal C = F.column0.magnitudeSquared() + F.column1.magnitudeSquared() + F.column2.magnitudeSquared() - 3.0f;

		applyToElem(elemNr, C, 0.0f, g1, g2, g3, invMasses);

		P.column0 = mVertices[tet[1]].getXYZ() - mVertices[tet[0]].getXYZ();
		P.column1 = mVertices[tet[2]].getXYZ() - mVertices[tet[0]].getXYZ();
		P.column2 = mVertices[tet[3]].getXYZ() - mVertices[tet[0]].getXYZ();

		F = P * ir;

		PxMat33& dF = P; //Re-use memory, possible since P is not used anymore afterwards
		dF.column0 = F.column1.cross(F.column2);
		dF.column1 = F.column2.cross(F.column0);
		dF.column2 = F.column0.cross(F.column1);

		g1 = dF.column0 * ir.column0.x + dF.column1 * ir.column1.x + dF.column2 * ir.column2.x;
		g2 = dF.column0 * ir.column0.y + dF.column1 * ir.column1.y + dF.column2 * ir.column2.y;
		g3 = dF.column0 * ir.column0.z + dF.column1 * ir.column1.z + dF.column2 * ir.column2.z;

		C = F.getDeterminant() - 1.0f;

		applyToElem(elemNr, C, 0.0f, g1, g2, g3, invMasses);
	}
};

void PxSoftBodyExt::relaxSoftBodyMesh(const PxVec4* verticesOriginal, PxVec4* verticesDeformed, PxU32 nbVertices, const PxU32* tetrahedra, PxU32 nbTetraheda, const bool* vertexIsFixed, PxU32 numIterations)
{
	InternalSoftBodyState state(verticesOriginal, verticesDeformed, nbVertices, tetrahedra, nbTetraheda, vertexIsFixed);
	for (PxU32 iter = 0; iter < numIterations; ++iter)
	{
		state.applyDelta();
		for (PxU32 i = 0; i < nbTetraheda; ++i)
			state.solveElem(i);		
	}
	return;
}
