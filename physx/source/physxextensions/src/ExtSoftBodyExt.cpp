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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "extensions/PxSoftBodyExt.h"
#include "extensions/PxTetMakerExt.h"

#include "GuTetrahedronMesh.h"
#include "PxBuffer.h"
#include "cooking/PxCooking.h"
#include "PxPhysics.h"
#include "extensions/PxRemeshingExt.h"

using namespace physx;
using namespace Cm;

//Computes the volume of the simulation mesh defined as the sum of the volumes of all tetrahedra
static PxReal computeSimulationMeshVolume(PxSoftBody& sb)
{
	const PxU32 numTetsGM = sb.getSimulationMesh()->getNbTetrahedrons();

	const PxU32* tetPtr32 = reinterpret_cast<const PxU32*>(sb.getSimulationMesh()->getTetrahedrons());
	const PxU16* tetPtr16 = reinterpret_cast<const PxU16*>(sb.getSimulationMesh()->getTetrahedrons());
	const bool sixteenBit = sb.getSimulationMesh()->getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES;

	PxVec4* positionInvGM = reinterpret_cast<PxVec4*>(sb.getSimPositionInvMassCPU()->map());

	PxReal volume = 0;
	for (PxU32 i = 0; i < numTetsGM; ++i)
	{
		PxVec4& x0 = positionInvGM[sixteenBit ? tetPtr16[4 * i] : tetPtr32[4 * i]];
		PxVec4& x1 = positionInvGM[sixteenBit ? tetPtr16[4 * i + 1] : tetPtr32[4 * i + 1]];
		PxVec4& x2 = positionInvGM[sixteenBit ? tetPtr16[4 * i + 2] : tetPtr32[4 * i + 2]];
		PxVec4& x3 = positionInvGM[sixteenBit ? tetPtr16[4 * i + 3] : tetPtr32[4 * i + 3]];

		const PxVec3 u1 = x1.getXYZ() - x0.getXYZ();
		const PxVec3 u2 = x2.getXYZ() - x0.getXYZ();
		const PxVec3 u3 = x3.getXYZ() - x0.getXYZ();

		PxMat33 Q = PxMat33(u1, u2, u3);

		const PxReal det = Q.getDeterminant();
		volume += det;
	}
	volume /= 6.0f;

	sb.getSimPositionInvMassCPU()->unmap();

	return volume;
}

//Recomputes the volume associated with a vertex. Every tetrahedron distributes a quarter of its volume to
//each vertex it is connected to. Finally the volume stored for every vertex is inverted.
static void updateNodeInverseVolumes(PxSoftBody& sb)
{
	const PxU32 numVertsGM = sb.getSimulationMesh()->getNbVertices();
	const PxU32 numTetsGM = sb.getSimulationMesh()->getNbTetrahedrons();

	const PxU32* tetPtr32 = reinterpret_cast<const PxU32*>(sb.getSimulationMesh()->getTetrahedrons());
	const PxU16* tetPtr16 = reinterpret_cast<const PxU16*>(sb.getSimulationMesh()->getTetrahedrons());
	const bool sixteenBit = sb.getSimulationMesh()->getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES;

	PxVec4* positionInvGM = reinterpret_cast<PxVec4*>(sb.getSimPositionInvMassCPU()->map());

	for (PxU32 i = 0; i < numVertsGM; ++i)
		positionInvGM[i].w = 0.0f;

	for (PxU32 i = 0; i < numTetsGM; ++i)
	{
		PxVec4& x0 = positionInvGM[sixteenBit ? tetPtr16[4 * i] : tetPtr32[4 * i]];
		PxVec4& x1 = positionInvGM[sixteenBit ? tetPtr16[4 * i + 1] : tetPtr32[4 * i + 1]];
		PxVec4& x2 = positionInvGM[sixteenBit ? tetPtr16[4 * i + 2] : tetPtr32[4 * i + 2]];
		PxVec4& x3 = positionInvGM[sixteenBit ? tetPtr16[4 * i + 3] : tetPtr32[4 * i + 3]];

		const PxVec3 u1 = x1.getXYZ() - x0.getXYZ();
		const PxVec3 u2 = x2.getXYZ() - x0.getXYZ();
		const PxVec3 u3 = x3.getXYZ() - x0.getXYZ();
	
		PxMat33 Q = PxMat33(u1, u2, u3);

		//det should be positive
		const PxReal det = Q.getDeterminant();

		if (det <= 1.e-9f)		
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "updateNodeInverseVolumes(): tretrahedron is degenerate or inverted");
		
		//Distribute one quarter of the volume to each vertex the tetrahedron is connected to
		const PxReal volume = det / 6.0f;
		x0.w += 0.25f * volume;
		x1.w += 0.25f * volume;
		x2.w += 0.25f * volume;
		x3.w += 0.25f * volume;
	}

	//Invert the volumes stored per vertex and copy it to the velocity buffer
	PxVec4* velocityInvGM = reinterpret_cast<PxVec4*>(sb.getSimVelocityInvMassCPU()->map());
	for (PxU32 i = 0; i < numVertsGM; ++i)
	{
		if (positionInvGM[i].w > 0) 
		{
			positionInvGM[i].w = 1.0f / positionInvGM[i].w;			
		}
		velocityInvGM[i].w = positionInvGM[i].w;
	}

	sb.getSimPositionInvMassCPU()->unmap();
	sb.getSimVelocityInvMassCPU()->unmap();
}

void PxSoftBodyExt::updateMass(PxSoftBody& sb, const PxReal density, const PxReal maxInvMassRatio)
{
	//Inverse volumes are recomputed to ensure that multiple subsequent calls of this method lead to correct results
	updateNodeInverseVolumes(sb);

	const PxU32 numVertsGM = sb.getSimulationMesh()->getNbVertices();

	PxVec4* positionInvGM = reinterpret_cast<PxVec4*>(sb.getSimPositionInvMassCPU()->map());
	PxVec4* velocityInvGM = reinterpret_cast<PxVec4*>(sb.getSimVelocityInvMassCPU()->map());

	PxReal minInvMass = PX_MAX_F32, maxInvMass = 0.f;
	for (PxU32 i = 0; i < numVertsGM; ++i)
	{
		const PxVec4& vert = positionInvGM[i];
		PxReal invMass = vert.w;
		invMass = invMass / (density);
		minInvMass = invMass > 0.f ? PxMin(invMass, minInvMass) : minInvMass;
		maxInvMass = PxMax(invMass, maxInvMass);
		positionInvGM[i] = PxVec4(vert.x, vert.y, vert.z, invMass);

		const PxVec4& vel = velocityInvGM[i];
		velocityInvGM[i] = PxVec4(vel.x, vel.y, vel.z, invMass);
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
				PxVec4& posInvMass = positionInvGM[i];
				PxVec4& velInvMass = velocityInvGM[i];
				posInvMass.w = PxMin(posInvMass.w, maxInvMass);
				velInvMass.w = PxMin(velInvMass.w, maxInvMass);
			}
		}
	}

	sb.getSimPositionInvMassCPU()->unmap();
	sb.getSimVelocityInvMassCPU()->unmap();	
}

void PxSoftBodyExt::setMass(PxSoftBody& sb, const PxReal mass, const PxReal maxInvMassRatio)
{
	//Compute the density such that density times volume is equal to the desired mass
	updateMass(sb, mass / computeSimulationMeshVolume(sb), maxInvMassRatio);
}

void PxSoftBodyExt::transform(PxSoftBody& sb, const PxTransform& transform, const PxReal scale)
{
	const PxU32 numVertsGM = sb.getSimulationMesh()->getNbVertices();
	const PxU32 numVerts = sb.getCollisionMesh()->getNbVertices();
	
	PxVec4* collPositionInvGM = reinterpret_cast<PxVec4*>(sb.getPositionInvMassCPU()->map());
	PxVec4* positionInvGM = reinterpret_cast<PxVec4*>(sb.getSimPositionInvMassCPU()->map());
	PxVec4* velocityInvGM = reinterpret_cast<PxVec4*>(sb.getSimVelocityInvMassCPU()->map());
	PxVec4* restPositions = reinterpret_cast<PxVec4*>(sb.getRestPositionInvMassCPU()->map());
	
	for (PxU32 i = 0; i < numVertsGM; ++i)
	{
		const PxVec4 tpInvMass = positionInvGM[i];
		const PxReal invMass = tpInvMass.w;
		PxVec3 vert = PxVec3(tpInvMass.x * scale, tpInvMass.y * scale, tpInvMass.z * scale);
		//Transform the vertex position and keep the inverse mass
		vert = transform.transform(vert);
		positionInvGM[i] = PxVec4(vert.x, vert.y, vert.z, invMass);

		PxVec4 vel = velocityInvGM[i];
		PxVec3 v = PxVec3(vel.x * scale, vel.y * scale, vel.z * scale);
		//Velocities are translation invariant, therefore only the direction needs to get adjusted. 
		//The inverse mass is stored as well to optimize memory access on the GPU
		v = transform.rotate(v);
		velocityInvGM[i] = PxVec4(v.x, v.y, v.z, invMass);
	}

	for (PxU32 i = 0; i < numVerts; ++i)
	{
		restPositions[i] = PxVec4(restPositions[i].x*scale, restPositions[i].y*scale, restPositions[i].z*scale, 1.f);

		const PxVec4 tpInvMass = collPositionInvGM[i];
		PxVec3 vert = PxVec3(tpInvMass.x * scale, tpInvMass.y * scale, tpInvMass.z * scale);
		vert = transform.transform(vert);
		collPositionInvGM[i] = PxVec4(vert.x, vert.y, vert.z, tpInvMass.w);
	}

	sb.getSimPositionInvMassCPU()->unmap();
	sb.getSimVelocityInvMassCPU()->unmap();
	sb.getRestPositionInvMassCPU()->unmap();
	sb.getPositionInvMassCPU()->unmap();

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

void PxSoftBodyExt::updateEmbeddedCollisionMesh(PxSoftBody& sb)
{
	Gu::SoftBodyAuxData* softBodyAuxData = static_cast<Gu::SoftBodyAuxData*>(sb.getSoftBodyAuxData());
	const PxU32* remapTable = softBodyAuxData->mVertsRemapInGridModel;
	PxReal* barycentricCoordinates = softBodyAuxData->mVertsBarycentricInGridModel;

	PxTetrahedronMesh* simMesh = sb.getSimulationMesh();
	const void* tets = simMesh->getTetrahedrons();
	const PxU32* tets32 = static_cast<const PxU32*>(tets);
	const PxU16* tets16 = static_cast<const PxU16*>(tets);

	bool sixteenBit = simMesh->getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES;

	PxVec4* positionInvGM = reinterpret_cast<PxVec4*>(sb.getSimPositionInvMassCPU()->map());
	PxVec4* positionInv = reinterpret_cast<PxVec4*>(sb.getPositionInvMassCPU()->map());

	const PxU32 numVerts = sb.getCollisionMesh()->getNbVertices();
	for (PxU32 i = 0; i < numVerts; ++i)
	{
		//The tetrahedra are ordered differently on the GPU, therefore the index must be taken from the remap table
		const PxU32 tetrahedronIdx = remapTable[i];		
		const PxVec4 p0 = positionInvGM[sixteenBit ? tets16[4 * tetrahedronIdx] : tets32[4 * tetrahedronIdx]];
		const PxVec4 p1 = positionInvGM[sixteenBit ? tets16[4 * tetrahedronIdx + 1] : tets32[4 * tetrahedronIdx + 1]];
		const PxVec4 p2 = positionInvGM[sixteenBit ? tets16[4 * tetrahedronIdx + 2] : tets32[4 * tetrahedronIdx + 2]];
		const PxVec4 p3 = positionInvGM[sixteenBit ? tets16[4 * tetrahedronIdx + 3] : tets32[4 * tetrahedronIdx + 3]];

		const PxReal* barycentric = &barycentricCoordinates[4*i];

		//Compute the embedded position as a weigted sum of vertices from the simulation mesh
		//This ensures that all tranformations and scale changes applied to the simulation mesh get transferred
		//to the collision mesh
		positionInv[i] = p0 * barycentric[0] + p1 * barycentric[1] + p2 * barycentric[2] + p3 * barycentric[3];
		positionInv[i].w = 1.0f;
	}

	sb.getSimPositionInvMassCPU()->unmap();
	sb.getPositionInvMassCPU()->unmap();
}

void PxSoftBodyExt::commit(PxSoftBody& sb, PxSoftBodyDataFlags flags, bool flush)
{
	//Updating the collision mesh's vertices ensures that simulation mesh and collision mesh are
	//represented in the same coordinate system and the same scale
	updateEmbeddedCollisionMesh(sb);

	//Schedule data uploads for all buffers specified
	if (flags & PxSoftBodyData::eSIM_POSITION_INVMASS)
		sb.writeData(PxSoftBodyData::eSIM_POSITION_INVMASS, *sb.getSimPositionInvMassCPU(), flush);

	if (flags & PxSoftBodyData::eSIM_VELOCITY)
		sb.writeData(PxSoftBodyData::eSIM_VELOCITY, *sb.getSimVelocityInvMassCPU(), flush);

	if (flags & PxSoftBodyData::eSIM_KINEMATIC_TARGET)
		sb.writeData(PxSoftBodyData::eSIM_KINEMATIC_TARGET, *sb.getKinematicTargetCPU(), flush);
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


		const PxReal maxInvMassRatio = 50.f;

		softBody->setParameter(femParams);
		softBody->setSolverIterationCounts(solverIterationCount);

		PxSoftBodyExt::transform(*softBody, transform, scale);
		PxSoftBodyExt::updateMass(*softBody, density, maxInvMassRatio);
		PxSoftBodyExt::commit(*softBody, PxSoftBodyData::eALL);
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
