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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "extensions/PxDeformableSurfaceExt.h"

#include "PxPhysics.h"
#include "cooking/PxCooking.h"
#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"
#include "extensions/PxCudaHelpersExt.h"

using namespace physx;

void PxDeformableSurfaceExt::copyToDevice(PxDeformableSurface& fm, PxDeformableSurfaceDataFlags flags, PxU32 nbVertices,
										  PxVec4* positionInvMassPinned, PxVec4* velocityPinned, PxVec4* restPositionPinned,
										  CUstream stream)
{
#if PX_SUPPORT_GPU_PHYSX
	PxScopedCudaLock _lock(*fm.getCudaContextManager());
	PxCudaContext* ctx = fm.getCudaContextManager()->getCudaContext();

	if(flags & PxDeformableSurfaceDataFlag::ePOSITION_INVMASS && positionInvMassPinned)
		ctx->memcpyHtoDAsync(reinterpret_cast<CUdeviceptr>(fm.getPositionInvMassBufferD()), positionInvMassPinned,
		                     nbVertices * sizeof(PxVec4), stream);

	if(flags & PxDeformableSurfaceDataFlag::eVELOCITY && velocityPinned)
		ctx->memcpyHtoDAsync(reinterpret_cast<CUdeviceptr>(fm.getVelocityBufferD()), velocityPinned,
		                     nbVertices * sizeof(PxVec4), stream);

	if(flags & PxDeformableSurfaceDataFlag::eREST_POSITION && restPositionPinned)
		ctx->memcpyHtoDAsync(reinterpret_cast<CUdeviceptr>(fm.getRestPositionBufferD()), restPositionPinned,
		                     nbVertices * sizeof(PxVec4), stream);

	if(stream == 0)
		ctx->streamSynchronize(stream);
#else
	PX_UNUSED(nbVertices);
	PX_UNUSED(positionInvMassPinned);
	PX_UNUSED(velocityPinned);
	PX_UNUSED(restPositionPinned);
	PX_UNUSED(stream);
#endif
	fm.markDirty(flags);
}

PxU32 PxDeformableSurfaceExt::allocateAndInitializeHostMirror(PxDeformableSurface& deformableSurface, const PxVec3* positions,
															  const PxVec3* velocities, const PxVec3* restPositions,
															  float clothMass, const PxTransform& transform,
															  PxCudaContextManager* cudaContextManager,
															  PxVec4*& positionInvMassPinned, PxVec4*& velocityPinned,
															  PxVec4*& restPositionPinned)
{
	PX_ASSERT(cudaContextManager != NULL);

	PxShape* shape = deformableSurface.getShape();
	if(shape == NULL)
		return 0;

	const PxTriangleMeshGeometry& triangleMeshGeometry = static_cast<const PxTriangleMeshGeometry&>(shape->getGeometry());

	PxTriangleMesh* triangleMesh = triangleMeshGeometry.triangleMesh;
	if(triangleMesh == NULL)
		return 0;

	const PxU32 nbVertices = triangleMesh->getNbVertices();
	const PxVec3* const initPositions = positions ? positions : triangleMesh->getVertices(); // triangle mesh may represent "rest configuration".

	allocateAndInitializeHostMirror(initPositions, velocities, restPositions, nbVertices, clothMass, transform,
	                                cudaContextManager, positionInvMassPinned, velocityPinned, restPositionPinned);

	return nbVertices;
}

PxU32 PxDeformableSurfaceExt::allocateAndInitializeHostMirror(const PxVec3* positions, const PxVec3* velocities,
															  const PxVec3* restPositions, PxU32 nbVertices, float clothMass,
															  const PxTransform& transform,
															  PxCudaContextManager* cudaContextManager,
															  PxVec4*& positionInvMassPinned, PxVec4*& velocityPinned,
															  PxVec4*& restPositionPinned)
{
#if PX_SUPPORT_GPU_PHYSX
	positionInvMassPinned = PX_EXT_PINNED_MEMORY_ALLOC(PxVec4, *cudaContextManager, nbVertices);
	velocityPinned = PX_EXT_PINNED_MEMORY_ALLOC(PxVec4, *cudaContextManager, nbVertices);
	restPositionPinned = PX_EXT_PINNED_MEMORY_ALLOC(PxVec4, *cudaContextManager, nbVertices);
#else
	PX_UNUSED(cudaContextManager);
#endif

	float invMass = static_cast<float>(nbVertices) / clothMass;

	for(PxU32 i = 0; i < nbVertices; ++i)
	{
		PxVec3 pos = positions[i];
		pos = transform.transform(pos);

		positionInvMassPinned[i] = PxVec4(pos.x, pos.y, pos.z, invMass);
		velocityPinned[i] = velocities ? PxVec4(velocities[i].x, velocities[i].y, velocities[i].z, invMass)
		                               : PxVec4(0.f, 0.f, 0.f, invMass);
		restPositionPinned[i] = restPositions
		                            ? PxVec4(restPositions[i].x, restPositions[i].y, restPositions[i].z, invMass)
		                            : PxVec4(pos.x, pos.y, pos.z, invMass);
	}

	return nbVertices;
}

void PxDeformableSurfaceExt::distributeTriangleMassToVertices(PxDeformableSurface& deformableSurface, const PxReal* triangleMasses,
															  PxVec4* positionInvMassPinned)
{
	PxShape* shape = deformableSurface.getShape();
	const PxTriangleMeshGeometry& triangleMeshGeometry = static_cast<const PxTriangleMeshGeometry&>(shape->getGeometry());
	PxTriangleMesh* triangleMesh = triangleMeshGeometry.triangleMesh;
	const PxU32 numVerts = triangleMesh->getNbVertices();
	const PxU32 numTriangles = triangleMesh->getNbTriangles();

	PxArray<PxReal> vertexMasses(numVerts, 0.f);
	const bool has16bitIndices = triangleMesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES;
	const void* indices = triangleMesh->getTriangles();

	PxReal oneThird = 1.0f / 3.0f;
	for(PxU32 i = 0; i < numTriangles; ++i)
	{
		PxU32 vref0, vref1, vref2;
		if (has16bitIndices)
		{
			vref0 = ((const PxU16*)indices)[i * 3 + 0];
			vref1 = ((const PxU16*)indices)[i * 3 + 1];
			vref2 = ((const PxU16*)indices)[i * 3 + 2];
		}
		else
		{
			vref0 = ((const PxU32*)indices)[i * 3 + 0];
			vref1 = ((const PxU32*)indices)[i * 3 + 1];
			vref2 = ((const PxU32*)indices)[i * 3 + 2];
		}

		PxReal vertexMass = oneThird * triangleMasses[i];

		vertexMasses[vref0] += vertexMass;
		vertexMasses[vref1] += vertexMass;
		vertexMasses[vref2] += vertexMass;
	}

	for(PxU32 i = 0; i < numVerts; ++i)
	{
		PxReal invMass = 1.f / vertexMasses[i];
		positionInvMassPinned[i].w = invMass;
	}
}

void PxDeformableSurfaceExt::distributeDensityToVertices(PxDeformableSurface& deformableSurface, PxReal massPerVolume, PxReal clothThickness,
	PxVec4* positionInvMassPinned)
{
	const PxReal massPerArea = massPerVolume * clothThickness;

	PxShape* shape = deformableSurface.getShape();
	const PxTriangleMeshGeometry& triangleMeshGeometry = static_cast<const PxTriangleMeshGeometry&>(shape->getGeometry());
	PxTriangleMesh* triangleMesh = triangleMeshGeometry.triangleMesh;
	const PxU32 numVerts = triangleMesh->getNbVertices();
	const PxU32 numTriangles = triangleMesh->getNbTriangles();

	PxArray<PxReal> vertexMasses(numVerts, 0.f);
	const bool has16bitIndices = triangleMesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES;
	const void* indices = triangleMesh->getTriangles();
	const PxVec3* vertices = triangleMesh->getVertices();

	PxReal oneThird = 1.0f / 3.0f;
	for (PxU32 i = 0; i < numTriangles; ++i)
	{
		PxU32 vref0, vref1, vref2;
		if (has16bitIndices)
		{
			vref0 = ((const PxU16*)indices)[i * 3 + 0];
			vref1 = ((const PxU16*)indices)[i * 3 + 1];
			vref2 = ((const PxU16*)indices)[i * 3 + 2];
		}
		else
		{
			vref0 = ((const PxU32*)indices)[i * 3 + 0];
			vref1 = ((const PxU32*)indices)[i * 3 + 1];
			vref2 = ((const PxU32*)indices)[i * 3 + 2];
		}

		const PxVec3 a = vertices[vref0];
		const PxVec3 b = vertices[vref1];
		const PxVec3 c = vertices[vref2];

		PxReal triangleArea = 0.5f * (b - a).cross(c - a).magnitude();
		PxReal triangleMass = massPerArea* triangleArea;
		PxReal vertexMass = oneThird * triangleMass;

		vertexMasses[vref0] += vertexMass;
		vertexMasses[vref1] += vertexMass;
		vertexMasses[vref2] += vertexMass;
	}

	for (PxU32 i = 0; i < numVerts; ++i)
	{
		PxReal invMass = 1.f / vertexMasses[i];
		positionInvMassPinned[i].w = invMass;
	}
}

void PxDeformableSurfaceExt::distributeMassToVertices(PxDeformableSurface& deformableSurface, PxReal totalMass, 
	PxVec4* positionInvMassPinned)
{
	PxShape* shape = deformableSurface.getShape();
	const PxTriangleMeshGeometry& triangleMeshGeometry = static_cast<const PxTriangleMeshGeometry&>(shape->getGeometry());
	PxTriangleMesh* triangleMesh = triangleMeshGeometry.triangleMesh;

	PxReal totalArea = 0.0f;
	
	const PxU32 numVerts = triangleMesh->getNbVertices();
	const PxU32 numTriangles = triangleMesh->getNbTriangles();

	PxArray<PxReal> vertexMasses(numVerts, 0.f);
	const bool has16bitIndices = triangleMesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES;
	const void* indices = triangleMesh->getTriangles();
	const PxVec3* vertices = triangleMesh->getVertices();

	for (PxU32 i = 0; i < numTriangles; ++i)
	{
		PxU32 vref0, vref1, vref2;
		if (has16bitIndices)
		{
			vref0 = ((const PxU16*)indices)[i * 3 + 0];
			vref1 = ((const PxU16*)indices)[i * 3 + 1];
			vref2 = ((const PxU16*)indices)[i * 3 + 2];
		}
		else
		{
			vref0 = ((const PxU32*)indices)[i * 3 + 0];
			vref1 = ((const PxU32*)indices)[i * 3 + 1];
			vref2 = ((const PxU32*)indices)[i * 3 + 2];
		}
		const PxVec3 a = vertices[vref0];
		const PxVec3 b = vertices[vref1];
		const PxVec3 c = vertices[vref2];

		totalArea += (b - a).cross(c - a).magnitude();
	}
	totalArea *= 0.5f;

	PxReal massPerArea = totalMass / totalArea;
	const PxReal clothThickness = 1.0f; //Value does not matter since it cancels out
	distributeDensityToVertices(deformableSurface, massPerArea / clothThickness, clothThickness, positionInvMassPinned);
}
