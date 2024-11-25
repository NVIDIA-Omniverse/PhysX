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

#ifndef PHYSX_SNIPPET_DEFORMABLE_SURFACE_H
#define PHYSX_SNIPPET_DEFORMABLE_SURFACE_H

#include "PxPhysicsAPI.h"
#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"
#include "PxDeformableSurface.h"
#include "geometry/PxTriangle.h"
#include "geometry/PxTriangleMeshGeometry.h"
#include "extensions/PxCudaHelpersExt.h"

class TestSurface
{
public:
	TestSurface() :
		mPositionsInvMass(NULL),
		mDeformableSurface(NULL),
		mCudaContextManager(NULL),		
		mTriangleMesh(NULL)
	{ }

	TestSurface(physx::PxDeformableSurface* deformableSurface, physx::PxCudaContextManager* cudaContextManager) :
		mDeformableSurface(deformableSurface),
		mCudaContextManager(cudaContextManager)
	{
		physx::PxShape* shape = deformableSurface->getShape();

		const physx::PxTriangleMeshGeometry& triangleMeshGeom = static_cast<const physx::PxTriangleMeshGeometry&>(shape->getGeometry());
		mTriangleMesh = triangleMeshGeom.triangleMesh;

		mPositionsInvMass = PX_EXT_PINNED_MEMORY_ALLOC(physx::PxVec4, *cudaContextManager, mTriangleMesh->getNbVertices());
	}

	~TestSurface()
	{
	}

	void release()
	{
		if (mDeformableSurface)
			mDeformableSurface->release();
		PX_EXT_PINNED_MEMORY_FREE(*mCudaContextManager, mPositionsInvMass);
	}

	void copyDeformedVerticesFromGPUAsync(CUstream stream)
	{	
		physx::PxScopedCudaLock _lock(*mCudaContextManager);
		mCudaContextManager->getCudaContext()->memcpyDtoHAsync(mPositionsInvMass, reinterpret_cast<CUdeviceptr>(mDeformableSurface->getPositionInvMassBufferD()), mTriangleMesh->getNbVertices() * sizeof(physx::PxVec4), stream);
	}

	void copyDeformedVerticesFromGPU()
	{	
		physx::PxScopedCudaLock _lock(*mCudaContextManager);
		mCudaContextManager->getCudaContext()->memcpyDtoH(mPositionsInvMass, reinterpret_cast<CUdeviceptr>(mDeformableSurface->getPositionInvMassBufferD()), mTriangleMesh->getNbVertices() * sizeof(physx::PxVec4));
	}


	physx::PxVec4* mPositionsInvMass;
	physx::PxDeformableSurface* mDeformableSurface;
	physx::PxCudaContextManager* mCudaContextManager;
	physx::PxTriangleMesh* mTriangleMesh;
};

#endif // PHYSX_SNIPPET_DEFORMABLE_SURFACE_H
