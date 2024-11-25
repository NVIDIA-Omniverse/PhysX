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

#ifndef PHYSX_SNIPPET_DEFORMABLE_VOLUME_SKINNING_H
#define PHYSX_SNIPPET_DEFORMABLE_VOLUME_SKINNING_H

#include "PxPhysicsAPI.h"
#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"
#include "extensions/PxCudaHelpersExt.h"

struct SkinnedMesh
{
	physx::PxArray<physx::PxVec3> mVertices;
	physx::PxArray<physx::PxU32> mTriangles;
};

struct BasePostSolveCallback : physx::PxPostSolveCallback
{
	virtual void synchronize() = 0;
	virtual physx::PxVec3* getSkinnedVertices(physx::PxU32 deformableVolumeIndex) = 0;
};

class DeformableVolume
{
public:
	DeformableVolume() :
		mPositionsInvMass(NULL),
		mDeformableVolume(NULL),
		mCudaContextManager(NULL)
	{ }

	DeformableVolume(physx::PxDeformableVolume* deformableVolume, physx::PxCudaContextManager* cudaContextManager) :
		mDeformableVolume(deformableVolume),
		mCudaContextManager(cudaContextManager)
	{
		mPositionsInvMass = PX_EXT_PINNED_MEMORY_ALLOC(physx::PxVec4, *cudaContextManager, deformableVolume->getCollisionMesh()->getNbVertices());
	}

	~DeformableVolume()
	{
	}

	void release()
	{
		if (mDeformableVolume)
			mDeformableVolume->release();

		PX_EXT_PINNED_MEMORY_FREE(*mCudaContextManager, mPositionsInvMass);
	}

	void copyDeformedVerticesFromGPUAsync(CUstream stream)
	{
		physx::PxTetrahedronMesh* tetMesh = mDeformableVolume->getCollisionMesh();

		physx::PxScopedCudaLock _lock(*mCudaContextManager);
		mCudaContextManager->getCudaContext()->memcpyDtoHAsync(mPositionsInvMass, reinterpret_cast<CUdeviceptr>(mDeformableVolume->getPositionInvMassBufferD()), tetMesh->getNbVertices() * sizeof(physx::PxVec4), stream);
	}

	void copyDeformedVerticesFromGPU()
	{
		physx::PxTetrahedronMesh* tetMesh = mDeformableVolume->getCollisionMesh();

		physx::PxScopedCudaLock _lock(*mCudaContextManager);
		mCudaContextManager->getCudaContext()->memcpyDtoH(mPositionsInvMass, reinterpret_cast<CUdeviceptr>(mDeformableVolume->getPositionInvMassBufferD()), tetMesh->getNbVertices() * sizeof(physx::PxVec4));
	}


	physx::PxVec4* mPositionsInvMass;
	physx::PxDeformableVolume* mDeformableVolume;
	physx::PxCudaContextManager* mCudaContextManager;
};

#endif
