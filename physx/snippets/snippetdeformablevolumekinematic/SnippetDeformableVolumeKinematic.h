// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PHYSX_SNIPPET_DEFORMABLE_VOLUME_KINEMATIC_H
#define PHYSX_SNIPPET_DEFORMABLE_VOLUME_KINEMATIC_H

#include "PxPhysicsAPI.h"
#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"
#include "extensions/PxCudaHelpersExt.h"

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
		PX_EXT_PINNED_MEMORY_FREE(*mCudaContextManager, mTargetPositionsH);
		PX_EXT_DEVICE_MEMORY_FREE(*mCudaContextManager, mTargetPositionsD);
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

	physx::PxVec4* mTargetPositionsH;
	physx::PxVec4* mTargetPositionsD;
	physx::PxU32 mTargetCount;
};

#endif // PHYSX_SNIPPET_DEFORMABLE_VOLUME_KINEMATIC_H
