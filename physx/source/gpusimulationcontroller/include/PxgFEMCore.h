// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_FEM_CORE_H
#define PXG_FEM_CORE_H

#include "PxgNonRigidCoreCommon.h"
#include "PxgSimulationCoreDesc.h"
#include "PxgDeformableContactWriters.h"
#include "PxgDeformableConstraints.h"
#include "foundation/PxVec4.h"

namespace physx
{
	// Forward declarations from PxgSolverCoreDesc.h and PxgArticulationCoreDesc.h
	struct PxgPrePrepDesc;
	struct PxgSolverCoreDesc;
	struct PxgSolverSharedDescBase;
	struct PxgArticulationCoreDesc;

	// The rigid material buffers store the index as a PxU16.
	PX_COMPILE_TIME_ASSERT(sizeof(PxMaterialTableIndex) <= sizeof(PxU16));

	class PxgFEMCore : public PxgNonRigidCore
	{
	public:

		PxgFEMCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
			PxgAllocatorDesc& allocDesc, PxgSimulationController* simController,
			PxgGpuContext* context, const PxU32 maxContacts, const PxU32 collisionStackSize, bool isTGS, PxsHeapStats::Enum statType);

		virtual ~PxgFEMCore();

		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getRigidDeformableContacts() { return mRigidDeformableContactPointBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getRigidDeformableNormalPens() { return mRigidDeformableContactNormalPenBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getRigidDeformableBarycentrics() { return mRigidDeformableContactBarycentricBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgDeformablePairInfo>& getRigidDeformableContactInfos() { return mRigidDeformableContactInfoBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU16>& getRigidDeformableContactRigidMaterials() { return mRigidDeformableContactRigidMaterialBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getRigidDeformableContactCount() { return mRigidDeformableContactCountBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getRigidDeformablePrevContactCount() { return mRigidDeformablePrevContactCountBuffer; }

		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getParticleDeformableContacts() { return mParticleDeformableContactPointBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getParticleDeformableNormalPens() { return mParticleDeformableContactNormalPenBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getParticleDeformableBarycentrics() { return mParticleDeformableContactBarycentricBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgDeformablePairInfo>& getParticleDeformableContactInfos() { return mParticleDeformableContactInfoBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getParticleDeformableContactCount() { return mParticleDeformableContactCountBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getParticleDeformablePrevContactCount() { return mParticleDeformablePrevContactCountBuffer; }

		PX_FORCE_INLINE PxgDevicePointer<PxReal> getSpeculativeCCDContactOffset() { return mSpeculativeCCDContactOffset.getTypedDevicePtr(); }

		// Create a contact writer initialized with this core's buffers
		PxgRigidDbContactWriter createRigidDbContactWriter();
		PxgParticleDbContactWriter createParticleDbContactWriter();

		void	reserveRigidDeltaVelBuf(PxU32 newCapacity);

		void	reorderRigidContacts();

	protected:

		// The counts common to every FEM core. Each core's copyContactCountsToHost() adds its own
		// deformable-vs-deformable counts on top.
		void copyRigidAndParticleContactCountsToHost();


		void accumulateRigidDeltas(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd,
			PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd, PxgDevicePointer<PxNodeIndex> rigidIdsd,
			PxgDevicePointer<PxU32> numIdsd, CUstream stream, bool isTGS);

		PxgTypedCudaBuffer<float4>					mRigidDeformableContactPointBuffer;
		PxgTypedCudaBuffer<float4>					mRigidDeformableContactNormalPenBuffer;
		PxgTypedCudaBuffer<float4>					mRigidDeformableContactBarycentricBuffer;
		PxgTypedCudaBuffer<PxgDeformablePairInfo>	mRigidDeformableContactInfoBuffer;
		PxgTypedCudaBuffer<PxU16>					mRigidDeformableContactRigidMaterialBuffer;
		PxgTypedCudaBuffer<PxU32>					mRigidDeformableContactCountBuffer;
		PxgTypedCudaBuffer<PxU32>					mRigidDeformablePrevContactCountBuffer;
		Cm::PinnableObject<PxU32>					mRigidDeformablePrevContactCountPinned;

		PxgTypedCudaBuffer<float4>					mRigidDeformableSortedContactPointBuffer;
		PxgTypedCudaBuffer<float4>					mRigidDeformableSortedContactNormalPenBuffer;
		PxgTypedCudaBuffer<float4>					mRigidDeformableSortedContactBarycentricBuffer;
		PxgTypedCudaBuffer<PxgDeformablePairInfo>	mRigidDeformableSortedContactInfoBuffer;
		PxgTypedCudaBuffer<PxU16>					mRigidDeformableSortedContactRigidMaterialBuffer;
		PxgTypedCudaBuffer<PxU64>					mRigidDeformableSortedRigidIdBuffer;

		PxgTypedCudaBuffer<PxgRigidDbContactBlock>	mRigidDeformableContactBlocks;
		//To do: ideally, we want to use two separate stream to solve the rigid body collision
		PxgTypedCudaBuffer<float2>		mRigidDeformableAppliedForcesBuffer; // (accumulated normal lambda, accumulated tangent lambda) per contact

		// Reference count of each rigid body that interacts with deformable objects.
		// A single rigid body can have multiple reference counts when it is in contact with multiple triangles, tetrahedra, vertices, etc.
		// from deformable surfaces or deformable bodies. Currently, this is used only for contact constraints, but it can also be used for
		// attachment constraints.
		PxgTypedCudaBuffer<PxU32>		mRigidDeformableRefCountBuffer;

		PxgTypedCudaBuffer<float4>					mParticleDeformableContactPointBuffer;
		PxgTypedCudaBuffer<float4>					mParticleDeformableContactNormalPenBuffer;
		PxgTypedCudaBuffer<float4>					mParticleDeformableContactBarycentricBuffer;
		PxgTypedCudaBuffer<PxgDeformablePairInfo>	mParticleDeformableContactInfoBuffer;
		PxgTypedCudaBuffer<PxU32>					mParticleDeformableContactCountBuffer;
		PxgTypedCudaBuffer<PxU32>					mParticleDeformablePrevContactCountBuffer;
		Cm::PinnableObject<PxU32>					mParticleDeformablePrevContactCountPinned;

		PxgTypedCudaBuffer<float4>					mParticleDeformableSortedContactPointBuffer;
		PxgTypedCudaBuffer<float4>					mParticleDeformableSortedContactNormalPenBuffer;
		PxgTypedCudaBuffer<float4>					mParticleDeformableSortedContactBarycentricBuffer;
		PxgTypedCudaBuffer<PxgDeformablePairInfo>	mParticleDeformableSortedContactInfoBuffer;

		PxgTypedCudaBuffer<PxgParticleDbContactBlock>	mParticleDeformableContactBlocks;
		PxgTypedCudaBuffer<float2>		mParticleDeformableAppliedDeformableForcesBuffer;  //applied force on the deformable
		PxgTypedCudaBuffer<float2>		mParticleDeformableAppliedParticleForcesBuffer; //applied force on the particle system

		PxgTypedCudaBuffer<PxReal>		mSpeculativeCCDContactOffset;

		PxgTypedCudaBuffer<float4>		mRigidDeltaVelBuf;

		//Temp buffer to accumulate rigid delta velocity changes
		PxgTypedCudaBuffer<PxVec4>		mTempBlockDeltaVelBuf;
		PxgTypedCudaBuffer<PxU64>		mTempBlockRigidIdBuf;

		//Temp buffer for sorted particle contacts 
		PxgCudaBuffer					mTempCellsHistogramBuf;
		PxgTypedCudaBuffer<PxU32>		mTempBlockCellsHistogramBuf;
		PxgTypedCudaBuffer<PxU32>		mTempHistogramCountBuf;

		bool							mIsTGS;

#if PX_ENABLE_SIM_STATS
		PxU32							mContactCountStats; // for simStats.
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

		CUevent							mFinalizeEvent;
	};
}

#endif