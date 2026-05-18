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

#ifndef PXG_FEM_CORE_H
#define PXG_FEM_CORE_H

#include "PxgNonRigidCoreCommon.h"
#include "PxgDeformableContactInfo.h"
#include "PxgDeformableConstraints.h"
#include "foundation/PxVec4.h"

namespace physx
{
	// Forward declarations from PxgSimulationCoreDesc.h
	struct PxgPrePrepDesc;
	struct PxgSolverCoreDesc;
	struct PxgSolverSharedDescBase;
	struct PxgArticulationCoreDesc;

	PX_COMPILE_TIME_ASSERT(sizeof(PxgFemFemContactInfo) == sizeof(PxgFemOtherContactInfo));

	class PxgFEMCore : public PxgNonRigidCore
	{
	public:

		PxgFEMCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
			PxgAllocatorDesc& allocDesc, PxgSimulationController* simController,
			PxgGpuContext* context, const PxU32 maxContacts, const PxU32 collisionStackSize, bool isTGS, PxsHeapStats::Enum statType);

		virtual ~PxgFEMCore();

		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getRigidContacts() { return mRigidContactPointBuf; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getRigidNormalPens() { return mRigidContactNormalPenBuf; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getRigidBarycentrics() { return mRigidContactBarycentricBuf; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgFemOtherContactInfo>& getRigidContactInfos() { return mRigidContactInfoBuf; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getRigidContactCount() { return mRigidTotalContactCountBuf; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getPrevRigidContactCount() { return mRigidPrevContactCountBuf; }

		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getFemContacts() { return mFemContactPointBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getFemNormalPens() { return mFemContactNormalPenBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getFemBarycentrics0() { return mFemContactBarycentric0Buffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getFemBarycentrics1() { return mFemContactBarycentric1Buffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgFemFemContactInfo>& getVolumeContactOrVTContactInfos() { return mVolumeContactOrVTContactInfoBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgFemFemContactInfo>& getEEContactInfos() { return mEEContactInfoBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getVolumeContactOrVTContactCount() { return mVolumeContactOrVTContactCountBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getEEContactCount() { return mEEContactCountBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getPrevFemContactCount() { return mPrevFemContactCountBuffer; }

		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getParticleContacts() { return mParticleContactPointBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getParticleNormalPens() { return mParticleContactNormalPenBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getParticleBarycentrics() { return mParticleContactBarycentricBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgFemOtherContactInfo>& getParticleContactInfos() { return mParticleContactInfoBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getParticleContactCount() { return mParticleTotalContactCountBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getPrevParticleContactCount() { return mPrevParticleContactCountBuffer; }

		PX_FORCE_INLINE PxgDevicePointer<PxReal> getSpeculativeCCDContactOffset() { return mSpeculativeCCDContactOffset.getTypedDevicePtr(); }

		// Create a contact writer initialized with this core's buffers
		PxgFEMContactWriter createContactWriter(bool useParticleForSorting = false);

		void	reserveRigidDeltaVelBuf(PxU32 newCapacity);

		void	reorderRigidContacts();

		void copyContactCountsToHost();

	protected:		
		
		void accumulateRigidDeltas(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd, 
			PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd,
			PxgDevicePointer<PxNodeIndex> rigidIdsd, PxgDevicePointer<PxU32> numIdsd, CUstream stream, bool isTGS);

		//rigid body and fem contacts
		PxgTypedCudaBuffer<float4>		mRigidContactPointBuf;				//float4
		PxgTypedCudaBuffer<float4>		mRigidContactNormalPenBuf;			//float4
		PxgTypedCudaBuffer<float4>		mRigidContactBarycentricBuf;		//float4
		PxgTypedCudaBuffer<PxgFemOtherContactInfo> mRigidContactInfoBuf;
		PxgTypedCudaBuffer<PxU32>		mRigidTotalContactCountBuf;			//PxU32
		PxgTypedCudaBuffer<PxU32>		mRigidPrevContactCountBuf;			//PxU32

		PxgTypedCudaBuffer<float4>		mRigidSortedContactPointBuf;
		PxgTypedCudaBuffer<float4>		mRigidSortedContactNormalPenBuf;
		PxgTypedCudaBuffer<float4>		mRigidSortedContactBarycentricBuf;	//float4
		PxgTypedCudaBuffer<PxU64>		mRigidSortedRigidIdBuf;
		PxgTypedCudaBuffer<PxgFemOtherContactInfo> mRigidSortedContactInfoBuf;

		// Reference count of each rigid body that interacts with deformable objects.
		// A single rigid body can have multiple reference counts when it is in contact with multiple triangles, tetrahedra, vertices, etc.
		// from deformable surfaces or deformable bodies. Currently, this is used only for contact constraints, but it can also be used for
		// attachment constraints.
		PxgTypedCudaBuffer<PxU32>		mFemRigidReferenceCount;

		//fem vs fem and fem self collision contacts
		PxgTypedCudaBuffer<float4>		mFemContactPointBuffer;				//float4
		PxgTypedCudaBuffer<float4>		mFemContactNormalPenBuffer;			//float4
		PxgTypedCudaBuffer<float4>		mFemContactBarycentric0Buffer;		//float4
		PxgTypedCudaBuffer<float4>		mFemContactBarycentric1Buffer;		//float4
		PxgTypedCudaBuffer<PxgFemFemContactInfo> mVolumeContactOrVTContactInfoBuffer;
		PxgTypedCudaBuffer<PxgFemFemContactInfo> mEEContactInfoBuffer;
		PxgTypedCudaBuffer<PxU32>		mVolumeContactOrVTContactCountBuffer;
		PxgTypedCudaBuffer<PxU32>		mEEContactCountBuffer;
		PxgTypedCudaBuffer<PxU32>		mPrevFemContactCountBuffer;

		PxgTypedCudaBuffer<PxReal>		mSpeculativeCCDContactOffset;

		//fem body vs particle system collision contacts
		PxgTypedCudaBuffer<float4>		mParticleContactPointBuffer;		//float4
		PxgTypedCudaBuffer<float4>		mParticleContactNormalPenBuffer;	//float4
		PxgTypedCudaBuffer<float4>		mParticleContactBarycentricBuffer;	//float4
		PxgTypedCudaBuffer<PxgFemOtherContactInfo> mParticleContactInfoBuffer;
		PxgTypedCudaBuffer<PxU32>		mParticleTotalContactCountBuffer;
		PxgTypedCudaBuffer<PxU32>		mPrevParticleContactCountBuffer;

		PxgTypedCudaBuffer<float4>		mParticleSortedContactPointBuffer;
		PxgTypedCudaBuffer<float4>		mParticleSortedContactBarycentricBuffer; //float4
		PxgTypedCudaBuffer<float4>		mParticleSortedContactNormalPenBuffer;
		PxgTypedCudaBuffer<PxgFemOtherContactInfo> mParticleSortedContactInfoBuffer;


		//contact prep buffer
		PxgTypedCudaBuffer<PxgFemRigidConstraintBlock> mRigidConstraintBuf;		//constraint prep for rigid body vs fem
		PxgCudaBuffer					mFemConstraintBuf;			//constraint prep for fem vs fem(including self collision)
		PxgTypedCudaBuffer<PxgFEMParticleConstraintBlock> mParticleConstraintBuf;		//constraint prep for particle vs fem

		//To do: ideally, we want to use two separate stream to solve the rigid body collision
		PxgTypedCudaBuffer<PxReal>		mRigidFEMAppliedForcesBuf;

		PxgTypedCudaBuffer<float4>		mFemAppliedForcesBuf; //applied force for fem due to collision between fem and fem or self collision
		
		PxgTypedCudaBuffer<float4>		mParticleAppliedFEMForcesBuf;  //applied force for fem due to collision between particle system and fem
		PxgTypedCudaBuffer<float4>		mParticleAppliedParticleForcesBuf; //applied force for particle system due to collision between particle system and fem

		PxgTypedCudaBuffer<float4>		mRigidDeltaVelBuf;

		//Temp buffer to accumulate rigid delta velocity changes
		PxgTypedCudaBuffer<PxVec4>		mTempBlockDeltaVelBuf;
		PxgTypedCudaBuffer<PxU64>		mTempBlockRigidIdBuf;

		//Temp buffer for sorted particle contacts 
		PxgCudaBuffer					mTempCellsHistogramBuf;
		PxgTypedCudaBuffer<PxU32>		mTempBlockCellsHistogramBuf;
		PxgTypedCudaBuffer<PxU32>		mTempHistogramCountBuf;

		bool							mIsTGS;

		struct ContactCounts
		{
			enum Enum
			{
				eRIGID					= 0,
				eVOLUME_CONTACTOR_VT	= 1,
				eEDGE_EDGE				= 2,
				ePARTICLE				= 3,
				eCOUNT					= 4
			};
		};
		Cm::PinnableArray<PxU32>		mContactCountsPrevTimestep;

#if PX_ENABLE_SIM_STATS
		PxU32							mContactCountStats; // for simStats.
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

		CUevent							mFinalizeEvent;
	};
}

#endif