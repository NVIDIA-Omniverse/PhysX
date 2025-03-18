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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef PXG_FEM_CORE_H
#define PXG_FEM_CORE_H

#include "PxgNonRigidCoreCommon.h"
#include "PxgSimulationCoreDesc.h"
#include "PxNodeIndex.h"
#include "foundation/PxSimpleTypes.h"

#include <vector_types.h>

namespace physx
{
	struct PxgPrePrepDesc;
	struct PxgSolverCoreDesc;
	struct PxgSolverSharedDescBase;
	struct PxgArticulationCoreDesc;

	//rigid vs soft body : pairInd0 is rigid , pairInd1 is soft body
	//particle vs soft body : pairInd0 is particle, pairInd1 is soft body
	//soft body vs soft body : pairInd0 is soft body, pairInd1 is soft body
	//soft body vs fem cloth : pairInd0 is soft body, pairInd1 is fem cloth
	//fem cloth vs fem cloth : pairInd0 is fem cloth, pairInd1 is fem cloth
	struct PX_ALIGN_PREFIX(16) PxgFemContactInfo
	{
		PxU64 pairInd0;
		PxU64 pairInd1;
	} 
	PX_ALIGN_SUFFIX(16);
	
	struct PX_ALIGN_PREFIX(16) PxgFemVsRigidContactInfo
	{
		PxU64 pairInd0; //Rigid
		PxU32 pairInd1; //Fem id
		PxU32 rigidMatInd;
	}
	PX_ALIGN_SUFFIX(16);

	PX_COMPILE_TIME_ASSERT(sizeof(PxgFemContactInfo) == sizeof(PxgFemVsRigidContactInfo));

	struct PxgFemRigidConstraintBlock
	{
		// resp represents the rigid-body term in the denominator of the impulse calculation (also referred to as the velocity multiplier
		// internally). Also refer to PBD (Position-Based Dynamics) papers.
		float4 raXn_resp[32];
		float4 raXnF0_resp[32];
		float4 raXnF1_resp[32];

		float4 normal_errorW[32];

		// Friction tangent + invMass of the rigid body (avoids needing to read the mass)
		// Second tangent can be found by cross producting normal with fricTan0
		float4 fricTan0_invMass0[32];
		float4 barycentric[32];
		PxReal maxPenBias[32];
	};

	struct PxgFEMParticleConstraintBlock
	{
		float4 normal_pen[32];
		float4 barycentric[32];
		PxReal velMultiplier[32];
	};

	struct PxgFEMRigidAttachmentConstraint
	{
		float4	baryOrType[32];
		float4	raXn0_biasW[32];
		float4	raXn1_biasW[32];
		float4	raXn2_biasW[32];
		float4	velMultiplierXYZ_invMassW[32];
		float4  low_high_limits[32];
		float4  axis_angle[32];
		PxU32	elemId[32];
		PxU64	rigidId[32];		//node index
	};

	struct PxgFEMFEMAttachmentConstraint
	{
		union
		{
			float4  low_high_limits[32];
			float4  low_high_angles[32];
		};

		union
		{
			float4  axis_angle[32];
			float4  attachmentBarycentric[32];
		};
		
		float4	barycentric0[32];
		float4	barycentric1[32];
		PxU64	elemId0[32]; //can be triangleId(cloth) or tetrahedron index
		PxU64	elemId1[32];//can be triangleId(cloth) or tetrahedron index
		float	constraintOffset[32];
	};


	class PxgFEMCore : public PxgNonRigidCore
	{
	public:

		PxgFEMCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
			PxgHeapMemoryAllocatorManager* heapMemoryManager, PxgSimulationController* simController,
			PxgGpuContext* context, const PxU32 maxContacts, const PxU32 collisionStackSize, bool isTGS, PxsHeapStats::Enum statType);

		virtual ~PxgFEMCore();

		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getRigidContacts() { return mRigidContactPointBuf; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getRigidNormalPens() { return mRigidContactNormalPenBuf; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getRigidBarycentrics() { return mRigidContactBarycentricBuf; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgFemContactInfo>& getRigidContactInfos() { return mRigidContactInfoBuf; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getRigidContactCount() { return mRigidTotalContactCountBuf; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getPrevRigidContactCount() { return mRigidPrevContactCountBuf; }

		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getFemContacts() { return mFemContactPointBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getFemNormalPens() { return mFemContactNormalPenBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getFemBarycentrics0() { return mFemContactBarycentric0Buffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getFemBarycentrics1() { return mFemContactBarycentric1Buffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgFemContactInfo>& getFemContactInfos() { return mFemContactInfoBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getFemContactCount() { return mFemTotalContactCountBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getPrevFemContactCount() { return mPrevFemContactCountBuffer; }

		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getParticleContacts() { return mParticleContactPointBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getParticleNormalPens() { return mParticleContactNormalPenBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<float4>& getParticleBarycentrics() { return mParticleContactBarycentricBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxgFemContactInfo>& getParticleContactInfos() { return mParticleContactInfoBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getParticleContactCount() { return mParticleTotalContactCountBuffer; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getPrevParticleContactCount() { return mPrevParticleContactCountBuffer; }

		PX_FORCE_INLINE PxgDevicePointer<PxReal> getSpeculativeCCDContactOffset() { return mSpeculativeCCDContactOffset.getTypedDevicePtr(); }

		void	reserveRigidDeltaVelBuf(PxU32 newCapacity);

		void	reorderRigidContacts();

		void copyContactCountsToHost();

	protected:		
		
		void accumulateRigidDeltas(PxgDevicePointer<PxgPrePrepDesc> prePrepDescd, PxgDevicePointer<PxgSolverCoreDesc> solverCoreDescd, 
			PxgDevicePointer<PxgSolverSharedDescBase> sharedDescd, PxgDevicePointer<PxgArticulationCoreDesc> artiCoreDescd,
			PxgDevicePointer<PxNodeIndex> rigidIdsd, PxgDevicePointer<PxU32> numIdsd, CUstream stream, CUevent waitEvent,
			bool isTGS);

		//rigid body and fem contacts
		PxgTypedCudaBuffer<float4>		mRigidContactPointBuf;				//float4
		PxgTypedCudaBuffer<float4>		mRigidContactNormalPenBuf;			//float4
		PxgTypedCudaBuffer<float4>		mRigidContactBarycentricBuf;		//float4
		PxgTypedCudaBuffer<PxgFemContactInfo> mRigidContactInfoBuf;			//PxgFemContactInfo
		PxgTypedCudaBuffer<PxU32>		mRigidTotalContactCountBuf;			//PxU32
		PxgTypedCudaBuffer<PxU32>		mRigidPrevContactCountBuf;			//PxU32

		PxgTypedCudaBuffer<float4>		mRigidSortedContactPointBuf;
		PxgTypedCudaBuffer<float4>		mRigidSortedContactNormalPenBuf;
		PxgTypedCudaBuffer<float4>		mRigidSortedContactBarycentricBuf;	//float4
		PxgTypedCudaBuffer<PxU64>		mRigidSortedRigidIdBuf;
		PxgTypedCudaBuffer<PxgFemContactInfo> mRigidSortedContactInfoBuf;	//PxgFemContactInfo

		PxgTypedCudaBuffer<float4>		mRigidLambdaNBuf;
		PxgTypedCudaBuffer<float4>		mFemLambdaNBuf;

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
		PxgTypedCudaBuffer<PxgFemContactInfo> mFemContactInfoBuffer;		//PxgFemContactInfo
		PxgTypedCudaBuffer<PxU32>		mFemTotalContactCountBuffer;
		PxgTypedCudaBuffer<PxU32>		mPrevFemContactCountBuffer;

		PxgTypedCudaBuffer<PxReal>		mSpeculativeCCDContactOffset;

		//fem body vs particle system collision contacts
		PxgTypedCudaBuffer<float4>		mParticleContactPointBuffer;		//float4
		PxgTypedCudaBuffer<float4>		mParticleContactNormalPenBuffer;	//float4
		PxgTypedCudaBuffer<float4>		mParticleContactBarycentricBuffer;	//float4
		PxgTypedCudaBuffer<PxgFemContactInfo> mParticleContactInfoBuffer;
		PxgTypedCudaBuffer<PxU32>		mParticleTotalContactCountBuffer;
		PxgTypedCudaBuffer<PxU32>		mPrevParticleContactCountBuffer;

		PxgTypedCudaBuffer<float4>		mParticleSortedContactPointBuffer;
		PxgTypedCudaBuffer<float4>		mParticleSortedContactBarycentricBuffer; //float4
		PxgTypedCudaBuffer<float4>		mParticleSortedContactNormalPenBuffer;
		PxgTypedCudaBuffer<PxgFemContactInfo> mParticleSortedContactInfoBuffer;


		//contact prep buffer
		PxgTypedCudaBuffer<PxgFemRigidConstraintBlock> mRigidConstraintBuf;		//constraint prep for rigid body vs fem
		PxgCudaBuffer					mFemConstraintBuf;			//constraint prep for fem vs fem(including self collision)
		PxgTypedCudaBuffer<PxgFEMParticleConstraintBlock> mParticleConstraintBuf;		//constraint prep for particle vs fem

		//To do: ideally, we want to use two separate stream to solve the rigid body collision
		PxgTypedCudaBuffer<float4>		mRigidAppliedRigidForcesBuf;	 //applied force for rigid body due to collision between fem and rigid body
		PxgTypedCudaBuffer<float4>		mRigidAppliedFEMForcesBuf;		//applied force for fem due to collision between fem and rigid body

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
		PxU32*							mRigidContactCountPrevTimestep; //Pinned memory
		PxU32*							mFemContactCountPrevTimestep; //Pinned memory
		PxU32*							mParticleContactCountPrevTimestep; //Pinned memory

#if PX_ENABLE_SIM_STATS
		PxU32							mContactCountStats; // for simStats.
#else
	PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif

		CUevent							mFinalizeEvent;
	};


	struct PxgFEMContactWriter
	{
		float4* outPoint;
		float4* outNormalPen;
		float4* outBarycentric;
		PxgFemContactInfo* outContactInfo;
		PxU32* totalContactCount;

		//Buffers for sorting. X either stands for Rigid or Particle
		PxU64* contactByX; //value
		PxU32* tempContactByX; //the lower 32 bit o value
		PxU32* contactIndexSortedByX; //rank
		PxU32* contactSortedByX; // AD: I'm not sure what we use this for. We do use the underlying buffer as the output buffer of RS_COPY_VALUE, so either we're skipping the radix sort in some cases or we write some unnecessary stuff here.

		PxU32 maxNumContacts;

		PxgFEMContactWriter(PxgFEMCore* femCore, bool useParticleForSorting = false)
		{
			//Ensure a rigid index has the same size as an encoded particle index
			PX_COMPILE_TIME_ASSERT(sizeof(PxNodeIndex) == sizeof(PxU64));
	
			if (useParticleForSorting)
			{
				outPoint = femCore->getParticleContacts().getTypedPtr();
				outNormalPen = femCore->getParticleNormalPens().getTypedPtr();
				outBarycentric = femCore->getParticleBarycentrics().getTypedPtr();
				outContactInfo = femCore->getParticleContactInfos().getTypedPtr();
				totalContactCount = femCore->getParticleContactCount().getTypedPtr();

				contactByX = femCore->getContactSortedByParticle().getTypedPtr();
				tempContactByX = femCore->getTempContactByParticle().getTypedPtr();
				contactIndexSortedByX = femCore->getContactRemapSortedByParticle().getTypedPtr();
				contactSortedByX = NULL; //Does not exist for particles
			}
			else
			{
				outPoint = femCore->getRigidContacts().getTypedPtr();
				outNormalPen = femCore->getRigidNormalPens().getTypedPtr();
				outBarycentric = femCore->getRigidBarycentrics().getTypedPtr();
				outContactInfo = femCore->getRigidContactInfos().getTypedPtr();
				totalContactCount = femCore->getRigidContactCount().getTypedPtr();

				contactByX = reinterpret_cast<PxU64*>(femCore->getContactByRigid().getTypedPtr());
				tempContactByX = femCore->getTempContactByRigid().getTypedPtr();
				contactIndexSortedByX = femCore->getContactRemapSortedByRigid().getTypedPtr();
				contactSortedByX = reinterpret_cast<PxU32*>(femCore->getContactSortedByRigid().getTypedPtr()); //Cast from larger type to smaller type - no memory overflow
			}
			maxNumContacts = femCore->mMaxContacts;
		}

		PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeContactCore(PxU32 index, const float4& contact, const float4& normalPen,	PxU64 rigidId)
		{
			if (index >= maxNumContacts)
				return false;

			contactByX[index] = rigidId;
			tempContactByX[index] = PxU32(rigidId & 0xffffffff);
			contactIndexSortedByX[index] = index;

			outPoint[index] = contact;
			outNormalPen[index] = normalPen;
			/*outContactInfo[index].pairInd0 = pairInd0;
			outContactInfo[index].pairInd1 = pairInd1;*/

			return true;
		}

		PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeContactNoBarycentric(PxU32 index, const float4& contact, const float4& normalPen,
			PxU64 pairInd0, PxU32 pairInd1, PxU64 rigidId)
		{
			if (index >= maxNumContacts)
				return false;

			contactByX[index] = rigidId;
			tempContactByX[index] = PxU32(rigidId & 0xffffffff);
			contactIndexSortedByX[index] = index;

			outPoint[index] = contact;
			outNormalPen[index] = normalPen;
			outContactInfo[index].pairInd0 = pairInd0;
			outContactInfo[index].pairInd1 = pairInd1;

			return true;
		}
		
		PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeRigidVsDeformableContactNoBarycentric(PxU32 index, const float4& contact, const float4& normalPen,
			PxU64 pairInd0, PxU32 pairInd1, PxU64 rigidId, PxU32 rigidBodyMaterialId)
		{
			if (index >= maxNumContacts)
				return false;

			contactByX[index] = rigidId;
			tempContactByX[index] = PxU32(rigidId & 0xffffffff);
			contactIndexSortedByX[index] = index;

			outPoint[index] = contact;
			outNormalPen[index] = normalPen;
			PxgFemVsRigidContactInfo* ptr = reinterpret_cast<PxgFemVsRigidContactInfo*>(&outContactInfo[index]);
			ptr->pairInd0 = pairInd0;
			ptr->pairInd1 = pairInd1;
			ptr->rigidMatInd = rigidBodyMaterialId;

			return true;
		}

		PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeContact(PxU32 index, const float4& contact, const float4& normalPen, const float4& barycentric,
			PxU64 pairInd0, PxU32 pairInd1, PxU64 id)
		{
			if (index >= maxNumContacts)
				return false;

			writeContactNoBarycentric(index, contact, normalPen, pairInd0, pairInd1, id);
			outBarycentric[index] = barycentric;			

			return true;
		}

		PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeContact(PxU32 index, const float4& contact, const float4& normalPen, const float4& barycentric,
			PxU64 pairInd0, PxU32 pairInd1, PxNodeIndex rigidId)
		{
			return writeContact(index, contact, normalPen, barycentric, pairInd0, pairInd1, rigidId.getInd());
		}	
		
		PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeRigidVsDeformableContact(PxU32 index, const float4& contact, const float4& normalPen, const float4& barycentric,
			PxU64 pairInd0, PxU32 pairInd1, PxU32 rigidMaterialIndex, PxNodeIndex rigidId)
		{
			bool result = writeContactCore(index, contact, normalPen, rigidId.getInd());
			if (result)
			{
				outBarycentric[index] = barycentric;
				PxgFemVsRigidContactInfo* femRigidInfo = reinterpret_cast<PxgFemVsRigidContactInfo*>(outContactInfo);
				femRigidInfo[index].pairInd0 = pairInd0;
				femRigidInfo[index].pairInd1 = pairInd1;
				femRigidInfo[index].rigidMatInd = rigidMaterialIndex;
			}
			return result;
		}

		PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeRigidVsDeformableContact32(PxU32 index, const float4& contact, const float4& normalPen, const float4& barycentric,
			PxU64 pairInd0, PxU32 pairInd1, PxU32 rigidMaterialIndex, PxU32 contactSortedByRigid_)
		{
			if (index >= maxNumContacts)
				return false;

			outPoint[index] = contact;
			outNormalPen[index] = normalPen;
			outBarycentric[index] = barycentric;

			PxgFemVsRigidContactInfo* femRigidInfo = reinterpret_cast<PxgFemVsRigidContactInfo*>(outContactInfo);
			femRigidInfo[index].pairInd0 = pairInd0;
			femRigidInfo[index].pairInd1 = pairInd1;
			femRigidInfo[index].rigidMatInd = rigidMaterialIndex;

			contactSortedByX[index] = contactSortedByRigid_;
			contactIndexSortedByX[index] = index;

			return true;
		}

		PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeContact32(PxU32 index, const float4& contact, const float4& normalPen, const float4& barycentric,
			PxU64 pairInd0, PxU32 pairInd1, PxU32 contactSortedByRigid_)
		{
			if (index >= maxNumContacts)
				return false;

			outPoint[index] = contact;
			outNormalPen[index] = normalPen;
			outBarycentric[index] = barycentric;
			outContactInfo[index].pairInd0 = pairInd0;
			outContactInfo[index].pairInd1 = pairInd1;

			contactSortedByX[index] = contactSortedByRigid_;
			contactIndexSortedByX[index] = index;

			return true;
		}
	};
}

#endif