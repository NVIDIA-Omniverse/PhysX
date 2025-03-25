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

#ifndef PXG_JOINT_MANAGER_H
#define PXG_JOINT_MANAGER_H

#include "foundation/PxPinnedArray.h"
#include "foundation/PxHashMap.h"
#include "CmIDPool.h"
#include "PxgD6JointData.h"
#include "PxgConstraintPrep.h"
#include "PxgConstraintIdMap.h"

namespace physx
{
	namespace IG
	{
		class IslandSim;
		class CPUExternalData;
		class GPUExternalData;
	}

	namespace Dy
	{
		struct Constraint;
	}

	struct PxgSolverConstraintManagerConstants;

	//This manager should separate the joint types because we just support D6Joint in the constaint pre-prepare code
	class PxgJointManager
	{
	public:
		typedef PxPinnedArray<PxgConstraintIdMapEntry> ConstraintIdMap;

		PxgJointManager(const PxVirtualAllocator& allocator, bool isDirectGpuApiEnabled);
		~PxgJointManager();

		void	reserveMemory(PxU32 maxConstraintRows);
		void	reserveMemoryPreAddRemove();  // reserveMemory() above can not be used because it gets called after 
		                                      // joints get added/removed during constraint partitioning

		void	registerJoint(const Dy::Constraint& constraint);
		void	removeJoint(PxU32 edgeIndex, PxArray<PxU32>& jointIndices, const IG::CPUExternalData& islandSimCpuData, const IG::GPUExternalData& islandSimGpuData);
		void	addJoint(	PxU32 edgeIndex, const Dy::Constraint* constraint, IG::IslandSim& islandSim, PxArray<PxU32>& jointIndices, 
							PxPinnedArray<PxgSolverConstraintManagerConstants>& managerIter, PxU32 uniqueId);
		void	updateJoint(PxU32 edgeIndex, const Dy::Constraint* constraint);
		void	update(PxArray<PxU32>& jointOutputIndex);
		void	reset();
		PxU32	getGpuNbRigidConstraints();
		PxU32	getGpuNbArtiConstraints();
		PxU32	getCpuNbRigidConstraints();
		PxU32	getCpuNbArtiConstraints();
		PxU32	getGpuNbActiveRigidConstraints(); 
		PxU32	getGpuNbActiveArtiConstraints();

		PX_FORCE_INLINE	const PxArray<const Dy::Constraint*>&		getCpuRigidConstraints()			const	{ return mCpuRigidConstraints;				}
		PX_FORCE_INLINE	const PxArray<const Dy::Constraint*>&		getCpuArtiConstraints()				const	{ return mCpuArtiConstraints;				}

		PX_FORCE_INLINE	const PxInt32ArrayPinned&					getDirtyGPURigidJointDataIndices()	const	{ return mDirtyGPURigidJointDataIndices;	}
		PX_FORCE_INLINE	const PxInt32ArrayPinned&					getDirtyGPUArtiJointDataIndices()	const	{ return mDirtyGPUArtiJointDataIndices;		}

		PX_FORCE_INLINE	const PxPinnedArray<PxgD6JointData>&		getGpuRigidJointData()				const	{ return mGpuRigidJointData;				}
		PX_FORCE_INLINE	const PxPinnedArray<PxgD6JointData>&		getGpuArtiJointData()				const	{ return mGpuArtiJointData;					}

		PX_FORCE_INLINE	const PxPinnedArray<PxgConstraintPrePrep>&	getGpuRigidJointPrePrep()			const	{ return mGpuRigidJointPrePrep;				}
		PX_FORCE_INLINE	const PxPinnedArray<PxgConstraintPrePrep>&	getGpuArtiJointPrePrep()			const	{ return mGpuArtiJointPrePrep;				}

		// PT: these ones are contained in this class but actually filled by external code, PxgJointManager doesn't touch them.(*)
		PX_FORCE_INLINE	PxPinnedArray<PxgConstraintData>&			getCpuRigidConstraintData()					{ return mCpuRigidConstraintData;			}
		PX_FORCE_INLINE	PxPinnedArray<PxgConstraintData>&			getCpuArtiConstraintData()					{ return mCpuArtiConstraintData;			}
		PX_FORCE_INLINE	PxPinnedArray<Px1DConstraint>&				getCpuRigidConstraintRows()					{ return mCpuRigidConstraintRows;			}
		PX_FORCE_INLINE	PxPinnedArray<Px1DConstraint>&				getCpuArtiConstraintRows()					{ return mCpuArtiConstraintRows;			}

		PX_FORCE_INLINE	const ConstraintIdMap&						getGpuConstraintIdMapHost()			const	{ return mGpuConstraintIdMapHost;			}

		PX_FORCE_INLINE	bool getAndClearConstraintIdMapDirtyFlag()
		{
			const bool isDirty = mIsGpuConstraintIdMapDirty;
			mIsGpuConstraintIdMapDirty = false;
			return isDirty;
		}

		private:

		PxHashMap<PxU32, PxU32> mGpuRigidConstraintIndices;
		PxHashMap<PxU32, PxU32> mGpuArtiConstraintIndices;
		PxHashMap<PxU32, PxU32> mCpuRigidConstraintIndices;
		PxHashMap<PxU32, PxU32> mCpuArtiConstraintIndices;

		PxArray<const Dy::Constraint*> mCpuRigidConstraints;
		PxArray<const Dy::Constraint*> mCpuArtiConstraints;

		PxArray<PxU32> mCpuRigidUniqueIndex;
		PxArray<PxU32> mCpuArtiUniqueIndex;

		PxArray<PxU32> mCpuRigidConstraintEdgeIndices;
		PxArray<PxU32> mCpuArtiConstraintEdgeIndices;

		PxPinnedArray<PxgD6JointData>		mGpuRigidJointData;		// this is the input (PxgD6JointData) for rigid body we need to DMA to GPU so that GPU can fill in PxgConstraintData
		PxPinnedArray<PxgD6JointData>		mGpuArtiJointData;		// this is the input (PxgD6JointData) for articulation we need to DMA to GPU so that GPU can fill in PxgConstraintData
		PxPinnedArray<PxgConstraintPrePrep>	mGpuRigidJointPrePrep;	// this is the input (PxgConstraintPrePrep) for rigid body we need to DMA to GPU so that GPU can fill in PxgConstraintData
		PxPinnedArray<PxgConstraintPrePrep>	mGpuArtiJointPrePrep;	// this is the input (PxgConstraintPrePrep) for articulation we need to DMA to GPU so that GPU can fill in PxgConstraintData

		PxPinnedArray<PxgConstraintData>	mCpuRigidConstraintData;	// (*) this need to append to the GPU result (PxgConstraintData) after the first past pre-prepare code
		PxPinnedArray<Px1DConstraint>		mCpuRigidConstraintRows;	// (*) this need to append to the GPU result (Px1DConstraint) after the first past pre-prepare code

		PxPinnedArray<PxgConstraintData>	mCpuArtiConstraintData;		// (*) this need to append to the GPU result (PxgConstraintData) after the first past pre-prepare code
		PxPinnedArray<Px1DConstraint>		mCpuArtiConstraintRows;		// (*) this need to append to the GPU result (Px1DConstraint) after the first past pre-prepare code

		PxInt32ArrayPinned					mDirtyGPURigidJointDataIndices;	// the dirty list indices of PxgD6JointData
		PxInt32ArrayPinned					mDirtyGPUArtiJointDataIndices;	// the dirty list indices of PxgD6JointData

		ConstraintIdMap mGpuConstraintIdMapHost;  // See PxgConstraintIdMapEntry for details. Only used when direct GPU API is enabled and
		                                          // for joint/constraints that have the shader run on GPU.

		PxHashMap<PxU32, PxU32> mEdgeIndexToGpuConstraintIdMap;  // Get from edge index to constraint ID. Only used when direct GPU API is
		                                                         // enabled and for joint/constraints that have the shader run on GPU.
		
		Cm::IDPool			mGpuRigidJointDataIDPool; //each PxgD6JointData has an unique id. We can recycle the id when a joint has been removed from the joint manager
		Cm::IDPool			mGpuArtiJointDataIDPool;
		public:
		// PT: not sure why these are here, it's computed by PxgGpuContext at the same time it fills the CPU constraint data (*)
		PxI32				mNbCpuRigidConstraintRows;
		PxI32				mNbCpuArtiConstraintRows;

	private:
		PxU32 mMaxConstraintId;
		// Tracks all-time highest constraint ID to reserve sufficient space for mGpuConstraintIdMapHost.

		bool mIsGpuConstraintIdMapDirty;  // set to true when mGpuConstraintIdMapHost changed and needs to get sent to GPU

		const bool mIsDirectGpuApiEnabled;
	};
}

#endif
