// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_JOINT_MANAGER_H
#define PXG_JOINT_MANAGER_H

#include "CmPinnableArray.h"
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
	class PxCudaContext;

	//This manager should separate the joint types because we just support D6Joint in the constaint pre-prepare code
	class PxgJointManager
	{
		struct GpuJoints
		{
			GpuJoints(Cm::VirtualAllocatorCallback& hostMappedAlloc);

			PxHashMap<PxU32, PxU32>					mConstraintIndices;
			Cm::PinnableArray<PxgD6JointData>		mJointDataMapped;				// input (PxgD6JointData) we need to DMA to GPU so that GPU can fill in PxgConstraintData
			Cm::PinnableArray<PxgConstraintPrePrep>	mJointPrePrepMapped;			// input (PxgConstraintPrePrep) we need to DMA to GPU so that GPU can fill in PxgConstraintData
			Cm::PinnableArray<PxU32>				mDirtyIndicesMapped;			// the dirty list indices of PxgD6JointData
			Cm::IDPool								mIDPool;						// each PxgD6JointData has an unique id. We can recycle the id when a joint has been
																					// removed from the joint manager
		};

		struct CpuJoints
		{
			CpuJoints(Cm::VirtualAllocatorCallback& hostAlloc);

			PxHashMap<PxU32, PxU32>				mConstraintIndices;
			PxArray<const Dy::Constraint*>		mConstraints;
			PxArray<PxU32>						mUniqueIndex;
			PxArray<PxU32>						mConstraintEdgeIndices;
			Cm::PinnableArray<PxgConstraintData>	mConstraintData;	// (*) this need to append to the GPU result (PxgConstraintData) after
																	// the first past pre-prepare code
			Cm::PinnableArray<Px1DConstraint>		mConstraintRows;	// (*) this need to append to the GPU result (Px1DConstraint) after the
																	// first past pre-prepare code
			PxI32								mNbConstraintRows;	// PT: not sure why these are here, it's computed by PxgGpuContext at 
																	// the same time it fills the CPU constraint data (*)
		};

	public:
		typedef Cm::PinnableArray<PxgConstraintIdMapEntry> ConstraintIdMap;

		PxgJointManager(Cm::VirtualAllocatorCallback& hostAlloc, Cm::VirtualAllocatorCallback& hostMappedAlloc,
						bool isDirectGpuApiEnabled, PxCudaContext* cudaContext = NULL);

		~PxgJointManager();

		void	reserveMemory(PxU32 maxConstraintRows);
		void	reserveMemoryPreAddRemove();  // reserveMemory() above can not be used because it gets called after 
		                                      // joints get added/removed during constraint partitioning

		void	registerJoint(const Dy::Constraint& constraint);
		void	removeJoint(PxU32 edgeIndex, PxArray<PxU32>& jointIndices, const IG::CPUExternalData& islandSimCpuData, const IG::GPUExternalData& islandSimGpuData);
		void	addJoint(	PxU32 edgeIndex, const Dy::Constraint* constraint, IG::IslandSim& islandSim, PxArray<PxU32>& jointIndices, 
							Cm::PinnableArray<PxgSolverConstraintManagerConstants>& managerIter, PxU32 uniqueId);
		void	updateJoint(PxU32 edgeIndex, const Dy::Constraint* constraint);
		void	update(PxArray<PxU32>& jointOutputIndex);
		void	reset();
		PxU32	getGpuNbRigidConstraints();
		PxU32	getGpuNbArtiConstraints();
		PxU32	getCpuNbRigidConstraints();
		PxU32	getCpuNbArtiConstraints();
		PxU32	getGpuNbActiveRigidConstraints(); 
		PxU32	getGpuNbActiveArtiConstraints();

		PX_FORCE_INLINE	const PxArray<const Dy::Constraint*>&		getCpuRigidConstraints()			const	{ return mCpuRigidJoints.mConstraints;			}
		PX_FORCE_INLINE	const PxArray<const Dy::Constraint*>&		getCpuArtiConstraints()				const	{ return mCpuArtiJoints.mConstraints;			}

		PX_FORCE_INLINE	const Cm::PinnableArray<PxU32>&				getDirtyGPURigidJointDataIndices()	const	{ return mGpuRigidJoints.mDirtyIndicesMapped;	}
		PX_FORCE_INLINE	const Cm::PinnableArray<PxU32>&				getDirtyGPUArtiJointDataIndices()	const	{ return mGpuArtiJoints.mDirtyIndicesMapped;	}

		PX_FORCE_INLINE	const Cm::PinnableArray<PxgD6JointData>&		getGpuRigidJointData()				const	{ return mGpuRigidJoints.mJointDataMapped;		}
		PX_FORCE_INLINE	const Cm::PinnableArray<PxgD6JointData>&		getGpuArtiJointData()				const	{ return mGpuArtiJoints.mJointDataMapped;		}

		PX_FORCE_INLINE	const Cm::PinnableArray<PxgConstraintPrePrep>&	getGpuRigidJointPrePrep()			const	{ return mGpuRigidJoints.mJointPrePrepMapped;	}
		PX_FORCE_INLINE	const Cm::PinnableArray<PxgConstraintPrePrep>&	getGpuArtiJointPrePrep()			const	{ return mGpuArtiJoints.mJointPrePrepMapped;	}

		// PT: these ones are contained in this class but actually filled by external code, PxgJointManager doesn't touch them.(*)
		PX_FORCE_INLINE	Cm::PinnableArray<PxgConstraintData>&			getCpuRigidConstraintData()					{ return mCpuRigidJoints.mConstraintData;		}
		PX_FORCE_INLINE	Cm::PinnableArray<PxgConstraintData>&			getCpuArtiConstraintData()					{ return mCpuArtiJoints.mConstraintData;		}
		PX_FORCE_INLINE	Cm::PinnableArray<Px1DConstraint>&				getCpuRigidConstraintRows()					{ return mCpuRigidJoints.mConstraintRows;		}
		PX_FORCE_INLINE	Cm::PinnableArray<Px1DConstraint>&				getCpuArtiConstraintRows()					{ return mCpuArtiJoints.mConstraintRows;		}

		PX_FORCE_INLINE PxI32										getNbCpuRigidConstraintRows()		const	{ return mCpuRigidJoints.mNbConstraintRows;		}
		PX_FORCE_INLINE PxI32										getNbCpuArtiConstraintRows()		const	{ return mCpuArtiJoints.mNbConstraintRows;		}
		PX_FORCE_INLINE	PxI32&										getNbCpuRigidConstraintRowsRef()			{ return mCpuRigidJoints.mNbConstraintRows;		}
		PX_FORCE_INLINE	PxI32&										getNbCpuArtiConstraintRowsRef()				{ return mCpuArtiJoints.mNbConstraintRows;		}

		PX_FORCE_INLINE	const ConstraintIdMap&						getGpuConstraintIdMapHost()			const	{ return mGpuConstraintIdMapHost;				}

		PX_FORCE_INLINE	bool getAndClearConstraintIdMapDirtyFlag()
		{
			const bool isDirty = mIsGpuConstraintIdMapDirty;
			mIsGpuConstraintIdMapDirty = false;
			return isDirty;
		}

	private:

		GpuJoints							mGpuRigidJoints;
		GpuJoints							mGpuArtiJoints;

		CpuJoints							mCpuRigidJoints;
		CpuJoints							mCpuArtiJoints;

		ConstraintIdMap						mGpuConstraintIdMapHost;	// See PxgConstraintIdMapEntry for details. Only used when direct GPU API is enabled and
																		// for joint/constraints that have the shader run on GPU.

		PxHashMap<PxU32, PxU32>				mEdgeIndexToGpuConstraintIdMap;	// Get from edge index to constraint ID. Only used when direct GPU API is
																			// enabled and for joint/constraints that have the shader run on GPU.
		PxU32								mMaxConstraintId;			// Tracks all-time highest constraint ID to reserve sufficient space for mGpuConstraintIdMapHost.
		bool								mIsGpuConstraintIdMapDirty;	// set to true when mGpuConstraintIdMapHost changed and needs to get sent to GPU
		const bool							mIsDirectGpuApiEnabled;
		PxCudaContext*						mCudaContext;
	};
}

#endif
