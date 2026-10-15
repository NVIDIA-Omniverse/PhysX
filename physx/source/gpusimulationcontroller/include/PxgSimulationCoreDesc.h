// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_SIMULATION_CORE_DESC_H
#define	PXG_SIMULATION_CORE_DESC_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "PxNodeIndex.h"
#include "DyVArticulation.h"
#include "PxDeformableSurface.h"
#include "PxDeformableVolume.h"

#include <vector_types.h>

namespace physx
{
	struct PxgSolverBodySleepData;
	struct PxgShape;
	struct PxgBodySim;
	struct PxgBodySimVelocities;
	struct PxgShapeSim;
	struct PxgArticulationLink;
	struct PxgArticulationLinkProp;
	struct PxsCachedTransform;
	struct PxgBodySimVelocityUpdate;
	struct PxgD6JointData;
	struct PxgConstraintPrePrep;
	class PxgArticulation;
	class PxgArticulationTendon;
	class PxGpuSpatialTendonData;
	class PxGpuFixedTendonData;
	class PxGpuTendonAttachmentData;
	class PxgArticulationTendonElementFixedData;
	class PxGpuTendonJointCoefficientData;

	namespace Dy
	{
		struct ArticulationJointCore;
		struct ArticulationMimicJointCore;
		class ArticulationJointCoreData;
	}
	
	struct PxgNewBodiesDesc
	{
		const PxgBodySim*	mNewBodySim;
		PxgBodySim*			mBodySimBufferDeviceData;
		PxgBodySimVelocities*	mPrevVelocitiesBuffer;		// PdHC: Previous velocities buffer for acceleration computation
		PxU32				mNbNewBodies;	//number of newly added bodies
	};

	struct PX_ALIGN_PREFIX(16) PxgArticulationSimUpdate
	{
		PxU32 dirtyFlags;							//What data is updated (ArticulationDirtyFlag)
		PxU32 articulationIndex;					//which articulation on the GPU			
		PxU32 linkStartIndex;						//Where the link/joint data starts in memory
		PxU32 dofDataStartIndex;					//Where the dof data starts in memory, e.g. jointVelocity, jointPosition, jointForce etc.

		PxU32 spatialTendonStartIndex;				//Where the spatial tendon starts in memory
		PxU32 spatialTendonAttachmentStartIndex;	//Where the spatial tendon attachment starts in memory
		PxU32 fixedTendonStartIndex;				//Where the fixed tendon starts in memory
		PxU32 fixedTendonJointStartIndex;			//Where the fixed tendon joint starts in memory

		PxU32 mimicJointStartIndex;
		PxU32 pathToRootIndex;						//Where the pathToRoot array starts in memory

		PxU8 userFlags;								//(PxArticulationFlag)
		PxU8 pad[7];
		
	}PX_ALIGN_SUFFIX(16);

	// Wrapper to align syntactically with PxgDeformablePairInfo related encoder functions.
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU64 PxgEncodeRigidNode(const PxNodeIndex& nodeIndex)
	{
		return nodeIndex.getInd();
	}

	// Wrapper to align syntactically with PxgDeformablePairInfo related accessor functions.
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxNodeIndex PxgGetRigidNode(const PxU64 ref)
	{
		return PxNodeIndex(ref);
	}

	// The element filter reference of one shape of a PxRigidStatic. Every static shares PX_INVALID_NODE, so a
	// filter against one is keyed on its shapes' transform cache ids. The low word stays PX_INVALID_NODE, which
	// marks a static in a PxNodeIndex, so the reference differs from that of any body or articulation link. The
	// high word holds the transform cache id plus one, so it also differs from PxgEncodeRigidNode(PxNodeIndex()),
	// the world frame reference of attachments. Only filter pairs hold these references and none is decoded.
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU64 PxgEncodeStaticShapeRef(const PxU32 transformCacheId)
	{
		PX_ASSERT(transformCacheId != PX_INVALID_U32); // the one value whose increment would wrap onto the world reference
		return PxU64(PX_INVALID_NODE) | ((PxU64(transformCacheId) + 1) << 32);
	}

	// The element filter reference of a rigid shape, from the node index the shape maps to and its transform
	// cache id. Bodies and articulation links are keyed on the node, statics on the shape.
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU64 PxgEncodeRigidFilterRef(const PxNodeIndex& rigidId, const PxU32 transformCacheId)
	{
		return rigidId.isStaticBody() ? PxgEncodeStaticShapeRef(transformCacheId) : PxgEncodeRigidNode(rigidId);
	}

	/**
	\brief The layout of a deformable participant, and the kind of mesh feature it carries.

	    [deformable:30][active:1][invalid:1][type:3][localEdge:2][featureId:27]
	       63..34         bit 33    bit 32   31..29    28..27         26..0

	- deformable: the deformable's GPU index. Whether that is a volume or a surface is defined by the
	  referencing code. Written by PxgEncodeDeformable* and read by PxgGetDeformableId.
	- active: marks whether the contact pair is active in the solver, and may change during the solve.
	  Written by PxgSetActivePairInfo and read by PxgIsActivePairInfo.
	- invalid: marks a contact that is not valid, determined before the solve starts. Written by
	  PxgEncodeInvalidDeformableRef and PxgInvalidatePairInfo, read by PxgIsValidPairInfo.
	- type: mesh feature type, see the Enum below. Written by PxgEncodeDeformable* and read by
	  PxgGetDeformableFeatureType.
	- localEdge: enumerates an edge relative to a triangle, only relevant with eTRIANGLE_EDGE. Written by
	  PxgEncodeDeformableTriangleEdge and read by PxgGetDeformableEdgeLocalIndex.
	- featureId: the vertex, triangle or tetrahedron index, the triangle of a triangle edge, or
	  ANY_FEATURE for an element filter that matches any of them. Written by PxgEncodeDeformable* and
	  read by PxgGetDeformableFeatureId.
	*/
	struct PxgDeformableRef
	{
		enum Enum
		{
			eVERTEX = 0,
			eTRIANGLE,
			eTETRAHEDRON,
			eTRIANGLE_EDGE, // the only kind that fills in the local edge selector

			eLAST
		};

		// Static allocation of bits, compile time asserted below.
		static constexpr PxU32 TYPE_BITS = 3;
		static constexpr PxU32 LOCAL_EDGE_BITS = 2;
		static constexpr PxU32 LOCAL_EDGE_MASK = (1u << LOCAL_EDGE_BITS) - 1;
		static constexpr PxU32 FEATURE_ID_BITS = 32 - TYPE_BITS - LOCAL_EDGE_BITS;
		static constexpr PxU32 FEATURE_ID_MASK = (1u << FEATURE_ID_BITS) - 1;

		static constexpr PxU64 INVALID_FLAG = PxU64(1) << 32;
		static constexpr PxU64 ACTIVE_FLAG = PxU64(1) << 33;

		static constexpr PxU32 DEFORMABLE_SHIFT = 34;
		static constexpr PxU32 DEFORMABLE_BITS = 64 - DEFORMABLE_SHIFT;

		// Limits dictated by the public API macros, as opposed to FEATURE_ID_MASK, which is the field's
		// capacity.
		static constexpr PxU32 MAX_NB_DEFORMABLES = PX_MAX_NB_DEFORMABLE_SURFACE;
		static constexpr PxU32 MAX_NB_FEATURES = PX_MAX_NB_DEFORMABLE_SURFACE_TRI;

		// Reserved id rather than a flag, so that it survives a bitwise key comparison unchanged.
		static constexpr PxU32 ANY_FEATURE = MAX_NB_FEATURES;
	};
	PX_COMPILE_TIME_ASSERT(PxgDeformableRef::eLAST <= (1 << PxgDeformableRef::TYPE_BITS));

	// The bounds above stand for these caps, so raising one alone breaks the build on purpose. Three
	// equalities against one anchor pin all four element caps, which is why the volume pair is not
	// compared to itself.
	PX_COMPILE_TIME_ASSERT(PX_MAX_NB_DEFORMABLE_SURFACE_TRI == PX_MAX_NB_DEFORMABLE_SURFACE_VTX);
	PX_COMPILE_TIME_ASSERT(PX_MAX_NB_DEFORMABLE_SURFACE_TRI == PX_MAX_NB_DEFORMABLE_VOLUME_TET);
	PX_COMPILE_TIME_ASSERT(PX_MAX_NB_DEFORMABLE_SURFACE_TRI == PX_MAX_NB_DEFORMABLE_VOLUME_VTX);
	PX_COMPILE_TIME_ASSERT(PX_MAX_NB_DEFORMABLE_SURFACE == PX_MAX_NB_DEFORMABLE_VOLUME);

	// The wildcard has to fit the field it shares with real element ids. Every allowed element staying
	// below it follows from where it is defined, so the condition worth guarding is this one.
	PX_COMPILE_TIME_ASSERT(PxgDeformableRef::ANY_FEATURE <= PxgDeformableRef::FEATURE_ID_MASK);

	// Statically assert validity of the bit assignments.
	PX_COMPILE_TIME_ASSERT(PxgDeformableRef::INVALID_FLAG == (PxU64(1) << 32));
	PX_COMPILE_TIME_ASSERT(PxgDeformableRef::ACTIVE_FLAG == (PxgDeformableRef::INVALID_FLAG << 1));
	PX_COMPILE_TIME_ASSERT(PxgDeformableRef::DEFORMABLE_SHIFT == 34);
	PX_COMPILE_TIME_ASSERT(PxU64(PxgDeformableRef::MAX_NB_DEFORMABLES)
		<= (PxU64(1) << PxgDeformableRef::DEFORMABLE_BITS));

	// Encodes a deformable GPU index, a feature type and a feature id. The local edge selector is left to
	// PxgEncodeDeformableTriangleEdge, and both flags are left clear, so the result is inactive and valid.
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU64 PxgEncodeDeformableRef(const PxU32 deformableId,
		const PxgDeformableRef::Enum type, const PxU32 featureId)
	{
		PX_ASSERT(deformableId < PxgDeformableRef::MAX_NB_DEFORMABLES);
		// A real element, or the element filter wildcard. A producer that resolves no feature calls
		// PxgEncodeInvalidDeformableRef rather than passing a reserved index here.
		PX_ASSERT(featureId < PxgDeformableRef::MAX_NB_FEATURES
			|| featureId == PxgDeformableRef::ANY_FEATURE);
		return (PxU64(deformableId) << PxgDeformableRef::DEFORMABLE_SHIFT)
			| (PxU64(type) << (PxgDeformableRef::FEATURE_ID_BITS + PxgDeformableRef::LOCAL_EDGE_BITS))
			| featureId;
	}

	// Encodes an invalid reference for a deformable, holding its id and no feature. The flag only
	// applies to ref1 of the pair, read by PxgIsValidPairInfo.
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU64 PxgEncodeInvalidDeformableRef(const PxU32 deformableId)
	{
		PX_ASSERT(deformableId < PxgDeformableRef::MAX_NB_DEFORMABLES);
		return (PxU64(deformableId) << PxgDeformableRef::DEFORMABLE_SHIFT)
			| PxgDeformableRef::INVALID_FLAG;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU64 PxgEncodeDeformableVertex(const PxU32 deformableId, const PxU32 vertexId)
	{
		return PxgEncodeDeformableRef(deformableId, PxgDeformableRef::eVERTEX, vertexId);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU64 PxgEncodeDeformableTriangle(const PxU32 deformableId, const PxU32 triangleId)
	{
		return PxgEncodeDeformableRef(deformableId, PxgDeformableRef::eTRIANGLE, triangleId);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU64 PxgEncodeDeformableTetrahedron(const PxU32 deformableId, const PxU32 tetrahedronId)
	{
		return PxgEncodeDeformableRef(deformableId, PxgDeformableRef::eTETRAHEDRON, tetrahedronId);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU64 PxgEncodeDeformableTriangleEdge(const PxU32 deformableId,
		const PxU32 triangleId, const PxU32 localEdge)
	{
		PX_ASSERT(localEdge <= PxgDeformableRef::LOCAL_EDGE_MASK);
		return PxgEncodeDeformableRef(deformableId, PxgDeformableRef::eTRIANGLE_EDGE, triangleId)
			| (PxU64(localEdge) << PxgDeformableRef::FEATURE_ID_BITS);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 PxgGetDeformableId(const PxU64 ref)
	{
		return PxU32(ref >> PxgDeformableRef::DEFORMABLE_SHIFT);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxgDeformableRef::Enum PxgGetDeformableFeatureType(const PxU64 ref)
	{
		return PxgDeformableRef::Enum(
			PxU32(ref) >> (PxgDeformableRef::FEATURE_ID_BITS + PxgDeformableRef::LOCAL_EDGE_BITS));
	}

	// Vertex, triangle or tetrahedron index, and the triangle of a triangle edge.
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 PxgGetDeformableFeatureId(const PxU64 ref)
	{
		return PxU32(ref) & PxgDeformableRef::FEATURE_ID_MASK;
	}

	// Reads as PxgGetDeformableFeatureId, and records at the call site which kind is being read.
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 PxgGetDeformableEdgeTriangle(const PxU64 ref)
	{
		return PxgGetDeformableFeatureId(ref);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 PxgGetDeformableEdgeLocalIndex(const PxU64 ref)
	{
		return (PxU32(ref) >> PxgDeformableRef::FEATURE_ID_BITS) & PxgDeformableRef::LOCAL_EDGE_MASK;
	}

	/**
	\brief The layout of a particle participant.

	    [systemId:30][active:1][invalid:1][   particleIndex:32   ]
	      63..34       bit 33    bit 32            31..0

	- systemId: the particle system's GPU index. Written by PxgEncodeParticleRef and read by
	  PxgGetParticleSystemId.
	- active and invalid: currently unused. No particle path writes or reads either one.
	- particleIndex: the particle's index within its system. Written by PxgEncodeParticleRef and read
	  by PxgGetParticleIndex.

	The system id is stored above the flag bits, at the offset PxgDeformableRef uses for the deformable
	id. Both kinds therefore hold the flags in the same two bits, which is what allows either one in
	ref1. Asserted below.
	*/
	struct PxgParticleRef
	{
		static constexpr PxU64 INVALID_FLAG = PxU64(1) << 32;
		static constexpr PxU64 ACTIVE_FLAG = PxU64(1) << 33;

		static constexpr PxU32 SYSTEM_SHIFT = 34;
		static constexpr PxU32 SYSTEM_BITS = 64 - SYSTEM_SHIFT;
	};

	// The two layouts have to keep agreeing, so that the pair accessors can read ref1 without knowing
	// which kind of reference it holds.
	PX_COMPILE_TIME_ASSERT(PxgParticleRef::INVALID_FLAG == PxgDeformableRef::INVALID_FLAG);
	PX_COMPILE_TIME_ASSERT(PxgParticleRef::ACTIVE_FLAG == PxgDeformableRef::ACTIVE_FLAG);
	PX_COMPILE_TIME_ASSERT(PxgParticleRef::SYSTEM_SHIFT == PxgDeformableRef::DEFORMABLE_SHIFT);

	// A particle participant is its system id over its index within that system, the last of the three
	// participant kinds a pair can hold. Both flag bits are left clear.
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU64 PxgEncodeParticleRef(const PxU32 particleSystemId, const PxU32 particleId)
	{
		PX_ASSERT(particleSystemId < (1u << PxgParticleRef::SYSTEM_BITS));
		PX_ASSERT(particleId < 0xffffffff);
		return (PxU64(particleSystemId) << PxgParticleRef::SYSTEM_SHIFT) | particleId;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 PxgGetParticleIndex(const PxU64 ref)
	{
		return (0xffffffff) & ref;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 PxgGetParticleSystemId(const PxU64 ref)
	{
		return PxU32(ref >> PxgParticleRef::SYSTEM_SHIFT);
	}

	// The key the contact radix sort's first stage takes, narrowed from the participant the contacts are
	// grouped by, which is a rigid or a particle. PxNodeIndex and PxgParticleRef both keep that index in
	// their low 32 bits, so one narrowing serves both. A second stage sorts the high 32 bits after it.
	// Contacts are never grouped by a deformable, whose low bits hold a feature descriptor, not an index.
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 PxgGetContactSortKey(const PxU64 participantRef)
	{
		return PxU32(participantRef);
	}

	// Two participant references, serving contacts, element filters and attachments alike. A word holds
	// a deformable feature, a rigid PxNodeIndex or a particle index, and which kind occupies which word
	// is the convention of the code that writes and reads the pair rather than a property of the pair.
	struct PX_ALIGN_PREFIX(16) PxgDeformablePairInfo
	{
		PxU64 ref0;
		PxU64 ref1;
	} PX_ALIGN_SUFFIX(16);

	PX_COMPILE_TIME_ASSERT(sizeof(PxgDeformablePairInfo) == 16);

	// True while the pair is usable. Only ref1 is probed, which is sound for either kind that can occupy
	// it: a deformable feature and a particle put the flags in the same two bits, asserted above. ref0
	// cannot be probed alongside it, because a rigid PxNodeIndex spends that bit on isArticulation() and
	// every articulation contact would read as invalid.
	PX_CUDA_CALLABLE PX_FORCE_INLINE bool PxgIsValidPairInfo(const PxgDeformablePairInfo& pair)
	{
		return (pair.ref1 & PxgDeformableRef::INVALID_FLAG) == 0;
	}

	// Marks the pair as invalid after it has been initialized. The flag is only applied to ref1, read by
	// PxgIsValidPairInfo. The operation is not thread safe.
	PX_CUDA_CALLABLE PX_FORCE_INLINE void PxgInvalidatePairInfo(PxgDeformablePairInfo& pair)
	{
		pair.ref1 |= PxgDeformableRef::INVALID_FLAG;
	}

	// The activity decided by the pre-count pass, which the solve then reads, so both run against one
	// decision taken on the same positions. The two rigid-deformable paths latch it instead of rewriting
	// it each iteration, which keeps a contact solving while it still holds an impulse to release.
	PX_CUDA_CALLABLE PX_FORCE_INLINE bool PxgIsActivePairInfo(const PxgDeformablePairInfo& pair)
	{
		return (pair.ref1 & PxgDeformableRef::ACTIVE_FLAG) != 0;
	}

	// Only the owner of a contact may call this, one thread per contact in the pre-count pass. It
	// rewrites ref1 in full, so a second writer would lose the identity rather than just the flag.
	PX_CUDA_CALLABLE PX_FORCE_INLINE void PxgSetActivePairInfo(PxgDeformablePairInfo& pair, const bool active)
	{
		pair.ref1 = active ? (pair.ref1 | PxgDeformableRef::ACTIVE_FLAG)
						   : (pair.ref1 & ~PxgDeformableRef::ACTIVE_FLAG);
	}

	// A total order over the whole pair, for the element filter arrays: they are sorted on the host and
	// binary-searched on the device for membership, never for a range. The identity itself needs no
	// order, so this is a free function rather than a method.
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxI32 PxgComparePairInfo(const PxgDeformablePairInfo& a, const PxgDeformablePairInfo& b)
	{
		if (a.ref0 < b.ref0)
			return -1;
		if (a.ref0 > b.ref0)
			return 1;
		if (a.ref1 < b.ref1)
			return -1;
		if (a.ref1 > b.ref1)
			return 1;
		return 0;
	}

	// Host-staged attachment, baked into PxgRigidDbAttachmentBlock or PxgDbDbAttachmentBlock by the
	// matching prep kernel. ref1 is always a deformable feature. ref0 is a rigid PxNodeIndex for a
	// rigid-deformable attachment and a deformable feature otherwise.
	PX_ALIGN_PREFIX(16)
	class PxgDeformableAttachment
	{
	public:
		PxgDeformablePairInfo pair;

		// Weights of the triangle or tetrahedron ref0 identifies. Where ref0 is a rigid this is the
		// attachment point in its local frame instead, with w unused.
		float4 barycentricOrPosition0;

		// Weights of the attached triangle's or tetrahedron's vertices. Unread when ref1 identifies a
		// vertex.
		float4 barycentric1;
	}PX_ALIGN_SUFFIX(16);

	// The record is uploaded as an array sized by sizeof, so a field added here costs memory per
	// attachment without any call site noticing.
	PX_COMPILE_TIME_ASSERT(sizeof(PxgDeformableAttachment) == 48);

	struct PxgUpdateArticulationDesc
	{
		PxgArticulation*						mNewArticulations;
		PxReal*									mNewLinkWakeCounters;
		PxgArticulationLink*					mNewLinks;
		PxgArticulationLinkProp*				mNewLinkProps;
		
		PxU32*									mNewLinkParents;
		Dy::ArticulationBitField*				mNewLinkChildren;
		PxTransform*							mNewLinkBody2Worlds;
		PxTransform*							mNewLinkBody2Actors;
		Cm::UnAlignedSpatialVector*				mNewLinkExtAccels;
		Dy::ArticulationJointCore*				mNewJointCores;
		Dy::ArticulationJointCoreData*			mNewJointData;
		PxgArticulationSimUpdate*				mIndicesOffset;

		PxgArticulation*						mArticulationPool;
		PxgSolverBodySleepData*					mArticulationSleepDataPool;

		PxGpuSpatialTendonData*					mNewSpatialTendonParamsPool;
		PxgArticulationTendon*					mNewSpatialTendonPool;
		PxgArticulationTendonElementFixedData*	mNewAttachmentFixedPool;
		PxGpuTendonAttachmentData*				mNewAttachmentModPool;
		PxU32*									mNewTendonAttachmentRemapPool;

		PxGpuFixedTendonData*					mNewFixedTendonParamsPool;
		PxgArticulationTendon*					mNewFixedTendonPool;
		PxgArticulationTendonElementFixedData*	mNewTendonJointFixedPool;
		PxGpuTendonJointCoefficientData*		mNewTendonJointCoefficientPool;
		PxU32*									mNewTendonTendonJointRemapPool;
		Dy::ArticulationMimicJointCore*			mNewArticulationMimicJointPool;	

		PxU32*									mNewPathToRootPool;
		
		PxU32									mNbNewArticulations;	//number of newly added aritculations	
	};

	struct PxgUpdatedBodiesDesc
	{
		PxgBodySim*					mBodySimBufferDeviceData;
		PxgBodySimVelocityUpdate*	mUpdatedBodySim;
		PxU32						mNbUpdatedBodies;
	};

	struct PxgUpdatedJointsDesc
	{
		const PxgD6JointData*		mD6RigidJointCPUPool;			//map memory
		PxgD6JointData*				mD6RigidJointGPUPool;			//device memory

		const PxgConstraintPrePrep*	mD6RigidJointPrePrepCPUPool;	//map memory
		PxgConstraintPrePrep*		mD6RigidJointPrePrepGPUPool;	//device memory

		const PxgD6JointData*		mD6ArtiJointCPUPool;			//map memory
		PxgD6JointData*				mD6ArtiJointGPUPool;			//device memory

		const PxgConstraintPrePrep*	mD6ArtiJointPrePrepCPUPool;		//map memory
		PxgConstraintPrePrep*		mD6ArtiJointPrePrepGPUPool;		//device memory

		const PxU32*				mUpdatedRigidJointIndices;		//map memory
		PxU32						mNbUpdatedRigidJoints;

		const PxU32*				mUpdatedArtiJointIndices;		//map memory
		PxU32						mNbUpdatedArtiJoints;
	};

	struct PxgSimulationCoreDesc
	{
		PxU32*					mChangedAABBMgrHandles;
		PxU32*					mFrozen;
		PxU32*					mUnfrozen;
		PxU32*					mFrozenBlockAndRes;//this array is used to store frozen block value for the scan and also store the final frozen shape index
		PxU32*					mUnfrozenBlockAndRes;//this array is used to store unfrozen block value for the scane and also store the final unfrozen shape index
		PxU32*					mUpdated;
		PxU32*					mActivate;
		PxU32*					mDeactivate;

		PxgBodySim*				mBodySimBufferDeviceData;
		const PxgShapeSim*		mShapeSimsBufferDeviceData;
		PxgArticulation*		mArticulationPool;
		PxgSolverBodySleepData*	mArticulationSleepDataPool;

		PxsCachedTransform*		mTransformCache;
		PxBounds3*				mBounds;

		PxU32*					mBodyDataIndices;

		PxgSolverBodySleepData*	mSleepData;
		PxgShape*				mShapes;
		
		PxU32					mNbTotalShapes;	//number of total shapes;
		PxU32					mBitMapWordCounts; //number of words in gChangedAABBMgrHandles, this will include shapes and aggregates

		PxU32					mTotalFrozenShapes;   // AD: these two members are the only reason we copy the whole descriptor back to cpu.
		PxU32					mTotalUnfrozenShapes;
	};

	struct PxgUpdateActorDataDesc
	{
		PxgBodySim*			mBodySimBufferDeviceData;

		PxgShape*			mShapes;

		PxsCachedTransform*	mTransformCache;
		PxBounds3*			mBounds;

		PxNodeIndex*		mRigidNodeIndices;
		PxU32*				mShapeIndices;

		PxU32*				mUpdated;					//Direct API changed handle
		PxU32*				mChangedAABBMgrHandles;		//CPU API changed handle

		PxU32				mBitMapWordCounts;

		const PxgShapeSim*	mShapeSimsBufferDeviceData;
	};
}
#endif
