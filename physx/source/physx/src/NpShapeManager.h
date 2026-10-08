// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef NP_SHAPE_MANAGER_H
#define NP_SHAPE_MANAGER_H

#include "NpShape.h"
#include "CmPtrTable.h"
#include "GuBVH.h"

#if PX_ENABLE_DEBUG_VISUALIZATION
	#include "common/PxRenderOutput.h"
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

#include "SqTypedef.h"

namespace physx
{

namespace Sq
{
	class PruningStructure;
	class PrunerManager;
}

class NpScene;

	// PT: if we go through an SQ virtual interface then the implementation can be different from our internal version,
	// and nothing says it uses the same types as what we have internally in SQ. So we need a separate set of types.
	typedef PxSQCompoundHandle		NpCompoundId;
	static const NpCompoundId NP_INVALID_COMPOUND_ID = NpCompoundId(Sq::INVALID_COMPOUND_ID);

class NpShapeManager : public PxUserAllocated
{
public:
// PX_SERIALIZATION
											NpShapeManager(const PxEMPTY);
					void					preExportDataReset();
					void					exportExtraData(PxSerializationContext& stream);
					void					importExtraData(PxDeserializationContext& context);
//~PX_SERIALIZATION
											NpShapeManager();
											~NpShapeManager();

	PX_FORCE_INLINE	PxU32					getNbShapes()		const	{ return mShapes.getCount();									}
	PX_FORCE_INLINE	NpShape* const*			getShapes()			const	{ return reinterpret_cast<NpShape*const*>(mShapes.getPtrs());	}
					PxU32					getShapes(PxShape** buffer, PxU32 bufferSize, PxU32 startIndex=0) const;

					bool					attachShape(NpShape& shape, PxRigidActor& actor);

	// PT: outcome of detachShape(). The caller emits the user-facing "not attached to this actor" message,
	// so it has to be able to tell that case apart from one detachShape() has already reported itself -
	// otherwise the log contradicts itself (OMPE-103062).
	struct DetachResult
	{
		enum Enum
		{
			eDETACHED,			//!< the shape was detached
			eNOT_ATTACHED,		//!< the shape is not attached to this actor; the caller reports it
			eERROR_REPORTED		//!< bookkeeping was inconsistent; detachShape() has already reported it
		};
	};

					DetachResult::Enum		detachShape(NpShape& s, PxRigidActor& actor, bool wakeOnLostTouch);
					void					detachAll(PxSceneQuerySystem* pxsq, const PxRigidActor& actor);

					void					setupSQShape(PxSceneQuerySystem& pxsq, const NpShape& shape, const NpActor& npActor, const PxRigidActor& actor, bool dynamic, const PxBounds3* bounds, const Sq::PruningStructure* ps);
					void					setupSceneQuery(PxSceneQuerySystem& pxsq, const NpActor& npActor, const PxRigidActor& actor, const NpShape& shape);
					void					setupAllSceneQuery(PxSceneQuerySystem& pxsq, const NpActor& npActor, const PxRigidActor& actor, const Sq::PruningStructure* ps, const PxBounds3* bounds, bool isDynamic);
					void					setupAllSceneQuery(PxSceneQuerySystem& pxsq, const PxRigidActor& actor, const Sq::PruningStructure* ps, const PxBounds3* bounds=NULL, const Gu::BVH* bvh = NULL);
					void					teardownAllSceneQuery(PxSceneQuerySystem& pxsq, const PxRigidActor& actor);
					void					teardownSceneQuery(PxSceneQuerySystem& pxsq, const PxRigidActor& actor, const NpShape& shape);
					void					markShapeForSQUpdate(PxSceneQuerySystem& pxsq, const PxShape& shape, const PxRigidActor& actor);
					void					markActorForSQUpdate(PxSceneQuerySystem& pxsq, const PxRigidActor& actor);

					PxBounds3				getWorldBounds_(const PxRigidActor&) const;

	PX_FORCE_INLINE	void					setPruningStructure(Sq::PruningStructure* ps) { mPruningStructure = ps;		}
	PX_FORCE_INLINE	Sq::PruningStructure*	getPruningStructure()					const { return mPruningStructure;	}

//	PX_FORCE_INLINE	bool					isSqCompound()				const	{ return mSqCompoundId != NP_INVALID_COMPOUND_ID;	}
//	PX_FORCE_INLINE	NpCompoundId			getCompoundID()				const	{ return mSqCompoundId;	}
//	PX_FORCE_INLINE	void					setCompoundID(NpCompoundId id)		{ mSqCompoundId = id;	}

	// PT: TODO: we don't really need to store the compound id anymore
	PX_FORCE_INLINE	bool					isSqCompound()				const	{ return mShapes.mFreeSlot != NP_INVALID_COMPOUND_ID;	}
	PX_FORCE_INLINE	NpCompoundId			getCompoundID()				const	{ return mShapes.mFreeSlot;	}
	PX_FORCE_INLINE	void					setCompoundID(NpCompoundId id)		{ mShapes.mFreeSlot = id;	}

					void					clearShapesOnRelease(NpScene& s, PxRigidActor&);

#if PX_ENABLE_DEBUG_VISUALIZATION
					void					visualize(PxRenderOutput& out, NpScene& scene, const PxRigidActor& actor, float scale)	const;
#else
					PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif
					// for batching
	PX_FORCE_INLINE	const Cm::PtrTable&		getShapeTable() const 		{	return mShapes; }
	static PX_FORCE_INLINE size_t			getShapeTableOffset()		{	return PX_OFFSET_OF_RT(NpShapeManager, mShapes); }
private:
					Cm::PtrTable			mShapes;
					Sq::PruningStructure*	mPruningStructure;  // Shape scene query data are pre-build in pruning structure
//					NpCompoundId			mSqCompoundId;

					void					releaseExclusiveUserReferences();
					void					setupSceneQuery_(PxSceneQuerySystem& pxsq, const NpActor& npActor, const PxRigidActor& actor, const NpShape& shape);
					void					addBVHShapes(PxSceneQuerySystem& pxsq, const PxRigidActor& actor, const Gu::BVH& bvh);
};

}

#endif
