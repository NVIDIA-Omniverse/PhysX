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
	static			void					getBinaryMetaData(PxOutputStream& stream);
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

					void					attachShape(NpShape& shape, PxRigidActor& actor);
					bool					detachShape(NpShape& s, PxRigidActor& actor, bool wakeOnLostTouch);
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
