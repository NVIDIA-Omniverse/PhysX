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

#ifndef NP_SCENE_QUERIES_H
#define NP_SCENE_QUERIES_H

#include "PxSceneQueryDesc.h"

#include "SqQuery.h"

#include "ScSqBoundsSync.h"
#if PX_SUPPORT_PVD
	#include "NpPvdSceneQueryCollector.h"
	#include "NpPvdSceneClient.h"
#endif

#include "PxSceneQuerySystem.h"

#include "NpBounds.h"	// PT: for SQ_PRUNER_EPSILON

namespace physx
{
	class PxScene;
	class PxSceneDesc;

namespace Vd
{
	class PvdSceneClient;
}

class NpSceneQueries : public Sc::SqBoundsSync
#if PX_SUPPORT_PVD
	, public Sq::PVDCapture
#endif
{
												PX_NOCOPY(NpSceneQueries)
	public:
	// PT: TODO: use PxSceneQueryDesc here, but we need some SQ-specific "scene limits"
												NpSceneQueries(const PxSceneDesc& desc, Vd::PvdSceneClient* pvd, PxU64 contextID);
												~NpSceneQueries();

	PX_FORCE_INLINE	PxSceneQuerySystem&			getSQAPI()			{ PX_ASSERT(mSQ);	return *mSQ;	}
	PX_FORCE_INLINE	const PxSceneQuerySystem&	getSQAPI()	const	{ PX_ASSERT(mSQ);	return *mSQ;	}

	protected:
	// SqBoundsSync
	virtual			void						sync(PxU32 prunerIndex, const ScPrunerHandle* handles, const PxU32* boundsIndices, const PxBounds3* bounds,
													const PxTransform32* transforms, PxU32 count, const PxBitMap& ignoredIndices)	PX_OVERRIDE;
	//~SqBoundsSync

	public:
					PxSceneQuerySystem*			mSQ;

#if PX_SUPPORT_PVD
					Vd::PvdSceneClient*			mPVDClient;
					//Scene query and hits for pvd, collected in current frame
			mutable Vd::PvdSceneQueryCollector	mSingleSqCollector;
	PX_FORCE_INLINE	Vd::PvdSceneQueryCollector&	getSingleSqCollector()	const	{ return mSingleSqCollector;	}

	// PVDCapture
	virtual			bool						transmitSceneQueries();
	virtual			void						raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal distance, const PxRaycastHit* hit, PxU32 hitsNum, const PxQueryFilterData& filterData, bool multipleHits);
	virtual			void						sweep(const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, PxReal distance, const PxSweepHit* hit, PxU32 hitsNum, const PxQueryFilterData& filterData, bool multipleHits);
	virtual			void						overlap(const PxGeometry& geometry, const PxTransform& pose, const PxOverlapHit* hit, PxU32 hitsNum, const PxQueryFilterData& filterData);
	//~PVDCapture
#endif // PX_SUPPORT_PVD
};

} // namespace physx, sq

#endif
