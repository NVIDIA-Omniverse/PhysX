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


#ifndef BP_BROADPHASE_SAP_H
#define BP_BROADPHASE_SAP_H

#include "BpBroadPhase.h"
#include "BpBroadPhaseSapAux.h"
#include "CmPool.h"
#include "CmTask.h"

namespace physx
{
class PxcScratchAllocator;

namespace Gu
{
	class Axes;
}

namespace Bp
{

class SapEndPoint;
class IntegerAABB;

class BroadPhaseBatchUpdateWorkTask: public Cm::Task
{
public:

	BroadPhaseBatchUpdateWorkTask(PxU64 contextId=0) :
		Cm::Task		(contextId), 
		mSap			(NULL),
		mAxis			(0xffffffff),
		mPairs			(NULL),
		mPairsSize		(0),
		mPairsCapacity	(0)
	{
	}

	virtual void runInternal();

	virtual const char* getName() const { return "BpBroadphaseSap.batchUpdate"; }

	void set(class BroadPhaseSap* sap, const PxU32 axis) {mSap = sap; mAxis = axis;}

	BroadPhasePair* getPairs() const {return mPairs;}
	PxU32 getPairsSize() const {return mPairsSize;}
	PxU32 getPairsCapacity() const {return mPairsCapacity;}

	void setPairs(BroadPhasePair* pairs, const PxU32 pairsCapacity) {mPairs = pairs; mPairsCapacity = pairsCapacity;}

	void setNumPairs(const PxU32 pairsSize) {mPairsSize=pairsSize;}

private:

	class BroadPhaseSap* mSap;
	PxU32 mAxis;

	BroadPhasePair* mPairs;
	PxU32 mPairsSize;
	PxU32 mPairsCapacity;
};

//KS - TODO, this could be reduced to U16 in smaller scenes
struct BroadPhaseActivityPocket
{
	PxU32 mStartIndex;
	PxU32 mEndIndex;
};


class BroadPhaseSap : public BroadPhase
{
	PX_NOCOPY(BroadPhaseSap)
public:

	friend class BroadPhaseBatchUpdateWorkTask;
	friend class SapUpdateWorkTask;
	friend class SapPostUpdateWorkTask;

										BroadPhaseSap(const PxU32 maxNbBroadPhaseOverlaps, const PxU32 maxNbStaticShapes, const PxU32 maxNbDynamicShapes, PxU64 contextID);
	virtual								~BroadPhaseSap();

	// BroadPhase
	virtual	PxBroadPhaseType::Enum		getType()					const	PX_OVERRIDE	PX_FINAL	{ return PxBroadPhaseType::eSAP;	}
	virtual	void						release()							PX_OVERRIDE	PX_FINAL;
	virtual	void						update(PxcScratchAllocator* scratchAllocator, const BroadPhaseUpdateData& updateData, physx::PxBaseTask* continuation)	PX_OVERRIDE	PX_FINAL;
	virtual	void						preBroadPhase(const Bp::BroadPhaseUpdateData&)	PX_OVERRIDE	PX_FINAL	{}
	virtual void						fetchBroadPhaseResults()	PX_OVERRIDE	PX_FINAL	{}
	virtual const BroadPhasePair*		getCreatedPairs(PxU32& nbCreatedPairs)	const	PX_OVERRIDE	PX_FINAL	{ nbCreatedPairs = mCreatedPairsSize;	return mCreatedPairsArray;	}
	virtual const BroadPhasePair*		getDeletedPairs(PxU32& nbDeletedPairs)	const	PX_OVERRIDE	PX_FINAL	{ nbDeletedPairs = mDeletedPairsSize;	return mDeletedPairsArray;	}
	virtual void						freeBuffers()	PX_OVERRIDE	PX_FINAL;
	virtual void						shiftOrigin(const PxVec3& shift, const PxBounds3* boundsArray, const PxReal* contactDistances)	PX_OVERRIDE	PX_FINAL;
#if PX_CHECKED
	virtual bool						isValid(const BroadPhaseUpdateData& updateData) const	PX_OVERRIDE	PX_FINAL;
#endif
	//~BroadPhase

private:
			void						resizeBuffers();

			PxcScratchAllocator*		mScratchAllocator;

	//Data passed in from updateV.
			const BpHandle*				mCreated;				
			PxU32						mCreatedSize;			
			const BpHandle*				mRemoved;				
			PxU32						mRemovedSize;				
			const BpHandle*				mUpdated;				
			PxU32						mUpdatedSize;				
			const PxBounds3*			mBoxBoundsMinMax;			
			const Bp::FilterGroup::Enum*mBoxGroups;
			const BpFilter*				mFilter;
			const PxReal*				mContactDistance;
			PxU32						mBoxesCapacity;

	//Boxes.
			SapBox1D*					mBoxEndPts[3];			//Position of box min/max in sorted arrays of end pts (needs to have mBoxesCapacity).

	//End pts (endpts of boxes sorted along each axis).
			ValType*					mEndPointValues[3];		//Sorted arrays of min and max box coords
			BpHandle*					mEndPointDatas[3];		//Corresponding owner id and isMin/isMax for each entry in the sorted arrays of min and max box coords.

			PxU8*						mBoxesUpdated;	
			BpHandle*					mSortedUpdateElements;	
			BroadPhaseActivityPocket*	mActivityPockets;
			BpHandle*					mListNext;
			BpHandle*					mListPrev;

			PxU32						mBoxesSize;				//Number of sorted boxes + number of unsorted (new) boxes
			PxU32						mBoxesSizePrev;			//Number of sorted boxes 
			PxU32						mEndPointsCapacity;		//Capacity of sorted arrays. 

	//Default maximum number of overlap pairs 
			PxU32						mDefaultPairsCapacity;

	//Box-box overlap pairs created or removed each update.
			BpHandle*					mData;
			PxU32						mDataSize;
			PxU32						mDataCapacity;

	//All current box-box overlap pairs.
			SapPairManager				mPairs;

	//Created and deleted overlap pairs reported back through api.
			BroadPhasePair*				mCreatedPairsArray;
			PxU32						mCreatedPairsSize;
			PxU32						mCreatedPairsCapacity;
			BroadPhasePair*				mDeletedPairsArray;
			PxU32						mDeletedPairsSize;
			PxU32						mDeletedPairsCapacity;
			PxU32						mActualDeletedPairSize;

			bool						setUpdateData(const BroadPhaseUpdateData& updateData);
			void						update();
			void						postUpdate();

	//Batch create/remove/update.
			void						batchCreate();
			void						batchRemove();
			void						batchUpdate();

			void						batchUpdate(const PxU32 Axis, BroadPhasePair*& pairs, PxU32& pairsSize, PxU32& pairsCapacity);

			void						batchUpdateFewUpdates(const PxU32 Axis, BroadPhasePair*& pairs, PxU32& pairsSize, PxU32& pairsCapacity);

			void						ComputeSortedLists(	//const PxVec4& globalMin, const PxVec4& globalMax,
															BpHandle* PX_RESTRICT newBoxIndicesSorted, PxU32& newBoxIndicesCount, BpHandle* PX_RESTRICT oldBoxIndicesSorted, PxU32& oldBoxIndicesCount,
															bool& allNewBoxesStatics, bool& allOldBoxesStatics);

			BroadPhaseBatchUpdateWorkTask mBatchUpdateTasks[3];

			const PxU64					mContextID;
#if PX_DEBUG
			bool						isSelfOrdered() const;
			bool						isSelfConsistent() const;
#endif
};

} //namespace Bp

} //namespace physx

#endif //BP_BROADPHASE_SAP_H
