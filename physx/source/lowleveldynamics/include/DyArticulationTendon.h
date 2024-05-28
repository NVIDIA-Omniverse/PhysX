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


#ifndef PXD_ARTICULATION_TENDON_H
#define PXD_ARTICULATION_TENDON_H

#include "foundation/PxVec3.h"
#include "foundation/PxQuat.h"
#include "foundation/PxTransform.h"
#include "foundation/PxVecMath.h"
#include "foundation/PxUtilities.h"
#include "CmUtils.h"
#include "CmIDPool.h"
#include "solver/PxSolverDefs.h"

namespace physx
{
namespace Dy
{

	typedef PxU64 ArticulationAttachmentBitField;

#define DY_ARTICULATION_ATTACHMENT_NONE 0xffffffff

	struct ArticulationAttachment
	{
		PxVec3 relativeOffset;		//relative offset to the link
		PxReal lowLimit;
		PxReal highLimit;
		PxReal restLength;
		
		PxReal coefficient;
		PxU32 parent;	//parent index
		PxU32 myInd;
		PxU32 mConstraintInd;
		PxU16 linkInd;
		PxU16 childCount;

		ArticulationAttachmentBitField children;
	};

	class ArticulationTendon
	{
	public:

		ArticulationTendon() : mStiffness(0.f), mDamping(0.f), mOffset(0.f), mLimitStiffness(0.f)
		{ 
		}
		PxReal								mStiffness;
		PxReal								mDamping;
		PxReal								mOffset;
		PxReal								mLimitStiffness;
	};

	class ArticulationSpatialTendon : public ArticulationTendon
	{
	public:

		ArticulationSpatialTendon()
		{
			mAttachments.reserve(64);
			mAttachments.forceSize_Unsafe(64);
			
		}

		PX_FORCE_INLINE ArticulationAttachment* getAttachments() { return mAttachments.begin(); }

		PX_FORCE_INLINE ArticulationAttachment& getAttachment(const PxU32 index) { return mAttachments[index]; }

		PX_FORCE_INLINE PxU32 getNumAttachments() { return mIDPool.getNumUsedID(); }

		PX_FORCE_INLINE PxU32 getNewID() 
		{ 
			const PxU32 index = mIDPool.getNewID();
			if (mAttachments.capacity() <= index)
			{
				mAttachments.resize(index * 2 + 1);
			}
			return index;
		}

		PX_FORCE_INLINE void freeID(const PxU32 index)
		{
			mIDPool.freeID(index);
		}

		PX_FORCE_INLINE PxU32 getTendonIndex() { return mIndex; }

		PX_FORCE_INLINE void setTendonIndex(const PxU32 index) { mIndex = index; }

	private:

		PxArray<ArticulationAttachment>		mAttachments;
		Cm::IDPool							mIDPool;
		PxU32								mIndex;
	};

	class ArticulationTendonJoint
	{
	public:
		PxU16					        axis;
		PxU16							startJointOffset;
		PxReal							coefficient;
		PxReal							recipCoefficient;
		PxU32							mConstraintInd;
		PxU32							parent;	//parent index
		PxU16							linkInd;
		PxU16							childCount;
		ArticulationAttachmentBitField	children;
	};

	class ArticulationFixedTendon : public ArticulationTendon
	{
	public:

		ArticulationFixedTendon() :mLowLimit(PX_MAX_F32), mHighLimit(-PX_MAX_F32), mRestLength(0.f)
		{
			mTendonJoints.reserve(64);
			mTendonJoints.forceSize_Unsafe(64);

		}


		PX_FORCE_INLINE ArticulationTendonJoint* getTendonJoints() { return mTendonJoints.begin(); }

		PX_FORCE_INLINE ArticulationTendonJoint& getTendonJoint(const PxU32 index) { return mTendonJoints[index]; }

		PX_FORCE_INLINE PxU32 getNumJoints() { return mIDPool.getNumUsedID(); }

		PX_FORCE_INLINE PxU32 getNewID()
		{
			const PxU32 index = mIDPool.getNewID();
			if (mTendonJoints.capacity() <= index)
			{
				mTendonJoints.resize(index * 2 + 1);
			}
			return index;
		}

		PX_FORCE_INLINE void freeID(const PxU32 index)
		{
			mIDPool.freeID(index);
		}

		PX_FORCE_INLINE PxU32 getTendonIndex() { return mIndex; }

		PX_FORCE_INLINE void setTendonIndex(const PxU32 index) { mIndex = index; }

		PxReal								mLowLimit;
		PxReal								mHighLimit;
		PxReal								mRestLength;

		PxReal								mError;

	private:
		PxArray<ArticulationTendonJoint>	mTendonJoints;

		Cm::IDPool							mIDPool;
		PxU32								mIndex;
	};


}//namespace Dy
}//namespace physx

#endif
