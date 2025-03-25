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

#ifndef PXC_NP_WORK_UNIT_H
#define PXC_NP_WORK_UNIT_H

#include "PxConstraintDesc.h"
#include "PxvGeometry.h"

// PT: the shapeCore structs are 16-bytes aligned by design so the low 4 bits of their pointers are available.
// We can store the geom types there since they fit. An alternative would be simply to read the types from
// shapeCore->mGeometry.getType() but that is one more indirection/cache miss. We might be using other shapeCore
// data everywhere we need the type so that might be irrelevant and could be revisited.
PX_COMPILE_TIME_ASSERT(physx::PxGeometryType::eGEOMETRY_COUNT<16);

namespace physx
{
struct PxsRigidCore;
struct PxsShapeCore;

namespace IG
{
	typedef PxU32 EdgeIndex;
}

struct PxcNpWorkUnitFlag
{
	enum Enum
	{
		eOUTPUT_CONTACTS			= 1 << 0,
		eOUTPUT_CONSTRAINTS			= 1 << 1,
		eDISABLE_STRONG_FRICTION	= 1 << 2,
		eARTICULATION_BODY0			= 1 << 3,
		eARTICULATION_BODY1			= 1 << 4,
		eDYNAMIC_BODY0				= 1 << 5,
		eDYNAMIC_BODY1				= 1 << 6,
		eSOFT_BODY					= 1 << 7,
		eMODIFIABLE_CONTACT			= 1 << 8,
		eFORCE_THRESHOLD			= 1 << 9,
		eDETECT_DISCRETE_CONTACT	= 1 << 10,
		eHAS_KINEMATIC_ACTOR		= 1 << 11,
		eDISABLE_RESPONSE			= 1 << 12,
		eDETECT_CCD_CONTACTS		= 1 << 13,
		eDOMINANCE_0				= 1 << 14,
		eDOMINANCE_1				= 1 << 15,
	};
};

struct PxcNpWorkUnitStatusFlag
{
	enum Enum
	{
		eHAS_NO_TOUCH				= (1 << 0),
		eHAS_TOUCH					= (1 << 1),
		//eHAS_SOLVER_CONSTRAINTS		= (1 << 2),
		eREQUEST_CONSTRAINTS		= (1 << 3),
		eHAS_CCD_RETOUCH			= (1 << 4),	// Marks pairs that are touching at a CCD pass and were touching at discrete collision or at a previous CCD pass already
												// but we can not tell whether they lost contact in a pass before. We send them as pure eNOTIFY_TOUCH_CCD events to the 
												// contact report callback if requested.
		eDIRTY_MANAGER				= (1 << 5),
		eREFRESHED_WITH_TOUCH		= (1 << 6),
		eTOUCH_KNOWN				= eHAS_NO_TOUCH | eHAS_TOUCH	// The touch status is known (if narrowphase never ran for a pair then no flag will be set)
	};
};

struct PxcNpWorkUnit
{
	const PxsRigidCore*	mRigidCore0;				// INPUT								//8
	const PxsRigidCore*	mRigidCore1;				// INPUT								//16
		
	private:
	const void*			mShapeCoreAndType0;			// INPUT								//24
	const void*			mShapeCoreAndType1;			// INPUT								//32
	public:

	PxU8*				mCCDContacts;				// OUTPUT								//40
	PxU8*				mFrictionDataPtr;			// INOUT								//48

	PxU16				mFlags;						// INPUT								//50
	PxU8				mFrictionPatchCount;		// INOUT 								//51
	PxU8				mStatusFlags;				// OUTPUT (see PxcNpWorkUnitStatusFlag) //52

	PxReal				mRestDistance;				// INPUT								//56

	PxU32				mTransformCache0;			//										//60
	PxU32				mTransformCache1;			//										//64
	
	IG::EdgeIndex		mEdgeIndex;					//inout the island gen edge index		//68
	PxU32				mNpIndex;					//INPUT									//72

	PxReal				mTorsionalPatchRadius;												//76
	PxReal				mMinTorsionalPatchRadius;											//80
	PxReal				mOffsetSlop;														//84
																							//88 pading

	///////////////////////////////////////////////////////////////////////////

	PX_FORCE_INLINE	const void*	encode(const PxsShapeCore* shapeCore)
	{
		const PxU64 type = PxU64(shapeCore->mGeometry.getType());
		PxU64 data = PxU64(shapeCore);
		PX_ASSERT(!(data & 15));
		data |= type;
		return reinterpret_cast<const void*>(data);
	}

	PX_FORCE_INLINE	void	setShapeCore0(const PxsShapeCore* shapeCore)
	{
		mShapeCoreAndType0 = encode(shapeCore);
	}

	PX_FORCE_INLINE	void	setShapeCore1(const PxsShapeCore* shapeCore)
	{
		mShapeCoreAndType1 = encode(shapeCore);
	}

	PX_FORCE_INLINE	const PxsShapeCore*	getShapeCore0()	const
	{
		return reinterpret_cast<const PxsShapeCore*>(PxU64(mShapeCoreAndType0) & ~15);
	}

	PX_FORCE_INLINE	const PxsShapeCore*	getShapeCore1()	const
	{
		return reinterpret_cast<const PxsShapeCore*>(PxU64(mShapeCoreAndType1) & ~15);
	}

	PX_FORCE_INLINE	PxGeometryType::Enum	getGeomType0()	const
	{
		return PxGeometryType::Enum(PxU64(mShapeCoreAndType0) & 15);
	}

	PX_FORCE_INLINE	PxGeometryType::Enum	getGeomType1()	const
	{
		return PxGeometryType::Enum(PxU64(mShapeCoreAndType1) & 15);
	}

	///////////////////////////////////////////////////////////////////////////

	PX_FORCE_INLINE	PxU8	getDominance0()	const
	{
		return (mFlags & PxcNpWorkUnitFlag::eDOMINANCE_0) ? 0 : 1;
	}

	PX_FORCE_INLINE	void	setDominance0(PxU8 v)
	{
		if(v==0)
			mFlags |= PxcNpWorkUnitFlag::eDOMINANCE_0;
		else
			mFlags &= ~PxcNpWorkUnitFlag::eDOMINANCE_0;
	}

	PX_FORCE_INLINE	PxU8	getDominance1()	const
	{
		return (mFlags & PxcNpWorkUnitFlag::eDOMINANCE_1) ? 0 : 1;
	}

	PX_FORCE_INLINE	void	setDominance1(PxU8 v)
	{
		if(v==0)
			mFlags |= PxcNpWorkUnitFlag::eDOMINANCE_1;
		else
			mFlags &= ~PxcNpWorkUnitFlag::eDOMINANCE_1;
	}

	PX_FORCE_INLINE	void	setInvMassScaleFromDominance(PxConstraintInvMassScale& invMassScales)	const
	{
		const PxReal dominance0 = getDominance0() ? 1.0f : 0.0f;
		const PxReal dominance1 = getDominance1() ? 1.0f : 0.0f;

		invMassScales.linear0 = invMassScales.angular0 = dominance0;
		invMassScales.linear1 = invMassScales.angular1 = dominance1;
	}

	///////////////////////////////////////////////////////////////////////////

	PX_FORCE_INLINE void	clearCachedState()
	{
		mFrictionDataPtr = NULL;
		mFrictionPatchCount = 0;
		mCCDContacts = NULL;
	}
};

}

#endif
