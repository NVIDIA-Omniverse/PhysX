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

#ifndef BP_FILTERING_H
#define BP_FILTERING_H

#include "PxvConfig.h"
#include "foundation/PxAssert.h"

namespace physx
{
	namespace Bp
	{

#define BP_USE_AGGREGATE_GROUP_TAIL	1
#define BP_FILTERING_TYPE_SHIFT_BIT	3
#define BP_FILTERING_TYPE_MASK		7

	/*
	\brief AABBManager volumes with the same filter group value are guaranteed never to generate an overlap pair.
	\note To ensure that static pairs never overlap, add static shapes with eSTATICS.
	The value eDYNAMICS_BASE provides a minimum recommended group value for dynamic shapes.
	If dynamics shapes are assigned group values greater than or equal to eDYNAMICS_BASE then
	they are allowed to generate broadphase overlaps with statics, and other dynamic shapes provided 
	they have different group values.
	\see AABBManager::createVolume
	*/
	struct FilterGroup
	{
		enum Enum
		{
			eSTATICS		= 0,
			eDYNAMICS_BASE	= 1,
#if BP_USE_AGGREGATE_GROUP_TAIL
			eAGGREGATE_BASE	= 0xfffffffe,
#endif
			eINVALID		= 0xffffffff
		};
	};

	struct FilterType
	{
		enum Enum
		{
			STATIC				= 0,
			KINEMATIC			= 1,
			DYNAMIC				= 2,
			AGGREGATE			= 3,
			DEFORMABLE_SURFACE	= 4,
			DEFORMABLE_VOLUME	= 5,
			PARTICLESYSTEM		= 6,

			COUNT				= 7
		};
	};

	PX_FORCE_INLINE Bp::FilterGroup::Enum getFilterGroup_Statics()
	{
		return Bp::FilterGroup::eSTATICS;
	}

	PX_FORCE_INLINE	Bp::FilterGroup::Enum	getFilterGroup_Dynamics(PxU32 rigidId, bool isKinematic)
	{
		const PxU32 group = rigidId + Bp::FilterGroup::eDYNAMICS_BASE;
		const PxU32 type = isKinematic ? FilterType::KINEMATIC : FilterType::DYNAMIC;
		return Bp::FilterGroup::Enum((group<< BP_FILTERING_TYPE_SHIFT_BIT)|type);
	}

	PX_FORCE_INLINE	Bp::FilterGroup::Enum	getFilterGroup(bool isStatic, PxU32 rigidId, bool isKinematic)
	{
		return isStatic ? getFilterGroup_Statics() : getFilterGroup_Dynamics(rigidId, isKinematic);
	}

	PX_FORCE_INLINE bool groupFiltering(const Bp::FilterGroup::Enum group0, const Bp::FilterGroup::Enum group1, const bool* PX_RESTRICT lut)
	{
/*		const int g0 = group0 & ~3;
		const int g1 = group1 & ~3;
		if(g0==g1)
			return false;*/
		if(group0==group1)
		{
			PX_ASSERT((group0 & ~BP_FILTERING_TYPE_MASK)==(group1 & ~BP_FILTERING_TYPE_MASK));
			return false;
		}

		const int type0 = group0 & BP_FILTERING_TYPE_MASK;
		const int type1 = group1 & BP_FILTERING_TYPE_MASK;
		return lut[type0*Bp::FilterType::COUNT+type1];
	}

	class BpFilter
	{
		public:
									BpFilter(bool discardKineKine, bool discardStaticKine);
									~BpFilter();

		PX_FORCE_INLINE	const bool*	getLUT()	const	{ return &mLUT[0][0];	}

						bool		mLUT[Bp::FilterType::COUNT][Bp::FilterType::COUNT];
	};
	}
}

#endif

