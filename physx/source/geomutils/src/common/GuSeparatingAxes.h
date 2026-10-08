// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_SEPARATINGAXES_H
#define GU_SEPARATINGAXES_H

#include "foundation/PxVec3.h"
#include "common/PxPhysXCommonConfig.h"

namespace physx
{
namespace Gu
{
	// PT: this is a number of axes. Multiply by sizeof(PxVec3) for size in bytes.
	#define SEP_AXIS_FIXED_MEMORY	256

	// This class holds a list of potential separating axes.
	// - the orientation is irrelevant so V and -V should be the same vector
	// - the scale is irrelevant so V and n*V should be the same vector
	// - a given separating axis should appear only once in the class
#if PX_VC 
    #pragma warning(push)
	#pragma warning( disable : 4251 ) // class needs to have dll-interface to be used by clients of class
#endif
	class SeparatingAxes
	{
	public:
		PX_INLINE SeparatingAxes() : mNbAxes(0)	{}

		bool addAxis(const PxVec3& axis);

		PX_FORCE_INLINE const PxVec3* getAxes() const
		{
			return mAxes;
		}

		PX_FORCE_INLINE PxU32 getNumAxes() const
		{
			return mNbAxes;
		}

		PX_FORCE_INLINE void reset()
		{
			mNbAxes = 0;
		}

	private:
		PxU32	mNbAxes;
		PxVec3	mAxes[SEP_AXIS_FIXED_MEMORY];
	};
#if PX_VC 
     #pragma warning(pop) 
#endif

	enum PxcSepAxisType
	{
		SA_NORMAL0,		// Normal of object 0
		SA_NORMAL1,		// Normal of object 1
		SA_EE			// Cross product of edges
	};

}
}

#endif
