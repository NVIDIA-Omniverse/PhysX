// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PX_ANISOTROPY_DATA_H
#define PX_ANISOTROPY_DATA_H


#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"

#if !PX_DOXYGEN
namespace physx
{
	/**
	\brief Data and settings to apply smoothing to an array of particle positions
	*/
	struct PxSmoothedPositionData
	{
		PxVec4* mPositions;	//!< The gpu array with the positions
		PxReal mSmoothing;	//!< The strength of the smoothing
	};

	/**
	\brief Data and settings to compute anisotropy information for an array of particle positions
	*/
	struct PxAnisotropyData
	{
		PxVec4* mAnisotropy_q1; //!< Gpu array containing the first direction (x, y, z) and magnitude (w) of the anisotropy
		PxVec4* mAnisotropy_q2; //!< Gpu array containing the second direction (x, y, z) and magnitude (w) of the anisotropy
		PxVec4* mAnisotropy_q3; //!< Gpu array containing the third direction (x, y, z) and magnitude (w) of the anisotropy
		PxReal mAnisotropy;		//!< Anisotropy scaling factor
		PxReal mAnisotropyMin;	//!< Lower anisotropy bound
		PxReal mAnisotropyMax;  //!< Upper anisotropy bound
		PxU32 mPadding;
	};

#endif	

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
