// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_ARTICULATION_TENDON_H
#define PXG_ARTICULATION_TENDON_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"

namespace physx
{

	class PxgArticulationTendonElementFixedData
	{
	public:
		PxU64	children;			//my children index						//8		8
		PxU32	parent;				//parent index							//4		12
		PxU32	linkInd;			//articulation link index				//4		16
	};


	class PxgArticulationTendon
	{
	public:
		void*							mFixedElements; //element fix in the initialization
		void*							mModElements; //element can be modified in run time
		PxU32							mNbElements;
	};

}//namespace physx

#endif