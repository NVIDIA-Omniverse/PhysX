// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_FEATHERSTONE_ARTICULATION_JOINT_DATA_H
#define DY_FEATHERSTONE_ARTICULATION_JOINT_DATA_H

#include "foundation/PxVec3.h"
#include "foundation/PxQuat.h"
#include "foundation/PxTransform.h"
#include "foundation/PxVecMath.h"
#include "CmUtils.h"
#include "CmSpatialVector.h"
#include "DyVArticulation.h"
#include "DyFeatherstoneArticulationUtils.h"
#include "DyArticulationJointCore.h"
#include <stdio.h>

namespace physx
{
	namespace Dy
	{
		class ArticulationJointCoreData
		{
		public:

			ArticulationJointCoreData() : jointOffset(0xffffffff)
			{
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE PxU8 countJointDofs(ArticulationJointCore* joint) const
			{
				PxU8 tDof = 0;

				for (PxU32 i = 0; i < DY_MAX_DOF; ++i)
				{
					if (joint->motion[i] != PxArticulationMotion::eLOCKED)
					{
						tDof++;
					}
				}

				return tDof;
			}

			PX_FORCE_INLINE PxU8 configureJointDofs(ArticulationJointCore* joint)
			{
				nbDof = 0;
				dofLimitMask = 0;

				for (PxU8 i = 0; i < DY_MAX_DOF; ++i)
				{
					if (joint->motion[i] != PxArticulationMotion::eLOCKED)
					{
						joint->invDofIds[i] = nbDof;
						joint->dofIds[nbDof] = i;

						if (joint->motion[i] == PxArticulationMotion::eLIMITED)
							dofLimitMask |= 1 << nbDof;

						nbDof++;
					}
				}
			
				return nbDof;
			}

			PxU32	jointOffset;	//4
			//degree of freedom
			PxU8	nbDof;			//1
			PxU8	dofLimitMask;	//1	
		};

	}//namespace Dy
}

#endif
