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

			ArticulationJointCoreData() : jointOffset(0xffffffff), dofConstraintMask(0)
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

			PX_FORCE_INLINE PxU8 configureJointDofs(ArticulationJointCore* joint, Cm::UnAlignedSpatialVector* jointAxis)
			{
					nbDof = 0;
					dofLimitMask = 0;

					//KS - no need to zero memory here.
					//PxMemZero(jointAxis, sizeof(jointAxis));

					for (PxU8 i = 0; i < DY_MAX_DOF; ++i)
					{
						if (joint->motion[i] != PxArticulationMotion::eLOCKED)
						{
							Cm::UnAlignedSpatialVector axis = Cm::UnAlignedSpatialVector::Zero();
							//axis is in the local space of joint
							axis[i] = 1.f;

							jointAxis[nbDof] = axis;

							joint->invDofIds[i] = nbDof;
							joint->dofIds[nbDof] = i;

							if (joint->motion[i] == PxArticulationMotion::eLIMITED)
								dofLimitMask |= 1 << nbDof;

							nbDof++;
						}
					}
			
				return nbDof;
			}

			PxU32	jointOffset;				//4
			//degree of freedom
			PxU8	nbDof;						//1
			PxU8	dofConstraintMask;	//1
			PxU8	dofLimitMask;					//1	

		};

	}//namespace Dy
}

#endif
