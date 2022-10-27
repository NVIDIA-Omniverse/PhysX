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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
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

			ArticulationJointCoreData() : jointOffset(0xffffffff), dofInternalConstraintMask(0)
			{
			}


			PX_CUDA_CALLABLE PX_FORCE_INLINE void computeMotionMatrix(ArticulationJointCore* joint,
				Cm::UnAlignedSpatialVector* motionMatrix, const Cm::UnAlignedSpatialVector* jointAxis)
			{
				const PxVec3 childOffset = -joint->childPose.p;

				//transpose(Tc)*S = 0
				//transpose(Ta)*S = 1
				switch (joint->jointType)
				{
				case PxArticulationJointType::ePRISMATIC:
				{
					const Cm::UnAlignedSpatialVector& jJointAxis = jointAxis[0];
					const PxVec3 u = (joint->childPose.rotate(jJointAxis.bottom)).getNormalized();

					motionMatrix[0] = Cm::UnAlignedSpatialVector(PxVec3(0.f), u);

					PX_ASSERT(dof == 1);

					break;
				}
				case PxArticulationJointType::eREVOLUTE:
				case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
				{
					const Cm::UnAlignedSpatialVector& jJointAxis = jointAxis[0];
					const PxVec3 u = (joint->childPose.rotate(jJointAxis.top)).getNormalized();
					const PxVec3 uXd = u.cross(childOffset);

					motionMatrix[0] = Cm::UnAlignedSpatialVector(u, uXd);

					break;
				}
				case PxArticulationJointType::eSPHERICAL:
				{

					for (PxU32 ind = 0; ind <dof; ++ind)
					{
						const Cm::UnAlignedSpatialVector& jJointAxis = jointAxis[ind];
						const PxVec3 u = (joint->childPose.rotate(jJointAxis.top)).getNormalized();

						const PxVec3 uXd = u.cross(childOffset);
						motionMatrix[ind] = Cm::UnAlignedSpatialVector(u, uXd);
					}

					break;
				}
				case PxArticulationJointType::eFIX:
				{
					PX_ASSERT(dof == 0);
					break;
				}
				default:
					break;
				}
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE PxU8 computeJointDofs(ArticulationJointCore* joint) const
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

			PX_CUDA_CALLABLE PX_FORCE_INLINE void computeJointAxis(const ArticulationJointCore* joint, Cm::UnAlignedSpatialVector* jointAxis)
			{
				for (PxU32 i = 0; i < dof; ++i)
				{
					PxU32 ind = joint->dofIds[i];

					Cm::UnAlignedSpatialVector axis = Cm::UnAlignedSpatialVector::Zero();
					//axis is in the local space of joint
					axis[ind] = 1.f;

					jointAxis[i] = axis;
				}
			}

			PX_FORCE_INLINE PxU32 computeJointDof(ArticulationJointCore* joint, const bool forceRecompute,
				Cm::UnAlignedSpatialVector* jointAxis)
			{
				if (joint->jointDirtyFlag & ArticulationJointCoreDirtyFlag::eMOTION || forceRecompute)
				{

					dof = 0;
					limitMask = 0;

					//KS - no need to zero memory here.
					//PxMemZero(jointAxis, sizeof(jointAxis));

					for (PxU8 i = 0; i < DY_MAX_DOF; ++i)
					{
						if (joint->motion[i] != PxArticulationMotion::eLOCKED)
						{
							Cm::UnAlignedSpatialVector axis = Cm::UnAlignedSpatialVector::Zero();
							//axis is in the local space of joint
							axis[i] = 1.f;

							jointAxis[dof] = axis;

							joint->invDofIds[i] = dof;
							joint->dofIds[dof] = i;

							if (joint->motion[i] == PxArticulationMotion::eLIMITED)
								limitMask |= 1 << dof;

							dof++;
						}
					}
				}

				return dof;

			}

			PxU32								jointOffset;					//4
			//degree of freedom
			PxU8								dof;							//1
			PxU8								dofInternalConstraintMask;		//1
			PxU8								limitMask;						//1	

		};

		

		struct ArticulationJointTargetData
		{
			PxReal								targetJointVelocity[3];			//12
			PxReal								targetJointPosition[3];			//24
			PxReal								armature[3];					//36


			ArticulationJointTargetData()
			{
				for (PxU32 i = 0; i < 3; ++i)
				{
					targetJointPosition[i] = 0.f;
					targetJointVelocity[i] = 0.f;
					armature[i] = 0.f;
				}
			}



			PX_CUDA_CALLABLE PX_FORCE_INLINE void setJointVelocityDrive(ArticulationJointCore* joint, const PxU32 dof)
			{
				if (joint->jointDirtyFlag & ArticulationJointCoreDirtyFlag::eTARGETVELOCITY)
				{
					for (PxU32 i = 0; i < dof; ++i)
					{
						PxU32 ind = joint->dofIds[i];

						targetJointVelocity[i] = joint->targetV[ind];
					}
					
					joint->jointDirtyFlag &= ~ArticulationJointCoreDirtyFlag::eTARGETVELOCITY;
				}
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE void setJointPoseDrive(ArticulationJointCore* joint, const PxU32 dof)
			{
				if (joint->jointDirtyFlag & ArticulationJointCoreDirtyFlag::eTARGETPOSE)
				{
					for (PxU32 i = 0; i < dof; ++i)
					{
						PxU32 ind = joint->dofIds[i];

						targetJointPosition[i] = joint->targetP[ind];
					}

					joint->jointDirtyFlag &= ~ArticulationJointCoreDirtyFlag::eTARGETPOSE;
				}
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE void setArmature(ArticulationJointCore* joint, const PxU32 dof)
			{
				if (joint->jointDirtyFlag & ArticulationJointCoreDirtyFlag::eARMATURE)
				{

					for (PxU32 i = 0; i < dof; ++i)
					{
						PxU32 ind = joint->dofIds[i];

						armature[i] = joint->armature[ind];
					}

					joint->jointDirtyFlag &= ~ArticulationJointCoreDirtyFlag::eARMATURE;
				}
			}

		};

	}//namespace Dy
}

#endif
