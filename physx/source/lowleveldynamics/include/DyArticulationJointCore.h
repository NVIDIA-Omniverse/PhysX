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

#ifndef DY_ARTICULATION_JOINT_CORE_H
#define DY_ARTICULATION_JOINT_CORE_H

#include "DyArticulationCore.h"
#include "solver/PxSolverDefs.h"
#include "PxArticulationJointReducedCoordinate.h"
#include "CmSpatialVector.h"

namespace physx
{
	namespace Dy
	{
		class ArticulationJointCoreData;

		PX_ALIGN_PREFIX(16)
		struct ArticulationJointCore
		{
		public:

			// PX_SERIALIZATION
			ArticulationJointCore(const PxEMPTY&) : jointDirtyFlag(PxEmpty) 
			{ 
				PX_COMPILE_TIME_ASSERT(sizeof(PxArticulationMotions) == sizeof(PxU8)); 
			}
			//~PX_SERIALIZATION

			ArticulationJointCore(const PxTransform& parentFrame, const PxTransform& childFrame)
			{
				//PxMarkSerializedMemory(this, sizeof(ArticulationJointCore));
				init(parentFrame, childFrame);
			}

			// PT: these ones don't update the dirty flags
			PX_FORCE_INLINE	void	initLimit(PxArticulationAxis::Enum axis, const PxArticulationLimit& limit)	{ limits[axis] = limit;					}
			PX_FORCE_INLINE	void	initDrive(PxArticulationAxis::Enum axis, const PxArticulationDrive& drive)	{ drives[axis] = drive;					}
			PX_FORCE_INLINE	void	initJointType(PxArticulationJointType::Enum type)							{ jointType = PxU8(type);				}
			PX_FORCE_INLINE	void	initMaxJointVelocity(const PxReal maxJointV)								{ maxJointVelocity = maxJointV;			}
			PX_FORCE_INLINE	void	initFrictionCoefficient(const PxReal coefficient)							{ frictionCoefficient = coefficient;	}

			void	init(const PxTransform& parentFrame, const PxTransform& childFrame)
			{
				PX_ASSERT(parentFrame.isValid());
				PX_ASSERT(childFrame.isValid());

				parentPose			= parentFrame;
				childPose			= childFrame;
				jointOffset			= 0;
				// PT: TODO: don't we need ArticulationJointCoreDirtyFlag::eFRAME here?
				jointDirtyFlag		= ArticulationJointCoreDirtyFlag::eMOTION;

				initFrictionCoefficient(0.05f);
				initMaxJointVelocity(100.0f);
				initJointType(PxArticulationJointType::eUNDEFINED);

				for(PxU32 i=0; i<PxArticulationAxis::eCOUNT; i++)
				{
					initLimit(PxArticulationAxis::Enum(i), PxArticulationLimit(0.0f, 0.0f));
					initDrive(PxArticulationAxis::Enum(i), PxArticulationDrive(0.0f, 0.0f, 0.0f, PxArticulationDriveType::eNONE));

					targetP[i] = 0.0f;
					targetV[i] = 0.0f;
					armature[i] = 0.0f;
					jointPos[i] = 0.0f;
					jointVel[i] = 0.0f;

					dofIds[i] = 0xff;
					invDofIds[i] = 0xff;
					motion[i] = PxArticulationMotion::eLOCKED;
				}
			}
			
			PX_CUDA_CALLABLE void setJointFrame(Cm::UnAlignedSpatialVector* motionMatrix, 
                                                const Cm::UnAlignedSpatialVector* jointAxis,
                                                PxQuat& relativeQuat,
												const PxU32 dofs)
			{
				if (jointDirtyFlag & ArticulationJointCoreDirtyFlag::eFRAME)
				{
					relativeQuat = (childPose.q * (parentPose.q.getConjugate())).getNormalized();

					computeMotionMatrix(motionMatrix, jointAxis, dofs);

					jointDirtyFlag &= ~ArticulationJointCoreDirtyFlag::eFRAME;
				}
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE void computeMotionMatrix(Cm::UnAlignedSpatialVector* motionMatrix,
																	  const Cm::UnAlignedSpatialVector* jointAxis,
																	  const PxU32 dofs)
			{
				const PxVec3 childOffset = -childPose.p;

				switch (jointType)
				{
				case PxArticulationJointType::ePRISMATIC:
				{
					const Cm::UnAlignedSpatialVector& jJointAxis = jointAxis[0];
					const PxVec3 u = (childPose.rotate(jJointAxis.bottom)).getNormalized();

					motionMatrix[0] = Cm::UnAlignedSpatialVector(PxVec3(0.f), u);

					PX_ASSERT(dofs == 1);

					break;
				}
				case PxArticulationJointType::eREVOLUTE:
				case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
				{
					const Cm::UnAlignedSpatialVector& jJointAxis = jointAxis[0];
					const PxVec3 u = (childPose.rotate(jJointAxis.top)).getNormalized();
					const PxVec3 uXd = u.cross(childOffset);

					motionMatrix[0] = Cm::UnAlignedSpatialVector(u, uXd);

					break;
				}
				case PxArticulationJointType::eSPHERICAL:
				{

					for (PxU32 ind = 0; ind < dofs; ++ind)
					{
						const Cm::UnAlignedSpatialVector& jJointAxis = jointAxis[ind];
						const PxVec3 u = (childPose.rotate(jJointAxis.top)).getNormalized();

						const PxVec3 uXd = u.cross(childOffset);
						motionMatrix[ind] = Cm::UnAlignedSpatialVector(u, uXd);
					}

					break;
				}
				case PxArticulationJointType::eFIX:
				{
					PX_ASSERT(dofs == 0);
					break;
				}
				default:
					break;
				}
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE void operator=(ArticulationJointCore& other)
			{
				parentPose = other.parentPose;
				childPose = other.childPose;

				//KS - temp place to put reduced coordinate limit and drive values
				for(PxU32 i=0; i<PxArticulationAxis::eCOUNT; i++)
				{
					limits[i] = other.limits[i];
					drives[i] = other.drives[i];
					targetP[i] = other.targetP[i];
					targetV[i] = other.targetV[i];
					armature[i] = other.armature[i];

					jointPos[i] = other.jointPos[i];
					jointVel[i] = other.jointVel[i];

					dofIds[i] = other.dofIds[i];
					invDofIds[i] = other.invDofIds[i];
					motion[i] = other.motion[i];
				}

				frictionCoefficient = other.frictionCoefficient;
				maxJointVelocity = other.maxJointVelocity;
				jointOffset = other.jointOffset;
				jointDirtyFlag = other.jointDirtyFlag;
				jointType = other.jointType;
			}

			PX_FORCE_INLINE	void	setParentPose(const PxTransform& t)										{ parentPose = t;			jointDirtyFlag |= ArticulationJointCoreDirtyFlag::eFRAME;				}
			PX_FORCE_INLINE	void	setChildPose(const PxTransform& t)										{ childPose = t;			jointDirtyFlag |= ArticulationJointCoreDirtyFlag::eFRAME;				}
			PX_FORCE_INLINE	void	setMotion(PxArticulationAxis::Enum axis, PxArticulationMotion::Enum m)	{ motion[axis] = PxU8(m);	jointDirtyFlag |= Dy::ArticulationJointCoreDirtyFlag::eMOTION;			}
			PX_FORCE_INLINE	void	setTargetP(PxArticulationAxis::Enum axis, PxReal value)					{ targetP[axis] = value;	jointDirtyFlag |= Dy::ArticulationJointCoreDirtyFlag::eTARGETPOSE;		}
			PX_FORCE_INLINE	void	setTargetV(PxArticulationAxis::Enum axis, PxReal value)					{ targetV[axis] = value;	jointDirtyFlag |= Dy::ArticulationJointCoreDirtyFlag::eTARGETVELOCITY;	}
			PX_FORCE_INLINE	void	setArmature(PxArticulationAxis::Enum axis, PxReal value)				{ armature[axis] = value;	jointDirtyFlag |= Dy::ArticulationJointCoreDirtyFlag::eARMATURE;		}

			// attachment points, don't change the order, otherwise it will break GPU code
			PxTransform						parentPose;								//28		28
			PxTransform						childPose;								//28		56

			//KS - temp place to put reduced coordinate limit and drive values
			PxArticulationLimit				limits[PxArticulationAxis::eCOUNT];		//48		104
			PxArticulationDrive				drives[PxArticulationAxis::eCOUNT];		//96		200
			PxReal							targetP[PxArticulationAxis::eCOUNT];	//24		224
			PxReal							targetV[PxArticulationAxis::eCOUNT];	//24		248
			PxReal							armature[PxArticulationAxis::eCOUNT];	//24		272
			
			PxReal							jointPos[PxArticulationAxis::eCOUNT];	//24		296	
			PxReal							jointVel[PxArticulationAxis::eCOUNT];	//24		320
			
			PxReal							frictionCoefficient;					//4			324
			PxReal							maxJointVelocity;						//4			328

			//this is the dof offset for the joint in the cache. 
			PxU32							jointOffset;							//4			332

			PxU8							dofIds[PxArticulationAxis::eCOUNT];		//6			338
			PxU8							motion[PxArticulationAxis::eCOUNT];		//6			344
			PxU8							invDofIds[PxArticulationAxis::eCOUNT];	//6			350

			ArticulationJointCoreDirtyFlags	jointDirtyFlag;							//1			351
			PxU8							jointType;								//1			352
		}PX_ALIGN_SUFFIX(16);
	}
}

#endif
