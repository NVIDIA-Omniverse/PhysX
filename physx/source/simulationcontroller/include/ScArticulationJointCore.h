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

#ifndef SC_ARTICULATION_JOINT_CORE_H
#define SC_ARTICULATION_JOINT_CORE_H

#include "foundation/PxTransform.h"
#include "common/PxMetaData.h"
#include "DyVArticulation.h"

namespace physx
{
namespace Sc
{
	class BodyCore;
	class ArticulationJointSim;
	class ArticulationCore;

	class ArticulationJointDesc
	{
	public:
		BodyCore*	parent;
		BodyCore*	child;
		PxTransform	parentPose;
		PxTransform	childPose;
	};

	class ArticulationJointCore
	{
	public:
// PX_SERIALIZATION
															ArticulationJointCore(const PxEMPTY) : mCore(PxEmpty), mSim(NULL) {}
						void								preExportDataReset() { mCore.jointDirtyFlag = Dy::ArticulationJointCoreDirtyFlag::eALL; }
		static			void								getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
															ArticulationJointCore(const PxTransform& parentFrame, const PxTransform& childFrame);
															~ArticulationJointCore();

		//Those methods are not allowed while the articulation are in the scene											
		PX_FORCE_INLINE	const PxTransform&					getParentPose() const { return mCore.parentPose; }
						void								setParentPose(const PxTransform&);

		PX_FORCE_INLINE	const PxTransform&					getChildPose() const { return mCore.childPose; }
						void								setChildPose(const PxTransform&);

		//Those functions doesn't change the articulation configuration so the application is allowed to change those value in run-time
		PX_FORCE_INLINE	PxArticulationLimit					getLimit(PxArticulationAxis::Enum axis)		const	{ return mCore.limits[axis];	}
						void								setLimit(PxArticulationAxis::Enum axis, const PxArticulationLimit& limit);

		PX_FORCE_INLINE	PxArticulationDrive					getDrive(PxArticulationAxis::Enum axis)		const	{ return mCore.drives[axis];	}
						void								setDrive(PxArticulationAxis::Enum axis, const PxArticulationDrive& drive);

						void								setTargetP(PxArticulationAxis::Enum axis, PxReal targetP);
		PX_FORCE_INLINE	PxReal								getTargetP(PxArticulationAxis::Enum axis)	const	{ return mCore.targetP[axis];	}

						void								setTargetV(PxArticulationAxis::Enum axis, PxReal targetV);
		PX_FORCE_INLINE	PxReal								getTargetV(PxArticulationAxis::Enum axis)	const	{ return mCore.targetV[axis];	}

						void								setArmature(PxArticulationAxis::Enum axis, PxReal armature);
		PX_FORCE_INLINE	PxReal								getArmature(PxArticulationAxis::Enum axis)	const	{ return mCore.armature[axis];	}

						void								setJointPosition(PxArticulationAxis::Enum axis, const PxReal jointPos);
						PxReal								getJointPosition(PxArticulationAxis::Enum axis)	const;

						void								setJointVelocity(PxArticulationAxis::Enum axis, const PxReal jointVel);
						PxReal								getJointVelocity(PxArticulationAxis::Enum axis)	const;

		// PT: TODO: don't we need to set ArticulationJointCoreDirtyFlag::eMOTION here?
		PX_FORCE_INLINE	void								setMotion(PxArticulationAxis::Enum axis, PxArticulationMotion::Enum motion)	{ mCore.motion[axis] = PxU8(motion);						}
		PX_FORCE_INLINE	PxArticulationMotion::Enum			getMotion(PxArticulationAxis::Enum axis)							const	{ return PxArticulationMotion::Enum(mCore.motion[axis]);	}

		PX_FORCE_INLINE	void								setJointType(PxArticulationJointType::Enum type)	{ mCore.initJointType(type);								}
		PX_FORCE_INLINE	PxArticulationJointType::Enum		getJointType()								const	{ return PxArticulationJointType::Enum(mCore.jointType);	}
						
		PX_FORCE_INLINE	void								setFrictionCoefficient(const PxReal coefficient)	{ mCore.initFrictionCoefficient(coefficient);	}
		PX_FORCE_INLINE	PxReal								getFrictionCoefficient()					const	{ return mCore.frictionCoefficient;				}

		PX_FORCE_INLINE	void								setMaxJointVelocity(const PxReal maxJointV)			{ mCore.initMaxJointVelocity(maxJointV);		}
		PX_FORCE_INLINE	PxReal								getMaxJointVelocity()						const	{ return mCore.maxJointVelocity;				}

		PX_FORCE_INLINE	ArticulationJointSim*					getSim()									const	{ return mSim;	}
		PX_FORCE_INLINE	void									setSim(ArticulationJointSim* sim)
																{
																	PX_ASSERT((sim==0) ^ (mSim == 0));
																	mSim = sim;
																}

		PX_FORCE_INLINE	Dy::ArticulationJointCore&				getCore()											{ return mCore;					}

		PX_FORCE_INLINE void									setArticulation(ArticulationCore* articulation)		{ mArticulation = articulation;	}
		PX_FORCE_INLINE	const ArticulationCore*					getArticulation()							const	{ return mArticulation;			}

		PX_FORCE_INLINE void									setRoot(PxArticulationJointReducedCoordinate* base)	{ mRootType = base;				}
		PX_FORCE_INLINE PxArticulationJointReducedCoordinate*	getRoot()									const	{ return mRootType;				}

		PX_FORCE_INLINE void									setLLIndex(const PxU32 llLinkIndex)					{ mLLLinkIndex = llLinkIndex;	}
	private:
						void									setSimDirty();
		PX_FORCE_INLINE	void									setDirty(Dy::ArticulationJointCoreDirtyFlag::Enum dirtyFlag)
																{
																	mCore.jointDirtyFlag |= dirtyFlag;
																	setSimDirty();
																}

						Dy::ArticulationJointCore				mCore;
						ArticulationJointSim*					mSim;
						ArticulationCore*						mArticulation;
						PxArticulationJointReducedCoordinate*	mRootType;
						PxU32									mLLLinkIndex;
#if PX_P64_FAMILY
						PxU32									pad;
#endif
	};

} // namespace Sc

}

#endif
