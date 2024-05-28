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

#ifndef SC_CONSTRAINT_SIM_H
#define SC_CONSTRAINT_SIM_H

#include "foundation/PxArray.h"
#include "PxSimulationEventCallback.h"
#include "DyConstraint.h"

namespace physx
{
namespace Sc
{
	class Scene;
	class ConstraintInteraction;
	class ConstraintCore;
	class RigidCore;
	class BodySim;
	class RigidSim;

	class ConstraintSim : public PxUserAllocated 
	{
		PX_NOCOPY(ConstraintSim)
	public:
		enum Enum
		{
			eBREAKABLE					=	(1<<1),	// The constraint can break
			eCHECK_MAX_FORCE_EXCEEDED	=	(1<<2),	// This constraint will get tested for breakage at the end of the sim step
			eBROKEN						=	(1<<3)
		};
												ConstraintSim(ConstraintCore& core, RigidCore* r0, RigidCore* r1, Scene& scene);
												~ConstraintSim();

						void					setBodies(RigidCore* r0, RigidCore* r1);

						void					setBreakForceLL(PxReal linear, PxReal angular);
		PX_FORCE_INLINE	void					setMinResponseThresholdLL(PxReal threshold)	{ mLowLevelConstraint.minResponseThreshold = threshold;	}
		PX_FORCE_INLINE	const void*				getConstantsLL()					const	{ return mLowLevelConstraint.constantBlock;	}

						void					postFlagChange(PxConstraintFlags oldFlags, PxConstraintFlags newFlags);

		PX_FORCE_INLINE	const Dy::Constraint&	getLowLevelConstraint()				const	{ return mLowLevelConstraint;	}
		PX_FORCE_INLINE	Dy::Constraint&			getLowLevelConstraint()						{ return mLowLevelConstraint;	}
		PX_FORCE_INLINE	ConstraintCore&			getCore()							const	{ return mCore;					}
		PX_FORCE_INLINE	BodySim*				getBody(PxU32 i)					const  // for static actors or world attached constraints NULL is returned
												{
													return mBodies[i];
												}

						void					getForce(PxVec3& force, PxVec3& torque);

		PX_FORCE_INLINE	PxU8					readFlag(PxU8 flag)	const	{ return PxU8(mFlags & flag);						}
		PX_FORCE_INLINE	void					setFlag(PxU8 flag)			{ mFlags |= flag;									}
		PX_FORCE_INLINE	void					clearFlag(PxU8 flag)		{ mFlags &= ~flag;									}
		PX_FORCE_INLINE	PxU32					isBroken()			const	{ return PxU32(mFlags) & ConstraintSim::eBROKEN;	}

		PX_FORCE_INLINE const ConstraintInteraction*	getInteraction() const { return mInteraction; }

	private:
						bool					createLLConstraint();
						void					destroyLLConstraint();

						Dy::Constraint			mLowLevelConstraint;
						Scene&					mScene;
						ConstraintCore&			mCore;
						ConstraintInteraction*	mInteraction;	// PT: why do we have an interaction object here?
						BodySim*				mBodies[2];
						PxU8					mFlags;
	};
} // namespace Sc

}

#endif
