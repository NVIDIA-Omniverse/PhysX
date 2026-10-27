// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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

	class ConstraintSim
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
		PX_FORCE_INLINE	BodySim*				getBody(PxU32 i)					const	// for static actors or world attached constraints NULL is returned
												{
													return mBodies[i];
												}

						void					getForce(PxVec3& force, PxVec3& torque);

		PX_FORCE_INLINE	PxU8					readFlag(PxU8 flag)	const	{ return PxU8(mFlags & flag);						}
		PX_FORCE_INLINE	void					setFlag(PxU8 flag)			{ mFlags |= flag;									}
		PX_FORCE_INLINE	void					clearFlag(PxU8 flag)		{ mFlags &= ~flag;									}
		PX_FORCE_INLINE	PxU32					isBroken()			const	{ return PxU32(mFlags) & ConstraintSim::eBROKEN;	}

		PX_FORCE_INLINE const ConstraintInteraction*	getInteraction() const { return mInteraction; }

						PxConstraintGPUIndex	getGPUIndex() const;

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
