// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_CONSTRAINT_INTERACTION_H
#define SC_CONSTRAINT_INTERACTION_H

#include "ScInteraction.h"

namespace physx
{
namespace Sc
{
	class ConstraintSim;
	class RigidSim;

	class ConstraintInteraction : public Interaction
	{
	public:
										ConstraintInteraction(ConstraintSim* shader, RigidSim& r0, RigidSim& r1);
										~ConstraintInteraction();

						bool			onActivate();
						bool			onDeactivate();

						void			updateState();
						void			destroy();  // disables the interaction and unregisters from the system. Does NOT delete the object. This is used on destruction but also when a constraint breaks.

		PX_FORCE_INLINE	ConstraintSim*	getConstraint()			{ return mConstraint;	}
		PX_FORCE_INLINE	IG::EdgeIndex	getEdgeIndex()	const	{ return mEdgeIndex;	}

	private:
						ConstraintSim*	mConstraint;
						IG::EdgeIndex	mEdgeIndex;
	};

} // namespace Sc

}

#endif
