// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_INTERACTION_FLAGS_H
#define SC_INTERACTION_FLAGS_H


namespace physx
{

namespace Sc
{
	struct InteractionFlag	// PT: TODO: use PxFlags
	{
		enum Enum
		{
			eRB_ELEMENT			= (1 << 0), // Interactions between rigid body shapes
			eCONSTRAINT			= (1 << 1),
			eFILTERABLE			= (1 << 2), // Interactions that go through the filter code
			eIN_DIRTY_LIST		= (1 << 3),	// The interaction is in the dirty list
			eIS_FILTER_PAIR		= (1 << 4),	// The interaction is tracked by the filter callback mechanism
			eIS_ACTIVE			= (1 << 5)
		};
	};

	struct InteractionDirtyFlag
	{
		enum Enum
		{
			eFILTER_STATE		= (1 << 0), // All changes filtering related
			eBODY_KINEMATIC		= (1 << 1) | eFILTER_STATE,  // A transition between dynamic and kinematic (and vice versa) require a refiltering
			eDOMINANCE			= (1 << 2),
			eREST_OFFSET		= (1 << 3),
			eVISUALIZATION		= (1 << 4)
		};
	};


} // namespace Sc


} // namespace physx


#endif

