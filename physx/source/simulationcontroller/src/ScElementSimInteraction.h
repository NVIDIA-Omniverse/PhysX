// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_ELEMENT_SIM_INTERACTION_H
#define SC_ELEMENT_SIM_INTERACTION_H

#include "ScInteraction.h"
#include "ScElementSim.h"

namespace physx
{
namespace Sc
{
	class ElementSimInteraction : public Interaction
	{
	public:
		PX_FORCE_INLINE	ElementSim&	getElement0()	const	{ return mElement0;	}
		PX_FORCE_INLINE	ElementSim&	getElement1()	const	{ return mElement1;	}

	protected:
		PX_INLINE					ElementSimInteraction(ElementSim& element0, ElementSim& element1, InteractionType::Enum type, PxU8 flags);
									~ElementSimInteraction() {}

		ElementSimInteraction& operator=(const ElementSimInteraction&);

						ElementSim&		mElement0;
						ElementSim&		mElement1;
						PxU32			mFlags;		// PT: moved there in padding bytes, from ShapeInteraction
	public:
						IG::EdgeIndex	mEdgeIndex;	// PT: moved there in padding bytes, from ShapeInteraction
	};

} // namespace Sc

//////////////////////////////////////////////////////////////////////////

PX_INLINE Sc::ElementSimInteraction::ElementSimInteraction(ElementSim& element0, ElementSim& element1, InteractionType::Enum type, PxU8 flags) :
	Interaction		(element0.getActor(), element1.getActor(), type, flags),
	mElement0		(element0),
	mElement1		(element1)
{
}


}

#endif
