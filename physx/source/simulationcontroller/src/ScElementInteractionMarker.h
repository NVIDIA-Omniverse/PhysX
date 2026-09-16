// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_ELEMENT_INTERACTION_MARKER_H
#define SC_ELEMENT_INTERACTION_MARKER_H

#include "ScElementSimInteraction.h"
#include "ScNPhaseCore.h"

namespace physx
{
namespace Sc
{
	class ElementInteractionMarker : public ElementSimInteraction
	{
	public:
		PX_INLINE		ElementInteractionMarker(ElementSim& element0, ElementSim& element1, bool createParallel/* = false*/);
						~ElementInteractionMarker();
	};

} // namespace Sc


PX_INLINE Sc::ElementInteractionMarker::ElementInteractionMarker(ElementSim& element0, ElementSim& element1, bool createParallel) :
	ElementSimInteraction(element0, element1, InteractionType::eMARKER, InteractionFlag::eRB_ELEMENT|InteractionFlag::eFILTERABLE)
{
	if(!createParallel)
	{
		// PT: no call to onActivate() here, interaction markers are always inactive
		registerInActors();
		Scene& scene = getScene();
		scene.registerInteraction(this, false);
	}
}

}

#endif

