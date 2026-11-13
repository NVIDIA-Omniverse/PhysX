// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ScArticulationJointSim.h"
#include "ScArticulationJointCore.h"
#include "ScBodySim.h"
#include "ScArticulationSim.h"
#include "ScArticulationCore.h"

using namespace physx;

Sc::ArticulationJointSim::ArticulationJointSim(ArticulationJointCore& joint, ActorSim& parent, ActorSim& child) :
	Interaction	(parent, child, InteractionType::eARTICULATION, 0),
	mCore		(joint)
{
	{
		onActivate();
		registerInActors();
	}

	BodySim& childBody = static_cast<BodySim&>(child),
		   & parentBody = static_cast<BodySim&>(parent);

	parentBody.getArticulation()->addBody(childBody, &parentBody, this);

	mCore.setSim(this);
}

Sc::ArticulationJointSim::~ArticulationJointSim()
{
	// articulation interactions do not make use of the dirty flags yet. If they did, a setClean(true) has to be introduced here.
	PX_ASSERT(!readInteractionFlag(InteractionFlag::eIN_DIRTY_LIST));
	PX_ASSERT(!getDirtyFlags());

	unregisterFromActors();

	mCore.setSim(NULL);
}

bool Sc::ArticulationJointSim::onActivate()
{
	const Sc::BodySim& parent = static_cast<BodySim&>(getActorSim0());
	const Sc::BodySim& child = static_cast<BodySim&>(getActorSim1());

	if(!(parent.isActive() && child.isActive()))
		return false;

	raiseInteractionFlag(InteractionFlag::eIS_ACTIVE);
	return true; 
}

bool Sc::ArticulationJointSim::onDeactivate()
{
	clearInteractionFlag(InteractionFlag::eIS_ACTIVE);
	return true;
}

void Sc::ArticulationJointSim::setDirty()
{
	Dy::ArticulationJointCore& llCore = mCore.getCore();
	ArticulationSim* sim = mCore.getArticulation()->getSim();
	sim->setJointDirty(llCore);
}
