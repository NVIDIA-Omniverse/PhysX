// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_ARTICULATION_JOINT_SIM_H
#define SC_ARTICULATION_JOINT_SIM_H

#include "ScInteraction.h"

namespace physx
{
namespace Sc
{

class ArticulationJointCore;
class BodySim;

class ArticulationJointSim : public Interaction
{
	PX_NOCOPY(ArticulationJointSim)
public:
											ArticulationJointSim(ArticulationJointCore& joint, ActorSim& parent, ActorSim& child);
											~ArticulationJointSim();

					bool					onActivate();
					bool					onDeactivate();

	PX_FORCE_INLINE	ArticulationJointCore&	getCore()	const	{ return mCore; }

					BodySim&				getParent()	const;
					BodySim&				getChild()	const;

					void					setDirty();
private:
					ArticulationJointCore&	mCore;
};

} // namespace Sc

}

#endif
