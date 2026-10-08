// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ScElementInteractionMarker.h"
#include "ScNPhaseCore.h"

using namespace physx;

Sc::ElementInteractionMarker::~ElementInteractionMarker()
{
	if(isRegistered())
		getScene().unregisterInteraction(this);

	unregisterFromActors();
}

