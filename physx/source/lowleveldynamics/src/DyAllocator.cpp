// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "DyAllocator.h"

namespace physx
{
namespace Dy
{
// PT: the various error messages we used here look the same but they are slightly different. We could unify this really.

const char* gConstraintDataErrorMsg_Null_Joints =
	"Reached limit set by PxSceneDesc::maxNbContactDataBlocks - ran out of buffer space for constraint prep. "
	"Either accept joints detaching/exploding or increase buffer size allocated for constraint prep by increasing PxSceneDesc::maxNbContactDataBlocks.";

const char* gConstraintDataErrorMsg_TooLarge_Joints =
	"Attempting to allocate more than 16K of constraint data. "
	"Either accept joints detaching/exploding or simplify constraints.";

const char* gConstraintDataErrorMsg_Null_Contacts =
	"Reached limit set by PxSceneDesc::maxNbContactDataBlocks - ran out of buffer space for constraint prep. "
	"Either accept dropped contacts or increase buffer size allocated for narrow phase by increasing PxSceneDesc::maxNbContactDataBlocks.";

const char* gConstraintDataErrorMsg_TooLarge_Contacts =
	"Attempting to allocate more than 16K of contact data for a single contact pair in constraint prep. "
	"Either accept dropped contacts or simplify collision geometry.";

const char* gFrictionDataErrorMsg_TooLarge =
	"Attempting to allocate more than 16K of friction data for a single contact pair in constraint prep. "
	"Either accept dropped contacts or simplify collision geometry.";
}
}
