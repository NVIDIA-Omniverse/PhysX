// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNI_PVD_COMMANDS_H
#define OMNI_PVD_COMMANDS_H

struct OmniPvdCommand
{
	enum Enum
	{
		eINVALID,
		eREGISTER_CLASS,
		eREGISTER_ENUM,
		eREGISTER_ATTRIBUTE,
		eREGISTER_CLASS_ATTRIBUTE,
		eREGISTER_UNIQUE_LIST_ATTRIBUTE,
		eSET_ATTRIBUTE,
		eADD_TO_UNIQUE_LIST_ATTRIBUTE,
		eREMOVE_FROM_UNIQUE_LIST_ATTRIBUTE,
		eCREATE_OBJECT,
		eDESTROY_OBJECT,
		eSTART_FRAME,
		eSTOP_FRAME,
		eRECORD_MESSAGE
	};
};

#endif
