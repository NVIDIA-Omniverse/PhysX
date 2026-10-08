// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_EDGE_TYPE_H
#define PXG_EDGE_TYPE_H

namespace physx
{
//This is the same as IG::Edge::EdgeType, but we have more enum type so we can represent articulation
//contacts and joints

struct PxgEdgeType
{
	enum Enum
	{
		eCONTACT_MANAGER	= 0,
		eCONSTRAINT,
		eARTICULATION_CONTACT,
		eARTICULATION_CONSTRAINT,
		eEDGE_TYPE_COUNT
	};
};

}

#endif
