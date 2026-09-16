// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuConvexEdgeFlags.h"
#include "GuFeatureCode.h"

using namespace physx;
using namespace Gu;

static FeatureCode computeFeatureCode(PxReal u, PxReal v)
{
	// Analysis
	if(u==0.0f)
	{
		if(v==0.0f)
		{
			// Vertex 0
			return FC_VERTEX0;
		}
		else if(v==1.0f)
		{
			// Vertex 2
			return FC_VERTEX2;
		}
		else
		{
			// Edge 0-2
			return FC_EDGE20;
		}
	}
	else if(u==1.0f)
	{
		if(v==0.0f)
		{
			// Vertex 1
			return FC_VERTEX1;
		}
	}
	else
	{
		if(v==0.0f)
		{
			// Edge 0-1
			return FC_EDGE01;
		}
		else
		{
			if((u+v)>=0.9999f)
			{
				// Edge 1-2
				return FC_EDGE12;
			}
			else
			{
				// Face
				return FC_FACE;
			}
		}
	}
	return FC_UNDEFINED;
}


bool Gu::selectNormal(PxU8 data, PxReal u, PxReal v)
{
	bool useFaceNormal = false;
	const FeatureCode FC = computeFeatureCode(u, v);
	switch(FC)
	{
		case FC_VERTEX0:
			if(!(data & (Gu::ETD_CONVEX_EDGE_01|Gu::ETD_CONVEX_EDGE_20)))
				useFaceNormal = true;
			break;
		case FC_VERTEX1:
			if(!(data & (Gu::ETD_CONVEX_EDGE_01|Gu::ETD_CONVEX_EDGE_12)))
				useFaceNormal = true;
			break;
		case FC_VERTEX2:
			if(!(data & (Gu::ETD_CONVEX_EDGE_12|Gu::ETD_CONVEX_EDGE_20)))
				useFaceNormal = true;
			break;
		case FC_EDGE01:
			if(!(data & Gu::ETD_CONVEX_EDGE_01))
				useFaceNormal = true;
			break;
		case FC_EDGE12:
			if(!(data & Gu::ETD_CONVEX_EDGE_12))
				useFaceNormal = true;
			break;
		case FC_EDGE20:
			if(!(data & Gu::ETD_CONVEX_EDGE_20))
				useFaceNormal = true;
			break;
		case FC_FACE:
			useFaceNormal = true;
			break;
		case FC_UNDEFINED:
			break;
	};
	return useFaceNormal;
}

