// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuSqInternal.h"
#include "CmVisualization.h"
#include "GuAABBTree.h"
#include "GuAABBTreeNode.h"
#include "GuIncrementalAABBTree.h"
#include "GuBVH.h"

using namespace physx;
using namespace Cm;
using namespace Gu;

static void drawBVH(const BVHNode* root, const BVHNode* node, PxRenderOutput& out_)
{
	renderOutputDebugBox(out_, node->mBV);
	if(node->isLeaf())
		return;
	drawBVH(root, node->getPos(root), out_);
	drawBVH(root, node->getNeg(root), out_);
}

void visualizeTree(PxRenderOutput& out, PxU32 color, const BVH* tree)
{
	if(tree && tree->getNodes())
	{
		out << PxTransform(PxIdentity);
		out << color;
		drawBVH(tree->getNodes(), tree->getNodes(), out);
	}
}

void visualizeTree(PxRenderOutput& out, PxU32 color, const AABBTree* tree)
{
	if(tree && tree->getNodes())
	{
		out << PxTransform(PxIdentity);
		out << color;
		drawBVH(tree->getNodes(), tree->getNodes(), out);
	}
}

void visualizeTree(PxRenderOutput& out, PxU32 color, const IncrementalAABBTree* tree, DebugVizCallback* cb)
{
	if(tree && tree->getNodes())
	{
		struct Local
		{
			static void _draw(const IncrementalAABBTreeNode* root, const IncrementalAABBTreeNode* node, PxRenderOutput& out_, DebugVizCallback* cb_)
			{
				PxBounds3 bounds;
				V4StoreU(node->mBVMin, &bounds.minimum.x);
				PX_ALIGN(16, PxVec4) max4;
				V4StoreA(node->mBVMax, &max4.x);
				bounds.maximum = PxVec3(max4.x, max4.y, max4.z);

				bool discard = false;
				if(cb_)
					discard = cb_->visualizeNode(*node, bounds);

				if(!discard)
					Cm::renderOutputDebugBox(out_, bounds);

				if(node->isLeaf())
					return;
				_draw(root, node->getPos(root), out_, cb_);
				_draw(root, node->getNeg(root), out_, cb_);
			}
		};
		out << PxTransform(PxIdentity);
		out << color;
		Local::_draw(tree->getNodes(), tree->getNodes(), out, cb);
	}
}

