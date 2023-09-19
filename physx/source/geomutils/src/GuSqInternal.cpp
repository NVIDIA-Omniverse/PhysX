// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

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

