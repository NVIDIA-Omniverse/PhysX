// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef CM_RENDER_BUFFER_H
#define CM_RENDER_BUFFER_H

#include "common/PxRenderBuffer.h"
#include "CmUtils.h"
#include "foundation/PxArray.h"
#include "foundation/PxUserAllocated.h"

namespace physx
{
namespace Cm
{
	/**
	Implementation of PxRenderBuffer.
	*/
	class RenderBuffer : public PxRenderBuffer, public PxUserAllocated
	{

		template <typename T>
		void append(PxArray<T>& dst, const T* src, PxU32 count)
		{
			dst.reserve(dst.size() + count);
			for(const T* end=src+count; src<end; ++src)
				dst.pushBack(*src);
		}

	public:

		RenderBuffer() :
			mPoints("renderBufferPoints"),
			mLines("renderBufferLines"),
			mTriangles("renderBufferTriangles")
		{}
		

		virtual PxU32 getNbPoints() const PX_OVERRIDE { return mPoints.size(); }
		virtual const PxDebugPoint* getPoints() const PX_OVERRIDE { return mPoints.begin(); }
		virtual void addPoint(const PxDebugPoint& point) PX_OVERRIDE { mPoints.pushBack(point); }

		virtual PxU32 getNbLines() const PX_OVERRIDE { return mLines.size(); }
		virtual const PxDebugLine* getLines() const PX_OVERRIDE { return mLines.begin(); }
		virtual void addLine(const PxDebugLine& line) PX_OVERRIDE { mLines.pushBack(line); }
		virtual PxDebugLine* reserveLines(const PxU32 nbLines) PX_OVERRIDE {return reserveContainerMemory(mLines, nbLines);}

		virtual PxDebugPoint* reservePoints(const PxU32 nbPoints) PX_OVERRIDE { return reserveContainerMemory(mPoints, nbPoints); }

		virtual PxU32 getNbTriangles() const PX_OVERRIDE { return mTriangles.size(); }
		virtual const PxDebugTriangle* getTriangles() const PX_OVERRIDE { return mTriangles.begin(); }
		virtual void addTriangle(const PxDebugTriangle& triangle) PX_OVERRIDE { mTriangles.pushBack(triangle); }

		virtual void append(const PxRenderBuffer& other) PX_OVERRIDE
		{
			append(mPoints, other.getPoints(), other.getNbPoints());
			append(mLines, other.getLines(), other.getNbLines());
			append(mTriangles, other.getTriangles(), other.getNbTriangles());
		}

		virtual void clear() PX_OVERRIDE
		{
			mPoints.clear(); 
			mLines.clear();
			mTriangles.clear();
		}

		virtual bool empty() const PX_OVERRIDE
		{
			return mPoints.empty() && mLines.empty() && mTriangles.empty();
		}

		virtual void shift(const PxVec3& delta) PX_OVERRIDE
		{
			for(PxU32 i=0; i < mPoints.size(); i++)
				mPoints[i].pos += delta;

			for(PxU32 i=0; i < mLines.size(); i++)
			{
				mLines[i].pos0 += delta;
				mLines[i].pos1 += delta;
			}

			for(PxU32 i=0; i < mTriangles.size(); i++)
			{
				mTriangles[i].pos0 += delta;
				mTriangles[i].pos1 += delta;
				mTriangles[i].pos2 += delta;
			}
		}

		PxArray<PxDebugPoint>		mPoints;
		PxArray<PxDebugLine>		mLines;
		PxArray<PxDebugTriangle>	mTriangles;
	};

} // Cm

}

#endif
