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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

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
		

		virtual PxU32 getNbPoints() const { return mPoints.size(); }
		virtual const PxDebugPoint* getPoints() const { return mPoints.begin(); }
		virtual void addPoint(const PxDebugPoint& point) { mPoints.pushBack(point); }

		virtual PxU32 getNbLines() const { return mLines.size(); }
		virtual const PxDebugLine* getLines() const { return mLines.begin(); }
		virtual void addLine(const PxDebugLine& line) { mLines.pushBack(line); }
		virtual PxDebugLine* reserveLines(const PxU32 nbLines) {return reserveContainerMemory(mLines, nbLines);}

		virtual PxDebugPoint* reservePoints(const PxU32 nbPoints) { return reserveContainerMemory(mPoints, nbPoints); }

		virtual PxU32 getNbTriangles() const { return mTriangles.size(); }
		virtual const PxDebugTriangle* getTriangles() const { return mTriangles.begin(); }
		virtual void addTriangle(const PxDebugTriangle& triangle) { mTriangles.pushBack(triangle); }

		virtual void append(const PxRenderBuffer& other)
		{
			append(mPoints, other.getPoints(), other.getNbPoints());
			append(mLines, other.getLines(), other.getNbLines());
			append(mTriangles, other.getTriangles(), other.getNbTriangles());
		}

		virtual void clear()
		{
			mPoints.clear(); 
			mLines.clear();
			mTriangles.clear();
		}

		virtual bool empty() const 
		{
			return mPoints.empty() && mLines.empty() && mTriangles.empty();
		}

		virtual void shift(const PxVec3& delta)
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
