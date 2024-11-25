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

#include "ExtInsideTester.h"
#include "foundation/PxBounds3.h"

using namespace physx;
using namespace Ext;

// ------------------------------------------------------------------------
void InsideTester::init(const PxVec3 *vertices, PxI32 numVertices, const PxI32 *triIndices, PxI32 numTris)
{
	PxArray<PxI32> newIds(numVertices, -1);

	mVertices.clear();
	mIndices.clear();

	for (PxI32 i = 0; i < 3 * numTris; i++)
	{
		PxI32 id = triIndices[i];
		if (newIds[id] < 0) 
		{
			newIds[id] = PxI32(mVertices.size());
			mVertices.pushBack(vertices[id]);
		}
		mIndices.pushBack(newIds[id]);
	}

	mGrids[0].init(0, mVertices, mIndices);
	mGrids[1].init(1, mVertices, mIndices);
	mGrids[2].init(2, mVertices, mIndices);
}

// ------------------------------------------------------------------------
bool InsideTester::isInside(const PxVec3 &pos)
{
	PxI32 vote = 0;
	vote += mGrids[0].numInside(pos, mVertices, mIndices);
	vote += mGrids[1].numInside(pos, mVertices, mIndices);
	vote += mGrids[2].numInside(pos, mVertices, mIndices);
	return (vote > 3);
}

// ------------------------------------------------------------------------
void InsideTester::Grid2d::init(PxI32 _dim0, const PxArray<PxVec3> &vertices, const PxArray<PxI32> &indices)
{
	first.clear();
	tris.clear();
	next.clear();
	num1 = num2 = 0;

	this->dim0 = _dim0;
	PxI32 dim1 = (_dim0 + 1) % 3;
	PxI32 dim2 = (_dim0 + 2) % 3;

	PxI32 numTris = PxI32(indices.size()) / 3;
	if (numTris == 0)
		return;

	PxBounds3 bounds, triBounds;
	bounds.setEmpty();
	PxReal avgSize = 0.0f;

	for (PxI32 i = 0; i < numTris; i++)
	{
		triBounds.setEmpty();
		triBounds.include(vertices[indices[3 * i]]);
		triBounds.include(vertices[indices[3 * i + 1]]);
		triBounds.include(vertices[indices[3 * i + 2]]);
		triBounds.minimum[dim0] = 0.0f;
		triBounds.maximum[dim0] = 0.0f;
		avgSize += triBounds.getDimensions().magnitude();
		bounds.include(triBounds);
	}
	if (bounds.isEmpty())
		return;

	avgSize /= PxReal(numTris);

	orig = bounds.minimum;
	spacing = avgSize;

	num1 = PxI32((bounds.maximum[dim1] - orig[dim1]) / spacing) + 2;
	num2 = PxI32((bounds.maximum[dim2] - orig[dim2]) / spacing) + 2;
	first.clear();
	first.resize(num1 * num2, -1);

	for (PxI32 i = 0; i < numTris; i++) 
	{
		triBounds.setEmpty();
		triBounds.include(vertices[indices[3 * i]] - orig);
		triBounds.include(vertices[indices[3 * i + 1]] - orig);
		triBounds.include(vertices[indices[3 * i + 2]] - orig);

		PxI32 min1 = PxI32(triBounds.minimum[dim1] / spacing);
		PxI32 min2 = PxI32(triBounds.minimum[dim2] / spacing);
		PxI32 max1 = PxI32(triBounds.maximum[dim1] / spacing);
		PxI32 max2 = PxI32(triBounds.maximum[dim2] / spacing);
		for (PxI32 i1 = min1; i1 <= max1; i1++)
		{
			for (PxI32 i2 = min2; i2 <= max2; i2++) 
			{
				PxI32 nr = i1 * num2 + i2;
				next.pushBack(first[nr]);
				first[nr] = PxI32(tris.size());
				tris.pushBack(i);
			}
		}
	}
}

// ------------------------------------------------------------------------
PxI32 InsideTester::Grid2d::numInside(const PxVec3 &pos, const PxArray<PxVec3> &vertices, const PxArray<PxI32> &indices)
{
	if (first.empty())
		return 0;

	PxI32 dim1 = (dim0 + 1) % 3;
	PxI32 dim2 = (dim0 + 2) % 3;

	PxReal r = 1e-5f;
	PxVec3 p = pos;

	p[dim1] = pos[dim1] + rnd.rand(0.0f, r);
	p[dim2] = pos[dim2] + rnd.rand(0.0f, r);

	PxI32 i1 = PxI32((p[dim1] - orig[dim1]) / spacing);
	PxI32 i2 = PxI32((p[dim2] - orig[dim2]) / spacing);
	if (i1 < 0 || i1 >= num1 || i2 < 0 || i2 >= num2)
		return false;

	PxI32 count1 = 0;
	PxI32 count2 = 0;

	PxI32 nr = first[i1 * num2 + i2];
	while (nr >= 0) 
	{
		PxI32 triNr = tris[nr];
		nr = next[nr];

		const PxVec3 &p0 = vertices[indices[3 * triNr]];
		const PxVec3 &p1 = vertices[indices[3 * triNr + 1]];
		const PxVec3 &p2 = vertices[indices[3 * triNr + 2]];

		bool side0 = (p1 - p0).cross(p - p0)[dim0] > 0.0f;
		bool side1 = (p2 - p1).cross(p - p1)[dim0] > 0.0f;
		bool side2 = (p0 - p2).cross(p - p2)[dim0] > 0.0f;
		if (side0 != side1 || side1 != side2)
			continue;

		// ray triangle intersection

		PxVec3 n = (p1 - p0).cross(p2 - p0);
		if (n[dim0] == 0.0f)
			continue;
		PxReal t = (p0 - p).dot(n) / n[dim0];
		if (t > 0.0f)
			count1++;
		else if (t < 0.0f)
			count2++;
	}

	PxI32 num = 0;
	if ((count1 % 2) == 1)
		num++;
	if ((count2 % 2) == 1)
		num++;
	return num;
}
