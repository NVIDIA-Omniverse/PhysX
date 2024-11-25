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

#include "foundation/PxBounds3.h"
#include "geometry/PxCapsuleGeometry.h"
#include "foundation/PxIntrinsics.h"
#include "GuInternal.h"
#include "GuBox.h"
#include "GuVecPlane.h"
#include "foundation/PxVecMath.h"

using namespace physx;
using namespace aos;

/**
Computes the aabb points.
\param		pts	[out] 8 box points
*/
void Gu::computeBoxPoints(const PxBounds3& bounds, PxVec3* PX_RESTRICT pts)
{
	PX_ASSERT(pts);

	// Get box corners
	const PxVec3& minimum = bounds.minimum;
	const PxVec3& maximum = bounds.maximum;

	//     7+------+6			0 = ---
	//     /|     /|			1 = +--
	//    / |    / |			2 = ++-
	//   / 4+---/--+5			3 = -+-
	// 3+------+2 /    y   z	4 = --+
	//  | /    | /     |  /		5 = +-+
	//  |/     |/      |/		6 = +++
	// 0+------+1      *---x	7 = -++

	// Generate 8 corners of the bbox
	pts[0] = PxVec3(minimum.x, minimum.y, minimum.z);
	pts[1] = PxVec3(maximum.x, minimum.y, minimum.z);
	pts[2] = PxVec3(maximum.x, maximum.y, minimum.z);
	pts[3] = PxVec3(minimum.x, maximum.y, minimum.z);
	pts[4] = PxVec3(minimum.x, minimum.y, maximum.z);
	pts[5] = PxVec3(maximum.x, minimum.y, maximum.z);
	pts[6] = PxVec3(maximum.x, maximum.y, maximum.z);
	pts[7] = PxVec3(minimum.x, maximum.y, maximum.z);
}

PxPlane Gu::getPlane(const PxTransform& pose)
{ 
	const PxVec3 n = pose.q.getBasisVector0();
	return PxPlane(n, -pose.p.dot(n)); 
}

void Gu::computeSweptBox(Gu::Box& dest, const PxVec3& extents, const PxVec3& center, const PxMat33& rot, const PxVec3& unitDir, PxReal distance)
{
	PxVec3 R1, R2;
	PxComputeBasisVectors(unitDir, R1, R2);

	PxReal dd[3];
	dd[0] = PxAbs(rot.column0.dot(unitDir));
	dd[1] = PxAbs(rot.column1.dot(unitDir));
	dd[2] = PxAbs(rot.column2.dot(unitDir));
	PxReal dmax = dd[0];
	PxU32 ax0=1;
	PxU32 ax1=2;
	if(dd[1]>dmax)
	{
		dmax=dd[1];
		ax0=0;
		ax1=2;
	}
	if(dd[2]>dmax)
	{
		dmax=dd[2];
		ax0=0;
		ax1=1;
	}
	if(dd[ax1]<dd[ax0])
		PxSwap(ax0, ax1);

	R1 = rot[ax0];
	R1 -= (R1.dot(unitDir))*unitDir;	// Project to plane whose normal is dir
	R1.normalize();
	R2 = unitDir.cross(R1);

	dest.setAxes(unitDir, R1, R2);

	PxReal offset[3];
	offset[0] = distance;
	offset[1] = distance*(unitDir.dot(R1));
	offset[2] = distance*(unitDir.dot(R2));

	for(PxU32 r=0; r<3; r++)
	{
		const PxVec3& R = dest.rot[r];
		dest.extents[r] = offset[r]*0.5f + PxAbs(rot.column0.dot(R))*extents.x + PxAbs(rot.column1.dot(R))*extents.y + PxAbs(rot.column2.dot(R))*extents.z;
	}

	dest.center = center + unitDir*distance*0.5f;
}
