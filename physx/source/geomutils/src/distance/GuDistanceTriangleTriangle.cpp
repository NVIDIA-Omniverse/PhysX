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

#include "GuDistanceTriangleTriangle.h"
#include "foundation/PxVecMath.h"

using namespace physx;
using namespace Gu;
using namespace aos;

void edgeEdgeDist(PxVec3& x, PxVec3& y, const PxVec3& p, const PxVec3& a, const PxVec3& q, const PxVec3& b);

float Gu::distanceTriangleTriangleSquared(PxVec3& cp, PxVec3& cq, const PxVec3p p[3], const PxVec3p q[3])
{
	PxVec3p Sv[3];
	V4StoreU(V4Sub(V4LoadU(&p[1].x), V4LoadU(&p[0].x)), &Sv[0].x);
	V4StoreU(V4Sub(V4LoadU(&p[2].x), V4LoadU(&p[1].x)), &Sv[1].x);
	V4StoreU(V4Sub(V4LoadU(&p[0].x), V4LoadU(&p[2].x)), &Sv[2].x);

	PxVec3p Tv[3];
	V4StoreU(V4Sub(V4LoadU(&q[1].x), V4LoadU(&q[0].x)), &Tv[0].x);
	V4StoreU(V4Sub(V4LoadU(&q[2].x), V4LoadU(&q[1].x)), &Tv[1].x);
	V4StoreU(V4Sub(V4LoadU(&q[0].x), V4LoadU(&q[2].x)), &Tv[2].x);

	PxVec3 minP, minQ;
	bool shown_disjoint = false;

	float mindd = PX_MAX_F32;

	for(PxU32 i=0;i<3;i++)
	{
		for(PxU32 j=0;j<3;j++)
		{
			edgeEdgeDist(cp, cq, p[i], Sv[i], q[j], Tv[j]);
			const PxVec3 V = cq - cp;
			const float dd = V.dot(V);

			if(dd<=mindd)
			{
				minP = cp;
				minQ = cq;
				mindd = dd;

				PxU32 id = i+2;
				if(id>=3)
					id-=3;
				PxVec3 Z = p[id] - cp;
				float a = Z.dot(V);
				id = j+2;
				if(id>=3)
					id-=3;
				Z = q[id] - cq;
				float b = Z.dot(V);

				if((a<=0.0f) && (b>=0.0f))
					return V.dot(V);

						if(a<=0.0f)	a = 0.0f;
				else	if(b>0.0f)	b = 0.0f;

				if((mindd - a + b) > 0.0f)
					shown_disjoint = true;
			}
		}
	}

	PxVec3 Sn = Sv[0].cross(Sv[1]);
	float Snl = Sn.dot(Sn);

	if(Snl>1e-15f)
	{
		const PxVec3 Tp((p[0] - q[0]).dot(Sn),
						(p[0] - q[1]).dot(Sn),
						(p[0] - q[2]).dot(Sn));

		int index = -1;
		if((Tp[0]>0.0f) && (Tp[1]>0.0f) && (Tp[2]>0.0f))
		{
			if(Tp[0]<Tp[1])		index = 0; else index = 1;
			if(Tp[2]<Tp[index])	index = 2;
		}
		else if((Tp[0]<0.0f) && (Tp[1]<0.0f) && (Tp[2]<0.0f))
		{
			if(Tp[0]>Tp[1])		index = 0; else index = 1;
			if(Tp[2]>Tp[index])	index = 2;
		}

		if(index >= 0) 
		{
			shown_disjoint = true;

			const PxVec3& qIndex = q[index];

			PxVec3 V = qIndex - p[0];
			PxVec3 Z = Sn.cross(Sv[0]);
			if(V.dot(Z)>0.0f)
			{
				V = qIndex - p[1];
				Z = Sn.cross(Sv[1]);
				if(V.dot(Z)>0.0f)
				{
					V = qIndex - p[2];
					Z = Sn.cross(Sv[2]);
					if(V.dot(Z)>0.0f)
					{
						cp = qIndex + Sn * Tp[index]/Snl;
						cq = qIndex;
						return (cp - cq).magnitudeSquared();
					}
				}
			}
		}
	}

	PxVec3 Tn = Tv[0].cross(Tv[1]);
	float Tnl = Tn.dot(Tn);
  
	if(Tnl>1e-15f)
	{
		const PxVec3 Sp((q[0] - p[0]).dot(Tn),
						(q[0] - p[1]).dot(Tn),
						(q[0] - p[2]).dot(Tn));

		int index = -1;
		if((Sp[0]>0.0f) && (Sp[1]>0.0f) && (Sp[2]>0.0f))
		{
			if(Sp[0]<Sp[1])		index = 0; else index = 1;
			if(Sp[2]<Sp[index])	index = 2;
		}
		else if((Sp[0]<0.0f) && (Sp[1]<0.0f) && (Sp[2]<0.0f))
		{
			if(Sp[0]>Sp[1])		index = 0; else index = 1;
			if(Sp[2]>Sp[index])	index = 2;
		}

		if(index >= 0)
		{ 
			shown_disjoint = true;

			const PxVec3& pIndex = p[index];

			PxVec3 V = pIndex - q[0];
			PxVec3 Z = Tn.cross(Tv[0]);
			if(V.dot(Z)>0.0f)
			{
				V = pIndex - q[1];
				Z = Tn.cross(Tv[1]);
				if(V.dot(Z)>0.0f)
				{
					V = pIndex - q[2];
					Z = Tn.cross(Tv[2]);
					if(V.dot(Z)>0.0f)
					{
						cp = pIndex;
						cq = pIndex + Tn * Sp[index]/Tnl;
						return (cp - cq).magnitudeSquared();
					}
				}
			}
		}
	}

	if(shown_disjoint)
	{
		cp = minP;
		cq = minQ;
		return mindd;
	}
	else return 0.0f;
}
