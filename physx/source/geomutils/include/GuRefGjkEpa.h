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

#ifndef GU_SLOW_GJK_EPA_H
#define GU_SLOW_GJK_EPA_H

#include "foundation/PxBasicTemplates.h"
#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"
#include "GuBounds.h"

namespace physx
{
	namespace Gu
	{
		PX_CUDA_CALLABLE PX_FORCE_INLINE
		static PxVec3d V3_To_V3d(const PxVec3& v)
		{
			return PxVec3d(PxF64(v.x), PxF64(v.y), PxF64(v.z));
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE
		static PxVec3 V3d_To_V3(const PxVec3d& v)
		{
			return PxVec3(PxF32(v.x), PxF32(v.y), PxF32(v.z));
		}

		// The reference implementation of GJK-EPA algorithm
		// the implementation uses doubles for maximum accuracy
		namespace RefGjkEpa
		{
			template <typename Support>
			struct Convex
			{
				static const PxU32 MAX_VERTS = 64;

				PX_CUDA_CALLABLE PX_FORCE_INLINE
				Convex(const Support& s, const PxTransform& p)
					:
					mS(s), mPose(p), mNumVerts(0)
				{
					const PxVec3 X(1, 0, 0), Y(0, 1, 0), Z(0, 0, 1);
					const PxBounds3 bounds(PxVec3(mS.supportLocal(-X).x, mS.supportLocal(-Y).y, mS.supportLocal(-Z).z),
						PxVec3(mS.supportLocal(X).x, mS.supportLocal(Y).y, mS.supportLocal(Z).z));
					PxReal maxExtent = PxMax(bounds.getDimensions().maxElement(), FLT_EPSILON);
					mAccuracy = maxExtent * 0.01f;
				}

				PX_CUDA_CALLABLE PX_FORCE_INLINE
				PxF64 getAccuracy() const
				{
					return PxF64(mAccuracy);
				}

				PX_CUDA_CALLABLE PX_FORCE_INLINE
				PxU8 supportIndex(const PxVec3d& dir)
				{
					PxVec3 d = V3d_To_V3(dir);
					PxVec3 v = mPose.transform(mS.supportLocal(mPose.rotateInv(d)));
					PxU32 closest = MAX_VERTS;
					const PxReal WELD_DIST_SQ = mAccuracy * mAccuracy;
					PxReal distSq = FLT_MAX;
					for (PxI32 i = mNumVerts - 1; i >= 0; --i)
					{
						PxReal dSq = (mVerts[i] - v).magnitudeSquared();

						if (dSq < WELD_DIST_SQ)
							return PxU8(i);

						if (distSq > dSq)
						{
							distSq = dSq;
							closest = i;
						}
					}

					PX_ASSERT(closest < MAX_VERTS || mNumVerts == 0);

					if (mNumVerts == MAX_VERTS)
						return PxU8(closest);

					mVerts[mNumVerts++] = v;

					return PxU8(mNumVerts - 1);
				}

				PX_CUDA_CALLABLE PX_FORCE_INLINE
				PxVec3d supportVertex(PxU8 index) const
				{
					PX_ASSERT(index < mNumVerts);
					return V3_To_V3d(mVerts[index]);
				}

			private:
				const Support& mS;
				const PxTransform& mPose;
				PxVec3 mVerts[MAX_VERTS];
				PxU32 mNumVerts;
				PxF32 mAccuracy;
			};

			struct GjkDistance
			{
				struct Vert
				{
					PxVec3d p;
					PxF64 s;
					PxU8 aI, bI;

					PX_CUDA_CALLABLE PX_FORCE_INLINE
					static Vert make(const PxVec3d& p, PxF64 s, PxU8 aI, PxU8 bI)
						{ Vert v; v.p = p; v.s = s; v.aI = aI; v.bI = bI; return v; }
				};

				PX_CUDA_CALLABLE PX_FORCE_INLINE
				GjkDistance()
					:
					mNum(0)
				{}

				PX_CUDA_CALLABLE PX_FORCE_INLINE
				bool addPoint(const PxVec3d& p, PxU8 aI, PxU8 bI)
				{
					for (PxU32 i = 0; i < mNum; ++i)
					{
						const Vert& v = mVerts[i];
						if (v.aI == aI && v.bI == bI)
							return false;
					}

					mVerts[mNum++] = Vert::make(p, 0.0, aI, bI);

					return true;
				}

				PX_CUDA_CALLABLE PX_FORCE_INLINE
				PxVec3d computeClosest()
				{
					switch (mNum)
					{
					case 1:
					{
						mVerts[0].s = 1.0;
						break;
					}
					case 2:
					{
						const PxVec3d a = mVerts[0].p, b = mVerts[1].p;
						const PxVec3d ba = a - b;
						const PxF64 sba = ba.dot(-b);
						if (sba <= 0)
						{
							mVerts[0] = mVerts[1];
							mVerts[0].s = 1.0;
							mNum = 1;
							break;
						}
						mVerts[0].s = sba / ba.magnitudeSquared();
						mVerts[1].s = 1.0 - mVerts[0].s;
						break;
					}
					case 3:
					{
						const PxVec3d a = mVerts[0].p, b = mVerts[1].p, c = mVerts[2].p;
						const PxVec3d ca = a - c, cb = b - c;
						const PxF64 sca = ca.dot(-c), scb = cb.dot(-c);
						if (sca <= 0 && scb <= 0)
						{
							mVerts[0] = mVerts[2];
							mVerts[0].s = 1.0;
							mNum = 1;
							break;
						}
						const PxVec3d abc = (b - a).cross(c - a);
						const PxF64 iabc = 1.0 / abc.magnitudeSquared();
						const PxVec3d pabc = abc * abc.dot(a) * iabc;
						const PxF64 tca = abc.dot((c - pabc).cross(a - pabc));
						if (sca > 0 && tca <= 0)
						{
							mVerts[1] = mVerts[2];
							mVerts[0].s = sca / ca.magnitudeSquared();
							mVerts[1].s = 1.0 - mVerts[0].s;
							mNum = 2;
							break;
						}
						const PxF64 tbc = abc.dot((b - pabc).cross(c - pabc));
						if (scb > 0 && tbc <= 0)
						{
							mVerts[0] = mVerts[1];
							mVerts[1] = mVerts[2];
							mVerts[0].s = scb / cb.magnitudeSquared();
							mVerts[1].s = 1.0 - mVerts[0].s;
							mNum = 2;
							break;
						}
						mVerts[0].s = tbc * iabc;
						mVerts[1].s = tca * iabc;
						mVerts[2].s = 1.0 - mVerts[0].s - mVerts[1].s;
						if (abc.dot(a) > 0) PxSwap(mVerts[0], mVerts[1]); // ???
						break;
					}
					case 4:
					{
						const PxVec3d a = mVerts[0].p, b = mVerts[1].p, c = mVerts[2].p, d = mVerts[3].p;
						const PxVec3d da = a - d, db = b - d, dc = c - d;
						const PxF64 sda = da.dot(-d), sdb = db.dot(-d), sdc = dc.dot(-d);
						if (sda <= 0 && sdb <= 0 && sdc <= 0)
						{
							mVerts[0] = mVerts[3];
							mVerts[0].s = 1.0;
							mNum = 1;
							break;
						}
						const PxVec3d dab = (a - d).cross(b - d);
						const PxF64 idab = 1.0 / dab.magnitudeSquared();
						const PxVec3d pdab = dab * dab.dot(d) * idab;
						const PxVec3d dbc = (b - d).cross(c - d);
						const PxF64 idbc = 1.0 / dbc.magnitudeSquared();
						const PxVec3d pdbc = dbc * dbc.dot(d) * idbc;
						const PxVec3d dca = (c - d).cross(a - d);
						const PxF64 idca = 1.0 / dca.magnitudeSquared();
						const PxVec3d pdca = dca * dca.dot(d) * idca;
						const PxF64 tda = dab.dot((d - pdab).cross(a - pdab));
						const PxF64 tad = dca.dot((a - pdca).cross(d - pdca));
						if (sda > 0 && tda <= 0 && tad <= 0)
						{
							mVerts[1] = mVerts[3];
							mVerts[0].s = sda / da.magnitudeSquared();
							mVerts[1].s = 1.0 - mVerts[0].s;
							mNum = 2;
							break;
						}
						const PxF64 tdb = dbc.dot((d - pdbc).cross(b - pdbc));
						const PxF64 tbd = dab.dot((b - pdab).cross(d - pdab));
						if (sdb > 0 && tdb <= 0 && tbd <= 0)
						{
							mVerts[0] = mVerts[1];
							mVerts[1] = mVerts[3];
							mVerts[0].s = sdb / db.magnitudeSquared();
							mVerts[1].s = 1.0 - mVerts[0].s;
							mNum = 2;
							break;
						}
						const PxF64 tcd = dbc.dot((c - pdbc).cross(d - pdbc));
						const PxF64 tdc = dca.dot((d - pdca).cross(c - pdca));
						if (sdc > 0 && tdc <= 0 && tcd <= 0)
						{
							mVerts[0] = mVerts[2];
							mVerts[1] = mVerts[3];
							mVerts[0].s = sdc / dc.magnitudeSquared();
							mVerts[1].s = 1.0 - mVerts[0].s;
							mNum = 2;
							break;
						}
						if (tda > 0 && tbd > 0 && dab.dot(d) < 0)
						{
							mVerts[2] = mVerts[3];
							mVerts[0].s = tbd * idab;
							mVerts[1].s = tda * idab;
							mVerts[2].s = 1.0 - mVerts[0].s - mVerts[1].s;
							mNum = 3;
							break;
						}
						if (tdb > 0 && tcd > 0 && dbc.dot(d) < 0)
						{
							mVerts[0] = mVerts[1];
							mVerts[1] = mVerts[2];
							mVerts[2] = mVerts[3];
							mVerts[0].s = tcd * idbc;
							mVerts[1].s = tdb * idbc;
							mVerts[2].s = 1.0 - mVerts[0].s - mVerts[1].s;
							mNum = 3;
							break;
						}
						if (tdc > 0 && tad > 0 && dca.dot(d) < 0)
						{
							mVerts[1] = mVerts[3];
							mVerts[0].s = tdc * idca;
							mVerts[2].s = tad * idca;
							mVerts[1].s = 1.0 - mVerts[0].s - mVerts[2].s;
							mNum = 3;
							break;
						}
						return PxVec3d(0);
					}
					}

					PxVec3d closest(0);
					for (PxU32 i = 0; i < mNum; ++i)
						closest += mVerts[i].p * mVerts[i].s;

					return closest;
				}

				template <typename Support>
				PX_CUDA_CALLABLE PX_FORCE_INLINE
				void computePoints(const Convex<Support>& a, const Convex<Support>& b, PxVec3d& pointA, PxVec3d& pointB)
				{
					pointA = pointB = PxVec3d(0);
					for (PxU32 i = 0; i < mNum; ++i)
					{
						pointA += a.supportVertex(mVerts[i].aI) * mVerts[i].s;
						pointB += b.supportVertex(mVerts[i].bI) * mVerts[i].s;
					}
				}

			private:

				Vert mVerts[4];
				PxU32 mNum;
			};

			template <typename Support>
			PX_CUDA_CALLABLE PX_FORCE_INLINE
			static PxReal gjkDistance(const Support& a, const Support& b, const PxTransform& poseA, const PxTransform& poseB, PxReal maxDist, PxVec3& pointA, PxVec3& pointB, PxVec3& axis)
			{
				GjkDistance gjk;
				Convex<Support> convexA(a, poseA), convexB(b, poseB);
				PxF64 epsDist = 0.001 * PxMin(convexA.getAccuracy(), convexB.getAccuracy());

				PxVec3d dir = -V3_To_V3d((poseA.p - poseB.p).magnitudeSquared() > FLT_EPSILON ? (poseA.p - poseB.p).getNormalized() : PxVec3(1, 0, 0));
				PxF64 closestDist = DBL_MAX;

				while (true)
				{
					const PxU8 aI = convexA.supportIndex(dir), bI = convexB.supportIndex(-dir);
					const PxVec3d aP = convexA.supportVertex(aI), bP = convexB.supportVertex(bI);
					const PxVec3d p = aP - bP;
					const PxF64 dist = p.dot(-dir);

					if (dist > PxF64(maxDist))
						return FLT_MAX;

					if (dist >= closestDist - epsDist || !gjk.addPoint(p, aI, bI))
					{
						PxVec3d pA, pB;
						gjk.computePoints(convexA, convexB, pA, pB);
						pointA = V3d_To_V3(pA);
						pointB = V3d_To_V3(pB);
						axis = V3d_To_V3(-dir);
						return PxF32(dist);
					}

					PxVec3d closest = gjk.computeClosest();
					closestDist = closest.magnitude();

					if (closestDist < 1e-6)
						return 0;

					dir = -closest / closestDist;
				}
			}

			struct EpaDepth
			{
				PX_CUDA_CALLABLE PX_FORCE_INLINE
				EpaDepth()
					:
					mNumVerts(0), mNumFaces(0), mNumPlanes(0),
					mClosest(Plane::make(PxVec4d(0, 0, 0, 0), 0, 0, 0))
				{}

				PX_CUDA_CALLABLE PX_FORCE_INLINE
				PxU8 findPlane(PxU8 v0, PxU8 v1, PxU8 v2)
				{
					PxF64 eps = 1e-6;
					PxVec3d a = mVerts[v0].pos, b = mVerts[v1].pos, c = mVerts[v2].pos;

					if (mNumVerts > 3)
					{
						for (PxI32 i = mNumPlanes - 1; i >= 0; --i)
						{
							const PxVec4d plane = mPlanes[i].plane;
							if (PxAbs(plane.dot(PxVec4d(a, 1))) < eps &&
								PxAbs(plane.dot(PxVec4d(b, 1))) < eps &&
								PxAbs(plane.dot(PxVec4d(c, 1))) < eps)
								return PxU8(i);
						}
					}

					const PxVec3d abc = (b - a).cross(c - a);
					const PxF64 labc = abc.magnitude();

					if (labc < eps)
						return 0xff;

					const PxVec3d n = abc / labc;
					const PxF64 d = n.dot(-a);
					const PxVec4d plane(n, d);

					PxF64 epsTest = 1e-5;
					for (PxU32 i = 0; i < mNumVerts; ++i)
						if (plane.dot(PxVec4d(mVerts[i].pos, 1)) > epsTest)
							return 0xff;

					mPlanes[mNumPlanes++] = Plane::make(plane, v0, v1, v2);

					return PxU8(mNumPlanes - 1);
				}

				PX_CUDA_CALLABLE PX_FORCE_INLINE
				bool addPoint(const PxVec3d& p, PxU8 aI, PxU8 bI)
				{
					if (mNumVerts > MAX_VERTS - 1)
						return false;

					for (PxU32 i = 0; i < mNumVerts; ++i)
						if (aI == mVerts[i].aI && bI == mVerts[i].bI)
							return false;

					Vert& v = mVerts[mNumVerts++];
					v = Vert::make(p, aI, bI);

					if (mNumVerts < 3)
						return true;

					if (mNumVerts == 3)
					{
						Face& f0 = mFaces[mNumFaces++];
						f0 = Face::make(findPlane(0, 1, 2), 0, 1, 2);
						Face& f1 = mFaces[mNumFaces++];
						f1 = Face::make(findPlane(1, 0, 2), 1, 0, 2);

						return true;
					}

					struct Edge
					{
						PxU8 v[2];
						PX_CUDA_CALLABLE PX_FORCE_INLINE
						static Edge make(PxU8 v0, PxU8 v1)
							{ Edge e; e.v[0] = v0; e.v[1] = v1; return e; }
					};
					const PxF64 eps = 1e-6;
					Edge edges[MAX_VERTS + MAX_FACES - 2]; // Euler's formula
					PxU32 numEdges = 0;
					for (PxI32 i = mNumPlanes - 1; i >= 0; --i)
					{
						const PxVec4d& plane = mPlanes[i].plane;
						if (plane.dot(PxVec4d(p, 1)) < eps)
							continue;

						for (PxI32 j = mNumFaces - 1; j >= 0; --j)
						{
							Face& f = mFaces[j];
							if (f.p == PxU8(i))
							{
								for (PxU32 k = 0; k < 3; ++k)
								{
									bool add = true;
									const PxU8 v0 = f.v[k], v1 = f.v[(k + 1) % 3];
									for (PxI32 l = numEdges - 1; l >= 0; --l)
									{
										const Edge& e = edges[l];
										if (v0 == e.v[1] && v1 == e.v[0])
										{
											add = false;
											edges[l] = edges[--numEdges];
											break;
										}
									}
									if (add)
										edges[numEdges++] = Edge::make(v0, v1);
								}
								mFaces[j] = mFaces[--mNumFaces];
							}
							else if (f.p == mNumPlanes - 1)
							{
								f.p = PxU8(i);
							}
						}

						mPlanes[i] = mPlanes[--mNumPlanes];
					}

					if (numEdges == 0)
						return false;

					if (mNumFaces > MAX_FACES - numEdges)
						return false;

					for (PxU32 i = 0; i < numEdges; ++i)
					{
						const Edge& e = edges[i];
						Face& f = mFaces[mNumFaces++];
						f = Face::make(findPlane(e.v[0], e.v[1], PxU8(mNumVerts - 1)), e.v[0], e.v[1], PxU8(mNumVerts - 1));
						if (f.p == 0xff)
							return false;
					}

					return true;
				}

				PX_CUDA_CALLABLE PX_FORCE_INLINE
				void computeClosest()
				{
					PX_ASSERT(mNumPlanes > 0);

					PxI32 closest = mNumPlanes - 1;
					PxF64 closestW = mPlanes[closest].plane.w;
					for (PxI32 i = closest - 1; i >= 0; --i)
					{
						if (mPlanes[i].plane.w > closestW)
						{
							closest = i;
							closestW = mPlanes[closest].plane.w;
						}
					}

					mClosest = mPlanes[closest];
				}

				PX_CUDA_CALLABLE PX_FORCE_INLINE
				PxF64 getDist() const
				{
					return mClosest.plane.w;
				}

				PX_CUDA_CALLABLE PX_FORCE_INLINE
				PxVec3d getDir() const
				{
					return mClosest.plane.getXYZ();
				}

				template <typename Support>
				PX_CUDA_CALLABLE PX_FORCE_INLINE
				void computePoints(const Convex<Support>& convexA, const Convex<Support>& convexB, PxVec3d& pointA, PxVec3d& pointB)
				{
					const Vert va = mVerts[mClosest.v[0]], vb = mVerts[mClosest.v[1]], vc = mVerts[mClosest.v[2]];
					const PxVec3d a = va.pos, b = vb.pos, c = vc.pos;
					const PxVec3d abc = (b - a).cross(c - a);
					const PxF64 iabc = 1.0 / abc.magnitudeSquared();
					const PxVec3d pabc = abc * abc.dot(a) * iabc;
					const PxF64 tbc = abc.dot((b - pabc).cross(c - pabc));
					const PxF64 tca = abc.dot((c - pabc).cross(a - pabc));
					const PxF64 tab = abc.dot((a - pabc).cross(b - pabc));
					const PxF64 sa = tbc * iabc, sb = tca * iabc, sc = tab * iabc;
					pointA = convexA.supportVertex(va.aI) * sa + convexA.supportVertex(vb.aI) * sb + convexA.supportVertex(vc.aI) * sc;
					pointB = convexB.supportVertex(va.bI) * sa + convexB.supportVertex(vb.bI) * sb + convexB.supportVertex(vc.bI) * sc;
				}

			private:

				static const PxU32 MAX_VERTS = 128;
				static const PxU32 MAX_FACES = 2 * MAX_VERTS - 4;

				struct Vert
				{
					PxVec3d pos;
					PxU8 aI, bI;

					PX_CUDA_CALLABLE PX_FORCE_INLINE
					static Vert make(const PxVec3d& pos, PxU8 aI, PxU8 bI)
						{ Vert v; v.pos = pos; v.aI = aI; v.bI = bI; return v; }
				};

				struct Plane
				{
					PxVec4d plane;
					PxU8 v[3];

					PX_CUDA_CALLABLE PX_FORCE_INLINE
					static Plane make(const PxVec4d& plane, PxU8 v0, PxU8 v1, PxU8 v2)
						{ Plane p; p.plane = plane; p.v[0] = v0; p.v[1] = v1; p.v[2] = v2; return p; }
				};

				struct Face
				{
					PxU8 p;
					PxU8 v[3];

					PX_CUDA_CALLABLE PX_FORCE_INLINE
					static Face make(PxU8 p, PxU8 v0, PxU8 v1, PxU8 v2)
						{ Face f; f.p = p; f.v[0] = v0; f.v[1] = v1; f.v[2] = v2; return f; }
				};

				Vert mVerts[MAX_VERTS];
				Face mFaces[MAX_FACES];
				Plane mPlanes[MAX_FACES];
				PxU32 mNumVerts, mNumFaces, mNumPlanes;
				Plane mClosest;
			};

			template <typename Support>
			PX_CUDA_CALLABLE PX_FORCE_INLINE
			static PxReal epaDepth(const Support& a, const Support& b, const PxTransform& poseA, const PxTransform& poseB, PxVec3& pointA, PxVec3& pointB, PxVec3& axis)
			{
				const PxF64 eps = PxF64(FLT_EPSILON);

				EpaDepth epa;
				Convex<Support> convexA(a, poseA), convexB(b, poseB);
				PxF64 epsDist = 0.001 * PxMin(convexA.getAccuracy(), convexB.getAccuracy());

				PxF64 closestDist = DBL_MAX;

				PxVec3d dir = V3_To_V3d(poseB.p - poseA.p);
				dir = dir.magnitude() > eps ? dir.getNormalized() : PxVec3d(1, 0, 0);

				const PxU8 aI0 = convexA.supportIndex(dir), bI0 = convexB.supportIndex(-dir);
				const PxVec3d aP0 = convexA.supportVertex(aI0), bP0 = convexB.supportVertex(bI0);
				const PxVec3d p0 = aP0 - bP0;
				epa.addPoint(p0, aI0, bI0);
				const PxU8 aI1 = convexA.supportIndex(-dir), bI1 = convexB.supportIndex(dir);
				const PxVec3d aP1 = convexA.supportVertex(aI1), bP1 = convexB.supportVertex(bI1);
				const PxVec3d p1 = aP1 - bP1;
				epa.addPoint(p1, aI1, bI1);
				const PxVec3d p0p1 = p1 - p0;
				dir = p0.cross(p0p1).cross(p0p1);
				if (dir.magnitude() < eps) dir = PxVec3d(0, 1, 0).cross(p0p1);
				if (dir.magnitude() < eps) dir = PxVec3d(0, 0, 1).cross(p0p1);
				dir = dir.getNormalized();

				while (true)
				{
					const PxU8 aI = convexA.supportIndex(dir), bI = convexB.supportIndex(-dir);
					const PxVec3d aP = convexA.supportVertex(aI), bP = convexB.supportVertex(bI);
					const PxVec3d p = aP - bP;
					const PxF64 dist = p.dot(-dir);

					if (dist >= closestDist - epsDist || !epa.addPoint(p, aI, bI))
					{
						PxVec3d pA, pB;
						epa.computePoints(convexA, convexB, pA, pB);
						pointA = V3d_To_V3(pA);
						pointB = V3d_To_V3(pB);
						axis = V3d_To_V3(-dir);
						return PxF32(dist);
					}

					epa.computeClosest();

					closestDist = epa.getDist();
					dir = epa.getDir();
				}

				return 0;
			}
		}
	}
}

#endif
