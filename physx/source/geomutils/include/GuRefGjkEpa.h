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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
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
		// The reference implementation of GJK-EPA algorithm
		namespace RefGjkEpa
		{
			template <typename Support>
			struct Convex
			{
				static const PxU32 MAX_VERTS = 32;

				PX_CUDA_CALLABLE
				Convex(const Support& s, const PxTransform& p)
					:
					mS(s), mPose(p), mNumVerts(0)
				{
					const PxVec3 X(1, 0, 0), Y(0, 1, 0), Z(0, 0, 1);
					const PxBounds3 bounds(PxVec3(mS.supportLocal(-X).x, mS.supportLocal(-Y).y, mS.supportLocal(-Z).z),
						PxVec3(mS.supportLocal(X).x, mS.supportLocal(Y).y, mS.supportLocal(Z).z));
					PxReal maxExtent = bounds.getDimensions().maxElement();
					mAccuracy = PxMax(maxExtent * 0.01f, FLT_EPSILON);
				}

				PX_CUDA_CALLABLE PX_INLINE
				PxReal getAccuracy() const
					{ return mAccuracy; }

				PX_CUDA_CALLABLE PX_INLINE
				PxBounds3 getBounds() const
				{
					const PxVec3 X(mPose.q.getInvBasisVector0()),
								 Y(mPose.q.getInvBasisVector1()),
								 Z(mPose.q.getInvBasisVector2());

					return PxBounds3(PxVec3(mPose.transform(mS.supportLocal(-X)).x,
											mPose.transform(mS.supportLocal(-Y)).y,
											mPose.transform(mS.supportLocal(-Z)).z),
									 PxVec3(mPose.transform(mS.supportLocal(X)).x,
											mPose.transform(mS.supportLocal(Y)).y,
											mPose.transform(mS.supportLocal(Z)).z));
				}

				PX_CUDA_CALLABLE
				PxU8 supportIndex(const PxVec3& dir)
				{
					PxVec3 d = dir;
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

				PX_CUDA_CALLABLE PX_INLINE
				const PxVec3& supportVertex(PxU8 index) const
					{ PX_ASSERT(index < mNumVerts); return mVerts[index]; }

				PX_CUDA_CALLABLE PX_INLINE
				const PxTransform& getPose() const
					{ return mPose; }

			private:

				const Support& mS;
				const PxTransform& mPose;
				PxVec3 mVerts[MAX_VERTS];
				PxU32 mNumVerts;
				PxF32 mAccuracy;
			};

			template <typename Support>
			struct GjkDistance
			{
				PX_CUDA_CALLABLE
				GjkDistance(Convex<Support>& convexA, Convex<Support>& convexB)
					:
					mConvexA(convexA), mConvexB(convexB),
					mNumVerts(0), mNumBestVerts(0), mIteration(0), mClosest(FLT_MAX)
				{
					mVerts[0] = mVerts[1] = mVerts[2] = mVerts[3] = Vert::make(PxVec3(0), 0, 0, 0);
					PxVec3 dir = mConvexB.getBounds().getCenter() - mConvexA.getBounds().getCenter();
					if (dir.normalize() < FLT_EPSILON) dir = PxVec3(1, 0, 0);
					addPoint(dir);
				}

				PX_CUDA_CALLABLE PX_INLINE
				PxReal getAccuracy() const
					{ return PxMin(mConvexA.getAccuracy(), mConvexB.getAccuracy()); }

				PX_CUDA_CALLABLE
				// adds a new point (the difference of the shapes support points in a given direction)
				// to the simplex. returns the projection of the difference of the support points on
				// the direction. positive values mean a gap, negative - an overlap. FLT_MAX signals
				// to finalize the algorithm. mIteration > MAX_ITERATIONS indicate that the algorithm
				// got stuck. most likely adding several same points over and over, but unable to
				// finish because of the floating point precision problems
				PxReal addPoint(const PxVec3& dir)
				{
					if (++mIteration > MAX_ITERATIONS)
						return FLT_MAX;

					const PxU8 aI = mConvexA.supportIndex(dir), bI = mConvexB.supportIndex(-dir);
					const PxVec3 p = mConvexA.supportVertex(aI) - mConvexB.supportVertex(bI);

					for (PxU32 i = 0; i < mNumVerts; ++i)
					{
						const Vert& v = mVerts[i];
						if (v.aI == aI && v.bI == bI)
							return FLT_MAX;
					}

					mVerts[mNumVerts++] = Vert::make(p, 0.0f, aI, bI);

					return p.dot(-dir);
				}

				PX_CUDA_CALLABLE
				void computeClosest()
				{
					Vert verts[4]; verts[0] = mVerts[0]; verts[1] = mVerts[1]; verts[2] = mVerts[2]; verts[3] = mVerts[3];
					verts[0].s = verts[1].s = verts[2].s = verts[3].s = 0;
					PxU32 numVerts = mNumVerts;

					switch (numVerts)
					{
					case 1:
					{
						verts[0].s = 1.0f;
						break;
					}
					case 2:
					{
						const PxVec3 a = verts[0].p, b = verts[1].p;
						const PxVec3 ba = a - b;
						const PxReal sba = ba.dot(-b);
						if (sba <= 0)
						{
							verts[0] = verts[1];
							verts[0].s = 1.0f;
							numVerts = 1;
							break;
						}
						verts[0].s = sba / ba.magnitudeSquared();
						verts[1].s = 1.0f - verts[0].s;
						break;
					}
					case 3:
					{
						const PxVec3 a = verts[0].p, b = verts[1].p, c = verts[2].p;
						const PxVec3 ca = a - c, cb = b - c;
						const PxReal sca = ca.dot(-c), scb = cb.dot(-c);
						if (sca <= 0 && scb <= 0)
						{
							verts[0] = verts[2];
							verts[0].s = 1.0f;
							numVerts = 1;
							break;
						}
						const PxVec3 abc = (b - a).cross(c - a);
						const PxReal iabc = 1.0f / abc.magnitudeSquared();
						const PxVec3 pabc = abc * abc.dot(a) * iabc;
						const PxReal tca = abc.dot((c - pabc).cross(a - pabc));
						if (sca > 0 && tca <= 0)
						{
							verts[1] = verts[2];
							verts[0].s = sca / ca.magnitudeSquared();
							verts[1].s = 1.0f - verts[0].s;
							numVerts = 2;
							break;
						}
						const PxReal tbc = abc.dot((b - pabc).cross(c - pabc));
						if (scb > 0 && tbc <= 0)
						{
							verts[0] = verts[1];
							verts[1] = verts[2];
							verts[0].s = scb / cb.magnitudeSquared();
							verts[1].s = 1.0f - verts[0].s;
							numVerts = 2;
							break;
						}
						verts[0].s = tbc * iabc;
						verts[1].s = tca * iabc;
						verts[2].s = 1.0f - verts[0].s - verts[1].s;
						if (abc.dot(a) > 0) PxSwap(verts[0], verts[1]);
						break;
					}
					case 4:
					{
						const PxVec3 a = verts[0].p, b = verts[1].p, c = verts[2].p, d = verts[3].p;
						const PxVec3 da = a - d, db = b - d, dc = c - d;
						const PxReal sda = da.dot(-d), sdb = db.dot(-d), sdc = dc.dot(-d);
						if (sda <= 0 && sdb <= 0 && sdc <= 0)
						{
							verts[0] = verts[3];
							verts[0].s = 1.0f;
							numVerts = 1;
							break;
						}
						const PxVec3 dab = (a - d).cross(b - d);
						const PxReal idab = 1.0f / dab.magnitudeSquared();
						const PxVec3 pdab = dab * dab.dot(d) * idab;
						const PxVec3 dbc = (b - d).cross(c - d);
						const PxReal idbc = 1.0f / dbc.magnitudeSquared();
						const PxVec3 pdbc = dbc * dbc.dot(d) * idbc;
						const PxVec3 dca = (c - d).cross(a - d);
						const PxReal idca = 1.0f / dca.magnitudeSquared();
						const PxVec3 pdca = dca * dca.dot(d) * idca;
						const PxReal tda = dab.dot((d - pdab).cross(a - pdab));
						const PxReal tad = dca.dot((a - pdca).cross(d - pdca));
						if (sda > 0 && tda <= 0 && tad <= 0)
						{
							verts[1] = verts[3];
							verts[0].s = sda / da.magnitudeSquared();
							verts[1].s = 1.0f - verts[0].s;
							numVerts = 2;
							break;
						}
						const PxReal tdb = dbc.dot((d - pdbc).cross(b - pdbc));
						const PxReal tbd = dab.dot((b - pdab).cross(d - pdab));
						if (sdb > 0 && tdb <= 0 && tbd <= 0)
						{
							verts[0] = verts[1];
							verts[1] = verts[3];
							verts[0].s = sdb / db.magnitudeSquared();
							verts[1].s = 1.0f - verts[0].s;
							numVerts = 2;
							break;
						}
						const PxReal tcd = dbc.dot((c - pdbc).cross(d - pdbc));
						const PxReal tdc = dca.dot((d - pdca).cross(c - pdca));
						if (sdc > 0 && tdc <= 0 && tcd <= 0)
						{
							verts[0] = verts[2];
							verts[1] = verts[3];
							verts[0].s = sdc / dc.magnitudeSquared();
							verts[1].s = 1.0f - verts[0].s;
							numVerts = 2;
							break;
						}
						const PxVec3 abc = (b - a).cross(c - a);
						if (tda > 0 && tbd > 0 && dab.dot(d) < 0 && abc.dot(dab) > 0)
						{
							verts[2] = verts[3];
							verts[0].s = tbd * idab;
							verts[1].s = tda * idab;
							verts[2].s = 1.0f - verts[0].s - verts[1].s;
							numVerts = 3;
							break;
						}
						if (tdb > 0 && tcd > 0 && dbc.dot(d) < 0 && abc.dot(dbc) > 0)
						{
							verts[0] = verts[1];
							verts[1] = verts[2];
							verts[2] = verts[3];
							verts[0].s = tcd * idbc;
							verts[1].s = tdb * idbc;
							verts[2].s = 1.0f - verts[0].s - verts[1].s;
							numVerts = 3;
							break;
						}
						if (tdc > 0 && tad > 0 && dca.dot(d) < 0 && abc.dot(dca) > 0)
						{
							verts[1] = verts[3];
							verts[0].s = tdc * idca;
							verts[2].s = tad * idca;
							verts[1].s = 1.0f - verts[0].s - verts[2].s;
							numVerts = 3;
							break;
						}
						break;
					}
					}

					PxVec3 closest = verts[0].p * verts[0].s + verts[1].p * verts[1].s + verts[2].p * verts[2].s + verts[3].p * verts[3].s;

					if (closest.magnitude() < mClosest.magnitude() - FLT_EPSILON)
					{
						mClosest = closest;
						mVerts[0] = verts[0]; mVerts[1] = verts[1]; mVerts[2] = verts[2]; mVerts[3] = verts[3];
						mNumVerts = numVerts;
					}
					else
						--mNumVerts;
				}

				PX_CUDA_CALLABLE PX_INLINE
				PxReal getDist() const
					{ return mClosest.magnitude(); }

				PX_CUDA_CALLABLE PX_INLINE
				PxVec3 getDir() const
					{ return -mClosest.getNormalized(); }

				PX_CUDA_CALLABLE
				void computePoints(PxVec3& pointA, PxVec3& pointB)
				{
					PxVec3 pA(0), pB(0);
					for (PxU32 i = 0; i < mNumVerts; ++i)
					{
						pA += mConvexA.supportVertex(mVerts[i].aI) * mVerts[i].s;
						pB += mConvexB.supportVertex(mVerts[i].bI) * mVerts[i].s;
					}
					pointA = pA; pointB = pB;
				}

			private:

				static const PxU32 MAX_ITERATIONS = 100;

				struct Vert
				{
					PxVec3 p;
					PxReal s;
					PxU8 aI, bI;

					PX_CUDA_CALLABLE PX_INLINE
					static Vert make(const PxVec3& p, PxReal s, PxU8 aI, PxU8 bI)
						{ Vert v; v.p = p; v.s = s; v.aI = aI; v.bI = bI; return v; }
				};

				Convex<Support>& mConvexA;
				Convex<Support>& mConvexB;
				PxU32 mNumVerts, mNumBestVerts, mIteration;
				Vert mVerts[4];
				PxVec3 mClosest;
			};

			template <typename Support>
			PX_CUDA_CALLABLE
			static PxReal computeGjkDistance(const Support& a, const Support& b, const PxTransform& poseA, const PxTransform& poseB,
				PxReal maxDist, PxVec3& pointA, PxVec3& pointB, PxVec3& axis)
			{
				Convex<Support> convexA(a, poseA), convexB(b, poseB);
				GjkDistance<Support> gjk(convexA, convexB);

				const PxReal distEps = 0.01f * gjk.getAccuracy();

				while (true)
				{
					gjk.computeClosest();

					const PxReal closestDist = gjk.getDist();
					if (closestDist < distEps)
						return 0;

					const PxVec3 dir = gjk.getDir();
					const PxReal proj = gjk.addPoint(dir);

					if (proj >= closestDist - distEps)
					{
						PxVec3 pA, pB;
						gjk.computePoints(pA, pB);
						pointA = pA; pointB = pB;
						axis = -dir;
						return closestDist;
					}

					if (proj > maxDist)
						return FLT_MAX;
				}
			}

			template <typename Support>
			struct EpaDepth
			{
				PX_CUDA_CALLABLE
				EpaDepth(Convex<Support>& convexA, Convex<Support>& convexB)
					:
					mConvexA(convexA), mConvexB(convexB),
					mNumVerts(0), mNumFaces(0),
					mClosest(Face::make(PxVec4(PxZero), 0, 0, 0)),
					mProj(-FLT_MAX)
				{
					PxVec3 dir0 = mConvexB.getBounds().getCenter() - mConvexA.getBounds().getCenter();
					if (dir0.normalize() < FLT_EPSILON) dir0 = PxVec3(1, 0, 0);
					PxVec3 dir1, dir2; PxComputeBasisVectors(dir0, dir1, dir2);
					addPoint(dir0); addPoint(-dir0); addPoint(dir1);
					if (mNumVerts < 3) addPoint(-dir1);
					if (mNumVerts < 3) addPoint(dir2);
					if (mNumVerts < 3) addPoint(-dir2);
					mProj = -FLT_MAX;
				}

				PX_CUDA_CALLABLE PX_INLINE
				bool isValid() const
					{ return mNumFaces > 0; }

				PX_CUDA_CALLABLE PX_INLINE
				PxReal getAccuracy() const
					{ return PxMin(mConvexA.getAccuracy(), mConvexB.getAccuracy()); }

				PX_CUDA_CALLABLE PX_INLINE
				PxVec3 supportVertex(PxU8 aI, PxU8 bI)
					{ return mConvexA.supportVertex(aI) - mConvexB.supportVertex(bI); }

				PX_CUDA_CALLABLE
				bool makePlane(PxU8 i0, PxU8 i1, PxU8 i2, PxVec4& plane)
				{
					const Vert v0 = mVerts[i0], v1 = mVerts[i1], v2 = mVerts[i2];
					const PxVec3 a = supportVertex(v0.aI, v0.bI), b = supportVertex(v1.aI, v1.bI), c = supportVertex(v2.aI, v2.bI);
					const PxVec3 abc = (b - a).cross(c - a) + (c - b).cross(a - b) + (a - c).cross(b - c);
					const PxReal labc = abc.magnitude();

					const PxReal normalEps = FLT_EPSILON;
					if (labc < normalEps)
						return false;

					const PxVec3 n = abc / labc;
					const PxReal d = n.dot(-a);

					plane = PxVec4(n, d);

					return true;
				}

				PX_CUDA_CALLABLE
				// adds a new point (the difference of the shapes support points in a given direction)
				// to the polytope. removes all faces below the new point, keeps the track of open edges
				// created by the face removal, and then creates a fan of new faces each containing the
				// new point and one of the open edges. returns the projection of the difference of the
				// support points on the direction. FLT_MAX signals to finalize the algorithm.
				PxReal addPoint(const PxVec3& dir)
				{
					if (mNumVerts > MAX_VERTS - 1)
						return FLT_MAX;

					const PxU8 aI = mConvexA.supportIndex(dir), bI = mConvexB.supportIndex(-dir);
					const PxVec3 p = mConvexA.supportVertex(aI) - mConvexB.supportVertex(bI);

					PxReal proj = p.dot(-dir);
					if (proj > mProj)
					{
						mProj = proj;
						mBest = mClosest;
					}

					for (PxU32 i = 0; i < mNumVerts; ++i)
						if (aI == mVerts[i].aI && bI == mVerts[i].bI)
							return FLT_MAX;

					Vert& v = mVerts[mNumVerts++];
					v = Vert::make(aI, bI);

					if (mNumVerts < 3)
						return 0;

					if (mNumVerts == 3)
					{
						PxVec4 plane0, plane1;
						if (!makePlane(0, 1, 2, plane0) || !makePlane(1, 0, 2, plane1))
							return FLT_MAX;

						mFaces[mNumFaces++] = Face::make(plane0, 0, 1, 2);
						mFaces[mNumFaces++] = Face::make(plane1, 1, 0, 2);

						return 0;
					}

					const PxReal testEps = 0.01f * getAccuracy();
					Edge edges[MAX_EDGES];
					PxU32 numEdges = 0;
					for (PxI32 i = mNumFaces - 1; i >= 0; --i)
					{
						Face& f = mFaces[i];
						if (f.plane.dot(PxVec4(p, 1)) > testEps)
						{
							for (PxU32 j = 0; j < 3; ++j)
							{
								bool add = true;
								const PxU8 v0 = f.v[j], v1 = f.v[(j + 1) % 3];
								for (PxI32 k = numEdges - 1; k >= 0; --k)
								{
									const Edge& e = edges[k];
									if (v0 == e.v[1] && v1 == e.v[0])
									{
										add = false;
										edges[k] = edges[--numEdges];
										break;
									}
								}
								if (add)
									edges[numEdges++] = Edge::make(v0, v1);
							}
							mFaces[i] = mFaces[--mNumFaces];
						}
					}

					if (numEdges == 0)
						return FLT_MAX;

					if (mNumFaces > MAX_FACES - numEdges)
						return FLT_MAX;

					for (PxU32 i = 0; i < numEdges; ++i)
					{
						const Edge& e = edges[i];
						PxU8 i0 = e.v[0], i1 = e.v[1], i2 = PxU8(mNumVerts - 1);

						PxVec4 plane;
						if (!makePlane(i0, i1, i2, plane))
							return FLT_MAX;

						mFaces[mNumFaces++] = Face::make(plane, i0, i1, i2);
					}

					return proj;
				}

				PX_CUDA_CALLABLE
				void computeClosest()
				{
					PX_ASSERT(mNumFaces > 0);

					PxU32 closest = PxU32(-1);
					PxReal closestW = -FLT_MAX;
					for (PxU32 i = 0; i < mNumFaces; ++i)
					{
						const PxReal w = mFaces[i].plane.w;
						if (w > closestW)
						{
							closest = i;
							closestW = w;
						}
					}

					PX_ASSERT(closest != PxU32(-1));

					mClosest = mFaces[closest];

					if (mNumFaces == 2)
						mBest = mClosest;
				}

				PX_CUDA_CALLABLE PX_INLINE
				PxReal getDist() const
					{ return mClosest.plane.w; }

				PX_CUDA_CALLABLE PX_INLINE
				PxVec3 getDir() const
					{ return mClosest.plane.getXYZ(); }

				PX_CUDA_CALLABLE PX_INLINE
				PxReal getBestDist() const
					{ return mBest.plane.w; }

				PX_CUDA_CALLABLE PX_INLINE
				PxVec3 getBestDir() const
					{ return mBest.plane.getXYZ(); }

				PX_CUDA_CALLABLE
				void computePoints(PxVec3& pointA, PxVec3& pointB)
				{
					const Vert v0 = mVerts[mBest.v[0]], v1 = mVerts[mBest.v[1]], v2 = mVerts[mBest.v[2]];
					const PxVec3 va0 = mConvexA.supportVertex(v0.aI), va1 = mConvexA.supportVertex(v1.aI), va2 = mConvexA.supportVertex(v2.aI);
					const PxVec3 vb0 = mConvexB.supportVertex(v0.bI), vb1 = mConvexB.supportVertex(v1.bI), vb2 = mConvexB.supportVertex(v2.bI);
					const PxVec3 a = va0 - vb0, b = va1 - vb1, c = va2 - vb2;
					const PxVec3 abc = (b - a).cross(c - a);
					const PxReal iabc = 1.0f / abc.magnitudeSquared();
					const PxVec3 pabc = abc * abc.dot(a) * iabc;
					const PxReal tbc = abc.dot((b - pabc).cross(c - pabc));
					const PxReal tca = abc.dot((c - pabc).cross(a - pabc));
					const PxReal tab = abc.dot((a - pabc).cross(b - pabc));
					const PxReal sa = tbc * iabc, sb = tca * iabc, sc = tab * iabc;
					pointA = va0 * sa + va1 * sb + va2 * sc;
					pointB = vb0 * sa + vb1 * sb + vb2 * sc;
				}

			private:

				static const PxU32 MAX_VERTS = 32;
				static const PxU32 MAX_FACES = 2 * MAX_VERTS - 4;
				static const PxU32 MAX_EDGES = MAX_VERTS + MAX_FACES - 2;

				struct Vert
				{
					PxU8 aI, bI;

					PX_CUDA_CALLABLE PX_INLINE
					static Vert make(PxU8 aI, PxU8 bI)
						{ Vert v; v.aI = aI; v.bI = bI; return v; }
				};

				struct Face
				{
					PxVec4 plane;
					PxU8 v[3];

					PX_CUDA_CALLABLE PX_INLINE
					static Face make(const PxVec4& plane, PxU8 v0, PxU8 v1, PxU8 v2)
						{ Face f; f.plane = plane; f.v[0] = v0; f.v[1] = v1; f.v[2] = v2; return f; }
				};

				struct Edge
				{
					PxU8 v[2];

					PX_CUDA_CALLABLE PX_INLINE
					static Edge make(PxU8 v0, PxU8 v1)
						{ Edge e; e.v[0] = v0; e.v[1] = v1; return e; }
				};

				Convex<Support>& mConvexA;
				Convex<Support>& mConvexB;
				Vert mVerts[MAX_VERTS];
				Face mFaces[MAX_FACES];
				PxU32 mNumVerts, mNumFaces;
				Face mClosest, mBest;
				PxReal mProj;
			};

			template <typename Support>
			PX_CUDA_CALLABLE
			static PxReal computeEpaDepth(const Support& a, const Support& b, const PxTransform& poseA, const PxTransform& poseB,
				PxVec3& pointA, PxVec3& pointB, PxVec3& axis)
			{
				Convex<Support> convexA(a, poseA), convexB(b, poseB);
				EpaDepth<Support> epa(convexA, convexB);
				if (!epa.isValid())
					return FLT_MAX;

				const PxReal distEps = 0.01f * epa.getAccuracy();

				while (true)
				{
					epa.computeClosest();

					const PxReal closestDist = epa.getDist();
					const PxVec3 dir = epa.getDir();

					const PxReal proj = epa.addPoint(dir);

					if (proj >= closestDist - distEps)
					{
						PxVec3 pA, pB;
						epa.computePoints(pA, pB);
						pointA = pA; pointB = pB;
						axis = -epa.getBestDir();
						return epa.getBestDist();
					}
				}
			}
		}
	}
}

#endif
