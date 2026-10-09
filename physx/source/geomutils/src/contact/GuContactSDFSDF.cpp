// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// SDF-vs-SDF contact generation by temporal caching and importance sampling, after
// Giles & Andrews, "Real-Time Collision Handling for Signed Distance Fields via Caching and
// Importance Sampling", CGF 45(7) 2026. Section and equation numbers below refer to the paper.
// See GuContactSDFSDF.h for an overview.

#include "GuContactSDFSDF.h"

#include "foundation/PxBounds3.h"
#include "foundation/PxMat33.h"
#include "foundation/PxMath.h"
#include "foundation/PxTransform.h"
#include "foundation/PxUnionCast.h"
#include "foundation/PxAtomic.h"
#include "foundation/PxTime.h"
#include "foundation/PxVec3.h"
#include "foundation/PxVecMath.h"
#include "geometry/PxBoxGeometry.h"
#include "geometry/PxMeshQuery.h"
#include "geometry/PxTriangleMeshGeometry.h"
#include "geomutils/PxContactBuffer.h"
#include "GuCollisionSDF.h"
#include "GuContactMeshMesh.h"
#include "GuContactMethodImpl.h"
#include "GuDistancePointTriangle.h"
#include "GuPersistentContactManifold.h"
#include "GuTriangleMesh.h"

using namespace physx;
using namespace Gu;
using namespace aos;

SDFSDFParams Gu::gSDFSDFParams;
SDFSDFStats Gu::gSDFSDFStats = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// The argument of SDF_STAT_ADD is always evaluated (it may be a call whose side effects matter,
// such as the Nelder-Mead iteration count); only the accumulation is compiled out in release.
#if GU_SDF_SDF_STATS
#define SDF_STAT_ADD(field, n) PxAtomicAdd(reinterpret_cast<volatile PxI32*>(&gSDFSDFStats.field), PxI32(n))
#else
#define SDF_STAT_ADD(field, n) ((void)(n))
#endif
#define SDF_STAT(field) SDF_STAT_ADD(field, 1)

namespace
{

static const PxU32 MAX_CACHED = GU_MAX_MANIFOLD_SIZE * GU_SINGLE_MANIFOLD_CACHE_SIZE;	// cache capacity of the multi-manifold (36)
static const PxU32 MAX_POINTS = MAX_CACHED + 8;	// cached + stochastic points of one step, before duplicate removal
static const PxU32 MAX_SEED_TRIANGLES = 64;		// per mesh, triangles inside V that vertex seeds are drawn from
static const PxU32 MAX_POLISH_TRIANGLES = 12;	// triangles around a converged point that are refined against the other SDF

// xorshift32. Seeded per pair and step from the relative pose, so contact generation stays
// deterministic regardless of thread scheduling.
struct Rng
{
	PxU32 s;
	explicit Rng(PxU32 seed) : s(seed ? seed : 0x9E3779B9u) {}
	PX_FORCE_INLINE PxU32 next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
	PX_FORCE_INLINE PxReal unit() { return PxReal(next() >> 8) * (1.0f / 16777216.0f); }	// [0, 1)
	PX_FORCE_INLINE PxU32 below(PxU32 n) { return next() % n; }
};

PX_FORCE_INLINE PxU32 hashFloat(PxU32 h, PxReal v)
{
	// FNV-1a style mix of the float's bits
	h ^= PxUnionCast<PxU32, PxF32>(v);
	return h * 16777619u;
}

// A rigid body carrying an SDF, with the transforms needed to evaluate its field at world points.
// The SDF lives in the mesh's "vertex space"; shape space adds PxMeshScale, world space adds the pose.
struct SDFBody
{
	const TriangleMesh&				mesh;
	const PxTriangleMeshGeometry&	geom;
	const PxTransform				pose;
	const CollisionSDF				sdf;
	PxMat33							vertexToWorld;		// R * S
	PxMat33							worldToVertex;		// S^-1 * R^T
	PxMat33							normalToWorld;		// R * S^-1 (inverse transpose of vertexToWorld)
	PxVec3							worldToVertexT;
	PxReal							distScale;			// vertex-space distances -> world (exact for uniform scale)
	PxReal							spacingWorld;		// SDF cell size in world units
	PxBounds3						worldBounds;		// world AABB of the SDF box

	bool							hasSdf;

	SDFBody(const PxTriangleMeshGeometry& g, const PxTransform& p) :
		mesh(static_cast<const TriangleMesh&>(*g.triangleMesh)), geom(g), pose(p), sdf(mesh.getSdfDataFast()), hasSdf(mesh.getSdfDataFast().mSdf != NULL)
	{
		const PxMat33Padded R(pose.q);
		const PxMat33 S = geom.scale.toMat33();
		const PxMat33 SInv = geom.scale.getInverse().toMat33();
		vertexToWorld = R * S;
		worldToVertex = SInv * R.getTranspose();
		worldToVertexT = -(worldToVertex * pose.p);
		normalToWorld = R * SInv;
		const PxVec3& s = geom.scale.scale;
		distScale = PxPow(PxAbs(s.x * s.y * s.z), 1.0f / 3.0f);
		spacingWorld = sdf.mSdf.mSpacing * distScale;
		worldBounds = PxBounds3::transformFast(vertexToWorld, hasSdf ? PxBounds3(sdf.mSdfBoxLower, sdf.mSdfBoxUpper) : PxBounds3::centerExtents(mesh.getLocalBoundsFast().mCenter, mesh.getLocalBoundsFast().mExtents));
		worldBounds.minimum += pose.p;
		worldBounds.maximum += pose.p;
	}

	PX_FORCE_INLINE PxVec3 toVertex(const PxVec3& xW) const { return worldToVertex * xW + worldToVertexT; }
	PX_FORCE_INLINE PxVec3 toWorld(const PxVec3& xV) const { return vertexToWorld * xV + pose.p; }

	// Signed distance at a world point, in world units
	PX_FORCE_INLINE PxReal phi(const PxVec3& xW) const
	{
		SDF_STAT(fieldEvals);
		return sdf.dist(toVertex(xW)) * distScale;
	}

	// Signed distance and unit outward normal (world) at a world point
	PX_FORCE_INLINE PxReal phi(const PxVec3& xW, PxVec3& nW) const
	{
		SDF_STAT(fieldEvals);
		PxVec3 g;
		const PxReal d = sdf.dist(toVertex(xW), &g) * distScale;
		nW = normalToWorld * g;
		const PxReal m2 = nW.magnitudeSquared();
		if (m2 > 1e-20f)
			nW *= PxRecipSqrt(m2);
		else
			nW = PxVec3(0.0f, 1.0f, 0.0f);	// at a critical point of the field; any direction is as good as another
		return d;
	}

	PX_FORCE_INLINE PxVec3 vertexWorld(PxU32 index) const
	{
		return toWorld(mesh.getVerticesFast()[index]);
	}

	PX_FORCE_INLINE IndexedTriangle32 triangle(PxU32 triIndex) const
	{
		return (mesh.getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES) ?
			getTriangleVertexIndices<PxU16>(mesh.getTrianglesFast(), triIndex) :
			getTriangleVertexIndices<PxU32>(mesh.getTrianglesFast(), triIndex);
	}

private:
	SDFBody& operator=(const SDFBody&);
};

// Eq. 2: smooth upper bound of max(phiA, phiB), the hardmax when epsilon is zero
PX_FORCE_INLINE PxReal softmax(PxReal phiA, PxReal phiB, PxReal epsilon)
{
	return 0.5f * (phiA + phiB + PxSqrt((phiA - phiB) * (phiA - phiB) + epsilon));
}

// Eq. 3: the collision objective, smallest deep inside the intersection of the two bodies, minus a
// repulsion from the points already in the manifold so that new seeds explore other regions.
struct Objective
{
	const SDFBody&	a;
	const SDFBody&	b;
	const PxVec3*	repel;
	PxU32			nbRepel;
	PxReal			epsilon;
	PxReal			alpha;

	PX_FORCE_INLINE PxReal operator()(const PxVec3& x) const
	{
		PxReal v = softmax(a.phi(x), b.phi(x), epsilon);
		for (PxU32 i = 0; i < nbRepel; ++i)
			v -= alpha * (x - repel[i]).magnitude();
		return v;
	}

private:
	Objective& operator=(const Objective&);
};

PX_FORCE_INLINE PxVec3 clampToBounds(const PxVec3& p, const PxBounds3& box)
{
	return p.maximum(box.minimum).minimum(box.maximum);
}

// Derivative-free Nelder-Mead simplex search [NM65] over the search domain V (Sec. 3.3), with the
// standard coefficients (reflection 1, expansion 2, contraction 1/2, shrink 1/2). Trial points are
// projected onto V. Terminates once the simplex lies within `tolerance` of its best vertex, or
// when the objective is flat over it. Returns the number of iterations.
static PxU32 nelderMead(const Objective& g, PxVec3& x, const PxBounds3& domain, const PxVec3& initialStep, PxReal tolerance, PxU32 maxIterations)
{
	x = clampToBounds(x, domain);
	PxVec3 v[4] = { x, x, x, x };
	PxReal f[4];
	for (PxU32 i = 0; i < 3; ++i)
	{
		// initial simplex along the axes, pointing into V
		v[i + 1][i] += (x[i] + initialStep[i] <= domain.maximum[i]) ? initialStep[i] : -initialStep[i];
		v[i + 1] = clampToBounds(v[i + 1], domain);
	}
	for (PxU32 i = 0; i < 4; ++i)
		f[i] = g(v[i]);

	PxU32 it = 0;
	for (; it < maxIterations; ++it)
	{
		// order best to worst
		for (PxU32 i = 1; i < 4; ++i)
			for (PxU32 j = i; j > 0 && f[j] < f[j - 1]; --j)
			{
				PxSwap(f[j], f[j - 1]);
				PxSwap(v[j], v[j - 1]);
			}

		const PxReal extent = PxMax((v[1] - v[0]).magnitude(), PxMax((v[2] - v[0]).magnitude(), (v[3] - v[0]).magnitude()));
		if (extent < tolerance || f[3] - f[0] < 0.05f * tolerance)
			break;

		const PxVec3 centroid = (v[0] + v[1] + v[2]) * (1.0f / 3.0f);
		const PxVec3 reflected = clampToBounds(centroid + (centroid - v[3]), domain);
		const PxReal fr = g(reflected);
		if (fr < f[0])
		{
			const PxVec3 expanded = clampToBounds(centroid + (centroid - v[3]) * 2.0f, domain);
			const PxReal fe = g(expanded);
			v[3] = fe < fr ? expanded : reflected;
			f[3] = PxMin(fe, fr);
		}
		else if (fr < f[2])
		{
			v[3] = reflected;
			f[3] = fr;
		}
		else
		{
			// contract towards the better of the reflected and worst points, shrink if that fails
			const bool outside = fr < f[3];
			const PxVec3 contracted = centroid + ((outside ? reflected : v[3]) - centroid) * 0.5f;
			const PxReal fc = g(contracted);
			if (fc < PxMin(fr, f[3]))
			{
				v[3] = contracted;
				f[3] = fc;
			}
			else
			{
				for (PxU32 i = 1; i < 4; ++i)
				{
					v[i] = v[0] + (v[i] - v[0]) * 0.5f;
					f[i] = g(v[i]);
				}
			}
		}
	}

	PxU32 best = 0;
	for (PxU32 i = 1; i < 4; ++i)
		if (f[i] < f[best])
			best = i;
	x = v[best];
	return it;
}

// One point of the contact manifold: {x*, d, n} of Algorithm 1 plus bookkeeping
struct ManifoldPoint
{
	PxVec3	x;				// minimizer (world)
	PxVec3	normal;			// unit normal, from body 0 towards body 1 (Eq. 5)
	PxReal	separation;		// overlap (< 0) or gap (> 0) of the two surfaces at x
	PxReal	softmax;		// Eq. 2 at x
	PxReal	rank;			// Eq. 3 against the final manifold, for sorting
	PxU32	faceIndex;		// triangle used by the polish step, or 0xFFFFFFFF
	bool	active;			// classified as a contact (Sec. 3.4)
	bool	cached;			// warm started from the cache
};

struct PairContext
{
	const SDFBody&		body0;
	const SDFBody&		body1;
	const SDFSDFParams&	P;
	PxBounds3			V;
	PxReal				margin, tau, epsilon, alpha, replacement;
	ManifoldPoint		points[MAX_POINTS];
	PxVec3				repel[MAX_POINTS];
	PxU32				nbPoints;

	bool				single;			// only one body has an SDF: `field` is that body, `surf` the other
	bool				fieldIs0;
	PxReal				L;

	PairContext(const SDFBody& b0, const SDFBody& b1, const SDFSDFParams& p) : body0(b0), body1(b1), P(p), nbPoints(0),
		single(!(b0.hasSdf && b1.hasSdf)), fieldIs0(b0.hasSdf), L(1.0f) {}
	PX_FORCE_INLINE const SDFBody& fieldBody() const { return fieldIs0 ? body0 : body1; }
	PX_FORCE_INLINE const SDFBody& surfBody() const { return fieldIs0 ? body1 : body0; }

	// Sec. 3.4 / Eq. 4-5: minimize g from `seed`, then derive the contact data at the minimizer.
	// `prevNormal` is the cached point's normal from the previous step (only used when `cached`).
	void addPoint(const PxVec3& seed, const PxVec3& step, bool cached, const PxVec3& prevNormal)
	{
		if (nbPoints == MAX_POINTS)
			return;

		if (single)
		{
			// One SDF: the seed is a point on (or near) the other mesh's surface; the surface descent
			// in the polish step does the optimizing. Classify and store it as is.
			PxVec3 n;
			const PxReal phi = fieldBody().phi(clampToBounds(seed, V), n);
			ManifoldPoint& pt = points[nbPoints];
			pt.x = clampToBounds(seed, V);
			pt.separation = phi;
			pt.softmax = phi;
			pt.normal = fieldIs0 ? n : -n;	// from body 0 towards body 1
			pt.faceIndex = 0xFFFFFFFF;
			pt.cached = cached;
			pt.active = phi < margin;
			if (pt.active) SDF_STAT(activePoints);
			repel[nbPoints] = pt.x;
			++nbPoints;
			return;
		}

		const Objective g = { body0, body1, repel, nbPoints, epsilon, alpha };
		PxVec3 x = seed;
		SDF_STAT_ADD(iterations, nelderMead(g, x, V, step, tau, P.maxIterations));
		if (cached && P.stickiness > 0.0f)
		{
			// On a flat contact the objective is flat and the simplex would random-walk the point by
			// up to its initial size every step, resetting friction anchors and jittering the
			// stack. Keep the advected position unless the re-optimized one is better by a fraction
			// of the replacement threshold: small enough that the repulsion can still spread a
			// cluster over a face as a body settles, large enough to damp the walk.
			const PxVec3 clampedSeed = clampToBounds(seed, V);
			if (g(x) > g(clampedSeed) - P.stickiness * replacement)
				x = clampedSeed;
		}

		PxVec3 nA, nB;
		const PxReal phiA = body0.phi(x, nA);
		const PxReal phiB = body1.phi(x, nB);
		nB = -nB;	// so that both candidate normals point from body 0 towards body 1

		ManifoldPoint& pt = points[nbPoints];
		pt.x = x;
		pt.separation = phiA + phiB;
		pt.softmax = softmax(phiA, phiB, epsilon);
		pt.faceIndex = 0xFFFFFFFF;
		pt.cached = cached;

		// Eq. 5: normal of the more deeply penetrating field. The optimizer converges where the two
		// fields are about equal, so either can win; where the surfaces face the same way (a thin
		// spike inside the other body) nA and nB are opposite and the choice would flip from step to
		// step. Cached points therefore keep whichever is closer to their previous normal.
		if (cached)
			pt.normal = nA.dot(prevNormal) > nB.dot(prevNormal) ? nA : nB;
		else
			pt.normal = phiA < phiB ? nA : nB;

		// Outside at least one body the separation is the gap between the closest surface points
		// (one Newton step along each field). Outside both the normal follows that gap, unless it
		// would reverse a cached normal.
		if (phiA > 0.0f || phiB > 0.0f)
		{
			const PxVec3 gap = nA * phiA + nB * phiB;	// closest point on B minus closest point on A
			pt.separation = gap.magnitude();
			if (phiA > 0.0f && phiB > 0.0f && pt.separation > 1e-6f && !(cached && gap.dot(prevNormal) < 0.0f))
				pt.normal = gap / pt.separation;
		}

		pt.active = pt.separation < margin;
		if (pt.active) SDF_STAT(activePoints);
		repel[nbPoints] = x;
		++nbPoints;
	}
};

// Triangles of a mesh inside the search domain, the pool that stochastic seeds are drawn from
struct SeedTriangles
{
	PxU32 indices[MAX_SEED_TRIANGLES];
	PxU32 count;

	// The overlap query returns the first MAX_SEED_TRIANGLES triangles in traversal order, which on
	// dense meshes biases seeds to one part of V. Once a cache exists, query a random sub-box of V
	// each step instead, so that successive steps cover all of it; fall back to the whole box when
	// the sub-box is empty.
	void gather(const SDFBody& body, const PxBounds3& V, Rng* rng)
	{
		bool overflow = false;
		if (rng)
		{
			const PxVec3 size = V.maximum - V.minimum;
			const PxVec3 half = size * 0.3f;
			const PxVec3 center = V.minimum + half + PxVec3(rng->unit(), rng->unit(), rng->unit()).multiply(size - half * 2.0f);
			count = PxMeshQuery::findOverlapTriangleMesh(PxBoxGeometry(half.maximum(PxVec3(1e-4f))), PxTransform(center), body.geom, body.pose, indices, MAX_SEED_TRIANGLES, 0, overflow);
			if (count)
				return;
		}
		const PxBoxGeometry box(V.getExtents());
		count = PxMeshQuery::findOverlapTriangleMesh(box, PxTransform(V.getCenter()), body.geom, body.pose, indices, MAX_SEED_TRIANGLES, 0, overflow);
	}
};

// STOCHASTICSAMPLE (Sec. 3.5.2). The paper draws from precomputed high-curvature surface points.
// We have the meshes, whose sharp features are vertices, so draw from the vertices (or a random
// surface point) of triangles inside V, and keep the best of a few candidates under the objective.
static PxVec3 stochasticSample(const PairContext& ctx, const SeedTriangles* tris, Rng& rng)
{
	const PxBounds3& V = ctx.V;
	PxVec3 best = V.minimum + PxVec3(rng.unit(), rng.unit(), rng.unit()).multiply(V.maximum - V.minimum);
	if (!ctx.P.curvatureSeeds || (tris[0].count + tris[1].count) == 0)
		return best;

	PxReal bestValue = PX_MAX_F32;
	const PxU32 total = tris[0].count + tris[1].count;

	if (ctx.single)
	{
		// One SDF: a random surface point of the plain mesh almost never lands within the contact
		// offset of the SDF body's first touching feature (a bunny's toes on a 20 m plane). Instead
		// take vertices of the SDF body inside V and project them onto the plain mesh's triangles
		// inside V; the deepest projection (vertex below the oriented surface) seeds the search,
		// which is what the per-triangle method finds by optimizing the whole triangle.
		const PxU32 fieldIdx = ctx.fieldIs0 ? 0 : 1, surfIdx = 1 - fieldIdx;
		const SDFBody& field = ctx.fieldBody();
		const SDFBody& surf = ctx.surfBody();
		if (tris[fieldIdx].count && tris[surfIdx].count)
		{
			const PxVec3 fieldCenter = field.worldBounds.getCenter();
			for (PxU32 c = 0; c < ctx.P.seedCandidates; ++c)
			{
				// Alternate two candidate kinds: a vertex of the SDF body (finds the feature that
				// touches first), and a uniform point of V (covers regions whose vertices the capped
				// triangle query did not return, e.g. a belly already pressed into the plane).
				// Both are projected onto the plain mesh and scored by depth below its surface.
				PxVec3 v;
				if (c & 1)
					v = V.minimum + PxVec3(rng.unit(), rng.unit(), rng.unit()).multiply(V.maximum - V.minimum);
				else
				{
					const IndexedTriangle32 tri = field.triangle(tris[fieldIdx].indices[rng.below(tris[fieldIdx].count)]);
					v = clampToBounds(field.vertexWorld(tri.mRef[rng.below(3)]), V);
				}
				// closest point on the plain triangles inside V, and the depth of v below them
				PxReal bestD2 = PX_MAX_F32;
				PxVec3 cp(0.0f), n(0.0f);
				for (PxU32 t = 0; t < tris[surfIdx].count; ++t)
				{
					const IndexedTriangle32 st = surf.triangle(tris[surfIdx].indices[t]);
					const PxVec3 s0 = surf.vertexWorld(st.mRef[0]), s1 = surf.vertexWorld(st.mRef[1]), s2 = surf.vertexWorld(st.mRef[2]);
					const PxVec3 q = closestPtPointTriangle2(v, s0, s1, s2, s1 - s0, s2 - s0);
					const PxReal d2 = (q - v).magnitudeSquared();
					if (d2 < bestD2)
					{
						bestD2 = d2;
						cp = q;
						n = (s1 - s0).cross(s2 - s0);
					}
				}
				const PxReal n2 = n.magnitudeSquared();
				if (n2 < 1e-20f)
					continue;
				n *= PxRecipSqrt(n2);
				if (n.dot(fieldCenter - cp) < 0.0f)
					n = -n;	// outward side of the plain mesh is where the SDF body sits
				// depth below the plain surface: the vertex's signed height for a vertex candidate,
				// the SDF body's field at the projection for a uniform candidate
				PxReal value = (c & 1) ? field.phi(cp) : (v - cp).dot(n);
				for (PxU32 r = 0; r < ctx.nbPoints; ++r)
					value -= ctx.alpha * (cp - ctx.repel[r]).magnitude();
				if (value < bestValue)
				{
					bestValue = value;
					best = clampToBounds(cp, V);
				}
			}
			if (bestValue < PX_MAX_F32)
				return best;
		}
		if (!tris[surfIdx].count)
			return best;
	}

	for (PxU32 c = 0; c < ctx.P.seedCandidates; ++c)
	{
		PxU32 pick, which;
		if (ctx.single)
		{
			which = ctx.fieldIs0 ? 1 : 0;
			pick = rng.below(tris[which].count);
		}
		else
		{
			pick = rng.below(total);
			which = pick < tris[0].count ? 0 : 1;
			if (which)
				pick -= tris[0].count;
		}
		const SDFBody& body = which ? ctx.body1 : ctx.body0;
		const IndexedTriangle32 tri = body.triangle(tris[which].indices[pick]);
		const PxVec3 v[3] = { body.vertexWorld(tri.mRef[0]), body.vertexWorld(tri.mRef[1]), body.vertexWorld(tri.mRef[2]) };

		// Half the candidates are vertices (the sharp features). The other half are the closest
		// point on the triangle to a uniform point of V: on a large triangle (a ground plane) a
		// uniformly chosen surface point almost never falls inside V, and clamping it would park
		// every seed on V's boundary.
		PxVec3 p;
		if ((rng.next() & 1) || !ctx.P.polishWithTriangles)
			p = v[rng.below(3)];
		else
		{
			const PxVec3 q = V.minimum + PxVec3(rng.unit(), rng.unit(), rng.unit()).multiply(V.maximum - V.minimum);
			p = closestPtPointTriangle2(q, v[0], v[1], v[2], v[1] - v[0], v[2] - v[0]);
		}
		p = clampToBounds(p, V);
		// A point on the surface of one body scores ~0 under the objective whatever its depth in the
		// other, so rank candidates by that depth (the deeper of the two fields), with the repulsion
		// of Eq. 3 so that new seeds explore away from the points already found
		PxReal value = ctx.single ? ctx.fieldBody().phi(p) : PxMin(ctx.body0.phi(p), ctx.body1.phi(p));
		for (PxU32 r = 0; r < ctx.nbPoints; ++r)
			value -= ctx.alpha * (p - ctx.repel[r]).magnitude();
		if (value < bestValue)
		{
			bestValue = value;
			best = p;
		}
	}
	return best;
}

// A converged point refined against the actual mesh surface of one body ("surface") and the field
// of the other ("field").
struct Polished
{
	PxVec3	point;			// on the surface body's mesh (world)
	PxVec3	normal;			// unit gradient of the field body at `point` (out of the field body)
	PxReal	separation;		// signed distance of `point` to the field body (world units)
	PxReal	facing;			// -(outward surface normal . field gradient): 1 when the surfaces face each other
	PxU32	faceIndex;		// triangle of the surface body
};

// Snap x to the closest point on the triangles of `surf` around it and evaluate the field of `field`
// there. Running the per-triangle optimizer over whole triangles instead would pull every point of
// a flat contact to the same deepest corner, so the manifold would lose its spread.
static bool polishPairing(const PairContext& ctx, const SDFBody& surf, const SDFBody& field, const PxVec3& x, Polished& out, bool cached)
{
	// x lies about |phi_surf(x)| from the surface of `surf`; search a box that is sure to contain it.
	// Without an SDF on the surface body (single-SDF pairs) the seeds are surface points already.
	const PxReal radius = surf.hasSdf ? PxMax(2.0f * ctx.tau, 1.5f * PxAbs(surf.phi(x)) + ctx.tau) : PxMax(4.0f * ctx.tau, 2.0f * PxAbs(field.phi(x)));
	PxU32 tris[MAX_POLISH_TRIANGLES];
	bool overflow = false;
	const PxU32 nb = PxMeshQuery::findOverlapTriangleMesh(PxBoxGeometry(PxVec3(radius)), PxTransform(x), surf.geom, surf.pose, tris, MAX_POLISH_TRIANGLES, 0, overflow);
	if (!nb)
		return false;

	PxReal bestDistSq = PX_MAX_F32;
	PxVec3 best(0.0f), triNormal(0.0f);
	for (PxU32 i = 0; i < nb; ++i)
	{
		const IndexedTriangle32 tri = surf.triangle(tris[i]);
		const PxVec3 v0 = surf.vertexWorld(tri.mRef[0]), v1 = surf.vertexWorld(tri.mRef[1]), v2 = surf.vertexWorld(tri.mRef[2]);
		const PxVec3 cp = closestPtPointTriangle2(x, v0, v1, v2, v1 - v0, v2 - v0);
		const PxReal d2 = (cp - x).magnitudeSquared();
		if (d2 < bestDistSq)
		{
			bestDistSq = d2;
			best = cp;
			triNormal = (v1 - v0).cross(v2 - v0);
			out.faceIndex = tris[i];
		}
	}

	out.point = best;
	out.separation = field.phi(best, out.normal);

	// Trust-region descent over the nearby surface (7b/7d of the plan): from the snapped point,
	// move along the triangles around it towards deeper penetration of the field, with the
	// repulsion of Eq. 3 so that the manifold keeps its spread, and never farther from x than a
	// radius that grows with the penetration. A flat resting contact (penetration ~ margin) moves
	// only a few tau; an ear tip 0.1 deep in the ground is reached from the ear's side.
	if (ctx.P.surfaceDescent && nb > 0)
	{
		// Single-SDF pairs have no Nelder-Mead stage: a fresh seed may travel far to find the deep
		// spot (like a random simplex spanning 10% of V), a cached point only tracks the contact it
		// already sits on (like a cached point's small simplex), otherwise every cached point of a
		// rocking body would jump to the single deepest spot and the support would collapse.
		const bool farSearch = ctx.single && !cached;
		const PxReal R = 2.0f * PxAbs(out.separation) + 2.0f * ctx.tau + (farSearch ? 0.25f * ctx.L : 0.0f);
		const PxReal RSq = R * R;
		PxReal bestObj = out.separation;
		for (PxU32 r = 0; r < ctx.nbPoints; ++r)
			bestObj -= ctx.alpha * (best - ctx.repel[r]).magnitude();
		// the descent must beat the snapped point by the replacement threshold, otherwise points
		// would creep along slightly tilted faces every step. In single-SDF mode a cached point is
		// gated like a cached Nelder-Mead point and a fresh seed accepts any improvement.
		bestObj -= ctx.single ? (cached ? ctx.P.stickiness * ctx.replacement : 0.0f) : ctx.replacement;
		PxVec3 bestPt = best, bestNormal = out.normal;
		PxReal bestSep = out.separation;
		PxU32 bestFace = out.faceIndex;

		// the three triangles whose closest points are nearest to x
		PxU32 order[MAX_POLISH_TRIANGLES];
		PxReal dist[MAX_POLISH_TRIANGLES];
		for (PxU32 i = 0; i < nb; ++i)
		{
			const IndexedTriangle32 tri = surf.triangle(tris[i]);
			const PxVec3 v0 = surf.vertexWorld(tri.mRef[0]), v1 = surf.vertexWorld(tri.mRef[1]), v2 = surf.vertexWorld(tri.mRef[2]);
			dist[i] = (closestPtPointTriangle2(x, v0, v1, v2, v1 - v0, v2 - v0) - x).magnitudeSquared();
			order[i] = i;
		}
		for (PxU32 i = 1; i < nb; ++i)
			for (PxU32 j = i; j > 0 && dist[order[j]] < dist[order[j - 1]]; --j)
				PxSwap(order[j], order[j - 1]);

		const PxU32 nbTris = PxMin(nb, 3u), nbIter = 6;
		for (PxU32 t = 0; t < nbTris; ++t)
		{
			const PxU32 ti = order[t];
			const IndexedTriangle32 tri = surf.triangle(tris[ti]);
			const PxVec3 v0 = surf.vertexWorld(tri.mRef[0]), v1 = surf.vertexWorld(tri.mRef[1]), v2 = surf.vertexWorld(tri.mRef[2]);
			const PxVec3 e1 = v1 - v0, e2 = v2 - v0;
			PxVec3 p = closestPtPointTriangle2(x, v0, v1, v2, e1, e2);
			PxVec3 n;
			PxReal sep = field.phi(p, n);
			SDF_STAT(descentEvals);
			PxReal obj = sep;
			for (PxU32 r = 0; r < ctx.nbPoints; ++r)
				obj -= ctx.alpha * (p - ctx.repel[r]).magnitude();
			PxReal step = 0.5f * R;
			for (PxU32 it = 0; it < nbIter && step > 0.25f * ctx.tau; ++it)
			{
				// descent direction: minus the field gradient plus the repulsion, projected on the triangle
				PxVec3 g = n;
				for (PxU32 r = 0; r < ctx.nbPoints; ++r)
				{
					const PxVec3 d = p - ctx.repel[r];
					const PxReal m = d.magnitude();
					if (m > 1e-9f)
						g -= d * (ctx.alpha / m);
				}
				PxVec3 trial = closestPtPointTriangle2(p - g * step, v0, v1, v2, e1, e2);
				if ((trial - x).magnitudeSquared() > RSq)
				{
					// stay inside the trust region: pull the trial back along the move
					const PxVec3 d = trial - p;
					const PxReal m = d.magnitude();
					if (m > 1e-9f)
					{
						const PxReal allowed = PxMax(0.0f, R - (p - x).magnitude());
						trial = closestPtPointTriangle2(p + d * (allowed / m), v0, v1, v2, e1, e2);
					}
				}
				PxVec3 tn;
				const PxReal tsep = field.phi(trial, tn);
				SDF_STAT(descentEvals);
				PxReal tobj = tsep;
				for (PxU32 r = 0; r < ctx.nbPoints; ++r)
					tobj -= ctx.alpha * (trial - ctx.repel[r]).magnitude();
				if (tobj < obj - 1e-7f && (trial - p).magnitudeSquared() > 1e-14f)
				{
					p = trial; n = tn; sep = tsep; obj = tobj;
				}
				else
					step *= 0.5f;
			}
			if (obj < bestObj)
			{
				bestObj = obj; bestPt = p; bestNormal = n; bestSep = sep; bestFace = tris[ti];
			}
		}
		if (bestPt != best)
			SDF_STAT(descentImproved);
		out.point = bestPt;
		out.normal = bestNormal;
		out.separation = bestSep;
		out.faceIndex = bestFace;
		best = bestPt;
		// the triangle normal below must belong to the final triangle
		const IndexedTriangle32 ftri = surf.triangle(out.faceIndex);
		const PxVec3 f0 = surf.vertexWorld(ftri.mRef[0]), f1 = surf.vertexWorld(ftri.mRef[1]), f2 = surf.vertexWorld(ftri.mRef[2]);
		triNormal = (f1 - f0).cross(f2 - f0);
	}
	const PxVec3 fieldNormal = out.normal;

	// Outward normal of the surface body at the point: the triangle normal, oriented by the body's
	// own field so that the mesh winding does not matter. PhysX does not require a winding for
	// plain triangle meshes (they collide double-sided), so without an SDF on the surface body the
	// triangle normal is oriented against the field's gradient and the facing test is moot.
	PxVec3 surfNormal(0.0f);
	const PxReal tn2 = triNormal.magnitudeSquared();
	if (surf.hasSdf)
	{
		surf.phi(best, surfNormal);
		if (tn2 > 1e-20f)
		{
			triNormal *= PxRecipSqrt(tn2);
			if (triNormal.dot(surfNormal) < 0.0f)
				triNormal = -triNormal;
			surfNormal = triNormal;
		}
	}
	else if (tn2 > 1e-20f)
	{
		// A plain mesh has no field to orient its triangles by, and the SDF body's gradient at a
		// point of the plain surface flips once a feature sinks past half its thickness (the
		// nearest SDF surface is then above the plane and the contact would push the body down).
		// Use the triangle normal oriented towards the SDF body as the separating direction: the
		// plain mesh's outward side is the one its contact partner sits on.
		triNormal *= PxRecipSqrt(tn2);
		if (triNormal.dot(field.worldBounds.getCenter() - best) < 0.0f)
			triNormal = -triNormal;
		surfNormal = triNormal;
		out.normal = -triNormal;
	}
	out.facing = -surfNormal.dot(fieldNormal);

	// Contact normal: the field's gradient (paper, Macklin), the surface triangle's normal, or their
	// average. Face-on-face contacts are exact with the triangle normal; a tip in a flank wants the
	// field's. The average keeps both roughly right and is smooth over a trilinear grid.
	if (ctx.P.normalSource == 1)
		out.normal = -surfNormal;
	else if (ctx.P.normalSource == 2)
	{
		PxVec3 n = fieldNormal - surfNormal;
		const PxReal m2 = n.magnitudeSquared();
		if (m2 > 1e-12f)
			out.normal = n * PxRecipSqrt(m2);
	}
	return true;
}

// Choose, per point, which body supplies the surface and which the field. Either body may be the
// one that penetrates: a bunny foot on the ground must snap to the foot and use the ground's
// gradient; snapping to the ground plane and using the bunny's gradient yields a normal along the
// foot's thickness, sideways or even downwards. The valid pairing is the one whose surface faces
// the push direction (surface normal opposite to the field gradient).
static bool polish(const PairContext& ctx, const PxVec3& x, Polished& out, bool& surfaceIs1, bool cached)
{
	Polished a, b;
	if (ctx.single)
	{
		const bool ok = polishPairing(ctx, ctx.surfBody(), ctx.fieldBody(), x, a, cached);
		if (!ok) { SDF_STAT(polishNoTriangles); return false; }
		if (a.facing <= -0.2f) { SDF_STAT(polishInverted); return false; }
		if (a.separation >= ctx.margin) { SDF_STAT(polishSeparated); return false; }
		out = a;
		surfaceIs1 = ctx.fieldIs0;	// the surface is the body without the SDF
		return true;
	}
	const bool okA = polishPairing(ctx, ctx.body1, ctx.body0, x, a, cached);	// surface 1, field 0
	const bool okB = polishPairing(ctx, ctx.body0, ctx.body1, x, b, cached);	// surface 0, field 1
	if (!okA && !okB)
	{
		SDF_STAT(polishNoTriangles);
		return false;
	}

	// A normal pointing into the surface body (facing < 0) would pull the bodies together. Grazing
	// contacts (facing ~ 0) are legitimate for thin features: the side of an ear in the ground gets
	// the ground's normal. Among acceptable pairings prefer the clearly better facing one, and when
	// they are alike the deeper one, which reports the true penetration (a foot on the ground: the
	// sole's depth in the ground, not half the foot's thickness at the ground plane).
	const PxReal inverted = -0.2f, alike = 0.3f;
	const bool accA = okA && a.facing > inverted, accB = okB && b.facing > inverted;
	if (!accA && !accB)
	{
		SDF_STAT(polishInverted);
		return false;
	}
	bool pickA;
	if (accA != accB)
		pickA = accA;
	else if (PxAbs(a.facing - b.facing) > alike)
		pickA = a.facing > b.facing;
	else
		pickA = a.separation <= b.separation;
	out = pickA ? a : b;
	surfaceIs1 = pickA;
	if (out.separation >= ctx.margin)
	{
		SDF_STAT(polishSeparated);
		return false;
	}
	return true;
}

} // anonymous namespace

PxU32 Gu::contactSDFSDF(
	const PxTriangleMeshGeometry& geom0, const PxTransform32& transform0,
	const PxTriangleMeshGeometry& geom1, const PxTransform32& transform1,
	const NarrowPhaseParams& params, Cache& cache, PxContactBuffer& contactBuffer)
{
	const SDFSDFParams& P = gSDFSDFParams;
#if GU_SDF_SDF_STATS
	const PxU64 t0 = PxTime::getCurrentCounterValue();
	struct Timer { PxU64 t0; ~Timer() { SDF_STAT_ADD(microseconds, PxU32(PxTime::getCounterFrequency().toTensOfNanos(PxTime::getCurrentCounterValue() - t0) / 100)); } } timer = { t0 };
	PxU64 tStage = t0;
#define SDF_STAGE(field) { const PxU64 tn = PxTime::getCurrentCounterValue(); SDF_STAT_ADD(field, PxU32(PxTime::getCounterFrequency().toTensOfNanos(tn - tStage) / 100)); tStage = tn; }
#else
#define SDF_STAGE(field) ((void)0)
#endif
	const SDFBody body0(geom0, transform0), body1(geom1, transform1);
	PairContext ctx(body0, body1, P);

	MultiplePersistentContactManifold* mm = cache.isMultiManifold() ? &cache.getMultipleManifold() : NULL;

	// Sec. 3.1: the search domain V is the overlap of the two SDF boxes, each grown by half the
	// contact margin so that contacts appear as soon as the bodies are within the margin
	ctx.margin = params.mContactDistance;
	ctx.V = body0.worldBounds;
	ctx.V.fattenSafe(0.5f * ctx.margin);
	PxBounds3 bounds1 = body1.worldBounds;
	bounds1.fattenSafe(0.5f * ctx.margin);
	ctx.V.minimum = ctx.V.minimum.maximum(bounds1.minimum);
	ctx.V.maximum = ctx.V.maximum.minimum(bounds1.maximum);
	SDF_STAT(calls);
	if (ctx.V.minimum.x > ctx.V.maximum.x || ctx.V.minimum.y > ctx.V.maximum.y || ctx.V.minimum.z > ctx.V.maximum.z)
	{
		SDF_STAT(emptyOverlap);
		if (mm)
			mm->clearManifold();
		return 0;
	}

	// Parameters relative to the pair's size L (the paper normalizes models to a unit cube)
	const PxReal L = PxMin((body0.worldBounds.maximum - body0.worldBounds.minimum).maxElement(),
						   (body1.worldBounds.maximum - body1.worldBounds.minimum).maxElement());
	const PxReal minSpacing = ctx.single ? ctx.fieldBody().spacingWorld : PxMin(body0.spacingWorld, body1.spacingWorld);
	ctx.L = L;
	ctx.tau = PxMax(P.tolerance * L, 0.5f * minSpacing);
	ctx.epsilon = P.epsilon * L * L;
	ctx.alpha = P.alpha;
	ctx.replacement = P.replacement * L;
	const PxVec3 vSize = ctx.V.maximum - ctx.V.minimum;
	const PxVec3 defaultStep = (vSize * 0.1f).maximum(PxVec3(0.5f * ctx.tau));

	// Deterministic per-pair, per-step random stream from the relative pose
	const PxTransform rel = transform0.transformInv(transform1);
	PxU32 seed = 2166136261u ^ (P.seedSalt * 0x9E3779B9u);
	seed = hashFloat(seed, rel.p.x); seed = hashFloat(seed, rel.p.y); seed = hashFloat(seed, rel.p.z);
	seed = hashFloat(seed, rel.q.x); seed = hashFloat(seed, rel.q.y); seed = hashFloat(seed, rel.q.z); seed = hashFloat(seed, rel.q.w);
	Rng rng(seed);

	SDF_STAGE(usCache);
	// Alg. 1 lines 4-10: warm start from the cached points of the previous step. Each cached point
	// is stored in both bodies' local frames; moving each with its body and averaging gives the
	// material-point advection of Eq. 6 without needing velocities.
	PxU32 nbCached = 0;
	const PxU32 nbCachedMax = PxMin(P.nbCached, MAX_CACHED);
	for (PxU32 m = 0; mm && m < mm->mNumManifolds && nbCached < nbCachedMax; ++m)
	{
		const SinglePersistentContactManifold& sm = *mm->getManifold(m);
		for (PxU32 i = 0; i < sm.mNumContacts && nbCached < nbCachedMax; ++i)
		{
			const MeshPersistentContact& c = sm.mContactPoints[i];
			PxVec3 localA, localB;
			PX_ALIGN(16, PxF32 normalPen[4]);
			V3StoreU(c.mLocalPointA, localA);
			V3StoreU(c.mLocalPointB, localB);
			V4StoreA(c.mLocalNormalPen, normalPen);

			const PxVec3 xa = transform0.transform(localA), xb = transform1.transform(localB);
			const PxVec3 seedPos = (xa + xb) * 0.5f;
			const PxVec3 prevNormal = transform0.rotate(PxVec3(normalPen[0], normalPen[1], normalPen[2]));
			// A cached point only has to follow the bodies: its simplex covers how far the two
			// material points disagree, at least 5 tau, never more than the default (not in the paper)
			const PxReal drift = PxMax((xa - xb).magnitude(), 5.0f * ctx.tau);
			const PxVec3 step = defaultStep.minimum(PxVec3(drift));
			ctx.addPoint(seedPos, step, true, prevNormal);
			++nbCached;
			SDF_STAT(cachedSeeds);
		}
	}

	// Alg. 1 lines 12-18: stochastic samples, which also top the cache up to n_cache points
	const PxU32 nbRandom = PxMin((ctx.single ? P.nbRandomSingle : P.nbRandom) + (nbCachedMax > nbCached ? nbCachedMax - nbCached : 0), MAX_POINTS - MAX_CACHED);
	SeedTriangles seedTris[2];
	seedTris[0].count = seedTris[1].count = 0;
	if (P.curvatureSeeds && nbRandom)
	{
		Rng* subBox = (P.subBoxSeeds && nbCached) ? &rng : NULL;
		seedTris[0].gather(body0, ctx.V, subBox);
		seedTris[1].gather(body1, ctx.V, subBox);
	}
	for (PxU32 i = 0; i < nbRandom; ++i)
	{
		const PxVec3 seedPos = stochasticSample(ctx, seedTris, rng);
		ctx.addPoint(seedPos, defaultStep, false, PxVec3(0.0f));
		SDF_STAT(randomSeeds);
	}

	SDF_STAGE(usSearch);
	// Alg. 1 lines 20-27 (Sec. 3.7): drop points that converged within tau of an earlier one.
	// Cached points come first, so they survive over new samples.
	ManifoldPoint* pts = ctx.points;
	PxU32 nbUnique = 0;
	const PxReal tauSq = ctx.tau * ctx.tau;
	for (PxU32 i = 0; i < ctx.nbPoints; ++i)
	{
		bool duplicate = false;
		for (PxU32 j = 0; j < nbUnique && !duplicate; ++j)
			duplicate = (pts[i].x - pts[j].x).magnitudeSquared() < tauSq;
		if (!duplicate)
			pts[nbUnique++] = pts[i];
	}

	// Polish the active points first: the surface point and its true penetration are what both the
	// solver and the ranking should see.
	Polished polished[MAX_POINTS];
	bool polishedOk[MAX_POINTS], surfaceIs1[MAX_POINTS];
	for (PxU32 i = 0; i < nbUnique; ++i)
	{
		ManifoldPoint& pt = pts[i];
		polishedOk[i] = false;
		surfaceIs1[i] = false;
		if (!pt.active || !P.polishWithTriangles)
			continue;
		if (polish(ctx, pt.x, polished[i], surfaceIs1[i], pt.cached))
		{
			polishedOk[i] = true;
			pt.x = polished[i].point;			// cache the refined location
			pt.faceIndex = polished[i].faceIndex;
			if (P.rankByDepth)
				pt.softmax = polished[i].separation;	// rank by the real penetration from here on
		}
		else
			pt.active = false;	// not a valid contact after all; it still stays in the cache ranking
	}

	SDF_STAGE(usPolish);
	// Alg. 1 line 28: rank by g against the final manifold, preferring deep points far from the
	// others. A new sample must improve on a cached point by a threshold to replace it, otherwise
	// equally deep points on flat contacts keep swapping and the manifold never settles.
	for (PxU32 i = 0; i < nbUnique; ++i)
	{
		PxReal r = pts[i].softmax + (pts[i].cached ? 0.0f : ctx.replacement);
		for (PxU32 j = 0; j < nbUnique; ++j)
			r -= ctx.alpha * (pts[i].x - pts[j].x).magnitude();
		pts[i].rank = r;
	}
	PxU32 orderIdx[MAX_POINTS];
	for (PxU32 i = 0; i < nbUnique; ++i)
		orderIdx[i] = i;
	for (PxU32 i = 1; i < nbUnique; ++i)	// stable insertion sort, ascending rank
		for (PxU32 j = i; j > 0 && pts[orderIdx[j - 1]].rank > pts[orderIdx[j]].rank; --j)
			PxSwap(orderIdx[j], orderIdx[j - 1]);

	// The rank decides which points survive into the cache, but contacts are emitted and cached
	// in their insertion order (cached points first, in last step's order, then new samples):
	// re-sorting by depth would shuffle nearly equal points every step, and the solver's load
	// distribution and friction anchors lurch with the order. Rebuild orderIdx as the kept set in
	// stable order.
	{
		const PxU32 nbKeepRank = PxMin(nbUnique, nbCachedMax);
		bool kept[MAX_POINTS];
		for (PxU32 i = 0; i < nbUnique; ++i)
			kept[i] = false;
		for (PxU32 k = 0; k < nbKeepRank; ++k)
			kept[orderIdx[k]] = true;
		PxU32 n = 0;
		for (PxU32 i = 0; i < nbUnique; ++i)
			if (kept[i])
				orderIdx[n++] = i;
		for (PxU32 i = 0; i < nbUnique; ++i)
			if (!kept[i])
				orderIdx[n++] = i;
	}

	// Emit contacts. PhysX normals point in the direction shape 0 must move; the paper's normal
	// points from body 0 towards body 1, so it is negated.
	PxU32 nbContacts = 0;
	bool anyPenetrating = false;
	for (PxU32 k = 0; k < nbUnique; ++k)
	{
		const PxU32 i = orderIdx[k];
		const ManifoldPoint& pt = pts[i];
		anyPenetrating |= pt.separation < 0.0f;
		if (!pt.active)
			continue;
		if (polishedOk[i])
		{
			const Polished& pol = polished[i];
			// pol.normal points out of the field body: shape 0 moves along it when it is the surface body
			const PxVec3 physxNormal = surfaceIs1[i] ? -pol.normal : pol.normal;
			if (contactBuffer.contact(pol.point, physxNormal, pol.separation, surfaceIs1[i] ? pol.faceIndex : PXC_CONTACT_NO_FACE_INDEX))
				++nbContacts;
		}
		else if (contactBuffer.contact(pt.x, -pt.normal, pt.separation))
			++nbContacts;
	}

	SDF_STAT_ADD(contacts, nbContacts);
	if (!nbContacts && anyPenetrating)
		SDF_STAT(callsNoContact);

	// Alg. 1 line 29: the best n_cache points become the next step's cache
	SDF_STAGE(usCache);
	if (mm)
	{
		mm->clearManifold();
		const PxU32 nbKeep = PxMin(nbUnique, nbCachedMax);
		PxU32 total = 0;
		while (total < nbKeep)
		{
			SinglePersistentContactManifold& sm = *mm->getEmptyManifold();
			sm.clearManifold();
			const PxU32 n = PxMin(nbKeep - total, PxU32(GU_SINGLE_MANIFOLD_CACHE_SIZE));
			for (PxU32 i = 0; i < n; ++i)
			{
				const ManifoldPoint& pt = pts[orderIdx[total + i]];
				MeshPersistentContact& c = sm.mContactPoints[i];
				const PxVec3 localN = transform0.rotateInv(pt.normal);
				c.mLocalPointA = V3LoadU(transform0.transformInv(pt.x));
				c.mLocalPointB = V3LoadU(transform1.transformInv(pt.x));
				c.mLocalNormalPen = V4LoadXYZW(localN.x, localN.y, localN.z, pt.softmax);
				c.mFaceIndex = pt.faceIndex;
			}
			sm.mNumContacts = n;
			mm->mNumManifolds++;
			total += n;
		}
		mm->mNumTotalContacts = PxU8(total);
		mm->setRelativeTransform(PxTransformV(V3LoadU(rel.p), QuatVLoadU(&rel.q.x)));
	}

	return nbContacts;
}
