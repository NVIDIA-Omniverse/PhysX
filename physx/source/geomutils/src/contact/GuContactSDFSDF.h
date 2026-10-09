// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_CONTACT_SDF_SDF_H
#define GU_CONTACT_SDF_SDF_H

// SDF-vs-SDF contact generation by temporal caching and importance sampling.
//
// Implements Algorithm 1 of
//   C. Giles and S. Andrews, "Real-Time Collision Handling for Signed Distance Fields via
//   Caching and Importance Sampling", Computer Graphics Forum 45(7), Pacific Graphics 2026.
//   Reference implementation (MIT): https://github.com/savant117/sdf-dcd-demo
//
// For a pair of triangle meshes that both carry an SDF, a handful of seed points are optimized
// (Nelder-Mead) towards points of deep intersection of the two fields. Seeds are the contact
// points of the previous step, carried in the pair's persistent contact manifold, plus a few
// stochastic samples drawn from mesh vertices inside the overlap of the two SDF boxes. Each
// converged point is then polished against the actual mesh surface with the existing per-triangle
// optimizer (GuContactMeshMesh.h), so contacts lie exactly on the mesh and use its normals.
//
// Compared to the all-overlapping-triangles approach in GuContactMeshMesh.cpp this produces a
// sparse, temporally stable manifold whose cost does not depend on the triangle count, and it
// touches far fewer SDF cells, which matters for lazily evaluated SDFs.

#include "foundation/PxPreprocessor.h"
#include "foundation/PxSimpleTypes.h"
#include "common/PxPhysXCommonConfig.h"

namespace physx
{
class PxContactBuffer;
class PxTriangleMeshGeometry;
struct PxTransformPadded;
typedef PxTransformPadded PxTransform32;

namespace Gu
{
	struct Cache;
	struct NarrowPhaseParams;

	// The path is enabled per scene with PxSceneFlag::eENABLE_SDF_SDF_CONTACTS (carried in
	// NarrowPhaseParams::mSDFSDFContacts); pairs where only one mesh has an SDF use the per-triangle
	// path regardless.

	// Tunables of the method, in units of the pair's size L (the larger extent of the smaller SDF
	// box in world space) unless noted. Defaults match Sec. 4 of the paper except for the sample
	// counts: the paper's 4 cached + 1 random points suit its own solver, PhysX's friction patches
	// need more support; 8 cached + 3 random lets gear towers, bunny piles and cows on spikes come
	// to rest about as fast as the per-triangle method (with 1 random point a pile of 27 bunnies
	// takes 1.5-2x longer to sleep and sometimes never does).
	struct SDFSDFParams
	{
		PxU32	nbCached;				// n_cache: points kept in the temporal cache (at most 36)
		PxU32	nbRandom;				// n_rand: new stochastic samples per step
		PxU32	nbRandomSingle;			// n_rand for pairs with one SDF (no Nelder-Mead stage: exploration is all they have)
		PxU32	maxIterations;			// Nelder-Mead iteration limit
		PxReal	tolerance;				// tau, relative to L (also the duplicate distance); never below the SDF spacing
		PxReal	epsilon;				// softmax smoothing, relative to L^2
		PxReal	alpha;					// repulsion weight (dimensionless)
		PxReal	replacement;			// improvement a new sample needs to replace a cached point, relative to L
		PxU32	seedCandidates;			// stochastic samples are the best of this many vertex candidates
		bool	polishWithTriangles;	// refine converged points against the mesh surface
		bool	curvatureSeeds;			// draw seeds from mesh vertices (false: uniform in the overlap box)
		PxU32	normalSource;			// contact normal: 0 field gradient, 1 surface triangle normal, 2 their average
		PxU32	seedSalt;				// mixed into the per-pair random seed (testing: robustness to the random stream)
		bool	surfaceDescent;			// after snapping, descend over the nearby surface towards deeper penetration (trust region)
		bool	subBoxSeeds;			// draw seed triangles from a random sub-box of the overlap once a cache exists
		bool	rankByDepth;			// rank points by their polished penetration instead of the objective at x*
		PxReal	stickiness;				// a cached point keeps its advected position unless re-optimizing improves g by this fraction of `replacement` (0: always re-optimize, as in the paper; 0.3 damps the simplex walk on flat contacts but makes piles settle 2x slower)

		PX_FORCE_INLINE SDFSDFParams() :
			nbCached(8), nbRandom(3), nbRandomSingle(3), maxIterations(50), tolerance(0.01f), epsilon(0.1f), alpha(0.01f),
			replacement(0.01f), seedCandidates(4), polishWithTriangles(true), curvatureSeeds(true), normalSource(0), seedSalt(0), surfaceDescent(true), subBoxSeeds(true), rankByDepth(true), stickiness(0.0f) {}
	};

	PX_PHYSX_COMMON_API extern SDFSDFParams gSDFSDFParams;

	// Diagnostic counters, accumulated atomically over all pairs. Only maintained in debug and
	// checked builds (GU_SDF_SDF_STATS); reset them yourself between readings.
	struct SDFSDFStats
	{
		PxU32	calls;				// pair evaluations
		PxU32	emptyOverlap;		// pairs whose SDF boxes do not overlap
		PxU32	cachedSeeds;		// warm-started points
		PxU32	randomSeeds;		// stochastic points
		PxU32	iterations;			// Nelder-Mead iterations
		PxU32	activePoints;		// points classified as contacts before polishing
		PxU32	polishNoTriangles;	// no surface triangle found around the point
		PxU32	polishInverted;		// both pairings had the normal pointing into the surface body
		PxU32	polishSeparated;	// surface point beyond the margin
		PxU32	contacts;			// contacts written
		PxU32	callsNoContact;		// pairs with penetrating points but no contact written
		PxU32	descentEvals;		// field evaluations spent in the surface descent
		PxU32	descentImproved;	// points the descent moved to a deeper position
		PxU32	fieldEvals;			// SDF evaluations (all stages)
		PxU32	microseconds;		// wall clock spent in contactSDFSDF
		PxU32	usSearch;			// of which: seeds + Nelder-Mead (addPoint)
		PxU32	usPolish;			// of which: polish + surface descent
		PxU32	usCache;			// of which: cache read/write and bookkeeping
	};
#define GU_SDF_SDF_STATS (PX_DEBUG || PX_CHECKED)
	PX_PHYSX_COMMON_API extern SDFSDFStats gSDFSDFStats;

	// Generate contacts between two triangle meshes of which at least one has an SDF. With two SDFs
	// the seeds are minimized over the SDF-SDF objective; with one, the seeds are surface points of
	// the other mesh and only the surface descent optimizes them against the field. Returns the
	// number of contacts written to `contactBuffer`. `cache` may or may not carry a multi-manifold;
	// without one the method runs statelessly.
	PX_PHYSX_COMMON_API PxU32 contactSDFSDF(
		const PxTriangleMeshGeometry& geom0, const PxTransform32& transform0,
		const PxTriangleMeshGeometry& geom1, const PxTransform32& transform1,
		const NarrowPhaseParams& params, Cache& cache, PxContactBuffer& contactBuffer);
}
}

#endif
