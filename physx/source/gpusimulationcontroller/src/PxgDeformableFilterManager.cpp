// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxgDeformableFilterManager.h"

#include "foundation/PxSort.h"

#include "GuTetrahedronMesh.h"
#include "GuTriangleMesh.h"
#include "PxgFEMCloth.h"

namespace physx
{
	namespace
	{
		struct ComparePairInfo
		{
			bool operator()(const PxgDeformablePairInfo& first, const PxgDeformablePairInfo& second) const
			{
				return PxgComparePairInfo(first, second) < 0;
			}
		};

		// Collapses a sorted list into unique pairs and the number of times each occurs.
		void collapseSorted(const PxArray<PxgDeformablePairInfo>& sorted, PxArray<PxgDeformablePairInfo>& pairs,
			PxArray<PxU32>& counts)
		{
			for (PxU32 i = 0; i < sorted.size(); ++i)
			{
				PxU32 count = 1;
				while (i + 1 < sorted.size() && PxgComparePairInfo(sorted[i], sorted[i + 1]) == 0)
				{
					++count;
					++i;
				}

				pairs.pushBack(sorted[i]);
				counts.pushBack(count);
			}
		}

		// Sorts appended entries and run-length collapses them into unique pairs and the number of times
		// each occurs. The same for an addition and a removal, which differ only in what the counts are
		// then used for.
		void stageFilterPairs(PxArray<PxgDeformablePairInfo>& appended, PxArray<PxgDeformablePairInfo>& pairs,
			PxArray<PxU32>& counts)
		{
			ComparePairInfo predicate;
			PxSort(appended.begin(), appended.size(), predicate);

			collapseSorted(appended, pairs, counts);
		}

		// Merges sorted, collapsed pairs into the sorted array. Pairs already there have the staged counts
		// added, the rest are inserted. Returns whether anything was inserted, which is the only case
		// where the uploaded array changes.
		bool mergeFilterPairs(const PxArray<PxgDeformablePairInfo>& stagedPairs, const PxArray<PxU32>& stagedCounts,
			PxgDeformableFilterPairs& filters)
		{
			Cm::PinnableArray<PxgDeformablePairInfo>& pairs = filters.mPairs;
			PxArray<PxU32>& references = filters.mReferences;

			const PxU32 nbPairs = pairs.size();
			const PxU32 nbStaged = stagedPairs.size();

			// The array grows by the number of staged pairs it does not hold yet.
			PxU32 nbInserted = 0;
			for (PxU32 i = 0, j = 0; j < nbStaged; ++j)
			{
				while (i < nbPairs && PxgComparePairInfo(pairs[i], stagedPairs[j]) < 0)
					++i;

				if (i < nbPairs && PxgComparePairInfo(pairs[i], stagedPairs[j]) == 0)
					++i;
				else
					++nbInserted;
			}

			pairs.reserve(nbPairs + nbInserted);
			references.reserve(nbPairs + nbInserted);
			pairs.forceSize_Unsafe(nbPairs + nbInserted);
			references.forceSize_Unsafe(nbPairs + nbInserted);

			// Backwards, so that an entry is only ever written to a slot already vacated.
			PxU32 write = nbPairs + nbInserted;
			PxU32 read = nbPairs;

			for (PxU32 j = nbStaged; j-- > 0; )
			{
				while (read > 0 && PxgComparePairInfo(pairs[read - 1], stagedPairs[j]) > 0)
				{
					--read;
					--write;
					pairs[write] = pairs[read];
					references[write] = references[read];
				}

				--write;
				if (read > 0 && PxgComparePairInfo(pairs[read - 1], stagedPairs[j]) == 0)
				{
					--read;
					pairs[write] = pairs[read];
					references[write] = references[read] + stagedCounts[j];
				}
				else
				{
					pairs[write] = stagedPairs[j];
					references[write] = stagedCounts[j];
				}
			}

			return nbInserted != 0;
		}

		// Subtracts the staged counts and drops the pairs that reach zero. Returns whether any pair was
		// dropped.
		bool unmergeFilterPairs(const PxArray<PxgDeformablePairInfo>& stagedPairs, const PxArray<PxU32>& stagedCounts,
			PxgDeformableFilterPairs& filters)
		{
			Cm::PinnableArray<PxgDeformablePairInfo>& pairs = filters.mPairs;
			PxArray<PxU32>& references = filters.mReferences;

			const PxU32 nbPairs = pairs.size();
			const PxU32 nbStaged = stagedPairs.size();

			PxU32 write = 0, j = 0;

			for (PxU32 i = 0; i < nbPairs; ++i)
			{
				// A staged pair the array does not hold is a removal of a filter that was never added.
				while (j < nbStaged && PxgComparePairInfo(stagedPairs[j], pairs[i]) < 0)
					++j;

				PxU32 refCount = references[i];
				if (j < nbStaged && PxgComparePairInfo(stagedPairs[j], pairs[i]) == 0)
				{
					refCount = refCount > stagedCounts[j] ? refCount - stagedCounts[j] : 0;
					++j;
				}

				if (refCount == 0)
					continue;

				pairs[write] = pairs[i];
				references[write] = refCount;
				++write;
			}

			pairs.forceSize_Unsafe(write);
			references.forceSize_Unsafe(write);

			return write != nbPairs;
		}

		// Maps a collision mesh element from the BV4 numbering the API uses to the BV32 numbering the GPU
		// mesh carries. The wildcard passes through unchanged.

		PxU32 toCollisionGpuTetIndex(const Gu::BVTetrahedronMesh& tetMesh, PxU32 tetId)
		{
			return tetId == PxgDeformableRef::ANY_FEATURE ? tetId : tetMesh.mGRB_faceRemapInverse[tetId];
		}

		PxU32 toCollisionGpuTriIndex(const Gu::TriangleMesh& triangleMesh, PxU32 triId)
		{
			return triId == PxgDeformableRef::ANY_FEATURE ? triId : triangleMesh.mGRB_faceRemapInverse[triId];
		}

		// Reads the three vertex indices of a triangle in the GPU mesh's numbering.
		void getGRBVertexIndices(PxU32& vref0, PxU32& vref1, PxU32& vref2, PxU32 triIndex,
			const Gu::TriangleMesh* triangleMesh)
		{
			const PxU32 offset = triIndex * 3;
			if(triangleMesh->has16BitIndices())
			{
				const PxU16* triInds = reinterpret_cast<PxU16*>(triangleMesh->mGRB_triIndices);
				vref0 = triInds[offset + 0];
				vref1 = triInds[offset + 1];
				vref2 = triInds[offset + 2];
			}
			else
			{
				const PxU32* triInds = reinterpret_cast<PxU32*>(triangleMesh->mGRB_triIndices);
				vref0 = triInds[offset + 0];
				vref1 = triInds[offset + 1];
				vref2 = triInds[offset + 2];
			}
		}

		// Converts source tetrahedron indices to GPU mesh indices and adds rigid-tetrahedron pairs to the
		// output array. Wildcards are skipped as a safeguard (already checked in NpDeformableElementFilter).
		// The rigid is passed as its filter reference, a node or a static's shape. See PxgEncodeRigidFilterRef.
		void appendRigidVolumeFilterPairs(const Gu::BVTetrahedronMesh& volumeMesh, PxU32 volumeId,
			PxU64 rigidRef, const PxU32* tetIndices, PxU32 tetIndicesSize,
			PxArray<PxgDeformablePairInfo>& appended)
		{
			appended.reserve(tetIndicesSize);

			for (PxU32 i = 0; i < tetIndicesSize; ++i)
			{
				if (tetIndices[i] == PxgDeformableRef::ANY_FEATURE)
					continue;

				const PxU32 volumeTetId = toCollisionGpuTetIndex(volumeMesh, tetIndices[i]);

				const PxgDeformablePairInfo pair = { rigidRef, PxgEncodeDeformableTetrahedron(volumeId, volumeTetId) };
				appended.pushBack(pair);
			}
		}

		// Expands a tetrahedron-tetrahedron pair to four vertex-tetrahedron pairs and appends them to the
		// output array. A wildcard tetrahedron contributes one entry standing for the whole volume.
		// Expects GPU indices.
		void appendTetVertexPairs(const Gu::BVTetrahedronMesh& volumeMesh, PxU32 volumeId, PxU32 tetId, PxU64 otherTetRef,
			PxArray<PxgDeformablePairInfo>& pairs)
		{
			if (tetId == PxgDeformableRef::ANY_FEATURE)
			{
				const PxgDeformablePairInfo pair = {
					PxgEncodeDeformableVertex(volumeId, PxgDeformableRef::ANY_FEATURE), otherTetRef };
				pairs.pushBack(pair);
				return;
			}

			const uint4 tetIndices = static_cast<const uint4*>(volumeMesh.mGRB_tetraIndices)[tetId];
			const PxU32* vertexIndices = reinterpret_cast<const PxU32*>(&tetIndices);

			for (PxU32 i = 0; i < 4; ++i)
			{
				const PxgDeformablePairInfo pair = { PxgEncodeDeformableVertex(volumeId, vertexIndices[i]), otherTetRef };
				pairs.pushBack(pair);
			}
		}

		// Converts source tetrahedron indices to GPU mesh indices and adds vertex-tetrahedron pairs to the
		// per-volume output arrays.
		void appendVolumeVolumeFilterPairs(const Gu::BVTetrahedronMesh& volumeMesh0, PxU32 volumeId0,
			const Gu::BVTetrahedronMesh& volumeMesh1, PxU32 volumeId1,
			const PxU32* tetIndices0, const PxU32* tetIndices1, PxU32 tetIndicesSize,
			PxArray<PxgDeformablePairInfo>& appended0, PxArray<PxgDeformablePairInfo>& appended1)
		{
			appended0.reserve(tetIndicesSize * 4);
			appended1.reserve(tetIndicesSize * 4);

			for (PxU32 i = 0; i < tetIndicesSize; ++i)
			{
				const PxU32 tetId0 = toCollisionGpuTetIndex(volumeMesh0, tetIndices0[i]);
				const PxU32 tetId1 = toCollisionGpuTetIndex(volumeMesh1, tetIndices1[i]);

				if (tetId0 == PxgDeformableRef::ANY_FEATURE && tetId1 == PxgDeformableRef::ANY_FEATURE)
					continue;

				// Collision runs in both directions, testing a vertex of one volume against a tetrahedron of
				// the other, so each direction needs its own entries.
				appendTetVertexPairs(volumeMesh0, volumeId0, tetId0, PxgEncodeDeformableTetrahedron(volumeId1, tetId1), appended0);

				appendTetVertexPairs(volumeMesh1, volumeId1, tetId1, PxgEncodeDeformableTetrahedron(volumeId0, tetId0), appended1);
			}
		}

		// Converts source indices to GPU mesh indices and adds the pairs the filter expands to. A filtered
		// triangle contributes one triangle entry for the face paths and one vertex entry per corner for the
		// vertex paths. A wildcard triangle contributes one entry of each kind standing for the whole
		// surface.
		void appendVolumeSurfaceFilterPairs(const Gu::BVTetrahedronMesh& volumeMesh, PxU32 volumeId,
			const Gu::TriangleMesh* surfaceMesh, PxU32 surfaceId,
			const PxU32* triIndices, const PxU32* tetIndices, PxU32 indicesSize, PxArray<PxgDeformablePairInfo>& appended)
		{
			appended.reserve(indicesSize * 4); // 3 verts and one tri

			for (PxU32 i = 0; i < indicesSize; ++i)
			{
				const PxU32 surfaceTriId = toCollisionGpuTriIndex(*surfaceMesh, triIndices[i]);
				const PxU32 volumeTetId = toCollisionGpuTetIndex(volumeMesh, tetIndices[i]);

				if (surfaceTriId == PxgDeformableRef::ANY_FEATURE && volumeTetId == PxgDeformableRef::ANY_FEATURE)
					continue;

				const PxU64 volumeTetRef = PxgEncodeDeformableTetrahedron(volumeId, volumeTetId);

				const PxgDeformablePairInfo trianglePair = { volumeTetRef, PxgEncodeDeformableTriangle(surfaceId, surfaceTriId) };
				appended.pushBack(trianglePair);

				if (surfaceTriId != PxgDeformableRef::ANY_FEATURE)
				{
					PxU32 surfaceVertexIds[3];
					getGRBVertexIndices(surfaceVertexIds[0], surfaceVertexIds[1], surfaceVertexIds[2], surfaceTriId, surfaceMesh);

					for (PxU32 j = 0; j < 3; ++j)
					{
						const PxgDeformablePairInfo pair = { volumeTetRef, PxgEncodeDeformableVertex(surfaceId, surfaceVertexIds[j]) };
						appended.pushBack(pair);
					}
				}
				else
				{
					const PxgDeformablePairInfo pair = { volumeTetRef,
						PxgEncodeDeformableVertex(surfaceId, PxgDeformableRef::ANY_FEATURE) };
					appended.pushBack(pair);
				}
			}
		}

		// Converts source triangle indices to GPU mesh indices and adds the pairs the filter expands to. A
		// filtered triangle contributes one triangle entry for the face paths and one vertex entry per
		// corner for the vertex paths.
		void appendRigidSurfaceFilterPairs(const Gu::TriangleMesh* surfaceMesh, PxU32 surfaceId,
			PxU64 rigidRef,
			const PxU32* triIndices, PxU32 triIndicesSize, PxArray<PxgDeformablePairInfo>& appended)
		{
			appended.reserve(triIndicesSize * 4); // 3 verts and one tri

			for (PxU32 i = 0; i < triIndicesSize; ++i)
			{
				if (triIndices[i] == PxgDeformableRef::ANY_FEATURE)
					continue;

				const PxU32 surfaceTriId = toCollisionGpuTriIndex(*surfaceMesh, triIndices[i]);

				const PxgDeformablePairInfo trianglePair = { rigidRef, PxgEncodeDeformableTriangle(surfaceId, surfaceTriId) };
				appended.pushBack(trianglePair);

				PxU32 surfaceVertexIds[3];
				getGRBVertexIndices(surfaceVertexIds[0], surfaceVertexIds[1], surfaceVertexIds[2], surfaceTriId, surfaceMesh);

				for (PxU32 j = 0; j < 3; ++j)
				{
					const PxgDeformablePairInfo pair = { rigidRef, PxgEncodeDeformableVertex(surfaceId, surfaceVertexIds[j]) };
					appended.pushBack(pair);
				}
			}
		}

		// Expands a triangle-triangle pair to three vertex-triangle pairs and appends them to the output
		// array. A wildcard triangle contributes one entry standing for the whole surface. Expects GPU
		// indices.
		void appendTriangleVertexPairs(const Gu::TriangleMesh& surfaceMesh, PxU32 surfaceId, PxU32 surfaceTriId,
			PxU64 otherSurfaceTriRef, PxArray<PxgDeformablePairInfo>& pairs)
		{
			if (surfaceTriId == PxgDeformableRef::ANY_FEATURE)
			{
				const PxgDeformablePairInfo pair = {
					PxgEncodeDeformableVertex(surfaceId, PxgDeformableRef::ANY_FEATURE), otherSurfaceTriRef };
				pairs.pushBack(pair);
				return;
			}

			PxU32 surfaceVertexIds[3];
			getGRBVertexIndices(surfaceVertexIds[0], surfaceVertexIds[1], surfaceVertexIds[2], surfaceTriId, &surfaceMesh);

			for (PxU32 i = 0; i < 3; ++i)
			{
				const PxgDeformablePairInfo pair = { PxgEncodeDeformableVertex(surfaceId, surfaceVertexIds[i]), otherSurfaceTriRef };
				pairs.pushBack(pair);
			}
		}

		// Local index of the edge between two vertices, or 3 if the triangle does not have that edge.
		PxU32 findLocalEdge(const Gu::TriangleMesh& surfaceMesh, PxU32 surfaceTriId, PxU32 vertexA, PxU32 vertexB)
		{
			PxU32 vertexIds[3];
			getGRBVertexIndices(vertexIds[0], vertexIds[1], vertexIds[2], surfaceTriId, &surfaceMesh);

			for (PxU32 e = 0; e < 3; ++e)
			{
				const PxU32 vertex0 = vertexIds[e];
				const PxU32 vertex1 = vertexIds[(e + 1) % 3];

				if ((vertex0 == vertexA && vertex1 == vertexB) || (vertex0 == vertexB && vertex1 == vertexA))
					return e;
			}

			return 3;
		}

		// For each of the three edges of a triangle, collects the reference identifying the triangle that
		// edge is tested from, and returns the number of references written. A wildcard triangle contributes
		// a single reference for the whole surface. Expects GPU indices.
		PxU32 collectTriangleEdgeRefs(const Gu::TriangleMesh& surfaceMesh, const PxgDeformableSurfaceTopology& topology,
			PxU32 surfaceId, PxU32 surfaceTriId, PxU64 edgeRefs[3])
		{
			if (surfaceTriId == PxgDeformableRef::ANY_FEATURE)
			{
				edgeRefs[0] = PxgEncodeDeformableTriangleEdge(surfaceId, PxgDeformableRef::ANY_FEATURE, 0);
				return 1;
			}

			const PxU32* accumulatedTriangleRefs = surfaceMesh.getAccumulatedTriangleRef();
			const PxU32* triangleRefs = surfaceMesh.getTriangleReferences();
			const PxU32 nbTriangleRefs = surfaceMesh.getNbTriangleReferences();
			const PxU32 nbVertices = surfaceMesh.getNbVerticesFast();

			PxU32 vertexIds[3];
			getGRBVertexIndices(vertexIds[0], vertexIds[1], vertexIds[2], surfaceTriId, &surfaceMesh);

			for (PxU32 e = 0; e < 3; ++e)
			{
				edgeRefs[e] = PxgEncodeDeformableTriangleEdge(surfaceId, surfaceTriId, e);

				if (topology.ownsEdge(surfaceTriId, e))
					continue;

				const PxU32 vertexA = vertexIds[e];
				const PxU32 vertexB = vertexIds[(e + 1) % 3];

				// The vertex-to-triangle map is keyed on GPU indices, so it needs no conversion here.
				const PxU32 begin = accumulatedTriangleRefs[vertexA];
				const PxU32 end = (vertexA + 1 < nbVertices) ? accumulatedTriangleRefs[vertexA + 1] : nbTriangleRefs;

				bool found = false;
				for (PxU32 i = begin; i < end; ++i)
				{
					const PxU32 otherTriId = triangleRefs[i];
					if (otherTriId == surfaceTriId)
						continue;

					const PxU32 otherLocalEdge = findLocalEdge(surfaceMesh, otherTriId, vertexA, vertexB);
					if (otherLocalEdge == 3 || !topology.ownsEdge(otherTriId, otherLocalEdge))
						continue;

					edgeRefs[e] = PxgEncodeDeformableTriangleEdge(surfaceId, otherTriId, otherLocalEdge);
					found = true;
					break;
				}

				PX_ASSERT(found);
				PX_UNUSED(found);
			}

			return 3;
		}

		// Pairs one surface's edge references with the other's and appends them to that surface's output
		// array. This is the edge-edge half of a triangle-triangle filter, in one direction.
		void appendTriangleEdgePairs(const PxU64* edgeRefs, PxU32 nbEdgeRefs, const PxU64* otherEdgeRefs, PxU32 nbOtherEdgeRefs,
			PxArray<PxgDeformablePairInfo>& pairs)
		{
			for (PxU32 i = 0; i < nbEdgeRefs; ++i)
			{
				for (PxU32 j = 0; j < nbOtherEdgeRefs; ++j)
				{
					const PxgDeformablePairInfo pair = { edgeRefs[i], otherEdgeRefs[j] };
					pairs.pushBack(pair);
				}
			}
		}

		// Converts source triangle indices to GPU mesh indices and adds vertex-triangle pairs to the
		// per-surface output arrays. Collision runs in both directions, testing a vertex of one surface
		// against a triangle of the other, so each direction needs its own entries.
		void appendSurfaceSurfaceFilterPairs(const Gu::TriangleMesh* surfaceMesh0,
			const PxgDeformableSurfaceTopology& topology0, PxU32 surfaceId0,
			const Gu::TriangleMesh* surfaceMesh1, const PxgDeformableSurfaceTopology& topology1, PxU32 surfaceId1,
			const PxU32* triIndices0, const PxU32* triIndices1, PxU32 triIndicesSize,
			PxArray<PxgDeformablePairInfo>& appended0, PxArray<PxgDeformablePairInfo>& appended1)
		{
			// Per filtered triangle pair, each array takes maximally three vertex-face and 3x3 edge-edge pairs.
			appended0.reserve(triIndicesSize * (3 + 3 * 3));
			appended1.reserve(triIndicesSize * (3 + 3 * 3));

			for (PxU32 i = 0; i < triIndicesSize; ++i)
			{
				const PxU32 surfaceTriId0 = toCollisionGpuTriIndex(*surfaceMesh0, triIndices0[i]);
				const PxU32 surfaceTriId1 = toCollisionGpuTriIndex(*surfaceMesh1, triIndices1[i]);

				if (surfaceTriId0 == PxgDeformableRef::ANY_FEATURE && surfaceTriId1 == PxgDeformableRef::ANY_FEATURE)
					continue;

				appendTriangleVertexPairs(*surfaceMesh0, surfaceId0, surfaceTriId0,
					PxgEncodeDeformableTriangle(surfaceId1, surfaceTriId1), appended0);

				appendTriangleVertexPairs(*surfaceMesh1, surfaceId1, surfaceTriId1,
					PxgEncodeDeformableTriangle(surfaceId0, surfaceTriId0), appended1);

				// Each edge of a single triangle is represented with an encoded triangle-edge reference. The
				// edge-owning triangle may be the triangle itself, or an adjacent triangle.
				PxU64 edgeRefs0[3], edgeRefs1[3];
				const PxU32 nbEdgeRefs0 = collectTriangleEdgeRefs(*surfaceMesh0, topology0, surfaceId0, surfaceTriId0, edgeRefs0);
				const PxU32 nbEdgeRefs1 = collectTriangleEdgeRefs(*surfaceMesh1, topology1, surfaceId1, surfaceTriId1, edgeRefs1);

				appendTriangleEdgePairs(edgeRefs0, nbEdgeRefs0, edgeRefs1, nbEdgeRefs1, appended0);
				appendTriangleEdgePairs(edgeRefs1, nbEdgeRefs1, edgeRefs0, nbEdgeRefs0, appended1);
			}
		}
	}

	PxgDeformableFilterManager::~PxgDeformableFilterManager()
	{
		// Deleting all pairs, even though they should already have been deleted by the prior deformable
		// removal.
		for (PxU32 i = 0; i < mPerActorPairs.size(); ++i)
			PX_DELETE(mPerActorPairs[i]);
	}

	PxgDeformableFilterPairs& PxgDeformableFilterManager::getOrCreate(PxU32 deformableId)
	{
		if (mPerActorPairs.size() <= deformableId)
			mPerActorPairs.resize(deformableId + 1, NULL);

		PxgDeformableFilterPairs*& filters = mPerActorPairs[deformableId];
		if (filters == NULL)
			filters = PX_NEW(PxgDeformableFilterPairs)(mHostAlloc, mHeapStatGroup);

		return *filters;
	}

	void PxgDeformableFilterManager::markDirty(PxU32 deformableId)
	{
		// A deformable with no pairs has no storage and nothing to upload, so this looks up rather than
		// creates.
		PxgDeformableFilterPairs* filters = deformableId < mPerActorPairs.size() ? mPerActorPairs[deformableId] : NULL;

		if (filters && filters->mDirty && !filters->mInDirtyList)
		{
			mDirtyIds.pushBack(deformableId);
			filters->mInDirtyList = true;
		}
	}

	// Sorts and collapses the entries expanded for one deformable, then applies them to its array. Only
	// the pairs are uploaded, so the deformable is filed for the next upload when a pair is inserted or
	// dropped, but not when a count alone changes.
	void PxgDeformableFilterManager::update(PxArray<PxgDeformablePairInfo>& appended, PxU32 deformableId, bool add)
	{
		if (appended.empty())
			return;

		PxArray<PxgDeformablePairInfo> pairs;
		PxArray<PxU32> counts;
		stageFilterPairs(appended, pairs, counts);

		PxgDeformableFilterPairs& filters = getOrCreate(deformableId);

		const bool pairsChanged = add ? mergeFilterPairs(pairs, counts, filters)
		                              : unmergeFilterPairs(pairs, counts, filters);

		if (pairsChanged)
		{
			filters.mDirty = true;
			markDirty(deformableId);
		}
	}

	void PxgDeformableFilterManager::release(PxU32 deformableId)
	{
		for (PxU32 i = 0; i < mDirtyIds.size(); ++i)
		{
			if (mDirtyIds[i] == deformableId)
			{
				mDirtyIds.replaceWithLast(i);
				break;
			}
		}

		if (deformableId < mPerActorPairs.size())
		{
			PX_DELETE(mPerActorPairs[deformableId]);
			mPerActorPairs[deformableId] = NULL;
		}
	}

	void PxgDeformableFilterManager::updateVolumeRigid(const Gu::BVTetrahedronMesh& volumeMesh, PxU32 volumeId,
		const PxNodeIndex& rigidNodeIndex, const PxU32* tetIndices, PxU32 tetIndicesSize, bool add)
	{
		PxArray<PxgDeformablePairInfo> appended;
		appendRigidVolumeFilterPairs(volumeMesh, volumeId, PxgEncodeRigidNode(rigidNodeIndex), tetIndices, tetIndicesSize, appended);

		update(appended, volumeId, add);
	}

	void PxgDeformableFilterManager::updateVolumeStaticShapes(const Gu::BVTetrahedronMesh& volumeMesh, PxU32 volumeId,
		const PxU32* shapeCacheIds, PxU32 nbShapes, const PxU32* tetIndices, PxU32 tetIndicesSize, bool add)
	{
		PxArray<PxgDeformablePairInfo> appended;
		appended.reserve(nbShapes * tetIndicesSize);
		for (PxU32 i = 0; i < nbShapes; ++i)
			appendRigidVolumeFilterPairs(volumeMesh, volumeId, PxgEncodeStaticShapeRef(shapeCacheIds[i]), tetIndices, tetIndicesSize, appended);

		update(appended, volumeId, add);
	}

	void PxgDeformableFilterManager::updateVolumeVolume(const Gu::BVTetrahedronMesh& volumeMesh0, PxU32 volumeId0,
		const Gu::BVTetrahedronMesh& volumeMesh1, PxU32 volumeId1, const PxU32* tetIndices0, const PxU32* tetIndices1,
		PxU32 tetIndicesSize, bool add)
	{
		PxArray<PxgDeformablePairInfo> appended0, appended1;
		appendVolumeVolumeFilterPairs(volumeMesh0, volumeId0, volumeMesh1, volumeId1, tetIndices0, tetIndices1,
			tetIndicesSize, appended0, appended1);

		update(appended0, volumeId0, add);
		update(appended1, volumeId1, add);
	}

	void PxgDeformableFilterManager::updateVolumeSurface(const Gu::BVTetrahedronMesh& volumeMesh, PxU32 volumeId,
		const Gu::TriangleMesh& surfaceMesh, PxU32 surfaceId, const PxU32* triIndices, const PxU32* tetIndices,
		PxU32 indicesSize, bool add)
	{
		PxArray<PxgDeformablePairInfo> appended;
		appendVolumeSurfaceFilterPairs(volumeMesh, volumeId, &surfaceMesh, surfaceId, triIndices, tetIndices,
			indicesSize, appended);

		update(appended, volumeId, add);
	}

	void PxgDeformableFilterManager::updateSurfaceRigid(const Gu::TriangleMesh& surfaceMesh, PxU32 surfaceId,
		const PxNodeIndex& rigidNodeIndex, const PxU32* triIndices, PxU32 triIndicesSize, bool add)
	{
		PxArray<PxgDeformablePairInfo> appended;
		appendRigidSurfaceFilterPairs(&surfaceMesh, surfaceId, PxgEncodeRigidNode(rigidNodeIndex), triIndices, triIndicesSize, appended);

		update(appended, surfaceId, add);
	}

	void PxgDeformableFilterManager::updateSurfaceStaticShapes(const Gu::TriangleMesh& surfaceMesh, PxU32 surfaceId,
		const PxU32* shapeCacheIds, PxU32 nbShapes, const PxU32* triIndices, PxU32 triIndicesSize, bool add)
	{
		PxArray<PxgDeformablePairInfo> appended;
		appended.reserve(nbShapes * triIndicesSize * 4); // 3 verts and one tri
		for (PxU32 i = 0; i < nbShapes; ++i)
			appendRigidSurfaceFilterPairs(&surfaceMesh, surfaceId, PxgEncodeStaticShapeRef(shapeCacheIds[i]), triIndices, triIndicesSize, appended);

		update(appended, surfaceId, add);
	}

	void PxgDeformableFilterManager::updateSurfaceSurface(const Gu::TriangleMesh& surfaceMesh0,
		const PxgDeformableSurfaceTopology& topology0, PxU32 surfaceId0, const Gu::TriangleMesh& surfaceMesh1,
		const PxgDeformableSurfaceTopology& topology1, PxU32 surfaceId1, const PxU32* triIndices0,
		const PxU32* triIndices1, PxU32 triIndicesSize, bool add)
	{
		PxArray<PxgDeformablePairInfo> appended0, appended1;
		appendSurfaceSurfaceFilterPairs(&surfaceMesh0, topology0, surfaceId0, &surfaceMesh1, topology1, surfaceId1,
			triIndices0, triIndices1, triIndicesSize, appended0, appended1);

		update(appended0, surfaceId0, add);
		update(appended1, surfaceId1, add);
	}
}
