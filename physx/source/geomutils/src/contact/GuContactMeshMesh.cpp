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

#include "foundation/PxArray.h"
#include "foundation/PxAssert.h"
#include "foundation/PxBounds3.h"
#include "foundation/PxMat33.h"
#include "foundation/PxMath.h"
#include "foundation/PxPreprocessor.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "foundation/PxVec3.h"
#include "geometry/PxBoxGeometry.h"
#include "geometry/PxMeshQuery.h"
#include "geometry/PxTriangleMesh.h"
#include "GuCollisionSDF.h"
#include "GuTriangleMesh.h"
#include "GuContactMeshMesh.h"
#include "GuContactMethodImpl.h"
#include "GuContactReduction.h"
#include "GuMidphaseInterface.h"
#include "GuTriangle.h"
#include "GuTriangleRefinement.h"

using namespace physx;
using namespace Gu;

using ContactReduction = SDFContactReduction<5, 10000, 32>;

const PxI32 maxRefinementLevel = 8;

struct TransformedTriangle
{
	PxVec3 v0, v1, v2;
	PxI16 refinementLevel;
	PxU8 boundary; // information about boundaries; currently unused
};

PX_FORCE_INLINE bool needsRefinement(const PxReal triRefThreshold, const TransformedTriangle& tri)
{
	return (
			(tri.v0-tri.v1).magnitudeSquared() > triRefThreshold ||
			(tri.v1-tri.v2).magnitudeSquared() > triRefThreshold ||
			(tri.v2-tri.v0).magnitudeSquared() > triRefThreshold
			) && tri.refinementLevel < maxRefinementLevel;
}

// Find contacts between an SDF and a triangle mesh and return the number of contacts generated
inline PxU32 sdfMeshCollision (
		const PxTransform32& PX_RESTRICT tfSdf, const PxTriangleMeshGeometry& PX_RESTRICT sdfGeom,
		const PxTransform32& PX_RESTRICT tfMesh, const PxTriangleMeshGeometry& PX_RESTRICT meshGeom,
		ContactReduction& contactReducer, const PxReal totalContactDistance, bool flipContactNormals
		)
{
	float min_separation = PX_MAX_REAL;
	const TriangleMesh& mesh = static_cast<const TriangleMesh&>(*meshGeom.triangleMesh);
	const TriangleMesh& sdfMesh = static_cast<const TriangleMesh&>(*sdfGeom.triangleMesh);

	const PxMeshScale& sdfScale = sdfGeom.scale, & meshScale = meshGeom.scale;

	const CollisionSDF& PX_RESTRICT sdf(sdfMesh.mSdfData);

	const PxTransform meshToSdf = tfSdf.transformInv(tfMesh);

	const PxMat33 sdfScaleMat = sdfScale.toMat33();
	PxBounds3 sdfBoundsAtWorldScale(sdfScaleMat.transform(sdf.mSdfBoxLower), sdfScaleMat.transform(sdf.mSdfBoxUpper));
	sdfBoundsAtWorldScale.fattenSafe(totalContactDistance);
	const PxTransform poseT(sdfBoundsAtWorldScale.getCenter());
	const PxBoxGeometry boxGeom(sdfBoundsAtWorldScale.getExtents());

	const PxReal sdfDiagSq = (sdf.mSdfBoxUpper - sdf.mSdfBoxLower).magnitudeSquared();
	const PxReal div = 1.0f / 256.0f;
	const PxReal triRefThreshold = sdfDiagSq * div;
	const bool singleSdf = meshGeom.triangleMesh->getSDF() == NULL;  // triangle subdivision if single SDF

	const PxU32 MAX_INTERSECTIONS = 1024 * 32;
	PxArray<PxU32> overlappingTriangles;
	overlappingTriangles.resize(MAX_INTERSECTIONS); //TODO: Not ideal, dynamic allocation for every function call
	//PxU32 overlappingTriangles[MAX_INTERSECTIONS]; //TODO: Is this too much memory to allocate on the stack?

	bool overflow = false;
	const PxU32 overlapCount = PxMeshQuery::findOverlapTriangleMesh(boxGeom, poseT, meshGeom, meshToSdf, overlappingTriangles.begin(), MAX_INTERSECTIONS, 0, overflow);
	PX_ASSERT(!overflow);

	// we use cullScale to account for SDF scaling whenever distances are
	const PxReal cullScale = totalContactDistance / sdfScale.scale.minElement();

	const PxVec3* PX_RESTRICT vertices = mesh.getVertices();
	const void* PX_RESTRICT tris = mesh.getTriangles();
	const bool has16BitIndices = mesh.getTriangleMeshFlags() & physx::PxTriangleMeshFlag::e16_BIT_INDICES;
	const PxU32 nbTris = overlapCount; // mesh.getNbTriangles();


	/* Transforms fused; unoptimized version:
	   v0 = shape2Vertex(
	   meshToSdf.transform(vertex2Shape(vertices[triIndices.mRef[0]], meshScale.scale, meshScale.rotation)),
	   sdfScale.scale, sdfScale.rotation); */
	const PxMat33 sdfScaleIMat = sdfScale.getInverse().toMat33();
	const PxMat33 fusedRotScale = sdfScaleIMat * PxMat33Padded(meshToSdf.q) * meshScale.toMat33();
	const PxVec3 fusedTranslate = sdfScaleIMat * meshToSdf.p;

	const PxMat33Padded tfSdfRotationMatrix(tfSdf.q);
	const PxMat33 pointToWorldR = tfSdfRotationMatrix * sdfScale.toMat33();
	const PxMat33 normalToWorld = tfSdfRotationMatrix * sdfScaleIMat;


	const PxU32 COLLISION_BUF_SIZE = 512;
	const PxU32 sudivBufSize = singleSdf ? maxRefinementLevel * 3 : 0;  // Overhead for subdivision (pop one, push four)
	PX_ASSERT(sudivBufSize < COLLISION_BUF_SIZE/4); // ensure reasonable buffer size


	TransformedTriangle goodTriangles[COLLISION_BUF_SIZE];

	PxU32 nbContacts = 0;
	for (PxU32 i = 0, allTrisProcessed = 0; !allTrisProcessed;)
	{
		// try to find `COLLISION_BUF_SIZE` triangles that cannot be culled immediately

		PxU32 nbGoodTris = 0;
		// Every triangle that overlaps with the sdf's axis aligned bounding box is checked against the sdf to see if an intersection
		// can be ruled out. If an intersection can be ruled out, the triangle is not further processed. Since SDF data is accessed, 
		// the check is more accurate (but still very fast) than a simple bounding box overlap test.
		// Performance measurements confirm that this pre-pruning loop actually increases performance significantly on some scenes
		for ( ; nbGoodTris < COLLISION_BUF_SIZE - sudivBufSize; ++i)
		{
			if (i == nbTris)
			{
				allTrisProcessed = true;
				break;
			}
			const PxU32 triIdx = overlappingTriangles[i];
			TransformedTriangle niceTri;

			const Gu::IndexedTriangle32 triIndices = has16BitIndices ?
				getTriangleVertexIndices<PxU16>(tris, triIdx) :
				getTriangleVertexIndices<PxU32>(tris, triIdx);

			niceTri.v0 = fusedTranslate + fusedRotScale * vertices[triIndices.mRef[0]];
			niceTri.v1 = fusedTranslate + fusedRotScale * vertices[triIndices.mRef[1]];
			niceTri.v2 = fusedTranslate + fusedRotScale * vertices[triIndices.mRef[2]];

			if (singleSdf)
				niceTri.refinementLevel = 0;

			// - triangles that are not culled are added to goodTriangles
			if (sdfTriangleSphericalCull(sdf, niceTri.v0, niceTri.v1, niceTri.v2, cullScale))
				goodTriangles[nbGoodTris++] = niceTri;
		}

		//  in promising triangles
		// - triangles are popped from goodTriangles and a contact generated or,
		//   if subdivision is indicated, their children are pushed on top
		for (PxU32 goodTriEnd = nbGoodTris; goodTriEnd > 0;)
		{
			const TransformedTriangle tri = goodTriangles[--goodTriEnd];  // pop
			// decide on need for subdivision
			if (singleSdf && needsRefinement(triRefThreshold, tri))
			{
				for (int childIdx = 0; childIdx < 4; ++childIdx)
				{
					TransformedTriangle child = tri;
					Gu::getSubTriangle4(childIdx, child.v0, child.v1, child.v2);
					++child.refinementLevel;
					if (sdfTriangleSphericalCull(sdf, child.v0, child.v1, child.v2, cullScale))
						goodTriangles[goodTriEnd++] = child;
				}
				continue;
			}

			// generate contacts
			PxVec3 sdfPoint, contactDir;
			PxReal separation = sdfTriangleCollision(sdf, tri.v0, tri.v1, tri.v2, sdfPoint, contactDir, cullScale);
			min_separation = PxMin(min_separation, separation);

			if (separation < cullScale)
			{
				const PxVec3 worldPoint = tfSdf.p + pointToWorldR * sdfPoint;
				contactDir = normalToWorld * contactDir;

				const PxReal magSq = contactDir.magnitudeSquared();

				// TODO(CA): handle case where only one mesh has an SDF and update heuristic once this is done
				if (magSq < 1e-6f) // ignore contacts with a bad/missing normal
					continue;
				const PxReal mag = PxRecipSqrt(magSq);

				if (singleSdf && tri.refinementLevel)
				{
					const PxVec3 n = (tri.v1 - tri.v0).getNormalized().cross(tri.v2 - tri.v0).getNormalized();
					const PxVec3 sdfBoxCenter = 0.5f * (sdf.mSdfBoxUpper + sdf.mSdfBoxLower);
					const PxReal triangleNormalSign = -PxSign((sdfBoxCenter - tri.v0).dot(n));
					contactDir = normalToWorld * triangleNormalSign * n;
					contactDir.normalize();
					contactDir /= mag ;
				}
				separation *= mag;
				contactDir *= mag;
				const TinyContact contact{flipContactNormals ? -contactDir : contactDir, separation, worldPoint};
				contactReducer.addContact(contact);
				++nbContacts;
			}
		}
	}
	return nbContacts;
};


bool Gu::contactMeshMesh(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);

	// Get meshes
	const PxTriangleMeshGeometry& geom0 = checkedCast<PxTriangleMeshGeometry>(shape0),
								& geom1 = checkedCast<PxTriangleMeshGeometry>(shape1);
	PX_ASSERT(geom0.triangleMesh != NULL && geom1.triangleMesh != NULL);


	PxU32 nbContacts = 0;
	const PxReal contactDistance = params.mContactDistance; // computed in `checkContactsMustBeGenerated`

	const bool mesh0PreferProj = static_cast<const TriangleMesh&>(*geom0.triangleMesh).getPreferSDFProjection(),
			   mesh1PreferProj = static_cast<const TriangleMesh&>(*geom1.triangleMesh).getPreferSDFProjection();
	const PxU32 n0 = geom0.triangleMesh->getNbVertices(), n1 = geom1.triangleMesh->getNbVertices();

	// sdf0first: in first pass, treat mesh0 as an sdf and mesh1 as a mesh
	const bool sdf0first = ((!mesh0PreferProj && mesh1PreferProj) || (mesh0PreferProj == mesh1PreferProj && n1 < n0));

	ContactReduction contactReducer;

	const bool geom0HasSdf = geom0.triangleMesh->getSDF() != NULL,
			   geom1HasSdf = geom1.triangleMesh->getSDF() != NULL;
	if (!(geom0HasSdf && geom1HasSdf))
	{
		PX_ASSERT(geom0HasSdf || geom1HasSdf); // require at least one SDF
		if (geom0HasSdf)
			nbContacts += sdfMeshCollision(transform0, geom0, transform1, geom1, contactReducer, contactDistance, true);
		else
			nbContacts += sdfMeshCollision(transform1, geom1, transform0, geom0, contactReducer, contactDistance, false);
	}
	else if (sdf0first)
	{
		nbContacts += sdfMeshCollision(transform0, geom0, transform1, geom1, contactReducer, contactDistance, true);
		nbContacts += sdfMeshCollision(transform1, geom1, transform0, geom0, contactReducer, contactDistance, false);
	}
	else
	{
		nbContacts += sdfMeshCollision(transform1, geom1, transform0, geom0, contactReducer, contactDistance, false);
		nbContacts += sdfMeshCollision(transform0, geom0, transform1, geom1, contactReducer, contactDistance, true);
	}

	contactReducer.flushToContactBuffer(contactBuffer);
	return nbContacts != 0;
}
