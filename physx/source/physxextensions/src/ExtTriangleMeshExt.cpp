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


#include "geometry/PxMeshQuery.h"
#include "geometry/PxGeometryQuery.h"
#include "geometry/PxTriangleMeshGeometry.h"
#include "geometry/PxHeightFieldGeometry.h"
#include "geometry/PxHeightField.h"
#include "geometry/PxTriangleMesh.h"
#include "extensions/PxTriangleMeshExt.h"
#include "GuSDF.h"
#include "GuTriangleMesh.h"

#include "foundation/PxAllocator.h"

using namespace physx;

PxMeshOverlapUtil::PxMeshOverlapUtil() : mResultsMemory(mResults), mNbResults(0), mMaxNbResults(256)
{
}

PxMeshOverlapUtil::~PxMeshOverlapUtil()
{
	if(mResultsMemory != mResults)
		PX_FREE(mResultsMemory);
}

PxU32 PxMeshOverlapUtil::findOverlap(const PxGeometry& geom, const PxTransform& geomPose, const PxTriangleMeshGeometry& meshGeom, const PxTransform& meshPose)
{
	bool overflow;
	PxU32 nbTouchedTris = PxMeshQuery::findOverlapTriangleMesh(geom, geomPose, meshGeom, meshPose, mResultsMemory, mMaxNbResults, 0, overflow);

	if(overflow)
	{
		const PxU32 maxNbTris = meshGeom.triangleMesh->getNbTriangles();
		if(!maxNbTris)
		{
			mNbResults = 0;
			return 0;
		}

		if(mMaxNbResults<maxNbTris)
		{
			if(mResultsMemory != mResults)
				PX_FREE(mResultsMemory);

			mResultsMemory = PX_ALLOCATE(PxU32, maxNbTris, "PxMeshOverlapUtil::findOverlap");
			mMaxNbResults = maxNbTris;
		}
		nbTouchedTris = PxMeshQuery::findOverlapTriangleMesh(geom, geomPose, meshGeom, meshPose, mResultsMemory, mMaxNbResults, 0, overflow);
		PX_ASSERT(nbTouchedTris);
		PX_ASSERT(!overflow);
	}
	mNbResults = nbTouchedTris;
	return nbTouchedTris;
}

PxU32 PxMeshOverlapUtil::findOverlap(const PxGeometry& geom, const PxTransform& geomPose, const PxHeightFieldGeometry& hfGeom, const PxTransform& hfPose)
{
	bool overflow = true;
	PxU32 nbTouchedTris = PxMeshQuery::findOverlapHeightField(geom, geomPose, hfGeom, hfPose, mResultsMemory, mMaxNbResults, 0, overflow);

	if(overflow)
	{
		const PxU32 maxNbTris = hfGeom.heightField->getNbRows()*hfGeom.heightField->getNbColumns()*2;
		if(!maxNbTris)
		{
			mNbResults = 0;
			return 0;
		}

		if(mMaxNbResults<maxNbTris)
		{
			if(mResultsMemory != mResults)
				PX_FREE(mResultsMemory);

			mResultsMemory = PX_ALLOCATE(PxU32, maxNbTris, "PxMeshOverlapUtil::findOverlap");
			mMaxNbResults = maxNbTris;
		}
		nbTouchedTris = PxMeshQuery::findOverlapHeightField(geom, geomPose, hfGeom, hfPose, mResultsMemory, mMaxNbResults, 0, overflow);
		PX_ASSERT(nbTouchedTris);
		PX_ASSERT(!overflow);
	}
	mNbResults = nbTouchedTris;
	return nbTouchedTris;

}
namespace
{
template<typename MeshGeometry>
bool computeMeshPenetrationT(PxVec3& direction, 
						     PxReal& depth,
							 const PxGeometry& geom, 
							 const PxTransform& geomPose, 
							 const MeshGeometry& meshGeom, 
							 const PxTransform& meshPose, 
							 PxU32 maxIter,
							 PxU32* nbIterOut)
{
	PxU32 nbIter = 0;
	PxTransform pose = geomPose;
	for (;  nbIter < maxIter; nbIter++)
	{
		PxVec3 currentDir;
		PxF32 currentDepth;

		if (!PxGeometryQuery::computePenetration(currentDir, currentDepth, geom, pose, meshGeom, meshPose))
			break;

		pose.p += currentDir * currentDepth;
	}

	if(nbIterOut)
		*nbIterOut = nbIter;

	PxVec3 diff = pose.p - geomPose.p;
	depth = diff.magnitude();	

	if (depth>0)
		direction = diff / depth;

	return nbIter!=0;
}
}

bool physx::PxComputeTriangleMeshPenetration(PxVec3& direction, 
											PxReal& depth,
											const PxGeometry& geom, 
											const PxTransform& geomPose, 
											const PxTriangleMeshGeometry& meshGeom, 
											const PxTransform& meshPose, 
											PxU32 maxIter,
											PxU32* nbIter)
{
	return computeMeshPenetrationT(direction, depth, geom, geomPose, meshGeom, meshPose, maxIter, nbIter);
}

bool physx::PxComputeHeightFieldPenetration(PxVec3& direction, 
										    PxReal& depth,
											const PxGeometry& geom, 
											const PxTransform& geomPose, 
											const PxHeightFieldGeometry& hfGeom, 
											const PxTransform& meshPose, 
											PxU32 maxIter,
											PxU32* nbIter)
{
	return computeMeshPenetrationT(direction, depth, geom, geomPose, hfGeom, meshPose, maxIter, nbIter);
}

bool physx::PxExtractIsosurfaceFromSDF(const PxTriangleMesh& triangleMesh, PxArray<PxVec3>& isosurfaceVertices, PxArray<PxU32>& isosurfaceTriangleIndices)
{
	PxU32 dimX, dimY, dimZ;
	triangleMesh.getSDFDimensions(dimX, dimY, dimZ);
	if (dimX == 0 || dimY == 0 || dimZ == 0)
		return false;

	const Gu::TriangleMesh* guTriangleMesh = static_cast<const Gu::TriangleMesh*>(&triangleMesh);
	const Gu::SDF& sdf = guTriangleMesh->getSdfDataFast();	

	extractIsosurfaceFromSDF(sdf, isosurfaceVertices, isosurfaceTriangleIndices);

	return true;
}
