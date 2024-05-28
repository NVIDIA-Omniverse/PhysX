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

#include "geometry/PxMeshScale.h"

#include "GuHeightFieldUtil.h"
#include "GuSweepSharedTests.h"
#include "GuHeightField.h"
#include "GuEntityReport.h"
#include "foundation/PxIntrinsics.h"
#include "CmScaling.h"

using namespace physx;

void Gu::HeightFieldUtil::computeLocalBounds(PxBounds3& bounds) const
{
	const PxMeshScale scale(PxVec3(mHfGeom->rowScale, mHfGeom->heightScale, mHfGeom->columnScale), PxQuat(PxIdentity));
	const PxMat33 mat33 = Cm::toMat33(scale);

	bounds.minimum = mat33.transform(mHeightField->getData().mAABB.getMin());
	bounds.maximum = mat33.transform(mHeightField->getData().mAABB.getMax());

	// PT: HFs will assert in Gu::intersectRayAABB2() if we don't deal with that
	const float deltaY = GU_MIN_AABB_EXTENT*0.5f - (bounds.maximum.y - bounds.minimum.y);
	if(deltaY>0.0f)
	{
		bounds.maximum.y += deltaY*0.6f;
		bounds.minimum.y -= deltaY*0.6f;
	}
}

static PX_FORCE_INLINE bool reportTriangle(Gu::OverlapReport& callback, PxU32 material, PxU32* PX_RESTRICT indexBuffer, const PxU32 bufferSize, PxU32& indexBufferUsed, PxU32 triangleIndex)
{
	if(material != PxHeightFieldMaterial::eHOLE) 
	{
		indexBuffer[indexBufferUsed++] = triangleIndex;

		if(indexBufferUsed >= bufferSize)
		{
			if(!callback.reportTouchedTris(indexBufferUsed, indexBuffer))
				return false;
			indexBufferUsed = 0;
		}
	}
	return true;
}

void Gu::HeightFieldUtil::overlapAABBTriangles(const PxBounds3& bounds, OverlapReport& callback, PxU32 batchSize) const
{
	PX_ASSERT(batchSize<=HF_OVERLAP_REPORT_BUFFER_SIZE);
	PX_ASSERT(!bounds.isEmpty());

	PxBounds3 localBounds = bounds;

	localBounds.minimum.x *= mOneOverRowScale;
	localBounds.minimum.y *= mOneOverHeightScale;
	localBounds.minimum.z *= mOneOverColumnScale;

	localBounds.maximum.x *= mOneOverRowScale;
	localBounds.maximum.y *= mOneOverHeightScale;
	localBounds.maximum.z *= mOneOverColumnScale;

	if(mHfGeom->rowScale < 0.0f)
		PxSwap(localBounds.minimum.x, localBounds.maximum.x);

	if(mHfGeom->columnScale < 0.0f)
		PxSwap(localBounds.minimum.z, localBounds.maximum.z);

	// early exit for aabb does not overlap in XZ plane
	// DO NOT MOVE: since rowScale / columnScale may be negative this has to be done after scaling localBounds
	const PxU32	nbRows = mHeightField->getNbRowsFast();
	const PxU32	nbColumns = mHeightField->getNbColumnsFast();
	if(localBounds.minimum.x > float(nbRows - 1))
		return;
	if(localBounds.minimum.z > float(nbColumns - 1))
		return;
	if(localBounds.maximum.x < 0.0f)
		return;
	if(localBounds.maximum.z < 0.0f)
		return;

	const PxU32 minRow = mHeightField->getMinRow(localBounds.minimum.x);
	const PxU32 maxRow = mHeightField->getMaxRow(localBounds.maximum.x);
	const PxU32 minColumn = mHeightField->getMinColumn(localBounds.minimum.z);
	const PxU32 maxColumn = mHeightField->getMaxColumn(localBounds.maximum.z);
	const PxU32 deltaColumn = maxColumn - minColumn;

	const PxU32 maxNbTriangles = 2 * deltaColumn * (maxRow - minRow);
	if(!maxNbTriangles)
		return;

	const PxU32 bufferSize = batchSize<=HF_OVERLAP_REPORT_BUFFER_SIZE ? batchSize : HF_OVERLAP_REPORT_BUFFER_SIZE;
	PxU32 indexBuffer[HF_OVERLAP_REPORT_BUFFER_SIZE];
	PxU32 indexBufferUsed = 0;

	PxU32 offset = minRow * nbColumns + minColumn;

	const PxReal miny = localBounds.minimum.y;
	const PxReal maxy = localBounds.maximum.y;
	const PxU32 columnStride = nbColumns - deltaColumn;

	for(PxU32 row=minRow; row<maxRow; row++)
	{
		for(PxU32 column=minColumn; column<maxColumn; column++)
		{
			const PxReal h0 = mHeightField->getHeight(offset);
			const PxReal h1 = mHeightField->getHeight(offset + 1);
			const PxReal h2 = mHeightField->getHeight(offset + nbColumns);
			const PxReal h3 = mHeightField->getHeight(offset + nbColumns + 1);

			const bool bmax = maxy < h0 && maxy < h1 && maxy < h2 && maxy < h3;
			const bool bmin = miny > h0 && miny > h1 && miny > h2 && miny > h3;

			if(!(bmax || bmin))
			{
				if(!reportTriangle(callback, mHeightField->getMaterialIndex0(offset), indexBuffer, bufferSize, indexBufferUsed, offset << 1))
					return;

				if(!reportTriangle(callback, mHeightField->getMaterialIndex1(offset), indexBuffer, bufferSize, indexBufferUsed, (offset << 1) + 1))
					return;
			}
			offset++;
		}
		offset += columnStride;
	}

	if(indexBufferUsed > 0)
		callback.reportTouchedTris(indexBufferUsed, indexBuffer);
}

PxU32 Gu::HeightFieldUtil::getTriangle(const PxTransform& pose, PxTriangle& worldTri,
									   PxU32* _vertexIndices, PxU32* adjacencyIndices, PxTriangleID triangleIndex, bool worldSpaceTranslation, bool worldSpaceRotation) const
{
#if PX_CHECKED
	if (!mHeightField->isValidTriangle(triangleIndex)) 
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "HeightFieldShape::getTriangle: Invalid triangle index!");
		return 0;
	}
#endif

	PxVec3 handedness(1.0f);	// Vector to invert normal coordinates according to the heightfield scales
	bool wrongHanded = false;
	if (mHfGeom->columnScale < 0)
	{
		wrongHanded = !wrongHanded;
		handedness.z = -1.0f;
	}
	if (mHfGeom->rowScale < 0)
	{
		wrongHanded = !wrongHanded;
		handedness.x = -1.0f;
	}

/*	if (0) // ptchernev: Iterating over triangles becomes a pain.
	{
		if (mHeightField.getTriangleMaterial(triangleIndex) == mHfGeom.holeMaterialIndex)
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "HeightFieldShape::getTriangle: Non-existing triangle (triangle has hole material)!");
			return 0;
		}
	}*/

	PxU32 vertexIndices[3];
	mHeightField->getTriangleVertexIndices(triangleIndex, vertexIndices[0], vertexIndices[1+wrongHanded], vertexIndices[2-wrongHanded]);

	if(adjacencyIndices)
	{
		mHeightField->getTriangleAdjacencyIndices(	triangleIndex, vertexIndices[0], vertexIndices[1+wrongHanded], vertexIndices[2-wrongHanded],
													adjacencyIndices[wrongHanded ? 2 : 0], adjacencyIndices[1], adjacencyIndices[wrongHanded ? 0 : 2]);
	}

	if(_vertexIndices)
	{
		_vertexIndices[0] = vertexIndices[0];
		_vertexIndices[1] = vertexIndices[1];
		_vertexIndices[2] = vertexIndices[2];
	}

	if (worldSpaceRotation)
	{
		if (worldSpaceTranslation)
		{
			for (PxU32 vi = 0; vi < 3; vi++)
				worldTri.verts[vi] = hf2worldp(pose, mHeightField->getVertex(vertexIndices[vi]));
		}
		else
		{
			for (PxU32 vi = 0; vi < 3; vi++)
			{
				// TTP 2390 
				// local space here is rotated (but not translated) world space
				worldTri.verts[vi] = pose.q.rotate(hf2shapep(mHeightField->getVertex(vertexIndices[vi])));
			}
		}
	}
	else
	{
		const PxVec3 offset = worldSpaceTranslation ? pose.p : PxVec3(0.0f);
		for (PxU32 vi = 0; vi < 3; vi++)
			worldTri.verts[vi] = hf2shapep(mHeightField->getVertex(vertexIndices[vi])) + offset;
	}
	return PxU32(mHeightField->getTriangleMaterial(triangleIndex) != PxHeightFieldMaterial::eHOLE);
}
