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

#include "NpDebugViz.h"

// PT: moving "all" debug viz code to the same file to improve cache locality when debug drawing things,
// share more code, and make sure all actors do thing consistently.

#include "NpScene.h"
#include "NpCheck.h"
#include "common/PxProfileZone.h"

using namespace physx;

#if PX_ENABLE_DEBUG_VISUALIZATION

#include "NpShapeManager.h"
#include "NpRigidStatic.h"
#include "NpRigidDynamic.h"
#include "NpArticulationReducedCoordinate.h"
#if PX_SUPPORT_GPU_PHYSX
	#include "NpDeformableVolume.h"
	#include "NpPBDParticleSystem.h"
#endif
#include "foundation/PxVecMath.h"
#include "geometry/PxMeshQuery.h"
#include "GuHeightFieldUtil.h"
#include "GuConvexEdgeFlags.h"
#include "GuMidphaseInterface.h"
#include "GuEdgeList.h"
#include "GuBounds.h"
#include "BpBroadPhase.h"
#include "BpAABBManager.h"
#include "GuConvexGeometry.h"

using namespace aos;
using namespace Gu;
using namespace Cm;

/////

static const PxU32 gCollisionShapeColor = PxU32(PxDebugColor::eARGB_MAGENTA);

static PX_FORCE_INLINE Vec4V multiply3x3V(const Vec4V p, const PxMat34& mat)
{
	Vec4V ResV = V4Scale(V4LoadU(&mat.m.column0.x), V4GetX(p));
	ResV = V4Add(ResV, V4Scale(V4LoadU(&mat.m.column1.x), V4GetY(p)));
	ResV = V4Add(ResV, V4Scale(V4LoadU(&mat.m.column2.x), V4GetZ(p)));
	return ResV;
}

// PT: beware, needs padding at the end of dst/src
static PX_FORCE_INLINE void transformV(PxVec3* dst, const PxVec3* src, const Vec4V p, const PxMat34& mat)
{
	const Vec4V vertexV = V4LoadU(&src->x);
	const Vec4V transformedV = V4Add(multiply3x3V(vertexV, mat), p);
	V4StoreU(transformedV, &dst->x);
}

static void visualizeSphere(const PxSphereGeometry& geometry, PxRenderOutput& out, const PxTransform& absPose)
{
	out << gCollisionShapeColor;	// PT: no need to output this for each segment!

	out << absPose;
	renderOutputDebugCircle(out, 100, geometry.radius);

	PxMat44 rotPose(absPose);
	PxSwap(rotPose.column1, rotPose.column2);
	rotPose.column1 = -rotPose.column1;
	out << rotPose;
	renderOutputDebugCircle(out, 100, geometry.radius);

	PxSwap(rotPose.column0, rotPose.column2);
	rotPose.column0 = -rotPose.column0;
	out << rotPose;
	renderOutputDebugCircle(out, 100, geometry.radius);
}

static void visualizePlane(const PxPlaneGeometry& /*geometry*/, PxRenderOutput& out, const PxTransform& absPose)
{
	PxMat44 rotPose(absPose);
	PxSwap(rotPose.column1, rotPose.column2);
	rotPose.column1 = -rotPose.column1;

	PxSwap(rotPose.column0, rotPose.column2);
	rotPose.column0 = -rotPose.column0;

	out << rotPose << gCollisionShapeColor;	// PT: no need to output this for each segment!
	for(PxReal radius = 2.0f; radius < 20.0f ; radius += 2.0f)
		renderOutputDebugCircle(out, 100, radius*radius);
}

static void visualizeCapsule(const PxCapsuleGeometry& geometry, PxRenderOutput& out, const PxTransform& absPose)
{
	out << gCollisionShapeColor;
	out.outputCapsule(geometry.radius, geometry.halfHeight, absPose);
}

static void visualizeBox(const PxBoxGeometry& geometry, PxRenderOutput& out, const PxTransform& absPose)
{
	out << gCollisionShapeColor;
	out << absPose;
	renderOutputDebugBox(out, PxBounds3(-geometry.halfExtents, geometry.halfExtents));
}

static void visualizeConvexCore(const PxConvexCoreGeometry& geometry, PxRenderOutput& out, const PxTransform& absPose, const PxBounds3& cullbox)
{
	Gu::visualize(geometry, absPose, true, cullbox, out);
}

static void visualizeConvexMesh(const PxConvexMeshGeometry& geometry, PxRenderOutput& out, const PxTransform& absPose)
{
	const ConvexMesh* convexMesh = static_cast<const ConvexMesh*>(geometry.convexMesh);
	const ConvexHullData& hullData = convexMesh->getHull();

	const PxVec3* vertices = hullData.getHullVertices();
	const PxU8* indexBuffer = hullData.getVertexData8();
	const PxU32 nbPolygons = convexMesh->getNbPolygonsFast();

	const PxMat33Padded m33(absPose.q);

	const PxMat44 m44(m33 * toMat33(geometry.scale), absPose.p);

	out << m44 << gCollisionShapeColor;	// PT: no need to output this for each segment!

	for(PxU32 i=0; i<nbPolygons; i++)
	{
		const PxU32 pnbVertices = hullData.mPolygons[i].mNbVerts;

		PxVec3 begin = m44.transform(vertices[indexBuffer[0]]);	// PT: transform it only once before the loop starts
		for(PxU32 j=1; j<pnbVertices; j++)
		{
			const PxVec3 end = m44.transform(vertices[indexBuffer[j]]);
			out.outputSegment(begin, end);
			begin = end;
		}
		out.outputSegment(begin, m44.transform(vertices[indexBuffer[0]]));

		indexBuffer += pnbVertices;
	}
}

static void getTriangle(PxU32 i, PxVec3* wp, const PxVec3* vertices, const void* indices, bool has16BitIndices)
{
	PxU32 ref0, ref1, ref2;
	getVertexRefs(i, ref0, ref1, ref2, indices, has16BitIndices);

	wp[0] = vertices[ref0];
	wp[1] = vertices[ref1];
	wp[2] = vertices[ref2];
}

// PT: beware with wp, needs padding
static void getWorldTriangle(PxU32 i, PxVec3* wp, const PxVec3* vertices, const void* indices, const PxMat34& absPose, bool has16BitIndices)
{
//	PxVec3 localVerts[3];
//	getTriangle(i, localVerts, vertices, indices, has16BitIndices);

//	wp[0] = absPose.transform(localVerts[0]);
//	wp[1] = absPose.transform(localVerts[1]);
//	wp[2] = absPose.transform(localVerts[2]);

	PxU32 ref0, ref1, ref2;
	getVertexRefs(i, ref0, ref1, ref2, indices, has16BitIndices);

	const Vec4V posV = Vec4V_From_Vec3V(V3LoadU(&absPose.p.x));

	transformV(&wp[0], &vertices[ref0], posV, absPose);
	transformV(&wp[1], &vertices[ref1], posV, absPose);
	transformV(&wp[2], &vertices[ref2], posV, absPose);
}

static void visualizeActiveEdges(PxRenderOutput& out, const TriangleMesh& mesh, PxU32 nbTriangles, const PxU32* results, const PxMat34& absPose)
{
	const PxU8* extraTrigData = mesh.getExtraTrigData();

	const PxVec3* vertices = mesh.getVerticesFast();
	const void* indices = mesh.getTrianglesFast();

	out << PxU32(PxDebugColor::eARGB_YELLOW);	// PT: no need to output this for each segment!

	const bool has16Bit = mesh.has16BitIndices();
	for(PxU32 i=0; i<nbTriangles; i++)
	{
		const PxU32 index = results ? results[i] : i;

		PxVec3 wp[3+1];
		getWorldTriangle(index, wp, vertices, indices, absPose, has16Bit);

		const PxU32 flags = getConvexEdgeFlags(extraTrigData, index);

		if(flags & ETD_CONVEX_EDGE_01)
			out.outputSegment(wp[0], wp[1]);

		if(flags & ETD_CONVEX_EDGE_12)
			out.outputSegment(wp[1], wp[2]);

		if(flags & ETD_CONVEX_EDGE_20)
			out.outputSegment(wp[0], wp[2]);
	}
}

static void visualizeFaceNormals(	PxReal fscale, PxRenderOutput& out, PxU32 nbTriangles, const PxVec3* vertices,
									const void* indices, bool has16Bit, const PxU32* results, const PxMat34& absPose, const PxMat44& midt)
{
	out << midt << PxU32(PxDebugColor::eARGB_DARKRED);	// PT: no need to output this for each segment!

	const float coeff = 1.0f / 3.0f;
	PxDebugLine* segments = out.reserveSegments(nbTriangles);
	for(PxU32 i=0; i<nbTriangles; i++)
	{
		const PxU32 index = results ? results[i] : i;
		PxVec3 wp[3+1];
		getWorldTriangle(index, wp, vertices, indices, absPose, has16Bit);

		const PxVec3 center = (wp[0] + wp[1] + wp[2]) * coeff;

		PxVec3 normal = (wp[0] - wp[1]).cross(wp[0] - wp[2]);
		PX_ASSERT(!normal.isZero());
		normal = normal.getNormalized();

		segments->pos0 = center;
		segments->pos1 = center + normal * fscale;
		segments->color0 = segments->color1 = PxU32(PxDebugColor::eARGB_DARKRED);
		segments++;
	}
}

static PxU32 MakeSolidColor(PxU32 alpha, PxU32 red, PxU32 green, PxU32 blue)
{
	return (alpha<<24) | (red << 16) |
		(green << 8) | blue;
}

static void decodeTriple(PxU32 id, PxU32& x, PxU32& y, PxU32& z)
{
	x = id & 0x000003FF;
	id = id >> 10;
	y = id & 0x000003FF;
	id = id >> 10;
	z = id & 0x000003FF;
}

PX_FORCE_INLINE PxU32 idx(PxU32 x, PxU32 y, PxU32 z, PxU32 width, PxU32 height)
{
	return z * (width) * (height) + y * width + x;
}

PX_FORCE_INLINE PxReal decode(PxU8* data, PxU32 bytesPerSparsePixel, PxReal subgridsMinSdfValue, PxReal subgridsMaxSdfValue)
{
	switch (bytesPerSparsePixel)
	{
	case 1:
		return PxReal(data[0]) * (1.0f / 255.0f) * (subgridsMaxSdfValue - subgridsMinSdfValue) + subgridsMinSdfValue;
	case 2:
	{
		PxU16* ptr = reinterpret_cast<PxU16*>(data);
		return PxReal(ptr[0]) * (1.0f / 65535.0f) * (subgridsMaxSdfValue - subgridsMinSdfValue) + subgridsMinSdfValue;
	}
	case 4:
		//If 4 bytes per subgrid pixel are available, then normal floats are used. No need to 
		//de-normalize integer values since the floats already contain real distance values
		PxReal* ptr = reinterpret_cast<PxReal*>(data);
		return ptr[0];
	}
	return 0;
}

PX_FORCE_INLINE PxU32 makeColor(PxReal v, PxReal invRange0, PxReal invRange1)
{
	PxVec3 midColor(0.f, 0, 255.f);
	PxVec3 lowColor(255.f, 0, 0);
	PxVec3 outColor(0, 255.f, 0.f);

	PxU32 color;
	if (v > 0.f)
	{
		PxReal scale = PxPow(v * invRange0, 0.25f);

		PxVec3 blendColor = midColor + (outColor - midColor) * scale;

		color = MakeSolidColor(
			0xff000000,
			PxU32(blendColor.x),
			PxU32(blendColor.y),
			PxU32(blendColor.z)
		);
	}
	else
	{
		PxReal scale = PxPow(v * invRange1, 0.25f);

		PxVec3 blendColor = midColor + (lowColor - midColor) * scale;

		color = MakeSolidColor(
			0xff000000,
			PxU32(blendColor.x),
			PxU32(blendColor.y),
			PxU32(blendColor.z)
		);
	}
	return color;
}

//Returns true if the number of samples chosen to visualize was reduced (to speed up rendering) compared to the total number of sdf samples available
static bool visualizeSDF(PxRenderOutput& out, const Gu::SDF& sdf, const PxMat34& absPose, bool limitNumberOfVisualizedSamples = false)
{
	//### DEFENSIVE coding for OM-122453 and OM-112365
	if (!sdf.mSdf)
	{
		PxGetFoundation().error(::physx::PxErrorCode::eINTERNAL_ERROR, PX_FL, "No distance data available (null pointer) when evaluating an SDF.");
		return false;
	}

	bool dataReductionActive = false;

	PxU32 upperByteLimit = 512u * 512u * 512u * sizeof(PxDebugPoint); //2GB - sizeof(PxDebugPoint)=16bytes

	bool repeat = true;	

	PxReal low = 0.0f, high = 0.0f;
	PxU32 count = 0;
	PxU32 upperLimit = limitNumberOfVisualizedSamples ? 128 : 4096;
	PxReal sdfSpacing = 0.0f;
	PxU32 strideX = 0;
	PxU32 strideY = 0;
	PxU32 strideZ = 0;
	PxU32 nbX = 0, nbY = 0, nbZ = 0;
	PxU32 subgridSize = 0;
	PxU32 subgridStride = 1;

	while (repeat)
	{
		repeat = false;
		
		PxU32 nbTargetSamplesPerAxis = upperLimit;
		subgridStride = 1;
		if (sdf.mSubgridSize == 0)
		{
			subgridSize = 1;
			sdfSpacing = sdf.mSpacing;
			nbX = sdf.mDims.x;
			nbY = sdf.mDims.y;
			nbZ = sdf.mDims.z;
		}
		else
		{
			//### DEFENSIVE coding for OM-122453 and OM-112365
			if (!sdf.mSubgridSdf || !sdf.mSubgridStartSlots)
			{
				PxGetFoundation().error(::physx::PxErrorCode::eINTERNAL_ERROR, PX_FL, "No sparse grid data available (null pointer) when evaluating an SDF.");
				return false;
			}

			subgridSize = sdf.mSubgridSize;
			sdfSpacing = subgridSize * sdf.mSpacing;
			nbX = sdf.mDims.x / subgridSize + 1;
			nbY = sdf.mDims.y / subgridSize + 1;
			nbZ = sdf.mDims.z / subgridSize + 1;
			//Limit the max number of visualized sparse grid samples
			if (limitNumberOfVisualizedSamples && subgridSize > 4)
			{
				subgridStride = (subgridSize + 3) / 4;
				dataReductionActive = true;
			}
		}

		//KS - a bit arbitrary, but let's limit how many points we churn out

		strideX = (nbX + nbTargetSamplesPerAxis - 1) / nbTargetSamplesPerAxis;
		strideY = (nbY + nbTargetSamplesPerAxis - 1) / nbTargetSamplesPerAxis;
		strideZ = (nbZ + nbTargetSamplesPerAxis - 1) / nbTargetSamplesPerAxis;
		if (strideX != 1 || strideY != 1 || strideZ != 1)
			dataReductionActive = true;

		low = PX_MAX_F32;
		high = -PX_MAX_F32;

		count = 0;



		for (PxU32 k = 0; k < nbZ; k += strideZ) 
		{
			for (PxU32 j = 0; j < nbY; j += strideY)
			{
				for (PxU32 i = 0; i < nbX; i += strideX)
				{
					PxReal v = sdf.mSdf[k*nbX*nbY + j * nbX + i];
					count++;

					low = PxMin(low, v);
					high = PxMax(high, v);

					if (sdf.mSubgridSize > 0 && k < nbZ - 1 && j < nbY - 1 && i < nbX - 1)
					{
						PxU32 startId = sdf.mSubgridStartSlots[k*(nbX - 1)*(nbY - 1) + j * (nbX - 1) + i];
						if (startId != 0xFFFFFFFFu)
						{
							PxU32 xBase, yBase, zBase;
							decodeTriple(startId, xBase, yBase, zBase);

							PX_ASSERT(xBase < sdf.mSdfSubgrids3DTexBlockDim.x);
							PX_ASSERT(yBase < sdf.mSdfSubgrids3DTexBlockDim.y);
							PX_ASSERT(zBase < sdf.mSdfSubgrids3DTexBlockDim.z);

							PxU32 localCount = subgridSize / subgridStride + 1;
							count += localCount * localCount * localCount;
						}
					}

					if (count * sizeof(PxDebugPoint) > upperByteLimit)
						break;
				}
				if (count * sizeof(PxDebugPoint) > upperByteLimit)
					break;
			}
			if (count * sizeof(PxDebugPoint) > upperByteLimit)
				break;
		}

		if (count * sizeof(PxDebugPoint) > upperByteLimit)
		{
			upperLimit /= 2;
			repeat = true;

			if (upperLimit == 1)
				return true;
		}
	}

	const PxReal range0 = high;
	const PxReal range1 = low;

	const PxReal invRange0 = 1.f / range0;
	const PxReal invRange1 = 1.f / range1;

	PxDebugPoint* points = out.reservePoints(count);

	PxVec3 localPos = sdf.mMeshLower;

	PxReal spacingX = sdfSpacing * strideX;
	PxReal spacingY = sdfSpacing * strideY;
	PxReal spacingZ = sdfSpacing * strideZ;
	
	for (PxU32 k = 0; k < nbZ; k += strideZ, localPos.z += spacingZ)
	{
		localPos.y = sdf.mMeshLower.y;
		for (PxU32 j = 0; j < nbY; j += strideY, localPos.y += spacingY)
		{
			localPos.x = sdf.mMeshLower.x;
			for (PxU32 i = 0; i < nbX; i += strideX, localPos.x += spacingX)
			{
				PxU32 color;
				if (sdf.mSubgridSize > 0 && k < nbZ - 1 && j < nbY - 1 && i < nbX - 1)
				{
					PxU32 startId = sdf.mSubgridStartSlots[k * (nbX - 1) * (nbY - 1) + j * (nbX - 1) + i];
					if (startId != 0xFFFFFFFFu)
					{
						PxU32 xBase, yBase, zBase;
						decodeTriple(startId, xBase, yBase, zBase);
						xBase *= (subgridSize + 1);
						yBase *= (subgridSize + 1);
						zBase *= (subgridSize + 1);

						//Visualize the subgrid
						for (PxU32 z = 0; z <= subgridSize; z += subgridStride)
						{
							for (PxU32 y = 0; y <= subgridSize; y += subgridStride)
							{
								for (PxU32 x = 0; x <= subgridSize; x += subgridStride)
								{
									PxReal value = decode(&sdf.mSubgridSdf[sdf.mBytesPerSparsePixel * idx(xBase + x, yBase + y, zBase + z,
										sdf.mSdfSubgrids3DTexBlockDim.x * (subgridSize + 1), sdf.mSdfSubgrids3DTexBlockDim.y * (subgridSize + 1))], sdf.mBytesPerSparsePixel, sdf.mSubgridsMinSdfValue, sdf.mSubgridsMaxSdfValue);
									color = makeColor(value, invRange0, invRange1);

									PxVec3 subgridLocalPos = localPos + sdf.mSpacing * PxVec3(PxReal(x), PxReal(y), PxReal(z));

									*points = PxDebugPoint(absPose.transform(subgridLocalPos), color);
									points++;
								}
							}
						}
					}
				}


				PxReal v = sdf.mSdf[k*nbX*nbY + j * nbX + i];

				color = makeColor(v, invRange0, invRange1);

				*points = PxDebugPoint(absPose.transform(localPos), color);
				points++;
			}
		}
	}
	return dataReductionActive;
}

static PX_FORCE_INLINE void outputTriangle(PxDebugLine* segments, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, PxU32 color)
{
	// PT: TODO: use SIMD
	segments[0] = PxDebugLine(v0, v1, color);
	segments[1] = PxDebugLine(v1, v2, color);
	segments[2] = PxDebugLine(v2, v0, color);
}

static void visualizeTriangleMesh(const PxTriangleMeshGeometry& geometry, PxRenderOutput& out, const PxTransform& pose, const PxBounds3& cullbox, PxReal fscale, bool visualizeShapes, bool visualizeEdges, bool useCullBox, bool visualizeSDFs)
{
	const TriangleMesh* triangleMesh = static_cast<const TriangleMesh*>(geometry.triangleMesh);
	
	const PxMat44 midt(PxIdentity);

		// PT: TODO: why do we compute it that way sometimes?
//		const PxMat34 vertex2worldSkew = pose * geometry.scale;

	const PxMat33Padded m33(pose.q);

	const PxMat34 absPose(m33 * toMat33(geometry.scale), pose.p);

	PxU32 nbTriangles = triangleMesh->getNbTrianglesFast();
	const PxU32 nbVertices = triangleMesh->getNbVerticesFast();
	const PxVec3* vertices = triangleMesh->getVerticesFast();
	const void* indices = triangleMesh->getTrianglesFast();
	const bool has16Bit = triangleMesh->has16BitIndices();

	bool drawSDF = visualizeSDFs && triangleMesh->getSDF();

	PxU32* results = NULL;
	if (!drawSDF)
	{
		if (useCullBox)
		{
			const Box worldBox(
				(cullbox.maximum + cullbox.minimum)*0.5f,
				(cullbox.maximum - cullbox.minimum)*0.5f,
				PxMat33(PxIdentity));

			// PT: TODO: use the callback version here to avoid allocating this huge array
			results = PX_ALLOCATE(PxU32, nbTriangles, "tmp triangle indices");
			LimitedResults limitedResults(results, nbTriangles, 0);
			Midphase::intersectBoxVsMesh(worldBox, *triangleMesh, pose, geometry.scale, &limitedResults);
			nbTriangles = limitedResults.mNbResults;

			if (visualizeShapes)
			{
				const PxU32 scolor = gCollisionShapeColor;

				out << midt << scolor;	// PT: no need to output this for each segment!

				// PT: TODO: don't render the same edge multiple times
				PxDebugLine* segments = out.reserveSegments(nbTriangles * 3);
				for (PxU32 i = 0; i < nbTriangles; i++)
				{
					PxVec3 wp[3 + 1];
					getWorldTriangle(results[i], wp, vertices, indices, absPose, has16Bit);
					outputTriangle(segments, wp[0], wp[1], wp[2], scolor);
					segments += 3;
				}
			}
		}
		else
		{
			if (visualizeShapes)
			{
				const PxU32 scolor = gCollisionShapeColor;

				out << midt << scolor;	// PT: no need to output this for each segment!

				const Vec4V posV = Vec4V_From_Vec3V(V3LoadU(&absPose.p.x));

				PxVec3* transformed = PX_ALLOCATE(PxVec3, (nbVertices + 1), "PxVec3");
				//			for(PxU32 i=0;i<nbVertices;i++)
				//				transformed[i] = absPose.transform(vertices[i]);
				for (PxU32 i = 0; i < nbVertices; i++)
				{
					//const Vec4V vertexV = V4LoadU(&vertices[i].x);
					//const Vec4V transformedV = V4Add(multiply3x3V(vertexV, absPose), posV);
					//V4StoreU(transformedV, &transformed[i].x);
					transformV(&transformed[i], &vertices[i], posV, absPose);
				}

				const Gu::EdgeList* edgeList = triangleMesh->requestEdgeList();
				if (edgeList)
				{
					PxU32 nbEdges = edgeList->getNbEdges();
					PxDebugLine* segments = out.reserveSegments(nbEdges);

					const Gu::EdgeData* edges = edgeList->getEdges();
					while (nbEdges--)
					{
						segments->pos0 = transformed[edges->Ref0];
						segments->pos1 = transformed[edges->Ref1];
						segments->color0 = segments->color1 = scolor;
						segments++;
						edges++;
					}
				}
				else
				{
					PxDebugLine* segments = out.reserveSegments(nbTriangles * 3);
					for (PxU32 i = 0; i < nbTriangles; i++)
					{
						PxVec3 wp[3];
						getTriangle(i, wp, transformed, indices, has16Bit);
						outputTriangle(segments, wp[0], wp[1], wp[2], scolor);
						segments += 3;
					}
				}

				PX_FREE(transformed);
			}
		}
	}

	if(fscale!=0.0f)
	{
		if(geometry.scale.hasNegativeDeterminant())
			fscale = -fscale;

		visualizeFaceNormals(fscale, out, nbTriangles, vertices, indices, has16Bit, results, absPose, midt);
	}

	if(visualizeEdges)
		visualizeActiveEdges(out, *triangleMesh, nbTriangles, results, absPose);

	if (drawSDF)
	{
		const Gu::SDF& sdf = triangleMesh->getSdfDataFast();
		//We have an SDF, we should debug render it...
		visualizeSDF(out, sdf, absPose);

	}

	PX_FREE(results);
}

static void visualizeHeightField(const PxHeightFieldGeometry& hfGeometry, PxRenderOutput& out, const PxTransform& absPose, const PxBounds3& cullbox, bool useCullBox)
{
	const HeightField* heightfield = static_cast<const HeightField*>(hfGeometry.heightField);

	// PT: TODO: the debug viz for HFs is minimal at the moment...
	const PxU32 scolor = gCollisionShapeColor;
	const PxMat44 midt = PxMat44(PxIdentity);

	HeightFieldUtil hfUtil(hfGeometry);

	const PxU32 nbRows = heightfield->getNbRowsFast();
	const PxU32 nbColumns = heightfield->getNbColumnsFast();
	const PxU32 nbVerts = nbRows * nbColumns;
	const PxU32 nbTriangles = 2 * nbVerts;

	out << midt << scolor;	// PT: no need to output the same matrix/color for each triangle

	if(useCullBox)
	{
		const PxTransform pose0((cullbox.maximum + cullbox.minimum)*0.5f);
		const PxBoxGeometry boxGeometry((cullbox.maximum - cullbox.minimum)*0.5f);

		PxU32* results = PX_ALLOCATE(PxU32, nbTriangles, "tmp triangle indices");

		bool overflow = false;
		PxU32 nbTouchedTris = PxMeshQuery::findOverlapHeightField(boxGeometry, pose0, hfGeometry, absPose, results, nbTriangles, 0, overflow);
		
		PxDebugLine* segments = out.reserveSegments(nbTouchedTris*3);

		for(PxU32 i=0; i<nbTouchedTris; i++)
		{
			const PxU32 index = results[i];
			PxTriangle currentTriangle;
			PxMeshQuery::getTriangle(hfGeometry, absPose, index, currentTriangle);

			//The check has been done in the findOverlapHeightField
			//if(heightfield->isValidTriangle(index) && heightfield->getTriangleMaterial(index) != PxHeightFieldMaterial::eHOLE)
			{
				outputTriangle(segments, currentTriangle.verts[0], currentTriangle.verts[1], currentTriangle.verts[2], scolor);
				segments+=3;
			}
		}
		PX_FREE(results);
	}
	else
	{
		// PT: transform vertices only once
		PxVec3* tmpVerts = PX_ALLOCATE(PxVec3, nbVerts, "PxVec3");
		// PT: TODO: optimize the following line
		for(PxU32 i=0;i<nbVerts;i++)
			tmpVerts[i] = absPose.transform(hfUtil.hf2shapep(heightfield->getVertex(i)));

		for(PxU32 i=0; i<nbTriangles; i++)
		{
			if(heightfield->isValidTriangle(i) && heightfield->getTriangleMaterial(i) != PxHeightFieldMaterial::eHOLE)
			{
				PxU32 vi0, vi1, vi2;
				heightfield->getTriangleVertexIndices(i, vi0, vi1, vi2);

				PxDebugLine* segments = out.reserveSegments(3);
				outputTriangle(segments, tmpVerts[vi0], tmpVerts[vi1], tmpVerts[vi2], scolor);
			}
		}
		PX_FREE(tmpVerts);
	}
}

static void visualize(const PxGeometry& geometry, PxRenderOutput& out, const PxTransform& absPose, const PxBounds3& cullbox, const PxReal fscale, bool visualizeShapes, bool visualizeEdges, bool useCullBox, bool visualizeSDFs)
{
	// triangle meshes can render active edges or face normals, but for other types we can just early out if there are no collision shapes
	if(!visualizeShapes && geometry.getType() != PxGeometryType::eTRIANGLEMESH)
		return;

	switch(geometry.getType())
	{
	case PxGeometryType::eSPHERE:
		visualizeSphere(static_cast<const PxSphereGeometry&>(geometry), out, absPose);
		break;
	case PxGeometryType::eBOX:
		visualizeBox(static_cast<const PxBoxGeometry&>(geometry), out, absPose);
		break;
	case PxGeometryType::ePLANE:
		visualizePlane(static_cast<const PxPlaneGeometry&>(geometry), out, absPose);
		break;
	case PxGeometryType::eCAPSULE:
		visualizeCapsule(static_cast<const PxCapsuleGeometry&>(geometry), out, absPose);
		break;
	case PxGeometryType::eCONVEXCORE:
		PX_ASSERT(static_cast<const PxConvexCoreGeometry&>(geometry).isValid());
		visualizeConvexCore(static_cast<const PxConvexCoreGeometry&>(geometry), out, absPose, cullbox);
		break;
	case PxGeometryType::eCONVEXMESH:
		visualizeConvexMesh(static_cast<const PxConvexMeshGeometry&>(geometry), out, absPose);
		break;
	case PxGeometryType::eTRIANGLEMESH:
		visualizeTriangleMesh(static_cast<const PxTriangleMeshGeometry&>(geometry), out, absPose, cullbox, fscale, visualizeShapes, visualizeEdges, useCullBox, visualizeSDFs);
		break;
	case PxGeometryType::eHEIGHTFIELD:
		visualizeHeightField(static_cast<const PxHeightFieldGeometry&>(geometry), out, absPose, cullbox, useCullBox);
		break;
	case PxGeometryType::eTETRAHEDRONMESH:
	case PxGeometryType::ePARTICLESYSTEM:
		// A.B. missing visualization code
		break;
	case PxGeometryType::eCUSTOM:
		PX_ASSERT(static_cast<const PxCustomGeometry&>(geometry).isValid());
		static_cast<const PxCustomGeometry&>(geometry).callbacks->visualize(geometry, out, absPose, cullbox);
		break;
	case PxGeometryType::eINVALID:
		break;
	case PxGeometryType::eGEOMETRY_COUNT:
		break;
	}
}

void NpShapeManager::visualize(PxRenderOutput& out, NpScene& scene, const PxRigidActor& actor, float scale) const
{
	PX_ASSERT(scale!=0.0f);	// Else we shouldn't have been called

	const Sc::Scene& scScene = scene.getScScene();

	const PxU32 nbShapes = getNbShapes();
	NpShape*const* PX_RESTRICT shapes = getShapes();

	const bool visualizeCompounds = (nbShapes>1) && scScene.getVisualizationParameter(PxVisualizationParameter::eCOLLISION_COMPOUNDS)!=0.0f;

	// PT: moved all these out of the loop, no need to grab them once per shape
	const PxBounds3& cullbox		= scScene.getVisualizationCullingBox();
	const bool visualizeAABBs		= scScene.getVisualizationParameter(PxVisualizationParameter::eCOLLISION_AABBS)!=0.0f;
	const bool visualizeShapes		= scScene.getVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES)!=0.0f;
	const bool visualizeEdges		= scScene.getVisualizationParameter(PxVisualizationParameter::eCOLLISION_EDGES)!=0.0f;
	const bool visualizeSDFs		= scScene.getVisualizationParameter(PxVisualizationParameter::eSDF)!=0.0f;
	const float fNormals			= scScene.getVisualizationParameter(PxVisualizationParameter::eCOLLISION_FNORMALS);
	const bool visualizeFNormals	= fNormals!=0.0f;
	const bool visualizeCollision	= visualizeShapes || visualizeFNormals || visualizeEdges || visualizeSDFs;
	const bool useCullBox			= !cullbox.isEmpty();
	const bool needsShapeBounds0	= visualizeCompounds || (visualizeCollision && useCullBox);
	const PxReal collisionAxes		= scale * scScene.getVisualizationParameter(PxVisualizationParameter::eCOLLISION_AXES);
	const PxReal fscale				= scale * fNormals;

	const PxTransform32 actorPose(actor.getGlobalPose());

	PxBounds3 compoundBounds(PxBounds3::empty());
	for(PxU32 i=0;i<nbShapes;i++)
	{
		const NpShape& npShape = *shapes[i];

		PxTransform32 absPose;
		aos::transformMultiply<true, true>(absPose, actorPose, npShape.getCore().getShape2Actor());

		const PxGeometry& geom = npShape.getCore().getGeometry();
		const bool shapeDebugVizEnabled = npShape.getCore().getFlags() & PxShapeFlag::eVISUALIZATION;

		const bool needsShapeBounds = needsShapeBounds0 || (visualizeAABBs && shapeDebugVizEnabled);
		const PxBounds3 currentShapeBounds = needsShapeBounds ? computeBounds(geom, absPose) : PxBounds3::empty();

		if(shapeDebugVizEnabled)
		{
			if(visualizeAABBs)
			{
				out << PxU32(PxDebugColor::eARGB_YELLOW) << PxMat44(PxIdentity);
				renderOutputDebugBox(out, currentShapeBounds);
			}

			if(collisionAxes != 0.0f)
			{
				out << PxMat44(absPose);
				Cm::renderOutputDebugBasis(out, PxDebugBasis(PxVec3(collisionAxes), 0xcf0000, 0x00cf00, 0x0000cf));
			}

			if(visualizeCollision)
			{
				if(!useCullBox || cullbox.intersects(currentShapeBounds))
					::visualize(geom, out, absPose, cullbox, fscale, visualizeShapes, visualizeEdges, useCullBox, visualizeSDFs);
			}
		}

		if(visualizeCompounds)
			compoundBounds.include(currentShapeBounds);
	}

	if(visualizeCompounds && !compoundBounds.isEmpty())
	{
		out << gCollisionShapeColor << PxMat44(PxIdentity);
		renderOutputDebugBox(out, compoundBounds);
	}
}

/////

static PX_FORCE_INLINE void visualizeActor(PxRenderOutput& out, const Sc::Scene& scScene, const PxRigidActor& actor, float scale)
{
	//visualize actor frames
	const PxReal actorAxes = scale * scScene.getVisualizationParameter(PxVisualizationParameter::eACTOR_AXES);
	if(actorAxes != 0.0f)
	{
		out << actor.getGlobalPose();
		Cm::renderOutputDebugBasis(out, PxDebugBasis(PxVec3(actorAxes)));
	}
}

void physx::visualizeRigidBody(PxRenderOutput& out, NpScene& scene, const PxRigidActor& actor, const Sc::BodyCore& core, float scale)
{
	PX_ASSERT(scale!=0.0f);	// Else we shouldn't have been called
	PX_ASSERT(core.getActorFlags() & PxActorFlag::eVISUALIZATION);	// Else we shouldn't have been called

	const Sc::Scene& scScene = scene.getScScene();

	visualizeActor(out, scScene, actor, scale);

	const PxTransform& body2World = core.getBody2World();

	const PxReal bodyAxes = scale * scScene.getVisualizationParameter(PxVisualizationParameter::eBODY_AXES);
	if(bodyAxes != 0.0f)
	{
		out << body2World;
		Cm::renderOutputDebugBasis(out, PxDebugBasis(PxVec3(bodyAxes)));
	}

	const PxReal linVelocity = scale * scScene.getVisualizationParameter(PxVisualizationParameter::eBODY_LIN_VELOCITY);
	if(linVelocity != 0.0f)
	{
		out << 0xffffff << PxMat44(PxIdentity);
		Cm::renderOutputDebugArrow(out, PxDebugArrow(body2World.p, core.getLinearVelocity() * linVelocity, 0.2f * linVelocity));
	}

	const PxReal angVelocity = scale * scScene.getVisualizationParameter(PxVisualizationParameter::eBODY_ANG_VELOCITY);
	if(angVelocity != 0.0f)
	{
		out << 0x000000 << PxMat44(PxIdentity);
		Cm::renderOutputDebugArrow(out, PxDebugArrow(body2World.p, core.getAngularVelocity() * angVelocity, 0.2f * angVelocity));
	}

	const PxReal massAxes = scale * scScene.getVisualizationParameter(PxVisualizationParameter::eBODY_MASS_AXES);
	if(massAxes != 0.0f)
	{
		const PxReal sleepTime = core.getWakeCounter() / scene.getWakeCounterResetValueInternal();
		PxU32 color = PxU32(0xff * (sleepTime>1.0f ? 1.0f : sleepTime));
		color = core.isSleeping() ? 0xff0000 : (color<<16 | color<<8 | color);
		PxVec3 dims = invertDiagInertia(core.getInverseInertia());
		dims = getDimsFromBodyInertia(dims, 1.0f / core.getInverseMass());
		out << color << core.getBody2World();
		const PxVec3 extents = dims * 0.5f;
		Cm::renderOutputDebugBox(out, PxBounds3(-extents, extents));
	}
}

void NpRigidStatic::visualize(PxRenderOutput& out, NpScene& scene, float scale) const
{
	PX_ASSERT(scale!=0.0f);	// Else we shouldn't have been called
	if(!(mCore.getActorFlags() & PxActorFlag::eVISUALIZATION))
		return;

	NpRigidStaticT::visualize(out, scene, scale);

	visualizeActor(out, scene.getScScene(), *this, scale);
}

void NpRigidDynamic::visualize(PxRenderOutput& out, NpScene& scene, float scale) const
{
	PX_ASSERT(scale!=0.0f);	// Else we shouldn't have been called
	if(!(mCore.getActorFlags() & PxActorFlag::eVISUALIZATION))
		return;

	NpRigidDynamicT::visualize(out, scene, scale);
}

void NpArticulationLink::visualize(PxRenderOutput& out, NpScene& scene, float scale) const
{
	PX_ASSERT(scale!=0.0f);	// Else we shouldn't have been called
	if(!(mCore.getActorFlags() & PxActorFlag::eVISUALIZATION))
		return;

	NpArticulationLinkT::visualize(out, scene, scale);

	const Sc::Scene& scScene = scene.getScScene();

	const PxReal frameScale = scale * scScene.getVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES);
	const PxReal limitScale = scale * scScene.getVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS);
	if(frameScale != 0.0f || limitScale != 0.0f)
	{
		ConstraintImmediateVisualizer viz(frameScale, limitScale, out);
		visualizeJoint(viz);
	}
}

void NpArticulationLink::visualizeJoint(PxConstraintVisualizer& jointViz) const
{
	const NpArticulationLink* parent = getParent();
	if(parent)
	{
		// PT:: tag: scalar transform*transform
		PxTransform cA2w = getGlobalPose().transform(mInboundJoint->getChildPose());
		// PT:: tag: scalar transform*transform
		PxTransform cB2w = parent->getGlobalPose().transform(mInboundJoint->getParentPose());
	
		jointViz.visualizeJointFrames(cA2w, cB2w);

		NpArticulationJointReducedCoordinate* impl = static_cast<NpArticulationJointReducedCoordinate*>(mInboundJoint);

		PX_ASSERT(getArticulation().getConcreteType() == PxConcreteType::eARTICULATION_REDUCED_COORDINATE);
		//(1) visualize any angular dofs/limits...

		const PxMat33 cA2w_m(cA2w.q), cB2w_m(cB2w.q);

		PxTransform parentFrame = cB2w;

		if (cA2w.q.dot(cB2w.q) < 0)
			cB2w.q = -cB2w.q;

		//const PxTransform cB2cA = cA2w.transformInv(cB2w);

		const PxTransform cA2cB = cB2w.transformInv(cA2w);

		Sc::ArticulationJointCore& joint = impl->getCore();

		if(joint.getMotion(PxArticulationAxis::eTWIST))
		{
			const PxArticulationLimit pair = joint.getLimit(PxArticulationAxis::Enum(PxArticulationAxis::eTWIST));

			jointViz.visualizeAngularLimit(parentFrame, pair.low, pair.high);
		}

		if (joint.getMotion(PxArticulationAxis::eSWING1))
		{
			const PxArticulationLimit pair = joint.getLimit(PxArticulationAxis::Enum(PxArticulationAxis::eSWING1));

			PxTransform tmp = parentFrame;
			tmp.q = tmp.q * PxQuat(-PxPiDivTwo, PxVec3(0.f, 0.f, 1.f));
				
			jointViz.visualizeAngularLimit(tmp, -pair.high, -pair.low);
		}

		if (joint.getMotion(PxArticulationAxis::eSWING2))
		{
			const PxArticulationLimit pair = joint.getLimit(PxArticulationAxis::Enum(PxArticulationAxis::eSWING2));

			PxTransform tmp = parentFrame;
			tmp.q = tmp.q * PxQuat(PxPiDivTwo, PxVec3(0.f, 1.f, 0.f));

			jointViz.visualizeAngularLimit(tmp, -pair.high, -pair.low);
		}

		for (PxU32 i = PxArticulationAxis::eX; i <= PxArticulationAxis::eZ; ++i)
		{
			if (joint.getMotion(PxArticulationAxis::Enum(i)) == PxArticulationMotion::eLIMITED)
			{
				const PxU32 index = i - PxArticulationAxis::eX;
				
				const PxArticulationLimit pair = joint.getLimit(PxArticulationAxis::Enum(i));
				const PxReal ordinate = cA2cB.p[index];
				const PxVec3& origin = cB2w.p;
				const PxVec3& axis = cA2w_m[index];
				const bool active = ordinate < pair.low || ordinate > pair.high;
				const PxVec3 p0 = origin + axis * pair.low;
				const PxVec3 p1 = origin + axis * pair.high;
				jointViz.visualizeLine(p0, p1, active ? 0xff0000u : 0xffffffu);
			}
		}
	}	
}

void NpArticulationReducedCoordinate::visualize(PxRenderOutput& out, NpScene& scene, float scale) const
{
	const PxU32 nbLinks = mArticulationLinks.size();
	for (PxU32 i = 0; i < nbLinks; i++)
		mArticulationLinks[i]->visualize(out, scene, scale);
}

#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

void NpScene::visualize()
{
	PX_PROFILE_ZONE("NpScene::visualize", getContextId());
	NP_READ_CHECK(this);

	mRenderBuffer.clear(); // clear last frame visualizations 

#if PX_ENABLE_DEBUG_VISUALIZATION
	const PxReal scale = mScene.getVisualizationParameter(PxVisualizationParameter::eSCALE);
	if(scale == 0.0f)
		return;

	PxRenderOutput out(mRenderBuffer);

	// Visualize scene axes
	const PxReal worldAxes = scale * mScene.getVisualizationParameter(PxVisualizationParameter::eWORLD_AXES);
	if(worldAxes != 0)
		Cm::renderOutputDebugBasis(out, PxDebugBasis(PxVec3(worldAxes)));

	// Visualize articulations
	const PxU32 articulationCount = mArticulations.size();
	for(PxU32 i=0;i<articulationCount;i++)
		static_cast<const NpArticulationReducedCoordinate *>(mArticulations.getEntries()[i])->visualize(out, *this, scale);

	// Visualize rigid actors
	const PxU32 rigidDynamicCount = mRigidDynamics.size();
	for(PxU32 i=0; i<rigidDynamicCount; i++)
		mRigidDynamics[i]->visualize(out, *this, scale);

	const PxU32 rigidStaticCount = mRigidStatics.size();
	for(PxU32 i=0; i<rigidStaticCount; i++)
		mRigidStatics[i]->visualize(out, *this, scale);

	// Visualize pruning structures
	const bool visStatic = mScene.getVisualizationParameter(PxVisualizationParameter::eCOLLISION_STATIC) != 0.0f;
	const bool visDynamic = mScene.getVisualizationParameter(PxVisualizationParameter::eCOLLISION_DYNAMIC) != 0.0f;
	//flushQueryUpdates(); // DE7834
	if(visStatic)
		getSQAPI().visualize(PxU32(PX_SCENE_PRUNER_STATIC), out);
	if(visDynamic)
		getSQAPI().visualize(PxU32(PX_SCENE_PRUNER_DYNAMIC), out);
	if(visStatic || visDynamic)
		getSQAPI().visualize(PxU32(PX_SCENE_COMPOUND_PRUNER), out);

	if(mScene.getVisualizationParameter(PxVisualizationParameter::eMBP_REGIONS) != 0.0f)
	{
		out << PxTransform(PxIdentity);

		const Bp::BroadPhase* bp = mScene.getAABBManager()->getBroadPhase();

		const PxU32 nbRegions = bp->getNbRegions();
		for(PxU32 i=0;i<nbRegions;i++)
		{
			PxBroadPhaseRegionInfo info;
			bp->getRegions(&info, 1, i);

			if(info.mActive)
				out << PxU32(PxDebugColor::eARGB_YELLOW);
			else
				out << PxU32(PxDebugColor::eARGB_BLACK);
			Cm::renderOutputDebugBox(out, info.mRegion.mBounds);
		}
	}

	if(mScene.getVisualizationParameter(PxVisualizationParameter::eCULL_BOX)!=0.0f)
	{
		const PxBounds3& cullbox = mScene.getVisualizationCullingBox();
		if(!cullbox.isEmpty())
		{
			out << PxU32(PxDebugColor::eARGB_YELLOW);
			Cm::renderOutputDebugBox(out, cullbox);
		}
	}

#if PX_SUPPORT_GPU_PHYSX
	// Visualize particle systems
	{
		{
			PxPBDParticleSystem*const* particleSystems = mPBDParticleSystems.getEntries();
			const PxU32 particleSystemCount = mPBDParticleSystems.size();

			for (PxU32 i = 0; i < particleSystemCount; i++)
				static_cast<NpPBDParticleSystem*>(particleSystems[i])->visualize(out, *this);
		}
	}

	// TOTO Visualize deformable surfaces

	// Visualize deformable volumes
	{
		PxDeformableVolume*const* deformableVolumes = mDeformableVolumes.getEntries();
		const PxU32 deformableVolumeCount = mDeformableVolumes.size();

		const bool visualize = mScene.getVisualizationParameter(PxVisualizationParameter::eSIMULATION_MESH) != 0.0f;
		for(PxU32 i=0; i< deformableVolumeCount; i++)
			deformableVolumes[i]->setDeformableVolumeFlag(PxDeformableVolumeFlag::eDISPLAY_SIM_MESH, visualize);
	}
#endif

#if PX_SUPPORT_PVD
	mScenePvdClient.visualize(mRenderBuffer);
#endif

#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif
}

