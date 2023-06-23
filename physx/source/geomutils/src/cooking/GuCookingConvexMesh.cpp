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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "GuCooking.h"
#include "GuCookingConvexMeshBuilder.h"
#include "GuCookingQuickHullConvexHullLib.h"
#include "GuConvexMesh.h"
#include "foundation/PxAlloca.h"
#include "foundation/PxFPU.h"
#include "common/PxInsertionCallback.h"

using namespace physx;
using namespace Gu;

///////////////////////////////////////////////////////////////////////////////

PX_IMPLEMENT_OUTPUT_ERROR

///////////////////////////////////////////////////////////////////////////////

// cook convex mesh from given desc, internal function to be shared between create/cook convex mesh
static bool cookConvexMeshInternal(const PxCookingParams& params, const PxConvexMeshDesc& desc_, ConvexMeshBuilder& meshBuilder, ConvexHullLib* hullLib, PxConvexMeshCookingResult::Enum* condition)
{
	if(condition)
		*condition = PxConvexMeshCookingResult::eFAILURE;

	if(!desc_.isValid())
		return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "Cooking::cookConvexMesh: user-provided convex mesh descriptor is invalid!");

	if(params.areaTestEpsilon <= 0.0f)
		return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "Cooking::cookConvexMesh: provided cooking parameter areaTestEpsilon is invalid!");

	if(params.planeTolerance < 0.0f)
		return outputError<PxErrorCode::eINVALID_PARAMETER>(__LINE__, "Cooking::cookConvexMesh: provided cooking parameter planeTolerance is invalid!");

	PxConvexMeshDesc desc = desc_;	
	bool polygonsLimitReached = false;

	// the convex will be cooked from provided points
	if(desc_.flags & PxConvexFlag::eCOMPUTE_CONVEX)
	{
		PX_ASSERT(hullLib);

		// clean up the indices information, it could have been set by accident
		desc.flags &= ~PxConvexFlag::e16_BIT_INDICES;
		desc.indices.count = 0;
		desc.indices.data = NULL;
		desc.indices.stride = 0;
		desc.polygons.count = 0;
		desc.polygons.data = NULL;
		desc.polygons.stride = 0;

		PxConvexMeshCookingResult::Enum res = hullLib->createConvexHull();
		if(res == PxConvexMeshCookingResult::eSUCCESS || res == PxConvexMeshCookingResult::ePOLYGONS_LIMIT_REACHED)
		{
			if(res == PxConvexMeshCookingResult::ePOLYGONS_LIMIT_REACHED)
				polygonsLimitReached = true;

			hullLib->fillConvexMeshDesc(desc);
		}
		else
		{
			if((res == PxConvexMeshCookingResult::eZERO_AREA_TEST_FAILED) && condition)
			{
				*condition = PxConvexMeshCookingResult::eZERO_AREA_TEST_FAILED;
			}
			
			return false;
		}
	}

	if(desc.points.count >= 256)
		return outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Cooking::cookConvexMesh: user-provided hull must have less than 256 vertices!");

	if(desc.polygons.count >= 256)
		return outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Cooking::cookConvexMesh: user-provided hull must have less than 256 faces!");

	if ((desc.flags & PxConvexFlag::eGPU_COMPATIBLE) || params.buildGPUData)
	{
		if (desc.points.count > 64)
			return outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Cooking::cookConvexMesh: GPU-compatible user-provided hull must have less than 65 vertices!");

		if (desc.polygons.count > 64)
			return outputError<PxErrorCode::eINTERNAL_ERROR>(__LINE__, "Cooking::cookConvexMesh: GPU-compatible user-provided hull must have less than 65 faces!");
	}
		
	if(!meshBuilder.build(desc, params.gaussMapLimit, false, hullLib))
		return false;

	if(condition)
		*condition = polygonsLimitReached ? PxConvexMeshCookingResult::ePOLYGONS_LIMIT_REACHED : PxConvexMeshCookingResult::eSUCCESS;

	return true;
}

static ConvexHullLib* createHullLib(PxConvexMeshDesc& desc, const PxCookingParams& params)
{	
	if(desc.flags & PxConvexFlag::eCOMPUTE_CONVEX)
	{			
		const PxU16 gpuMaxVertsLimit = 64;
		const PxU16 gpuMaxFacesLimit = 64;

		// GRB supports 64 verts max
		if((desc.flags & PxConvexFlag::eGPU_COMPATIBLE) || params.buildGPUData)
		{
			desc.vertexLimit = PxMin(desc.vertexLimit, gpuMaxVertsLimit);
			desc.polygonLimit = PxMin(desc.polygonLimit, gpuMaxFacesLimit);
		}

		return PX_NEW(QuickHullConvexHullLib) (desc, params);
	}
	return NULL;
}

bool immediateCooking::cookConvexMesh(const PxCookingParams& params, const PxConvexMeshDesc& desc_, PxOutputStream& stream, PxConvexMeshCookingResult::Enum* condition)
{	
	PX_FPU_GUARD;

	// choose cooking library if needed
    PxConvexMeshDesc desc = desc_;
	ConvexHullLib* hullLib = createHullLib(desc, params);

	ConvexMeshBuilder meshBuilder(params.buildGPUData);
	if(!cookConvexMeshInternal(params, desc, meshBuilder, hullLib, condition))
	{
		PX_DELETE(hullLib);
		return false;
	}

	// save the cooked results into stream
	if(!meshBuilder.save(stream, platformMismatch()))
	{		
		if(condition)
			*condition = PxConvexMeshCookingResult::eFAILURE;

		PX_DELETE(hullLib);
		return false;
	}
	
	PX_DELETE(hullLib);
	return true;
}

PxConvexMesh* immediateCooking::createConvexMesh(const PxCookingParams& params, const PxConvexMeshDesc& desc_, PxInsertionCallback& insertionCallback, PxConvexMeshCookingResult::Enum* condition)
{
	PX_FPU_GUARD;

	// choose cooking library if needed
	PxConvexMeshDesc desc = desc_;
	ConvexHullLib* hullLib = createHullLib(desc, params);

	// cook the mesh
	ConvexMeshBuilder meshBuilder(params.buildGPUData);
	if(!cookConvexMeshInternal(params, desc, meshBuilder, hullLib, condition))
	{
		PX_DELETE(hullLib);		
		return NULL;
	}
	
	// copy the constructed data into the new mesh

	ConvexHullInitData meshData;
	meshBuilder.copy(meshData);

	// insert into physics
	PxConvexMesh* convexMesh = static_cast<PxConvexMesh*>(insertionCallback.buildObjectFromData(PxConcreteType::eCONVEX_MESH, &meshData));
	if(!convexMesh)
	{
		if(condition)
			*condition = PxConvexMeshCookingResult::eFAILURE;
		PX_DELETE(hullLib);
		return NULL;
	}

	PX_DELETE(hullLib);
	return convexMesh;
}

bool immediateCooking::validateConvexMesh(const PxCookingParams& params, const PxConvexMeshDesc& desc)
{
	ConvexMeshBuilder mesh(params.buildGPUData);
	return mesh.build(desc, params.gaussMapLimit, true);
}

bool immediateCooking::computeHullPolygons(const PxCookingParams& params, const PxSimpleTriangleMesh& mesh, PxAllocatorCallback& inCallback, PxU32& nbVerts, PxVec3*& vertices,
											PxU32& nbIndices, PxU32*& indices, PxU32& nbPolygons, PxHullPolygon*& hullPolygons)
{
	PxVec3* geometry = reinterpret_cast<PxVec3*>(PxAlloca(sizeof(PxVec3)*mesh.points.count));
	immediateCooking::gatherStrided(mesh.points.data, geometry, mesh.points.count, sizeof(PxVec3), mesh.points.stride);

	PxU32* topology = reinterpret_cast<PxU32*>(PxAlloca(sizeof(PxU32)*3*mesh.triangles.count));
	if(mesh.flags & PxMeshFlag::e16_BIT_INDICES)
	{
		// conversion; 16 bit index -> 32 bit index & stride
		PxU32* dest = topology;
		const PxU32* pastLastDest = topology + 3*mesh.triangles.count;
		const PxU8* source = reinterpret_cast<const PxU8*>(mesh.triangles.data);
		while (dest < pastLastDest)
		{
			const PxU16 * trig16 = reinterpret_cast<const PxU16*>(source);
			*dest++ = trig16[0];
			*dest++ = trig16[1];
			*dest++ = trig16[2];
			source += mesh.triangles.stride;
		}
	}
	else
	{
		immediateCooking::gatherStrided(mesh.triangles.data, topology, mesh.triangles.count, sizeof(PxU32) * 3, mesh.triangles.stride);
	}

	ConvexMeshBuilder meshBuilder(params.buildGPUData);
	if(!meshBuilder.computeHullPolygons(mesh.points.count, geometry, mesh.triangles.count, topology, inCallback, nbVerts, vertices, nbIndices, indices, nbPolygons, hullPolygons))
		return false;

	return true;
}
