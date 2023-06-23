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


#include "GuCookingSDF.h"
#include "cooking/PxTriangleMeshDesc.h"
#include "GuSDF.h"
#include "GuCooking.h"

namespace physx
{

	struct MeshData
	{
		MeshData(const PxTriangleMeshDesc& desc)
		{
			m_positions.resize(desc.points.count);
			m_indices.resize(desc.triangles.count * 3);

			immediateCooking::gatherStrided(desc.points.data, &m_positions[0], desc.points.count, sizeof(PxVec3), desc.points.stride);
			immediateCooking::gatherStrided(desc.triangles.data, &m_indices[0], desc.triangles.count, 3 * sizeof(PxU32), desc.triangles.stride);
		}

		void GetBounds(PxVec3& outMinExtents, PxVec3& outMaxExtents) const
		{
			PxVec3 minExtents(FLT_MAX);
			PxVec3 maxExtents(-FLT_MAX);

			for (PxU32 i = 0; i < m_positions.size(); ++i)
			{
				const PxVec3& a = m_positions[i];

				minExtents = a.minimum(minExtents);
				maxExtents = a.maximum(maxExtents);
			}

			outMinExtents = minExtents;
			outMaxExtents = maxExtents;
		}

		PxArray<PxVec3> m_positions;
		PxArray<PxU32> m_indices;
	};

	PX_PHYSX_COMMON_API void quantizeSparseSDF(PxSdfBitsPerSubgridPixel::Enum bitsPerSubgridPixel,
		const PxArray<PxReal>& uncompressedSdfDataSubgrids, PxArray<PxU8>& compressedSdfDataSubgrids,
		PxReal subgridsMinSdfValue, PxReal subgridsMaxSdfValue)
	{
		PxU32 bytesPerPixel = PxU32(bitsPerSubgridPixel);

		compressedSdfDataSubgrids.resize(uncompressedSdfDataSubgrids.size() * bytesPerPixel);

		PxReal* ptr32 = reinterpret_cast<PxReal*>(compressedSdfDataSubgrids.begin());
		PxU16* ptr16 = reinterpret_cast<PxU16*>(compressedSdfDataSubgrids.begin());
		PxU8* ptr8 = compressedSdfDataSubgrids.begin();

		PxReal s = 1.0f / (subgridsMaxSdfValue - subgridsMinSdfValue);

		for (PxU32 i = 0; i < uncompressedSdfDataSubgrids.size(); ++i)
		{
			PxReal v = uncompressedSdfDataSubgrids[i];
			PxReal vNormalized = (v - subgridsMinSdfValue) * s;
			switch (bitsPerSubgridPixel)
			{
			case PxSdfBitsPerSubgridPixel::e8_BIT_PER_PIXEL:
				ptr8[i] = PxU8(255.0f * vNormalized);
				break;
			case PxSdfBitsPerSubgridPixel::e16_BIT_PER_PIXEL:
				ptr16[i] = PxU16(65535.0f * vNormalized);
				break;
			case PxSdfBitsPerSubgridPixel::e32_BIT_PER_PIXEL:
				ptr32[i] = v;
				break;			
			}
		}
	}

	PX_FORCE_INLINE PxU32 idxCompact(PxU32 x, PxU32 y, PxU32 z, PxU32 width, PxU32 height)
	{
		return z * (width) * (height)+y * (width)+x;
	}

	void convert16To32Bits(PxSimpleTriangleMesh mesh, PxArray<PxU32>& indices32)
	{
		indices32.resize(3 * mesh.triangles.count);
		if (mesh.flags & PxMeshFlag::e16_BIT_INDICES)
		{
			// conversion; 16 bit index -> 32 bit index & stride
			PxU32* dest = indices32.begin();
			const PxU32* pastLastDest = indices32.begin() + 3 * mesh.triangles.count;
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
			immediateCooking::gatherStrided(mesh.triangles.data, indices32.begin(), mesh.triangles.count, sizeof(PxU32) * 3, mesh.triangles.stride);
		}
	}

	static bool createSDFSparse(PxTriangleMeshDesc& desc, PxSDFDesc& sdfDesc, PxArray<PxReal>& sdf, PxArray<PxU8>& sdfDataSubgrids,
		PxArray<PxU32>& sdfSubgridsStartSlots)
	{
		PX_ASSERT(sdfDesc.subgridSize > 0);

		MeshData mesh(desc);

		PxVec3 meshLower, meshUpper;
		if (sdfDesc.sdfBounds.isEmpty())
			mesh.GetBounds(meshLower, meshUpper);
		else
		{
			meshLower = sdfDesc.sdfBounds.minimum;
			meshUpper = sdfDesc.sdfBounds.maximum;
		}

		PxVec3 edges = meshUpper - meshLower;

		const PxReal spacing = sdfDesc.spacing;

		// tweak spacing to avoid edge cases for vertices laying on the boundary
		// just covers the case where an edge is a whole multiple of the spacing.
		PxReal spacingEps = spacing * (1.0f - 1e-4f);

		// make sure to have at least one particle in each dimension
		PxI32 dx, dy, dz;
		dx = spacing > edges.x ? 1 : PxI32(edges.x / spacingEps);
		dy = spacing > edges.y ? 1 : PxI32(edges.y / spacingEps);
		dz = spacing > edges.z ? 1 : PxI32(edges.z / spacingEps);

		dx += 4;
		dy += 4;
		dz += 4;

		//Make sure that dx, dy and dz are multiple of subgridSize
		dx = ((dx + sdfDesc.subgridSize - 1) / sdfDesc.subgridSize) * sdfDesc.subgridSize;
		dy = ((dy + sdfDesc.subgridSize - 1) / sdfDesc.subgridSize) * sdfDesc.subgridSize;
		dz = ((dz + sdfDesc.subgridSize - 1) / sdfDesc.subgridSize) * sdfDesc.subgridSize;

		PX_ASSERT(dx % sdfDesc.subgridSize == 0);
		PX_ASSERT(dy % sdfDesc.subgridSize == 0);
		PX_ASSERT(dz % sdfDesc.subgridSize == 0);

		// we shift the voxelization bounds so that the voxel centers
		// lie symmetrically to the center of the object. this reduces the 
		// chance of missing features, and also better aligns the particles
		// with the mesh
		PxVec3 meshOffset;
		meshOffset.x = 0.5f * (spacing - (edges.x - (dx - 1)*spacing));
		meshOffset.y = 0.5f * (spacing - (edges.y - (dy - 1)*spacing));
		meshOffset.z = 0.5f * (spacing - (edges.z - (dz - 1)*spacing));
		meshLower -= meshOffset;

		sdfDesc.meshLower = meshLower;

		sdfDesc.dims.x = dx;
		sdfDesc.dims.y = dy;
		sdfDesc.dims.z = dz;


		PxReal narrowBandThickness = sdfDesc.narrowBandThicknessRelativeToSdfBoundsDiagonal * edges.magnitude();
		PxReal subgridsMinSdfValue, subgridsMaxSdfValue;
		PxArray<PxReal> denseSdf;
		{		
			PxArray<PxU32> indices32;
			PxArray<PxVec3> vertices;
			const PxVec3* verticesPtr = NULL;
			bool baseMeshSpecified = sdfDesc.baseMesh.triangles.data && sdfDesc.baseMesh.points.data;
			if (baseMeshSpecified)
			{
				convert16To32Bits(sdfDesc.baseMesh, indices32);

				if (sdfDesc.baseMesh.points.stride != sizeof(PxVec3))
				{
					vertices.resize(sdfDesc.baseMesh.points.count);
					immediateCooking::gatherStrided(sdfDesc.baseMesh.points.data, vertices.begin(), sdfDesc.baseMesh.points.count, sizeof(PxVec3), sdfDesc.baseMesh.points.stride);
					verticesPtr = vertices.begin();
				}
				else
					verticesPtr = reinterpret_cast<const PxVec3*>(sdfDesc.baseMesh.points.data);
			}

			PxArray<PxReal> sparseSdf;
			Gu::SDFUsingWindingNumbersSparse(
				baseMeshSpecified ? verticesPtr : &mesh.m_positions[0],
				baseMeshSpecified ? indices32.begin() : &mesh.m_indices[0], 
				baseMeshSpecified ? indices32.size() : mesh.m_indices.size(), 
				dx, dy, dz,
				meshLower, meshLower + PxVec3(static_cast<PxReal>(dx), static_cast<PxReal>(dy), static_cast<PxReal>(dz)) * spacing, narrowBandThickness, sdfDesc.subgridSize,
				sdf, sdfSubgridsStartSlots, sparseSdf, denseSdf, subgridsMinSdfValue, subgridsMaxSdfValue, 16);

			PxArray<PxReal> uncompressedSdfDataSubgrids;
			Gu::convertSparseSDFTo3DTextureLayout(dx, dy, dz, sdfDesc.subgridSize, sdfSubgridsStartSlots.begin(), sparseSdf.begin(), sparseSdf.size(), uncompressedSdfDataSubgrids,
				sdfDesc.sdfSubgrids3DTexBlockDim.x, sdfDesc.sdfSubgrids3DTexBlockDim.y, sdfDesc.sdfSubgrids3DTexBlockDim.z);

			if (sdfDesc.bitsPerSubgridPixel == 4)
			{
				//32bit values are stored as normal floats while 16bit and 8bit values are scaled to 0...1 range and then scaled back to original range
				subgridsMinSdfValue = 0.0f;
				subgridsMaxSdfValue = 1.0f;
			}

			quantizeSparseSDF(sdfDesc.bitsPerSubgridPixel, uncompressedSdfDataSubgrids, sdfDataSubgrids, 
				subgridsMinSdfValue, subgridsMaxSdfValue);
		}

		sdfDesc.sdf.count = sdf.size();
		sdfDesc.sdf.stride = sizeof(PxReal);
		sdfDesc.sdf.data = sdf.begin();
		sdfDesc.sdfSubgrids.count = sdfDataSubgrids.size();
		sdfDesc.sdfSubgrids.stride = sizeof(PxU8);
		sdfDesc.sdfSubgrids.data = sdfDataSubgrids.begin();
		sdfDesc.sdfStartSlots.count = sdfSubgridsStartSlots.size();
		sdfDesc.sdfStartSlots.stride = sizeof(PxU32);
		sdfDesc.sdfStartSlots.data = sdfSubgridsStartSlots.begin();

		sdfDesc.subgridsMinSdfValue = subgridsMinSdfValue;
		sdfDesc.subgridsMaxSdfValue = subgridsMaxSdfValue;

		return true;
	}

	static bool createSDF(PxTriangleMeshDesc& desc, PxSDFDesc& sdfDesc, PxArray<PxReal>& sdf, PxArray<PxU8>& sdfDataSubgrids, PxArray<PxU32>& sdfSubgridsStartSlots)
	{
		if (sdfDesc.subgridSize > 0)
		{
			return createSDFSparse(desc, sdfDesc, sdf, sdfDataSubgrids, sdfSubgridsStartSlots);
		}

		MeshData mesh(desc);

		PxVec3 meshLower, meshUpper;
		if (sdfDesc.sdfBounds.isEmpty())
			mesh.GetBounds(meshLower, meshUpper);
		else
		{
			meshLower = sdfDesc.sdfBounds.minimum;
			meshUpper = sdfDesc.sdfBounds.maximum;
		}

		PxVec3 edges = meshUpper - meshLower;

		const PxReal spacing = sdfDesc.spacing;

		// tweak spacing to avoid edge cases for vertices laying on the boundary
		// just covers the case where an edge is a whole multiple of the spacing.
		PxReal spacingEps = spacing * (1.0f - 1e-4f);

		// make sure to have at least one particle in each dimension
		PxI32 dx, dy, dz;
		dx = spacing > edges.x ? 1 : PxI32(edges.x / spacingEps);
		dy = spacing > edges.y ? 1 : PxI32(edges.y / spacingEps);
		dz = spacing > edges.z ? 1 : PxI32(edges.z / spacingEps);

		dx += 4;
		dy += 4;
		dz += 4;

		const PxU32 numVoxels = dx * dy * dz;

		// we shift the voxelization bounds so that the voxel centers
		// lie symmetrically to the center of the object. this reduces the 
		// chance of missing features, and also better aligns the particles
		// with the mesh
		PxVec3 meshOffset;
		meshOffset.x = 0.5f * (spacing - (edges.x - (dx - 1)*spacing));
		meshOffset.y = 0.5f * (spacing - (edges.y - (dy - 1)*spacing));
		meshOffset.z = 0.5f * (spacing - (edges.z - (dz - 1)*spacing));
		meshLower -= meshOffset;

		sdfDesc.meshLower = meshLower;

		sdfDesc.dims.x = dx;
		sdfDesc.dims.y = dy;
		sdfDesc.dims.z = dz;

		sdf.resize(numVoxels);
		Gu::SDFUsingWindingNumbers(&mesh.m_positions[0], &mesh.m_indices[0], mesh.m_indices.size(), dx, dy, dz, &sdf[0], meshLower,
			meshLower + PxVec3(static_cast<PxReal>(dx), static_cast<PxReal>(dy), static_cast<PxReal>(dz)) * spacing, NULL, true, sdfDesc.numThreadsForSdfConstruction);

		sdfDesc.sdf.count = sdfDesc.dims.x * sdfDesc.dims.y * sdfDesc.dims.z;
		sdfDesc.sdf.stride = sizeof(PxReal);
		sdfDesc.sdf.data = &sdf[0];

		return true;
	}

	bool buildSDF(PxTriangleMeshDesc& desc, PxArray<PxReal>& sdf, PxArray<PxU8>& sdfDataSubgrids, PxArray<PxU32>& sdfSubgridsStartSlots)
	{
		PxSDFDesc& sdfDesc = *desc.sdfDesc;

		if (!sdfDesc.sdf.data && sdfDesc.spacing > 0.f)
		{
			// Calculate signed distance field here if no sdf data provided.
			createSDF(desc, sdfDesc, sdf, sdfDataSubgrids, sdfSubgridsStartSlots);
			sdfDesc.sdf.stride = sizeof(PxReal);
		}
		return true;
	}
}
