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

#ifndef GU_SDF_H
#define GU_SDF_H
/** \addtogroup geomutils
@{
*/

#include "common/PxPhysXCommonConfig.h"
#include "foundation/PxVec3.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxArray.h"

namespace physx
{
	namespace Gu
	{
		/**
		\brief Represents dimensions of signed distance field
		*/
		class Dim3
		{
		public:
			/**
			\brief Constructor
			*/
			Dim3()
			{
			}

			/**
			\brief Constructor
			*/
			Dim3(PxZERO d) : x(0), y(0), z(0)
			{
				PX_UNUSED(d);
			}

			/**
			\brief Constructor
			*/
			Dim3(PxU32 _x, PxU32 _y, PxU32 _z) : x(_x), y(_y), z(_z)
			{
			}

			/**
			\brief Copy constructor
			*/
			Dim3(const Dim3& d) : x(d.x), y(d.y), z(d.z)
			{
			}

			PxU32					x;		//!< Size of X dimension
			PxU32					y;		//!< Size of Y dimension
			PxU32					z;		//!< Size of Z dimension
		};

		/**
		\brief Represents a signed distance field.
		*/
		class SDF : public PxUserAllocated
		{
		public:

// PX_SERIALIZATION
			SDF(const PxEMPTY) {}
			static		void					getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
			/**
			\brief Constructor
			*/
			SDF() : mSdf(NULL), mSubgridStartSlots(NULL), mSubgridSdf(NULL)
			{
			}

			/**
			\brief Constructor
			*/
			SDF(PxZERO s)
				: mMeshLower(PxZero), mSpacing(0.0f), mDims(PxZero), mNumSdfs(0), mSdf(NULL),
				mSubgridSize(PxZero), mNumStartSlots(0), mSubgridStartSlots(NULL), mNumSubgridSdfs(0), mSubgridSdf(NULL), mSdfSubgrids3DTexBlockDim(PxZero),
				mSubgridsMinSdfValue(0.0f), mSubgridsMaxSdfValue(0.0f), mBytesPerSparsePixel(0)
			{
				PX_UNUSED(s);
			}

			/**
			\brief Copy constructor
			*/
			SDF(const SDF& sdf) 
				: mMeshLower(sdf.mMeshLower), mSpacing(sdf.mSpacing), mDims(sdf.mDims), mNumSdfs(sdf.mNumSdfs), mSdf(sdf.mSdf),
				mSubgridSize(sdf.mSubgridSize), mNumStartSlots(sdf.mNumStartSlots), mSubgridStartSlots(sdf.mSubgridStartSlots), mNumSubgridSdfs(sdf.mNumSubgridSdfs), mSubgridSdf(sdf.mSubgridSdf), mSdfSubgrids3DTexBlockDim(sdf.mSdfSubgrids3DTexBlockDim),
				mSubgridsMinSdfValue(sdf.mSubgridsMinSdfValue), mSubgridsMaxSdfValue(sdf.mSubgridsMaxSdfValue), mBytesPerSparsePixel(sdf.mBytesPerSparsePixel)
			{
			}

			/**
			\brief Destructor
			*/
			~SDF();
			

			PxReal* allocateSdfs(const PxVec3& meshLower, const PxReal& spacing, const PxU32 dimX, const PxU32 dimY, const PxU32 dimZ,
				const PxU32 subgridSize, const PxU32 sdfSubgrids3DTexBlockDimX, const PxU32 sdfSubgrids3DTexBlockDimY, const PxU32 sdfSubgrids3DTexBlockDimZ,
				PxReal subgridsMinSdfValue, PxReal subgridsMaxSdfValue, PxU32 bytesPerSparsePixel);
			
			PxVec3					mMeshLower;		//!< Lower bound of the original mesh
			PxReal					mSpacing;		//!< Spacing of each sdf voxel
			Dim3					mDims;			//!< Dimension of the sdf
			PxU32					mNumSdfs;		//!< Number of sdf values
			PxReal*					mSdf;			//!< Array of sdf

			// Additional data to support sparse grid SDFs
			PxU32					mSubgridSize;				//!< The number of cells in a sparse subgrid block (full block has mSubgridSize^3 cells and (mSubgridSize+1)^3 samples). If set to zero, this indicates that only a dense background grid SDF is used without sparse blocks
			PxU32					mNumStartSlots;				//!< Array length of mSubgridStartSlots. Only used for serialization
			PxU32*					mSubgridStartSlots;			//!< Array with start indices into the subgrid texture for every subgrid block. 10bits for z coordinate, 10bits for y and 10bits for x
			PxU32					mNumSubgridSdfs;			//!< Array length of mSubgridSdf. Only used for serialization
			PxU8*					mSubgridSdf;				//!< The data to create the 3d texture containg the packed subgrid blocks. Stored as PxU8 to support multiple formats (8, 16 and 32 bits per pixel)
			Dim3					mSdfSubgrids3DTexBlockDim;	//!< Subgrid sdf is layed out as a 3d texture including packed blocks of size (mSubgridSize+1)^3
			PxReal					mSubgridsMinSdfValue;		//!< The minimum value over all subgrid blocks. Used if normalized textures are used which is the case for 8 and 16bit formats
			PxReal					mSubgridsMaxSdfValue;		//!< The maximum value over all subgrid blocks. Used if normalized textures are used which is the case for 8 and 16bit formats
			PxU32					mBytesPerSparsePixel;		//!< The number of bytes per subgrid pixel
		};

		/**
		\brief Returns the number of times a point is enclosed by a triangle mesh. Therefore points with a winding number of 0 lie oufside of the mesh, others lie inside. The sign of the winding number
		is dependent ond the triangle orientation. For close meshes, a robust inside/outside check should not test for a value of 0 exactly, inside = PxAbs(windingNumber) > 0.5f should be preferred.
		
		\param[in] vertices The triangle mesh's vertices
		\param[in] indices The triangle mesh's indices
		\param[in] numTriangleIndices The number of indices
		\param[in] width The number of grid points along the x direction
		\param[in] height The number of grid points along the y direction
		\param[in] depth The number of grid points along the z direction
		\param[out] windingNumbers The winding number for the center of every grid cell, index rule is: index = z * width * height + y * width + x
		\param[in] minExtents The grid's lower corner
		\param[in] maxExtents The grid's upper corner
		\param[out] sampleLocations Optional buffer to output the grid sample locations, index rule is: index = z * width * height + y * width + x
		*/
		PX_PHYSX_COMMON_API void windingNumbers(const PxVec3* vertices, const PxU32* indices, PxU32 numTriangleIndices, PxU32 width, PxU32 height, PxU32 depth, 
			PxReal* windingNumbers, PxVec3 minExtents, PxVec3 maxExtents, PxVec3* sampleLocations = NULL);
		
		/**
		\brief Returns if a point is enclosed by a triangle mesh. 

		\param[in] vertices The triangle mesh's vertices
		\param[in] indices The triangle mesh's indices
		\param[in] numTriangleIndices The number of indices
		\param[in] width The number of grid points along the x direction
		\param[in] height The number of grid points along the y direction
		\param[in] depth The number of grid points along the z direction
		\param[out] insideResult Booleans that indicate if the center of a grid cell is inside or outside, index rule is: index = z * width * height + y * width + x
		\param[in] minExtents The grid's lower corner, the box formed by minExtent and maxExtent must include all vertices
		\param[in] maxExtents The grid's upper corner, the box formed by minExtent and maxExtent must include all vertices
		\param[out] sampleLocations Optional buffer to output the grid sample locations, index rule is: index = z * width * height + y * width + x
		*/
		PX_PHYSX_COMMON_API void windingNumbersInsideCheck(const PxVec3* vertices, const PxU32* indices, PxU32 numTriangleIndices, PxU32 width, PxU32 height, PxU32 depth,
			bool* insideResult, PxVec3 minExtents, PxVec3 maxExtents, PxVec3* sampleLocations = NULL);

		/**
		\brief Returns the distance to the mesh's surface for all samples in a grid. The sign is dependent on the triangle orientation. Negative distances indicate that a sample is inside the mesh, positive
		distances mean the sample is outside of the mesh.

		\param[in] vertices The triangle mesh's vertices
		\param[in] indices The triangle mesh's indices
		\param[in] numTriangleIndices The number of indices
		\param[in] width The number of grid points along the x direction
		\param[in] height The number of grid points along the y direction
		\param[in] depth The number of grid points along the z direction
		\param[out] sdf The signed distance field (negative values indicate that a point is inside of the mesh), index rule is: index = z * width * height + y * width + x
		\param[in] minExtents The grid's lower corner, the box formed by minExtent and maxExtent must include all vertices
		\param[in] maxExtents The grid's upper corner, the box formed by minExtent and maxExtent must include all vertices
		\param[out] sampleLocations Optional buffer to output the grid sample locations, index rule is: index = z * width * height + y * width + x
		\param[in] cellCenteredSamples Determines if the sample points are chosen at cell centers or at cell origins
		\param[in] numThreads The number of cpu threads to use during the computation
		*/
		PX_PHYSX_COMMON_API void SDFUsingWindingNumbers(const PxVec3* vertices, const PxU32* indices, PxU32 numTriangleIndices, PxU32 width, PxU32 height, PxU32 depth, 			
			PxReal* sdf, PxVec3 minExtents, PxVec3 maxExtents, PxVec3* sampleLocations = NULL, bool cellCenteredSamples = true, PxU32 numThreads = 1);

		/**
		\brief Returns the distance to the mesh's surface for all samples in a grid. The sign is dependent on the triangle orientation. Negative distances indicate that a sample is inside the mesh, positive
		distances mean the sample is outside of the mesh. Near mesh surfaces, a higher resolution is available than further away from the surface (sparse sdf format) to save memory.
		The samples are not cell centered but located at the cell origin. This is a requirement of the sparse grid format.

		\param[in] vertices The triangle mesh's vertices
		\param[in] indices The triangle mesh's indices
		\param[in] numTriangleIndices The number of indices
		\param[in] width The number of grid points along the x direction
		\param[in] height The number of grid points along the y direction
		\param[in] depth The number of grid points along the z direction
		\param[in] minExtents The grid's lower corner, the box formed by minExtent and maxExtent must include all vertices
		\param[in] maxExtents The grid's upper corner, the box formed by minExtent and maxExtent must include all vertices

		\param[in] narrowBandThicknessRelativeToExtentDiagonal The thickness of the narrow band as a fraction of the sdf box diagonal length. Can be as small as 0 but a value of at least 0.01 is recommended.  
		\param[in] cellsPerSubgrid The number of cells in a sparse subgrid block (full block has mSubgridSize^3 cells and (mSubgridSize+1)^3 samples)
		\param[out] sdfCoarse The coarse sdf as a dense 3d array of lower resolution (resulution is (with/cellsPerSubgrid+1, height/cellsPerSubgrid+1, depth/cellsPerSubgrid+1))
		\param[out] sdfFineStartSlots The start slot indices of the subgrid blocks. If a subgrid block is empty, the start slot will be 0xFFFFFFFF
		\param[out] subgridData The array containing subgrid data blocks
		\param[out] denseSdf Provides acces to the denxe sdf that is used for compuation internally
		\param[out] subgridsMinSdfValue The minimum value over all subgrid blocks. Used if normalized textures are used which is the case for 8 and 16bit formats
		\param[out] subgridsMaxSdfValue	The maximum value over all subgrid blocks. Used if normalized textures are used which is the case for 8 and 16bit formats
		\param[in] numThreads The number of cpu threads to use during the computation
		*/
		PX_PHYSX_COMMON_API void SDFUsingWindingNumbersSparse(const PxVec3* vertices, const PxU32* indices, PxU32 numTriangleIndices, PxU32 width, PxU32 height, PxU32 depth,
			const PxVec3& minExtents, const PxVec3& maxExtents, PxReal narrowBandThicknessRelativeToExtentDiagonal, PxU32 cellsPerSubgrid,
			PxArray<PxReal>& sdfCoarse, PxArray<PxU32>& sdfFineStartSlots, PxArray<PxReal>& subgridData, PxArray<PxReal>& denseSdf,
			PxReal& subgridsMinSdfValue, PxReal& subgridsMaxSdfValue, PxU32 numThreads = 1);
	

		/**
		\brief Converts a sparse grid sdf to a format that can be used to create a 3d texture. 3d textures support very efficient 
		trilinear interpolation on the GPU which is very important during sdf evaluation.

		\param[in] width The number of grid points along the x direction
		\param[in] height The number of grid points along the y direction
		\param[in] depth The number of grid points along the z direction
		\param[in] cellsPerSubgrid The number of cells in a sparse subgrid block (full block has mSubgridSize^3 cells and (mSubgridSize+1)^3 samples)
		\param[in,out] sdfFineStartSlots Array with linear start indices into the subgrid data array. This array gets converted by this method to start indices for every subgrid block in the 3d texture. The result uses 10bits for z coordinate, 10bits for y and 10bits for x
		\param[in] sdfFineSubgridsIn Subgrid data array
		\param[in] sdfFineSubgridsSize Number of elements in sdfFineSubgridsIn
		\param[out] subgrids3DTexFormat The subgrid data organized in a 3d texture compatible order
		\param[out] numSubgridsX Number of subgrid blocks in the 3d texture along x. The full texture dimension along x will be numSubgridsX*(cellsPerSubgrid+1).
		\param[out] numSubgridsY Number of subgrid blocks in the 3d texture along y. The full texture dimension along y will be numSubgridsY*(cellsPerSubgrid+1).
		\param[out] numSubgridsZ Number of subgrid blocks in the 3d texture along z. The full texture dimension along z will be numSubgridsZ*(cellsPerSubgrid+1).
		*/
		PX_PHYSX_COMMON_API void convertSparseSDFTo3DTextureLayout(PxU32 width, PxU32 height, PxU32 depth, PxU32 cellsPerSubgrid,
			PxU32* sdfFineStartSlots, const PxReal* sdfFineSubgridsIn, PxU32 sdfFineSubgridsSize, PxArray<PxReal>& subgrids3DTexFormat,
			PxU32& numSubgridsX, PxU32& numSubgridsY, PxU32& numSubgridsZ);	

		/**
		\brief Extracts an isosurface as a triangular mesh from a signed distance function

		\param[in] sdf The signed distance function
		\param[out] isosurfaceVertices The vertices of the extracted isosurface
		\param[out] isosurfaceTriangleIndices The triangles of the extracted isosurface
		*/
		PX_PHYSX_COMMON_API void extractIsosurfaceFromSDF(const Gu::SDF& sdf, PxArray<PxVec3>& isosurfaceVertices, PxArray<PxU32>& isosurfaceTriangleIndices);


		/**
		\brief A class that allows to efficiently project points onto the surface of a triangle mesh.
		*/
		class PxPointOntoTriangleMeshProjector
		{
		public:			
			/**
			\brief Projects a point onto the surface of a triangle mesh.

			\param[in] point The point to project
			\return the projected point
			*/
			virtual PxVec3 projectPoint(const PxVec3& point) = 0;

			/**
			\brief Projects a point onto the surface of a triangle mesh.

			\param[in] point The point to project
			\param[out] closestTriangleIndex The index of the triangle on which the projected point is located
			\return the projected point
			*/
			virtual PxVec3 projectPoint(const PxVec3& point, PxU32& closestTriangleIndex) = 0;

			/**
			\brief Releases the instance and its data
			*/
			virtual void release() = 0;
		};

		/**
		\brief Creates a helper class that allows to efficiently project points onto the surface of a triangle mesh.

		\param[in] vertices The triangle mesh's vertices
		\param[in] triangleIndices The triangle mesh's indices
		\param[in] numTriangles The number of triangles
		\return A point onto triangle mesh projector instance. The caller needs to delete the instance once it is not used anymore by calling its release method
		*/
		PX_PHYSX_COMMON_API PxPointOntoTriangleMeshProjector* PxCreatePointOntoTriangleMeshProjector(const PxVec3* vertices, const PxU32* triangleIndices, PxU32 numTriangles);
	}
}

/** @} */
#endif


