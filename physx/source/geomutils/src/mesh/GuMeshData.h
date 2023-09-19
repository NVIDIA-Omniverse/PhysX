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

#ifndef GU_MESH_DATA_H
#define GU_MESH_DATA_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"
#include "foundation/PxBounds3.h"
#include "geometry/PxTriangleMesh.h"
#include "geometry/PxTetrahedronMesh.h"

#include "foundation/PxUserAllocated.h"
#include "foundation/PxAllocator.h"
#include "GuRTree.h"
#include "GuBV4.h"
#include "GuBV32.h"
#include "GuSDF.h"

namespace physx
{
namespace Gu {
	
// 1: support stackless collision trees for non-recursive collision queries
// 2: height field functionality not supported anymore
// 3: mass struct removed
// 4: bounding sphere removed
// 5: RTree added, opcode tree still in the binary image, physx 3.0
// 6: opcode tree removed from binary image
// 7: convex decomposition is out
// 8: adjacency information added
// 9: removed leaf triangles and most of opcode data, changed rtree layout
// 10: float rtrees
// 11: new build, isLeaf added to page
// 12: isLeaf is now the lowest bit in ptrs
// 13: TA30159 removed deprecated convexEdgeThreshold and bumped version
// 14: added midphase ID
// 15: GPU data simplification
// 16: vertex2Face mapping enabled by default if using GPU

#define PX_MESH_VERSION 16
#define PX_TET_MESH_VERSION 1
#define PX_SOFTBODY_MESH_VERSION 2

// these flags are used to indicate/validate the contents of a cooked mesh file
enum InternalMeshSerialFlag
{
	IMSF_MATERIALS		=	(1<<0),	//!< if set, the cooked mesh file contains per-triangle material indices
	IMSF_FACE_REMAP		=	(1<<1),	//!< if set, the cooked mesh file contains a remap table
	IMSF_8BIT_INDICES	=	(1<<2),	//!< if set, the cooked mesh file contains 8bit indices (topology)
	IMSF_16BIT_INDICES	=	(1<<3),	//!< if set, the cooked mesh file contains 16bit indices (topology)
	IMSF_ADJACENCIES	=	(1<<4),	//!< if set, the cooked mesh file contains adjacency structures
	IMSF_GRB_DATA		=	(1<<5),	//!< if set, the cooked mesh file contains GRB data structures
	IMSF_SDF			=	(1<<6),	//!< if set, the cooked mesh file contains SDF data structures
	IMSF_VERT_MAPPING	=   (1<<7), //!< if set, the cooked mesh file contains vertex mapping information
	IMSF_GRB_INV_REMAP	=	(1<<8),	//!< if set, the cooked mesh file contains vertex inv mapping information. Required for cloth
	IMSF_INERTIA		=	(1<<9)	//!< if set, the cooked mesh file contains inertia tensor for the mesh
};

#if PX_VC
#pragma warning(push)
#pragma warning(disable: 4324)	// Padding was added at the end of a structure because of a __declspec(align) value.
#endif

	class MeshDataBase : public PxUserAllocated
	{
	public:
		PxMeshMidPhase::Enum	mType;
		PxU8					mFlags;
		PxU32					mNbVertices;
		PxVec3*					mVertices;

		PxReal					mMass;					//this is mass assuming a unit density that can be scaled by instances!
		PxMat33					mInertia;				//in local space of mesh!
		PxVec3					mLocalCenterOfMass;		//local space com

		PxBounds3				mAABB;
		PxReal					mGeomEpsilon;

		PxU32*					mFaceRemap;
		
		// GRB data -------------------------
		void*					mGRB_primIndices;		//!< GRB: GPU-friendly primitive indices(either triangle or tetrahedron)
		PxU32*					mGRB_faceRemap;			//!< GRB: this remap the GPU triangle indices to CPU triangle indices
		PxU32*					mGRB_faceRemapInverse;	//
		// End of GRB data ------------------

		// SDF data
		SDF						mSdfData;

		//Cloth data : each vert has a list of associated triangles in the mesh, this is for attachement constraints to enable default filtering
		PxU32*					mAccumulatedTrianglesRef;//runsum
		PxU32*					mTrianglesReferences;
		PxU32					mNbTrianglesReferences;

		MeshDataBase() :
			mFlags					(0),
			mNbVertices				(0),
			mVertices				(NULL),
			mMass					(0.f),
			mInertia				(PxZero),
			mLocalCenterOfMass		(0.f),
			mAABB					(PxBounds3::empty()),
			mGeomEpsilon			(0.0f),
			mFaceRemap				(NULL),
			mGRB_primIndices		(NULL),
			mGRB_faceRemap			(NULL),
			mGRB_faceRemapInverse	(NULL),
			mSdfData				(PxZero),
			mAccumulatedTrianglesRef(NULL),
			mTrianglesReferences	(NULL),
			mNbTrianglesReferences	(0)
		{
		}

		virtual ~MeshDataBase()
		{
			PX_FREE(mVertices);
			PX_FREE(mFaceRemap);
			PX_FREE(mGRB_primIndices);	
			PX_FREE(mGRB_faceRemap);
			PX_FREE(mGRB_faceRemapInverse);
			
			PX_FREE(mAccumulatedTrianglesRef);

			PX_FREE(mTrianglesReferences);
		}

		PX_NOINLINE PxVec3* allocateVertices(PxU32 nbVertices)
		{
			PX_ASSERT(!mVertices);
			// PT: we allocate one more vertex to make sure it's safe to V4Load the last one
			const PxU32 nbAllocatedVerts = nbVertices + 1;
			mVertices = PX_ALLOCATE(PxVec3, nbAllocatedVerts, "PxVec3");
			mNbVertices = nbVertices;
			return mVertices;
		}

		PX_FORCE_INLINE	bool	has16BitIndices()	const
		{
			return (mFlags & PxTriangleMeshFlag::e16_BIT_INDICES) ? true : false;
		}
	};

	class TriangleMeshData : public MeshDataBase
	{
		public:
		
		PxU32			mNbTriangles;
		void*			mTriangles;

		PxU32*			mAdjacencies;
		PxU8*			mExtraTrigData;
		PxU16*			mMaterialIndices;

		// GRB data -------------------------
		void*			mGRB_primAdjacencies;	//!< GRB: adjacency data, with BOUNDARY and NONCONVEX flags (flags replace adj indices where applicable) [uin4]
		Gu::BV32Tree*	mGRB_BV32Tree;
		// End of GRB data ------------------

		TriangleMeshData() :
			mNbTriangles			(0),
			mTriangles				(NULL),
			mAdjacencies			(NULL),
			mExtraTrigData			(NULL),
			mMaterialIndices		(NULL),
			mGRB_primAdjacencies	(NULL),
			mGRB_BV32Tree			(NULL)
		{
		}

		virtual ~TriangleMeshData()
		{
			PX_FREE(mTriangles);
			PX_FREE(mAdjacencies);
			PX_FREE(mMaterialIndices);
			PX_FREE(mExtraTrigData);
			PX_FREE(mGRB_primAdjacencies);
			PX_DELETE(mGRB_BV32Tree);
		}

		PX_NOINLINE PxU32* allocateAdjacencies()
		{
			PX_ASSERT(mNbTriangles);
			PX_ASSERT(!mAdjacencies);
			mAdjacencies = PX_ALLOCATE(PxU32, mNbTriangles * 3, "mAdjacencies");
			mFlags |= PxTriangleMeshFlag::eADJACENCY_INFO;
			return mAdjacencies;
		}

		PX_NOINLINE PxU32* allocateFaceRemap()
		{
			PX_ASSERT(mNbTriangles);
			PX_ASSERT(!mFaceRemap);
			mFaceRemap = PX_ALLOCATE(PxU32, mNbTriangles, "mFaceRemap");
			return mFaceRemap;
		}

		PX_NOINLINE void* allocateTriangles(PxU32 nbTriangles, bool force32Bit, PxU32 allocateGPUData = 0)
		{
			PX_ASSERT(mNbVertices);
			PX_ASSERT(!mTriangles);

			bool index16 = mNbVertices <= 0xffff && !force32Bit;
			if(index16)
				mFlags |= PxTriangleMeshFlag::e16_BIT_INDICES;

			mTriangles = PX_ALLOC(nbTriangles * (index16 ? sizeof(PxU16) : sizeof(PxU32)) * 3, "mTriangles");
			if (allocateGPUData)
				mGRB_primIndices = PX_ALLOC(nbTriangles * (index16 ? sizeof(PxU16) : sizeof(PxU32)) * 3, "mGRB_triIndices");
			mNbTriangles = nbTriangles;
			return mTriangles;
		}

		PX_NOINLINE PxU16* allocateMaterials()
		{
			PX_ASSERT(mNbTriangles);
			PX_ASSERT(!mMaterialIndices);
			mMaterialIndices = PX_ALLOCATE(PxU16, mNbTriangles, "mMaterialIndices");
			return mMaterialIndices;
		}

		PX_NOINLINE PxU8* allocateExtraTrigData()
		{
			PX_ASSERT(mNbTriangles);
			PX_ASSERT(!mExtraTrigData);
			mExtraTrigData = PX_ALLOCATE(PxU8, mNbTriangles, "mExtraTrigData");
			return mExtraTrigData;
		}

		PX_FORCE_INLINE void	setTriangleAdjacency(PxU32 triangleIndex, PxU32 adjacency, PxU32 offset)
		{
			PX_ASSERT(mAdjacencies); 
			mAdjacencies[triangleIndex*3 + offset] = adjacency; 
		}
	};

	class RTreeTriangleData : public TriangleMeshData
	{
		public:
								RTreeTriangleData()		{ mType = PxMeshMidPhase::eBVH33; }
		virtual					~RTreeTriangleData()	{}

				Gu::RTree		mRTree;
	};

	class BV4TriangleData : public TriangleMeshData
	{
		public:
								BV4TriangleData()	{ mType = PxMeshMidPhase::eBVH34;	}
		virtual					~BV4TriangleData()	{}

				Gu::SourceMesh	mMeshInterface;
				Gu::BV4Tree		mBV4Tree;
	};

	// PT: TODO: the following classes should probably be in their own specific files (e.g. GuTetrahedronMeshData.h, GuSoftBodyMeshData.h)

	class TetrahedronMeshData : public PxTetrahedronMeshData
	{
	public:
		PxU32					mNbVertices;
		PxVec3*					mVertices;
		PxU16*					mMaterialIndices; //each tetrahedron should have a material index

		PxU32					mNbTetrahedrons;
		void*					mTetrahedrons;	  //IndTetrahedron32		

		PxU8					mFlags;

		PxReal					mGeomEpsilon;
		PxBounds3				mAABB;

		TetrahedronMeshData() :			
			mNbVertices(0),
			mVertices(NULL),
			mMaterialIndices(NULL),
			mNbTetrahedrons(0),
			mTetrahedrons(NULL),			
			mFlags(0),
			mGeomEpsilon(0.0f),
			mAABB(PxBounds3::empty())
		{}

		TetrahedronMeshData(PxVec3* vertices, PxU32 nbVertices, void* tetrahedrons, PxU32 nbTetrahedrons, PxU8 flags, PxReal geomEpsilon, PxBounds3 aabb) :
			mNbVertices(nbVertices),
			mVertices(vertices),
			mNbTetrahedrons(nbTetrahedrons),
			mTetrahedrons(tetrahedrons),
			mFlags(flags),
			mGeomEpsilon(geomEpsilon),
			mAABB(aabb)
		{}

		void allocateTetrahedrons(const PxU32 nbGridTetrahedrons, const PxU32 allocateGPUData = 0)
		{
			if (allocateGPUData)
			{
				mTetrahedrons = PX_ALLOC(nbGridTetrahedrons * sizeof(PxU32) * 4, "mGridModelTetrahedrons");			
			}

			mNbTetrahedrons = nbGridTetrahedrons;
		}

		PxVec3* allocateVertices(PxU32 nbVertices, const PxU32 allocateGPUData = 1)
		{
			PX_ASSERT(!mVertices);
			// PT: we allocate one more vertex to make sure it's safe to V4Load the last one
			if (allocateGPUData)
			{
				const PxU32 nbAllocatedVerts = nbVertices + 1;
				mVertices = PX_ALLOCATE(PxVec3, nbAllocatedVerts, "PxVec3");
			}
			mNbVertices = nbVertices;
			return mVertices;
		}

		PxU16* allocateMaterials()
		{
			PX_ASSERT(mNbTetrahedrons);
			PX_ASSERT(!mMaterialIndices);
			mMaterialIndices = PX_ALLOCATE(PxU16, mNbTetrahedrons, "mMaterialIndices");
			return mMaterialIndices;
		}

		PX_FORCE_INLINE	bool	has16BitIndices()	const
		{
			return (mFlags & PxTriangleMeshFlag::e16_BIT_INDICES) ? true : false;
		}

		~TetrahedronMeshData()
		{
			PX_FREE(mTetrahedrons);
			PX_FREE(mVertices);
			PX_FREE(mMaterialIndices)
		}
	};

	class SoftBodyCollisionData : public PxSoftBodyCollisionData
	{
	public:
		PxU32*					mFaceRemap;

		// GRB data -------------------------
		void *					mGRB_primIndices;				//!< GRB: GPU-friendly primitive indices(either triangle or tetrahedron)
		PxU32*					mGRB_faceRemap;					//!< GRB: this remap the GPU triangle indices to CPU triangle indices
		PxU32*					mGRB_faceRemapInverse;
		Gu::BV32Tree*			mGRB_BV32Tree;
		PxU8*					mGRB_tetraSurfaceHint;


		// End of GRB data ------------------
		Gu::TetrahedronSourceMesh	mMeshInterface;
		Gu::BV4Tree					mBV4Tree;

		PxMat33*				mTetraRestPoses;


		SoftBodyCollisionData() : 
			mFaceRemap(NULL),
			mGRB_primIndices(NULL),
			mGRB_faceRemap(NULL),
			mGRB_faceRemapInverse(NULL),
			mGRB_BV32Tree(NULL),
			mGRB_tetraSurfaceHint(NULL),
			mTetraRestPoses(NULL)
		{}

		virtual ~SoftBodyCollisionData()
		{
			PX_FREE(mGRB_tetraSurfaceHint);
			PX_DELETE(mGRB_BV32Tree);
			PX_FREE(mFaceRemap);
			PX_FREE(mGRB_primIndices);
			PX_FREE(mGRB_faceRemap);
			PX_FREE(mGRB_faceRemapInverse);
			PX_FREE(mTetraRestPoses);
		}

		PxU32* allocateFaceRemap(PxU32 nbTetrahedrons)
		{
			PX_ASSERT(nbTetrahedrons);
			PX_ASSERT(!mFaceRemap);
			mFaceRemap = PX_ALLOCATE(PxU32, nbTetrahedrons, "mFaceRemap");
			return mFaceRemap;
		}

		void allocateCollisionData(PxU32 nbTetrahedrons)
		{
			mGRB_primIndices = PX_ALLOC(nbTetrahedrons * 4 * sizeof(PxU32), "mGRB_primIndices");
			mGRB_tetraSurfaceHint = PX_ALLOCATE(PxU8, nbTetrahedrons, "mGRB_tetraSurfaceHint");

			mTetraRestPoses = PX_ALLOCATE(PxMat33, nbTetrahedrons, "mTetraRestPoses");

			
		}
	};

	class CollisionMeshMappingData : public PxCollisionMeshMappingData
	{
	public:
		PxReal*					mVertsBarycentricInGridModel;
		PxU32*					mVertsRemapInGridModel;

		PxU32*					mTetsRemapColToSim;
		PxU32					mTetsRemapSize;
		PxU32*					mTetsAccumulatedRemapColToSim; //runsum, size of number of tetrahedrons in collision mesh
		
		//in the collision model, each vert has a list of associated simulation tetrahedrons, this is for attachement constraints to enable default filtering
		PxU32*					mCollisionAccumulatedTetrahedronsRef;//runsum
		PxU32*					mCollisionTetrahedronsReferences;
		PxU32					mCollisionNbTetrahedronsReferences;

		PxU32*					mCollisionSurfaceVertToTetRemap;
		PxU8*					mCollisionSurfaceVertsHint;

		CollisionMeshMappingData() : 
			mVertsBarycentricInGridModel(NULL),
			mVertsRemapInGridModel(NULL),
			mTetsRemapColToSim(NULL),
			mTetsRemapSize(0),
			mTetsAccumulatedRemapColToSim(NULL),
			mCollisionAccumulatedTetrahedronsRef(NULL),
			mCollisionTetrahedronsReferences(NULL),
			mCollisionNbTetrahedronsReferences(0),
			mCollisionSurfaceVertToTetRemap(NULL),
			mCollisionSurfaceVertsHint(NULL)			
		{

		}

		virtual ~CollisionMeshMappingData()
		{
			PX_FREE(mVertsBarycentricInGridModel);
			PX_FREE(mVertsRemapInGridModel);
			PX_FREE(mTetsRemapColToSim);
			PX_FREE(mTetsAccumulatedRemapColToSim);
			PX_FREE(mCollisionAccumulatedTetrahedronsRef);
			PX_FREE(mCollisionTetrahedronsReferences);
			PX_FREE(mCollisionSurfaceVertsHint);
			PX_FREE(mCollisionSurfaceVertToTetRemap);
		}

		void allocatemappingData(const PxU32 nbVerts, const PxU32 tetRemapSize, const PxU32 nbColTetrahedrons, const PxU32 allocateGPUData = 0)
		{
			if (allocateGPUData)
			{	
				mVertsBarycentricInGridModel = reinterpret_cast<PxReal*>(PX_ALLOC(nbVerts * sizeof(PxReal) * 4, "mVertsBarycentricInGridModel"));
				mVertsRemapInGridModel = reinterpret_cast<PxU32*>(PX_ALLOC(nbVerts * sizeof(PxU32), "mVertsRemapInGridModel"));
				mTetsRemapColToSim = reinterpret_cast<PxU32*>(PX_ALLOC(tetRemapSize * sizeof(PxU32), "mTetsRemapInSimModel"));
				mTetsAccumulatedRemapColToSim = reinterpret_cast<PxU32*>(PX_ALLOC(nbColTetrahedrons * sizeof(PxU32), "mTetsAccumulatedRemapInSimModel"));
				mCollisionSurfaceVertsHint = reinterpret_cast<PxU8*>(PX_ALLOC(nbVerts * sizeof(PxU8), "mCollisionSurfaceVertsHint"));
				mCollisionSurfaceVertToTetRemap = reinterpret_cast<PxU32*>(PX_ALLOC(nbVerts * sizeof(PxU32), "mCollisionSurfaceVertToTetRemap"));
			}
			mTetsRemapSize = tetRemapSize;
		}

		void allocateTetRefData(const PxU32 totalTetReference, const PxU32 nbCollisionVerts, const PxU32 allocateGPUData /*= 0*/)
		{
			if (allocateGPUData)
			{
				mCollisionAccumulatedTetrahedronsRef = reinterpret_cast<PxU32*>(PX_ALLOC(nbCollisionVerts * sizeof(PxU32), "mGMAccumulatedTetrahedronsRef"));
				mCollisionTetrahedronsReferences = reinterpret_cast<PxU32*>(PX_ALLOC(totalTetReference * sizeof(PxU32), "mGMTetrahedronsReferences"));
				
			}

			mCollisionNbTetrahedronsReferences = totalTetReference;
		}

		virtual void release()
		{
			PX_DELETE_THIS; 
		}
	};	

	class SoftBodySimulationData : public PxSoftBodySimulationData
	{
	public:
		PxReal*					mGridModelInvMass;

		PxMat33*				mGridModelTetraRestPoses;

		PxU32					mGridModelNbPartitions;
		PxU32					mGridModelMaxTetsPerPartitions;

		PxU32*					mGridModelOrderedTetrahedrons; // the corresponding tetrahedron index for the runsum

		PxU32*					mGMRemapOutputCP;
		PxU32*					mGMAccumulatedPartitionsCP; //runsum for the combined partition 
		PxU32*					mGMAccumulatedCopiesCP; //runsum for the vert copies in combined partitions

		PxU32					mGMRemapOutputSize;

		PxU32*					mGMPullIndices;

		PxU32					mNumTetsPerElement;

		SoftBodySimulationData() :
			mGridModelInvMass(NULL),
			mGridModelTetraRestPoses(NULL),
			mGridModelNbPartitions(0),
			mGridModelOrderedTetrahedrons(NULL),
			mGMRemapOutputCP(NULL),
			mGMAccumulatedPartitionsCP(NULL),
			mGMAccumulatedCopiesCP(NULL),
			mGMRemapOutputSize(0),
			mGMPullIndices(NULL)
		{}

		virtual ~SoftBodySimulationData()
		{
			PX_FREE(mGridModelInvMass);
			PX_FREE(mGridModelTetraRestPoses);
			PX_FREE(mGridModelOrderedTetrahedrons);
			PX_FREE(mGMRemapOutputCP);
			PX_FREE(mGMAccumulatedPartitionsCP);
			PX_FREE(mGMAccumulatedCopiesCP);		
			PX_FREE(mGMPullIndices);
		}

		void allocateGridModelData(const PxU32 nbGridTetrahedrons, const PxU32 nbGridVerts,
			const PxU32 nbVerts, const PxU32 nbPartitions, const PxU32 remapOutputSize, const PxU32 numTetsPerElement, const PxU32 allocateGPUData = 0)
		{
			PX_UNUSED(nbVerts);

			if (allocateGPUData)
			{
				const PxU32 numElements = nbGridTetrahedrons / numTetsPerElement;
				const PxU32 numVertsPerElement = (numTetsPerElement == 6 || numTetsPerElement == 5) ? 8 : 4;

				mGridModelInvMass = reinterpret_cast<float*>(PX_ALLOC(nbGridVerts * sizeof(float), "mGridModelInvMass"));
				mGridModelTetraRestPoses = reinterpret_cast<PxMat33*>(PX_ALLOC(nbGridTetrahedrons * sizeof(PxMat33), "mGridModelTetraRestPoses"));

				mGridModelOrderedTetrahedrons = reinterpret_cast<PxU32*>(PX_ALLOC(numElements * sizeof(PxU32), "mGridModelOrderedTetrahedrons"));
				mGMRemapOutputCP = reinterpret_cast<PxU32*>(PX_ALLOC(remapOutputSize * sizeof(PxU32), "mGMRemapOutputCP"));
				mGMAccumulatedPartitionsCP = reinterpret_cast<PxU32*>(PX_ALLOC(nbPartitions * sizeof(PxU32), "mGMAccumulatedPartitionsCP"));
				mGMAccumulatedCopiesCP = reinterpret_cast<PxU32*>(PX_ALLOC(nbGridVerts * sizeof(PxU32), "mGMAccumulatedCopiesCP"));			
				mGMPullIndices = reinterpret_cast<PxU32*>(PX_ALLOC(numElements * numVertsPerElement * sizeof(PxU32) , "mGMPullIndices"));
			}
			
			mGridModelNbPartitions = nbPartitions;
			mGMRemapOutputSize = remapOutputSize;
		}
	};

	class CollisionTetrahedronMeshData : public PxCollisionTetrahedronMeshData
	{
	public:
		TetrahedronMeshData* mMesh;
		SoftBodyCollisionData* mCollisionData;

		virtual PxTetrahedronMeshData* getMesh() { return mMesh; }
		virtual const PxTetrahedronMeshData* getMesh() const { return mMesh; }
		virtual PxSoftBodyCollisionData* getData() { return mCollisionData; }
		virtual const PxSoftBodyCollisionData* getData() const { return mCollisionData; }

		virtual ~CollisionTetrahedronMeshData()
		{
			PX_FREE(mMesh);
			PX_FREE(mCollisionData);
		}

		virtual void release()
		{
			PX_DELETE_THIS; 
		}
	};

	class SimulationTetrahedronMeshData : public PxSimulationTetrahedronMeshData
	{
	public:
		TetrahedronMeshData* mMesh;
		SoftBodySimulationData* mSimulationData;

		virtual PxTetrahedronMeshData* getMesh() { return mMesh; }
		virtual PxSoftBodySimulationData* getData() { return mSimulationData; }

		virtual ~SimulationTetrahedronMeshData()
		{
			PX_FREE(mMesh);
			PX_FREE(mSimulationData);
		}

		virtual void release()
		{
			PX_DELETE_THIS; 
		}
	};

	class SoftBodyMeshData : public PxUserAllocated
	{
		PX_NOCOPY(SoftBodyMeshData)
	public:	
		TetrahedronMeshData& mSimulationMesh;
		SoftBodySimulationData& mSimulationData;
		TetrahedronMeshData& mCollisionMesh;
		SoftBodyCollisionData& mCollisionData;	
		CollisionMeshMappingData& mMappingData;

		SoftBodyMeshData(TetrahedronMeshData& simulationMesh, SoftBodySimulationData& simulationData, 
			TetrahedronMeshData& collisionMesh, SoftBodyCollisionData& collisionData, CollisionMeshMappingData& mappingData) :
			mSimulationMesh(simulationMesh),
			mSimulationData(simulationData),
			mCollisionMesh(collisionMesh),
			mCollisionData(collisionData),
			mMappingData(mappingData)
		{ }
	};

#if PX_VC
#pragma warning(pop)
#endif


} // namespace Gu

}

#endif // #ifdef GU_MESH_DATA_H
