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

#ifndef GU_TETRAHEDRONMESH_H
#define GU_TETRAHEDRONMESH_H

#include "foundation/PxIO.h"
#include "geometry/PxTetrahedronMeshGeometry.h"
#include "geometry/PxTetrahedronMesh.h"
#include "geometry/PxTetrahedron.h"
#include "geometry/PxSimpleTriangleMesh.h"
#include "CmRefCountable.h"
#include "common/PxRenderOutput.h"
#include "GuMeshData.h"
#include "GuCenterExtents.h"
#include "GuMeshFactory.h"

namespace physx
{
	namespace Gu
	{
		class MeshFactory;
#if PX_VC
#pragma warning(push)
#pragma warning(disable: 4324)	// Padding was added at the end of a structure because of a __declspec(align) value.
#endif

		class SoftBodyAuxData : public PxSoftBodyAuxData, public PxUserAllocated
		{
		public:
			SoftBodyAuxData(SoftBodySimulationData& d, SoftBodyCollisionData& c, CollisionMeshMappingData& e);

			virtual ~SoftBodyAuxData();

			virtual const char*				getConcreteTypeName()	const { return "PxSoftBodyAuxData"; }

			virtual	void					acquireReference()				{ Cm::RefCountable_incRefCount(*this);			}
			virtual	PxU32					getReferenceCount()		const	{ return Cm::RefCountable_getRefCount(*this);	}
			virtual	void					release()						{ Cm::RefCountable_decRefCount(*this);			}
			virtual	void					onRefCountZero()				{ PX_DELETE_THIS; }
			virtual PxReal*					getGridModelInvMass()			{ return mGridModelInvMass; }

			PX_FORCE_INLINE				PxU32					getNbTetRemapSizeFast()					const { return mTetsRemapSize; }
			PX_FORCE_INLINE				PxReal*					getGridModelInvMassFast()				{ return mGridModelInvMass; }	
			PX_FORCE_INLINE				PxU32					getNbGMPartitionFast()					const { return mGMNbPartitions; }

			PX_FORCE_INLINE				PxU32					getGMRemapOutputSizeFast()				const { return mGMRemapOutputSize; }
			PX_FORCE_INLINE				PxU32					getGMMaxTetsPerPartitionsFast()			const { return mGMMaxMaxTetsPerPartitions; }


			PX_FORCE_INLINE				PxU32*					getCollisionAccumulatedTetrahedronRefs()		const { return mCollisionAccumulatedTetrahedronsRef; }
			PX_FORCE_INLINE				PxU32*					getCollisionTetrahedronRefs()					const { return mCollisionTetrahedronsReferences; }
			PX_FORCE_INLINE				PxU32					getCollisionNbTetrahedronRefs()					const { return mCollisionNbTetrahedronsReferences; }

			PX_FORCE_INLINE				PxU32*					getCollisionSurfaceVertToTetRemap()			const { return mCollisionSurfaceVertToTetRemap;  }
			PX_FORCE_INLINE				PxMat33*				getGridModelRestPosesFast()				{ return mGridModelTetraRestPoses; }
			PX_FORCE_INLINE				PxMat33*				getRestPosesFast()						{ return mTetraRestPoses; }

			float*					mGridModelInvMass;

			PxMat33*				mGridModelTetraRestPoses;
			PxU32*					mGridModelOrderedTetrahedrons;


			PxU32					mGMNbPartitions;
			PxU32					mGMMaxMaxTetsPerPartitions;
			PxU32					mGMRemapOutputSize;
			PxU32*					mGMRemapOutputCP;

			PxU32*					mGMAccumulatedPartitionsCP;
			PxU32*					mGMAccumulatedCopiesCP;

			PxU32*					mCollisionAccumulatedTetrahedronsRef;
			PxU32*					mCollisionTetrahedronsReferences;
			PxU32					mCollisionNbTetrahedronsReferences;

			PxU8*					mCollisionSurfaceVertsHint;
			PxU32*					mCollisionSurfaceVertToTetRemap;

			PxReal*					mVertsBarycentricInGridModel;
			PxU32*					mVertsRemapInGridModel;


			PxU32*					mTetsRemapColToSim;
			PxU32					mTetsRemapSize;
			PxU32*					mTetsAccumulatedRemapColToSim;

			PxU32*					mGMPullIndices;

			PxMat33*				mTetraRestPoses;

			PxU32					mNumTetsPerElement;
		};
		
		class TetrahedronMesh : public PxTetrahedronMesh, public PxUserAllocated
		{
		public:
											TetrahedronMesh(PxU32 nbVertices, PxVec3* vertices, PxU32 nbTetrahedrons, void* tetrahedrons, PxU8 flags, PxBounds3 aabb, PxReal geomEpsilon);
											TetrahedronMesh(TetrahedronMeshData& mesh);
											TetrahedronMesh(MeshFactory* meshFactory, TetrahedronMeshData& mesh);
			virtual							~TetrahedronMesh();

			virtual const char*				getConcreteTypeName()	const { return "PxTetrahedronMesh"; }

			virtual	void					acquireReference()				{ Cm::RefCountable_incRefCount(*this);			}
			virtual	PxU32					getReferenceCount()		const	{ return Cm::RefCountable_getRefCount(*this);	}
			virtual	void					release()						{ Cm::RefCountable_decRefCount(*this);			}
			virtual	void					onRefCountZero();

			virtual	PxU32						getNbVertices()						const { return mNbVertices; }
			virtual const PxVec3*				getVertices()						const { return mVertices; }
			virtual	PxU32						getNbTetrahedrons()					const { return mNbTetrahedrons; }
			virtual	const void*					getTetrahedrons()					const { return mTetrahedrons; }
			virtual	PxTetrahedronMeshFlags		getTetrahedronMeshFlags()			const { return PxTetrahedronMeshFlags(mFlags); }
			virtual	const PxU32*				getTetrahedraRemap()				const { return NULL; }

			PX_FORCE_INLINE				PxU32					getNbVerticesFast()						const { return mNbVertices; }
			PX_FORCE_INLINE				PxVec3*					getVerticesFast()						const { return mVertices; }
			PX_FORCE_INLINE				PxU32					getNbTetrahedronsFast()					const { return mNbTetrahedrons; }
			PX_FORCE_INLINE				const void*				getTetrahedronsFast()					const { return mTetrahedrons; }
			PX_FORCE_INLINE				bool					has16BitIndices()						const { return (mFlags & PxMeshFlag::e16_BIT_INDICES) ? true : false; }
			PX_FORCE_INLINE				bool					hasPerTriangleMaterials()				const { return mMaterialIndices != NULL; }
			PX_FORCE_INLINE				const PxU16*			getMaterials()							const { return mMaterialIndices; }

			PX_FORCE_INLINE	const CenterExtents&				getLocalBoundsFast() const { return mAABB; }

			PX_FORCE_INLINE	const CenterExtentsPadded&			getPaddedBounds() const
			{
				// PT: see compile-time assert in cpp
				return static_cast<const CenterExtentsPadded&>(mAABB);
			}

			virtual	PxBounds3 getLocalBounds() const
			{
				PX_ASSERT(mAABB.isValid());
				return PxBounds3::centerExtents(mAABB.mCenter, mAABB.mExtents);
			}

			PxU32					mNbVertices;
			PxVec3*					mVertices;
			PxU32					mNbTetrahedrons;
			void*					mTetrahedrons;

			PxU8					mFlags;					//!< Flag whether indices are 16 or 32 bits wide	

			PxU16*					mMaterialIndices;		//!< the size of the array is mNbTetrahedrons.

			// PT: WARNING: bounds must be followed by at least 32bits of data for safe SIMD loading			
			CenterExtents			mAABB;
			PxReal					mGeomEpsilon;	

			MeshFactory*			mMeshFactory;					// PT: changed to pointer for serialization
		};

		PX_FORCE_INLINE const Gu::TetrahedronMesh*	_getTetraMeshData(const PxTetrahedronMeshGeometry& meshGeom)
		{
			return static_cast<const Gu::TetrahedronMesh*>(meshGeom.tetrahedronMesh);
		}

		class BVTetrahedronMesh : public TetrahedronMesh
		{
		public:
			BVTetrahedronMesh(TetrahedronMeshData& mesh, SoftBodyCollisionData& d, MeshFactory* factory = NULL);

			virtual ~BVTetrahedronMesh()
			{
				PX_FREE(mGRB_tetraIndices);
				PX_FREE(mGRB_tetraSurfaceHint);
				PX_FREE(mGRB_faceRemap);
				PX_FREE(mGRB_faceRemapInverse);
				PX_DELETE(mGRB_BV32Tree);
				PX_FREE(mFaceRemap);
			}

			//virtual PxBounds3					refitBVH();

			PX_FORCE_INLINE				const Gu::BV4Tree&		getBV4Tree()						const { return mBV4Tree; }

			PX_FORCE_INLINE				Gu::BV4Tree&			getBV4Tree() { return mBV4Tree; }

			PX_FORCE_INLINE				void*					getGRBTetraFaceRemap()				{ return mGRB_faceRemap; }

			PX_FORCE_INLINE				void*					getGRBTetraFaceRemapInverse()		{ return mGRB_faceRemapInverse; }

			virtual						const PxU32*			getTetrahedraRemap()				const { return mFaceRemap; }

			PX_FORCE_INLINE bool isTetMeshGPUCompatible() const
			{
				return mGRB_BV32Tree != NULL;
			}

			PxU32*					mFaceRemap;				//!< new faces to old faces mapping (after cleaning, etc). Usage: old = faceRemap[new]

			// GRB data -------------------------
			void*					mGRB_tetraIndices;				//!< GRB: GPU-friendly tri indices [uint4]
			PxU8*					mGRB_tetraSurfaceHint;
			PxU32*					mGRB_faceRemap;
			PxU32*					mGRB_faceRemapInverse;
			Gu::BV32Tree*			mGRB_BV32Tree;					//!< GRB: BV32 tree

		private:
			Gu::TetrahedronSourceMesh			mMeshInterface4;
			Gu::BV4Tree							mBV4Tree;
			Gu::TetrahedronSourceMesh			mMeshInterface32;
		};

		// Possible optimization: align the whole struct to cache line
		class SoftBodyMesh : public PxSoftBodyMesh, public PxUserAllocated
		{
		public:

			virtual const char*				getConcreteTypeName()	const { return "PxSoftBodyMesh"; }

			// PX_SERIALIZATION
			virtual void								exportExtraData(PxSerializationContext& ctx);
			void										importExtraData(PxDeserializationContext&);
			//PX_PHYSX_COMMON_API	static	void		getBinaryMetaData(PxOutputStream& stream);
			virtual	void								release();

			void										resolveReferences(PxDeserializationContext&) {}
			virtual	void								requiresObjects(PxProcessPxBaseCallback&) {}
			//~PX_SERIALIZATION

			virtual	void								acquireReference()			{ Cm::RefCountable_incRefCount(*this);			}
			virtual	PxU32								getReferenceCount()	const	{ return Cm::RefCountable_getRefCount(*this);	}
			virtual	void								onRefCountZero();

			//virtual	PxMeshMidPhase::Enum	getMidphaseID()			const { return PxMeshMidPhase::eBVH34; }

			SoftBodyMesh(MeshFactory* factory, SoftBodyMeshData& data);
			
			virtual										~SoftBodyMesh();

			void										setMeshFactory(MeshFactory* factory) { mMeshFactory = factory; }
			
			virtual const PxTetrahedronMesh*			getCollisionMesh() const { return mCollisionMesh; }
			virtual PxTetrahedronMesh*					getCollisionMesh() { return mCollisionMesh; }
			
			PX_FORCE_INLINE const BVTetrahedronMesh*	getCollisionMeshFast() const { return mCollisionMesh; }
			PX_FORCE_INLINE BVTetrahedronMesh*			getCollisionMeshFast() { return mCollisionMesh; }

			virtual const PxTetrahedronMesh*			getSimulationMesh() const { return mSimulationMesh; }
			virtual PxTetrahedronMesh*					getSimulationMesh() { return mSimulationMesh; }

			PX_FORCE_INLINE const TetrahedronMesh*		getSimulationMeshFast() const { return mSimulationMesh; }
			PX_FORCE_INLINE TetrahedronMesh*			getSimulationMeshFast() { return mSimulationMesh; }

			virtual const PxSoftBodyAuxData*	getSoftBodyAuxData() const { return mSoftBodyAuxData; }
			virtual PxSoftBodyAuxData*			getSoftBodyAuxData() { return mSoftBodyAuxData; }

			PX_FORCE_INLINE const SoftBodyAuxData* getSoftBodyAuxDataFast() const { return mSoftBodyAuxData; }
			PX_FORCE_INLINE SoftBodyAuxData*	getSoftBodyAuxDataFast() { return mSoftBodyAuxData; }

		protected:
			TetrahedronMesh*			mSimulationMesh;
			BVTetrahedronMesh*			mCollisionMesh;
			SoftBodyAuxData*			mSoftBodyAuxData;

			MeshFactory*				mMeshFactory;					// PT: changed to pointer for serialization
		};

#if PX_VC
#pragma warning(pop)
#endif

	} // namespace Gu
}

#endif
