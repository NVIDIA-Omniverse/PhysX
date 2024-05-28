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

#ifndef GU_MESH_FACTORY_H
#define GU_MESH_FACTORY_H

#include "foundation/PxIO.h"
#include "foundation/PxHashSet.h"
#include "foundation/PxUserAllocated.h"
#include "geometry/PxTriangleMesh.h"
#include "geometry/PxTetrahedronMesh.h"
#include "geometry/PxConvexMesh.h"
#include "geometry/PxHeightField.h"
#include "geometry/PxBVH.h"
#include "PxPhysXConfig.h"

#include "foundation/PxMutex.h"
#include "foundation/PxArray.h"

// PT: added for platforms that compile the onRefCountZero template immediately
#include "CmUtils.h"
#include "foundation/PxFoundation.h"

namespace physx
{
namespace Gu
{
	class ConvexMesh;
	class HeightField;
	class TriangleMesh;
	class TriangleMeshData;
	class SoftBodyMesh;
	class SoftBodyMeshData;
	class TetrahedronMesh;
	class TetrahedronMeshData;
	class BVH;
	struct ConvexHullInitData;
	class BVHData;

	class MeshFactoryListener
	{
	protected:
		virtual ~MeshFactoryListener(){}
	public:
		virtual void onMeshFactoryBufferRelease(const PxBase* object, PxType type) = 0;
#if PX_SUPPORT_OMNI_PVD
		virtual void onObjectAdd(const PxBase*) {}
		virtual void onObjectRemove(const PxBase*) {}
#endif
	};

	#if PX_VC 
		#pragma warning(push)
		#pragma warning( disable : 4251 ) // class needs to have dll-interface to be used by clients of class
	#endif

	class PX_PHYSX_COMMON_API MeshFactory : public PxUserAllocated
	{
		PX_NOCOPY(MeshFactory)
	public:
										MeshFactory();
	protected:
		virtual							~MeshFactory();

	public:
		void							release();

		// Triangle meshes
		void							addTriangleMesh(Gu::TriangleMesh* np, bool lock=true);
		PxTriangleMesh*					createTriangleMesh(PxInputStream& stream);
		PxTriangleMesh*					createTriangleMesh(void* triangleMeshData);
		bool							removeTriangleMesh(PxTriangleMesh&);
		PxU32							getNbTriangleMeshes()	const;
		PxU32							getTriangleMeshes(PxTriangleMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

		// Tetrahedron meshes
		void							addTetrahedronMesh(Gu::TetrahedronMesh* np, bool lock = true);
		PxTetrahedronMesh*				createTetrahedronMesh(PxInputStream& stream);
		PxTetrahedronMesh*				createTetrahedronMesh(void* tetrahedronMeshData);
		bool							removeTetrahedronMesh(PxTetrahedronMesh&);
		PxU32							getNbTetrahedronMeshes()	const;
		PxU32							getTetrahedronMeshes(PxTetrahedronMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

		// SoftBody meshes
		void							addSoftBodyMesh(Gu::SoftBodyMesh* np, bool lock = true);
		PxSoftBodyMesh*					createSoftBodyMesh(PxInputStream& stream);
		PxSoftBodyMesh*					createSoftBodyMesh(void* tetrahedronMeshData);
		bool							removeSoftBodyMesh(PxSoftBodyMesh&);
		PxU32							getNbSoftBodyMeshes()	const;
		PxU32							getSoftBodyMeshes(PxSoftBodyMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

		// Convexes
		void							addConvexMesh(Gu::ConvexMesh* np, bool lock=true);
		PxConvexMesh*					createConvexMesh(PxInputStream&);
		PxConvexMesh*					createConvexMesh(void* convexMeshData);
		bool							removeConvexMesh(PxConvexMesh&);
		PxU32							getNbConvexMeshes() const;
		PxU32							getConvexMeshes(PxConvexMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

		// Heightfields
		void							addHeightField(Gu::HeightField* np, bool lock=true);
		PxHeightField*					createHeightField(void* heightFieldMeshData);
		PxHeightField*					createHeightField(PxInputStream&);
		bool							removeHeightField(PxHeightField&);
		PxU32							getNbHeightFields()	const;
		PxU32							getHeightFields(PxHeightField** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

		// BVH
		void							addBVH(Gu::BVH* np, bool lock=true);
		PxBVH*							createBVH(PxInputStream&);
		PxBVH*							createBVH(void* bvhData);
		bool							removeBVH(PxBVH&);
		PxU32							getNbBVHs() const;
		PxU32							getBVHs(PxBVH** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

		void							addFactoryListener(MeshFactoryListener& listener);
		void							removeFactoryListener(MeshFactoryListener& listener);
		void							notifyFactoryListener(const PxBase*, PxType typeID);

		bool							remove(PxBase&);

	protected:

		PxTriangleMesh*					createTriangleMesh(Gu::TriangleMeshData& data);
		PxTetrahedronMesh*				createTetrahedronMesh(Gu::TetrahedronMeshData& data);
		PxSoftBodyMesh*					createSoftBodyMesh(Gu::SoftBodyMeshData& data);
		PxConvexMesh*					createConvexMesh(Gu::ConvexHullInitData& data);
		PxBVH*							createBVH(Gu::BVHData& data);

		mutable PxMutex					mTrackingMutex;
	private:
		PxCoalescedHashSet<Gu::TriangleMesh*>		mTriangleMeshes;
		PxCoalescedHashSet<Gu::TetrahedronMesh*>	mTetrahedronMeshes;
		PxCoalescedHashSet<Gu::SoftBodyMesh*>		mSoftBodyMeshes;
		PxCoalescedHashSet<Gu::ConvexMesh*>			mConvexMeshes;
		PxCoalescedHashSet<Gu::HeightField*>		mHeightFields;
		PxCoalescedHashSet<Gu::BVH*>				mBVHs;

		PxArray<MeshFactoryListener*>				mFactoryListeners;

#if PX_SUPPORT_OMNI_PVD
	protected:
		void							notifyListenersAdd(const PxBase*);
		void							notifyListenersRemove(const PxBase*);
#endif
	};
	#if PX_VC 
		 #pragma warning(pop) 
	#endif

		template<typename T>
		PX_INLINE void onRefCountZero(T* object, Gu::MeshFactory* mf, bool cndt, const char* errorMsg)
		{
			if(mf)
			{
				if(cndt || mf->remove(*object))
				{
					const PxType type = object->getConcreteType();
					Cm::deletePxBase(object);
					mf->notifyFactoryListener(object, type);
					return;
				}
	
				// PT: if we reach this point, we didn't find the mesh in the Physics object => don't delete!
				// This prevents deleting the object twice.
				PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, errorMsg);
			}
			else
				Cm::deletePxBase(object);
		}
	}

}
#endif
