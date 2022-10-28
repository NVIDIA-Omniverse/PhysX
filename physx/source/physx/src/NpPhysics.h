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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef NP_PHYSICS_H
#define NP_PHYSICS_H

#include "PxPhysics.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxHashSet.h"
#include "foundation/PxHashMap.h"
#include "GuMeshFactory.h"
#include "NpMaterial.h"
#include "NpFEMSoftBodyMaterial.h"
#include "NpFEMClothMaterial.h"
#include "NpPBDMaterial.h"
#include "NpFLIPMaterial.h"
#include "NpMPMMaterial.h"
#include "NpPhysicsInsertionCallback.h"
#include "NpMaterialManager.h"
#include "ScPhysics.h"

#ifdef LINUX
#include <string.h>
#endif

#if PX_SUPPORT_GPU_PHYSX
#include "device/PhysXIndicator.h"
#endif

#include "PsPvd.h"

#if PX_SUPPORT_OMNI_PVD
class OmniPvdPxSampler;
namespace physx
{
	class PxOmniPvd;
}
#endif

namespace physx
{

#if PX_SUPPORT_PVD
namespace Vd
{
	class PvdPhysicsClient;
}
#endif
	struct NpMaterialIndexTranslator
	{
		NpMaterialIndexTranslator() : indicesNeedTranslation(false) {}

		PxHashMap<PxU16, PxU16>	map;
		bool						indicesNeedTranslation;
	};

	class NpScene;	
	struct PxvOffsetTable;

#if PX_VC
#pragma warning(push)
#pragma warning(disable:4996)	// We have to implement deprecated member functions, do not warn.
#endif

template <typename T> class NpMaterialAccessor;

class NpPhysics : public PxPhysics, public PxUserAllocated
{
	NpPhysics& operator=(const NpPhysics&);
	NpPhysics(const NpPhysics &);

	struct NpDelListenerEntry : public PxUserAllocated
	{
		NpDelListenerEntry(const PxDeletionEventFlags& de, bool restrictedObjSet)
			: flags(de)
			, restrictedObjectSet(restrictedObjSet)
		{
		}

		PxHashSet<const PxBase*> registeredObjects;  // specifically registered objects for deletion events
		PxDeletionEventFlags flags;
		bool restrictedObjectSet;
	};


									NpPhysics(	const PxTolerancesScale& scale, 
												const PxvOffsetTable& pxvOffsetTable,
												bool trackOutstandingAllocations, 
                                                physx::pvdsdk::PsPvd* pvd,
												PxFoundation&,
												physx::PxOmniPvd* omniPvd);
	virtual							~NpPhysics();

public:
	
	static      NpPhysics*			createInstance(	PxU32 version, 
													PxFoundation& foundation, 
													const PxTolerancesScale& scale,
													bool trackOutstandingAllocations,
													physx::pvdsdk::PsPvd* pvd,
													physx::PxOmniPvd* omniPvd);

	static		PxU32			releaseInstance();

	static      NpPhysics&		getInstance() { return *mInstance; }

	virtual     void			release()	PX_OVERRIDE;

	virtual		PxOmniPvd*			getOmniPvd()	PX_OVERRIDE;

	virtual		PxScene*		createScene(const PxSceneDesc&)	PX_OVERRIDE;
				void			releaseSceneInternal(PxScene&);
	virtual		PxU32			getNbScenes()	const	PX_OVERRIDE;
	virtual		PxU32			getScenes(PxScene** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const	PX_OVERRIDE;

	virtual		PxRigidStatic*						createRigidStatic(const PxTransform&)	PX_OVERRIDE;
	virtual		PxRigidDynamic*						createRigidDynamic(const PxTransform&)	PX_OVERRIDE;
	virtual		PxArticulationReducedCoordinate*	createArticulationReducedCoordinate()	PX_OVERRIDE;
	virtual		PxSoftBody*							createSoftBody(PxCudaContextManager& cudaContextManager)	PX_OVERRIDE;
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	virtual		PxHairSystem*				createHairSystem(PxCudaContextManager& cudaContextManager)	PX_OVERRIDE;
	virtual		PxFEMCloth*					createFEMCloth(PxCudaContextManager& cudaContextManager)	PX_OVERRIDE;
#endif
	virtual		PxPBDParticleSystem*		createPBDParticleSystem(PxCudaContextManager& cudaContexManager, PxU32 maxNeighborhood)	PX_OVERRIDE;
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	virtual		PxFLIPParticleSystem*		createFLIPParticleSystem(PxCudaContextManager& cudaContexManager)	PX_OVERRIDE;
	virtual		PxMPMParticleSystem*		createMPMParticleSystem(PxCudaContextManager& cudaContexManager)	PX_OVERRIDE;
	virtual		PxCustomParticleSystem*		createCustomParticleSystem(PxCudaContextManager& cudaContexManager, PxU32 maxNeighborhood)	PX_OVERRIDE;
#endif
	virtual		PxBuffer*					createBuffer(PxU64 byteSize, PxBufferType::Enum bufferType, PxCudaContextManager* cudaContextManager)	PX_OVERRIDE;
	virtual		PxConstraint*				createConstraint(PxRigidActor* actor0, PxRigidActor* actor1, PxConstraintConnector& connector, const PxConstraintShaderTable& shaders, PxU32 dataSize)	PX_OVERRIDE;
	virtual		PxAggregate*				createAggregate(PxU32 maxActors, PxU32 maxShapes, PxAggregateFilterHint filterHint)	PX_OVERRIDE;

	virtual		PxShape*					createShape(const PxGeometry&, PxMaterial*const *, PxU16, bool, PxShapeFlags shapeFlags)	PX_OVERRIDE;
	virtual		PxShape*					createShape(const PxGeometry&, PxFEMSoftBodyMaterial*const *, PxU16, bool, PxShapeFlags shapeFlags)	PX_OVERRIDE;
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	virtual		PxShape*					createShape(const PxGeometry&, PxFEMClothMaterial*const *, PxU16, bool, PxShapeFlags shapeFlags)	PX_OVERRIDE;
#endif
	virtual		PxU32						getNbShapes()	const	PX_OVERRIDE;
	virtual		PxU32						getShapes(PxShape** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const	PX_OVERRIDE;

	virtual		PxMaterial*					createMaterial(PxReal staticFriction, PxReal dynamicFriction, PxReal restitution)	PX_OVERRIDE;
	virtual		PxU32						getNbMaterials() const	PX_OVERRIDE;
	virtual		PxU32						getMaterials(PxMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const	PX_OVERRIDE;

	virtual		PxFEMSoftBodyMaterial*		createFEMSoftBodyMaterial(PxReal youngs, PxReal poissons, PxReal dynamicFriction)	PX_OVERRIDE;
	virtual		PxU32						getNbFEMSoftBodyMaterials() const	PX_OVERRIDE;
	virtual		PxU32						getFEMSoftBodyMaterials(PxFEMSoftBodyMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const	PX_OVERRIDE;

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	virtual		PxFEMClothMaterial*			createFEMClothMaterial(PxReal youngs, PxReal poissons, PxReal dynamicFriction)	PX_OVERRIDE;
#endif
	virtual		PxU32						getNbFEMClothMaterials() const	PX_OVERRIDE;
	virtual		PxU32						getFEMClothMaterials(PxFEMClothMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const	PX_OVERRIDE;

	virtual		PxPBDMaterial*				createPBDMaterial(PxReal friction, PxReal damping, PxReal adhesion, PxReal viscosity, PxReal vorticityConfinement, PxReal surfaceTension, PxReal cohesion, PxReal lift, PxReal drag, PxReal cflCoefficient, PxReal gravityScale)	PX_OVERRIDE;
	virtual		PxU32						getNbPBDMaterials() const	PX_OVERRIDE;
	virtual		PxU32						getPBDMaterials(PxPBDMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const	PX_OVERRIDE;
	
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	virtual		PxFLIPMaterial*				createFLIPMaterial(PxReal friction, PxReal damping, PxReal maxVelocity, PxReal viscosity, PxReal gravityScale)	PX_OVERRIDE;
	virtual		PxU32						getNbFLIPMaterials() const	PX_OVERRIDE;
	virtual		PxU32						getFLIPMaterials(PxFLIPMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const	PX_OVERRIDE;
	
	virtual		PxMPMMaterial*				createMPMMaterial(PxReal friction, PxReal damping, PxReal maxVelocity, bool isPlastic, PxReal youngsModulus, PxReal poissons, PxReal hardening, PxReal criticalCompression, PxReal criticalStretch, PxReal tensileDamageSensitivity, PxReal compressiveDamageSensitivity, PxReal attractiveForceResidual, PxReal gravityScale)	PX_OVERRIDE;
	virtual		PxU32						getNbMPMMaterials() const	PX_OVERRIDE;
	virtual		PxU32						getMPMMaterials(PxMPMMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const	PX_OVERRIDE;

	virtual		PxCustomMaterial*			createCustomMaterial(void* gpuBuffer)	PX_OVERRIDE;
	// PT: what's going on here? Incomplete API?
	virtual		PxU32						getNbCustomMaterials() const	/*PX_OVERRIDE*/;
	virtual		PxU32						getCustomMaterials(PxCustomMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0) const	/*PX_OVERRIDE*/;
#endif
	virtual		PxTriangleMesh*				createTriangleMesh(PxInputStream&)	PX_OVERRIDE;
	virtual		PxU32						getNbTriangleMeshes()	const	PX_OVERRIDE;
	virtual		PxU32						getTriangleMeshes(PxTriangleMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex=0)	const	PX_OVERRIDE;

	virtual		PxTetrahedronMesh*			createTetrahedronMesh(PxInputStream&)	PX_OVERRIDE;
	virtual		PxU32						getNbTetrahedronMeshes()	const	PX_OVERRIDE;
	virtual		PxU32						getTetrahedronMeshes(PxTetrahedronMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex = 0)	const	PX_OVERRIDE;

	virtual		PxSoftBodyMesh*				createSoftBodyMesh(PxInputStream&)	PX_OVERRIDE;

	virtual		PxHeightField*				createHeightField(PxInputStream& stream)	PX_OVERRIDE;
	virtual		PxU32						getNbHeightFields()	const	PX_OVERRIDE;
	virtual		PxU32						getHeightFields(PxHeightField** userBuffer, PxU32 bufferSize, PxU32 startIndex=0)	const	PX_OVERRIDE;

	virtual		PxConvexMesh*				createConvexMesh(PxInputStream&)	PX_OVERRIDE;
	virtual		PxU32						getNbConvexMeshes() const	PX_OVERRIDE;
	virtual		PxU32						getConvexMeshes(PxConvexMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const	PX_OVERRIDE;

	virtual		PxBVH*						createBVH(PxInputStream&)	PX_OVERRIDE;
	virtual		PxU32						getNbBVHs() const	PX_OVERRIDE;
	virtual		PxU32						getBVHs(PxBVH** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const	PX_OVERRIDE;

	virtual		PxParticleBuffer*			createParticleBuffer(PxU32 maxParticles, PxU32 maxVolumes, PxCudaContextManager* cudaContexManager)	PX_OVERRIDE;
	virtual		PxParticleAndDiffuseBuffer*	createParticleAndDiffuseBuffer(PxU32 maxParticles, PxU32 maxVolumes, PxU32 maxDiffuseParticles, PxCudaContextManager* cudaContexManager)	PX_OVERRIDE;
	virtual		PxParticleClothBuffer*		createParticleClothBuffer(PxU32 maxParticles, PxU32 maxNumVolumes, PxU32 maxNumCloths, PxU32 maxNumTriangles, PxU32 maxNumSprings, PxCudaContextManager* cudaContexManager)	PX_OVERRIDE;
	virtual		PxParticleRigidBuffer*		createParticleRigidBuffer(PxU32 maxParticles, PxU32 maxNumVolumes, PxU32 maxNumRigids, PxCudaContextManager* cudaContexManager)	PX_OVERRIDE;

#if PX_SUPPORT_GPU_PHYSX
	void							registerPhysXIndicatorGpuClient();
	void							unregisterPhysXIndicatorGpuClient();
#else
	PX_FORCE_INLINE void			registerPhysXIndicatorGpuClient() {}
	PX_FORCE_INLINE void			unregisterPhysXIndicatorGpuClient() {}
#endif

	virtual		PxPruningStructure*			createPruningStructure(PxRigidActor*const* actors, PxU32 nbActors)	PX_OVERRIDE;

	virtual		const PxTolerancesScale&	getTolerancesScale() const	PX_OVERRIDE;

	virtual		PxFoundation&		getFoundation()	PX_OVERRIDE;

	PX_INLINE	NpScene*			getScene(PxU32 i) const { return mSceneArray[i]; }
	PX_INLINE	PxU32				getNumScenes() const { return mSceneArray.size(); }
#if PX_CHECKED
	static PX_INLINE	void		heightfieldsAreRegistered() { mHeightFieldsRegistered = true;  }
#endif

	virtual		void				registerDeletionListener(PxDeletionListener& observer, const PxDeletionEventFlags& deletionEvents, bool restrictedObjectSet)	PX_OVERRIDE;
	virtual		void				unregisterDeletionListener(PxDeletionListener& observer)	PX_OVERRIDE;
	virtual		void				registerDeletionListenerObjects(PxDeletionListener& observer, const PxBase* const* observables, PxU32 observableCount)	PX_OVERRIDE;
	virtual		void				unregisterDeletionListenerObjects(PxDeletionListener& observer, const PxBase* const* observables, PxU32 observableCount)	PX_OVERRIDE;

				void				notifyDeletionListeners(const PxBase*, void* userData, PxDeletionEventFlag::Enum deletionEvent);
	PX_FORCE_INLINE void			notifyDeletionListenersUserRelease(const PxBase* b, void* userData) { notifyDeletionListeners(b, userData, PxDeletionEventFlag::eUSER_RELEASE); }
	PX_FORCE_INLINE void			notifyDeletionListenersMemRelease(const PxBase* b, void* userData) { notifyDeletionListeners(b, userData, PxDeletionEventFlag::eMEMORY_RELEASE); }

	virtual		PxInsertionCallback&	getPhysicsInsertionCallback() PX_OVERRIDE	{ return mObjectInsertion; }

				void				removeMaterialFromTable(NpMaterial&);
				void				updateMaterial(NpMaterial&);
				bool				sendMaterialTable(NpScene&);

				NpMaterialManager<NpMaterial>&				getMaterialManager()	{	return mMasterMaterialManager;	}
				NpMaterialManager<NpFEMSoftBodyMaterial>&	getFEMSoftBodyMaterialManager() { return mMasterFEMSoftBodyMaterialManager; }
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
				NpMaterialManager<NpFEMClothMaterial>&		getFEMClothMaterialManager() { return mMasterFEMClothMaterialManager; }
#endif
				NpMaterialManager<NpPBDMaterial>&			getPBDMaterialManager() { return mMasterPBDMaterialManager; }
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
				NpMaterialManager<NpFLIPMaterial>&			getFLIPMaterialManager() { return mMasterFLIPMaterialManager; }
				NpMaterialManager<NpMPMMaterial>&			getMPMMaterialManager() { return mMasterMPMMaterialManager; }
				NpMaterialManager<NpCustomMaterial>&		getCustomMaterialManager() { return mMasterCustomMaterialManager; }
#endif
				/*template <typename T>
				NpMaterialManager<T>* getMaterialManagerT();
				template <>
				NpMaterialManager<NpMaterial>*		getMaterialManagerT() { return &mMasterMaterialManager; }
				template <>
				NpMaterialManager<NpFEMSoftBodyMaterial>*		getMaterialManagerT() { return &mMasterFEMSoftBodyMaterialManager; }
				template <>
				NpMaterialManager<NpFEMClothMaterial>*		getMaterialManagerT() { return &mMasterFEMClothMaterialManager; }*/

				NpMaterial*									addMaterial(NpMaterial* np);

				void										removeFEMSoftBodyMaterialFromTable(NpFEMSoftBodyMaterial&);
				void										updateFEMSoftBodyMaterial(NpFEMSoftBodyMaterial&);
								
				NpFEMSoftBodyMaterial*						addFEMMaterial(NpFEMSoftBodyMaterial* np);

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
				void										removeFEMClothMaterialFromTable(NpFEMClothMaterial&);
				void										updateFEMClothMaterial(NpFEMClothMaterial&);
				NpFEMClothMaterial*							addFEMClothMaterial(NpFEMClothMaterial* np);
#endif
				void										removePBDMaterialFromTable(NpPBDMaterial&);
				void										updatePBDMaterial(NpPBDMaterial&);
				NpPBDMaterial*								addPBDMaterial(NpPBDMaterial* np);
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
				void										removeFLIPMaterialFromTable(NpFLIPMaterial&);
				void										updateFLIPMaterial(NpFLIPMaterial&);
				NpFLIPMaterial*								addFLIPMaterial(NpFLIPMaterial* np);
				
				void										removeMPMMaterialFromTable(NpMPMMaterial&);
				void										updateMPMMaterial(NpMPMMaterial&);
				NpMPMMaterial*								addMPMMaterial(NpMPMMaterial* np);

				void										removeCustomMaterialFromTable(NpCustomMaterial&);
				void										updateCustomMaterial(NpCustomMaterial&);
				NpCustomMaterial*							addCustomMaterial(NpCustomMaterial* np);
#endif
	static		void				initOffsetTables(PxvOffsetTable& pxvOffsetTable);

	static bool apiReentryLock;

#if PX_SUPPORT_OMNI_PVD
				OmniPvdPxSampler*	mOmniPvdSampler;
				PxOmniPvd*			mOmniPvd;
#endif

private:
				typedef PxCoalescedHashMap<PxDeletionListener*, NpDelListenerEntry*> DeletionListenerMap;

				PxArray<NpScene*>	mSceneArray;

				Sc::Physics										mPhysics;
				NpMaterialManager<NpMaterial>					mMasterMaterialManager;
				NpMaterialManager<NpFEMSoftBodyMaterial>		mMasterFEMSoftBodyMaterialManager;
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
				NpMaterialManager<NpFEMClothMaterial>			mMasterFEMClothMaterialManager;
#endif
				NpMaterialManager<NpPBDMaterial>				mMasterPBDMaterialManager;
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
				NpMaterialManager<NpFLIPMaterial>				mMasterFLIPMaterialManager;
				NpMaterialManager<NpMPMMaterial>				mMasterMPMMaterialManager;
				NpMaterialManager<NpCustomMaterial>				mMasterCustomMaterialManager;
#endif
				NpPhysicsInsertionCallback	mObjectInsertion;

				struct MeshDeletionListener: public Gu::MeshFactoryListener
				{
					void onMeshFactoryBufferRelease(const PxBase* object, PxType type)
					{
						PX_UNUSED(type);
						NpPhysics::getInstance().notifyDeletionListeners(object, NULL, PxDeletionEventFlag::eMEMORY_RELEASE);
					}
				};

				PxMutex									mDeletionListenerMutex;
				DeletionListenerMap						mDeletionListenerMap;
				MeshDeletionListener					mDeletionMeshListener;
				bool									mDeletionListenersExist;

				PxMutex									mSceneAndMaterialMutex;				// guarantees thread safety for API calls related to scene and material containers

				PxFoundation&							mFoundation;

#if PX_SUPPORT_GPU_PHYSX
				PhysXIndicator							mPhysXIndicator;
				PxU32									mNbRegisteredGpuClients;
				PxMutex									mPhysXIndicatorMutex;
#endif
#if PX_SUPPORT_PVD	
				physx::pvdsdk::PsPvd*  mPvd;
                Vd::PvdPhysicsClient*   mPvdPhysicsClient;
#endif

	static		PxU32				mRefCount;
	static		NpPhysics*			mInstance;

#if PX_CHECKED
	static		bool				mHeightFieldsRegistered;	//just for error checking
#endif

	friend class NpCollection;

#if PX_SUPPORT_OMNI_PVD
	class OmniPvdListener : public physx::NpFactoryListener
	{
	public:
		virtual void onMeshFactoryBufferRelease(const PxBase*, PxType) {}
		virtual void onObjectAdd(const PxBase*);
		virtual void onObjectRemove(const PxBase*);
	}
	mOmniPvdListener;
#endif
};

template <> class NpMaterialAccessor<NpMaterial>
{
public:
	static NpMaterialManager<NpMaterial>& getMaterialManager(NpPhysics& physics)
	{
		return physics.getMaterialManager();
	}
};

template <> class NpMaterialAccessor<NpFEMSoftBodyMaterial>
{
public:
	static NpMaterialManager<NpFEMSoftBodyMaterial>& getMaterialManager(NpPhysics& physics)
	{
		return physics.getFEMSoftBodyMaterialManager();
	}
};

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
template <> class NpMaterialAccessor<NpFEMClothMaterial>
{
public:
	static NpMaterialManager<NpFEMClothMaterial>& getMaterialManager(NpPhysics& physics)
	{
		return physics.getFEMClothMaterialManager();
	}
};
#endif

template <> class NpMaterialAccessor<NpPBDMaterial>
{
public:
	static NpMaterialManager<NpPBDMaterial>& getMaterialManager(NpPhysics& physics)
	{
		return physics.getPBDMaterialManager();
	}
};

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
template <> class NpMaterialAccessor<NpFLIPMaterial>
{
public:
	static NpMaterialManager<NpFLIPMaterial>& getMaterialManager(NpPhysics& physics)
	{
		return physics.getFLIPMaterialManager();
	}
};

template <> class NpMaterialAccessor<NpMPMMaterial>
{
public:
	static NpMaterialManager<NpMPMMaterial>& getMaterialManager(NpPhysics& physics)
	{
		return physics.getMPMMaterialManager();
	}
};

template <> class NpMaterialAccessor<NpCustomMaterial>
{
public:
	static NpMaterialManager<NpCustomMaterial>& getMaterialManager(NpPhysics& physics)
	{
		return physics.getCustomMaterialManager();
	}
};
#endif

#if PX_VC
#pragma warning(pop)
#endif
}

#endif
