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

#include "NpPhysics.h"

// PX_SERIALIZATION
#include "foundation/PxProfiler.h"
#include "foundation/PxIO.h"
#include "foundation/PxErrorCallback.h"
#include "foundation/PxString.h"
#include "foundation/PxPhysicsVersion.h"
#include "common/PxTolerancesScale.h"
#include "CmCollection.h"
#include "CmUtils.h"
#include "NpRigidStatic.h"
#include "NpRigidDynamic.h"
#if PX_SUPPORT_GPU_PHYSX
#include "NpSoftBody.h"
#include "NpParticleSystem.h"
#include "NpHairSystem.h"
#endif
#include "NpArticulationReducedCoordinate.h"
#include "NpArticulationLink.h"
#include "NpMaterial.h"
#include "GuHeightFieldData.h"
#include "GuHeightField.h"
#include "GuConvexMesh.h"
#include "GuTriangleMesh.h"
#include "foundation/PxIntrinsics.h"
#include "PxvGlobals.h"		// dynamic registration of HFs & articulations in LL
#include "GuOverlapTests.h" // dynamic registration of HFs in Gu
#include "PxDeletionListener.h"
#include "PxPhysicsSerialization.h"
#include "PvdPhysicsClient.h"

#if PX_SUPPORT_OMNI_PVD
#include "omnipvd/NpOmniPvd.h"
#include "OmniPvdWriter.h"
#endif

#if PX_SUPPORT_GPU_PHYSX
#include "PxPhysXGpu.h"
#endif

//~PX_SERIALIZATION

#include "NpFactory.h"

#if PX_SWITCH
#include "switch/NpMiddlewareInfo.h"
#endif

#if PX_SUPPORT_OMNI_PVD
#	define OMNI_PVD_NOTIFY_ADD(OBJECT) mOmniPvdListener.onObjectAdd(OBJECT)
#	define OMNI_PVD_NOTIFY_REMOVE(OBJECT) mOmniPvdListener.onObjectRemove(OBJECT)
#else
#	define OMNI_PVD_NOTIFY_ADD(OBJECT)
#	define OMNI_PVD_NOTIFY_REMOVE(OBJECT)
#endif

using namespace physx;
using namespace Cm;

bool		NpPhysics::apiReentryLock	= false;
NpPhysics*	NpPhysics::mInstance		= NULL;
PxU32		NpPhysics::mRefCount		= 0;

#if PX_CHECKED
bool		NpPhysics::mHeightFieldsRegistered = false;	//just for error checking
#endif

NpPhysics::NpPhysics(const PxTolerancesScale& scale, const PxvOffsetTable& pxvOffsetTable, bool trackOutstandingAllocations, pvdsdk::PsPvd* pvd, PxFoundation& foundation, PxOmniPvd* omniPvd) :
	mSceneArray					("physicsSceneArray"),
	mPhysics					(scale, pxvOffsetTable),
	mDeletionListenersExist		(false),
	mFoundation					(foundation)
#if PX_SUPPORT_GPU_PHYSX
	, mNbRegisteredGpuClients	(0)
#endif	
{
	PX_UNUSED(trackOutstandingAllocations);

	//mMasterMaterialTable.reserve(10);
		
#if PX_SUPPORT_PVD	
	mPvd = pvd;
	if(pvd)
	{		
	    mPvdPhysicsClient = PX_NEW(Vd::PvdPhysicsClient)(mPvd);	
	    foundation.registerErrorCallback(*mPvdPhysicsClient);
		foundation.registerAllocationListener(*mPvd);	
	}
	else
	{		
		mPvdPhysicsClient = NULL;
	}
#else
	PX_UNUSED(pvd);
#endif

	// PT: please leave this commented-out block here.
/*
	printf("sizeof(NpScene)          = %d\n", sizeof(NpScene));
	printf("sizeof(NpShape)          = %d\n", sizeof(NpShape));
	printf("sizeof(NpActor)          = %d\n", sizeof(NpActor));
	printf("sizeof(NpRigidStatic)    = %d\n", sizeof(NpRigidStatic));
	printf("sizeof(NpRigidDynamic)   = %d\n", sizeof(NpRigidDynamic));
	printf("sizeof(NpMaterial)       = %d\n", sizeof(NpMaterial));
	printf("sizeof(NpConstraint)     = %d\n", sizeof(NpConstraint));
	printf("sizeof(NpAggregate)      = %d\n", sizeof(NpAggregate));
	printf("sizeof(NpArticulationRC) = %d\n", sizeof(NpArticulationReducedCoordinate));

	printf("sizeof(GeometryUnion)            = %d\n", sizeof(GeometryUnion));
	printf("sizeof(PxGeometry)               = %d\n", sizeof(PxGeometry));
	printf("sizeof(PxPlaneGeometry)          = %d\n", sizeof(PxPlaneGeometry));
	printf("sizeof(PxSphereGeometry)         = %d\n", sizeof(PxSphereGeometry));
	printf("sizeof(PxCapsuleGeometry)        = %d\n", sizeof(PxCapsuleGeometry));
	printf("sizeof(PxBoxGeometry)            = %d\n", sizeof(PxBoxGeometry));
	printf("sizeof(PxConvexMeshGeometry)     = %d\n", sizeof(PxConvexMeshGeometry));
	printf("sizeof(PxConvexMeshGeometryLL)   = %d\n", sizeof(PxConvexMeshGeometryLL));
	printf("sizeof(PxTriangleMeshGeometry)   = %d\n", sizeof(PxTriangleMeshGeometry));
	printf("sizeof(PxTriangleMeshGeometryLL) = %d\n", sizeof(PxTriangleMeshGeometryLL));
	printf("sizeof(PxsShapeCore)             = %d\n", sizeof(PxsShapeCore));
*/

#if PX_SUPPORT_OMNI_PVD
	mOmniPvdSampler = NULL;
	mOmniPvd = NULL;
	if (omniPvd)
	{
		OmniPvdWriter* omniWriter = omniPvd->getWriter();
		if (omniWriter && omniWriter->getWriteStream())
		{
			mOmniPvdSampler = PX_NEW(::OmniPvdPxSampler)();
			mOmniPvd = omniPvd;
			NpOmniPvd* npOmniPvd = static_cast<NpOmniPvd*>(mOmniPvd);
			NpOmniPvd::incRefCount();
			npOmniPvd->mPhysXSampler = mOmniPvdSampler; // Dirty hack to do startSampling from PxOmniPvd
			mOmniPvdSampler->setOmniPvdWriter(omniPvd->getWriter());
		}
	}
#else
	PX_UNUSED(omniPvd);
#endif
}

NpPhysics::~NpPhysics()
{
	// Release all scenes in case the user didn't do it
	PxU32 nbScenes = mSceneArray.size();
	NpScene** scenes = mSceneArray.begin();
	for(PxU32 i=0;i<nbScenes;i++)
		PX_DELETE(scenes[i]);
	mSceneArray.clear();

	//PxU32 matCount = mMasterMaterialTable.size();
	//while (mMasterMaterialTable.size() > 0)
	//{
	//	// It's done this way since the material destructor removes the material from the table and adjusts indices

	//	PX_ASSERT(mMasterMaterialTable[0]->getRefCount() == 1);
	//	mMasterMaterialTable[0]->decRefCount();
	//}
	//mMasterMaterialTable.clear();

	mMasterMaterialManager.releaseMaterials();
	mMasterFEMSoftBodyMaterialManager.releaseMaterials();
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	mMasterFEMClothMaterialManager.releaseMaterials();
#endif
	mMasterPBDMaterialManager.releaseMaterials();
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	mMasterFLIPMaterialManager.releaseMaterials();
	mMasterMPMMaterialManager.releaseMaterials();
	mMasterCustomMaterialManager.releaseMaterials();
#endif

#if PX_SUPPORT_PVD	
	if(mPvd)
	{	
		mPvdPhysicsClient->destroyPvdInstance(this);
		mPvd->removeClient(mPvdPhysicsClient);
		mFoundation.deregisterErrorCallback(*mPvdPhysicsClient);
		PX_DELETE(mPvdPhysicsClient);	
		mFoundation.deregisterAllocationListener(*mPvd);
	}	
#endif

	const DeletionListenerMap::Entry* delListenerEntries = mDeletionListenerMap.getEntries();
	const PxU32 delListenerEntryCount = mDeletionListenerMap.size();
	for(PxU32 i=0; i < delListenerEntryCount; i++)
	{
		NpDelListenerEntry* listener = delListenerEntries[i].second;
		PX_DELETE(listener);
	}
	mDeletionListenerMap.clear();

#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_DESTROY(physics, static_cast<PxPhysics&>(*this))
	PX_DELETE(mOmniPvdSampler);
	if (mOmniPvd)
	{
		NpOmniPvd::decRefCount();
	}
#endif
}

PxOmniPvd* NpPhysics::getOmniPvd()
{
#if PX_SUPPORT_OMNI_PVD
	return mOmniPvd;
#else
	return NULL;
#endif
}

void NpPhysics::initOffsetTables(PxvOffsetTable& pxvOffsetTable)
{
	// init offset tables for Pxs/Sc/Px conversions
	{
		Sc::OffsetTable& offsetTable =  Sc::gOffsetTable;
		offsetTable.scRigidStatic2PxActor				= -ptrdiff_t(NpRigidStatic::getCoreOffset());
		offsetTable.scRigidDynamic2PxActor				= -ptrdiff_t(NpRigidDynamic::getCoreOffset());
		offsetTable.scArticulationLink2PxActor			= -ptrdiff_t(NpArticulationLink::getCoreOffset());
#if PX_SUPPORT_GPU_PHYSX
		offsetTable.scSoftBody2PxActor					= -ptrdiff_t(NpSoftBody::getCoreOffset());
		offsetTable.scPBDParticleSystem2PxActor			= -ptrdiff_t(NpPBDParticleSystem::getCoreOffset());
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
		offsetTable.scFLIPParticleSystem2PxActor		= -ptrdiff_t(NpFLIPParticleSystem::getCoreOffset());
		offsetTable.scMPMParticleSystem2PxActor			= -ptrdiff_t(NpMPMParticleSystem::getCoreOffset());
		offsetTable.scCustomParticleSystem2PxActor		= -ptrdiff_t(NpCustomParticleSystem::getCoreOffset());
		offsetTable.scHairSystem2PxActor				= -ptrdiff_t(NpHairSystem::getCoreOffset());
#endif
#endif
		offsetTable.scArticulationRC2Px					= -ptrdiff_t(NpArticulationReducedCoordinate::getCoreOffset());
		offsetTable.scArticulationJointRC2Px			= -ptrdiff_t(NpArticulationJointReducedCoordinate::getCoreOffset());
		offsetTable.scConstraint2Px						= -ptrdiff_t(NpConstraint::getCoreOffset());
		offsetTable.scShape2Px							= -ptrdiff_t(NpShape::getCoreOffset());

		for(PxU32 i=0;i<PxActorType::eACTOR_COUNT;i++)
			offsetTable.scCore2PxActor[i] = 0;
		offsetTable.scCore2PxActor[PxActorType::eRIGID_STATIC] = offsetTable.scRigidStatic2PxActor;
		offsetTable.scCore2PxActor[PxActorType::eRIGID_DYNAMIC] = offsetTable.scRigidDynamic2PxActor;
		offsetTable.scCore2PxActor[PxActorType::eARTICULATION_LINK] = offsetTable.scArticulationLink2PxActor;
		offsetTable.scCore2PxActor[PxActorType::eSOFTBODY] = offsetTable.scSoftBody2PxActor;
		offsetTable.scCore2PxActor[PxActorType::ePBD_PARTICLESYSTEM] = offsetTable.scPBDParticleSystem2PxActor;
		offsetTable.scCore2PxActor[PxActorType::eFLIP_PARTICLESYSTEM] = offsetTable.scFLIPParticleSystem2PxActor;
		offsetTable.scCore2PxActor[PxActorType::eMPM_PARTICLESYSTEM] = offsetTable.scMPMParticleSystem2PxActor;
		offsetTable.scCore2PxActor[PxActorType::eCUSTOM_PARTICLESYSTEM] = offsetTable.scCustomParticleSystem2PxActor;
		offsetTable.scCore2PxActor[PxActorType::eHAIRSYSTEM] = offsetTable.scHairSystem2PxActor;
	}
	{
		Sc::OffsetTable& scOffsetTable = Sc::gOffsetTable;
		pxvOffsetTable.pxsShapeCore2PxShape			= scOffsetTable.scShape2Px				- ptrdiff_t(Sc::ShapeCore::getCoreOffset());
		pxvOffsetTable.pxsRigidCore2PxRigidBody		= scOffsetTable.scRigidDynamic2PxActor	- ptrdiff_t(Sc::BodyCore::getCoreOffset());
		pxvOffsetTable.pxsRigidCore2PxRigidStatic	= scOffsetTable.scRigidStatic2PxActor	- ptrdiff_t(Sc::StaticCore::getCoreOffset());
	}
}

NpPhysics* NpPhysics::createInstance(PxU32 version, PxFoundation& foundation, const PxTolerancesScale& scale, bool trackOutstandingAllocations, pvdsdk::PsPvd* pvd, PxOmniPvd* omniPvd)
{
#if PX_SWITCH
	NpSetMiddlewareInfo();  // register middleware info such that PhysX usage can be tracked
#endif
	
	if (version!=PX_PHYSICS_VERSION) 
	{
		char buffer[256];
		Pxsnprintf(buffer, 256, "Wrong version: PhysX version is 0x%08x, tried to create 0x%08x", PX_PHYSICS_VERSION, version);
		foundation.getErrorCallback().reportError(PxErrorCode::eINVALID_PARAMETER, buffer, __FILE__, __LINE__);
		return NULL;
	}

	if (!scale.isValid())
	{
		foundation.getErrorCallback().reportError(PxErrorCode::eINVALID_PARAMETER, "Scale invalid.\n", __FILE__, __LINE__);
		return NULL; 
	}

	if(0 == mRefCount)
	{
		PX_ASSERT(&foundation == &PxGetFoundation());

		PxIncFoundationRefCount();

		// init offset tables for Pxs/Sc/Px conversions
		PxvOffsetTable pxvOffsetTable;
		initOffsetTables(pxvOffsetTable);

		//SerialFactory::createInstance();
		mInstance = PX_NEW (NpPhysics)(scale, pxvOffsetTable, trackOutstandingAllocations, pvd, foundation, omniPvd);
		NpFactory::createInstance();

#if PX_SUPPORT_OMNI_PVD
		if (omniPvd)
			NpFactory::getInstance().addFactoryListener(mInstance->mOmniPvdListener);
#endif

#if PX_SUPPORT_PVD			
	    if(pvd)
		{			
			NpFactory::getInstance().setNpFactoryListener( *mInstance->mPvdPhysicsClient );					
			pvd->addClient(mInstance->mPvdPhysicsClient);
		}
#endif

		NpFactory::getInstance().addFactoryListener(mInstance->mDeletionMeshListener);
	}
	++mRefCount;

	return mInstance;
}

PxU32 NpPhysics::releaseInstance()
{
	PX_ASSERT(mRefCount > 0);
	if (--mRefCount) 
		return mRefCount;

#if PX_SUPPORT_PVD	
	if(mInstance->mPvd)
	{	
		NpFactory::getInstance().removeFactoryListener( *mInstance->mPvdPhysicsClient );		
	}
#endif
	
	NpFactory::destroyInstance();

	PX_ASSERT(mInstance);
	PX_DELETE(mInstance);

	PxDecFoundationRefCount();

	return mRefCount;
}

void NpPhysics::release()
{
	NpPhysics::releaseInstance();
}

PxScene* NpPhysics::createScene(const PxSceneDesc& desc)
{
	PX_CHECK_AND_RETURN_NULL(desc.isValid(), "Physics::createScene: desc.isValid() is false!");

	const PxTolerancesScale& scale = mPhysics.getTolerancesScale();
	const PxTolerancesScale& descScale = desc.getTolerancesScale();
	PX_UNUSED(scale);
	PX_UNUSED(descScale);
	PX_CHECK_AND_RETURN_NULL((descScale.length == scale.length) && (descScale.speed == scale.speed), "Physics::createScene: PxTolerancesScale must be the same as used for creation of PxPhysics!");

	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);  // done here because scene constructor accesses profiling manager of the SDK

	NpScene* npScene = PX_NEW (NpScene)(desc, *this);
	if(!npScene)
	{
		mFoundation.error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "Unable to create scene.");
		return NULL;
	}
	if(!npScene->getTaskManagerFast())
	{
		mFoundation.error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "Unable to create scene. Task manager creation failed.");
		return NULL;
	}

	npScene->loadFromDesc(desc);

#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_ADD(physics, scenes, static_cast<PxPhysics&>(*this), static_cast<PxScene&>(*npScene))
#endif

#if PX_SUPPORT_PVD
	if(mPvd)
	{
		npScene->getScenePvdClientInternal().setPsPvd(mPvd);		
		mPvd->addClient(&npScene->getScenePvdClientInternal());
	}
#endif

	if (!sendMaterialTable(*npScene) || !npScene->getScScene().isValid())
	{
		PX_DELETE(npScene);
		mFoundation.error(PxErrorCode::eOUT_OF_MEMORY, __FILE__, __LINE__, "Unable to create scene.");
		return NULL;
	}

	mSceneArray.pushBack(npScene);
	return npScene;
}

void NpPhysics::releaseSceneInternal(PxScene& scene)
{
	NpScene* pScene =  static_cast<NpScene*>(&scene);

#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_REMOVE(physics, scenes, static_cast<PxPhysics&>(*this), scene)
#endif

	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);
	for(PxU32 i=0;i<mSceneArray.size();i++)
	{
		if(mSceneArray[i]==pScene)
		{
			mSceneArray.replaceWithLast(i);
			PX_DELETE(pScene);
			return;
		}
	}
}

PxU32 NpPhysics::getNbScenes() const
{
	PxMutex::ScopedLock lock(const_cast<PxMutex&>(mSceneAndMaterialMutex));
	return mSceneArray.size();
}

PxU32 NpPhysics::getScenes(PxScene** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	PxMutex::ScopedLock lock(const_cast<PxMutex&>(mSceneAndMaterialMutex));
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mSceneArray.begin(), mSceneArray.size());
}

PxRigidStatic* NpPhysics::createRigidStatic(const PxTransform& globalPose)
{
	PX_CHECK_AND_RETURN_NULL(globalPose.isSane(), "PxPhysics::createRigidStatic: invalid transform");
	return NpFactory::getInstance().createRigidStatic(globalPose.getNormalized());
}

PxShape* NpPhysics::createShape(const PxGeometry& geometry, PxMaterial*const * materials, PxU16 materialCount, bool isExclusive, PxShapeFlags shapeFlags)
{
	PX_CHECK_AND_RETURN_NULL(materials, "createShape: material pointer is NULL");
	PX_CHECK_AND_RETURN_NULL(materialCount>0, "createShape: material count is zero");

#if PX_CHECKED
	const bool isHeightfield = geometry.getType() == PxGeometryType::eHEIGHTFIELD;
	if (isHeightfield)
		{
		PX_CHECK_AND_RETURN_NULL(mHeightFieldsRegistered, "NpPhysics::createShape: Creating Heightfield shape without having called PxRegister[Unified]HeightFields()!");
		}
	const bool hasMeshTypeGeom = isHeightfield || (geometry.getType() == PxGeometryType::eTRIANGLEMESH) || (geometry.getType() == PxGeometryType::eTETRAHEDRONMESH);
	PX_CHECK_AND_RETURN_NULL(!(hasMeshTypeGeom && (shapeFlags & PxShapeFlag::eTRIGGER_SHAPE)), "NpPhysics::createShape: triangle mesh/heightfield/tetrahedron mesh triggers are not supported!");
	PX_CHECK_AND_RETURN_NULL(!((shapeFlags & PxShapeFlag::eSIMULATION_SHAPE) && (shapeFlags & PxShapeFlag::eTRIGGER_SHAPE)), "NpPhysics::createShape: shapes cannot simultaneously be trigger shapes and simulation shapes.");
#endif

	return NpFactory::getInstance().createShape(geometry, shapeFlags, materials, materialCount, isExclusive);
}

PxShape* NpPhysics::createShape(const PxGeometry& geometry, PxFEMSoftBodyMaterial*const * materials, PxU16 materialCount, bool isExclusive, PxShapeFlags shapeFlags)
{
	PX_CHECK_AND_RETURN_NULL(materials, "createShape: material pointer is NULL");
	PX_CHECK_AND_RETURN_NULL(materialCount > 0, "createShape: material count is zero");
	PX_CHECK_AND_RETURN_NULL(geometry.getType() == PxGeometryType::eTETRAHEDRONMESH, "createShape: soft bodies only accept PxTetrahedronMeshGeometry");
	PX_CHECK_AND_RETURN_NULL(shapeFlags & PxShapeFlag::eSIMULATION_SHAPE, "createShape: soft body shapes must be simulation shapes");

	return NpFactory::getInstance().createShape(geometry, shapeFlags, materials, materialCount, isExclusive);
}

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
PxShape* NpPhysics::createShape(const PxGeometry& geometry, PxFEMClothMaterial*const * materials, PxU16 materialCount, bool isExclusive, PxShapeFlags shapeFlags)
{
	PX_CHECK_AND_RETURN_NULL(materials, "createShape: material pointer is NULL");
	PX_CHECK_AND_RETURN_NULL(materialCount > 0, "createShape: material count is zero");
	PX_CHECK_AND_RETURN_NULL(geometry.getType() == PxGeometryType::eTRIANGLEMESH, "createShape: cloth only accept PxTriangleMeshGeometry");
	PX_CHECK_AND_RETURN_NULL(shapeFlags & PxShapeFlag::eSIMULATION_SHAPE, "createShape: cloth shapes must be simulation shapes");

	return NpFactory::getInstance().createShape(geometry, shapeFlags, materials, materialCount, isExclusive);
}
#endif

PxU32 NpPhysics::getNbShapes()	const
{
	return NpFactory::getInstance().getNbShapes();
}

PxU32 NpPhysics::getShapes(PxShape** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const
{
	return NpFactory::getInstance().getShapes(userBuffer, bufferSize, startIndex);
}

PxRigidDynamic* NpPhysics::createRigidDynamic(const PxTransform& globalPose)
{
	PX_CHECK_AND_RETURN_NULL(globalPose.isSane(), "PxPhysics::createRigidDynamic: invalid transform");
	return NpFactory::getInstance().createRigidDynamic(globalPose.getNormalized());
}

PxConstraint* NpPhysics::createConstraint(PxRigidActor* actor0, PxRigidActor* actor1, PxConstraintConnector& connector, const PxConstraintShaderTable& shaders, PxU32 dataSize)
{
	return NpFactory::getInstance().createConstraint(actor0, actor1, connector, shaders, dataSize);
}

PxArticulationReducedCoordinate* NpPhysics::createArticulationReducedCoordinate()
{
	return NpFactory::getInstance().createArticulationRC();
}

PxSoftBody* NpPhysics::createSoftBody(PxCudaContextManager& cudaContextManager)
{
	return NpFactory::getInstance().createSoftBody(cudaContextManager);
}

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
PxFEMCloth* NpPhysics::createFEMCloth(PxCudaContextManager& cudaContextManager)
{
	return NpFactory::getInstance().createFEMCloth(cudaContextManager);
}
#endif

PxPBDParticleSystem* NpPhysics::createPBDParticleSystem(PxCudaContextManager& cudaContexManager, PxU32 maxNeighborhood)
{
	return NpFactory::getInstance().createPBDParticleSystem(maxNeighborhood, cudaContexManager);
}

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
PxFLIPParticleSystem* NpPhysics::createFLIPParticleSystem(PxCudaContextManager& cudaContexManager)
{
	return NpFactory::getInstance().createFLIPParticleSystem(cudaContexManager);
}

PxMPMParticleSystem* NpPhysics::createMPMParticleSystem(PxCudaContextManager& cudaContexManager)
{
	return NpFactory::getInstance().createMPMParticleSystem( cudaContexManager);
}

PxCustomParticleSystem* NpPhysics::createCustomParticleSystem(PxCudaContextManager& cudaContexManager, PxU32 maxNeighborhood)
{
	return NpFactory::getInstance().createCustomParticleSystem(cudaContexManager, maxNeighborhood);
}
PxHairSystem* NpPhysics::createHairSystem(PxCudaContextManager& cudaContextManager)
{
	return NpFactory::getInstance().createHairSystem(cudaContextManager);
}
#endif

PxBuffer* NpPhysics::createBuffer(PxU64 byteSize, PxBufferType::Enum bufferType, PxCudaContextManager* cudaContexManager)
{
	if(!cudaContexManager)
		return NULL;

#if PX_SUPPORT_GPU_PHYSX
	PxPhysXGpu* physxGpu = PxvGetPhysXGpu(true);
	PX_ASSERT(physxGpu);
	return physxGpu->createBuffer(byteSize, bufferType, cudaContexManager, NULL);
#else
	PX_UNUSED(byteSize);
	PX_UNUSED(bufferType);
	return NULL;
#endif
}

PxAggregate* NpPhysics::createAggregate(PxU32 maxActors, PxU32 maxShapes, PxAggregateFilterHint filterHint)
{
	PX_CHECK_AND_RETURN_VAL(!(PxGetAggregateSelfCollisionBit(filterHint) && PxGetAggregateType(filterHint)==PxAggregateType::eSTATIC),
		"PxPhysics::createAggregate: static aggregates with self-collisions are not allowed.", NULL);

	return NpFactory::getInstance().createAggregate(maxActors, maxShapes, filterHint);
}

///////////////////////////////////////////////////////////////////////////////

NpMaterial* NpPhysics::addMaterial(NpMaterial* m)
{
	if(!m)
		return NULL;

	OMNI_PVD_NOTIFY_ADD(m);

	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	//the handle is set inside the setMaterial method
	if(mMasterMaterialManager.setMaterial(*m))
	{
		// Let all scenes know of the new material
		for(PxU32 i=0; i < mSceneArray.size(); i++)
		{
			NpScene* s = getScene(i);
			s->addMaterial(*m);
		}
		return m;
	}
	else
	{
		mFoundation.error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PxPhysics::createMaterial: limit of 64K materials reached.");
		m->release();
		return NULL;
	}
}

PxMaterial* NpPhysics::createMaterial(PxReal staticFriction, PxReal dynamicFriction, PxReal restitution)
{
	PxMaterial* m = NpFactory::getInstance().createMaterial(staticFriction, dynamicFriction, restitution);

	if (m)
		return addMaterial(static_cast<NpMaterial*>(m));
	else
		return NULL;
}

PxU32 NpPhysics::getNbMaterials() const
{
	PxMutex::ScopedLock lock(const_cast<PxMutex&>(mSceneAndMaterialMutex));
	return mMasterMaterialManager.getNumMaterials();
}

PxU32 NpPhysics::getMaterials(PxMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	PxMutex::ScopedLock lock(const_cast<PxMutex&>(mSceneAndMaterialMutex));
	NpMaterialManagerIterator<NpMaterial> iter(mMasterMaterialManager);
	PxU32 writeCount =0;
	PxU32 index = 0;
	NpMaterial* mat;
	while(iter.getNextMaterial(mat))
	{
		if(index++ < startIndex)
			continue;
		if(writeCount == bufferSize)
			break;
		userBuffer[writeCount++] = mat;
	}
	return writeCount;
}

void NpPhysics::removeMaterialFromTable(NpMaterial& m)
{
	OMNI_PVD_NOTIFY_REMOVE(&m);

	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	// Let all scenes know of the deleted material
	for(PxU32 i=0; i < mSceneArray.size(); i++)
	{
		NpScene* s = getScene(i);
		s->removeMaterial(m);
	}

	mMasterMaterialManager.removeMaterial(m);
}

void NpPhysics::updateMaterial(NpMaterial& m)
{
	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	// Let all scenes know of the updated material
	for(PxU32 i=0; i < mSceneArray.size(); i++)
	{
		NpScene* s = getScene(i);
		s->updateMaterial(m);
	}
	mMasterMaterialManager.updateMaterial(m);
}

bool NpPhysics::sendMaterialTable(NpScene& scene)
{
	// note: no lock here because this method gets only called at scene creation and there we do lock

	NpMaterialManagerIterator<NpMaterial> iter(mMasterMaterialManager);
	NpMaterial* mat;
	while(iter.getNextMaterial(mat))
		scene.addMaterial(*mat);

#if PX_SUPPORT_GPU_PHYSX
	
	NpMaterialManagerIterator<NpFEMSoftBodyMaterial> iterSoftBody(mMasterFEMSoftBodyMaterialManager);
	NpFEMSoftBodyMaterial* softmat;
	while (iterSoftBody.getNextMaterial(softmat))
		scene.addMaterial(*softmat);
	
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	NpMaterialManagerIterator<NpFEMClothMaterial> iterCloth(mMasterFEMClothMaterialManager);
	NpFEMClothMaterial* clothmat;
	while (iterCloth.getNextMaterial(clothmat))
		scene.addMaterial(*clothmat);
#endif

	NpMaterialManagerIterator<NpPBDMaterial> iterPBD(mMasterPBDMaterialManager);
	NpPBDMaterial* pbdmat;
	while (iterPBD.getNextMaterial(pbdmat))
		scene.addMaterial(*pbdmat);

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	NpMaterialManagerIterator<NpFLIPMaterial> iterFLIP(mMasterFLIPMaterialManager);
	NpFLIPMaterial* flipmat;
	while (iterFLIP.getNextMaterial(flipmat))
		scene.addMaterial(*flipmat);

	NpMaterialManagerIterator<NpMPMMaterial> iterMPM(mMasterMPMMaterialManager);
	NpMPMMaterial* mpmmat;
	while (iterMPM.getNextMaterial(mpmmat))
		scene.addMaterial(*mpmmat);

	NpMaterialManagerIterator<NpCustomMaterial> iterCustom(mMasterCustomMaterialManager);
	NpCustomMaterial* custommat;
	while (iterCustom.getNextMaterial(custommat))
		scene.addMaterial(*custommat);
#endif

#endif //PX_SUPPORT_GPU_PHYSX

	return true;
}

///////////////////////////////////////////////////////////////////////////////

NpFEMSoftBodyMaterial* NpPhysics::addFEMMaterial(NpFEMSoftBodyMaterial* m)
{
	if (!m)
		return NULL;

#if PX_SUPPORT_GPU_PHYSX
	OMNI_PVD_NOTIFY_ADD(m);

	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	//the handle is set inside the setMaterial method
	if (mMasterFEMSoftBodyMaterialManager.setMaterial(*m))
	{
		// Let all scenes know of the new material
		for (PxU32 i = 0; i < mSceneArray.size(); i++)
		{
			NpScene* s = getScene(i);
			s->addMaterial(*m);
		}
		return m;
	}
	else
	{
		mFoundation.error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PxPhysics::createMaterial: limit of 64K materials reached.");
		m->release();
		return NULL;
	}
#else
	m->release();
	return NULL;
#endif
}

PxFEMSoftBodyMaterial* NpPhysics::createFEMSoftBodyMaterial(PxReal youngs, PxReal poissons, PxReal dynamicFriction)
{
	PxFEMSoftBodyMaterial* m = NpFactory::getInstance().createFEMSoftBodyMaterial(youngs, poissons, dynamicFriction);
	return addFEMMaterial(static_cast<NpFEMSoftBodyMaterial*>(m));
}

PxU32 NpPhysics::getNbFEMSoftBodyMaterials() const
{
	PxMutex::ScopedLock lock(const_cast<PxMutex&>(mSceneAndMaterialMutex));
	return mMasterFEMSoftBodyMaterialManager.getNumMaterials();
}

PxU32 NpPhysics::getFEMSoftBodyMaterials(PxFEMSoftBodyMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	PxMutex::ScopedLock lock(const_cast<PxMutex&>(mSceneAndMaterialMutex));
	NpMaterialManagerIterator<NpFEMSoftBodyMaterial> iter(mMasterFEMSoftBodyMaterialManager);
	PxU32 writeCount = 0;
	PxU32 index = 0;
	NpFEMSoftBodyMaterial* mat;
	while (iter.getNextMaterial(mat))
	{
		if (index++ < startIndex)
			continue;
		if (writeCount == bufferSize)
			break;
		userBuffer[writeCount++] = mat;
	}
	return writeCount;
}

void NpPhysics::removeFEMSoftBodyMaterialFromTable(NpFEMSoftBodyMaterial& m)
{
#if PX_SUPPORT_GPU_PHYSX
	OMNI_PVD_NOTIFY_REMOVE(&m);

	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	// Let all scenes know of the deleted material
	for (PxU32 i = 0; i < mSceneArray.size(); i++)
	{
		NpScene* s = getScene(i);
		s->removeMaterial(m);
	}

	mMasterFEMSoftBodyMaterialManager.removeMaterial(m);
#else
	PX_UNUSED(m);
#endif
}

void NpPhysics::updateFEMSoftBodyMaterial(NpFEMSoftBodyMaterial& m)
{

#if PX_SUPPORT_GPU_PHYSX
	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	// Let all scenes know of the updated material
	for (PxU32 i = 0; i < mSceneArray.size(); i++)
	{
		NpScene* s = getScene(i);
		s->updateMaterial(m);
	}
	mMasterFEMSoftBodyMaterialManager.updateMaterial(m);
#else
	PX_UNUSED(m);
#endif
}

///////////////////////////////////////////////////////////////////////////////

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
NpFEMClothMaterial* NpPhysics::addFEMClothMaterial(NpFEMClothMaterial* m)
{
	if (!m)
		return NULL;

#if PX_SUPPORT_GPU_PHYSX
	OMNI_PVD_NOTIFY_ADD(m);

	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	//the handle is set inside the setMaterial method
	if (mMasterFEMClothMaterialManager.setMaterial(*m))
	{
		// Let all scenes know of the new material
		for (PxU32 i = 0; i < mSceneArray.size(); i++)
		{
			NpScene* s = getScene(i);
			s->addMaterial(*m);
		}
		return m;
	}
	else
	{
		mFoundation.error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PxPhysics::addFEMClothMaterial: limit of 64K materials reached.");
		m->release();
		return NULL;
	}
#else
	m->release();
	return NULL;
#endif
}

PxFEMClothMaterial* NpPhysics::createFEMClothMaterial(PxReal youngs, PxReal poissons, PxReal dynamicFriction)
{
	PxFEMClothMaterial* m = NpFactory::getInstance().createFEMClothMaterial(youngs, poissons, dynamicFriction);
	return addFEMClothMaterial(static_cast<NpFEMClothMaterial*>(m));
}
#endif

PxU32 NpPhysics::getNbFEMClothMaterials() const
{
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	PxMutex::ScopedLock lock(const_cast<PxMutex&>(mSceneAndMaterialMutex));
	return mMasterFEMClothMaterialManager.getNumMaterials();
#else
	return 0;
#endif
}

PxU32 NpPhysics::getFEMClothMaterials(PxFEMClothMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	PxMutex::ScopedLock lock(const_cast<PxMutex&>(mSceneAndMaterialMutex));
	NpMaterialManagerIterator<NpFEMClothMaterial> iter(mMasterFEMClothMaterialManager);
	PxU32 writeCount = 0;
	PxU32 index = 0;
	NpFEMClothMaterial* mat;
	while (iter.getNextMaterial(mat))
	{
		if (index++ < startIndex)
			continue;
		if (writeCount == bufferSize)
			break;
		userBuffer[writeCount++] = mat;
	}
	return writeCount;
#else
	PX_UNUSED(userBuffer); PX_UNUSED(bufferSize); PX_UNUSED(startIndex);
	return 0;
#endif
}

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
void NpPhysics::removeFEMClothMaterialFromTable(NpFEMClothMaterial& m)
{
#if PX_SUPPORT_GPU_PHYSX
	OMNI_PVD_NOTIFY_REMOVE(&m);

	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	// Let all scenes know of the deleted material
	for (PxU32 i = 0; i < mSceneArray.size(); i++)
	{
		NpScene* s = getScene(i);
		s->removeMaterial(m);
	}

	mMasterFEMClothMaterialManager.removeMaterial(m);
#else
	PX_UNUSED(m);
#endif
}

void NpPhysics::updateFEMClothMaterial(NpFEMClothMaterial& m)
{
#if PX_SUPPORT_GPU_PHYSX
	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	// Let all scenes know of the updated material
	for (PxU32 i = 0; i < mSceneArray.size(); i++)
	{
		NpScene* s = getScene(i);
		s->updateMaterial(m);
	}
	mMasterFEMClothMaterialManager.updateMaterial(m);
#else
	PX_UNUSED(m);
#endif
}
#endif

///////////////////////////////////////////////////////////////////////////////

NpPBDMaterial* NpPhysics::addPBDMaterial(NpPBDMaterial* m)
{
	if (!m)
		return NULL;

#if PX_SUPPORT_GPU_PHYSX
	OMNI_PVD_NOTIFY_ADD(m);

	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	//the handle is set inside the setMaterial method
	if (mMasterPBDMaterialManager.setMaterial(*m))
	{
		// Let all scenes know of the new material
		for (PxU32 i = 0; i < mSceneArray.size(); i++)
		{
			NpScene* s = getScene(i);
			s->addMaterial(*m);
		}
		return m;
	}
	else
	{
		mFoundation.error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PxPhysics::addPBDMaterial: limit of 64K materials reached.");
		m->release();
		return NULL;
	}
#else
	m->release();
	return NULL;
#endif
}

PxPBDMaterial* NpPhysics::createPBDMaterial(PxReal friction, PxReal damping, PxReal adhesion, PxReal viscosity, PxReal vorticityConfinement, PxReal surfaceTension, 
	PxReal cohesion, PxReal lift, PxReal drag, PxReal cflCoefficient, PxReal gravityScale)
{
	PxPBDMaterial* m = NpFactory::getInstance().createPBDMaterial(friction, damping, adhesion, viscosity, vorticityConfinement, surfaceTension, cohesion, lift, drag, cflCoefficient, gravityScale);
	return addPBDMaterial(static_cast<NpPBDMaterial*>(m));
}

PxU32 NpPhysics::getNbPBDMaterials() const
{
	PxMutex::ScopedLock lock(const_cast<PxMutex&>(mSceneAndMaterialMutex));
	return mMasterPBDMaterialManager.getNumMaterials();
}

PxU32 NpPhysics::getPBDMaterials(PxPBDMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	PxMutex::ScopedLock lock(const_cast<PxMutex&>(mSceneAndMaterialMutex));
	NpMaterialManagerIterator<NpPBDMaterial> iter(mMasterPBDMaterialManager);
	PxU32 writeCount = 0;
	PxU32 index = 0;
	NpPBDMaterial* mat;
	while (iter.getNextMaterial(mat))
	{
		if (index++ < startIndex)
			continue;
		if (writeCount == bufferSize)
			break;
		userBuffer[writeCount++] = mat;
	}
	return writeCount;
}

void NpPhysics::removePBDMaterialFromTable(NpPBDMaterial& m)
{
#if PX_SUPPORT_GPU_PHYSX
	OMNI_PVD_NOTIFY_REMOVE(&m);

	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	// Let all scenes know of the deleted material
	for (PxU32 i = 0; i < mSceneArray.size(); i++)
	{
		NpScene* s = getScene(i);
		s->removeMaterial(m);
	}

	mMasterPBDMaterialManager.removeMaterial(m);
#else
	PX_UNUSED(m);
#endif
}

void NpPhysics::updatePBDMaterial(NpPBDMaterial& m)
{
#if PX_SUPPORT_GPU_PHYSX
	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	// Let all scenes know of the updated material
	for (PxU32 i = 0; i < mSceneArray.size(); i++)
	{
		NpScene* s = getScene(i);
		s->updateMaterial(m);
	}
	mMasterPBDMaterialManager.updateMaterial(m);
#else
	PX_UNUSED(m);
#endif
}

///////////////////////////////////////////////////////////////////////////////

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
NpFLIPMaterial* NpPhysics::addFLIPMaterial(NpFLIPMaterial* m)
{
	if (!m)
		return NULL;

#if PX_SUPPORT_GPU_PHYSX
	OMNI_PVD_NOTIFY_ADD(m);

	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	//the handle is set inside the setMaterial method
	if (mMasterFLIPMaterialManager.setMaterial(*m))
	{
		// Let all scenes know of the new material
		for (PxU32 i = 0; i < mSceneArray.size(); i++)
		{
			NpScene* s = getScene(i);
			s->addMaterial(*m);
		}
		return m;
	}
	else
	{
		mFoundation.error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PxPhysics::addFLIPMaterial: limit of 64K materials reached.");
		m->release();
		return NULL;
	}
#else
	m->release();
	return NULL;
#endif
}

PxFLIPMaterial* NpPhysics::createFLIPMaterial(PxReal friction, PxReal damping, PxReal maxVelocity, PxReal viscosity, PxReal gravityScale)
{
	PxFLIPMaterial* m = NpFactory::getInstance().createFLIPMaterial(friction, damping, maxVelocity, viscosity, gravityScale);
	return addFLIPMaterial(static_cast<NpFLIPMaterial*>(m));
}

PxU32 NpPhysics::getNbFLIPMaterials() const
{
	PxMutex::ScopedLock lock(const_cast<PxMutex&>(mSceneAndMaterialMutex));
	return mMasterFLIPMaterialManager.getNumMaterials();
}

PxU32 NpPhysics::getFLIPMaterials(PxFLIPMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	PxMutex::ScopedLock lock(const_cast<PxMutex&>(mSceneAndMaterialMutex));
	NpMaterialManagerIterator<NpFLIPMaterial> iter(mMasterFLIPMaterialManager);
	PxU32 writeCount = 0;
	PxU32 index = 0;
	NpFLIPMaterial* mat;
	while (iter.getNextMaterial(mat))
	{
		if (index++ < startIndex)
			continue;
		if (writeCount == bufferSize)
			break;
		userBuffer[writeCount++] = mat;
	}
	return writeCount;
}

void NpPhysics::removeFLIPMaterialFromTable(NpFLIPMaterial& m)
{
#if PX_SUPPORT_GPU_PHYSX
	OMNI_PVD_NOTIFY_REMOVE(&m);

	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	// Let all scenes know of the deleted material
	for (PxU32 i = 0; i < mSceneArray.size(); i++)
	{
		NpScene* s = getScene(i);
		s->removeMaterial(m);
	}

	mMasterFLIPMaterialManager.removeMaterial(m);
#else
	PX_UNUSED(m);
#endif
}

void NpPhysics::updateFLIPMaterial(NpFLIPMaterial& m)
{
#if PX_SUPPORT_GPU_PHYSX
	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	// Let all scenes know of the updated material
	for (PxU32 i = 0; i < mSceneArray.size(); i++)
	{
		NpScene* s = getScene(i);
		s->updateMaterial(m);
	}
	mMasterFLIPMaterialManager.updateMaterial(m);
#else
	PX_UNUSED(m);
#endif
}

///////////////////////////////////////////////////////////////////////////////

NpMPMMaterial* NpPhysics::addMPMMaterial(NpMPMMaterial* m)
{
	if (!m)
		return NULL;

#if PX_SUPPORT_GPU_PHYSX
	OMNI_PVD_NOTIFY_ADD(m);

	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	//the handle is set inside the setMaterial method
	if (mMasterMPMMaterialManager.setMaterial(*m))
	{
		// Let all scenes know of the new material
		for (PxU32 i = 0; i < mSceneArray.size(); i++)
		{
			NpScene* s = getScene(i);
			s->addMaterial(*m);
		}
		return m;
	}
	else
	{
		mFoundation.error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PxPhysics::addMPMMaterial: limit of 64K materials reached.");
		m->release();
		return NULL;
	}
#else
	m->release();
	return NULL;
#endif
}

PxMPMMaterial* NpPhysics::createMPMMaterial(PxReal friction, PxReal damping, PxReal maxVelocity, bool isPlastic, PxReal youngsModulus, PxReal poissons, PxReal hardening, PxReal criticalCompression, PxReal criticalStretch, PxReal tensileDamageSensitivity, PxReal compressiveDamageSensitivity, PxReal attractiveForceResidual, PxReal gravityScale)
{
	PxMPMMaterial* m = NpFactory::getInstance().createMPMMaterial(friction, damping, maxVelocity, isPlastic, youngsModulus, poissons, hardening, criticalCompression, criticalStretch, tensileDamageSensitivity, compressiveDamageSensitivity, attractiveForceResidual, gravityScale);
	return addMPMMaterial(static_cast<NpMPMMaterial*>(m));
}

PxU32 NpPhysics::getNbMPMMaterials() const
{
	PxMutex::ScopedLock lock(const_cast<PxMutex&>(mSceneAndMaterialMutex));
	return mMasterMPMMaterialManager.getNumMaterials();
}

PxU32 NpPhysics::getMPMMaterials(PxMPMMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	PxMutex::ScopedLock lock(const_cast<PxMutex&>(mSceneAndMaterialMutex));
	NpMaterialManagerIterator<NpMPMMaterial> iter(mMasterMPMMaterialManager);
	PxU32 writeCount = 0;
	PxU32 index = 0;
	NpMPMMaterial* mat;
	while (iter.getNextMaterial(mat))
	{
		if (index++ < startIndex)
			continue;
		if (writeCount == bufferSize)
			break;
		userBuffer[writeCount++] = mat;
	}
	return writeCount;
}

void NpPhysics::removeMPMMaterialFromTable(NpMPMMaterial& m)
{
#if PX_SUPPORT_GPU_PHYSX
	OMNI_PVD_NOTIFY_REMOVE(&m);

	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	// Let all scenes know of the deleted material
	for (PxU32 i = 0; i < mSceneArray.size(); i++)
	{
		NpScene* s = getScene(i);
		s->removeMaterial(m);
	}

	mMasterMPMMaterialManager.removeMaterial(m);
#else
	PX_UNUSED(m);
#endif
}

void NpPhysics::updateMPMMaterial(NpMPMMaterial& m)
{
#if PX_SUPPORT_GPU_PHYSX
	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	// Let all scenes know of the updated material
	for (PxU32 i = 0; i < mSceneArray.size(); i++)
	{
		NpScene* s = getScene(i);
		s->updateMaterial(m);
	}
	mMasterMPMMaterialManager.updateMaterial(m);
#else
	PX_UNUSED(m);
#endif
}

///////////////////////////////////////////////////////////////////////////////

NpCustomMaterial* NpPhysics::addCustomMaterial(NpCustomMaterial* m)
{
	if (!m)
		return NULL;

#if PX_SUPPORT_GPU_PHYSX
	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	//the handle is set inside the setMaterial method
	if (mMasterCustomMaterialManager.setMaterial(*m))
	{
		// Let all scenes know of the new material
		for (PxU32 i = 0; i < mSceneArray.size(); i++)
		{
			NpScene* s = getScene(i);
			s->addMaterial(*m);
		}
		return m;
	}
	else
	{
		mFoundation.error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PxPhysics::addCustomMaterial: limit of 64K materials reached.");
		m->release();
		return NULL;
	}
#else
	m->release();
	return NULL;
#endif
}

PxCustomMaterial* NpPhysics::createCustomMaterial(void* gpuBuffer)
{
	PxCustomMaterial* m = NpFactory::getInstance().createCustomMaterial(gpuBuffer);
	return addCustomMaterial(static_cast<NpCustomMaterial*>(m));
}

PxU32 NpPhysics::getNbCustomMaterials() const
{
	PxMutex::ScopedLock lock(const_cast<PxMutex&>(mSceneAndMaterialMutex));
	return mMasterCustomMaterialManager.getNumMaterials();
}

PxU32 NpPhysics::getCustomMaterials(PxCustomMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	PxMutex::ScopedLock lock(const_cast<PxMutex&>(mSceneAndMaterialMutex));
	NpMaterialManagerIterator<NpCustomMaterial> iter(mMasterCustomMaterialManager);
	PxU32 writeCount = 0;
	PxU32 index = 0;
	NpCustomMaterial* mat;
	while (iter.getNextMaterial(mat))
	{
		if (index++ < startIndex)
			continue;
		if (writeCount == bufferSize)
			break;
		userBuffer[writeCount++] = mat;
	}
	return writeCount;
}

void NpPhysics::removeCustomMaterialFromTable(NpCustomMaterial& m)
{
#if PX_SUPPORT_GPU_PHYSX
	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	// Let all scenes know of the deleted material
	for (PxU32 i = 0; i < mSceneArray.size(); i++)
	{
		NpScene* s = getScene(i);
		s->removeMaterial(m);
	}

	mMasterCustomMaterialManager.removeMaterial(m);
#else
	PX_UNUSED(m);
#endif
}

void NpPhysics::updateCustomMaterial(NpCustomMaterial& m)
{

#if PX_SUPPORT_GPU_PHYSX
	PxMutex::ScopedLock lock(mSceneAndMaterialMutex);

	// Let all scenes know of the updated material
	for (PxU32 i = 0; i < mSceneArray.size(); i++)
	{
		NpScene* s = getScene(i);
		s->updateMaterial(m);
	}
	mMasterCustomMaterialManager.updateMaterial(m);
#else
	PX_UNUSED(m);
#endif
}
#endif

///////////////////////////////////////////////////////////////////////////////

PxTriangleMesh* NpPhysics::createTriangleMesh(PxInputStream& stream)
{
	return NpFactory::getInstance().createTriangleMesh(stream);
}

PxU32 NpPhysics::getNbTriangleMeshes() const
{
	return NpFactory::getInstance().getNbTriangleMeshes();
}

PxU32 NpPhysics::getTriangleMeshes(PxTriangleMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	return NpFactory::getInstance().getTriangleMeshes(userBuffer, bufferSize, startIndex);
}

///////////////////////////////////////////////////////////////////////////////

PxTetrahedronMesh* NpPhysics::createTetrahedronMesh(PxInputStream& stream)
{
	return NpFactory::getInstance().createTetrahedronMesh(stream);
}

PxU32 NpPhysics::getNbTetrahedronMeshes() const
{
	return NpFactory::getInstance().getNbTetrahedronMeshes();
}

PxU32 NpPhysics::getTetrahedronMeshes(PxTetrahedronMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	return NpFactory::getInstance().getTetrahedronMeshes(userBuffer, bufferSize, startIndex);
}

PxSoftBodyMesh* NpPhysics::createSoftBodyMesh(PxInputStream& stream)
{
	return NpFactory::getInstance().createSoftBodyMesh(stream);
}

///////////////////////////////////////////////////////////////////////////////

PxHeightField* NpPhysics::createHeightField(PxInputStream& stream)
{
	return NpFactory::getInstance().createHeightField(stream);
}

PxU32 NpPhysics::getNbHeightFields() const
{
	return NpFactory::getInstance().getNbHeightFields();
}

PxU32 NpPhysics::getHeightFields(PxHeightField** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	return NpFactory::getInstance().getHeightFields(userBuffer, bufferSize, startIndex);
}

///////////////////////////////////////////////////////////////////////////////

PxConvexMesh* NpPhysics::createConvexMesh(PxInputStream& stream)
{
	return NpFactory::getInstance().createConvexMesh(stream);
}

PxU32 NpPhysics::getNbConvexMeshes() const
{
	return NpFactory::getInstance().getNbConvexMeshes();
}

PxU32 NpPhysics::getConvexMeshes(PxConvexMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	return NpFactory::getInstance().getConvexMeshes(userBuffer, bufferSize, startIndex);
}

///////////////////////////////////////////////////////////////////////////////

PxBVH* NpPhysics::createBVH(PxInputStream& stream)
{
	return NpFactory::getInstance().createBVH(stream);
}

PxU32 NpPhysics::getNbBVHs() const
{
	return NpFactory::getInstance().getNbBVHs();
}

PxU32 NpPhysics::getBVHs(PxBVH** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	return NpFactory::getInstance().getBVHs(userBuffer, bufferSize, startIndex);
}

///////////////////////////////////////////////////////////////////////////////

PxParticleBuffer* NpPhysics::createParticleBuffer(PxU32 maxParticles, PxU32 maxVolumes, PxCudaContextManager* cudaContexManager)
{
	return NpFactory::getInstance().createParticleBuffer(maxParticles, maxVolumes, cudaContexManager);
}

PxParticleAndDiffuseBuffer* NpPhysics::createParticleAndDiffuseBuffer(PxU32 maxParticles, PxU32 maxVolumes, PxU32 maxDiffuseParticles, PxCudaContextManager* cudaContexManager)
{
	return NpFactory::getInstance().createParticleAndDiffuseBuffer(maxParticles, maxVolumes, maxDiffuseParticles, cudaContexManager);
}

PxParticleClothBuffer* NpPhysics::createParticleClothBuffer(PxU32 maxParticles, PxU32 maxNumVolumes, PxU32 maxNumCloths, PxU32 maxNumTriangles, PxU32 maxNumSprings, PxCudaContextManager* cudaContexManager)
{
	return NpFactory::getInstance().createParticleClothBuffer(maxParticles, maxNumVolumes, maxNumCloths, maxNumTriangles, maxNumSprings, cudaContexManager);
}

PxParticleRigidBuffer* NpPhysics::createParticleRigidBuffer(PxU32 maxParticles, PxU32 maxNumVolumes, PxU32 maxNumRigids, PxCudaContextManager* cudaContexManager)
{
	return NpFactory::getInstance().createParticleRigidBuffer(maxParticles, maxNumVolumes, maxNumRigids, cudaContexManager);
}

///////////////////////////////////////////////////////////////////////////////

PxPruningStructure* NpPhysics::createPruningStructure(PxRigidActor*const* actors, PxU32 nbActors)
{
	PX_SIMD_GUARD;

	PX_ASSERT(actors);
	PX_ASSERT(nbActors > 0);

	Sq::PruningStructure* ps = PX_NEW(Sq::PruningStructure)();	
	if(!ps->build(actors, nbActors))
	{
		PX_DELETE(ps);		
	}
	return ps;
}

///////////////////////////////////////////////////////////////////////////////

#if PX_SUPPORT_GPU_PHYSX
void NpPhysics::registerPhysXIndicatorGpuClient()
{
	PxMutex::ScopedLock lock(mPhysXIndicatorMutex);

	++mNbRegisteredGpuClients;

	mPhysXIndicator.setIsGpu(mNbRegisteredGpuClients>0);
}

void NpPhysics::unregisterPhysXIndicatorGpuClient()
{
	PxMutex::ScopedLock lock(mPhysXIndicatorMutex);

	if (mNbRegisteredGpuClients)
		--mNbRegisteredGpuClients;

	mPhysXIndicator.setIsGpu(mNbRegisteredGpuClients>0);
}
#endif

///////////////////////////////////////////////////////////////////////////////

void NpPhysics::registerDeletionListener(PxDeletionListener& observer, const PxDeletionEventFlags& deletionEvents, bool restrictedObjectSet)
{
	PxMutex::ScopedLock lock(mDeletionListenerMutex);

	const DeletionListenerMap::Entry* entry = mDeletionListenerMap.find(&observer);
	if(!entry)
	{
		NpDelListenerEntry* e = PX_NEW(NpDelListenerEntry)(deletionEvents, restrictedObjectSet);
		if (e)
		{
			if (mDeletionListenerMap.insert(&observer, e))
				mDeletionListenersExist = true;
			else
			{
				PX_DELETE(e);
				PX_ALWAYS_ASSERT();
			}
		}
	}
	else
		PX_ASSERT(mDeletionListenersExist);
}

void NpPhysics::unregisterDeletionListener(PxDeletionListener& observer)
{
	PxMutex::ScopedLock lock(mDeletionListenerMutex);

	const DeletionListenerMap::Entry* entry = mDeletionListenerMap.find(&observer);
	if(entry)
	{
		NpDelListenerEntry* e = entry->second;
		mDeletionListenerMap.erase(&observer);
		PX_DELETE(e);
	}
	mDeletionListenersExist = mDeletionListenerMap.size()>0;
}

void NpPhysics::registerDeletionListenerObjects(PxDeletionListener& observer, const PxBase* const* observables, PxU32 observableCount)
{
	PxMutex::ScopedLock lock(mDeletionListenerMutex);

	const DeletionListenerMap::Entry* entry = mDeletionListenerMap.find(&observer);
	if(entry)
	{
		NpDelListenerEntry* e = entry->second;
		PX_CHECK_AND_RETURN(e->restrictedObjectSet, "PxPhysics::registerDeletionListenerObjects: deletion listener is not configured to receive events from specific objects.");

		e->registeredObjects.reserve(e->registeredObjects.size() + observableCount);
		for(PxU32 i=0; i < observableCount; i++)
			e->registeredObjects.insert(observables[i]);
	}
	else
	{
		PX_CHECK_AND_RETURN(false, "PxPhysics::registerDeletionListenerObjects: deletion listener has to be registered in PxPhysics first.");
	}
}

void NpPhysics::unregisterDeletionListenerObjects(PxDeletionListener& observer, const PxBase* const* observables, PxU32 observableCount)
{
	PxMutex::ScopedLock lock(mDeletionListenerMutex);

	const DeletionListenerMap::Entry* entry = mDeletionListenerMap.find(&observer);
	if(entry)
	{
		NpDelListenerEntry* e = entry->second;
		if (e->restrictedObjectSet)
		{
			for(PxU32 i=0; i < observableCount; i++)
				e->registeredObjects.erase(observables[i]);
		}
		else
		{
			PX_CHECK_AND_RETURN(false, "PxPhysics::unregisterDeletionListenerObjects: deletion listener is not configured to receive events from specific objects.");
		}
	}
	else
	{
		PX_CHECK_AND_RETURN(false, "PxPhysics::unregisterDeletionListenerObjects: deletion listener has to be registered in PxPhysics first.");
	}
}

void NpPhysics::notifyDeletionListeners(const PxBase* base, void* userData, PxDeletionEventFlag::Enum deletionEvent)
{
	// we don't protect the check for whether there are any listeners, because we don't want to take a hit in the 
	// common case where there are no listeners. Note the API comments here, that users should not register or 
	// unregister deletion listeners while deletions are occurring

	if(mDeletionListenersExist)
	{
		PxMutex::ScopedLock lock(mDeletionListenerMutex);

		const DeletionListenerMap::Entry* delListenerEntries = mDeletionListenerMap.getEntries();
		const PxU32 delListenerEntryCount = mDeletionListenerMap.size();
		for(PxU32 i=0; i < delListenerEntryCount; i++)
		{
			const NpDelListenerEntry* entry = delListenerEntries[i].second;
			
			if (entry->flags & deletionEvent)
			{
				if (entry->restrictedObjectSet)
				{
					if (entry->registeredObjects.contains(base))
						delListenerEntries[i].first->onRelease(base, userData, deletionEvent);
				}
				else
					delListenerEntries[i].first->onRelease(base, userData, deletionEvent);
			}
		}
	}
}

#if PX_SUPPORT_OMNI_PVD
void NpPhysics::OmniPvdListener::onObjectAdd(const PxBase* object)
{
	::OmniPvdPxSampler::getInstance()->onObjectAdd(object);
}

void NpPhysics::OmniPvdListener::onObjectRemove(const PxBase* object)
{
	::OmniPvdPxSampler::getInstance()->onObjectRemove(object);
}
#endif

///////////////////////////////////////////////////////////////////////////////

const PxTolerancesScale& NpPhysics::getTolerancesScale() const
{
	return mPhysics.getTolerancesScale();
}

PxFoundation& NpPhysics::getFoundation()
{
	return mFoundation;
}

PxPhysics& PxGetPhysics()
{
	return NpPhysics::getInstance();
}

PxPhysics* PxCreateBasePhysics(PxU32 version, PxFoundation& foundation, const PxTolerancesScale& scale, bool trackOutstandingAllocations, PxPvd* pvd, PxOmniPvd* omniPvd)
{
	return NpPhysics::createInstance(version, foundation, scale, trackOutstandingAllocations, static_cast<pvdsdk::PsPvd*>(pvd), omniPvd);
}

//void PxRegisterArticulations(PxPhysics& physics)
//{
//	PX_UNUSED(&physics);	// for the moment
//	Dy::PxvRegisterArticulations();
//	NpFactory::registerArticulations();	
//}

void PxRegisterArticulationsReducedCoordinate(PxPhysics& physics)
{
	PX_UNUSED(&physics);	// for the moment
	Dy::PxvRegisterArticulationsReducedCoordinate();
	NpFactory::registerArticulationRCs();
}

void PxRegisterHeightFields(PxPhysics& physics)
{
	PX_UNUSED(&physics);	// for the moment
	PX_CHECK_AND_RETURN(NpPhysics::getInstance().getNumScenes() == 0, "PxRegisterHeightFields: it is illegal to call a heightfield registration function after you have a scene.");

	PxvRegisterHeightFields();
	Gu::registerHeightFields();	
#if PX_CHECKED
	NpPhysics::heightfieldsAreRegistered();
#endif
}

void PxAddCollectionToPhysics(const PxCollection& collection)
{
	NpFactory& factory = NpFactory::getInstance();
	const Cm::Collection& c = static_cast<const Cm::Collection&>(collection);	
    factory.addCollection(c);
}
