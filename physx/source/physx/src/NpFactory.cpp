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

#include "geometry/PxGeometryQuery.h"
#include "NpFactory.h"
#include "NpPhysics.h"
#include "ScPhysics.h"
#include "GuHeightField.h"
#include "GuTriangleMesh.h"
#include "GuConvexMesh.h"

#include "NpConnector.h"
#include "NpPtrTableStorageManager.h"
#include "CmCollection.h"
#include "NpArticulationReducedCoordinate.h"
#include "NpArticulationJointReducedCoordinate.h"
#include "NpRigidStatic.h"
#include "NpRigidDynamic.h"
#include "NpArticulationTendon.h"
#include "NpAggregate.h"

#include "PxvGlobals.h"

#if PX_SUPPORT_GPU_PHYSX
#include "NpParticleSystem.h"
#include "NpSoftBody.h"
#include "NpFEMCloth.h"
#include "NpHairSystem.h"
#include "PxPhysXGpu.h"
#endif

#if PX_SUPPORT_OMNI_PVD
#	define OMNI_PVD_NOTIFY_ADD(OBJECT) notifyListenersAdd(OBJECT)
#	define OMNI_PVD_NOTIFY_REMOVE(OBJECT) notifyListenersRemove(OBJECT)
#else
#	define OMNI_PVD_NOTIFY_ADD(OBJECT)
#	define OMNI_PVD_NOTIFY_REMOVE(OBJECT)
#endif

using namespace physx;
using namespace Cm;

NpFactory::NpFactory() :
	Gu::MeshFactory()
	, mConnectorArrayPool("connectorArrayPool")
	, mPtrTableStorageManager(PX_NEW(NpPtrTableStorageManager))
	, mMaterialPool("MaterialPool")
	, mGpuMemStat(0)
#if PX_SUPPORT_PVD
	, mNpFactoryListener(NULL)
#endif	
{
}

template <typename T>
static void releaseAll(PxHashSet<T*>& container)
{
	// a bit tricky: release will call the factory back to remove the object from
	// the tracking array, immediately invalidating the iterator. Reconstructing the
	// iterator per delete can be expensive. So, we use a temporary object.
	//
	// a coalesced hash would be efficient too, but we only ever iterate over it
	// here so it's not worth the 2x remove penalty over the normal hash.

	PxArray<T*, PxReflectionAllocator<T*> > tmp;
	tmp.reserve(container.size());
	for(typename PxHashSet<T*>::Iterator iter = container.getIterator(); !iter.done(); ++iter)
		tmp.pushBack(*iter);

	PX_ASSERT(tmp.size() == container.size());
	for(PxU32 i=0;i<tmp.size();i++)
		tmp[i]->release();
}

NpFactory::~NpFactory()
{
	PX_DELETE(mPtrTableStorageManager);
}

void NpFactory::release()
{
	releaseAll(mAggregateTracking);
	releaseAll(mConstraintTracking);
	releaseAll(mArticulationTracking);
	releaseAll(mActorTracking);
	while(mShapeTracking.size())
		static_cast<NpShape*>(mShapeTracking.getEntries()[0])->releaseInternal();

#if PX_SUPPORT_GPU_PHYSX
	releaseAll(mParticleBufferTracking);
#endif

	Gu::MeshFactory::release();  // deletes the class
}

void NpFactory::createInstance()
{
	PX_ASSERT(!mInstance);
	mInstance = PX_NEW(NpFactory)();
}

void NpFactory::destroyInstance()
{
	PX_ASSERT(mInstance);
	mInstance->release();
	mInstance = NULL;
}

NpFactory* NpFactory::mInstance = NULL;

///////////////////////////////////////////////////////////////////////////////

template <class T0, class T1>
static void addToTracking(T1& set, T0* element, PxMutex& mutex, bool lock)
{
	if(!element)
		return;

	if(lock)
		mutex.lock();

	set.insert(element);

	if(lock)
		mutex.unlock();
}

/////////////////////////////////////////////////////////////////////////////// Actors

void NpFactory::addRigidStatic(PxRigidStatic* npActor, bool lock)
{
	addToTracking(mActorTracking, npActor, mTrackingMutex, lock);
	OMNI_PVD_NOTIFY_ADD(npActor);
}

void NpFactory::addRigidDynamic(PxRigidDynamic* npBody, bool lock)
{
	addToTracking(mActorTracking, npBody, mTrackingMutex, lock);
	OMNI_PVD_NOTIFY_ADD(npBody);
}

void NpFactory::addShape(PxShape* shape, bool lock)
{
	addToTracking(mShapeTracking, shape, mTrackingMutex, lock);
	OMNI_PVD_NOTIFY_ADD(shape);
}

void NpFactory::onActorRelease(PxActor* a)
{
	OMNI_PVD_NOTIFY_REMOVE(a);
	PxMutex::ScopedLock lock(mTrackingMutex);
	mActorTracking.erase(a);
}

void NpFactory::onShapeRelease(PxShape* a)
{
	OMNI_PVD_NOTIFY_REMOVE(a);
	PxMutex::ScopedLock lock(mTrackingMutex);
	mShapeTracking.erase(a);
}

void NpFactory::addArticulation(PxArticulationReducedCoordinate* npArticulation, bool lock)
{
	addToTracking(mArticulationTracking, npArticulation, mTrackingMutex, lock);
	OMNI_PVD_NOTIFY_ADD(npArticulation);
}

#if PX_SUPPORT_GPU_PHYSX
void NpFactory::addParticleBuffer(PxParticleBuffer* buffer, bool lock)
{
	addToTracking(mParticleBufferTracking, buffer, mTrackingMutex, lock);
}

void NpFactory::onParticleBufferReleaseInternal(PxParticleBuffer* buffer)
{
	PxMutex::ScopedLock lock(mTrackingMutex);
	mParticleBufferTracking.erase(buffer);
}
#endif

void NpFactory::releaseArticulationToPool(PxArticulationReducedCoordinate& articulation)
{
	PX_ASSERT(articulation.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	
	PX_ASSERT(articulation.getConcreteType() == PxConcreteType::eARTICULATION_REDUCED_COORDINATE);
	PxMutex::ScopedLock lock(mArticulationRCPoolLock);
	mArticulationRCPool.destroy(static_cast<NpArticulationReducedCoordinate*>(&articulation));
}

PxArticulationReducedCoordinate* NpFactory::createArticulationRC()
{
	NpArticulationReducedCoordinate* npArticulation = NpFactory::getInstance().createNpArticulationRC();
	if(npArticulation)
		addArticulation(npArticulation);
	else
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Articulation initialization failed: returned NULL.");

	// OMNI_PVD_CREATE()
	return npArticulation;
}

NpArticulationReducedCoordinate* NpFactory::createNpArticulationRC()
{
	PxMutex::ScopedLock lock(mArticulationRCPoolLock);
	return mArticulationRCPool.construct();
}

void NpFactory::onArticulationRelease(PxArticulationReducedCoordinate* a)
{
	OMNI_PVD_NOTIFY_REMOVE(a);
	PxMutex::ScopedLock lock(mTrackingMutex);
	mArticulationTracking.erase(a);
}

NpArticulationLink* NpFactory::createNpArticulationLink(NpArticulationReducedCoordinate& root, NpArticulationLink* parent, const PxTransform& pose)
{
	 NpArticulationLink* npArticulationLink;
	{
		PxMutex::ScopedLock lock(mArticulationLinkPoolLock);		
		npArticulationLink = mArticulationLinkPool.construct(pose, root, parent);
	}
	OMNI_PVD_NOTIFY_ADD(npArticulationLink);
	return npArticulationLink;
}

void NpFactory::releaseArticulationLinkToPool(NpArticulationLink& articulationLink)
{
	PX_ASSERT(articulationLink.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	OMNI_PVD_NOTIFY_REMOVE(&articulationLink);
	PxMutex::ScopedLock lock(mArticulationLinkPoolLock);
	mArticulationLinkPool.destroy(&articulationLink);
}

PxArticulationLink* NpFactory::createArticulationLink(NpArticulationReducedCoordinate& root, NpArticulationLink* parent, const PxTransform& pose)
{
	PX_CHECK_AND_RETURN_NULL(pose.isValid(),"Supplied articulation link pose is not valid. Articulation link creation method returns NULL.");
	PX_CHECK_AND_RETURN_NULL((!parent || (&parent->getRoot() == &root)), "specified parent link is not part of the destination articulation. Articulation link creation method returns NULL.");
	
	NpArticulationLink* npArticulationLink = NpFactory::getInstance().createNpArticulationLink(root, parent, pose);
	if (!npArticulationLink)
	{
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,  "Articulation link initialization failed: returned NULL.");
		return NULL;
	}

	PxArticulationJointReducedCoordinate* npArticulationJoint = 0;
	if (parent)
	{
		PxTransform parentPose = parent->getCMassLocalPose().transformInv(pose);
		PxTransform childPose = PxTransform(PxIdentity);
						
		npArticulationJoint = root.createArticulationJoint(*parent, parentPose, *npArticulationLink, childPose);
		if (!npArticulationJoint)
		{
			PX_DELETE(npArticulationLink);
	
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Articulation link initialization failed due to joint creation failure: returned NULL.");
			return NULL;
		}

		npArticulationLink->setInboundJoint(*npArticulationJoint);
	}

	return npArticulationLink;
}

NpArticulationJointReducedCoordinate* NpFactory::createNpArticulationJointRC(NpArticulationLink& parent, const PxTransform& parentFrame, NpArticulationLink& child, const PxTransform& childFrame)
{
	NpArticulationJointReducedCoordinate* npArticulationJoint;
	{
		PxMutex::ScopedLock lock(mArticulationJointRCPoolLock);
		npArticulationJoint = mArticulationRCJointPool.construct(parent, parentFrame, child, childFrame);
	}
	OMNI_PVD_NOTIFY_ADD(npArticulationJoint);
	return npArticulationJoint;
}

void NpFactory::releaseArticulationJointRCToPool(NpArticulationJointReducedCoordinate& articulationJoint)
{
	PX_ASSERT(articulationJoint.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	OMNI_PVD_NOTIFY_REMOVE(&articulationJoint);
	PxMutex::ScopedLock lock(mArticulationJointRCPoolLock);
	mArticulationRCJointPool.destroy(&articulationJoint);
}

/////////////////////////////////////////////////////////////////////////////// soft body

#if PX_SUPPORT_GPU_PHYSX
PxSoftBody* NpFactory::createSoftBody(PxCudaContextManager& cudaContextManager)
{
	NpSoftBody* sb;
	{	PxMutex::ScopedLock lock(mSoftBodyPoolLock);
		sb = mSoftBodyPool.construct(cudaContextManager);	}
	OMNI_PVD_NOTIFY_ADD(sb);
	return sb;
}

void NpFactory::releaseSoftBodyToPool(PxSoftBody& softBody)
{
	PX_ASSERT(softBody.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	OMNI_PVD_NOTIFY_REMOVE(&softBody);
	PxMutex::ScopedLock lock(mSoftBodyPoolLock);
	mSoftBodyPool.destroy(static_cast<NpSoftBody*>(&softBody));
}

/////////////////////////////////////////////////////////////////////////////// FEM cloth

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
PxFEMCloth* NpFactory::createFEMCloth(PxCudaContextManager& cudaContextManager)
{
	PxMutex::ScopedLock lock(mFEMClothPoolLock);
	return mFEMClothPool.construct(cudaContextManager);
}

void NpFactory::releaseFEMClothToPool(PxFEMCloth& femCloth)
{
	PX_ASSERT(femCloth.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mFEMClothPoolLock);
	mFEMClothPool.destroy(static_cast<NpFEMCloth*>(&femCloth));
}
#endif

//////////////////////////////////////////////////////////////////////////////// particle system

PxPBDParticleSystem* NpFactory::createPBDParticleSystem(PxU32 maxNeighborhood, PxCudaContextManager& cudaContextManager)
{
	PxMutex::ScopedLock lock(mPBDParticleSystemPoolLock);

	//PX_CHECK_MSG(NpPBDParticleSystem::getNumAvailableSystems() > 0, "Max. number of concurrent PxParticleSystem is 256.");

	return mPBDParticleSystemPool.construct(maxNeighborhood, cudaContextManager);
}

void NpFactory::releasePBDParticleSystemToPool(PxPBDParticleSystem& particleSystem)
{
	PX_ASSERT(particleSystem.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mPBDParticleSystemPoolLock);
	mPBDParticleSystemPool.destroy(static_cast<NpPBDParticleSystem*>(&particleSystem));
}

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
PxFLIPParticleSystem* NpFactory::createFLIPParticleSystem(PxCudaContextManager& cudaContextManager)
{
	PxMutex::ScopedLock lock(mFLIPParticleSystemPoolLock);

	//PX_CHECK_MSG(NpFLIPParticleSystem::getNumAvailableSystems() > 0, "Max. number of concurrent PxParticleSystem is 256.");

	return mFLIPParticleSystemPool.construct(cudaContextManager);
}

void NpFactory::releaseFLIPParticleSystemToPool(PxFLIPParticleSystem& particleSystem)
{
	PX_ASSERT(particleSystem.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mFLIPParticleSystemPoolLock);
	mFLIPParticleSystemPool.destroy(static_cast<NpFLIPParticleSystem*>(&particleSystem));
}

PxMPMParticleSystem* NpFactory::createMPMParticleSystem(PxCudaContextManager& cudaContextManager)
{
	PxMutex::ScopedLock lock(mMPMParticleSystemPoolLock);

	//PX_CHECK_MSG(NpMPMParticleSystem::getNumAvailableSystems() > 0, "Max. number of concurrent PxParticleSystem is 256.");

	return mMPMParticleSystemPool.construct(cudaContextManager);
}

void NpFactory::releaseMPMParticleSystemToPool(PxMPMParticleSystem& particleSystem)
{
	PX_ASSERT(particleSystem.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mMPMParticleSystemPoolLock);
	mMPMParticleSystemPool.destroy(static_cast<NpMPMParticleSystem*>(&particleSystem));
}
#endif

/////////////////////////////////////////////////////////////////////////////// Particle Buffers

PxParticleBuffer* NpFactory::createParticleBuffer(PxU32 maxParticles, PxU32 maxVolumes, PxCudaContextManager* cudaContextManager)
{
	if(!cudaContextManager)
		return NULL;

	PxPhysXGpu* physxGpu = PxvGetPhysXGpu(true);
	PX_ASSERT(physxGpu);

	PxParticleBuffer* buffer = physxGpu->createParticleBuffer(maxParticles, maxVolumes, cudaContextManager, &mGpuMemStat, NpFactory::onParticleBufferRelease);
	addParticleBuffer(buffer);
	return buffer;
}

PxParticleAndDiffuseBuffer* NpFactory::createParticleAndDiffuseBuffer(PxU32 maxParticles, PxU32 maxVolumes, PxU32 maxDiffuseParticles, PxCudaContextManager* cudaContextManager)
{
	if(!cudaContextManager)
		return NULL;

	PxPhysXGpu* physxGpu = PxvGetPhysXGpu(true);
	PX_ASSERT(physxGpu);

	PxParticleAndDiffuseBuffer* diffuseBuffer = physxGpu->createParticleAndDiffuseBuffer(maxParticles, maxVolumes, maxDiffuseParticles, cudaContextManager, &mGpuMemStat, NpFactory::onParticleBufferRelease);
	addParticleBuffer(diffuseBuffer);
	return diffuseBuffer;
}

PxParticleClothBuffer* NpFactory::createParticleClothBuffer(PxU32 maxParticles, PxU32 maxNumVolumes, PxU32 maxNumCloths, PxU32 maxNumTriangles, PxU32 maxNumSprings, PxCudaContextManager* cudaContextManager)
{
	if(!cudaContextManager)
		return NULL;

	PxPhysXGpu* physxGpu = PxvGetPhysXGpu(true);
	PX_ASSERT(physxGpu);

	PxParticleClothBuffer* clothBuffer = physxGpu->createParticleClothBuffer(maxParticles, maxNumVolumes, maxNumCloths, maxNumTriangles, maxNumSprings, cudaContextManager, &mGpuMemStat, NpFactory::onParticleBufferRelease);
	addParticleBuffer(clothBuffer);
	return clothBuffer;
}

PxParticleRigidBuffer* NpFactory::createParticleRigidBuffer(PxU32 maxParticles, PxU32 maxNumVolumes, PxU32 maxNumRigids, PxCudaContextManager* cudaContextManager)
{
	if(!cudaContextManager)
		return NULL;

	PxPhysXGpu* physxGpu = PxvGetPhysXGpu(true);
	PX_ASSERT(physxGpu);

	PxParticleRigidBuffer* rigidBuffer = physxGpu->createParticleRigidBuffer(maxParticles, maxNumVolumes, maxNumRigids, cudaContextManager, &mGpuMemStat, NpFactory::onParticleBufferRelease);
	addParticleBuffer(rigidBuffer);
	return rigidBuffer;
}

void NpFactory::onParticleBufferRelease(PxParticleBuffer* buffer)
{
	NpFactory::getInstance().onParticleBufferReleaseInternal(buffer);
}

/////////////////////////////////////////////////////////////////////////////// HairSystem

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
PxHairSystem* NpFactory::createHairSystem(PxCudaContextManager& cudaContextManager)
{
	PxMutex::ScopedLock lock(mHairSystemPoolLock);
	return mHairSystemPool.construct(cudaContextManager);
}

void NpFactory::releaseHairSystemToPool(PxHairSystem& hairSystem)
{
	PX_ASSERT(hairSystem.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mHairSystemPoolLock);
	mHairSystemPool.destroy(static_cast<NpHairSystem*>(&hairSystem));
}
#endif
#endif

/////////////////////////////////////////////////////////////////////////////// constraint

void NpFactory::addConstraint(PxConstraint* npConstraint, bool lock)
{
	addToTracking(mConstraintTracking, npConstraint, mTrackingMutex, lock);
}

PxConstraint* NpFactory::createConstraint(PxRigidActor* actor0, PxRigidActor* actor1, PxConstraintConnector& connector, const PxConstraintShaderTable& shaders, PxU32 dataSize)
{
	PX_CHECK_AND_RETURN_NULL((actor0 && actor0->getConcreteType()!=PxConcreteType::eRIGID_STATIC) || (actor1 && actor1->getConcreteType()!=PxConcreteType::eRIGID_STATIC), "createConstraint: At least one actor must be dynamic or an articulation link");

	NpConstraint* npConstraint;
	{
		PxMutex::ScopedLock lock(mConstraintPoolLock);
		npConstraint = mConstraintPool.construct(actor0, actor1, connector, shaders, dataSize);
	}
	addConstraint(npConstraint);
	connector.connectToConstraint(npConstraint);
	return npConstraint;
}

void NpFactory::releaseConstraintToPool(NpConstraint& constraint)
{
	PX_ASSERT(constraint.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mConstraintPoolLock);
	mConstraintPool.destroy(&constraint);
}

void NpFactory::onConstraintRelease(PxConstraint* c)
{
	PxMutex::ScopedLock lock(mTrackingMutex);
	mConstraintTracking.erase(c);
}

/////////////////////////////////////////////////////////////////////////////// aggregate

void NpFactory::addAggregate(PxAggregate* npAggregate, bool lock)
{
	addToTracking(mAggregateTracking, npAggregate, mTrackingMutex, lock);
	OMNI_PVD_NOTIFY_ADD(npAggregate);
}

PxAggregate* NpFactory::createAggregate(PxU32 maxActors, PxU32 maxShapes, PxAggregateFilterHint filterHint)
{
	NpAggregate* npAggregate;
	{
		PxMutex::ScopedLock lock(mAggregatePoolLock);
		npAggregate = mAggregatePool.construct(maxActors, maxShapes, filterHint);
	}

	addAggregate(npAggregate);
	return npAggregate;
}

void NpFactory::releaseAggregateToPool(NpAggregate& aggregate)
{
	PX_ASSERT(aggregate.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mAggregatePoolLock);
	mAggregatePool.destroy(&aggregate);
}

void NpFactory::onAggregateRelease(PxAggregate* a)
{
	OMNI_PVD_NOTIFY_REMOVE(a);
	PxMutex::ScopedLock lock(mTrackingMutex);
	mAggregateTracking.erase(a);
}

///////////////////////////////////////////////////////////////////////////////

PxMaterial* NpFactory::createMaterial(PxReal staticFriction, PxReal dynamicFriction, PxReal restitution)
{
	PX_CHECK_AND_RETURN_NULL(dynamicFriction >= 0.0f, "createMaterial: dynamicFriction must be >= 0.");
	PX_CHECK_AND_RETURN_NULL(staticFriction >= 0.0f, "createMaterial: staticFriction must be >= 0.");
	PX_CHECK_AND_RETURN_NULL(restitution >= 0.0f || restitution <= 1.0f, "createMaterial: restitution must be between 0 and 1.");

	PxsMaterialData materialData;
	materialData.staticFriction = staticFriction;
	materialData.dynamicFriction = dynamicFriction;
	materialData.restitution = restitution;

	NpMaterial* npMaterial;
	{
		PxMutex::ScopedLock lock(mMaterialPoolLock);		
		npMaterial = mMaterialPool.construct(materialData);
	}
	return npMaterial;	
}

void NpFactory::releaseMaterialToPool(NpMaterial& material)
{
	PX_ASSERT(material.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mMaterialPoolLock);
	mMaterialPool.destroy(&material);
}

///////////////////////////////////////////////////////////////////////////////

#if PX_SUPPORT_GPU_PHYSX
PxFEMSoftBodyMaterial* NpFactory::createFEMSoftBodyMaterial(PxReal youngs, PxReal poissons, PxReal dynamicFriction)
{
#if PX_SUPPORT_GPU_PHYSX
	PX_CHECK_AND_RETURN_NULL(youngs >= 0.0f, "createFEMSoftBodyMaterial: youngs must be >= 0.");
	PX_CHECK_AND_RETURN_NULL(poissons >= 0.0f && poissons < 0.5f, "createFEMSoftBodyMaterial: poissons must be in range[0.f, 0.5f).");
	PX_CHECK_AND_RETURN_NULL(dynamicFriction >= 0.0f, "createMaterial: dynamicFriction must be >= 0.");

	PxsFEMSoftBodyMaterialData materialData;
	materialData.youngs = youngs;
	materialData.poissons = poissons;
	materialData.dynamicFriction = dynamicFriction;
	materialData.damping = 0.f;
	materialData.dampingScale = toUniformU16(1.f);
	materialData.materialModel = PxFEMSoftBodyMaterialModel::eCO_ROTATIONAL;
	materialData.deformThreshold = PX_MAX_F32;
	materialData.deformLowLimitRatio = 1.f;
	materialData.deformHighLimitRatio = 1.f;

	NpFEMSoftBodyMaterial* npMaterial;
	{
		PxMutex::ScopedLock lock(mFEMMaterialPoolLock);
		npMaterial = mFEMMaterialPool.construct(materialData);
	}
	return npMaterial;

#else
	PX_UNUSED(youngs);
	PX_UNUSED(poissons);
	PX_UNUSED(dynamicFriction);
	PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxFEMMaterial is not supported on this platform.");
	return NULL;
#endif
}

void NpFactory::releaseFEMMaterialToPool(PxFEMSoftBodyMaterial& material_)
{
#if PX_SUPPORT_GPU_PHYSX
	NpFEMSoftBodyMaterial& material = static_cast<NpFEMSoftBodyMaterial&>(material_);
	PX_ASSERT(material.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mFEMMaterialPoolLock);
	mFEMMaterialPool.destroy(&material);
#else
	PX_UNUSED(material_);
#endif
}

///////////////////////////////////////////////////////////////////////////////

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
PxFEMClothMaterial* NpFactory::createFEMClothMaterial(PxReal youngs, PxReal poissons, PxReal dynamicFriction, PxReal thickness)
{
#if PX_SUPPORT_GPU_PHYSX
	PX_CHECK_AND_RETURN_NULL(youngs >= 0.0f, "createFEMClothMaterial: youngs must be >= 0.");
	PX_CHECK_AND_RETURN_NULL(poissons >= 0.0f && poissons < 0.5f, "createFEMClothMaterial: poissons must be in range[0.f, 0.5f).");
	PX_CHECK_AND_RETURN_NULL(dynamicFriction >= 0.0f, "createMaterial: dynamicFriction must be >= 0.");
	PX_CHECK_AND_RETURN_NULL(thickness >= 0.0f, "createMaterial: thickness must be > 0.");

	PxsFEMClothMaterialData materialData;
	materialData.youngs = youngs;
	materialData.poissons = poissons;
	materialData.dynamicFriction = dynamicFriction;
	materialData.thickness = thickness;

	NpFEMClothMaterial* npMaterial = NULL;
	{
		PxMutex::ScopedLock lock(mFEMClothMaterialPoolLock);
		npMaterial = mFEMClothMaterialPool.construct(materialData);
	}
	return npMaterial;
#else
	PX_UNUSED(youngs);
	PX_UNUSED(poissons);
	PX_UNUSED(dynamicFriction);
	PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxFEMClothMaterial is not supported on this platform.");
	return NULL;
#endif
}

void NpFactory::releaseFEMClothMaterialToPool(PxFEMClothMaterial& material_)
{
#if PX_SUPPORT_GPU_PHYSX
	NpFEMClothMaterial& material = static_cast<NpFEMClothMaterial&>(material_);
	PX_ASSERT(material.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mFEMClothMaterialPoolLock);
	mFEMClothMaterialPool.destroy(&material);
#else
	PX_UNUSED(material_);
#endif
}
#endif

///////////////////////////////////////////////////////////////////////////////

PxPBDMaterial* NpFactory::createPBDMaterial(PxReal friction, PxReal damping, PxReal adhesion, PxReal viscosity, PxReal vorticityConfinement, 
	PxReal surfaceTension, PxReal cohesion, PxReal lift, PxReal drag, PxReal cflCoefficient, PxReal gravityScale)
{
#if PX_SUPPORT_GPU_PHYSX

	PxsPBDMaterialData materialData;
	materialData.friction = friction;
	materialData.damping = damping;
	materialData.viscosity = viscosity;
	materialData.vorticityConfinement = vorticityConfinement;
	materialData.surfaceTension = surfaceTension;
	materialData.cohesion = cohesion;
	materialData.adhesion = adhesion;
	materialData.lift = lift;
	materialData.drag = drag;
	materialData.cflCoefficient = cflCoefficient;
	materialData.gravityScale = gravityScale;
	materialData.particleFrictionScale = 1.f;
	materialData.adhesionRadiusScale = 0.f;
	materialData.particleAdhesionScale = 1.f;

	NpPBDMaterial* npMaterial;
	{
		PxMutex::ScopedLock lock(mPBDMaterialPoolLock);
		npMaterial = mPBDMaterialPool.construct(materialData);
	}
	return npMaterial;
#else
	PX_UNUSED(friction);
	PX_UNUSED(damping);
	PX_UNUSED(adhesion);
	PX_UNUSED(viscosity);
	PX_UNUSED(vorticityConfinement);
	PX_UNUSED(surfaceTension);
	PX_UNUSED(cohesion);
	PX_UNUSED(lift);
	PX_UNUSED(drag);
	PX_UNUSED(cflCoefficient);
	PX_UNUSED(gravityScale);
	PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxPBDMaterial is not supported on this platform.");
	return NULL;
#endif
}

void NpFactory::releasePBDMaterialToPool(PxPBDMaterial& material_)
{
#if PX_SUPPORT_GPU_PHYSX
	NpPBDMaterial& material = static_cast<NpPBDMaterial&>(material_);
	PX_ASSERT(material.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mPBDMaterialPoolLock);
	mPBDMaterialPool.destroy(&material);
#else
	PX_UNUSED(material_);
#endif
}

///////////////////////////////////////////////////////////////////////////////

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
PxFLIPMaterial* NpFactory::createFLIPMaterial(PxReal friction, PxReal damping, PxReal adhesion, PxReal viscosity, PxReal gravityScale)
{
#if PX_SUPPORT_GPU_PHYSX

	PxsFLIPMaterialData materialData;
	materialData.friction = friction;
	materialData.damping = damping;
	materialData.adhesion = adhesion;
	materialData.viscosity = viscosity;
	materialData.gravityScale = gravityScale;
	materialData.adhesionRadiusScale = 0.f;

	NpFLIPMaterial* npMaterial;
	{
		PxMutex::ScopedLock lock(mFLIPMaterialPoolLock);
		npMaterial = mFLIPMaterialPool.construct(materialData);
	}
	return npMaterial;

#else
	PX_UNUSED(friction);
	PX_UNUSED(damping);
	PX_UNUSED(adhesion);
	PX_UNUSED(viscosity);
	PX_UNUSED(gravityScale);
	PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxFLIPMaterial is not supported on this platform.");
	return NULL;
#endif
}

void NpFactory::releaseFLIPMaterialToPool(PxFLIPMaterial& material_)
{
#if PX_SUPPORT_GPU_PHYSX
	NpFLIPMaterial& material = static_cast<NpFLIPMaterial&>(material_);
	PX_ASSERT(material.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mFLIPMaterialPoolLock);
	mFLIPMaterialPool.destroy(&material);
#else
	PX_UNUSED(material_);
#endif
}

///////////////////////////////////////////////////////////////////////////////

PxMPMMaterial* NpFactory::createMPMMaterial(
	PxReal friction, PxReal damping, PxReal adhesion, 
	bool isPlastic, PxReal youngsModulus, PxReal poissons, PxReal hardening, 
	PxReal criticalCompression, PxReal criticalStretch, 
	PxReal tensileDamageSensitivity, PxReal compressiveDamageSensitivity, 
	PxReal attractiveForceResidual, PxReal gravityScale)
{
#if PX_SUPPORT_GPU_PHYSX
	PxsMPMMaterialData materialData;
	materialData.friction = friction;
	materialData.damping = damping;
	materialData.adhesion = adhesion;
	materialData.adhesionRadiusScale = 0.f;
	materialData.isPlastic = isPlastic;
	materialData.youngsModulus = youngsModulus;
	materialData.poissonsRatio = poissons;
	materialData.hardening = hardening;
	materialData.criticalCompression = criticalCompression;
	materialData.criticalStretch = criticalStretch;
	materialData.tensileDamageSensitivity = tensileDamageSensitivity;
	materialData.compressiveDamageSensitivity = compressiveDamageSensitivity;
	materialData.attractiveForceResidual = attractiveForceResidual;
	materialData.gravityScale = gravityScale;
	materialData.materialModel = PxMPMMaterialModel::eNEO_HOOKEAN;
	materialData.cuttingFlags = PxMPMCuttingFlag::eNONE;
	materialData.yieldStress = 30000;
	materialData.sandFrictionAngle = 3.1415926535898f * 0.25f;
	materialData.density = 500.0f;
	materialData.stretchAndShearDamping = 0.0f;
	materialData.rotationalDamping = 0.0f;

	NpMPMMaterial* npMaterial;
	{
		PxMutex::ScopedLock lock(mMPMMaterialPoolLock);
		npMaterial = mMPMMaterialPool.construct(materialData);
	}
	return npMaterial;

#else
	PX_UNUSED(friction);
	PX_UNUSED(damping);
	PX_UNUSED(adhesion);
	PX_UNUSED(isPlastic);
	PX_UNUSED(youngsModulus);
	PX_UNUSED(poissons);
	PX_UNUSED(hardening);
	PX_UNUSED(criticalCompression);
	PX_UNUSED(criticalStretch);
	PX_UNUSED(tensileDamageSensitivity);
	PX_UNUSED(compressiveDamageSensitivity);
	PX_UNUSED(attractiveForceResidual);
	PX_UNUSED(gravityScale);
	PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxMPMMaterial is not supported on this platform.");
	return NULL;
#endif
}

void NpFactory::releaseMPMMaterialToPool(PxMPMMaterial& material_)
{
#if PX_SUPPORT_GPU_PHYSX
	NpMPMMaterial& material = static_cast<NpMPMMaterial&>(material_);
	PX_ASSERT(material.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mMPMMaterialPoolLock);
	mMPMMaterialPool.destroy(&material);
#else
	PX_UNUSED(material_);
#endif
}

#endif // PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
#endif // PX_SUPPORT_GPU_PHYSX

///////////////////////////////////////////////////////////////////////////////

NpConnectorArray* NpFactory::acquireConnectorArray()
{
	PxMutexT<>::ScopedLock l(mConnectorArrayPoolLock);
	return mConnectorArrayPool.construct();
}

void NpFactory::releaseConnectorArray(NpConnectorArray* array)
{
	PxMutexT<>::ScopedLock l(mConnectorArrayPoolLock);
	mConnectorArrayPool.destroy(array);
}

///////////////////////////////////////////////////////////////////////////////

#if PX_CHECKED
bool checkShape(const PxGeometry& g, const char* errorMsg)
{
	const bool isValid = PxGeometryQuery::isValid(g);
	if(!isValid)
		PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, errorMsg);
	return isValid;
}
#endif

template <typename PxMaterialType, typename NpMaterialType>
NpShape* NpFactory::createShapeInternal(const PxGeometry& geometry,
	PxShapeFlags shapeFlags,
	PxMaterialType*const* materials,
	PxU16 materialCount,
	bool isExclusive,
	PxShapeCoreFlag::Enum flag)
{
#if PX_CHECKED
	if(!checkShape(geometry, "Supplied PxGeometry is not valid. Shape creation method returns NULL."))
		return NULL;

	// Check for invalid material table setups
	if(!NpShape::checkMaterialSetup(geometry, "Shape creation", materials, materialCount))
		return NULL;
#endif

	PxInlineArray<PxU16, 4> materialIndices("NpFactory::TmpMaterialIndexBuffer");
	materialIndices.resize(materialCount);
	if (materialCount == 1)
		materialIndices[0] = static_cast<NpMaterialType*>(materials[0])->mMaterial.mMaterialIndex;
	else
		NpMaterialType::getMaterialIndices(materials, materialIndices.begin(), materialCount);

	NpShape* npShape;
	{
		PxMutex::ScopedLock lock(mShapePoolLock);
		PxU16* mi = materialIndices.begin(); // required to placate pool constructor arg passing
		npShape = mShapePool.construct(geometry, shapeFlags, mi, materialCount, isExclusive, flag);
	}

	if (!npShape)
		return NULL;

	// PT: TODO: add material base class, move this to NpShape, drop getMaterial<>
	for (PxU32 i = 0; i < materialCount; i++)
	{
		PxMaterialType* mat = npShape->getMaterial<PxMaterialType, NpMaterialType>(i);
		RefCountable_incRefCount(*mat);
	}
	addShape(npShape);

	return npShape;
}

NpShape* NpFactory::createShape(const PxGeometry& geometry,
								PxShapeFlags shapeFlags,
								PxMaterial*const* materials,
								PxU16 materialCount,
								bool isExclusive)
{
	return createShapeInternal<PxMaterial, NpMaterial>(geometry, shapeFlags, materials, materialCount, isExclusive, PxShapeCoreFlag::Enum(0));
}

NpShape* NpFactory::createShape(const PxGeometry& geometry,
	PxShapeFlags shapeFlags,
	PxFEMSoftBodyMaterial*const* materials,
	PxU16 materialCount,
	bool isExclusive)
{
#if PX_SUPPORT_GPU_PHYSX
	return createShapeInternal<PxFEMSoftBodyMaterial, NpFEMSoftBodyMaterial>(geometry, shapeFlags, materials, materialCount, isExclusive, PxShapeCoreFlag::eSOFT_BODY_SHAPE);
#else
	PX_UNUSED(geometry); PX_UNUSED(shapeFlags); PX_UNUSED(materials); PX_UNUSED(materialCount); PX_UNUSED(isExclusive);
	return NULL;
#endif
}

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION && PX_SUPPORT_GPU_PHYSX
NpShape* NpFactory::createShape(const PxGeometry& geometry,
	PxShapeFlags shapeFlags,
	PxFEMClothMaterial*const* materials,
	PxU16 materialCount,
	bool isExclusive)
{
	return createShapeInternal<PxFEMClothMaterial, NpFEMClothMaterial>(geometry, shapeFlags, materials, materialCount, isExclusive, PxShapeCoreFlag::eCLOTH_SHAPE);
}
#endif

void NpFactory::releaseShapeToPool(NpShape& shape)
{
	PX_ASSERT(shape.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mShapePoolLock);
	mShapePool.destroy(&shape);
}

PxU32 NpFactory::getNbShapes() const
{
	// PT: TODO: isn't there a lock missing here? See usage in MeshFactory
	return mShapeTracking.size();
}

PxU32 NpFactory::getShapes(PxShape** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const
{
	// PT: TODO: isn't there a lock missing here? See usage in MeshFactory
	return getArrayOfPointers(userBuffer, bufferSize, startIndex, mShapeTracking.getEntries(), mShapeTracking.size());
}

///////////////////////////////////////////////////////////////////////////////

PxRigidStatic* NpFactory::createRigidStatic(const PxTransform& pose)
{
	PX_CHECK_AND_RETURN_NULL(pose.isValid(), "pose is not valid. createRigidStatic returns NULL.");

	NpRigidStatic* npActor;
	{
		PxMutex::ScopedLock lock(mRigidStaticPoolLock);
		npActor = mRigidStaticPool.construct(pose);
	}
	addRigidStatic(npActor);
	return npActor;
}

void NpFactory::releaseRigidStaticToPool(NpRigidStatic& rigidStatic)
{
	PX_ASSERT(rigidStatic.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mRigidStaticPoolLock);
	mRigidStaticPool.destroy(&rigidStatic);
}

///////////////////////////////////////////////////////////////////////////////

PxRigidDynamic* NpFactory::createRigidDynamic(const PxTransform& pose)
{
	PX_CHECK_AND_RETURN_NULL(pose.isValid(), "pose is not valid. createRigidDynamic returns NULL.");

	NpRigidDynamic* npBody;
	{
		PxMutex::ScopedLock lock(mRigidDynamicPoolLock);
		npBody = mRigidDynamicPool.construct(pose);
	}
	addRigidDynamic(npBody);
	return npBody;
}

void NpFactory::releaseRigidDynamicToPool(NpRigidDynamic& rigidDynamic)
{
	PX_ASSERT(rigidDynamic.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mRigidDynamicPoolLock);
	mRigidDynamicPool.destroy(&rigidDynamic);
}

///////////////////////////////////////////////////////////////////////////////

// PT: this function is here to minimize the amount of locks when deserializing a collection
void NpFactory::addCollection(const Collection& collection)
{
	PxU32 nb = collection.getNbObjects();
	const PxPair<PxBase* const, PxSerialObjectId>* entries = collection.internalGetObjects();
	// PT: we take the lock only once, here
	PxMutex::ScopedLock lock(mTrackingMutex);

	for(PxU32 i=0;i<nb;i++)
	{
		PxBase* s = entries[i].first;
		const PxType serialType = s->getConcreteType();
//////////////////////////
		if(serialType==PxConcreteType::eHEIGHTFIELD)
		{
			Gu::HeightField* gu = static_cast<Gu::HeightField*>(s);
			gu->setMeshFactory(this);
			addHeightField(gu, false);
		}
		else if(serialType==PxConcreteType::eCONVEX_MESH)
		{
			Gu::ConvexMesh* gu = static_cast<Gu::ConvexMesh*>(s);
			gu->setMeshFactory(this);
			addConvexMesh(gu, false);
		}
		else if(serialType==PxConcreteType::eTRIANGLE_MESH_BVH33 || serialType==PxConcreteType::eTRIANGLE_MESH_BVH34)
		{
			Gu::TriangleMesh* gu = static_cast<Gu::TriangleMesh*>(s);
			gu->setMeshFactory(this);
			addTriangleMesh(gu, false);
		}
		else if(serialType==PxConcreteType::eRIGID_DYNAMIC)
		{
			NpRigidDynamic* np = static_cast<NpRigidDynamic*>(s);
			addRigidDynamic(np, false);
		}
		else if(serialType==PxConcreteType::eRIGID_STATIC)
		{
			NpRigidStatic* np = static_cast<NpRigidStatic*>(s);
			addRigidStatic(np, false);
		}
		else if(serialType==PxConcreteType::eSHAPE)
		{
			NpShape* np = static_cast<NpShape*>(s);
			addShape(np, false);
		}
		else if(serialType==PxConcreteType::eMATERIAL)
		{
		}
		else if(serialType==PxConcreteType::eCONSTRAINT)
		{
			NpConstraint* np = static_cast<NpConstraint*>(s);
			addConstraint(np, false);
		}
		else if(serialType==PxConcreteType::eAGGREGATE)
		{
			NpAggregate* np = static_cast<NpAggregate*>(s);
			addAggregate(np, false);

			// PT: TODO: double-check this.... is it correct?			
			for(PxU32 j=0;j<np->getCurrentSizeFast();j++)
			{
				PxBase* actor = np->getActorFast(j);
				const PxType serialType1 = actor->getConcreteType();

				if(serialType1==PxConcreteType::eRIGID_STATIC)
					addRigidStatic(static_cast<NpRigidStatic*>(actor), false);
				else if(serialType1==PxConcreteType::eRIGID_DYNAMIC)
					addRigidDynamic(static_cast<NpRigidDynamic*>(actor), false);
				else if(serialType1==PxConcreteType::eARTICULATION_LINK)
				{}
				else PX_ASSERT(0);
			}
		}
		else if (serialType == PxConcreteType::eARTICULATION_REDUCED_COORDINATE)
		{
			NpArticulationReducedCoordinate* np = static_cast<NpArticulationReducedCoordinate*>(s);
			addArticulation(np, false);
		}
		else if(serialType==PxConcreteType::eARTICULATION_LINK)
		{
//			NpArticulationLink* np = static_cast<NpArticulationLink*>(s);
		}
		else if(serialType==PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE)
		{
//			NpArticulationJoint* np = static_cast<NpArticulationJoint*>(s);
		}
		else
		{
//			assert(0);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////

#if PX_SUPPORT_PVD
void NpFactory::setNpFactoryListener( NpFactoryListener& inListener)
{
	mNpFactoryListener = &inListener;
	addFactoryListener(inListener);
}
#endif

///////////////////////////////////////////////////////////////////////////////

static PX_FORCE_INLINE void releaseToPool(NpRigidStatic* np)
{
	NpFactory::getInstance().releaseRigidStaticToPool(*np);
}

static PX_FORCE_INLINE void releaseToPool(NpRigidDynamic* np)
{
	NpFactory::getInstance().releaseRigidDynamicToPool(*np);
}

static PX_FORCE_INLINE void releaseToPool(NpArticulationLink* np)
{
	NpFactory::getInstance().releaseArticulationLinkToPool(*np);
}

static PX_FORCE_INLINE void releaseToPool(PxArticulationJointReducedCoordinate* np)
{
	PX_ASSERT(np->getConcreteType() == PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE);
	NpFactory::getInstance().releaseArticulationJointRCToPool(*static_cast<NpArticulationJointReducedCoordinate*>(np));
}

static PX_FORCE_INLINE void releaseToPool(PxArticulationReducedCoordinate* np)
{
	NpFactory::getInstance().releaseArticulationToPool(*np);
}

static PX_FORCE_INLINE void releaseToPool(NpAggregate* np)
{
	NpFactory::getInstance().releaseAggregateToPool(*np);
}

static PX_FORCE_INLINE void releaseToPool(NpShape* np)
{
	NpFactory::getInstance().releaseShapeToPool(*np);
}

static PX_FORCE_INLINE void releaseToPool(NpConstraint* np)
{
	NpFactory::getInstance().releaseConstraintToPool(*np);
}

#if PX_SUPPORT_GPU_PHYSX
static PX_FORCE_INLINE void releaseToPool(NpSoftBody* np)
{
	NpFactory::getInstance().releaseSoftBodyToPool(*np);
}

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
static PX_FORCE_INLINE void releaseToPool(NpFEMCloth* np)
{
	NpFactory::getInstance().releaseFEMClothToPool(*np);
}
#endif

static PX_FORCE_INLINE void releaseToPool(NpPBDParticleSystem* np)
{
	NpFactory::getInstance().releasePBDParticleSystemToPool(*np);
}

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
static PX_FORCE_INLINE void releaseToPool(NpFLIPParticleSystem* np)
{
	NpFactory::getInstance().releaseFLIPParticleSystemToPool(*np);
}
#endif

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
static PX_FORCE_INLINE void releaseToPool(NpMPMParticleSystem* np)
{
	NpFactory::getInstance().releaseMPMParticleSystemToPool(*np);
}
#endif

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
static PX_FORCE_INLINE void releaseToPool(NpHairSystem* np)
{
	NpFactory::getInstance().releaseHairSystemToPool(*np);
}
#endif
#endif

template<class T>
static PX_FORCE_INLINE void NpDestroy(T* np)
{
	void* ud = np->userData;

	if(np->getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
		releaseToPool(np);
	else
		np->~T();

	NpPhysics::getInstance().notifyDeletionListenersMemRelease(np, ud);
}

void physx::NpDestroyRigidActor(NpRigidStatic* np)									{ NpDestroy(np);	}
void physx::NpDestroyRigidDynamic(NpRigidDynamic* np)								{ NpDestroy(np);	}
void physx::NpDestroyAggregate(NpAggregate* np)										{ NpDestroy(np);	}
void physx::NpDestroyShape(NpShape* np)												{ NpDestroy(np);	}
void physx::NpDestroyConstraint(NpConstraint* np)									{ NpDestroy(np);	}
void physx::NpDestroyArticulationLink(NpArticulationLink* np)						{ NpDestroy(np);	}
void physx::NpDestroyArticulationJoint(PxArticulationJointReducedCoordinate* np)	{ NpDestroy(np);	}
void physx::NpDestroyArticulation(PxArticulationReducedCoordinate* np)				{ NpDestroy(np);	}
#if PX_SUPPORT_GPU_PHYSX
void physx::NpDestroySoftBody(NpSoftBody* np)										{ NpDestroy(np);	}
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
void physx::NpDestroyFEMCloth(NpFEMCloth* np)										{ NpDestroy(np);	}
#endif
void physx::NpDestroyParticleSystem(NpPBDParticleSystem* np)						{ NpDestroy(np);	}
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
void physx::NpDestroyParticleSystem(NpFLIPParticleSystem* np)						{ NpDestroy(np);	}
void physx::NpDestroyParticleSystem(NpMPMParticleSystem* np)						{ NpDestroy(np);	}
void physx::NpDestroyHairSystem(NpHairSystem* np)									{ NpDestroy(np);	}
#endif
#endif
