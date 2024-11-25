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

#if PX_SUPPORT_GPU_PHYSX
#include "NpPBDParticleSystem.h"
#include "NpParticleBuffer.h"
#include "NpDeformableSurface.h"
#include "NpDeformableVolume.h"
#include "NpDeformableAttachment.h"
#include "NpDeformableElementFilter.h"
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
	releaseAll(mAttachmentTracking);
	releaseAll(mElementFilterTracking);
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
		PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "Articulation link initialization failed: returned NULL.");
		return NULL;
	}
	OMNI_PVD_NOTIFY_ADD(npArticulationLink);
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

NpArticulationMimicJoint* NpFactory::createNpArticulationMimicJoint
(const PxArticulationJointReducedCoordinate& jointA, const PxArticulationAxis::Enum axisA, 
 const PxArticulationJointReducedCoordinate& jointB, const PxArticulationAxis::Enum axisB, 
 const PxReal gearRatio, const PxReal offset,
 const PxReal naturalFrequency, const PxReal dampingRatio)
{
	NpArticulationMimicJoint* npArticulationMimicJoint;
	{
		PxMutex::ScopedLock lock(mArticulationMimicJointPoolLock);
		npArticulationMimicJoint = mArticulationMimicJointPool.construct(jointA, axisA, jointB, axisB, gearRatio, offset, naturalFrequency, dampingRatio);
	}
	OMNI_PVD_NOTIFY_ADD(npArticulationMimicJoint);
	return npArticulationMimicJoint;
}

void NpFactory::releaseArticulationMimicJointToPool(NpArticulationMimicJoint& articulationMimicJoint)
{
	PX_ASSERT(articulationMimicJoint.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	OMNI_PVD_NOTIFY_REMOVE(&articulationMimicJoint);
	PxMutex::ScopedLock lock(mArticulationMimicJointPoolLock);
	mArticulationMimicJointPool.destroy(&articulationMimicJoint);
}

#if PX_SUPPORT_GPU_PHYSX

/////////////////////////////////////////////////////////////////////////////// deformable surface

PxDeformableSurface* NpFactory::createDeformableSurface(PxCudaContextManager& cudaContextManager)
{
	NpDeformableSurface* ds;
	{	PxMutex::ScopedLock lock(mDeformableSurfacePoolLock);
	ds = mDeformableSurfacePool.construct(cudaContextManager);	}
	OMNI_PVD_NOTIFY_ADD(ds);
	return ds;
}

void NpFactory::releaseDeformableSurfaceToPool(PxDeformableSurface& deformableSurface)
{
	PX_ASSERT(deformableSurface.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	OMNI_PVD_NOTIFY_REMOVE(&deformableSurface);
	PxMutex::ScopedLock lock(mDeformableSurfacePoolLock);
	mDeformableSurfacePool.destroy(static_cast<NpDeformableSurface*>(&deformableSurface));
}

/////////////////////////////////////////////////////////////////////////////// deformable volume

PxDeformableVolume* NpFactory::createDeformableVolume(PxCudaContextManager& cudaContextManager)
{
	NpDeformableVolume* dv;
	{	PxMutex::ScopedLock lock(mDeformableVolumePoolLock);
	dv = mDeformableVolumePool.construct(cudaContextManager);	}
	OMNI_PVD_NOTIFY_ADD(dv);
	return dv;
}

void NpFactory::releaseDeformableVolumeToPool(PxDeformableVolume& deformableVolume)
{
	PX_ASSERT(deformableVolume.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	OMNI_PVD_NOTIFY_REMOVE(&deformableVolume);
	PxMutex::ScopedLock lock(mDeformableVolumePoolLock);
	mDeformableVolumePool.destroy(static_cast<NpDeformableVolume*>(&deformableVolume));
}

/////////////////////////////////////////////////////////////////////////////// attachment

void NpFactory::addAttachment(PxDeformableAttachment* npAttachment, bool lock)
{
	addToTracking(mAttachmentTracking, npAttachment, mTrackingMutex, lock);
	OMNI_PVD_NOTIFY_ADD(npAttachment);
}

PxDeformableAttachment* NpFactory::createDeformableAttachment(const PxDeformableAttachmentData& data)
{
	AttachmentInfo info;

	if (NpDeformableAttachment::parseAttachment(data, info))
	{
		NpDeformableAttachment* npAttachment;
		{
			PxMutex::ScopedLock lock(mAttachmentPoolLock);
			npAttachment = mAttachmentPool.construct(data, info);
		}

		addAttachment(npAttachment);
		return npAttachment;
	}

	return NULL;
}

void NpFactory::releaseAttachmentToPool(PxDeformableAttachment& attachment)
{
	PX_ASSERT(attachment.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mAttachmentPoolLock);
	mAttachmentPool.destroy(static_cast<NpDeformableAttachment*>(&attachment));
}

void NpFactory::onAttachmentRelease(PxDeformableAttachment* a)
{
	OMNI_PVD_NOTIFY_REMOVE(a);
	PxMutex::ScopedLock lock(mTrackingMutex);
	mAttachmentTracking.erase(a);
}

/////////////////////////////////////////////////////////////////////////////// element filter

void NpFactory::addElementFilter(PxDeformableElementFilter* npElementFilter, bool lock)
{
	addToTracking(mElementFilterTracking, npElementFilter, mTrackingMutex, lock);
	OMNI_PVD_NOTIFY_ADD(npElementFilter);
}

PxDeformableElementFilter* NpFactory::createDeformableElementFilter(const PxDeformableElementFilterData& data)
{
	ElementFilterInfo info;

	if (NpDeformableElementFilter::parseElementFilter(data, info))
	{
		NpDeformableElementFilter* npElementFilter;
		{
			PxMutex::ScopedLock lock(mElementFilterPoolLock);
			npElementFilter = mElementFilterPool.construct(data, info);
		}

		addElementFilter(npElementFilter);
		return npElementFilter;
	}

	return NULL;
}

void NpFactory::releaseElementFilterToPool(PxDeformableElementFilter& elementFilter)
{
	PX_ASSERT(elementFilter.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mElementFilterPoolLock);
	mElementFilterPool.destroy(static_cast<NpDeformableElementFilter*>(&elementFilter));
}

void NpFactory::onElementFilterRelease(PxDeformableElementFilter* e)
{
	OMNI_PVD_NOTIFY_REMOVE(e);
	PxMutex::ScopedLock lock(mTrackingMutex);
	mElementFilterTracking.erase(e);
}

//////////////////////////////////////////////////////////////////////////////// particle system

PxPBDParticleSystem* NpFactory::createPBDParticleSystem(PxU32 maxNeighborhood, PxReal neighborhoodScale, PxCudaContextManager& cudaContextManager)
{
	PxMutex::ScopedLock lock(mPBDParticleSystemPoolLock);
	PxPBDParticleSystem* ps = mPBDParticleSystemPool.construct(maxNeighborhood, neighborhoodScale, cudaContextManager);
	OMNI_PVD_NOTIFY_ADD(ps);
	return ps;
}

void NpFactory::releasePBDParticleSystemToPool(PxPBDParticleSystem& particleSystem)
{
	PX_ASSERT(particleSystem.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	OMNI_PVD_NOTIFY_REMOVE(&particleSystem);
	PxMutex::ScopedLock lock(mPBDParticleSystemPoolLock);
	mPBDParticleSystemPool.destroy(static_cast<NpPBDParticleSystem*>(&particleSystem));
}

/////////////////////////////////////////////////////////////////////////////// Particle Buffers

PxParticleBuffer* NpFactory::createParticleBuffer(PxU32 maxParticles, PxU32 maxVolumes, PxCudaContextManager& cudaContextManager)
{
	PxMutex::ScopedLock lock(mParticleBufferPoolLock);
	PxParticleBuffer* buffer = mParticleBufferPool.construct(maxParticles, maxVolumes, cudaContextManager);
	addParticleBuffer(buffer);
	return buffer;
}

PxParticleAndDiffuseBuffer* NpFactory::createParticleAndDiffuseBuffer(PxU32 maxParticles, PxU32 maxVolumes, PxU32 maxDiffuseParticles, PxCudaContextManager& cudaContextManager)
{
	PxMutex::ScopedLock lock(mParticleAndDiffuseBufferPoolLock);
	PxParticleAndDiffuseBuffer* buffer = mParticleAndDiffuseBufferPool.construct(maxParticles, maxVolumes, maxDiffuseParticles, cudaContextManager);
	addParticleBuffer(buffer);
	return buffer;
}

PxParticleClothBuffer* NpFactory::createParticleClothBuffer(PxU32 maxParticles, PxU32 maxNumVolumes, PxU32 maxNumCloths, PxU32 maxNumTriangles, PxU32 maxNumSprings, PxCudaContextManager& cudaContextManager)
{
	PxMutex::ScopedLock lock(mParticleClothBufferPoolLock);
	PxParticleClothBuffer* buffer = mParticleClothBufferPool.construct(maxParticles, maxNumVolumes, maxNumCloths, maxNumTriangles, maxNumSprings, cudaContextManager);
	addParticleBuffer(buffer);
	return buffer;
}

PxParticleRigidBuffer* NpFactory::createParticleRigidBuffer(PxU32 maxParticles, PxU32 maxNumVolumes, PxU32 maxNumRigids, PxCudaContextManager& cudaContextManager)
{
	PxMutex::ScopedLock lock(mParticleRigidBufferPoolLock);
	PxParticleRigidBuffer* buffer = mParticleRigidBufferPool.construct(maxParticles, maxNumVolumes, maxNumRigids, cudaContextManager);
	addParticleBuffer(buffer);
	return buffer;
}

void NpFactory::addParticleBuffer(PxParticleBuffer* buffer, bool lock)
{
	addToTracking(mParticleBufferTracking, buffer, mTrackingMutex, lock);
	OMNI_PVD_NOTIFY_ADD(buffer);
}

void NpFactory::releaseParticleBufferToPool(PxParticleBuffer& particleBuffer)
{
	PX_ASSERT(particleBuffer.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mParticleBufferPoolLock);
	mParticleBufferPool.destroy(static_cast<NpParticleBuffer*>(&particleBuffer));
}

void NpFactory::releaseParticleAndDiffuseBufferToPool(PxParticleAndDiffuseBuffer& particleBuffer)
{
	PX_ASSERT(particleBuffer.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mParticleAndDiffuseBufferPoolLock);
	mParticleAndDiffuseBufferPool.destroy(static_cast<NpParticleAndDiffuseBuffer*>(&particleBuffer));
}

void NpFactory::releaseParticleClothBufferToPool(PxParticleClothBuffer& particleBuffer)
{
	PX_ASSERT(particleBuffer.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mParticleClothBufferPoolLock);
	mParticleClothBufferPool.destroy(static_cast<NpParticleClothBuffer*>(&particleBuffer));
}

void NpFactory::releaseParticleRigidBufferToPool(PxParticleRigidBuffer& particleBuffer)
{
	PX_ASSERT(particleBuffer.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mParticleRigidBufferPoolLock);
	mParticleRigidBufferPool.destroy(static_cast<NpParticleRigidBuffer*>(&particleBuffer));
}

void NpFactory::onParticleBufferRelease(PxParticleBuffer* buffer)
{
	OMNI_PVD_NOTIFY_REMOVE(buffer);
	PxMutex::ScopedLock lock(mTrackingMutex);
	mParticleBufferTracking.erase(buffer);
}

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

///////////////////////////////////////////////////////////////////////////////

PxDeformableSurfaceMaterial* NpFactory::createDeformableSurfaceMaterial(PxReal youngs, PxReal poissons, PxReal dynamicFriction, PxReal thickness, 
	PxReal bendingStiffness, PxReal damping, PxReal bendingDamping)
{
	PX_CHECK_AND_RETURN_NULL(youngs >= 0.0f, "createDeformableSurfaceMaterial: youngs must be >= 0.");
	PX_CHECK_AND_RETURN_NULL(poissons >= 0.0f && poissons < 0.5f, "createDeformableSurfaceMaterial: poissons must be in range[0.f, 0.5f).");
	PX_CHECK_AND_RETURN_NULL(dynamicFriction >= 0.0f, "createDeformableSurfaceMaterial: dynamicFriction must be >= 0.");
	PX_CHECK_AND_RETURN_NULL(thickness >= 0.0f, "createDeformableSurfaceMaterial: thickness must be > 0.");
	PX_CHECK_AND_RETURN_NULL(bendingStiffness >= 0.0f, "createDeformableSurfaceMaterial: bendingStiffness must be >= 0.");
	PX_CHECK_AND_RETURN_NULL(damping >= 0.0f, "createDeformableSurfaceMaterial: damping must be >= 0.");
	PX_CHECK_AND_RETURN_NULL(bendingDamping >= 0.0f, "createDeformableSurfaceMaterial: bendingDamping must be >= 0.");

	PxsDeformableSurfaceMaterialData materialData;
	materialData.youngs = youngs;
	materialData.poissons = poissons;
	materialData.dynamicFriction = dynamicFriction;
	materialData.thickness = thickness;
	materialData.bendingStiffness = bendingStiffness;
	materialData.damping = damping;
	materialData.bendingDamping = bendingDamping;

	NpDeformableSurfaceMaterial* npMaterial = NULL;
	{
		PxMutex::ScopedLock lock(mDeformableSurfaceMaterialPoolLock);
		npMaterial = mDeformableSurfaceMaterialPool.construct(materialData);
	}
	return npMaterial;
}
#endif

#if PX_SUPPORT_GPU_PHYSX
void NpFactory::releaseDeformableSurfaceMaterialToPool(PxDeformableSurfaceMaterial& material_)
{
    NpDeformableSurfaceMaterial& material = static_cast<NpDeformableSurfaceMaterial&>(material_);
    PX_ASSERT(material.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
    PxMutex::ScopedLock lock(mDeformableSurfaceMaterialPoolLock);
    mDeformableSurfaceMaterialPool.destroy(&material);
}
#endif
///////////////////////////////////////////////////////////////////////////////

#if PX_SUPPORT_GPU_PHYSX
PxDeformableVolumeMaterial* NpFactory::createDeformableVolumeMaterial(PxReal youngs, PxReal poissons, PxReal dynamicFriction)
{
#if PX_SUPPORT_GPU_PHYSX
	PX_CHECK_AND_RETURN_NULL(youngs >= 0.0f, "createDeformableVolumeMaterial: youngs must be >= 0.");
	PX_CHECK_AND_RETURN_NULL(poissons >= 0.0f && poissons < 0.5f, "createDeformableVolumeMaterial: poissons must be in range[0.f, 0.5f).");
	PX_CHECK_AND_RETURN_NULL(dynamicFriction >= 0.0f, "createDeformableVolumeMaterial: dynamicFriction must be >= 0.");

	PxsDeformableVolumeMaterialData materialData;
	materialData.youngs = youngs;
	materialData.poissons = poissons;
	materialData.dynamicFriction = dynamicFriction;
	materialData.damping = 0.f;
	materialData.dampingScale = toUniformU16(1.f);
	materialData.materialModel = PxDeformableVolumeMaterialModel::eCO_ROTATIONAL;
	materialData.deformThreshold = PX_MAX_F32;
	materialData.deformLowLimitRatio = 1.f;
	materialData.deformHighLimitRatio = 1.f;

	NpDeformableVolumeMaterial* npMaterial;
	{
		PxMutex::ScopedLock lock(mDeformableVolumeMaterialPoolLock);
		npMaterial = mDeformableVolumeMaterialPool.construct(materialData);
	}
	return npMaterial;

#else
	PX_UNUSED(youngs);
	PX_UNUSED(poissons);
	PX_UNUSED(dynamicFriction);
	PX_UNUSED(thickness);
	PX_UNUSED(bendingStiffness);
	PX_UNUSED(damping);
	PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "PxDeformableVolumeMaterial is not supported on this platform.");
	return NULL;
#endif
}

void NpFactory::releaseDeformableVolumeMaterialToPool(PxDeformableVolumeMaterial& material_)
{
#if PX_SUPPORT_GPU_PHYSX
	NpDeformableVolumeMaterial& material = static_cast<NpDeformableVolumeMaterial&>(material_);
	PX_ASSERT(material.getBaseFlags() & PxBaseFlag::eOWNS_MEMORY);
	PxMutex::ScopedLock lock(mDeformableVolumePoolLock);
	mDeformableVolumeMaterialPool.destroy(&material);
#else
	PX_UNUSED(material_);
#endif
}

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

#if PX_SUPPORT_GPU_PHYSX
NpShape* NpFactory::createShape(const PxGeometry& geometry,
    PxShapeFlags shapeFlags,
    PxDeformableSurfaceMaterial* const* materials,
    PxU16 materialCount,
    bool isExclusive)
{
    return createShapeInternal<PxDeformableSurfaceMaterial, NpDeformableSurfaceMaterial>(geometry, shapeFlags, materials, materialCount, isExclusive, PxShapeCoreFlag::eDEFORMABLE_SURFACE_SHAPE);
}

NpShape* NpFactory::createShape(const PxGeometry& geometry,
    PxShapeFlags shapeFlags,
    PxDeformableVolumeMaterial* const* materials,
    PxU16 materialCount,
    bool isExclusive)
{
    return createShapeInternal<PxDeformableVolumeMaterial, NpDeformableVolumeMaterial>(geometry, shapeFlags, materials, materialCount, isExclusive, PxShapeCoreFlag::eDEFORMABLE_VOLUME_SHAPE);
}

#else

NpShape* NpFactory::createShape(const PxGeometry& geometry,
    PxShapeFlags shapeFlags,
    PxDeformableSurfaceMaterial* const* materials,
    PxU16 materialCount,
    bool isExclusive)
{
    PX_UNUSED(geometry);
    PX_UNUSED(shapeFlags);
    PX_UNUSED(materials);
    PX_UNUSED(materialCount);
    PX_UNUSED(isExclusive);
    return NULL;
}

NpShape* NpFactory::createShape(const PxGeometry& geometry,
    PxShapeFlags shapeFlags,
    PxDeformableVolumeMaterial* const* materials,
    PxU16 materialCount,
    bool isExclusive)
{
    PX_UNUSED(geometry);
    PX_UNUSED(shapeFlags);
    PX_UNUSED(materials);
    PX_UNUSED(materialCount);
    PX_UNUSED(isExclusive);
    return NULL;
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
				{
					addRigidStatic(static_cast<NpRigidStatic*>(actor), false);
				}
				else if(serialType1==PxConcreteType::eRIGID_DYNAMIC)
				{
					addRigidDynamic(static_cast<NpRigidDynamic*>(actor), false);
				}
				else if(serialType1==PxConcreteType::eARTICULATION_LINK)
				{
					// This is not needed as the articulation links get handled separately
				}
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
			OMNI_PVD_NOTIFY_ADD(static_cast<NpArticulationLink*>(s));
		}
		else if(serialType==PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE)
		{
			OMNI_PVD_NOTIFY_ADD(static_cast<PxArticulationJointReducedCoordinate*>(s));
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

static PX_FORCE_INLINE void releaseToPool(PxArticulationMimicJoint* np)
{
	PX_ASSERT(np->getConcreteType() == PxConcreteType::eARTICULATION_MIMIC_JOINT);
	NpFactory::getInstance().releaseArticulationMimicJointToPool(*static_cast<NpArticulationMimicJoint*>(np));
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
static PX_FORCE_INLINE void releaseToPool(NpDeformableSurface* np)
{
	NpFactory::getInstance().releaseDeformableSurfaceToPool(*np);
}

static PX_FORCE_INLINE void releaseToPool(NpDeformableVolume* np)
{
	NpFactory::getInstance().releaseDeformableVolumeToPool(*np);
}

static PX_FORCE_INLINE void releaseToPool(NpDeformableAttachment* np)
{
	NpFactory::getInstance().releaseAttachmentToPool(*np);
}

static PX_FORCE_INLINE void releaseToPool(NpDeformableElementFilter* np)
{
	NpFactory::getInstance().releaseElementFilterToPool(*np);
}

static PX_FORCE_INLINE void releaseToPool(NpPBDParticleSystem* np)
{
	NpFactory::getInstance().releasePBDParticleSystemToPool(*np);
}

static PX_FORCE_INLINE void releaseToPool(NpParticleBuffer* np)
{
	NpFactory::getInstance().releaseParticleBufferToPool(*np);
}

static PX_FORCE_INLINE void releaseToPool(NpParticleAndDiffuseBuffer* np)
{
	NpFactory::getInstance().releaseParticleAndDiffuseBufferToPool(*np);
}

static PX_FORCE_INLINE void releaseToPool(NpParticleClothBuffer* np)
{
	NpFactory::getInstance().releaseParticleClothBufferToPool(*np);
}

static PX_FORCE_INLINE void releaseToPool(NpParticleRigidBuffer* np)
{
	NpFactory::getInstance().releaseParticleRigidBufferToPool(*np);
}
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
void physx::NpDestroyArticulationMimicJoint(PxArticulationMimicJoint* np)			{ NpDestroy(np);	}
void physx::NpDestroyArticulation(PxArticulationReducedCoordinate* np)				{ NpDestroy(np);	}
#if PX_SUPPORT_GPU_PHYSX
void physx::NpDestroyDeformableSurface(NpDeformableSurface* np)						{ NpDestroy(np);	}
void physx::NpDestroyDeformableVolume(NpDeformableVolume* np)						{ NpDestroy(np);	}
void physx::NpDestroyAttachment(NpDeformableAttachment* np)							{ NpDestroy(np);	}
void physx::NpDestroyElementFilter(NpDeformableElementFilter* np)					{ NpDestroy(np);	}
void physx::NpDestroyParticleSystem(NpPBDParticleSystem* np)						{ NpDestroy(np);	}
void physx::NpDestroyParticleBuffer(NpParticleBuffer* np)							{ NpDestroy(np);	}
void physx::NpDestroyParticleBuffer(NpParticleAndDiffuseBuffer* np)					{ NpDestroy(np); }
void physx::NpDestroyParticleBuffer(NpParticleClothBuffer* np)						{ NpDestroy(np); }
void physx::NpDestroyParticleBuffer(NpParticleRigidBuffer* np)						{ NpDestroy(np); }
#endif
