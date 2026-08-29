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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef NP_FACTORY_H
#define NP_FACTORY_H

#include "foundation/PxPool.h"
#include "foundation/PxMutex.h"
#include "foundation/PxHashSet.h"

#include "GuMeshFactory.h"
#include "PxPhysXConfig.h"
#include "PxShape.h"
#include "PxAggregate.h"
#include "PxvGeometry.h"
#include "solver/PxSolverDefs.h"

#if PX_SUPPORT_GPU_PHYSX
	#include "NpDeformableSurface.h"  // to be deleted
#endif

namespace physx
{
class PxCudaContextManager;

class PxActor;

class PxRigidActor;

class PxRigidStatic;
class NpRigidStatic;

class PxRigidDynamic;
class NpRigidDynamic;

class NpConnectorArray;

struct PxConstraintShaderTable;
class PxConstraintConnector;
class PxConstraint;
class NpConstraint;

class PxArticulationReducedCoordinate;
class NpArticulationReducedCoordinate;
class PxArticulationLink;
class NpArticulationLink;
class NpArticulationJointReducedCoordinate;
class PxArticulationMimicJoint;
class NpArticulationMimicJoint;

class PxDeformableSurface;
class PxDeformableVolume;
class PxDeformableAttachment;
class PxDeformableElementFilter;

#if PX_SUPPORT_GPU_PHYSX
class NpDeformableSurface;
class NpDeformableVolume;
class NpPBDParticleSystem;
class NpParticleBuffer;
class NpParticleAndDiffuseBuffer;
class NpDeformableAttachment;
class NpDeformableElementFilter;

class NpDeformableSurfaceMaterial;
class NpDeformableVolumeMaterial;
class NpPBDMaterial;
#endif

class PxMaterial;
class NpMaterial;

class PxDeformableSurfaceMaterial;
class PxDeformableVolumeMaterial;
class PxPBDMaterial;

class PxGeometry;

class NpShape;

class NpAggregate;

class NpPtrTableStorageManager;

namespace Cm
{
   class Collection;
}

class NpFactoryListener : public Gu::MeshFactoryListener
{
protected:
	virtual ~NpFactoryListener(){}
};

class NpFactory : public Gu::MeshFactory
{
	PX_NOCOPY(NpFactory)
public:
														NpFactory();
private:
														~NpFactory();

				template <typename PxMaterialType, typename NpMaterialType>
				NpShape*								createShapeInternal(const PxGeometry& geometry, PxShapeFlags shapeFlags, PxMaterialType*const* materials, PxU16 materialCount, bool isExclusive, PxShapeCoreFlag::Enum flag);
public:
	static		void									createInstance();
	static		void									destroyInstance();

				void									release();

				void									addCollection(const Cm::Collection& collection);

	PX_INLINE static NpFactory&							getInstance() { return *mInstance; }

				// Rigid dynamic
				PxRigidDynamic*							createRigidDynamic(const PxTransform& pose);
				void									addRigidDynamic(PxRigidDynamic*, bool lock=true);
				void									releaseRigidDynamicToPool(NpRigidDynamic&);
				// The rigid dynamics the factory tracks, regardless of scene membership. Used by the
				// full-state OmniPVD snapshot.
				PxU32									getNbRigidDynamics() const;
				PxU32									getRigidDynamics(PxRigidDynamic** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

				// Rigid static
				PxRigidStatic*							createRigidStatic(const PxTransform& pose);
				void									addRigidStatic(PxRigidStatic*, bool lock=true);
				void									releaseRigidStaticToPool(NpRigidStatic&);
				// The rigid statics the factory tracks, regardless of scene membership. Used by the
				// full-state OmniPVD snapshot.
				PxU32									getNbRigidStatics() const;
				PxU32									getRigidStatics(PxRigidStatic** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

				// Deformable surfaces, deformable volumes and PBD particle systems the factory tracks.
				// Kept in their own sets (GPU-only object types) so the rigid-actor set above stays
				// homogeneous. Used by the full-state OmniPVD snapshot.
				PxU32 getNbDeformableSurfaces() const;
				PxU32 getDeformableSurfaces(PxActor** userBuffer, PxU32 bufferSize, PxU32 startIndex) const;
				PxU32 getNbDeformableVolumes() const;
				PxU32 getDeformableVolumes(PxActor** userBuffer, PxU32 bufferSize, PxU32 startIndex) const;
				PxU32 getNbPBDParticleSystems() const;
				PxU32 getPBDParticleSystems(PxActor** userBuffer, PxU32 bufferSize, PxU32 startIndex) const;

				// Shapes
				NpShape*								createShape(const PxGeometry& geometry, PxShapeFlags shapeFlags, PxMaterial*const* materials, PxU16 materialCount, bool isExclusive);
				NpShape*								createShape(const PxGeometry& geometry, PxShapeFlags shapeFlags, PxDeformableSurfaceMaterial*const* materials, PxU16 materialCount, bool isExclusive);
				NpShape*								createShape(const PxGeometry& geometry, PxShapeFlags shapeFlags, PxDeformableVolumeMaterial*const* materials, PxU16 materialCount, bool isExclusive);
				void									addShape(PxShape*, bool lock=true);
				void									releaseShapeToPool(NpShape&);
				PxU32									getNbShapes() const;
				PxU32									getShapes(PxShape** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

				// Constraints
				PxConstraint*							createConstraint(PxRigidActor* actor0, PxRigidActor* actor1, PxConstraintConnector& connector, const PxConstraintShaderTable& shaders, PxU32 dataSize);
				void									addConstraint(PxConstraint*, bool lock=true);
				void									releaseConstraintToPool(NpConstraint&);
				PxU32									getNbConstraints() const;
				PxU32									getConstraints(PxConstraint** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

				// Articulations
				void									addArticulation(PxArticulationReducedCoordinate*, bool lock=true);
				void									releaseArticulationToPool(PxArticulationReducedCoordinate& articulation);
				PxArticulationReducedCoordinate*		createArticulationRC();
				PxU32									getNbArticulations() const;
				PxU32 getArticulations(PxArticulationReducedCoordinate** userBuffer, PxU32 bufferSize, PxU32 startIndex) const;

				// Articulation links
				NpArticulationLink*						createNpArticulationLink(NpArticulationReducedCoordinate& root, NpArticulationLink* parent, const PxTransform& pose);
				void									releaseArticulationLinkToPool(NpArticulationLink& articulation);
				PxArticulationLink*						createArticulationLink(NpArticulationReducedCoordinate& root, NpArticulationLink* parent, const PxTransform& pose);

				NpArticulationJointReducedCoordinate*	createNpArticulationJointRC(NpArticulationLink& parent, const PxTransform& parentFrame, NpArticulationLink& child, const PxTransform& childFrame);
				void									releaseArticulationJointRCToPool(NpArticulationJointReducedCoordinate& articulationJoint);

				NpArticulationMimicJoint*				createNpArticulationMimicJoint(
															const PxArticulationJointReducedCoordinate& jointA, const PxArticulationAxis::Enum axisA, 
															const PxArticulationJointReducedCoordinate& jointB, const PxArticulationAxis::Enum axisB,		
															const PxReal gearRatio, const PxReal offset,
															const PxReal naturalFrequency, const PxReal dampingRatio);
				void									releaseArticulationMimicJointToPool(NpArticulationMimicJoint& articulationMimicJoint);

#if PX_SUPPORT_GPU_PHYSX
				// Deformable surfaces
				PxDeformableSurface*					createDeformableSurface(PxCudaContextManager& cudaContextManager);
				void									releaseDeformableSurfaceToPool(PxDeformableSurface& femCloth);

				// Deformable volumes
				PxDeformableVolume*						createDeformableVolume(PxCudaContextManager& cudaContextManager);
				void									releaseDeformableVolumeToPool(PxDeformableVolume& softBody);

				// Attachments
				PxDeformableAttachment*					createDeformableAttachment(const PxDeformableAttachmentData& data);
				void									addAttachment(PxDeformableAttachment*, bool lock = true);
				void									releaseAttachmentToPool(PxDeformableAttachment& attachment);
				void									onAttachmentRelease(PxDeformableAttachment*);

				// Attachments
				PxDeformableElementFilter*				createDeformableElementFilter(const PxDeformableElementFilterData& data);
				void									addElementFilter(PxDeformableElementFilter*, bool lock = true);
				void									releaseElementFilterToPool(PxDeformableElementFilter& elementFilter);
				void									onElementFilterRelease(PxDeformableElementFilter*);

				//Particle systems
				PxPBDParticleSystem*					createPBDParticleSystem(PxU32 maxNeighborhood, PxReal neighborhoodScale, PxCudaContextManager& cudaContextManager);
				void									releasePBDParticleSystemToPool(PxPBDParticleSystem& particleSystem);

				//Particle buffers
				PxParticleBuffer*						createParticleBuffer(PxU32 maxParticles, PxCudaContextManager& cudaContextManager);
				PxParticleAndDiffuseBuffer*				createParticleAndDiffuseBuffer(PxU32 maxParticles, PxU32 maxDiffuseParticles, PxCudaContextManager& cudaContextManager);
				void									addParticleBuffer(PxParticleBuffer* buffer, bool lock = true);
				void									releaseParticleBufferToPool(PxParticleBuffer& particleBuffer);
				void									releaseParticleAndDiffuseBufferToPool(PxParticleAndDiffuseBuffer& particleBuffer);

				// All particle buffers the factory tracks, whether or not they are attached to a system.
				PxU32									getNbParticleBuffers() const;
				PxU32									getParticleBuffers(PxParticleBuffer** userBuffer, PxU32 bufferSize, PxU32 startIndex) const;
#endif
				// Aggregates
				PxAggregate*							createAggregate(PxU32 maxActors, PxU32 maxShapes, PxAggregateFilterHint filterHint);
				void									addAggregate(PxAggregate*, bool lock=true);
				void									releaseAggregateToPool(NpAggregate&);
				PxU32									getNbAggregates() const;
				PxU32 getAggregates(PxAggregate** userBuffer, PxU32 bufferSize, PxU32 startIndex) const;

				// Materials
				PxMaterial*								createMaterial(PxReal staticFriction, PxReal dynamicFriction, PxReal restitution);
				void									releaseMaterialToPool(NpMaterial& material);

#if PX_SUPPORT_GPU_PHYSX

				PxDeformableSurfaceMaterial*			createDeformableSurfaceMaterial(PxReal youngs, PxReal poissons, PxReal dynamicFriction, PxReal thickness, PxReal bendingStiffness, PxReal elasticityDamping, PxReal bendingDamping);
				void									releaseDeformableSurfaceMaterialToPool(PxDeformableSurfaceMaterial& material);

				PxDeformableVolumeMaterial*				createDeformableVolumeMaterial(PxReal youngs, PxReal poissons, PxReal dynamicFriction, PxReal elasticityDamping);
				void									releaseDeformableVolumeMaterialToPool(PxDeformableVolumeMaterial& material);

				PxPBDMaterial*							createPBDMaterial(PxReal friction, PxReal damping, PxReal adhesion, PxReal viscosity, PxReal vorticityConfinement, PxReal surfaceTension, PxReal cohesion, PxReal lift, PxReal drag, PxReal cflCoefficient, PxReal gravityScale);
				void									releasePBDMaterialToPool(PxPBDMaterial& material);
#endif
				// It's easiest to track these uninvasively, so it's OK to use the Px pointers
				void									onActorRelease(PxActor*);
				void									onConstraintRelease(PxConstraint*);
				void									onAggregateRelease(PxAggregate*);
				void									onArticulationRelease(PxArticulationReducedCoordinate*);
				void									onShapeRelease(PxShape*);

#if PX_SUPPORT_GPU_PHYSX
				void									onParticleBufferRelease(PxParticleBuffer*);
#endif
				NpConnectorArray*						acquireConnectorArray();
				void									releaseConnectorArray(NpConnectorArray*);
				
	PX_FORCE_INLINE	NpPtrTableStorageManager&			getPtrTableStorageManager()	{ return *mPtrTableStorageManager; }

#if PX_SUPPORT_PVD
				void									setNpFactoryListener( NpFactoryListener& );
#endif

private:
				PxPool<NpConnectorArray>				mConnectorArrayPool;
				PxMutex									mConnectorArrayPoolLock;

				NpPtrTableStorageManager*				mPtrTableStorageManager;

				// The tracking sets a full-state snapshot enumerates are coalesced so their getters can
				// hand getEntries() straight to getArrayOfPointers() (see getShapes). mAttachmentTracking
				// and mElementFilterTracking are not snapshot-enumerated, so they stay plain hash sets.
				PxCoalescedHashSet<PxAggregate*>						mAggregateTracking;
				PxCoalescedHashSet<PxArticulationReducedCoordinate*>	mArticulationTracking;
				PxCoalescedHashSet<PxConstraint*>						mConstraintTracking;
				PxCoalescedHashSet<PxRigidStatic*>						mRigidStaticTracking;
				PxCoalescedHashSet<PxRigidDynamic*>						mRigidDynamicTracking;
				PxCoalescedHashSet<PxShape*>							mShapeTracking;
#if PX_SUPPORT_GPU_PHYSX
				PxHashSet<PxDeformableAttachment*>		mAttachmentTracking;
				PxHashSet<PxDeformableElementFilter*>	mElementFilterTracking;
				PxCoalescedHashSet<PxParticleBuffer*>	mParticleBufferTracking;
				PxCoalescedHashSet<PxActor*>			mDeformableSurfaceTracking;
				PxCoalescedHashSet<PxActor*>			mDeformableVolumeTracking;
				PxCoalescedHashSet<PxActor*>			mPBDParticleSystemTracking;
#endif
				PxPool2<NpRigidDynamic, 4096>			mRigidDynamicPool;
				PxMutex									mRigidDynamicPoolLock;

				PxPool2<NpRigidStatic, 4096>			mRigidStaticPool;
				PxMutex									mRigidStaticPoolLock;

				PxPool2<NpShape, 4096>					mShapePool;
				PxMutex									mShapePoolLock;

				PxPool2<NpAggregate, 4096>				mAggregatePool;
				PxMutex									mAggregatePoolLock;

				PxPool2<NpConstraint, 4096>				mConstraintPool;
				PxMutex									mConstraintPoolLock;

				PxPool2<NpMaterial, 4096>				mMaterialPool;
				PxMutex									mMaterialPoolLock;

				PxPool2<NpArticulationReducedCoordinate, 4096>	mArticulationRCPool;
				PxMutex											mArticulationRCPoolLock;

				PxPool2<NpArticulationLink, 4096>		mArticulationLinkPool;
				PxMutex									mArticulationLinkPoolLock;

				PxPool2<NpArticulationJointReducedCoordinate, 4096> mArticulationRCJointPool;
				PxMutex												mArticulationJointRCPoolLock;

				PxPool2<NpArticulationMimicJoint, 4096>	mArticulationMimicJointPool;
				PxMutex									mArticulationMimicJointPoolLock;

#if PX_SUPPORT_GPU_PHYSX
				PxPool2<NpDeformableSurface, 1024>		mDeformableSurfacePool;
				PxMutex									mDeformableSurfacePoolLock;

				PxPool2<NpDeformableVolume, 1024>		mDeformableVolumePool;
				PxMutex									mDeformableVolumePoolLock;

				PxPool2<NpDeformableAttachment, 1024>	mAttachmentPool;
				PxMutex									mAttachmentPoolLock;

				PxPool2<NpDeformableElementFilter, 1024> mElementFilterPool;
				PxMutex									mElementFilterPoolLock;

				PxPool2<NpPBDParticleSystem, 1024>		mPBDParticleSystemPool;
				PxMutex									mPBDParticleSystemPoolLock;

				PxPool2<NpParticleBuffer, 1024>			mParticleBufferPool;
				PxMutex									mParticleBufferPoolLock;

				PxPool2<NpParticleAndDiffuseBuffer, 1024> mParticleAndDiffuseBufferPool;
				PxMutex									mParticleAndDiffuseBufferPoolLock;

				PxPool2<NpDeformableSurfaceMaterial, 1024>	mDeformableSurfaceMaterialPool;
				PxMutex										mDeformableSurfaceMaterialPoolLock;

				PxPool2<NpDeformableVolumeMaterial, 1024>	mDeformableVolumeMaterialPool;
				PxMutex										mDeformableVolumeMaterialPoolLock;

				PxPool2<NpPBDMaterial, 1024>			mPBDMaterialPool;
				PxMutex									mPBDMaterialPoolLock;
#endif

	static		NpFactory*								mInstance;

#if PX_SUPPORT_PVD
				NpFactoryListener*						mNpFactoryListener;
#endif
};

	void	NpDestroyRigidActor(NpRigidStatic* np);
	void	NpDestroyRigidDynamic(NpRigidDynamic* np);
	void	NpDestroyArticulationLink(NpArticulationLink* np);
	void	NpDestroyArticulationJoint(PxArticulationJointReducedCoordinate* np);
	void	NpDestroyArticulationMimicJoint(PxArticulationMimicJoint* np);
	void	NpDestroyArticulation(PxArticulationReducedCoordinate* artic);
	void	NpDestroyAggregate(NpAggregate* np);
	void	NpDestroyShape(NpShape* np);
	void	NpDestroyConstraint(NpConstraint* np);

#if PX_SUPPORT_GPU_PHYSX
	void	NpDestroyDeformableSurface(NpDeformableSurface* np);
	void	NpDestroyDeformableVolume(NpDeformableVolume* np);
	void	NpDestroyAttachment(NpDeformableAttachment* np);
	void	NpDestroyElementFilter(NpDeformableElementFilter* np);
	void	NpDestroyParticleSystem(NpPBDParticleSystem* particleSystem);
	void	NpDestroyParticleBuffer(NpParticleBuffer* particleBuffer);
	void	NpDestroyParticleBuffer(NpParticleAndDiffuseBuffer* particleBuffer);
#endif
}

#endif // NP_FACTORY_H
