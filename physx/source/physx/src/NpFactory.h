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

#include "NpFEMCloth.h"  // to be deleted

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

class PxSoftBody;
class PxFEMCloth;
class PxParticleSystem;
class PxHairSystem;

#if PX_SUPPORT_GPU_PHYSX
class NpSoftBody;
class NpFEMCloth;
class NpPBDParticleSystem;
class NpFLIPParticleSystem;
class NpMPMParticleSystem;
class NpHairSystem;

class NpFEMSoftBodyMaterial;
class NpFEMClothMaterial;
class NpPBDMaterial;
class NpFLIPMaterial;
class NpMPMMaterial;
#endif

class PxMaterial;
class NpMaterial;

class PxFEMSoftBodyMaterial;
class PxFEMClothMaterial;
class PxPBDMaterial;
class PxFLIPMaterial;
class PxMPMMaterial;

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
	static		void									onParticleBufferRelease(PxParticleBuffer* buffer);

				void									release();

				void									addCollection(const Cm::Collection& collection);

	PX_INLINE static NpFactory&							getInstance() { return *mInstance; }

				// Rigid dynamic
				PxRigidDynamic*							createRigidDynamic(const PxTransform& pose);
				void									addRigidDynamic(PxRigidDynamic*, bool lock=true);
				void									releaseRigidDynamicToPool(NpRigidDynamic&);
// PT: TODO: add missing functions
//				PxU32									getNbRigidDynamics() const;
//				PxU32									getRigidDynamics(PxRigidDynamic** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

				// Rigid static
				PxRigidStatic*							createRigidStatic(const PxTransform& pose);
				void									addRigidStatic(PxRigidStatic*, bool lock=true);
				void									releaseRigidStaticToPool(NpRigidStatic&);
// PT: TODO: add missing functions
//				PxU32									getNbRigidStatics() const;
//				PxU32									getRigidStatics(PxRigidStatic** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

				// Shapes
				NpShape*								createShape(const PxGeometry& geometry, PxShapeFlags shapeFlags, PxMaterial*const* materials, PxU16 materialCount, bool isExclusive);
				NpShape*								createShape(const PxGeometry& geometry, PxShapeFlags shapeFlags, PxFEMSoftBodyMaterial*const* materials, PxU16 materialCount, bool isExclusive);
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION && PX_SUPPORT_GPU_PHYSX
				NpShape*								createShape(const PxGeometry& geometry, PxShapeFlags shapeFlags, PxFEMClothMaterial*const* materials, PxU16 materialCount, bool isExclusive);
#endif
				void									addShape(PxShape*, bool lock=true);
				void									releaseShapeToPool(NpShape&);
				PxU32									getNbShapes() const;
				PxU32									getShapes(PxShape** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

				// Constraints
				PxConstraint*							createConstraint(PxRigidActor* actor0, PxRigidActor* actor1, PxConstraintConnector& connector, const PxConstraintShaderTable& shaders, PxU32 dataSize);
				void									addConstraint(PxConstraint*, bool lock=true);
				void									releaseConstraintToPool(NpConstraint&);
// PT: TODO: add missing functions
//				PxU32									getNbConstraints() const;
//				PxU32									getConstraints(PxConstraint** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

				// Articulations
				void									addArticulation(PxArticulationReducedCoordinate*, bool lock=true);
				void									releaseArticulationToPool(PxArticulationReducedCoordinate& articulation);
				PxArticulationReducedCoordinate*		createArticulationRC();
				NpArticulationReducedCoordinate*		createNpArticulationRC();

				// Articulation links
				NpArticulationLink*						createNpArticulationLink(NpArticulationReducedCoordinate& root, NpArticulationLink* parent, const PxTransform& pose);
				void									releaseArticulationLinkToPool(NpArticulationLink& articulation);
				PxArticulationLink*						createArticulationLink(NpArticulationReducedCoordinate& root, NpArticulationLink* parent, const PxTransform& pose);

				NpArticulationJointReducedCoordinate*	createNpArticulationJointRC(NpArticulationLink& parent, const PxTransform& parentFrame, NpArticulationLink& child, const PxTransform& childFrame);
				void									releaseArticulationJointRCToPool(NpArticulationJointReducedCoordinate& articulationJoint);

#if PX_SUPPORT_GPU_PHYSX
				//Soft bodys
				PxSoftBody*								createSoftBody(PxCudaContextManager& cudaContextManager);
				void									releaseSoftBodyToPool(PxSoftBody& softBody);
				
	#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
				// FEMCloth
				PxFEMCloth*								createFEMCloth(PxCudaContextManager& cudaContextManager);
				void									releaseFEMClothToPool(PxFEMCloth& femCloth);
	#endif
				//Particle systems
				PxPBDParticleSystem*					createPBDParticleSystem(PxU32 maxNeighborhood, PxCudaContextManager& cudaContextManager);
				void									releasePBDParticleSystemToPool(PxPBDParticleSystem& particleSystem);

	#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
				//Particle systems
				PxFLIPParticleSystem*					createFLIPParticleSystem(PxCudaContextManager& cudaContextManager);
				void									releaseFLIPParticleSystemToPool(PxFLIPParticleSystem& particleSystem);

				//Particle systems
				PxMPMParticleSystem*					createMPMParticleSystem(PxCudaContextManager& cudaContextManager);
				void									releaseMPMParticleSystemToPool(PxMPMParticleSystem& particleSystem);
	#endif
				//Particle buffers
				PxParticleBuffer*						createParticleBuffer(PxU32 maxParticles, PxU32 maxVolumes, PxCudaContextManager* cudaContextManager);

				//Diffuse Particle buffers
				PxParticleAndDiffuseBuffer*				createParticleAndDiffuseBuffer(PxU32 maxParticles, PxU32 maxVolumes, PxU32 maxDiffuseParticles, PxCudaContextManager* cudaContextManager);

				//Particle cloth buffers
				PxParticleClothBuffer*					createParticleClothBuffer(PxU32 maxParticles, PxU32 maxNumVolumes, PxU32 maxNumCloths, PxU32 maxNumTriangles, PxU32 maxNumSprings, PxCudaContextManager* cudaContextManager);

				//Particle rigid buffers
				PxParticleRigidBuffer*					createParticleRigidBuffer(PxU32 maxParticles, PxU32 maxNumVolumes, PxU32 maxNumRigids, PxCudaContextManager* cudaContextManager);

	#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
				// HairSystem
				PxHairSystem*							createHairSystem(PxCudaContextManager& cudaContextManager);
				void									releaseHairSystemToPool(PxHairSystem& hairSystem);
	#endif
#endif
				// Aggregates
				PxAggregate*							createAggregate(PxU32 maxActors, PxU32 maxShapes, PxAggregateFilterHint filterHint);
				void									addAggregate(PxAggregate*, bool lock=true);
				void									releaseAggregateToPool(NpAggregate&);
// PT: TODO: add missing functions
//				PxU32									getNbAggregates() const;
//				PxU32									getAggregates(PxAggregate** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

				// Materials
				PxMaterial*								createMaterial(PxReal staticFriction, PxReal dynamicFriction, PxReal restitution);
				void									releaseMaterialToPool(NpMaterial& material);

#if PX_SUPPORT_GPU_PHYSX
				PxFEMSoftBodyMaterial*					createFEMSoftBodyMaterial(PxReal youngs, PxReal poissons, PxReal dynamicFriction);
				void									releaseFEMMaterialToPool(PxFEMSoftBodyMaterial& material);

				PxFEMClothMaterial*						createFEMClothMaterial(PxReal youngs, PxReal poissons, PxReal dynamicFriction);
				void									releaseFEMClothMaterialToPool(PxFEMClothMaterial& material);

				PxPBDMaterial*							createPBDMaterial(PxReal friction, PxReal damping, PxReal adhesion, PxReal viscosity, PxReal vorticityConfinement, PxReal surfaceTension, PxReal cohesion, PxReal lift, PxReal drag, PxReal cflCoefficient, PxReal gravityScale);
				void									releasePBDMaterialToPool(PxPBDMaterial& material);
				
	#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
				PxFLIPMaterial*							createFLIPMaterial(PxReal friction, PxReal damping, PxReal adhesion, PxReal viscosity, PxReal gravityScale);
				void									releaseFLIPMaterialToPool(PxFLIPMaterial& material);
	#endif
				PxMPMMaterial*							createMPMMaterial(PxReal friction, PxReal damping, PxReal adhesion, bool isPlastic, PxReal youngsModulus, PxReal poissons, PxReal hardening, PxReal criticalCompression, PxReal criticalStretch, PxReal tensileDamageSensitivity, PxReal compressiveDamageSensitivity, PxReal attractiveForceResidual, PxReal gravityScale);
				void									releaseMPMMaterialToPool(PxMPMMaterial& material);
#endif
				// It's easiest to track these uninvasively, so it's OK to use the Px pointers
				void									onActorRelease(PxActor*);
				void									onConstraintRelease(PxConstraint*);
				void									onAggregateRelease(PxAggregate*);
				void									onArticulationRelease(PxArticulationReducedCoordinate*);
				void									onShapeRelease(PxShape*);

#if PX_SUPPORT_GPU_PHYSX
				void									addParticleBuffer(PxParticleBuffer* buffer, bool lock = true);
				void									onParticleBufferReleaseInternal(PxParticleBuffer* buffer);
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

				PxHashSet<PxAggregate*>							mAggregateTracking;
				PxHashSet<PxArticulationReducedCoordinate*>		mArticulationTracking;
				PxHashSet<PxConstraint*>						mConstraintTracking;
				PxHashSet<PxActor*>								mActorTracking;				
				PxCoalescedHashSet<PxShape*>					mShapeTracking;
#if PX_SUPPORT_GPU_PHYSX
				PxHashSet<PxParticleBuffer*>				mParticleBufferTracking;
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

#if PX_SUPPORT_GPU_PHYSX
				PxPool2<NpSoftBody, 4096>				mSoftBodyPool;
				PxMutex									mSoftBodyPoolLock;

	#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
				PxPool2<NpFEMCloth, 4096>				mFEMClothPool;
				PxMutex									mFEMClothPoolLock;
	#endif
				PxPool2<NpPBDParticleSystem, 4096>		mPBDParticleSystemPool;
				PxMutex									mPBDParticleSystemPoolLock;
	#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
				PxPool2<NpFLIPParticleSystem, 4096>		mFLIPParticleSystemPool;
				PxMutex									mFLIPParticleSystemPoolLock;

				PxPool2<NpMPMParticleSystem, 4096>		mMPMParticleSystemPool;
				PxMutex									mMPMParticleSystemPoolLock;
	#endif
				PxPool2<NpFEMSoftBodyMaterial, 4096>	mFEMMaterialPool;
				PxMutex									mFEMMaterialPoolLock;

	#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
				PxPool2<NpFEMClothMaterial, 4096>		mFEMClothMaterialPool;
				PxMutex									mFEMClothMaterialPoolLock;
	#endif
				PxPool2<NpPBDMaterial, 4096>			mPBDMaterialPool;
				PxMutex									mPBDMaterialPoolLock;
	#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
				PxPool2<NpFLIPMaterial, 4096>			mFLIPMaterialPool;
				PxMutex									mFLIPMaterialPoolLock;
				
				PxPool2<NpMPMMaterial, 4096>			mMPMMaterialPool;
				PxMutex									mMPMMaterialPoolLock;

				/*PxPool2<NpFEMSoftBodyMaterial, 4096>	mFEMMaterialPool;
				PxMutex									mFEMMaterialPoolLock;*/

				PxPool2<NpHairSystem, 4096>				mHairSystemPool;
				PxMutex									mHairSystemPoolLock;
	#endif
#endif

	static		NpFactory*								mInstance;
				PxU64									mGpuMemStat;

#if PX_SUPPORT_PVD
				NpFactoryListener*						mNpFactoryListener;
#endif
};

	void	NpDestroyRigidActor(NpRigidStatic* np);
	void	NpDestroyRigidDynamic(NpRigidDynamic* np);
	void	NpDestroyArticulationLink(NpArticulationLink* np);
	void	NpDestroyArticulationJoint(PxArticulationJointReducedCoordinate* np);
	void	NpDestroyArticulation(PxArticulationReducedCoordinate* artic);
	void	NpDestroyAggregate(NpAggregate* np);
	void	NpDestroyShape(NpShape* np);
	void	NpDestroyConstraint(NpConstraint* np);

#if PX_SUPPORT_GPU_PHYSX
	void	NpDestroySoftBody(NpSoftBody* softBody);
	void	NpDestroyFEMCloth(NpFEMCloth* femCloth);
	void	NpDestroyParticleSystem(NpPBDParticleSystem* particleSystem);
	void	NpDestroyParticleSystem(NpFLIPParticleSystem* particleSystem);
	void	NpDestroyParticleSystem(NpMPMParticleSystem* particleSystem);
	void	NpDestroyHairSystem(NpHairSystem* hairSystem);
#endif
}

#endif
