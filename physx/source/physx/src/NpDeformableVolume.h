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

#ifndef NP_DEFORMABLE_VOLUME_H
#define NP_DEFORMABLE_VOLUME_H

#include "foundation/PxPreprocessor.h"
#if PX_SUPPORT_GPU_PHYSX
#include "PxDeformableVolume.h"
#include "ScDeformableVolumeCore.h"
#include "GuTetrahedronMesh.h"
#include "NpActorTemplate.h"

namespace physx
{

class NpScene;
class NpShape;
class PxsMemoryManager;
class PxVirtualAllocatorCallback;

class NpDeformableVolume : public NpActorTemplate<PxDeformableVolume>
{
public:
	NpDeformableVolume(PxCudaContextManager& cudaContextManager);
	NpDeformableVolume(PxBaseFlags baseFlags, PxCudaContextManager& cudaContextManager);
	virtual ~NpDeformableVolume() { releaseAllocator(); }
	void exportData(PxSerializationContext& /*context*/) const{}
	
	// PxActor API

	virtual void							release();
	virtual PxActorType::Enum				getType() const { return PxActorType::eDEFORMABLE_VOLUME; }
	virtual PxBounds3						getWorldBounds(float inflation = 1.01f) const;

	// PxDeformableBody API

	virtual		void						setDeformableBodyFlag(PxDeformableBodyFlag::Enum flag, bool val);
	virtual		void						setDeformableBodyFlags(PxDeformableBodyFlags flags);
	virtual		PxDeformableBodyFlags		getDeformableBodyFlags() const;

	virtual		void						setLinearDamping(const PxReal linearDamping);
	virtual		PxReal						getLinearDamping() const;

	virtual		void						setMaxVelocity(const PxReal maxVelocity);
	virtual		PxReal						getMaxVelocity() const;

	virtual		void						setMaxDepenetrationVelocity(const PxReal maxDepenetrationVelocity);
	virtual		PxReal						getMaxDepenetrationVelocity() const;

	virtual		void						setSelfCollisionFilterDistance(const PxReal selfCollisionFilterDistance);
	virtual		PxReal						getSelfCollisionFilterDistance() const;

	virtual		void						setSolverIterationCounts(PxU32 minPositionIters, PxU32 minVelocityIters);
	virtual		void						getSolverIterationCounts(PxU32& minPositionIters, PxU32& minVelocityIters) const;

	virtual		void						setSleepThreshold(const PxReal sleepThreshold);
	virtual		PxReal						getSleepThreshold() const;

	virtual		void						setSettlingThreshold(const PxReal settlingThreshold);
	virtual		PxReal						getSettlingThreshold() const;

	virtual		void						setSettlingDamping(const PxReal linearDamping);
	virtual		PxReal						getSettlingDamping() const;

	virtual		void						setWakeCounter(PxReal wakeCounterValue);
	virtual		PxReal						getWakeCounter() const;
	virtual		bool						isSleeping() const;

	virtual		PxShape*					getShape();
	virtual		bool						attachShape(PxShape& shape);
	virtual		void						detachShape();

	virtual		PxCudaContextManager*		getCudaContextManager() const;

	PX_DEPRECATED virtual void				setParameter(const PxFEMParameters& params);
	PX_DEPRECATED virtual PxFEMParameters	getParameter() const;

	// PxDeformableVolume API
	
	virtual void							setDeformableVolumeFlag(PxDeformableVolumeFlag::Enum flag, bool val);
	virtual void							setDeformableVolumeFlags(PxDeformableVolumeFlags flags);
	virtual PxDeformableVolumeFlags			getDeformableVolumeFlags() const;

	virtual void							setSelfCollisionStressTolerance(const PxReal selfCollisionStressTolerance);
	virtual PxReal							getSelfCollisionStressTolerance() const;

	virtual PxVec4*							getPositionInvMassBufferD();
	virtual PxVec4*							getRestPositionBufferD();

	virtual PxVec4*							getSimPositionInvMassBufferD();
	virtual PxVec4*							getSimVelocityBufferD();

	virtual void							markDirty(PxDeformableVolumeDataFlags flags);

	virtual	void							setKinematicTargetBufferD(const PxVec4* positions);
	PX_DEPRECATED virtual void				setKinematicTargetBufferD(const PxVec4* positions, PxDeformableVolumeFlags flags);

	virtual bool							attachSimulationMesh(PxTetrahedronMesh& simulationMesh, PxDeformableVolumeAuxData& softBodyAuxData);
	virtual void							detachSimulationMesh();
	virtual PxTetrahedronMesh*				getSimulationMesh() { return mSimulationMesh; }
	virtual const PxTetrahedronMesh*		getSimulationMesh() const { return mSimulationMesh; }

	virtual PxTetrahedronMesh*				getCollisionMesh();
	virtual const PxTetrahedronMesh*		getCollisionMesh() const;

	virtual PxDeformableVolumeAuxData*		getDeformableVolumeAuxData() { return mAuxData; }
	virtual const PxDeformableVolumeAuxData*
											getDeformableVolumeAuxData() const { return mAuxData; }

	virtual PxU32							getGpuDeformableVolumeIndex();

	PX_DEPRECATED virtual void				addParticleFilter(PxPBDParticleSystem* particlesystem, const PxParticleBuffer* buffer, PxU32 particleId, PxU32 tetId);
	PX_DEPRECATED virtual void				removeParticleFilter(PxPBDParticleSystem* particlesystem, const PxParticleBuffer* buffer, PxU32 particleId, PxU32 tetId);

	PX_DEPRECATED virtual PxU32				addParticleAttachment(PxPBDParticleSystem* particlesystem, const PxParticleBuffer* buffer, PxU32 particleId, PxU32 tetId, const PxVec4& barycentric);
	PX_DEPRECATED virtual void				removeParticleAttachment(PxPBDParticleSystem* particlesystem, PxU32 handle);

	PX_DEPRECATED virtual void				addRigidFilter(PxRigidActor* actor, PxU32 vertId);
	PX_DEPRECATED virtual void				removeRigidFilter(PxRigidActor* actor, PxU32 vertId);

	PX_DEPRECATED virtual PxU32				addRigidAttachment(PxRigidActor* actor, PxU32 vertId, const PxVec3& actorSpacePose, PxConeLimitedConstraint* constraint);
	PX_DEPRECATED virtual void				removeRigidAttachment(PxRigidActor* actor, PxU32 handle);

	PX_DEPRECATED virtual void				addTetRigidFilter(PxRigidActor* actor, PxU32 tetIdx);
	PX_DEPRECATED virtual void				removeTetRigidFilter(PxRigidActor* actor, PxU32 tetIdx);

	PX_DEPRECATED virtual PxU32				addTetRigidAttachment(PxRigidActor* actor, PxU32 tetIdx, const PxVec4& barycentric, const PxVec3& actorSpacePose,
												PxConeLimitedConstraint* constraint);
	PX_DEPRECATED virtual void				addSoftBodyFilter(PxDeformableVolume* softbody0, PxU32 tetIdx0, PxU32 tetIdx1);
	PX_DEPRECATED virtual void				removeSoftBodyFilter(PxDeformableVolume* softbody0, PxU32 tetIdx0, PxU32 tetIdx1);
	PX_DEPRECATED virtual void				addSoftBodyFilters(PxDeformableVolume* softbody0, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize);
	PX_DEPRECATED virtual void				removeSoftBodyFilters(PxDeformableVolume* softbody0, PxU32* tetIndices0, PxU32* tetIndices1, PxU32 tetIndicesSize);

	PX_DEPRECATED virtual PxU32				addSoftBodyAttachment(PxDeformableVolume* softbody0, PxU32 tetIdx0, const PxVec4& tetBarycentric0, PxU32 tetIdx1, const PxVec4& tetBarycentric1,
												PxConeLimitedConstraint* constraint, PxReal constraintOffset);
	PX_DEPRECATED virtual void				removeSoftBodyAttachment(PxDeformableVolume* softbody0, PxU32 handle);

	// Internal

	PX_FORCE_INLINE	const Sc::DeformableVolumeCore&		getCore()	const { return mCore; }
	PX_FORCE_INLINE	Sc::DeformableVolumeCore&			getCore() { return mCore; }
	static PX_FORCE_INLINE size_t						getCoreOffset() { return PX_OFFSET_OF_RT(NpDeformableVolume, mCore); }

	void									updateMaterials();

private:
	void 									createAllocator();
	void 									releaseAllocator();

	NpShape*								mShape; //deformable volume can only have one tetrahedron mesh shape.
	Gu::TetrahedronMesh*					mSimulationMesh;
	Gu::DeformableVolumeAuxData*			mAuxData;
	Sc::DeformableVolumeCore				mCore;
	PxCudaContextManager*					mCudaContextManager;
	PxsMemoryManager*						mMemoryManager;
	PxVirtualAllocatorCallback*				mDeviceMemoryAllocator;
};

Sc::DeformableVolumeCore* getDeformableVolumeCore(PxActor* actor);

} // namespace physx

#endif //PX_SUPPORT_GPU_PHYSX
#endif // NP_DEFORMABLE_VOLUME_H
