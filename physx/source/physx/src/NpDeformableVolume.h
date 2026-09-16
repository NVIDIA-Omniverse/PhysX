// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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

class NpDeformableVolume : public NpActorTemplate<PxDeformableVolume>
{
public:
	NpDeformableVolume(PxCudaContextManager& cudaContextManager);
	NpDeformableVolume(PxBaseFlags baseFlags, PxCudaContextManager& cudaContextManager);
	virtual ~NpDeformableVolume() { releaseAllocator(); }
	void exportData(PxSerializationContext& /*context*/) const{}
	
	// PxActor API

	virtual void							release() PX_OVERRIDE;
	virtual PxActorType::Enum				getType() const PX_OVERRIDE { return PxActorType::eDEFORMABLE_VOLUME; }
	virtual PxBounds3						getWorldBounds(float inflation = 1.01f) const PX_OVERRIDE;
	virtual void							setActorFlag(PxActorFlag::Enum flag, bool value) PX_OVERRIDE;
	virtual void							setActorFlags(PxActorFlags inFlags) PX_OVERRIDE;

	// PxDeformableBody API

	virtual		void						setDeformableBodyFlag(PxDeformableBodyFlag::Enum flag, bool val) PX_OVERRIDE;
	virtual		void						setDeformableBodyFlags(PxDeformableBodyFlags flags) PX_OVERRIDE;
	virtual		PxDeformableBodyFlags		getDeformableBodyFlags() const PX_OVERRIDE;

	virtual		void						setLinearDamping(const PxReal linearDamping) PX_OVERRIDE;
	virtual		PxReal						getLinearDamping() const PX_OVERRIDE;

	virtual		void						setMaxLinearVelocity(const PxReal maxLinearVelocity) PX_OVERRIDE;
	virtual		PxReal						getMaxLinearVelocity() const PX_OVERRIDE;

	virtual		void						setMaxDepenetrationVelocity(const PxReal maxDepenetrationVelocity) PX_OVERRIDE;
	virtual		PxReal						getMaxDepenetrationVelocity() const PX_OVERRIDE;

	virtual		void						setSelfCollisionFilterDistance(const PxReal selfCollisionFilterDistance) PX_OVERRIDE;
	virtual		PxReal						getSelfCollisionFilterDistance() const PX_OVERRIDE;

	virtual		void						setSolverIterationCounts(PxU32 minPositionIters, PxU32 minVelocityIters) PX_OVERRIDE;
	virtual		void						getSolverIterationCounts(PxU32& minPositionIters, PxU32& minVelocityIters) const PX_OVERRIDE;

	virtual		void						setSleepThreshold(const PxReal sleepThreshold) PX_OVERRIDE;
	virtual		PxReal						getSleepThreshold() const PX_OVERRIDE;

	virtual		void						setSettlingThreshold(const PxReal settlingThreshold) PX_OVERRIDE;
	virtual		PxReal						getSettlingThreshold() const PX_OVERRIDE;

	virtual		void						setSettlingDamping(const PxReal linearDamping) PX_OVERRIDE;
	virtual		PxReal						getSettlingDamping() const PX_OVERRIDE;

	virtual		void						setWakeCounter(PxReal wakeCounterValue) PX_OVERRIDE;
	virtual		PxReal						getWakeCounter() const PX_OVERRIDE;
	virtual		bool						isSleeping() const PX_OVERRIDE;

	virtual		PxShape*					getShape() PX_OVERRIDE;
	virtual		bool						attachShape(PxShape& shape) PX_OVERRIDE;
	virtual		void						detachShape() PX_OVERRIDE;

	virtual		PxCudaContextManager*		getCudaContextManager() const PX_OVERRIDE;

	// PxDeformableVolume API
	
	virtual void							setDeformableVolumeFlag(PxDeformableVolumeFlag::Enum flag, bool val) PX_OVERRIDE;
	virtual void							setDeformableVolumeFlags(PxDeformableVolumeFlags flags) PX_OVERRIDE;
	virtual PxDeformableVolumeFlags			getDeformableVolumeFlags() const PX_OVERRIDE;

	virtual void							setSelfCollisionStressTolerance(const PxReal selfCollisionStressTolerance) PX_OVERRIDE;
	virtual PxReal							getSelfCollisionStressTolerance() const PX_OVERRIDE;

	virtual PxVec4*							getPositionInvMassBufferD() PX_OVERRIDE;
	virtual PxVec4*							getRestPositionBufferD() PX_OVERRIDE;

	virtual PxVec4*							getSimPositionInvMassBufferD() PX_OVERRIDE;
	virtual PxVec4*							getSimVelocityBufferD() PX_OVERRIDE;

	virtual void							markDirty(PxDeformableVolumeDataFlags flags) PX_OVERRIDE;

	virtual	void							setKinematicTargetBufferD(const PxVec4* positions) PX_OVERRIDE;

	virtual bool							attachSimulationMesh(PxTetrahedronMesh& simulationMesh, PxDeformableVolumeAuxData& softBodyAuxData) PX_OVERRIDE;
	virtual void							detachSimulationMesh() PX_OVERRIDE;
	virtual PxTetrahedronMesh*				getSimulationMesh() PX_OVERRIDE { return mSimulationMesh; }
	virtual const PxTetrahedronMesh*		getSimulationMesh() const PX_OVERRIDE { return mSimulationMesh; }

	virtual PxTetrahedronMesh*				getCollisionMesh() PX_OVERRIDE;
	virtual const PxTetrahedronMesh*		getCollisionMesh() const PX_OVERRIDE;

	virtual PxDeformableVolumeAuxData*		getDeformableVolumeAuxData() PX_OVERRIDE { return mAuxData; }
	virtual const PxDeformableVolumeAuxData*
											getDeformableVolumeAuxData() const PX_OVERRIDE	{ return mAuxData; }

	virtual PxU32							getGpuDeformableVolumeIndex() PX_OVERRIDE;

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
	Cm::VirtualAllocatorCallback*				mDeviceMemoryAllocator;
};

Sc::DeformableVolumeCore* getDeformableVolumeCore(PxActor* actor);

} // namespace physx

#endif //PX_SUPPORT_GPU_PHYSX
#endif // NP_DEFORMABLE_VOLUME_H
