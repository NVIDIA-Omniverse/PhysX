// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef NP_DEFORMABLE_SURFACE
#define NP_DEFORMABLE_SURFACE

#include "foundation/PxAllocator.h"
#include "foundation/PxPreprocessor.h"
#if PX_SUPPORT_GPU_PHYSX
#include "PxDeformableSurface.h"
#include "ScDeformableSurfaceCore.h"
#include "NpActorTemplate.h"

namespace physx
{

class NpShape;
class PxsMemoryManager;

class NpDeformableSurface : public NpActorTemplate<PxDeformableSurface>
{
public:
	NpDeformableSurface(PxCudaContextManager& cudaContextManager);
	NpDeformableSurface(PxBaseFlags baseFlags, PxCudaContextManager& cudaContextManager);
	virtual ~NpDeformableSurface() { releaseAllocator(); }
	void exportData(PxSerializationContext& /*context*/) const {}

	// PxActor API

	virtual void							release() PX_OVERRIDE;
	virtual PxActorType::Enum				getType() const PX_OVERRIDE { return PxActorType::eDEFORMABLE_SURFACE; }
	virtual PxBounds3	 					getWorldBounds(float inflation = 1.01f) const PX_OVERRIDE;
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

	// PxDeformableSurface API

	virtual	void							setDeformableSurfaceFlag(PxDeformableSurfaceFlag::Enum flag, bool val) PX_OVERRIDE;
	virtual	void							setDeformableSurfaceFlags(PxDeformableSurfaceFlags flags) PX_OVERRIDE;
	virtual	PxDeformableSurfaceFlags		getDeformableSurfaceFlags() const PX_OVERRIDE;

	virtual	void							setNbCollisionPairUpdatesPerTimestep(const PxU32 frequency) PX_OVERRIDE;
	virtual	PxU32							getNbCollisionPairUpdatesPerTimestep() const PX_OVERRIDE;

	virtual	void							setNbCollisionSubsteps(const PxU32 frequency) PX_OVERRIDE;
	virtual	PxU32							getNbCollisionSubsteps() const PX_OVERRIDE;

	virtual PxVec4*							getPositionInvMassBufferD() PX_OVERRIDE;
	virtual PxVec4*							getVelocityBufferD() PX_OVERRIDE;
	virtual PxVec4*							getRestPositionBufferD() PX_OVERRIDE;
		
	virtual void							markDirty(PxDeformableSurfaceDataFlags flags) PX_OVERRIDE;

	// Internal

	PX_FORCE_INLINE	const Sc::DeformableSurfaceCore&	getCore() const { return mCore; }
	PX_FORCE_INLINE	Sc::DeformableSurfaceCore&			getCore() { return mCore; }
	static PX_FORCE_INLINE size_t						getCoreOffset() { return PX_OFFSET_OF_RT(NpDeformableSurface, mCore); }

	void									updateMaterials();

private:
	void 									createAllocator();
	void 									releaseAllocator();

	NpShape*								mShape; //deformable surface can only have one triangle mesh shape.
	Sc::DeformableSurfaceCore				mCore;
	PxCudaContextManager*					mCudaContextManager;
	PxsMemoryManager*						mMemoryManager;
	Cm::VirtualAllocatorCallback*				mDeviceMemoryAllocator;
};

Sc::DeformableSurfaceCore* getDeformableSurfaceCore(PxActor* actor);

} // namespace physx

#endif // PX_SUPPORT_GPU_PHYSX
#endif // NP_DEFORMABLE_SURFACE
