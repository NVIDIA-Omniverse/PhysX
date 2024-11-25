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
class PxVirtualAllocatorCallback;

class NpDeformableSurface : public NpActorTemplate<PxDeformableSurface>
{
public:
	NpDeformableSurface(PxCudaContextManager& cudaContextManager);
	NpDeformableSurface(PxBaseFlags baseFlags, PxCudaContextManager& cudaContextManager);
	virtual ~NpDeformableSurface() { releaseAllocator(); }
	void exportData(PxSerializationContext& /*context*/) const {}

	// PxActor API

	virtual void							release();
	virtual PxActorType::Enum				getType() const { return PxActorType::eDEFORMABLE_SURFACE; }
	virtual PxBounds3	 					getWorldBounds(float inflation = 1.01f) const;

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

	// PxDeformableSurface API

	virtual	void							setDeformableSurfaceFlag(PxDeformableSurfaceFlag::Enum flag, bool val);
	virtual	void							setDeformableSurfaceFlags(PxDeformableSurfaceFlags flags);
	virtual	PxDeformableSurfaceFlags		getDeformableSurfaceFlags() const;

	virtual	void							setNbCollisionPairUpdatesPerTimestep(const PxU32 frequency);
	virtual	PxU32							getNbCollisionPairUpdatesPerTimestep() const;

	virtual	void							setNbCollisionSubsteps(const PxU32 frequency);
	virtual	PxU32							getNbCollisionSubsteps() const;

	virtual PxVec4*							getPositionInvMassBufferD();
	virtual PxVec4*							getVelocityBufferD();
	virtual PxVec4*							getRestPositionBufferD();
		
	virtual void							markDirty(PxDeformableSurfaceDataFlags flags);

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
	PxVirtualAllocatorCallback*				mDeviceMemoryAllocator;
};

Sc::DeformableSurfaceCore* getDeformableSurfaceCore(PxActor* actor);

} // namespace physx

#endif // PX_SUPPORT_GPU_PHYSX
#endif // NP_DEFORMABLE_SURFACE
