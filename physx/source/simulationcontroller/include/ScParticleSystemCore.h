// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_PARTICLESYSTEM_CORE_H
#define SC_PARTICLESYSTEM_CORE_H

#include "foundation/PxPreprocessor.h"
#if PX_SUPPORT_GPU_PHYSX
#include "PxParticleSystem.h"
#include "foundation/PxAssert.h"
#include "ScActorCore.h"
#include "ScShapeCore.h"
#include "PxFiltering.h"
#include "DyParticleSystem.h"
#include "ScParticleSystemShapeCore.h"

namespace physx
{
	namespace Sc
	{
		class ParticleSystemSim;
		class BodyCore;

		class ParticleSystemCore : public ActorCore
		{
			// PX_SERIALIZATION
		public:
			ParticleSystemCore(const PxEMPTY) : ActorCore(PxEmpty) {}
			//~PX_SERIALIZATION
			ParticleSystemCore(PxActorType::Enum actorType);
			~ParticleSystemCore();

			//---------------------------------------------------------------------------------
			// External API
			//---------------------------------------------------------------------------------

			PxReal						getSleepThreshold() const;
			void						setSleepThreshold(const PxReal v);

			PxReal						getRestOffset() const;
			void						setRestOffset(const PxReal v);

			PxReal						getContactOffset() const;
			void						setContactOffset(const PxReal v);

			PxReal						getParticleContactOffset() const;
			void						setParticleContactOffset(const PxReal v);

			PxReal						getSolidRestOffset() const;
			void						setSolidRestOffset(const PxReal v);

			PxReal						getFluidRestOffset() const;
			void						setFluidRestOffset(const PxReal v);

			PxReal						getMaxDepenetrationVelocity() const;
			void						setMaxDepenetrationVelocity(const PxReal v);

			PxReal						getMaxVelocity() const;
			void						setMaxVelocity(const PxReal v);

			PxParticleSystemCallback*	getParticleSystemCallback() const;
			void						setParticleSystemCallback(PxParticleSystemCallback* callback);
			
			PxReal						getFluidBoundaryDensityScale() const;
			void						setFluidBoundaryDensityScale(const PxReal v);

			PxU32						getGridSizeX() const;
			void						setGridSizeX(const PxU32 v);

			PxU32						getGridSizeY() const;
			void						setGridSizeY(const PxU32 v);

			PxU32						getGridSizeZ() const;
			void						setGridSizeZ(const PxU32 v);

			PxU16						getSolverIterationCounts() const { return mShapeCore.getLLCore().solverIterationCounts; }
			void						setSolverIterationCounts(PxU16 c);

			PxReal						getWakeCounter() const;
			void						setWakeCounter(const PxReal v);
			void						setWakeCounterInternal(const PxReal v);

			bool						isSleeping() const;
			void						wakeUp(PxReal wakeCounter);
			void						putToSleep();

			PxActor*					getPxActor() const;


			PxParticleFlags				getFlags() const { return mShapeCore.getLLCore().mFlags; }

			void						setFlags(PxParticleFlags flags);

			PxParticleLockFlags			getLockFlags() const { return mShapeCore.getLLCore().mLockFlags; }

			void						setLockFlags(PxParticleLockFlags lockFlags) { mShapeCore.getLLCore().mLockFlags = lockFlags; }

			void						setWind(const PxVec3& wind) {mShapeCore.getLLCore().mWind = wind;}

			PxVec3						getWind() const { return mShapeCore.getLLCore().mWind; }

			PxSparseGridParams			getSparseGridParams() const { return mShapeCore.getLLCore().sparseGridParams; }
			void						setSparseGridParams(const PxSparseGridParams& params) { mShapeCore.getLLCore().sparseGridParams = params; }

			//---------------------------------------------------------------------------------
			// Internal API
			//---------------------------------------------------------------------------------
		public:
			ParticleSystemSim*			getSim()			const;

			PX_FORCE_INLINE	const ParticleSystemShapeCore&	getShapeCore() const	{ return mShapeCore; }
			PX_FORCE_INLINE	ParticleSystemShapeCore&	getShapeCore() { return mShapeCore; }

			void setDirty(const bool dirty);

		private:
			//ParticleSystemSim*						mSim;
			ParticleSystemShapeCore					mShapeCore;
		};

	} // namespace Sc
}
#endif

#endif
