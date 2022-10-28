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

#ifndef SC_PARTICLESYSTEM_CORE_H
#define SC_PARTICLESYSTEM_CORE_H

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
			//= ATTENTION! =====================================================================================
			// Changing the data layout of this class breaks the binary serialization format.  See comments for 
			// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
			// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
			// accordingly.
			//==================================================================================================

			//---------------------------------------------------------------------------------
			// Construction, destruction & initialization
			//---------------------------------------------------------------------------------

			// PX_SERIALIZATION
		public:
			ParticleSystemCore(const PxEMPTY) : ActorCore(PxEmpty) {}
			static		void							getBinaryMetaData(PxOutputStream& stream);
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

			PxCustomParticleSystemSolverCallback*	getParticleSystemSolverCallback() const;
			void						setParticleSystemSolverCallback(PxCustomParticleSystemSolverCallback* callback);
			
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

			// TOFIX
			void						enableCCD(const bool enable);

			PxParticleFlags				getFlags() const { return mShapeCore.getLLCore().mFlags; }

			//void						setFlags(PxParticleFlags flags) { mShapeCore.getLLCore().mFlags = flags; }
			void						setFlags(PxParticleFlags flags);

			void						setWind(const PxVec3& wind) {mShapeCore.getLLCore().mWind = wind;}

			PxVec3						getWind() const { return mShapeCore.getLLCore().mWind; }

			void						setSolverType(const PxParticleSolverType::Enum solverType) { mShapeCore.getLLCore().solverType = solverType; }
			PxParticleSolverType::Enum	getSolverType() const { return mShapeCore.getLLCore().solverType; }
			
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
			PxSparseGridParams			getSparseGridParams() const { return mShapeCore.getLLCore().sparseGridParams; }
			void						setSparseGridParams(const PxSparseGridParams& params) { mShapeCore.getLLCore().sparseGridParams = params; }

			PxFLIPParams				getFLIPParams() const { return mShapeCore.getLLCore().flipParams; }
			void						setFLIPParams(const PxFLIPParams& params) { mShapeCore.getLLCore().flipParams = params; }

			PxMPMParams					getMPMParams() const { return mShapeCore.getLLCore().mpmParams; }
			void						setMPMParams(const PxMPMParams& params) { mShapeCore.getLLCore().mpmParams = params; }
#endif
			
			void						addRigidAttachment(Sc::BodyCore* core);

			void						removeRigidAttachment(Sc::BodyCore* core);

			void						setPeriodicBoundary(const PxVec3& boundary) { mShapeCore.setPeriodicBoundary(boundary); }

			PxVec3						getPeriodicBoundary() const { return mShapeCore.getPeriodicBoundary(); }

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
