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

#ifndef SC_PARTICLESYSTEM_SIM_H
#define SC_PARTICLESYSTEM_SIM_H

#include "foundation/PxUserAllocated.h"
#include "ScActorSim.h"
#include "ScParticleSystemCore.h" 
#include "ScParticleSystemShapeSim.h"

namespace physx
{
	namespace Sc
	{
		class Scene;

		class ParticleSystemSim : public ActorSim
		{
		public:
			ParticleSystemSim(ParticleSystemCore& core, Scene& scene);

			~ParticleSystemSim();

			PX_INLINE	Dy::ParticleSystem*		getLowLevelParticleSystem() const { return mLLParticleSystem; }
			PX_INLINE	ParticleSystemCore&		getCore() const { return static_cast<ParticleSystemCore&>(mCore); }

			virtual			PxActor*		getPxActor() const { return getCore().getPxActor(); }

			void							updateBounds();
			void							updateBoundsInAABBMgr();
			PxBounds3						getBounds() const;
		
			bool							isSleeping() const;
			bool							isActive() const { return true; }
			void							sleepCheck(PxReal dt);

			void							setActive(const bool b, const PxU32 infoFlag = 0);

			const			ParticleSystemShapeSim& getShapeSim() const	 { return mShapeSim; }
							ParticleSystemShapeSim& getShapeSim()		 { return mShapeSim; }

			virtual			void			registerCountedInteraction() { mNumCountedInteractions++; }
			virtual			void			unregisterCountedInteraction() { mNumCountedInteractions--; }
			virtual			PxU32			getNumCountedInteractions()	const { return mNumCountedInteractions; }

			virtual			void			activate();
			virtual			void			deactivate();

		private:
			ParticleSystemSim & operator=(const ParticleSystemSim&);

			Dy::ParticleSystem*									mLLParticleSystem;

			ParticleSystemShapeSim								mShapeSim;

			PxU32												mNumCountedInteractions;

		};

	} // namespace Sc

}

#endif
