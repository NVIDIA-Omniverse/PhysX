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

#ifndef SC_HAIR_SYSTEM_SIM_H
#define SC_HAIR_SYSTEM_SIM_H

#include "foundation/PxPreprocessor.h"
#if PX_SUPPORT_GPU_PHYSX
#include "DyHairSystem.h"
#include "ScHairSystemCore.h"
#include "ScHairSystemShapeSim.h"
#include "ScActorSim.h"

namespace physx
{
	namespace Sc
	{
		class Scene;

		class HairSystemSim : public ActorSim
		{
			PX_NOCOPY(HairSystemSim)
		public:
			HairSystemSim(HairSystemCore& core, Scene& scene);
			~HairSystemSim();

			PX_INLINE Dy::HairSystem* getLowLevelHairSystem() const { return mLLHairSystem; }
			PX_INLINE HairSystemCore& getCore() const { return static_cast<HairSystemCore&>(mCore); }

			virtual PxActor* getPxActor() const { return getCore().getPxActor(); }

			PxBounds3 getBounds() const;
			void updateBounds();
			void updateBoundsInAABBMgr();

			bool isSleeping() const;
			bool isActive() const { return !isSleeping(); }

			void setActive(bool active, bool asPartOfCreation=false);

			void onSetWakeCounter();

			HairSystemShapeSim& getShapeSim() { return mShapeSim; }

		private:
			Dy::HairSystem*		mLLHairSystem;
			HairSystemShapeSim	mShapeSim;

// PT: as far as I can tell these are never actually called
//						void activate();
//						void deactivate();
		};
	} // namespace Sc
} // namespace physx
#endif

#endif

