// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_PARTICLESYSTEM_SHAPECORE_H
#define SC_PARTICLESYSTEM_SHAPECORE_H

#include "foundation/PxPreprocessor.h"
#if PX_SUPPORT_GPU_PHYSX
#include "foundation/PxUserAllocated.h"
#include "PxvGeometry.h"
#include "foundation/PxUtilities.h"
#include "PxFiltering.h"
#include "PxShape.h"
#include "ScShapeCore.h"
#include "DyParticleSystemCore.h"
#include "common/PxRenderOutput.h"

namespace physx
{
	namespace Sc
	{
		class Scene;
		class ParticleSystemCore;
		class ParticleSystemSim;

		class ParticleSystemShapeCore : public Sc::ShapeCore
		{
		public:
			// PX_SERIALIZATION
			ParticleSystemShapeCore(const PxEMPTY);
			//~PX_SERIALIZATION

			ParticleSystemShapeCore();
			~ParticleSystemShapeCore();

			PX_FORCE_INLINE	const Dy::ParticleSystemCore&	getLLCore() const { return mLLCore; }

			PX_FORCE_INLINE	Dy::ParticleSystemCore&	getLLCore() { return mLLCore; }

			void initializeLLCoreData(PxU32 maxNeighborhood, PxReal neighborhoodScale);

			PxU64 getGpuMemStat() { return mGpuMemStat; }

		protected:
			Dy::ParticleSystemCore	mLLCore;
			PxU64					mGpuMemStat;
		};

	} // namespace Sc
}
#endif

#endif
