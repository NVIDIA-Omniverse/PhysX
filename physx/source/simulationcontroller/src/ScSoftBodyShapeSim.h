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

#ifndef SC_SOFTBODY_SHAPE_SIM_H
#define SC_SOFTBODY_SHAPE_SIM_H

#include "foundation/PxPreprocessor.h"
#if PX_SUPPORT_GPU_PHYSX
#include "PxPhysXConfig.h"

#include "ScElementSim.h"
#include "ScShapeSimBase.h"

namespace physx
{
	namespace Sc
	{
		class SoftBodySim;

		/**
		A collision detection primitive for soft body.
		*/
		class SoftBodyShapeSim : public ShapeSimBase
		{
			PxTransform initialTransform;
			PxReal initialScale;

			SoftBodyShapeSim& operator=(const SoftBodyShapeSim &);
		public:
								SoftBodyShapeSim(SoftBodySim& softbody);
			virtual				~SoftBodyShapeSim();

			PX_FORCE_INLINE void setInitialTransform(const PxTransform& transform, PxReal scale)
			{
				initialTransform = transform;
				initialScale = scale;
				//The base class constructor ensures that getElementID() points to a valid entry in the bounds array
				getScene().getBoundsArray().setBounds(getWorldBounds(), getElementID());
			}

			void				attachShapeCore(const ShapeCore* core);
			// ElementSim implementation
			virtual		void	getFilterInfo(PxFilterObjectAttributes& filterAttr, PxFilterData& filterData) const;
			// ~ElementSim

			PxBounds3			getWorldBounds() const;
			
			//PX_FORCE_INLINE	SoftBodySim&			getBodySim()		const { return static_cast<SoftBodySim&>(getActor()); }
			SoftBodySim&		getBodySim() const;

			void				updateBounds();
			void				updateBoundsInAABBMgr();
			PxBounds3			getBounds() const;

			void				createLowLevelVolume();
			void				destroyLowLevelVolume();
		};

	} // namespace Sc
}
#endif

#endif
