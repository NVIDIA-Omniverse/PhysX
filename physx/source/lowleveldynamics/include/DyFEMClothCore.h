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

#ifndef PXDV_FEMCLOTH_CORE_H
#define PXDV_FEMCLOTH_CORE_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "foundation/PxArray.h"
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
#include "PxFEMCloth.h"
#endif
#include "PxsFEMClothMaterialCore.h"

namespace physx
{
	class PxBuffer;

	namespace Dy
	{
		struct FEMClothCore
		{
		public:

#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
			PxFEMParameters							parameters;
#endif
			PxU16									solverIterationCounts;
			bool									dirty;
			PxReal									wakeCounter;
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
			PxFEMClothFlags							mFlags;
#endif
			// Ratio between target volume and rest volume in inflatable simulation.
			PxReal									mRestVolumeScale;

			PxArray<PxU16>							mMaterialHandles;
			PxBuffer*								mClothPositionInvMass;
			PxBuffer*								mClothVelocity;
			PxBuffer*								mClothRestPosition;

			// multimaterial bending effects
			PxArray<PxReal>							mBendingScales;

			// wind
			PxReal									drag;
			PxReal									lift;
			PxVec3									wind;
			PxReal									airDensity;

			PxReal									maxVelocity;

			// negative values mean no activation angle: apply bending force toward rest bending angle
			PxReal									mBendingActivationAngle;

			// number of collision pair updates per timestep. Collision pair is updated at least once per timestep and increasing the frequency provides better collision pairs.
			PxU32									NbCollisionPairUpdatesPerTimestep;

			// number of collision substeps in each sub-timestep. Collision constraints can be applied multiple times in each sub-timestep.
			PxU32									nbCollisionSubsteps;


			FEMClothCore()
			{
		        drag = 0.f;
		        lift = 0.f;
		        wind = PxVec3(0.f);
		        airDensity = 1.225f; // default: 1.225 kg/m^3
		        maxVelocity = 0.f;
		        mBendingActivationAngle = -1.f;
		        mRestVolumeScale = 0.0f; // No inflatable simulation by default.

				NbCollisionPairUpdatesPerTimestep = 1;
				nbCollisionSubsteps = 1;
			}

			void setMaterial(const PxU16 materialHandle)
			{
				mMaterialHandles.pushBack(materialHandle);
			}

			void clearMaterials() { mMaterialHandles.clear(); }
		};
	}
}

#endif
